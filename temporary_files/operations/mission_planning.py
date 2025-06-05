import numpy as np
import gurobipy as gp
from gurobipy import GRB
import constants

def get_node_index(node, nodes, n_depots, n_orders, drone=None, count0=0, count1=0):
    if hasattr(node, "order_id"):
        return nodes.index(node)
    elif hasattr(node, "depot_id"):
        # For depots, you may need to know which copy (start/end) is intended.
        return nodes.index(node) + n_depots * count0
    else:
        # For restaurants, you may need to know which order (copy) is intended.
        if drone is not None:
            return nodes.index(drone.restaurant_order_nodes[count1]) - n_orders
        else:
            return nodes.index(node)  # fallback

class MissionPlanning:
    def __init__(self, Simulation):
        self.simulation = Simulation
        self.drones = self.simulation.drones
        self.depots = self.simulation.city.depots
        self.restaurants = self.simulation.city.restaurants
        self.orders = []
    
    def get_orders(self):
        orders = [self.simulation.order_book[order_id] for order_id in self.simulation.order_book if not self.simulation.order_book[order_id].status]
        return orders

    def basic_heuristic(self):
        self.orders = self.get_orders()
        self.orders = [order for order in self.orders if not order.being_delivered]
        for order in list(self.orders):
            ready_depots = [depot for depot in self.depots if len(depot.current_drones) > 0]
            nearest_depot = order.restaurant.nearest(ready_depots)
            #print(f"Order {order.order_id} assigned to depot {nearest_depot.depot_id} and drone {nearest_depot.current_drones[0].drone_id}")
            if nearest_depot:
                nearest_depot.current_drones[0].set_targets([order.restaurant, order, order.nearest(self.depots)])
                order.being_delivered = True

    def set_warm_start(self, x, t_dep, t_arr, drones, nodes, n_orders, n_depots):
        """
        Set a warm start for the Gurobi model using the previous mission planning solution,
        updated to the current simulation time.
        """
        current_time = self.simulation.timestamp
        for k, drone in enumerate(drones):
            # Only proceed if the drone has a previous plan
            if drone.targets:
                count0 = 0
                count1 = 0
                for idx in range(len(drone.targets) - 1):
                    node_from = drone.targets[idx]
                    node_to = drone.targets[idx + 1]
                    try:
                        i = get_node_index(node_from, nodes, n_depots, n_orders, drone, count0, count1)
                        j = get_node_index(node_to, nodes, n_depots, n_orders, drone, count0, count1)
                        # Update counters if needed
                        if hasattr(node_from, "depot_id"):
                            count0 += 1
                        elif not hasattr(node_from, "order_id"):
                            count1 += 1
                    except ValueError:
                        continue  # Node not in current node list (e.g., order already delivered)
                    # Set the route variable
                    x[i, j, k].Start = 1
                    # Update times relative to current simulation time
                    arr_time = max(drone.arrival_times[idx], current_time)
                    dep_time = max(drone.departure_times[idx], current_time)
                    t_arr[i, k].Start = arr_time
                    t_dep[i, k].Start = dep_time
                # Set arrival/departure for the last node in the route
                last_idx = len(drone.targets) - 1
                if last_idx >= 0:
                    try:
                        last_node = drone.targets[last_idx]
                        i = nodes.index(last_node)
                        t_arr[i, k].Start = max(drone.arrival_times[last_idx], current_time)
                        t_dep[i, k].Start = max(drone.departure_times[last_idx], current_time)
                    except ValueError:
                        pass        

    def setup_problem(self):
        drones=self.drones
        depots=self.depots
        orders = self.get_orders()
        orders_to_deliver = [order for order in orders if order.arrival_time < self.simulation.timestamp + constants.time_to_consider_order]
        orders = [order for order in orders_to_deliver if not order.being_delivered]    
        orders.sort(key=lambda order: order.arrival_time)
        orders = orders[:constants.max_orders_per_mission]  # Limit the number of orders to consider
        restaurants = [order.restaurant for order in orders]
        drone_start_nodes = []
        drone_earliest_dep = []
        orders_being_delivered = [order for order in orders if order.being_delivered]
        nodes = [*depots, *depots, *restaurants, *orders, *orders_being_delivered]
        for drone in drones:
            being_delivered_orders = [t for t in drone.targets if hasattr(t, "being_delivered") and t.being_delivered]
            if being_delivered_orders:
                nodes.append(being_delivered_orders[-1])
                drone_start_nodes.append(nodes.index(being_delivered_orders[-1]))
                #drone_earliest_dep.append(drone.departure_times[drone.targets.index(being_delivered_orders[-1])])
                drone_earliest_dep.append(self.simulation.timestamp)
            else:
                if drone.targets:
                    if hasattr(drone.targets[0], "depot_id"):
                        drone_start_nodes.append(nodes.index(drone.targets[0]))
                    else:
                        drone_start_nodes.append(nodes.index(drone.restaurant_order_nodes[0])-len(orders))
                    #drone_earliest_dep.append(drone.departure_times[0])
                    drone_earliest_dep.append(self.simulation.timestamp)
                else:
                    drone_start_nodes.append(nodes.index(drone.depot))
                    drone_earliest_dep.append(self.simulation.timestamp)
        nodes = [*nodes, *depots]
        print(f"Nodes: {[node.name for node in nodes]}")
        print(len(nodes), "nodes in total")
        #print(f"Orders: {[order.name for order in orders]}")
        #print(f"Orders being delivered: {[order.name for order in orders_being_delivered]}")
        drone_batteries = np.array([drone.battery for drone in drones])
        distance_matrix = np.zeros((len(nodes), len(nodes)))
        for i, node1 in enumerate(nodes):
            for j, node2 in enumerate(nodes):
                if i != j:
                    distance_matrix[i, j] = node1.distance(node2)        
        return drones, drone_start_nodes, drone_earliest_dep, depots, restaurants, orders, drone_batteries, nodes, distance_matrix

    def solve_mission_planning(self):
        drones, drone_start_nodes, drone_earliest_dep, depots, restaurants, orders, drone_batteries, nodes, distance_matrix = self.setup_problem()
        M = 1e4  # big-M
        n_nodes = len(nodes)
        n_drones = len(drones)
        n_orders = len(orders)
        if n_orders <= 0:
            return
        n_restaurants = len(restaurants)
        n_depots = len(depots)
        waiting_times = [node.waiting_time for node in nodes]  # waiting times at each node
        model = gp.Model("DARP")
        model.Params.OutputFlag = 0
        model.Params.TimeLimit = 100
        model.Params.MIPGap = 0.1  # Set a gap for suboptimal solutions
        model.Params.Heuristics = 0.5
        model.Params.Presolve = 2
        model.Params.Cuts = 2
        model.params.MIPFocus = 1  # Focus on finding feasible solutions quickly
        q = []
        for i in range(len(nodes)):
            if i < n_depots * 2:
                q.append(0)
            elif i < n_depots * 2 + n_orders:
                q.append(int(orders[i - n_depots * 2].demand))
            elif i < n_depots * 2 + n_orders * 2:
                q.append(int(-orders[i - n_depots * 2 - n_orders].demand))
            else:
                q.append(0)
        # Decision variables
        x = model.addVars(n_nodes, n_nodes, n_drones, vtype=GRB.BINARY, name="x")  # route selection
        t_dep = model.addVars(n_nodes, n_drones, vtype=GRB.CONTINUOUS, name="t_dep", ub=5e3, lb=0)  # departure times
        t_arr = model.addVars(n_nodes, n_drones, vtype=GRB.CONTINUOUS, name="t_arr", ub=5e3, lb=0)  # arrival times
        l = model.addVars(n_nodes, n_drones, vtype=GRB.INTEGER, name="load", lb=0, ub = drones[0].max_capacity)  # load at each node for each drone
        b = model.addVars(n_nodes, n_drones, vtype=GRB.CONTINUOUS, name="battery", lb=0, ub=100)  # binary variable for battery usage
        order_arrival = model.addVars(n_orders, vtype=GRB.CONTINUOUS, name="order_arrival", lb=0, ub=5e3)  # arrival time at each order node
        # 1. Each drone ends at a depot
        #for k in range(n_drones):
        #   model.addConstr(gp.quicksum(x[i, d, k] for d in range(n_depots) for i in range(n_nodes)) == 1, name=f"end_depot_{k}")

        for k in range(n_drones):
            for d in range(n_nodes-n_depots, n_nodes):
                for j in range(n_nodes):
                    if j != d:
                        model.addConstr(
                            t_arr[d, k] >= t_arr[j, k] - M * (1 - x[j, d, k]),
                            name=f"depot_arrival_last_{k}_{d}_{j}"
                        )
                        x[d, j, k].ub = 0  # no direct depot->node routes
        
        # 1a. No direct depot->depot routes
        for k in range(n_drones):
            for d1 in range(n_depots * 2):
                for d2 in range(n_depots * 2):
                    if d1 != d2:
                        x[d1, d2, k].ub = 0
        
        # 1. Each drone starts at its current node
        for k, drone in enumerate(drones):
            s = drone_start_nodes[k]
            print(f"Drone {drone.drone_id} starts at node {s} (node name: {nodes[s].name})")
            model.addConstr(gp.quicksum(x[s, j, k] for j in range(n_nodes) if s!=j) == 1, name=f"start_at_current_{k}")
            for j in range(n_nodes):
                if j != s:
                    model.addConstr(
                        t_dep[s, k] <= t_arr[j, k],
                        name=f"start_departure_first_{k}_{j}"
            )
            #model.addConstr(l[s,k] == drone.load, name=f"start_load_{k}")

        # 1b. Drone can only leave its start node after it arrives there
        for k in range(n_drones):
            s = drone_start_nodes[k]
            model.addConstr(t_dep[s, k] >= drone_earliest_dep[k], name=f"earliest_dep_{k}")
            model.addConstr(t_dep[s, k] >= t_arr[s, k], name=f"start_departure_{k}")

        # 2. Each order is served by exactly one drone
        for o, order in enumerate(orders):
            cust = n_depots * 2 + n_restaurants + o  # index of the order node in nodes
            model.addConstr(gp.quicksum(x[i, cust, k] for i in range(n_nodes) if i != cust for k in range(n_drones)) == 1, name=f"serve_order_{o}")

        # 3. Drones must arrive at customer after order time
        for o, order in enumerate(orders):
            cust = n_depots * 2 + n_restaurants + o
            order_time = order.arrival_time
            for k in range(n_drones):
                model.addConstr(t_arr[cust, k] >= order_time - M * (1 - gp.quicksum(x[i, cust, k] for i in range(n_nodes) if i != cust)), name=f"arr_after_order_{o}_{k}")

        # 4. Flow conservation (leave every node you arrive at, except depots)
        for k in range(n_drones):
            s = drone_start_nodes[k]
            for v in range(n_nodes):
                if v < n_nodes-n_depots and v!=s:  # not a depot
                    model.addConstr(
                        gp.quicksum(x[i, v, k] for i in range(n_nodes) if i != v) ==
                        gp.quicksum(x[v, j, k] for j in range(n_nodes) if j != v),
                        name=f"flow_{v}_{k}"
                    )

        # 5. Battery constraints
        for k, drone in enumerate(drones):
            s = drone_start_nodes[k]
            model.addConstr(b[s, k] == drone.battery, name=f"start_battery_{k}")
            for i in range(n_nodes):
                for j in range(n_nodes):
                    travel_time = distance_matrix[i, j] / drone.speed
                    if i != j and i >= n_depots * 2:
                        model.addConstr(
                            b[j, k] <= b[i, k] - (distance_matrix[i, j] * drone.energy_per_meter)
                              - constants.TO_land_energy + M * (1 - x[i, j, k]),
                            name=f"battery_{i}_{j}_{k}"
                        )
                    elif i != j:
                        model.addConstr(
                            b[j, k] <= 100 - (distance_matrix[i, j] * drone.energy_per_meter)
                              - constants.TO_land_energy + M * (1 - x[i, j, k]),
                            name=f"battery_from_depot_{i}_{j}_{k}"
                        )

        for o in range(n_orders):
            cust = n_depots * 2 + n_restaurants + o
            for k in range(n_drones):
                model.addConstr(
                    order_arrival[o] >= t_arr[cust, k] - M * (1 - gp.quicksum(x[i, cust, k] for i in range(n_nodes) if i != cust)),
                    name=f"arrival_lb_{o}_{k}"
                )
                model.addConstr(
                    order_arrival[o] <= t_arr[cust, k] + M * (1 - gp.quicksum(x[i, cust, k] for i in range(n_nodes) if i != cust)),
                    name=f"arrival_ub_{o}_{k}"
                )

        # 6. Capacity constraints
        for k, drone in enumerate(drones):
            for i in range(n_nodes):
                for j in range(n_nodes):
                    if i != j:
                        model.addConstr(
                            l[j, k] >= l[i, k] + q[j] - M * (1 - x[i, j, k]),
                            name=f"capacity_{i}_{j}_{k}"
                        )

        depot_ids = [n for n, node in enumerate(nodes) if hasattr(node, "depot_id")]
        for k in range(n_drones):
            for j in depot_ids:
                l[j, k].ub = 0  # load at depots is 0
        
        # 7. Restaurant before customer
        for o, order in enumerate(orders):
            rest = n_depots * 2 + o 
            cust = n_depots * 2 + n_restaurants + o
            for k in range(n_drones):
                s = drone_start_nodes[k]
                model.addConstr(
                    t_arr[cust, k] >= t_dep[rest, k] - M * (1 - gp.quicksum(x[i, cust, k] for i in range(n_nodes) if i != cust)),
                    name=f"rest_before_cust_{o}_{k}"
                )
                # If drone k serves customer, it must also visit restaurant
                if s != rest:
                    model.addConstr(
                        gp.quicksum(x[i, cust, k] for i in range(n_nodes) if i != cust) <=
                        gp.quicksum(x[i, rest, k] for i in range(n_nodes) if i != rest),
                        name=f"must_visit_restaurant_{o}_{k}"
                    )
                else:
                    model.addConstr(
                        gp.quicksum(x[i, cust, k] for i in range(n_nodes) if i == cust) == 1
                    )

        # 8. Delivery time limit (30 min from pickup at restaurant to delivery at customer)
        for o, order in enumerate(orders):
            rest = n_depots * 2 + o 
            cust = n_depots * 2 + n_restaurants + o
            for k in range(n_drones):
                travel_time = distance_matrix[rest, cust] / drones[k].speed
                model.addConstr(
                    t_arr[cust, k] - t_arr[rest, k] <= constants.pizza_cooling_time + M * (1 - gp.quicksum(x[i, cust, k] for i in range(n_nodes) if i != cust)),
                    name=f"delivery_time_{o}_{k}"
                )

        # 9. No direct depot->customer or restaurant->depot
        for k in range(n_drones):
            for d in range(n_depots * 2):
                for o in range(n_orders):
                    cust = n_depots * 2 + n_restaurants + o
                    x[d, cust, k].ub = 0
                for r in range(n_restaurants):
                    rest = n_depots * 2 + r
                    x[rest, d, k].ub = 0
                    # ToDo: consider removing this constraint

        # 10. Time propagation (if drone k travels i->j, arrival at j >= departure at i + travel_time)
        for k in range(n_drones):
            for i in range(n_nodes):
                for j in range(n_nodes):
                    if i != j:
                        travel_time = distance_matrix[i, j] / drones[k].speed
                        model.addConstr(
                            t_arr[j, k] >= t_dep[i, k] + travel_time - M * (1 - x[i, j, k]),
                            name=f"time_prop_{i}_{j}_{k}"
                        )
                        model.addConstr(
                            t_arr[j, k] <= t_dep[i, k] + travel_time + constants.max_hover_time + M * (1 - x[i, j, k]),
                        )

        # 11. Departure after arrival
        for k in range(n_drones):
            s = drone_start_nodes[k]
            for i in range(n_nodes):
                if i != s: 
                    model.addConstr(t_dep[i, k] >= t_arr[i, k] + waiting_times[i]*(0 if i >= n_nodes - n_depots and gp.quicksum(x[d,i,k] for d in range(n_depots*2)) else 1)
                                    - M * (1 - gp.quicksum(x[j, i, k] for j in range(n_nodes) if j != i)), name=f"dep_after_arr_{i}_{k}")
                if i >= n_depots * 2 and i < n_nodes - n_depots:  # if it's not a depot
                    model.addConstr(t_dep[i, k] <=t_arr[i,k] + constants.max_waiting_time
                                    + M * (1 - gp.quicksum(x[j, i, k] for j in range(n_nodes) if j != i)), name=f"dep_before_max_wait_{i}_{k}")
        # Objective: Minimize weighted sum of total distance and total delay
        weight_dist = 0.5
        weight_delay = 0.5
        weight_finish_time = 0.001
        weight_travel_time = 0.5
        weight_n_trips = 100
        #total_travel_time = gp.quicksum(
        #    travel_time_arc[i, j, k]
        #    for i in range(n_nodes) for j in range(n_nodes) if i != j for k in range(n_drones)
        #)
        total_n_trips = gp.quicksum(
            x[i, j, k] for i in range(n_nodes) for j in range(n_nodes) if i != j and not i in range(n_depots*2) and j in range(n_nodes-n_depots, n_nodes) for k in range(n_drones)
        )
        total_distance = gp.quicksum(distance_matrix[i, j] * x[i, j, k] for i in range(n_nodes) for j in range(n_nodes) if i != j for k in range(n_drones))
        #total_delay = gp.quicksum(
        #    (t_arr[n_depots * 2 + n_restaurants + o, k] - orders[o].arrival_time) *
        #    gp.quicksum(x[i, n_depots * 2 + n_restaurants + o, k] for i in range(n_nodes) if i != n_depots * 2 + n_restaurants + o)
        #    for o in range(n_orders)
        #    for k in range(n_drones)
        #)
        total_delay = gp.quicksum(order_arrival[o] - orders[o].arrival_time for o in range(n_orders))
        total_finish_time = gp.quicksum(t_dep[d, k] for d in range(n_nodes-n_depots, n_nodes) for k in range(n_drones))
        model.setObjective(weight_n_trips * total_n_trips + weight_dist * total_distance + weight_delay * total_delay + total_finish_time * weight_finish_time, GRB.MINIMIZE)
        self.set_warm_start(x, t_dep, t_arr, drones, nodes, n_orders, n_depots)
        model.optimize()
        if model.status == GRB.OPTIMAL:
            self.assign_routes(x, t_dep, t_arr, drones, nodes, n_orders)
            #print(f"total distance: {model.ObjVal:.2f} m")
            #print(f"total delay: {sum((t_arr[n_depots + n_restaurants + o, k].X - orders[o].arrival_time) * x[i, n_depots + n_restaurants + o, k].X for o in range(n_orders) for k in range(n_drones) for i in range(n_nodes) if i != n_depots + n_restaurants + o):.2f} s")
            #print(f"total finish time: {sum(x[i, d, k].X * t_arr[d, k].X for i in range(n_nodes) for d in range(n_depots) for k in range(n_drones) if i != d):.2f} s")
        elif model.status == GRB.TIME_LIMIT or model.status == GRB.INTERRUPTED or model.status == GRB.SUBOPTIMAL:
            self.assign_routes(x, t_dep, t_arr, drones, nodes, n_orders)
            print("No optimal solution found")
        if model.status == GRB.INFEASIBLE:
            model.computeIIS()
            model.write("model.ilp")
            print("Wrote IIS to model.ilp")

    def assign_routes(self, x, t_dep, t_arr, drones, nodes, n_orders):
        for k, drone in enumerate(drones):
            print(f"drone {drone.drone_id} orders {drone.restaurant_order_nodes}")
            # Collect all legs for this drone
            legs = []
            for i in range(len(nodes)):
                for j in range(len(nodes)):
                    if i != j and x[i, j, k].X > 0.5:
                        print(f"Drone {drone.drone_id} leg: {i} -> {j}, arr: {t_arr[i, k].X}, dep: {t_dep[i, k].X},")
                        legs.append((
                            i, j,
                            t_arr[i, k].X,
                            t_dep[i, k].X,
                        ))
            # Sort legs by departure time
            legs.sort(key=lambda leg: leg[3])

            if legs:

                if drone.targets and drone.state != 'waiting':
                    if hasattr(drone.targets[0], "order_id"):
                        being_delivered_orders = [t for t in drone.targets if hasattr(t, "order_id") and t.being_delivered]
                        being_delivered_orders = being_delivered_orders[1:]
                        n_being_delivered_orders = len(being_delivered_orders)
                        routes = being_delivered_orders
                        departure_times = [drone.departure_times[i] for i in range(n_being_delivered_orders)]
                        arrival_times = [drone.arrival_times[i] for i in range(n_being_delivered_orders)]
                    else:
                        routes = [drone.targets[0]]
                        departure_times = [drone.departure_times[0]] 
                        arrival_times = [drone.arrival_times[0]]
                        restaurant_order_nodes = [drone.restaurant_order_nodes[0]]
                else:
                    routes = []
                    arrival_times = []
                    departure_times = []
                    restaurant_order_nodes = []

                for leg in legs:
                    #print(f"Drone {drone.drone_id} leg: {nodes[leg[0]].name} -> {nodes[leg[1]].name}, arr: {leg[2]}, dep: {leg[3]}")
                    routes.append(nodes[leg[0]])
                    arrival_times.append(leg[2])
                    departure_times.append(leg[3])
                    if hasattr(nodes[leg[0]], "restaurant_id"):
                        restaurant_order_nodes.append(nodes[leg[0] + n_orders])

                # Add the last node (the depot)
                routes.append(nodes[legs[-1][1]])
                arrival_times.append(t_arr[legs[-1][1], k].X)
                departure_times.append(t_dep[legs[-1][1], k].X)

                #print(f"Drone {drone.drone_id} assigned targets: {[node.name for node in drone.targets]}")
                drone.set_targets(routes)
                drone.arrival_times = arrival_times
                drone.departure_times = departure_times
                drone.restaurant_order_nodes = restaurant_order_nodes
            print(f"Drone {drone.drone_id} assigned targets: {[node.name for node in drone.targets]}")
            #print(f"Arrival times: {drone.arrival_times}")
            #print(f"Departure times: {drone.departure_times}")