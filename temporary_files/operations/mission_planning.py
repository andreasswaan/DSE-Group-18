import numpy as np
import gurobipy as gp
from gurobipy import GRB
import constants
from constants import Order

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

    def setup_problem(self):
        drones=self.drones
        depots=self.depots
        restaurants=self.restaurants
        orders = self.get_orders()
        orders_to_deliver = [order for order in orders if order.arrival_time < self.simulation.timestamp + constants.time_to_consider_order]
        orders = [order for order in orders_to_deliver if not order.being_delivered]    
        drone_start_nodes = []
        drone_earliest_dep = []
        nodes = [*depots, *restaurants, *orders]
        orders_being_delivered = [order for order in orders if order.being_delivered]
        for drone in drones:
            being_delivered_orders = [t for t in drone.targets if isinstance(t, Order) and t.being_delivered]
            if being_delivered_orders:
                nodes.append(being_delivered_orders[-1])
                drone_start_nodes.append(nodes.index(being_delivered_orders[-1]))
                #drone_earliest_dep.append(drone.departure_times[drone.targets.index(being_delivered_orders[-1])])
                drone_earliest_dep.append(self.simulation.timestamp)
            else:
                if drone.targets:
                    drone_start_nodes.append(nodes.index(drone.targets[0]))
                    #drone_earliest_dep.append(drone.departure_times[0])
                    drone_earliest_dep.append(self.simulation.timestamp)
                else:
                    drone_start_nodes.append(nodes.index(drone.depot))
                    drone_earliest_dep.append(self.simulation.timestamp)
        print(f"Orders: {[order.name for order in orders]}")
        print(f"Orders being delivered: {[order.name for order in orders_being_delivered]}")
        drone_batteries = np.array([drone.battery for drone in drones])
        distance_matrix = np.zeros((len(nodes), len(nodes)))
        for i, node1 in enumerate(nodes):
            for j, node2 in enumerate(nodes):
                if i != j:
                    distance_matrix[i, j] = node1.distance(node2)        
        return drones, drone_start_nodes, drone_earliest_dep, depots, restaurants, orders, drone_batteries, nodes, distance_matrix

    def solve_mission_planning(self):
        drones, drone_start_nodes, drone_earliest_dep, depots, restaurants, orders, drone_batteries, nodes, distance_matrix = self.setup_problem()
        M = 1e5  # big-M
        n_nodes = len(nodes)
        n_drones = len(drones)
        n_orders = len(orders)
        n_restaurants = len(restaurants)
        n_depots = len(depots)
        waiting_times = []
        for i in range(n_nodes):
            if i < n_depots:
                waiting_times.append(constants.battery_swap_time)
            elif i < n_depots + n_restaurants:
                waiting_times.append(constants.waiting_time_restaurant)
            else:
                waiting_times.append(constants.waiting_time_customer)
        model = gp.Model("DARP")
        model.Params.OutputFlag = 0
        model.Params.TimeLimit = 10
        # Decision variables
        x = model.addVars(n_nodes, n_nodes, n_drones, vtype=GRB.BINARY, name="x")  # route selection
        t_dep = model.addVars(n_nodes, n_drones, vtype=GRB.CONTINUOUS, name="t_dep", ub=2e4, lb=0)  # departure times
        t_arr = model.addVars(n_nodes, n_drones, vtype=GRB.CONTINUOUS, name="t_arr", ub=2e4, lb=0)  # arrival times

        # 1. Each drone ends at a depot
        #for k in range(n_drones):
        #   model.addConstr(gp.quicksum(x[i, d, k] for d in range(n_depots) for i in range(n_nodes)) == 1, name=f"end_depot_{k}")

        for k in range(n_drones):
            for d in range(n_depots):
                for j in range(n_nodes):
                    if j != d:
                        model.addConstr(
                            t_arr[d, k] >= t_arr[j, k] - M * (1 - x[j, d, k]),
                            name=f"depot_arrival_last_{k}_{d}_{j}"
                        )
        
        # 1a. No direct depot->depot routes
        for k in range(n_drones):
            for d1 in range(n_depots):
                for d2 in range(n_depots):
                    if d1 != d2:
                        x[d1, d2, k].ub = 0
        
        # 1. Each drone starts at its current node
        for k in range(n_drones):
            s = drone_start_nodes[k]
            #print(f"Drone {k} starts at node {s} (node name: {nodes[s].name})")
            model.addConstr(gp.quicksum(x[s, j, k] for j in range(n_nodes) if s!=j) >= 1, name=f"start_at_current_{k}")
            for j in range(n_nodes):
                if j != s:
                    model.addConstr(
                        t_dep[s, k] <= t_dep[j, k] + M * (1 - x[s, j, k]),
                        name=f"start_departure_first_{k}_{j}"
            )

        # 1b. Drone can only leave its start node after it arrives there
        for k in range(n_drones):
            s = drone_start_nodes[k]
            model.addConstr(t_dep[s, k] >= drone_earliest_dep[k], name=f"earliest_dep_{k}")

        # 2. Each order is served by exactly one drone
        for o, order in enumerate(orders):
            cust = n_depots + n_restaurants + o  # index of the order node in nodes
            model.addConstr(gp.quicksum(x[i, cust, k] for i in range(n_nodes) if i != cust for k in range(n_drones)) == 1, name=f"serve_order_{o}")

        # 3. Drones must arrive at customer after order time
        for o, order in enumerate(orders):
            cust = n_depots + n_restaurants + o
            order_time = order.arrival_time
            for k in range(n_drones):
                model.addConstr(t_arr[cust, k] >= order_time - M * (1 - gp.quicksum(x[i, cust, k] for i in range(n_nodes) if i != cust)), name=f"arr_after_order_{o}_{k}")

        # 4. Flow conservation (leave every node you arrive at, except depots)
        for k in range(n_drones):
            s = drone_start_nodes[k]
            print(f"Drone {k} start node: {s} (node name: {nodes[s].name})")
            for v in range(n_nodes):
                if v >= n_depots and v!=s:  # not a depot
                    model.addConstr(
                        gp.quicksum(x[i, v, k] for i in range(n_nodes) if i != v) ==
                        gp.quicksum(x[v, j, k] for j in range(n_nodes) if j != v),
                        name=f"flow_{v}_{k}"
                    )
            if s < n_depots:
                # If the start node is a depot, it can only leave if it has arrived there
                model.addConstr(
                    gp.quicksum(x[d, j, k] for d in range(n_depots) for j in range(n_nodes) if j != d) ==
                    gp.quicksum(x[i, d, k] for d in range(n_depots) for i in range(n_nodes) if i != d),
                    name=f"flow_start_depot_{k}"
                )
            else:
                model.addConstr(
                    gp.quicksum(x[d, j, k] for d in range(n_depots) for j in range(n_nodes) if j != d) ==
                    gp.quicksum(x[i, d, k] for d in range(n_depots) for i in range(n_nodes) if i != d) - 1,
                    name=f"flow_start_depot_{k}"
                )

        # 5. Battery constraints
        for k, drone in enumerate(drones):
            max_distance = drone_batteries[k] * 500  # in meters
            model.addConstr(
                gp.quicksum(distance_matrix[i, j] * x[i, j, k] for i in range(n_nodes) for j in range(n_nodes) if i != j) <= max_distance,
                name=f"battery_{k}"
            )

        # 6. Capacity constraints
        for k, drone in enumerate(drones):
            for o, order in enumerate(orders):
                rest = n_depots + order.restaurant.restaurant_id  # index of restaurant node
                demand = order.demand
                model.addConstr(
                    demand * gp.quicksum(x[i, rest, k] for i in range(n_nodes) if i != rest) <= drone.max_capacity,
                    name=f"capacity_{o}_{k}"
                )

        # 7. Restaurant before customer
        for o, order in enumerate(orders):
            rest = n_depots + order.restaurant.restaurant_id
            cust = n_depots + n_restaurants + o
            for k in range(n_drones):
                model.addConstr(
                    t_arr[cust, k] >= t_arr[rest, k] - M * (1 - gp.quicksum(x[i, cust, k] for i in range(n_nodes) if i != cust)),
                    name=f"rest_before_cust_{o}_{k}"
                )
                # If drone k serves customer, it must also visit restaurant
                model.addConstr(
                    gp.quicksum(x[i, cust, k] for i in range(n_nodes) if i != cust) <=
                    gp.quicksum(x[i, rest, k] for i in range(n_nodes) if i != rest),
                    name=f"must_visit_restaurant_{o}_{k}"
                )

        # 8. Delivery time limit (30 min from pickup at restaurant to delivery at customer)
        for o, order in enumerate(orders):
            rest = n_depots + order.restaurant.restaurant_id
            cust = n_depots + n_restaurants + o
            for k in range(n_drones):
                travel_time = distance_matrix[rest, cust] / drones[k].speed
                model.addConstr(
                    t_arr[cust, k] - t_arr[rest, k] <= constants.pizza_cooling_time + M * (1 - gp.quicksum(x[i, cust, k] for i in range(n_nodes) if i != cust)),
                    name=f"delivery_time_{o}_{k}"
                )

        # 9. No direct depot->customer or restaurant->depot
        for k in range(n_drones):
            for d in range(n_depots):
                for o in range(n_orders):
                    cust = n_depots + n_restaurants + o
                    x[d, cust, k].ub = 0
                for r in range(n_restaurants):
                    rest = n_depots + r
                    x[rest, d, k].ub = 0

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
                    # ToDo: add upper bound on time of arrival at j

        # 11. Departure after arrival
        for k in range(n_drones):
            for i in range(n_nodes):
                model.addConstr(t_dep[i, k] >= t_arr[i, k] + waiting_times[i]
                                - M * (1 - gp.quicksum(x[j, i, k] for j in range(n_nodes) if j != i)), name=f"dep_after_arr_{i}_{k}")
                if i >= n_depots:  # if it's not a depot
                    model.addConstr(t_dep[i, k] <=t_arr[i,k] + constants.max_waiting_time
                                    + M * (1 - gp.quicksum(x[j, i, k] for j in range(n_nodes) if j != i)), name=f"dep_before_max_wait_{i}_{k}")
        # Objective: Minimize weighted sum of total distance and total delay
        weight_dist = 0.5
        weight_delay = 0.5
        weight_finish_time = 0.01

        total_distance = gp.quicksum(distance_matrix[i, j] * x[i, j, k] for i in range(n_nodes) for j in range(n_nodes) if i != j for k in range(n_drones))
        total_delay = gp.quicksum(
            (t_arr[n_depots + n_restaurants + o, k] - orders[o].arrival_time) *
            gp.quicksum(x[i, n_depots + n_restaurants + o, k] for i in range(n_nodes) if i != n_depots + n_restaurants + o)
            for o in range(n_orders)
            for k in range(n_drones)
        )
        total_finish_time = 0
        total_finish_time = gp.quicksum(x[i, d, k] * t_arr[d, k] for i in range(n_nodes) for d in range(n_depots) for k in range(n_drones) if i != d)
        model.setObjective(weight_dist * total_distance + weight_delay * total_delay + total_finish_time * weight_finish_time, GRB.MINIMIZE)

        model.optimize()
        if model.status == GRB.OPTIMAL:
            self.assign_routes(x, t_dep, t_arr, drones, nodes, waiting_times)
        else:
            print("No optimal solution found")
        if model.status == GRB.INFEASIBLE:
            model.computeIIS()
            model.write("model.ilp")
            print("Wrote IIS to model.ilp")

    def assign_routes(self, x, t_dep, t_arr, drones, nodes, waiting_times):
        for k, drone in enumerate(drones):
            # Collect all legs for this drone
            legs = []
            for i in range(len(nodes)):
                for j in range(len(nodes)):
                    if i != j and x[i, j, k].X > 0.5:
                        legs.append((
                            i, j,
                            t_arr[i, k].X,
                            t_dep[i, k].X,
                        ))
            # Sort legs by departure time
            legs.sort(key=lambda leg: leg[3])

            # Build the ordered route
            routes = []
            arrival_times = []
            departure_times = []

            if legs:

                for leg in legs:
                    print(f"Drone {drone.drone_id} leg: {nodes[leg[0]].name} -> {nodes[leg[1]].name}, arr: {leg[2]}, dep: {leg[3]}")
                    routes.append(nodes[leg[0]])
                    arrival_times.append(leg[2])
                    departure_times.append(leg[3])

                # Add the last node (the depot)
                routes.append(nodes[legs[-1][1]])
                arrival_times.append(t_arr[legs[-1][1], k].X)
                departure_times.append(arrival_times[-1] + waiting_times[legs[-1][1]])

             #   print(f"Drone {drone.drone_id} route: {[node.name for node in routes]}")
                if drone.targets:
                    if isinstance(drone.targets[0], Order):
                        being_delivered_orders = [t for t in drone.targets if isinstance(t, Order) and t.being_delivered]
                        n_being_delivered_orders = len(being_delivered_orders)
                        routes = being_delivered_orders + routes
                        drone.departure_times = [drone.departure_times[i] for i in range(n_being_delivered_orders)] + departure_times
                        drone.arrival_times = [drone.arrival_times[i] for i in range(n_being_delivered_orders)] + arrival_times
                    else:
                        routes = [drone.targets[0]] + routes
                        drone.departure_times = [drone.departure_times[0]] + departure_times
                        drone.arrival_times = [drone.arrival_times[0]] + arrival_times
                else:
                    drone.departure_times = departure_times
                    drone.arrival_times = arrival_times
                #print(f"Drone {drone.drone_id} assigned targets: {[node.name for node in drone.targets]}")
                drone.set_targets(routes)
            print(f"Drone {drone.drone_id} assigned targets: {[node.name for node in drone.targets]}")
            print(f"Departure times: {drone.departure_times}")
            print(f"Arrival times: {drone.arrival_times}")