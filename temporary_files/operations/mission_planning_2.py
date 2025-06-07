import os
os.environ["OMP_NUM_THREADS"] = "1"
import numpy as np
import gurobipy as gp
from gurobipy import GRB
import constants
from sklearn.cluster import KMeans

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

    def solve_mission_planning(self):
        drones, drone_start_nodes, depots, restaurants, orders, nodes, distance_matrix, order_clusters = self.setup_problem()
        for cluster in order_clusters:
            if len(cluster) > 0:
                # Split cluster into chunks of size max_orders_per_mission
                for i in range(0, len(cluster), constants.max_orders_per_mission):
                    subcluster = cluster[i:i + constants.max_orders_per_mission]
                    if len(subcluster) > 0:
                        drones = [drone for drone in drones if drone.available_time <= self.simulation.timestamp and drone.depot is not None]
                        self.solve(drones, drone_start_nodes, depots, restaurants, subcluster, nodes, distance_matrix)
        
    def setup_problem(self):
        #print("-------------------------time:", self.simulation.timestamp, "-------------------------")
        #for drone in self.drones:
        #    print(f"drone {drone.drone_id}")
            #print(f"drone {drone.drone_id} departure times: {drone.departure_times}, arrival times: {drone.arrival_times}")
        drones=[drone for drone in self.drones if drone.available_time <= self.simulation.timestamp and drone.depot is not None] 
        depots=self.depots
        orders = self.get_orders()
        orders_to_deliver = [order for order in orders if order.arrival_time < self.simulation.timestamp + constants.time_to_consider_order and order.arrival_time > self.simulation.timestamp]
        orders = [order for order in orders_to_deliver if not order.being_delivered]
        order_clusters = self.cluster_orders(orders, int(np.ceil(len(orders)/constants.max_orders_per_mission)))
        orders.sort(key=lambda order: order.arrival_time)
        orders = orders[:constants.max_orders_per_mission]  # Limit the number of orders to consider
        restaurants = [order.restaurant for order in orders]
        drone_start_nodes = [drone.depot.depot_id for drone in drones if drone.depot is not None]
        nodes = [*depots, *restaurants, *orders, *depots]
        #print(f"Orders: {[order.name for order in orders]}")
        #print(f"Orders being delivered: {[order.name for order in orders_being_delivered]}")
        distance_matrix = np.zeros((len(nodes), len(nodes)))
        for i, node1 in enumerate(nodes):
            for j, node2 in enumerate(nodes):
                if i != j:
                    distance_matrix[i, j] = node1.distance(node2)        
        return drones, drone_start_nodes, depots, restaurants, orders, nodes, distance_matrix, order_clusters

    def solve(self, drones, drone_start_nodes, depots, restaurants, orders, nodes, distance_matrix):
        #drones, drone_start_nodes, depots, restaurants, orders, nodes, distance_matrix = self.setup_problem()
        M = 1e4  # big-M
        n_nodes = len(nodes)
        n_drones = len(drones)
        n_orders = len(orders)
        if n_orders <= constants.max_orders_per_mission - 1:
            return
        if n_drones <= 0:
            return
        #print(f" -------------------------------- Time: {self.simulation.timestamp} --------------------------------")
        #print(f"Nodes: {[node.name for node in nodes]}")
        #print(f"order arrival times: {[order.arrival_time for order in orders]}")
        n_restaurants = len(restaurants)
        n_depots = len(depots)
        waiting_times = [node.waiting_time for node in nodes]  # waiting times at each node
        model = gp.Model("DARP")
        model.Params.OutputFlag = 1
        model.Params.TimeLimit = 5
        model.Params.MIPGap = 0.1  # Set a gap for suboptimal solutions
        model.Params.Heuristics = 0.05
        model.Params.Presolve = 2
        model.Params.Cuts = 2
        model.params.MIPFocus = 0  # Focus on finding feasible solutions quickly
        travel_matrix = np.zeros(np.shape(distance_matrix))
        for i in range(len(nodes)):
            for j in range(len(nodes)):
                travel_matrix[i,j] = 0 if distance_matrix[i,j] == 0 else 1
        q = []
        for i in range(len(nodes)):
            if i < n_depots:
                q.append(0)
            elif i < n_depots + n_orders:
                q.append(int(orders[i - n_depots].demand))
            elif i < n_depots + n_orders * 2:
                q.append(int(-orders[i - n_depots - n_orders].demand))
            else:
                q.append(0)
        # Decision variables
        x = model.addVars(n_nodes, n_nodes, n_drones, vtype=GRB.BINARY, name="x")  # route selection
        t_dep = model.addVars(n_nodes, n_drones, vtype=GRB.CONTINUOUS, name="t_dep", ub=3e3, lb=0)  # departure times
        t_arr = model.addVars(n_nodes, n_drones, vtype=GRB.CONTINUOUS, name="t_arr", ub=3e3, lb=0)  # arrival times
        l = model.addVars(n_nodes, n_drones, vtype=GRB.INTEGER, name="load", lb=0, ub = drones[0].max_capacity)  # load at each node for each drone
        b = model.addVars(n_nodes, n_drones, vtype=GRB.CONTINUOUS, name="battery", lb=0, ub=100)  # binary variable for battery usage
        order_arrival = model.addVars(n_orders, vtype=GRB.CONTINUOUS, name="order_arrival", lb=0, ub=5e3)  # arrival time at each order node
        
        # 1. Each drone ends at a depot
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
            for d1 in range(n_depots):
                for d2 in range(n_depots):
                    if d1 != d2:
                        x[d1, d2, k].ub = 0
        
        # 1. Each drone starts at its current node
        for k, drone in enumerate(drones):
            s = drone_start_nodes[k]
            #print(f"Drone {k} starts at node {s} ({nodes[s].name})")
            model.addConstr(gp.quicksum(x[s, j, k] for j in range(n_nodes) if s!=j) == 1, name=f"start_at_current_{k}")
            for j in range(n_nodes):
                if j != s:
                    model.addConstr(
                        t_dep[s, k] <= t_arr[j, k] + M * (1 - x[s, j, k]),
                        name=f"start_departure_first_{k}_{j}"
            )
            #model.addConstr(l[s,k] == drone.load, name=f"start_load_{k}")

        # 1b. Drone can only leave its start node after it arrives there
        for k in range(n_drones):
            s = drone_start_nodes[k]
            model.addConstr(t_dep[s, k] >= 0, name=f"earliest_dep_{k}")
            model.addConstr(t_dep[s, k] >= t_arr[s, k], name=f"start_departure_{k}")
            if s > n_depots:
                model.addConstr(t_dep[s, k] <= 0 + constants.max_waiting_time - waiting_times[s], name=f"max_wait_{k}")

       # 2. Each order is served by at most one drone
        for o, order in enumerate(orders):
            cust = n_depots + n_restaurants + o  # index of the order node in nodes
            model.addConstr(gp.quicksum(x[i, cust, k] for i in range(n_nodes) if i != cust for k in range(n_drones)) <= 1, name=f"serve_order_{o}")
       
        # 3. Drones must arrive at customer after order time
        for o, order in enumerate(orders):
            cust = n_depots + n_restaurants + o
            order_time = order.arrival_time - self.simulation.timestamp
            for k in range(n_drones):
                model.addConstr(t_arr[cust, k] >= order_time - M * (1 - gp.quicksum(x[i, cust, k] for i in range(n_nodes) if i != cust)), name=f"arr_after_order_{o}_{k}")
                model.addConstr(
                    t_arr[cust, k] <= order_time + constants.deliver_time_window + M * (1 - gp.quicksum(x[i, cust, k] for i in range(n_nodes) if i != cust)),
                    name=f"arr_before_cust_{o}_{k}"
                )
        
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
                    if i != j and i >= n_depots:
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

        # order arrival time constraints
        for o in range(n_orders):
            cust = n_depots + n_restaurants + o
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
            rest = n_depots + o 
            cust = n_depots + n_restaurants + o
            for k in range(n_drones):
                model.addConstr(
                    t_arr[cust, k] >= t_dep[rest, k] - M * (1 - gp.quicksum(x[i, cust, k] for i in range(n_nodes) if i != cust)),
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
            rest = n_depots + o 
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
                    # ToDo: consider removing this constraint

        # 10. Time propagation (if drone k travels i->j, arrival at j >= departure at i + travel_time)
        for k in range(n_drones):
            for i in range(n_nodes):
                for j in range(n_nodes):
                    if i != j:
                        travel_time = distance_matrix[i, j] / drones[k].speed
                        travel = 0 if travel_time == 0 else 1
                        model.addConstr(
                            t_arr[j, k] >= t_dep[i, k] + travel_time - M * (1 - x[i, j, k]),
                            name=f"time_prop_{i}_{j}_{k}"
                        )
                        model.addConstr(
                            t_arr[j, k] <= t_dep[i, k] + travel_time + constants.max_hover_time * travel + M * (1 - x[i, j, k]),
                        )

        # 11. Departure after arrival
        for k in range(n_drones):
            s = drone_start_nodes[k]
            for i in range(n_nodes):
                if i != s: 
                    model.addConstr(t_dep[i, k] >= t_arr[i, k] + waiting_times[i]*(0 if i >= n_nodes - n_depots and gp.quicksum(x[d,i,k] for d in range(n_depots*2)) else 1)
                                    - M * (1 - gp.quicksum(x[j, i, k] for j in range(n_nodes) if j != i)), name=f"dep_after_arr_{i}_{k}")
                if i >= n_depots and i < n_nodes - n_depots:  # if it's not a depot
                    model.addConstr(t_dep[i, k] <=t_arr[i, k] + constants.max_waiting_time
                                    + M * (1 - gp.quicksum(x[j, i, k] for j in range(n_nodes) if j != i)), name=f"dep_before_max_wait_{i}_{k}")
        # Objective: Minimize weighted sum of total distance and total delay
        weight_dist = -0.5
        weight_finish_time = -0.5
        weight_orders_delivered = 500

        total_orders_delivered = gp.quicksum(x[i, n_depots + n_restaurants + o, k] for o in range(n_orders) for i in range(n_nodes) if i != n_depots + n_restaurants + o for k in range(n_drones))
        #total_distance = gp.quicksum(distance_matrix[i, j] * x[i, j, k] for i in range(n_nodes) for j in range(n_nodes) if i != j for k in range(n_drones))
        total_finish_time = gp.quicksum(t_dep[d, k] for d in range(n_nodes-n_depots, n_nodes) for k in range(n_drones))
        
        model.setObjective(weight_orders_delivered * total_orders_delivered + weight_finish_time * total_finish_time, GRB.MAXIMIZE)
        
        model.optimize()
        if model.status == GRB.OPTIMAL:
            #print("------------------------------------- Optimal solution found -------------------------------------")
            self.assign_routes(x, t_dep, t_arr, drones, nodes, n_orders)
            #print(f"total distance: {model.ObjVal:.2f} m")
            #print(f"total delay: {sum((t_arr[n_depots + n_restaurants + o, k].X - orders[o].arrival_time) * x[i, n_depots + n_restaurants + o, k].X for o in range(n_orders) for k in range(n_drones) for i in range(n_nodes) if i != n_depots + n_restaurants + o):.2f} s")
            #print(f"total finish time: {sum(x[i, d, k].X * t_arr[d, k].X for i in range(n_nodes) for d in range(n_depots) for k in range(n_drones) if i != d):.2f} s")
        elif model.status == GRB.TIME_LIMIT or model.status == GRB.INTERRUPTED or model.status == GRB.SUBOPTIMAL:
            #print("------------------------------------- NO optimal solution found -------------------------------------")
            self.assign_routes(x, t_dep, t_arr, drones, nodes, n_orders)
        if model.status == GRB.INFEASIBLE:
            model.computeIIS()
            model.write("model.ilp")
            print("Wrote IIS to model.ilp")

    def assign_routes(self, x, t_dep, t_arr, drones, nodes, n_orders):
        for k, drone in enumerate(drones):
            # Collect all legs for this drone
            legs = []
            for i in range(len(nodes)):
                for j in range(len(nodes)):
                    if i != j and x[i, j, k].X > 0.5:
                        #print(f"Drone {drone.drone_id} leg: {i} -> {j}, arr: {t_arr[i, k].X + self.simulation.timestamp}, dep: {t_dep[i, k].X + self.simulation.timestamp},")
                        legs.append((
                            i, j,
                            t_arr[i, k].X + self.simulation.timestamp,
                            t_dep[i, k].X + self.simulation.timestamp,
                        ))
            # Sort legs by departure time
            legs.sort(key=lambda leg: leg[3])

            if legs:
                routes = []
                arrival_times = []
                departure_times = []
                restaurant_order_nodes = []
                for leg in legs:
                    #print(f"Drone {drone.drone_id} leg: {nodes[leg[0]].name} -> {nodes[leg[1]].name}, arr: {leg[2]}, dep: {leg[3]}")
                    routes.append(nodes[leg[0]])
                    arrival_times.append(leg[2])
                    departure_times.append(leg[3])
                    if hasattr(nodes[leg[0]], "mean_nr_orders"):
                        restaurant_order_nodes.append(nodes[leg[0] + n_orders])

                # Add the last node (the depot)
                routes.append(nodes[legs[-1][1]])
                arrival_times.append(t_arr[legs[-1][1], k].X)
                departure_times.append(t_dep[legs[-1][1], k].X)

                #print(f"Drone {drone.drone_id} assigned targets: {[node.name for node in drone.targets]}")
                drone.arrival_times = arrival_times
                drone.departure_times = departure_times
                #drone.restaurant_order_nodes = restaurant_order_nodes
                drone.set_targets(routes)
            #print(f"Drone {drone.drone_id} assigned targets: {[node.name for node in drone.targets]}")
            #print(f"Arrival times: {drone.arrival_times}")
            #print(f"Departure times: {drone.departure_times}")
    
    def cluster_orders(self, orders, n_clusters):
        """
        Cluster orders based on order location, restaurant location, and arrival time.
        Returns a list of clusters, each a list of orders.
        """
        if len(orders) == 0:
            return []

        # Build feature matrix: [order_x, order_y, restaurant_x, restaurant_y, arrival_time]
        features = np.array([
            [
                order.xpos,
                order.ypos,
                order.restaurant.xpos,
                order.restaurant.ypos,
            ]
            for order in orders
        ])

        # Fit KMeans
        kmeans = KMeans(n_clusters=min(n_clusters, len(orders)), random_state=0)
        labels = kmeans.fit_predict(features)

        # Group orders by cluster
        clusters = [[] for _ in range(n_clusters)]
        for order, label in zip(orders, labels):
            clusters[label].append(order)

        return clusters
