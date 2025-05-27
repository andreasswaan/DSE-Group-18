


class MissionPlanning:
    def __init__(self, Simulation):
        self.simulation = Simulation
        self.drones = self.simulation.drones
        self.depots = self.simulation.city.depots
        self.restaurants = self.simulation.city.restaurants
        self.orders = []
    
    def get_orders(self):
        orders =[self.simulation.order_book[order_id] for order_id in self.simulation.order_book if not self.simulation.order_book[order_id].status]
        return orders

    def basic_heuristic(self):
        self.orders = self.get_orders()
        ready_depots = [depot for depot in self.depots if len(depot.current_drones) > 0]
        for order in list(self.orders):
            nearest_depot = order.restaurant.nearest(ready_depots)
            if nearest_depot:
                nearest_depot.current_drones[0].add_targets([order.restaurant, order, order.nearest(self.depots)])
                self.orders.remove(order)


    # Approach 1: not time based
    # Output: for drone in drones:
    #    drone.orders = [location1, location2, location3]
    # Approach 2: time based
    # Output: for drone in drones:
    #   drone.orders = [[location1,time1], [location2,time2], [location3,time3]] 