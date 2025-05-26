


class MissionPlanning:
    def __init__(self, Simulation):
        self.simulation = Simulation
        self.drones = self.simulation.drones
        self.depots = self.simulation.city.depots
        self.restaurants = self.simulation.city.restaurants
        self.orders = []

    def calculate_distance_to_order(self, order, point):
        # Calculate the distance between two points
        return ((order['x_delivery_loc'] - point['x']) ** 2 + (order['y_delivery_loc'] - point['y']) ** 2) ** 0.5
    
    def get_orders(self):
        return [self.simulation.order_book[order_id] for order_id in self.simulation.order_book if not self.simulation.order_book[order_id]['status']]
        
    def get_nearest_depot(self, order):
        # Get the nearest restaurant to the order
        nearest_depot = None
        min_distance = float('inf')
        for depot in self.depots:
            distance = self.calculate_distance(order, depot)
            if distance < min_distance:
                min_distance = distance
                nearest_depot = depot
        return nearest_depot

    def basic_heuristic(self):
        self.orders = self.get_orders()
        for order in self.orders:
            nearest_depot = self.get_nearest_depot(restaurantorder.

    # Approach 1: not time based
    # Output: for drone in drones:
    #    drone.orders = [location1, location2, location3]
    # Approach 2: time based
    # Output: for drone in drones:
    #   drone.orders = [[location1,time1], [location2,time2], [location3,time3]] 