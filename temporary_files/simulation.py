import numpy as np
import matplotlib.pyplot as plt
import copy


# pizza size to weight guide in kg
pizza_guide = {'s': 0.3, 'm': 0.4, 'l': 0.5}

class Restaurant:
    def __init__(self,
                 restaurant_dict: dict):
        self.xpos = restaurant_dict['xpos']
        self.ypos = restaurant_dict['ypos']
        self.restaurant_id = restaurant_dict['restaurant_id']
        self.name = restaurant_dict['name']
        self.mean_nr_orders = restaurant_dict['mean_nr_orders']
        self.mean_order_size = restaurant_dict['mean_order_size']
    
    def generate_orders(self, dt, inv_total_time):
        mean_nr_orders = self.mean_nr_orders * dt * inv_total_time
        num_orders = np.random.poisson(mean_nr_orders)
        orders = []
        for i in range(num_orders):
            nr_small = max(0, np.round(np.random.normal(self.mean_order_size['small'], 1)))
            nr_medium = max(0, np.round(np.random.normal(self.mean_order_size['medium'], 1)))
            nr_large = max(0, np.round(np.random.normal(self.mean_order_size['large'], 1)))
            order = {'s': nr_small, 'm': nr_medium, 'l': nr_large}
            orders.append(order)
        return orders


class Drone:
    def __init__(self,
                 drone_dict: dict):
        self.drone_id = drone_dict['drone_id']
        self.depot = drone_dict['depot']

        self.max_battery_level = 100

        # define static characteristics

        self.xpos = self.depot.xpos
        self.ypos = self.depot.ypos
        self.xvel = 0
        self.yvel = 0
        self.xacc = 0
        self.yacc = 0

        self.current_battery_level = self.max_battery_level
        self.is_charging = False

        self.current_payload = {'s': 0, 'm': 0, 'l': 0}
        self.current_payload_weight = self.calculate_payload_weight()
        self.is_delivering = False
    
    def calculate_payload_weight(self):
        # calculate the weight of the current payload
        weight = 0
        for size, count in self.current_payload.items():
            weight += pizza_guide[size] * count
        return weight
    
    def update_drone(self, dt):
        self.xpos += 0.5 * self.xacc * dt**2 + self.xvel * dt
        self.ypos += 0.5 * self.yacc * dt**2 + self.yvel * dt
        self.xvel += self.xacc * dt
        self.yvel += self.yacc * dt
        self.battery_level -= 0.1 * (self.xvel**2 + self.yvel**2) * dt



class Depot:
    def __init__(self,
                 depot_dict: dict):
        self.depot_id = depot_dict['depot_id']
        self.xpos = depot_dict['xpos']
        self.ypos = depot_dict['ypos']
        self.capacity = depot_dict['capacity']
        self.current_drones = []
    
    def initialise_current_drones(self,
                                  drones: list[Drone]):
        self.current_drones = drones

class City:
    def __init__(self,
                 city_dict: dict):
        self.city_name = city_dict['city_name']
        self.restaurants = city_dict['restaurants']
        self.depots = city_dict['depots']
        self.tall_buildings = city_dict['tall_buildings']
        self.silent_zones = city_dict['silent_zones']
        self.population = city_dict['population']

        self.reso = 100
        self.map = np.zeros((self.reso, self.reso, 5))
        # 100 x 100 2d map, 4 pieces of info at each coordinate - restaurant, depot, tall building, silent zone
        # each coordinate corresponds to a 1x1 square in the city
        self.map[:, :, 0] = self.population / (self.reso * self.reso)
        for restaurant in self.restaurants: 
            self.map[restaurant.xpos, restaurant.ypos, 1] = 1
        for depot in self.depots:
            self.map[depot.xpos, depot.ypos, 2] = 1
        for tall_building in self.tall_buildings:
            self.map[tall_building.xpos, tall_building.ypos, 3] = tall_building.height
        for silent_zone in self.silent_zones:
            self.map[silent_zone.xpos, silent_zone.ypos, 4] = 1
        
        self.population_density = self.map[:, :, 0]
    
    def generate_order_location(self):
        weights = self.population_density.flatten()
        weights /= np.sum(weights)
        index = np.random.choice(np.arange(self.reso * self.reso), p=weights)
        x = index % self.reso
        y = index // self.reso
        return x, y
        

class Logger:
    def __init__(self):
        self.drone_log = dict()
        self.order_log = dict()
        self.objective_log = dict()


class Simulation:
    def __init__(self,
                 city: City,
                 logger: Logger):
        self.city = city
        self.logger = logger
        self.order_book = dict()
        self.total_time = 3600 * 4
        self.inv_total_time = 1 / self.total_time

        self.dt = 5
        self.mp_interval = 500
        self.pp_interval = 50

        self.timestamp = 0
    
    def assign_drone_to_order(self, order_id: str):
        # assign a drone to the order
        # check if there are any drones available
        for depot in self.city.depots:
            if depot.current_drones:
                drone = depot.current_drones[0]
                # assign the drone to the order
                drone.is_delivering = True
                drone.current_payload = self.order_book[order_id]
                drone.current_payload_weight = drone.calculate_payload_weight()
                return drone
        return None
    
    def take_step(self):

        for restaurant in self.city.restaurants:
            orders = restaurant.generate_orders(self.dt, self.inv_total_time)
            if orders:
                for order in orders:
                    order['restaurant_id'] = restaurant.restaurant_id
                    order['time'] = self.timestamp
                    order['x_delivery_loc'], order['y_delivery_loc'] = self.city.generate_order_location()
                    order['status'] = False # not delivered
                    order_id = f'ord{len(self.logger.order_log)}'
                    self.order_book[order_id] = order

                    logorder = copy.deepcopy(order)
                    logorder['order_id'] = order_id
                    del logorder['time']
                    self.logger.order_log[order['time']] = logorder
        

        self.timestamp += self.dt


restaurant_dict1 = {
    'xpos': 10,
    'ypos': 20,
    'restaurant_id': 'rest1',
    'name': 'Pizza Place',
    'mean_nr_orders': 400,
    'mean_order_size': {'small': 2, 'medium': 3, 'large': 1}
}

Restaurant1 = Restaurant(restaurant_dict1)



depot_dict1 = {
    'depot_id': 'depot1',
    'xpos': 10,
    'ypos': 20,
    'capacity': 10,
}

Depot1 = Depot(depot_dict1)

drone_dict1 = {
    'drone_id': 'drone1',
    'depot': Depot1
}

Drone1 = Drone(drone_dict1)

city_dict = {
    'city_name': 'Delft',
    'restaurants': [Restaurant1],
    'depots': [Depot1],
    'tall_buildings': [],
    'silent_zones': [],
    'population': 10000
}


my_sim = Simulation(
    city=City(city_dict),
    logger=Logger()
)

for i in range(200):
    my_sim.take_step()
    print(my_sim.timestamp)
    if i % 20 == 0:
        print(len(my_sim.order_book))