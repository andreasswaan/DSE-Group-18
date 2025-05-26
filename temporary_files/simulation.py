import numpy as np
import matplotlib.pyplot as plt
import copy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mission_planning import MissionPlanning
# pizza size to weight guide in kg
pizza_guide = {'s': 0.3, 'm': 0.4, 'l': 0.5}

class point():
    def __init__(self, x_pos: float, y_pos: float) -> None:
        self.x_pos = x_pos
        self.y_pos = y_pos

    def distance(self, other: 'point') -> float:
        return np.sqrt((self.x_pos - other.x_pos)**2 + (self.y_pos - other.y_pos)**2)
    
    def nearest(self, places: list['point']) -> 'point':
        distance = 10000000
        nearest_place = None
        for i in places:
            if self.distance(i) < distance:
                distance = self.distance(i)
                nearest_place = i
        return nearest_place

class Restaurant(point):
    def __init__(self,
                 restaurant_dict: dict):
        super().__init__(restaurant_dict['xpos'], restaurant_dict['ypos'])
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


class Drone(point):
    def __init__(self,
                 drone_dict: dict):
        self.drone_id = drone_dict['drone_id']
        self.depot = drone_dict['depot']

        self.max_battery_level = 100

        # define static characteristics

        super().__init__(self.depot.xpos, self.depot.ypos)
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


class Depot(point):
    def __init__(self,
                 depot_dict: dict):
        self.depot_id = depot_dict['depot_id']
        super().__init__(depot_dict['xpos'], depot_dict['ypos'])
        self.capacity = depot_dict['capacity']
        self.current_drones = depot_dict['drones']

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
        self.drones = [depot.current_drones for depot in self.city.depots]

        self.dt = 5
        self.mp_interval = 100
        self.pp_interval = 50
        self.mp = MissionPlanning(self)
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

        if self.timestamp % self.mp_interval == 0:
            # run mission planning
            self.mp.basic_heuristic()
            for depot in self.city.depots:
                for order in depot.orders:
                    order_id = f'ord{len(self.logger.order_log)}'
                    order['order_id'] = order_id
                    self.order_book[order_id] = order

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

drone_dict1 = {
    'drone_id': 'drone1',
    'depot': 1
}

drone_list = [drone_dict1]

depot_dict1 = {
    'depot_id': 1,
    'xpos': 10,
    'ypos': 20,
    'capacity': 10,
    'drones': [drone for drone in drone_list if drone['depot'] == 1]}

Depot1 = Depot(depot_dict1)

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

#for i in range(200):
#    my_sim.take_step()
#    print(my_sim.timestamp)
#    if i % 20 == 0:
#        print(len(my_sim.order_book))

def animate_simulation(sim, steps=100, interval=200):
    city = sim.city
    fig, ax = plt.subplots(figsize=(6, 6))
    im = ax.imshow(city.map[:, :, 0], cmap='Greys', alpha=0.3, origin='lower')
    scat_orders = ax.scatter([], [], c='red', label='Orders', s=30)
    scat_restaurants = ax.scatter(
        [r.xpos for r in city.restaurants],
        [r.ypos for r in city.restaurants],
        c='blue', marker='s', label='Restaurants', s=60
    )
    scat_depots = ax.scatter(
        [d.xpos for d in city.depots],
        [d.ypos for d in city.depots],
        c='green', marker='^', label='Depots', s=60
    )
    ax.set_xlim(0, city.reso)
    ax.set_ylim(0, city.reso)
    ax.legend(loc='upper right')
    ax.set_title('Simulation Map Animation')

    order_xs, order_ys = [], []

    def update(frame):
        sim.take_step()
        # Collect all order locations
        order_xs.clear()
        order_ys.clear()
        for order in sim.order_book.values():
            order_xs.append(order['x_delivery_loc'])
            order_ys.append(order['y_delivery_loc'])
        scat_orders.set_offsets(np.c_[order_xs, order_ys])
        ax.set_title(f'Simulation Map Animation\nTime: {sim.timestamp}s, Orders: {len(sim.order_book)}')
        return scat_orders,

    ani = animation.FuncAnimation(
        fig, update, frames=steps, interval=interval, blit=False, repeat=False
    )
    plt.show()
animate_simulation(my_sim, steps=200, interval=20)