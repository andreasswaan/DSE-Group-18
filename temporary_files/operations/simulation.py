import numpy as np
import matplotlib.pyplot as plt
import copy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from temporary_files.operations.mission_planning import MissionPlanning
import financial_model
import constants
# pizza size to weight guide in kg
pizza_guide = {'s': 0.3, 'm': 0.4, 'l': 0.5}

class point():
    def __init__(self, xpos: float, ypos: float) -> None:
        self.xpos = xpos
        self.ypos = ypos

    def distance(self, other: 'point') -> float:
        return np.sqrt((self.xpos - other.xpos)**2 + (self.ypos - other.ypos)**2)
    
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

class Order(point):
    def __init__(self,
                 order_dict: dict):
        super().__init__(order_dict['x_delivery_loc'], order_dict['y_delivery_loc'])
        self.order_id = order_dict['order_id']
        self.restaurant_id = order_dict['restaurant_id']
        self.restaurant = order_dict['restaurant']
        self.status = order_dict['status']  # True if delivered, False otherwise
        self.time = order_dict['time']
        self.s = order_dict['s']
        self.m = order_dict['m']
        self.l = order_dict['l']

class Drone(point):
    def __init__(self,
                 drone_dict: dict):
        self.drone_id = drone_dict['drone_id']
        self.depot = depots[drone_dict['depot']]
        self.depot.current_drones.append(self)
        self.targets = []
        self.target = None
        self.max_battery_level = 100
        self.speed = 10*100/6000*5  # m/s, horizontal speed
        self.distance_travelled = 0
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
    
    def add_targets(self, targets: list[point]):
        self.targets.append(targets)
        if self.target is None:
            self.target = targets[0]
        if self.depot is not None:
            self.depot.current_drones.remove(self)
            self.depot = None
    
    def set_targets(self, targets: list[point]):
        self.targets=(targets)
        self.target = targets[0]
        print(targets)
        print(self.depot.current_drones)
        if self.depot is not None:
            self.depot.current_drones.remove(self)
            self.depot = None

    def update_drone(self, dt):
        """self.xpos += 0.5 * self.xacc * dt**2 + self.xvel * dt
        self.ypos += 0.5 * self.yacc * dt**2 + self.yvel * dt
        self.xvel += self.xacc * dt
        self.yvel += self.yacc * dt
        self.battery_level -= 0.1 * (self.xvel**2 + self.yvel**2) * dt"""
        if self.target is None:
            return
        direction_vector = np.array([self.target.xpos - self.xpos, self.target.ypos - self.ypos])
        step_vector = direction_vector / np.linalg.norm(direction_vector) * self.speed * dt
        if np.linalg.norm(direction_vector) < np.linalg.norm(step_vector):
            self.xpos, self.ypos = self.target.xpos, self.target.ypos
            self.arrive()
            self.distance_travelled += np.linalg.norm(direction_vector)
        else:
            self.xpos += step_vector[0]
            self.ypos += step_vector[1]
            self.distance_travelled += np.linalg.norm(step_vector)

    def arrive(self):
        if isinstance(self.target, Order):
            self.target.status = True
        self.targets.pop(0)
        if isinstance(self.target, Depot):
            self.target.current_drones.append(self)
        if self.targets:
            self.target = self.targets[0]

class Depot(point):
    def __init__(self,
                 depot_dict: dict):
        self.depot_id = depot_dict['depot_id']
        super().__init__(depot_dict['xpos'], depot_dict['ypos'])
        self.capacity = depot_dict['capacity']
        self.current_drones = []
        self.energy_spent: float = 0.0 #kWh


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
    
    def reset(self):
        self.depots = [Depot(i) for i in depot_dict]
        self.restaurants = [Restaurant(i) for i in restaurant_dict]     

class Logger:
    def __init__(self):
        self.drone_log = dict()
        self.order_log = dict()
        self.objective_log = dict()

    def reset(self):
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
        self.total_time = constants.time_window
        self.inv_total_time = 1 / self.total_time
        self.drones = [drone for depot in self.city.depots for drone in depot.current_drones]
        self.financial_model = financial_model.FinancialModel(self)
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
    
    def change_order_volume(self, order_volume_ratio: float):
        for restaurant in self.city.restaurants:
            restaurant.mean_nr_orders *= order_volume_ratio

    def take_step(self):
        
        for restaurant in self.city.restaurants:
            orders = restaurant.generate_orders(self.dt, self.inv_total_time)
            if orders:
                for order in orders:
                    order['restaurant_id'] = restaurant.restaurant_id
                    order['restaurant'] = restaurant
                    order['time'] = self.timestamp
                    order['x_delivery_loc'], order['y_delivery_loc'] = self.city.generate_order_location()
                    order['status'] = False # not delivered
                    order_id = f'ord{len(self.logger.order_log)}'
                    order['order_id'] = order_id
                    self.order_book[order_id] = Order(order)

                    logorder = copy.deepcopy(order)
                    logorder['order_id'] = order_id
                    del logorder['time']
                    self.logger.order_log[order['time']] = logorder

        if self.timestamp % self.mp_interval == 0:
            # run mission planning
            self.mp.basic_heuristic()

        for drone in self.drones:
            drone.update_drone(self.dt)
        self.timestamp += self.dt
    
    def reset(self):
        # reset the simulation
        self.order_book = dict()
        self.timestamp = 0
        self.logger.reset()
        self.city.reset()
        self.drones = [depot.current_drones for depot in self.city.depots]


restaurant_dict = [{
    'xpos': 50,
    'ypos': 20,
    'restaurant_id': 'rest1',
    'name': 'Pizza Place',
    'mean_nr_orders': 400,
    'mean_order_size': {'small': 2, 'medium': 3, 'large': 1}
}]

Restaurant0 = Restaurant(restaurant_dict[0])
restaurants = [Restaurant0]

depot_dict = [{
    'depot_id': 0,
    'xpos': 10,
    'ypos': 20,
    'capacity': 10,
    #'drones': [drone for drone in drone_list if drone['depot'] == 1]
    }, {
    'depot_id': 1,
    'xpos': 90,
    'ypos': 80,
    'capacity': 10,
    }]

Depot0 = Depot(depot_dict[0])
Depot1 = Depot(depot_dict[1])
depots = [Depot0, Depot1]

# Example: 8 drones at depot 0, 7 drones at depot 1
drone_dict = [
    {'drone_id': f'drone{i+1}', 'depot': 0 if i < 8 else 1}
    for i in range(15)
]

drone_list = [Drone(drone_dict[i]) for i in range(len(drone_dict))]

city_dict = {
    'city_name': 'Delft',
    'restaurants': restaurants,
    'depots': depots,
    'tall_buildings': [],
    'silent_zones': [],
    'population': 10000
}

my_sim = Simulation(
    city=City(city_dict),
    logger=Logger()
)

#print(my_sim.financial_model.calculate_ROI())
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
    scat_drones = ax.scatter([], [], c='orange', label='Drones', s=40, marker='o')
    ax.set_xlim(0, city.reso)
    ax.set_ylim(0, city.reso)
    ax.legend(loc='upper right')
    ax.set_title('Simulation Map Animation')

    order_xs, order_ys = [], []
    drone_xs, drone_ys = [], []

    def update(frame):
        sim.take_step()
        # Collect undelivered order locations
        order_xs.clear()
        order_ys.clear()
        for order in sim.order_book.values():
            if not order.status:
                order_xs.append(order.xpos)
                order_ys.append(order.ypos)
        scat_orders.set_offsets(np.c_[order_xs, order_ys])

        # Collect drone locations
        drone_xs.clear()
        drone_ys.clear()
        for drone in sim.drones:
            drone_xs.append(drone.xpos)
            drone_ys.append(drone.ypos)
        scat_drones.set_offsets(np.c_[drone_xs, drone_ys])

        ax.set_title(f'Simulation Map Animation\nTime: {sim.timestamp}s, Orders: {len(order_xs)}')
        return scat_orders, scat_drones

    ani = animation.FuncAnimation(
        fig, update, frames=steps, interval=interval, blit=False, repeat=False
    )
    plt.show()
n_steps = int(constants.time_window / my_sim.dt)
animate_simulation(my_sim, n_steps, interval=20)


