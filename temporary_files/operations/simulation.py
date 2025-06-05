import numpy as np
import matplotlib.pyplot as plt
import copy
import matplotlib.animation as animation
from temporary_files.operations.mission_planning_2 import MissionPlanning
import financial_model
import constants
import scipy.stats as stats
# pizza size to weight guide in kg
np.random.seed(40)  # for reproducibility
pizza_guide = {'s': 0.3, 'm': 0.4, 'l': 0.5}
n_drones = 2
class Point():
    def __init__(self, xpos: float, ypos: float) -> None:
        self.xpos = xpos
        self.ypos = ypos

    def distance(self, other: 'Point') -> float:
        return np.sqrt((self.xpos - other.xpos)**2 + (self.ypos - other.ypos)**2)
    
    def nearest(self, places: list['Point']) -> 'Point':
        distance = 10000000
        nearest_place = None
        for i in places:
            if self.distance(i) < distance:
                distance = self.distance(i)
                nearest_place = i
        return nearest_place

class Restaurant(Point):
    def __init__(self,
                 restaurant_dict: dict):
        super().__init__(restaurant_dict['xpos'], restaurant_dict['ypos'])
        self.restaurant_id = restaurant_dict['restaurant_id']
        self.name = restaurant_dict['name']
        self.mean_nr_orders = restaurant_dict['mean_nr_orders']
        self.mean_order_size = restaurant_dict['mean_order_size']
        self.waiting_time = constants.waiting_time_restaurant  # seconds
    
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

class Order(Point):
    def __init__(self,
                 order_dict: dict):
        super().__init__(order_dict['x_delivery_loc'], order_dict['y_delivery_loc'])
        self.order_id = order_dict['order_id']
        self.name = self.order_id
        self.restaurant_id = order_dict['restaurant_id']
        self.restaurant = order_dict['restaurant']
        self.status = order_dict['status']  # True if delivered, False otherwise
        self.time = order_dict['time']
        self.s = order_dict['s']
        self.m = order_dict['m']
        self.l = order_dict['l']
        self.being_delivered = False  # True if a drone is currently delivering this order
        self.demand = self.s + self.m + self.l
        self.arrival_time = order_dict['arrival_time']
        self.waiting_time = constants.waiting_time_customer  # seconds

class Drone(Point):
    def __init__(self,
                 drone_dict: dict):
        self.drone_id = drone_dict['drone_id']
        self.depot = depots[drone_dict['depot']]
        self.depot.current_drones.append(self)
        self.targets = []
        self.available_time = 0
        self.target = None
        self.max_battery_level = 100
        self.energy_per_meter = 0.005  # kWh per meter, this is a placeholder value
        self.speed = 10*100/6000*5  # m/s, horizontal speed
        self.distance_travelled = 0
        self.max_capacity = 10 # max number of pizzas it can carry
        self.battery = 100  # battery level in percentage
        self.departure_times = []  # list of departure times for each mission
        self.arrival_times = []  # list of arrival times for each mission
        self.simulation = None  # reference to the simulation object
        self.state = 'idle'  # state of the drone, can be 'idle', 'charging', 'delivering', etc.
        self.load = 0  # current load of the drone, in number of pizzas
        self.restaurant_order_nodes = []  # list of restaurant order nodes, used for mission planning
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
    

    def get_target(self):
        if self.targets:
            # If there are scheduled departures and the next departure is now or in the past
            #if self.departure_times and self.departure_times[0] <= self.simulation.timestamp:
            for target in self.targets:
                if isinstance(target, Order):
                    target.being_delivered = True  #ToDo: we might want to remove this restriction later
            #if isinstance(self.targets[0], Order):
            #    self.state = 'delivering'
            #else:
            #    self.state = 'other'
            self.departure_times.pop(0)
            self.arrival_times.pop(0)
            self.targets.pop(0)
            if isinstance(self.target, Restaurant):
                self.restaurant_order_nodes.pop(0)
            if isinstance(self.target, Order):
                self.target.status = True
            self.target = self.targets[0] if self.targets else None 
            self.state = 'moving'  
            
    def calculate_payload_weight(self):
        # calculate the weight of the current payload
        weight = 0
        for size, count in self.current_payload.items():
            weight += pizza_guide[size] * count
        return weight
    
    def add_targets(self, targets: list[Point]):
        self.targets.append(targets)
        if self.target is None:
            self.target = targets[0]
        if self.depot is not None:
            self.depot.current_drones.remove(self)
            self.depot = None
    
    def set_targets(self, targets: list[Point]):
        if self.target is None:
            self.state = 'waiting'
        self.targets = targets
        #self.target = self.get_target()
        if self.depot is not None:
            self.depot.current_drones.remove(self)
            self.depot = None  
        self.available_time = self.departure_times[-1] if self.departure_times else 0

    def update_drone(self, dt):
        """self.xpos += 0.5 * self.xacc * dt**2 + self.xvel * dt
        self.ypos += 0.5 * self.yacc * dt**2 + self.yvel * dt
        self.xvel += self.xacc * dt
        self.yvel += self.yacc * dt
        self.battery_level -= 0.1 * (self.xvel**2 + self.yvel**2) * dt"""
        if self.departure_times and self.departure_times[0] <= self.simulation.timestamp and self.targets and self.state == 'waiting':
            self.get_target()
        self.move_to_target(dt)
    
    def move_to_target(self, dt):
        if self.target is None:
            return
        direction_vector = np.array([self.target.xpos - self.xpos, self.target.ypos - self.ypos])
        norm = np.linalg.norm(direction_vector)
        if norm == 0:
            step_vector = np.array([0.0, 0.0])
        else:
            step_vector = direction_vector / norm * self.speed * dt
        if np.linalg.norm(direction_vector) <= np.linalg.norm(step_vector):
            self.xpos, self.ypos = self.target.xpos, self.target.ypos
            self.arrive()
            self.distance_travelled += np.linalg.norm(direction_vector)
        else:
            self.xpos += step_vector[0]
            self.ypos += step_vector[1]
            self.distance_travelled += np.linalg.norm(step_vector)

    def arrive(self):
        if isinstance(self.target, Depot):
            self.target.current_drones.append(self)
            self.depot = self.target
        #self.target = self.get_target()
        self.state = 'waiting'


class Depot(Point):
    def __init__(self,
                 depot_dict: dict):
        self.depot_id = depot_dict['depot_id']
        super().__init__(depot_dict['xpos'], depot_dict['ypos'])
        self.capacity = depot_dict['capacity']
        self.current_drones = []
        self.energy_spent: float = 0.0 #kWh
        self.name = f'Depot {self.depot_id}'
        self.waiting_time = constants.battery_swap_time

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
        for drone in self.drones:
            drone.simulation = self
        self.financial_model = financial_model.FinancialModel(self)
        self.dt = 5
        self.mp_interval = constants.mp_interval
        self.mp = MissionPlanning(self)
        self.timestamp = 0
        self.take_orders(dt=constants.initial_orders_time)  

    def change_order_volume(self, order_volume_ratio: float):
        for restaurant in self.city.restaurants:
            restaurant.mean_nr_orders *= order_volume_ratio

    def take_orders(self, dt=None):
        if dt is None:
            dt = self.dt
        for restaurant in self.city.restaurants:
            orders = restaurant.generate_orders(dt, self.inv_total_time)
            if orders:
                for order in orders:
                    order['restaurant_id'] = restaurant.restaurant_id
                    order['arrival_time'] = self.calculate_arrival_time()
                    order['restaurant'] = restaurant
                    order['time'] = self.timestamp
                    order['x_delivery_loc'], order['y_delivery_loc'] = self.city.generate_order_location()
                    order['status'] = False # not delivered
                    order_id = len(self.logger.order_log)
                    order['order_id'] = order_id
                    self.order_book[order_id] = Order(order)

                    logorder = copy.deepcopy(order)
                    logorder['order_id'] = order_id
                    del logorder['time']
                    self.logger.order_log[order['time']] = logorder

    def take_step(self):        
        self.take_orders()

        if self.timestamp % self.mp_interval == 0:
            # run mission planning
            self.mp.solve_mission_planning()
            #self.mp.basic_heuristic()

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

    def calculate_arrival_time(self):
        min_order_time: int = self.timestamp + constants.min_order_delay
        loc: float = ((min_order_time) * 4 + constants.time_window) / 5
        a_trunc, b_trunc = min_order_time, constants.time_window
        a, b = (a_trunc - loc) / constants.scale, (b_trunc - loc) / constants.scale
        order_time: float = stats.truncnorm.rvs(a, b, loc=loc, scale=constants.scale, size=1)[0]
        order_time = int(np.round(order_time / 300) * 300)
        return order_time

restaurant_dict = [{
    'xpos': 50,
    'ypos': 20,
    'restaurant_id': 0,
    'name': 'Pizza Place0',
    'mean_nr_orders': 400,
    'mean_order_size': {'small': 0, 'medium': 3, 'large': 0}
},
{
    'xpos': 50,
    'ypos': 50,
    'restaurant_id': 1,
    'name': 'Pizza Place1',
    'mean_nr_orders': 400,
    'mean_order_size': {'small': 0, 'medium': 3, 'large': 0}
},
{
    'xpos': 20,
    'ypos': 80,
    'restaurant_id': 2,
    'name': 'Pizza Place2',
    'mean_nr_orders': 400,
    'mean_order_size': {'small': 0, 'medium': 3, 'large': 0}
}]

Restaurant0 = Restaurant(restaurant_dict[0])
Restaurant1 = Restaurant(restaurant_dict[1])
Restaurant2 = Restaurant(restaurant_dict[2])
restaurants = [Restaurant0, Restaurant1, Restaurant2]

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
    {'drone_id': i, 'depot': 0 if i%2 == 0 else 1}
    for i in range(n_drones)
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
    title_text = ax.text(0.5, 1.01, '', transform=ax.transAxes, ha='center', va='bottom', fontsize=12)

    order_xs, order_ys = [], []
    drone_xs, drone_ys = [], []

    # Prepare text artists for order IDs
    order_id_texts = []
    for _ in range(100):
        txt = ax.text(0, 0, '', color='black', fontsize=8, ha='center', va='bottom')
        txt.set_visible(False)
        order_id_texts.append(txt)

    def update(frame):
        sim.take_step()
        # Collect undelivered order locations and IDs
        order_xs.clear()
        order_ys.clear()
        order_ids = []
        for order in sim.order_book.values():
            if not order.status and order.arrival_time <= sim.timestamp + constants.time_to_consider_order:
                order_xs.append(order.xpos)
                order_ys.append(order.ypos)
                order_ids.append(order.arrival_time)
        scat_orders.set_offsets(np.c_[order_xs, order_ys])

        # Update order ID texts
        for i, txt in enumerate(order_id_texts):
            if i < len(order_xs):
                txt.set_position((order_xs[i], order_ys[i] + 1.5))
                txt.set_text(str(order_ids[i]))
                txt.set_visible(True)
            else:
                txt.set_visible(False)

        # Collect drone locations
        drone_xs.clear()
        drone_ys.clear()
        for drone in sim.drones:
            drone_xs.append(drone.xpos)
            drone_ys.append(drone.ypos)
        scat_drones.set_offsets(np.c_[drone_xs, drone_ys])

        title_text.set_text(f'Simulation Map Animation\nTime: {sim.timestamp}s, Orders: {len(order_xs)}')
        return (scat_orders, scat_drones, scat_restaurants, scat_depots, title_text, *order_id_texts)

    ani = animation.FuncAnimation(
        fig, update, frames=steps, interval=interval, blit=False, repeat=False
    )
    plt.show()
n_steps = int(constants.time_window / my_sim.dt)
my_sim.change_order_volume(5)
animate_simulation(my_sim, n_steps, interval=10)