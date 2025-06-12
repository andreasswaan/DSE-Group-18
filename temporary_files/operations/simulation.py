import json
import numpy as np
import scipy.stats as stats
import copy
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from temporary_files.operations.mission_planning_2 import MissionPlanning

from pathplanning.path_planning_final import calculate_smooth_path

import financial_model
import constants
# pizza size to weight guide in kg
np.random.seed(4)  # for reproducibility
pizza_guide = {'s': 0.3, 'm': 0.4, 'l': 0.5}
n_drones = 20
class Point():
    def __init__(self, xpos: float, ypos: float) -> None:
        self.xpos = xpos
        self.ypos = ypos

    def distance(self, other: 'Point') -> float:
        return np.sqrt((self.xpos - other.xpos)**2 + (self.ypos - other.ypos)**2)
    
    def nearest(self, places: list['Point']) -> 'Point':
        distance = float('inf')
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
        #print(self.restaurant_id, self.order_id)

class Drone(Point):
    def __init__(self,
                 drone_dict: dict):
        self.drone_id = drone_dict['drone_id']
        self.mission_log = []  # list of missions the drone has completed
        self.depot = depots[drone_dict['depot']]
        self.depot.current_drones.append(self)
        self.targets = []
        self.available_time = 0
        self.target = None
        self.max_battery_level = 100
        self.energy_per_meter = constants.energy_per_metre  # kWh per meter, this is a placeholder value
        self.speed = 15 / 10  # m/s, horizontal speed
        self.distance_travelled = [0, 0, 0, 0, 0, 0]
        self.max_capacity = 5 # max number of pizzas it can carry
        self.battery = 100  # battery level in percentage
        self.departure_times = []  # list of departure times for each mission
        self.arrival_times = []  # list of arrival times for each mission
        self.simulation = None  # reference to the simulation object
        self.state = 'idle'  # state of the drone, can be 'idle', 'charging', 'delivering', etc.
        self.load = 0  # current load of the drone, in number of pizzas
        self.restaurant_order_nodes = []  # list of restaurant order nodes, used for mission planning
        self.path = []  # list of points in the path, used for plotting
        self.movement_path = []  # list of points in the path, used for movement
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
            #if isinstance(self.targets[0], Order):
            #    self.state = 'delivering'
            #else:
            #    self.state = 'other'
            self.departure_times.pop(0)
            self.arrival_times.pop(0)
            self.targets.pop(0)
            if isinstance(self.target, Order):
                self.target.status = True
            self.target = self.targets[0] if self.targets else None 
            self.calculate_path()
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
        self.mission_log.append(list(targets))  # log the current targets before setting new ones
        if self.target is None:
            self.state = 'waiting'
        self.targets = targets
        #self.target = self.get_target()
        if self.depot is not None:
            self.depot.current_drones.remove(self)
            self.depot = None  
        #self.available_time = self.departure_times[-1] if self.departure_times else 0
        for target in self.targets:
            if isinstance(target, Order):
                target.being_delivered = True 

    def update_drone(self, dt):
        """self.xpos += 0.5 * self.xacc * dt**2 + self.xvel * dt
        self.ypos += 0.5 * self.yacc * dt**2 + self.yvel * dt
        self.xvel += self.xacc * dt
        self.yvel += self.yacc * dt
        self.battery_level -= 0.1 * (self.xvel**2 + self.yvel**2) * dt"""
        if self.departure_times and self.departure_times[0] <= self.simulation.timestamp and self.targets and self.state == 'waiting':
            self.get_target()
        #self.move_to_target(dt)
        self.move_to_target_along_path(dt)

    def move_to_target(self, dt):
        if self.target is None:
            return
        direction_vector = np.array([self.target.xpos - self.xpos, self.target.ypos - self.ypos])
        norm = np.linalg.norm(direction_vector)
        if norm == 0:
            return
        else:
            step_vector = direction_vector / norm * self.speed * dt
        if np.linalg.norm(direction_vector) <= np.linalg.norm(step_vector):
            self.xpos, self.ypos = self.target.xpos, self.target.ypos
            self.arrive()
            self.distance_travelled[int(self.load)] += np.linalg.norm(direction_vector) * 10  # convert to meters
        else:
            self.xpos += step_vector[0]
            self.ypos += step_vector[1]
            self.distance_travelled[int(self.load)] += np.linalg.norm(step_vector) * 10  # convert to meters
            self.battery -= self.energy_per_meter * np.linalg.norm(step_vector) * 10  # energy consumption in kWh

    def move_to_target_along_path(self, dt):
        if self.target is None or not self.movement_path or len(self.movement_path) == 0 or len(self.movement_path) == 1:
            self.move_to_target(dt)
            return

        # Move along the movement_path, but do not modify self.path (for plotting)
        while len(self.movement_path) > 1:
            next_x, next_y = self.movement_path[1]
            direction_vector = np.array([next_x - self.xpos, next_y - self.ypos])
            norm = np.linalg.norm(direction_vector)
            if norm < self.speed * dt:
                # Move directly to the waypoint and remove it from the movement_path
                self.xpos, self.ypos = next_x, next_y
                self.movement_path.pop(0)
            else:
                # Move towards the next waypoint
                step_vector = direction_vector / norm * self.speed * dt
                self.xpos += step_vector[0]
                self.ypos += step_vector[1]
                self.distance_travelled += np.linalg.norm(step_vector)
                return  # Only move once per call

        # If only one point left, move towards it (the final target)
        if len(self.movement_path) == 1:
            final_x, final_y = self.movement_path[0]
            direction_vector = np.array([final_x - self.xpos, final_y - self.ypos])
            norm = np.linalg.norm(direction_vector)
            if norm < self.speed * dt:
                self.xpos, self.ypos = final_x, final_y
                self.arrive()
                self.movement_path = []
            else:
                step_vector = direction_vector / norm * self.speed * dt
                self.xpos += step_vector[0]
                self.ypos += step_vector[1]
                self.distance_travelled += np.linalg.norm(step_vector)

    def arrive(self):
        self.battery -= constants.TO_land_energy
        if isinstance(self.target, Depot):
            self.target.current_drones.append(self)
            self.depot = self.target
            self.available_time = self.simulation.timestamp + self.target.waiting_time
            self.depot.charge_drone(self)
        elif isinstance(self.target, Order):
            self.load -= self.target.demand
        elif isinstance(self.target, Restaurant):
            self.load += self.restaurant_order_nodes[0].demand
            self.restaurant_order_nodes.pop(0) 
        #self.target = self.get_target()
        self.state = 'waiting'

    def calculate_path(self):
        
        if self.target is None:
            return
        
        minimum_turn_radius = 70 # [m]
        map_resolution = 10 # [m]
        conversion_factor = minimum_turn_radius / map_resolution
        
        idx_start = (int(self.xpos // conversion_factor), int(self.ypos // conversion_factor))
        idx_target = (int(self.target.xpos // conversion_factor), int(self.target.ypos // conversion_factor))
        
        if idx_start == idx_target:
            self.path = [(self.xpos, self.ypos), (self.target.xpos, self.target.ypos)]
            self.movement_path = list(self.path)  # Store a copy for movement
            return
        
        self.path, _, _ = calculate_smooth_path(idx_start, idx_target, walkable, density_map=grid['weight_grid'], 
                                          MIN_TURN_RADIUS_GRID=70, alpha=0.7) # path, step_cost, weight_cost
        self.movement_path = list(self.path) 
        
        if self.path is not None and len(self.path) > 0:
            # Multiply each coordinate in self.path by conversion_factor
            self.path = [(x * conversion_factor, y * conversion_factor) for x, y in self.path]
            self.movement_path = list(self.path)  # Store a copy for movement


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

    def charge_drone(self, drone:Drone):
        energy_needed = drone.max_battery_level - drone.battery
        self.energy_spent += energy_needed * 0.00444
        drone.battery = drone.max_battery_level

class City:
    def __init__(self,
                 city_dict: dict):
        self.city_name = city_dict['city_name']
        self.restaurants = city_dict['restaurants']
        self.depots = city_dict['depots']
        self.tall_buildings = city_dict['tall_buildings']
        self.silent_zones = city_dict['silent_zones']
        self.population = city_dict['population']

        # Use bounds from population data to set the resolution
        max_x = max([cell['xpos'] for cell in self.population]) + 1
        max_y = max([cell['ypos'] for cell in self.population]) + 1
        
        self.map = np.zeros((max_x, max_y, 4))
        # 3 pieces of info at each coordinate - restaurant, depot, silent zone
        # each coordinate corresponds to a 1x1 square in the city
        for population_cell in self.population:
            self.map[population_cell['xpos'], population_cell['ypos'], 0] = population_cell['value']
        # self.map[:, :, 0] = self.population
        for restaurant in self.restaurants: 
            self.map[restaurant.xpos, restaurant.ypos, 1] = 1
        for depot in self.depots:
            self.map[depot.xpos, depot.ypos, 2] = 1
        for silent_zone in self.silent_zones:
            self.map[silent_zone.xpos, silent_zone.ypos, 3] = 1
        
        self.population_density = self.map[:, :, 0]
        
    
    def generate_order_location(self):
        weights = self.population_density.flatten()
        weights /= np.sum(weights)
        index = np.random.choice(np.arange(self.map.shape[0] * self.map.shape[1]), p=weights)
        x = index // self.map.shape[1]
        y = index % self.map.shape[1]
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
        self.dt = 10
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
        if self.timestamp < constants.time_window - constants.min_order_delay:
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

                        #logorder = copy.deepcopy(order)
                        #logorder['order_id'] = order_id
                        #del logorder['time']
                        #self.logger.order_log[order['time']] = logorder

                        logorder = copy.deepcopy(order)
                        self.logger.order_log[order['order_id']] = logorder

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

# Helper functions
def load_city_data_from_json(filename):
    """
    Loads city_name, population, restaurants, and silent_zones from a JSON city grid file.
    Returns: city_name, population (as np.array), restaurants (list), silent_zones (list)
    """
    with open(filename, 'r') as f:
        city_dict = json.load(f)
    city_name = city_dict['city_name']
    population = np.array(city_dict['population'])
    restaurants = city_dict['restaurants']
    silent_zones = city_dict['silent_zones']
    return city_name, population, restaurants, silent_zones


 
# -------- SIMULATION SETUP --------

city_name, population_dict, restaurant_dict, silent_zones_dict = \
load_city_data_from_json("temporary_files/operations/delft_city_grid_10_test.json")
    
restaurants = [Restaurant(r) for r in restaurant_dict]

# make each silent zone a point object:
silent_zones = [Point(s['xpos'], s['ypos']) for s in silent_zones_dict if s.get('value')]

# Create depot objects
depot_dict = [{
'depot_id': 0,
'xpos': 100,
'ypos': 200,
'capacity': 10,
}, {
'depot_id': 1,
'xpos': 350,
'ypos': 350,
'capacity': 10,
}]

Depot0 = Depot(depot_dict[0])
Depot1 = Depot(depot_dict[1])
depots = [Depot0, Depot1]

# Create drone objects
drone_dict = [{'drone_id': i, 'depot': 0 if i%2 == 0 else 1} for i in range(n_drones)]
drone_list = [Drone(drone_dict[i]) for i in range(len(drone_dict))] 

# Create the city object
city_dict = {
    'city_name': city_name,
    'restaurants': restaurants,
    'depots': depots,
    'tall_buildings': [],  # Placeholder, can be populated later
    'silent_zones': silent_zones,
    'population': population_dict
}

my_sim = Simulation(
    city=City(city_dict),
    logger=Logger()
)


# Setup Path Planning Grid
def load_delft_grid(path="pathplanning/data/delft_grid_data_70_border.npz"):
    data = np.load(path, allow_pickle=True)
    return {
        'weight_grid': data["weight_grid"],
        'obstacle_grid': data["obstacle_grid"],
    }
    
grid = load_delft_grid()
walkable = ~grid['obstacle_grid']

    
    
        

# -------- SIMULATION ANIMATION --------

def animate_simulation(sim, steps=100, interval=200):
    city = sim.city
    fig, ax = plt.subplots(figsize=(6, 6))
    im = ax.imshow(city.map[:, :, 0].T, cmap='YlOrRd', alpha=1, origin='lower')

    # Show silent zones as dark gray where silent zone is true
    silent_zone_mask = city.map[:, :, 3].T > 0
    silent_zone_overlay = np.zeros((city.map.shape[1], city.map.shape[0], 4))
    silent_zone_overlay[silent_zone_mask] = [0.2, 0.2, 0.2, 1]
    ax.imshow(silent_zone_overlay, origin='lower')

    scat_orders = ax.scatter([], [], c='cyan', label='Orders', s=20)
    scat_restaurants = ax.scatter(
        [r.xpos for r in city.restaurants],
        [r.ypos for r in city.restaurants],
        c='blue', marker='s', label='Restaurants', s=20, edgecolors=''
    )
    scat_depots = ax.scatter(
        [d.xpos for d in city.depots],
        [d.ypos for d in city.depots],
        c='green', marker='^', label='Depots', s=30
    )
    scat_drones = ax.scatter([], [], c='orange', label='Drones', s=20, marker='o')

    # Create a Line2D object for each drone's path
    path_lines = []
    for _ in sim.drones:
        line, = ax.plot([], [], c='magenta', lw=2, alpha=0.8, label='_nolegend_')
        path_lines.append(line)

    ax.set_xlim(0, city.map.shape[0])
    ax.set_ylim(0, city.map.shape[1])
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

        # Update each drone's path line
        for i, drone in enumerate(sim.drones):
            if hasattr(drone, 'path') and drone.path and len(drone.path) > 1:
                path_x, path_y = zip(*drone.path)
                path_lines[i].set_data(path_x, path_y)
            else:
                path_lines[i].set_data([], [])

        title_text.set_text(f'Simulation Map Animation\nTime: {sim.timestamp}s, Orders: {len(order_xs)}')
        return (scat_orders, scat_drones, scat_restaurants, scat_depots, *path_lines, title_text, *order_id_texts)

    ani = animation.FuncAnimation(
        fig, update, frames=steps, interval=interval, blit=True, repeat=False
    )
    plt.show()
n_steps = int(constants.time_window / my_sim.dt)
my_sim.change_order_volume(1)
#animate_simulation(my_sim, n_steps, interval=10)
for i in range(n_steps):
    my_sim.take_step()
print(my_sim.financial_model.calculate_daily_profit())
print(my_sim.financial_model.calculate_revenue())
for i in range (my_sim.drones[0].max_capacity):
    print(f"average distance travelled at capacity {i}: {sum(drone.distance_travelled[i] for drone in my_sim.drones) / sum(len(drone.mission_log) for drone in my_sim.drones)} m")
average_num_TO_landings = sum(
    sum(len(drone.mission_log[i]) - 1 for i in range(len(drone.mission_log))) / len(drone.mission_log) for drone in my_sim.drones) / len(my_sim.drones)
print(f"Average number of TO landings per drone: {average_num_TO_landings}")