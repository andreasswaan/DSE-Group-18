import random
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import scipy.stats as stats
matplotlib.use('TkAgg')  # Use a faster backend

animation= True
# Simulation parameters
max_order_time = 4*60*60
t_max = int(4.5*60*60)
time_step = 10
n_time_steps = int(t_max / time_step)
city_map = [10000,10000]
area_size = 100
n_areas_x = int(city_map[0] / area_size)
n_areas_y = int(city_map[1] / area_size)
n_pizzas = 100
n_drones = 20
n_depots = 2
n_restaurants = 4
endurance = 45*60
charge_time = 45*60
drone_speed = 15
speeds = [drone_speed, drone_speed*0.95, drone_speed*0.9, drone_speed*0.85, drone_speed*0.8, drone_speed*0.75]
endurances = np.array([endurance, endurance*0.9, endurance*0.8, endurance*0.7, endurance*0.6, endurance*0.5])
depletion_rates = 100 / endurances
loading_time = 60
delivering_time = 60
fraction_asap_orders = 0.5
min_order_delay = 30*60
scale=90*60 # standard deviation for the truncated normal distribution

#assumptions
# conservative:
    # 1. 200 deliveries at 1-5 pizzas each
    # 2. Drones only leave depot when they are fully charged
    # 3. Every delivery is perfectly on time
    # 4. No multiple deliveries
# non-conservative:
    # 1. 4 decently placed restaurants
    # 2. Depots are decently placed
    # 3. Drones start charging immediately when they arrive at depot
    # 4. Ideal restaurants, 15 min cooking time
    # 5. No wind or obstacles, drones fly in straight lines
# idk if conservative or non-conservative:
    # 1. Constant population density
    # 2. 1 minute loading and delivery time
    # 3. Scheduled vs. ASAP orders are weirdly distributed, although it looks reasonable
    # 4. Square city map
    # 5. Speed and endurance at max load are 75% and 50% of max speed and endurance
    #    respectively, and they scale linearly with load

class point():
    def __init__(self, id: int, x: float, y: float, t: int) -> None:
        self.x = x
        self.y = y
        self.id = id
        self.t = t

    def distance(self, other: 'point') -> float:
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def nearest(self, places: list['point']) -> 'point':
        distance = 10000000
        nearest_place = None
        for i in places:
            if self.distance(i) < distance:
                distance = self.distance(i)
                nearest_place = i
        return nearest_place

class depot(point):
    def __init__(self, id: int, x: float, y: float, t: int = 0) -> None:
        super().__init__(id, x, y, t)
        self.drones: list[drone] = []
        self.idle_drones: list[drone] = []
        self.n_idle_drones: int = 0
        self.n_drones: int = 0
        self.requests: list[list] = []  # requests are stored as [restaurant, ready_time]
        self.energy_spent: float = 0.0
        
    def receive_drone(self, drone: 'drone') -> None:
        self.n_drones += 1
        self.drones.append(drone)
        global min_battery
        if drone.battery < min_battery:
            min_battery = drone.battery
        self.energy_spent += 100 - drone.battery

    def send_drone(self, drone: 'drone', target_place: 'point') -> None:
        self.n_drones -= 1
        self.n_idle_drones -= 1
        self.idle_drones.remove(drone)
        self.drones.remove(drone)
        drone.set_target(target_place)
        drone.state = 'picking_up'
        self.energy_spent -= 100 - drone.battery

    def charge_drones(self) -> None:
        for i in self.drones:
            if i.state == 'charging':
                i.charge()
            if i.state == 'charged':
                self.n_idle_drones += 1
                self.idle_drones.append(i)
                i.state = 'idle'

    def receive_request(self, restaurant: 'restaurant', ready_time: int) -> None:
        time_to_restaurant = self.distance(restaurant) / drone_speed
        send_time = ready_time - time_to_restaurant
        self.requests.append([restaurant, send_time])

    def create_drone(self) -> None:
        new_drone = drone(len(drone_list), self.x, self.y, self.t)
        drone_list.append(new_drone)
        self.drones.append(new_drone)
        global points
        points.append(new_drone)
        self.n_drones += 1
        self.n_idle_drones += 1
        self.idle_drones.append(new_drone)

    def send_drones(self) -> None:
        for i in self.requests:
            if i[1] <= self.t:
                if self.n_idle_drones == 0:
                    self.create_drone()
                self.send_drone(self.idle_drones[0], i[0])
                self.requests.remove(i) 

    def update(self, t: int) -> None:
        self.t = t
        if t == 0:
            self.drones = []
            self.idle_drones = []
            self.n_idle_drones = 0
            self.n_drones = 0
            self.requests = []
        self.charge_drones()
        self.send_drones()

class drone(point):
    def __init__(self, id: int, x: float, y: float, t: int) -> None:
        super().__init__(id, x, y, t)
        self.charge_time: int = charge_time
        self.battery: float = 100.0
        self.speed: float = speeds[0]
        self.depletion_rate: float = depletion_rates[0]
        self.state: str = 'idle'
        self.current_order: list | None = None
        self.target: list[float] | None = None
        self.target_place: point | None = None
        self.n_pizzas: int = 0
        self.distance_travelled: float = 0.0
        self.timer: int = 0

    def set_target(self, target_place: 'point') -> None:
        self.target = [target_place.x, target_place.y]
        self.target_place = target_place

    def add_pizzas(self, n_pizzas: int) -> None:
        self.n_pizzas += n_pizzas
        self.depletion_rate = depletion_rates[self.n_pizzas]
        self.speed = speeds[self.n_pizzas]

    def move_towards_target(self) -> None:
        if self.target is None:
            return
        direction_vector = np.array([self.target_place.x - self.x, self.target_place.y - self.y])
        step_vector = direction_vector / np.linalg.norm(direction_vector) * self.speed * time_step
        if np.linalg.norm(direction_vector) < np.linalg.norm(step_vector):
            self.x, self.y = self.target_place.x, self.target_place.y
            self.target = None
            self.arrive()
            self.distance_travelled += np.linalg.norm(direction_vector)
        else:
            self.x += step_vector[0]
            self.y += step_vector[1]
            self.distance_travelled += np.linalg.norm(step_vector)
        self.battery -= self.depletion_rate * time_step

    def charge(self) -> None:
        self.battery += time_step / self.charge_time * 100
        if self.battery >= 100:
            self.battery = 100
            self.state = 'charged'

    def arrive(self) -> None:
        self.target_place.receive_drone(self)
        if isinstance(self.target_place, restaurant):
            self.state = 'loading'
            self.timer = loading_time
        elif isinstance(self.target_place, area):
            self.state = 'delivering'
            self.timer = delivering_time
        elif isinstance(self.target_place, depot):
            self.state = 'charging'
        
    def set_current_order(self, order: list) -> None:
        self.current_order = order
        self.set_target(order[0])
        self.add_pizzas(order[2])

    def update(self, t: int) -> None:
        self.t = t
        if self.state == 'loading':
            if self.timer > 0:
                self.timer -= time_step
            else:
                self.state = 'transporting'
        elif self.state == 'delivering':
            if self.timer > 0:
                self.timer -= time_step
                self.battery -= self.depletion_rate * time_step
            else:
                self.state = 'returning'
                self.set_target(self.nearest(depots))
        else:
            self.move_towards_target()

class area(point):
    def __init__(self, id: int, x1: float, x2: float, y1: float, y2:float, t:int=0):
        super().__init__(id, (x1+x2)/2, (y1+y2)/2, t)
        self.x1:float = x1
        self.x2:float = x2
        self.y1:float = y1
        self.y2:float = y2
        self.area_size:float = (x2-x1)*(y2-y1)
        self.center:list[float] = [(x1+x2)/2, (y1+y2)/2]
        density:float = 1
        self.p_order: float = (
            density * self.area_size * n_pizzas / city_map[0] / city_map[1] / (max_order_time / time_step)
        )
        self.orders_placed: int = 0
        self.orders: list[list] = [] # orders are stored as [area, order_id, n_pizzas, order_time]
        self.nearest_restaurant:restaurant = self.nearest(restaurants)

    def place_order(self, order_id: int) -> int:
        min_order_time: int = self.t + min_order_delay
        if random.random() < fraction_asap_orders + min_order_time / max_order_time * (1 - fraction_asap_orders):
            self.orders_placed += 1
            n_pizzas: int = random.randint(1, 5)
            order: list = [self, order_id, n_pizzas, 'asap']
            self.orders.append(order)
            self.nearest_restaurant.receive_order(order)
        else:
            loc: float = ((min_order_time) * 2 + max_order_time) / 3
            a_trunc, b_trunc = min_order_time, max_order_time
            a, b = (a_trunc - loc) / scale, (b_trunc - loc) / scale
            order_time: float = stats.truncnorm.rvs(a, b, loc=loc, scale=scale, size=1)[0]
            order_time = int(np.round(order_time / 300) * 300)
            self.orders_placed += 1
            n_pizzas = random.randint(1, 5)
            order = [self, order_id, n_pizzas, order_time]
            self.orders.append(order)
            self.nearest_restaurant.receive_order(order)
        return order_id + 1

    def receive_drone(self, drone: 'drone') -> None:
        self.orders_placed -= 1
        self.orders.remove(drone.current_order)
        global orders_delivered
        orders_delivered += 1
        drone.add_pizzas(-drone.current_order[2])

    def update(self, t: int) -> None:
        global order_id
        self.t = t
        order_id = self.place_order(order_id)


class restaurant(point):
    def __init__(self, id: int, x: float, y: float, t: int = 0) -> None:
        super().__init__(id, x, y, t)
        self.drones: list[drone] = []
        self.n_drones: int = 0
        self.cooking_time: int = 15 * 60
        self.orders: list[list] = []  # orders are stored as [area, order_id, n_pizzas, order_time, ready_time]
        self.nearest_depot: depot = self.nearest(depots)

    def receive_order(self, order: list) -> None:
        if order[3] == 'asap':
            ready_time: int = self.t + self.cooking_time
        else:
            ready_time = order[3] - loading_time - self.distance(order[0]) / speeds[order[2]]
        order.append(ready_time)
        self.orders.append(order)
        self.request_drone(ready_time)

    def receive_drone(self, drone: 'drone') -> None:
        self.n_drones += 1
        self.drones.append(drone)

    def send_drone(self, drone: 'drone', order: list) -> None:
        self.n_drones -= 1
        order.remove(order[4])
        self.drones.remove(drone)
        drone.set_current_order(order)

    def send_drones(self) -> None:
        ready_orders: list = [order for order in self.orders if order[4] <= self.t]
        for i in ready_orders:
            transporting_drone: drone | None = next((drone for drone in self.drones if drone.state == 'transporting'), None)
            if transporting_drone:
                self.send_drone(self.drones[0], i)
                self.orders.remove(i)

    def request_drone(self, ready_time: int) -> None:
        self.nearest_depot.receive_request(self, ready_time)

    def update(self, t: int) -> None:
        self.t = t
        self.send_drones()

# Update all points
def update_time(t: int) -> None:
    for i in points:
        i.update(t)
    for i in areas:
        if t < max_order_time and random.random() < i.p_order:
            i.update(t)

# initialize stuff
depot1 = depot(0, city_map[0]/4, city_map[1]/4)
depot2 = depot(1, 3*city_map[0]/4, 3*city_map[1]/4)
depot1.n_drones = 0
depot2.n_drones = 0
drone_list = []
orders_delivered = 0
min_battery=100
depots = [depot1, depot2]
restaurant1 = restaurant(0, city_map[0]/4, city_map[1]/2)
restaurant2 = restaurant(1, 3*city_map[0]/4, city_map[1]/2)
restaurant3 = restaurant(2, city_map[1]/2, city_map[1]/4)
restaurant4 = restaurant(3, city_map[1]/2, 3*city_map[1]/4)
restaurants = [restaurant1, restaurant2, restaurant3, restaurant4]
areas=[]
for i in range(n_areas_x):
    for j in range(n_areas_y):
        area_id = i*n_areas_y + j
        x1 = i*area_size
        x2 = (i+1)*area_size
        y1 = j*area_size
        y2 = (j+1)*area_size
        areas.append(area(area_id, x1,x2, y1,y2))
order_coords = []
order_id=0
points = []
points.extend(restaurants)
points.extend(depots)

# run simulation
for t in range(0, t_max+time_step, time_step):
    update_time(t)

#print results
print('Orders placed:', order_id)
print('Drones created:', len(drone_list))
print('Orders delivered:', orders_delivered)
print('min battery:', min_battery)
print('energy spent:', depot1.energy_spent + depot2.energy_spent)
print('average distance travelled:', sum([i.distance_travelled for i in drone_list])/len(drone_list))

#reset simulation
for i in drone_list:
    del i
order_id=0
drone_list = []
orders_delivered = 0

# Create animation
if animation:
    # Initialize the figure and axis
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(0, city_map[0])
    ax.set_ylim(0, city_map[1])
    ax.set_title("Drone Delivery Simulation")
    ax.set_xlabel("X Coordinate")
    ax.set_ylabel("Y Coordinate")

    # Plot depots and restaurants
    depot_scatter = ax.scatter([depot1.x, depot2.x], [depot1.y, depot2.y], c='blue', label='Depots', s=200)
    restaurant_scatter = ax.scatter(
        [r.x for r in restaurants], [r.y for r in restaurants], c='green', label='Restaurants', s=200
    )

    # Initialize scatter plots for drones, orders, and scheduled orders
    drone_scatter = ax.scatter([], [], c='red', label='Drones', s=50)
    order_scatter = ax.scatter([], [], c='orange', label='ASAP Orders', s=50)
    scheduled_order_scatter = ax.scatter([], [], c='purple', label='Scheduled Orders', s=50)

    # Add legend
    ax.legend(loc='upper right')

    # Add text annotations for the number of drones at each depot
    depot1_text = ax.text(depot1.x, depot1.y + 150, f"Drones: {depot1.n_drones}", color='blue', fontsize=10, ha='center')
    depot2_text = ax.text(depot2.x, depot2.y + 150, f"Drones: {depot2.n_drones}", color='blue', fontsize=10, ha='center')

    # Add a time indicator
    time_text = ax.text(0.05, 0.95, "", transform=ax.transAxes, fontsize=12, verticalalignment='top')

    # Initialize a dictionary to store text annotations for scheduled orders
    scheduled_order_texts = {}

    # Precompute area coordinates
    area_coords = {area.id: (area.x, area.y) for area in areas}

    # Function to update the animation frame
    def update(frame):
        global order_coords, drone_list

        # Clear previous orders and drones
        orders_x, orders_y = [], []
        scheduled_orders_x, scheduled_orders_y = [], []
        drones_x, drones_y = [], []

        # List to store all text annotations for scheduled orders
        scheduled_order_text_elements = []

        # Update orders
        for area in areas:
            for order in area.orders:
                if order[3] == 'asap':  # ASAP orders
                    orders_x.append(area_coords[area.id][0])
                    orders_y.append(area_coords[area.id][1])
                else:  # Scheduled orders
                    scheduled_orders_x.append(area_coords[area.id][0])
                    scheduled_orders_y.append(area_coords[area.id][1])
                    # Add or update the text annotation for the scheduled order
                    if order[1] not in scheduled_order_texts:
                        scheduled_order_texts[order[1]] = ax.text(
                            area_coords[area.id][0], area_coords[area.id][1] + 100,
                            f"{order[3] // 3600 + 18:02}:{(order[3] % 3600) // 60:02}",
                            fontsize=8, color='purple', ha='center'
                        )
                    # Add the text annotation to the list of elements to return
                    scheduled_order_text_elements.append(scheduled_order_texts[order[1]])

        # Remove text annotations for delivered scheduled orders
        active_order_ids = {order[1] for area in areas for order in area.orders}
        delivered_order_ids = set(scheduled_order_texts.keys()) - active_order_ids
        for order_id in delivered_order_ids:
            scheduled_order_texts[order_id].remove()  # Remove the text annotation from the plot
            del scheduled_order_texts[order_id]  # Remove it from the dictionary

        # Update drones
        for drone in drone_list:
            drones_x.append(drone.x)
            drones_y.append(drone.y)

        # Update scatter plot data
        if orders_x and orders_y:
            order_scatter.set_offsets(list(zip(orders_x, orders_y)))
        if scheduled_orders_x and scheduled_orders_y:
            scheduled_order_scatter.set_offsets(list(zip(scheduled_orders_x, scheduled_orders_y)))
        if drone_list:
            drone_scatter.set_offsets([(drone.x, drone.y) for drone in drone_list])

        # Update depot drone counts
        depot1_text.set_text(f"{depot1.n_drones}, {depot1.n_idle_drones}")
        depot2_text.set_text(f"{depot2.n_drones}, {depot2.n_idle_drones}")

        # Update time indicator
        current_time = frame * time_step
        hours = current_time // 3600 + 18
        minutes = (current_time % 3600) // 60
        seconds = current_time % 60
        time_text.set_text(f"Time: {hours:02}:{minutes:02}:{seconds:02}")

        # Run simulation step
        update_time(frame * time_step)

        # Return all updated elements for blitting
        return (
            drone_scatter,
            order_scatter,
            scheduled_order_scatter,
            depot1_text,
            depot2_text,
            time_text,
            *scheduled_order_text_elements  # Include all scheduled order text annotations
        )

    # Create animation
    frames = n_time_steps 
    ani = FuncAnimation(fig, update, frames=frames, interval=10, repeat=False, blit=True)

    # Show animation
    plt.show()