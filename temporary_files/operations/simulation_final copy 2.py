import sys
import argparse
import os
import json
import numpy as np
import scipy.stats as stats
import copy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import requests
from pyproj import Transformer
from matplotlib.widgets import Button
import threading
from temporary_files.operations.db import db
import matplotlib.image as mpimg
from matplotlib.lines import Line2D

callback_done = threading.Event()
from temporary_files.operations.mission_planning_2 import MissionPlanning

from pathplanning.path_planning_final import calculate_smooth_path

import financial_model
import constants

np.random.seed(42)
pizza_guide = {"s": 0.3, "m": 0.4, "l": 0.5}
n_drones = constants.n_drones
ORDERS_URL = "http://192.168.56.1:5000/get_orders"
seen_order_ids = set()
running = False

database_orders = []


def on_snapshot(col_snapshot, changes, read_time):
    print(f"Received {len(changes)} documents")
    print(changes)
    for change in changes:
        print(f"{change.document.to_dict()}")
        order = change.document.to_dict()
        order["id"] = change.document.id  # Add document ID to the order
        database_orders.append(order)
    callback_done.set()


def start_delivery(event):
    global running
    running = True
    print("Delivery started")
    # plt.close(event.canvas.figure)


class Point:
    def __init__(self, xpos: float, ypos: float) -> None:
        self.xpos = xpos
        self.ypos = ypos

    def distance(self, other: "Point") -> float:
        return np.sqrt((self.xpos - other.xpos) ** 2 + (self.ypos - other.ypos) ** 2)

    def nearest(self, places: list["Point"]) -> "Point":
        distance = 10000000
        nearest_place = None
        for i in places:
            if self.distance(i) < distance:
                distance = self.distance(i)
                nearest_place = i
        return nearest_place


class Restaurant(Point):
    def __init__(self, restaurant_dict: dict):
        super().__init__(restaurant_dict["xpos"], restaurant_dict["ypos"])
        self.restaurant_id = restaurant_dict["restaurant_id"]
        self.name = restaurant_dict["name"]
        self.mean_nr_orders = restaurant_dict["mean_nr_orders"]
        self.mean_order_size = restaurant_dict["mean_order_size"]
        self.waiting_time = constants.waiting_time_restaurant  # seconds

    def generate_orders(self, dt, inv_total_time):
        mean_nr_orders = self.mean_nr_orders * dt * inv_total_time
        num_orders = np.random.poisson(mean_nr_orders)
        orders = []
        for i in range(num_orders):
            nr_small = max(
                0, np.round(np.random.normal(self.mean_order_size["small"], 1))
            )
            nr_medium = max(
                0, np.round(np.random.normal(self.mean_order_size["medium"], 1))
            )
            nr_large = max(
                0, np.round(np.random.normal(self.mean_order_size["large"], 1))
            )
            order = {"s": nr_small, "m": nr_medium, "l": nr_large}
            orders.append(order)
        return orders


class Order(Point):
    def __init__(self, order_dict: dict):
        super().__init__(order_dict["x_delivery_loc"], order_dict["y_delivery_loc"])
        self.order_id = order_dict["order_id"]
        self.name = self.order_id
        self.restaurant_id = order_dict["restaurant_id"]
        self.restaurant = order_dict["restaurant"]
        self.status = order_dict["status"]  # True if delivered, False otherwise
        self.time = order_dict["time"]
        self.s = order_dict["s"]
        self.m = order_dict["m"]
        self.l = order_dict["l"]
        self.being_delivered = (
            False  # True if a drone is currently delivering this order
        )
        self.demand = self.s + self.m + self.l
        self.arrival_time = order_dict["arrival_time"]
        self.waiting_time = constants.waiting_time_customer  # seconds
        self.delivered_at = None


class Drone(Point):
    def __init__(self, drone_dict: dict):
        self.drone_id = drone_dict["drone_id"]
        self.depot = depots[drone_dict["depot"]]
        self.depot.current_drones.append(self)
        self.targets = []
        self.mission_log = []  # log of missions, each mission is a list of points
        self.available_time = 0
        self.target = None
        self.max_battery_level = 100
        self.energy_per_meter = (
            constants.energy_per_metre
        )  # kWh per meter, this is a placeholder value
        self.speed = 15 / 10  # m/s, horizontal speed
        self.max_capacity = 6  # max number of pizzas it can carry
        self.distance_travelled = np.zeros(self.max_capacity + 1)
        self.battery = 100  # battery level in percentage
        self.departure_times = []  # list of departure times for each mission
        self.arrival_times = []  # list of arrival times for each mission
        self.simulation = None  # reference to the simulation object
        self.state = (
            "idle"  # state of the drone, can be 'idle', 'charging', 'delivering', etc.
        )
        self.load = 0  # current load of the drone, in number of pizzas
        self.restaurant_order_nodes = (
            []
        )  # list of restaurant order nodes, used for mission planning
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
        self.current_payload = {"s": 0, "m": 0, "l": 0}
        self.current_payload_weight = self.calculate_payload_weight()
        self.is_delivering = False

    def get_target(self):
        if self.targets:
            # If there are scheduled departures and the next departure is now or in the past
            # if self.departure_times and self.departure_times[0] <= self.simulation.timestamp:
            # if isinstance(self.targets[0], Order):
            #    self.state = 'delivering'
            # else:
            #    self.state = 'other'
            self.departure_times.pop(0)
            self.arrival_times.pop(0)
            self.targets.pop(0)
            # if isinstance(self.target, Restaurant):
            #    self.restaurant_order_nodes.pop(0)
            # if isinstance(self.target, Order):
            #    self.target.status = True
            self.target = self.targets[0] if self.targets else None
            self.calculate_path()
            self.state = "moving"

    def calculate_payload_weight(self):
        # calculate the weight of the current payload
        weight = 0
        for size, count in self.current_payload.items():
            weight += pizza_guide[size] * count
        return weight

    def find_battery_and_time_required(self, targets: list[Point]) -> float:
        if not targets:
            return 0.0
        arrival_times = []
        departure_times = []
        for j in range(len(targets)):
            short_targets = targets[: j + 1]
            total_distance = 0.0
            for i in range(len(short_targets) - 1):
                total_distance += short_targets[i].distance(short_targets[i + 1])
            time_needed = total_distance / self.speed * 1.2 + sum(
                [target.waiting_time for target in short_targets[1:]]
            )
            arrival_times.append(time_needed)
            departure_times.append(arrival_times[-1] + short_targets[-1].waiting_time)
            if j == len(short_targets) - 1:
                energy_needed = (
                    total_distance * self.energy_per_meter * 10 * 1.2
                    + constants.TO_land_energy * (len(short_targets) - 1)
                )
        return energy_needed, time_needed, arrival_times, departure_times

    def check_planned_orders(
        self, path: list[Point], planned_orders: list[Order], arrival_times: list[float]
    ):
        for planned in planned_orders:
            time_to_order = arrival_times[path.index(planned) - 1]
            if (
                self.simulation.timestamp + time_to_order
                > planned.arrival_time + constants.deliver_time_window
                or self.simulation.timestamp + time_to_order < planned.arrival_time
            ):
                return False
        return True

    def greedy_heuristic(self):
        if self.target is None:
            self.state = "moving"
        orders = self.simulation.mp.get_orders()
        orders = [
            order
            for order in orders
            if not order.being_delivered
            and order.status == False
            and order.demand + self.load <= self.max_capacity
        ]
        orders = sorted(orders, key=lambda x: x.restaurant.distance(self))
        orders = [
            orders[i] for i in range(min(len(orders), 10))
        ]  # limit the number of orders to max_orders_per_mission
        if not orders:
            return
        targets = list(self.targets)
        if self.target is not None:
            first_node = self.target
        else:
            first_node = self.depot

        for order in orders:
            if self.load == 0:
                path = [first_node, order.restaurant, order, order.nearest(depots)]
                planned_orders = False
            else:
                planned_orders = [
                    target for target in targets if isinstance(target, Order)
                ]
                nodes = [order.restaurant] + planned_orders
                path = [first_node]
                while nodes:
                    nearest_node = path[-1].nearest(nodes)
                    path.append(nearest_node)
                    nodes.remove(nearest_node)
                    if nearest_node == order.restaurant:
                        nodes.append(order)
                path.append(order.nearest(depots))

            energy_needed, _, arrival_times, departure_times = (
                self.find_battery_and_time_required(path)
            )
            # if planned_orders:
            #    if not self.check_planned_orders(path, planned_orders, arrival_times):
            #        continue
            time_to_order = arrival_times[path.index(order) - 1]
            if self.battery >= energy_needed + 20:
                self.targets = []
                self.departure_times = []
                self.arrival_times = []
                self.restaurant_order_nodes = []
                self.departure_times.append(self.simulation.timestamp)
                self.arrival_times.append(self.simulation.timestamp)
                self.targets.append(first_node)
                order.being_delivered = True
                self.targets.extend(path[1:])
                self.departure_times.extend(departure_times[1:])
                self.arrival_times.extend(arrival_times[1:])
                for target in self.targets:
                    if isinstance(target, Order):
                        self.restaurant_order_nodes.append(order)
                if self.depot is not None:
                    self.depot.current_drones.remove(self)
                    self.depot = None
                print(
                    f"Drone {self.drone_id} assigned targets: {[node.name for node in self.targets]}"
                )
                return

    def set_targets(self, targets: list[Point]):
        self.mission_log.append(
            list(targets)
        )  # log the current targets before setting new ones
        if self.target is None:
            self.state = "waiting"
        self.targets = targets
        # self.target = self.get_target()
        if self.depot is not None:
            self.depot.current_drones.remove(self)
            self.depot = None
        # self.available_time = self.departure_times[-1] if self.departure_times else 0
        for target in self.targets:
            if isinstance(target, Order):
                target.being_delivered = (
                    True  # ToDo: we might want to remove this restriction later
                )

    def update_drone(self, dt):
        if (
            self.available_time < self.simulation.timestamp
            and self.depot is not None
            or self.departure_times
            and self.departure_times[0] <= self.simulation.timestamp
            and self.targets
            and self.state == "waiting"
        ):
            self.greedy_heuristic()
            self.get_target()
        # self.move_to_target(dt)
        self.move_to_target_along_path(dt)

    def arrive(self):
        self.battery -= constants.TO_land_energy
        if isinstance(self.target, Depot):
            self.target.current_drones.append(self)
            self.depot = self.target
            self.available_time = self.simulation.timestamp + self.target.waiting_time
            self.depot.charge_drone(self)
        elif isinstance(self.target, Order):
            self.load -= self.target.demand
            self.departure_times[0] = (
                self.simulation.timestamp + self.target.waiting_time
            )
            self.target.delivered_at = self.simulation.timestamp
            # if (
            #    self.simulation.timestamp
            #    > self.target.arrival_time + constants.deliver_time_window
            # ):
            if False:
                print(
                    f"Warning: Drone {self.drone_id} delivered order {self.target.order_id} late at {self.simulation.timestamp}. \
                      Expected delivery time was {self.target.arrival_time}."
                )
            else:
                self.target.status = True
        elif isinstance(self.target, Restaurant):
            self.load += self.restaurant_order_nodes[0].demand
            self.restaurant_order_nodes.pop(0)
            self.departure_times[0] = (
                self.simulation.timestamp + self.target.waiting_time
            )
        # self.target = self.get_target()
        self.state = "waiting"

    def move_to_target(self, dt):
        if self.target is None:
            return
        direction_vector = np.array(
            [self.target.xpos - self.xpos, self.target.ypos - self.ypos]
        )
        norm = np.linalg.norm(direction_vector)
        if norm == 0:
            step_vector = np.array([0.0, 0.0])
            return
        else:
            step_vector = direction_vector / norm * self.speed * dt
        if np.linalg.norm(direction_vector) <= np.linalg.norm(step_vector):
            self.xpos, self.ypos = self.target.xpos, self.target.ypos
            self.arrive()
            # self.distance_travelled[int(self.load)] += (
            #    np.linalg.norm(direction_vector) * 10
            # )
            return
        else:
            self.xpos += step_vector[0]
            self.ypos += step_vector[1]
            # self.distance_travelled[int(self.load)] += np.linalg.norm(step_vector) * 10

    def move_to_target_along_path(self, dt):
        if self.target is None:
            return
        if not self.movement_path or len(self.movement_path) == 0:
            # print(f"Warning: No path found for drone {self.drone_id}, moving directly toward target!")
            self.move_to_target(dt)
            return

        distance_per_frame = self.speed * dt

        # If only one point left, move directly to the target (the final target)
        if len(self.movement_path) == 1:
            self.xpos, self.ypos = self.target.xpos, self.target.ypos
            self.arrive()
            """self.distance_travelled[int(self.load)] += (
                np.linalg.norm(
                    np.array([self.target.xpos, self.target.ypos])
                    - np.array([self.xpos, self.ypos])
                )
                * 10
            )"""
            self.movement_path = []
            return

        # Move along as many segments as possible
        temp = 0
        dist = 0
        start = np.array(self.movement_path[0])
        while temp + 1 < len(self.movement_path):
            seg = np.array(self.movement_path[temp + 1]) - np.array(
                self.movement_path[temp]
            )
            seg_len = np.linalg.norm(seg)
            if dist + seg_len < distance_per_frame:
                dist += seg_len
                temp += 1
            else:
                break

        remaining = distance_per_frame - dist

        # Interpolate within the next segment
        start = np.array(self.movement_path[temp - 1])
        end = np.array(self.movement_path[temp])
        direction = end - start
        segment_length = np.linalg.norm(direction)
        if segment_length > 0:
            step = direction / segment_length * remaining
            self.xpos, self.ypos = start + step
        else:
            self.xpos, self.ypos = end
        # Remove all waypoints up to temp
        self.movement_path = self.movement_path[temp:]
        # self.distance_travelled[int(self.load)] += distance_per_frame * 10
        return

    def calculate_path(self):

        if self.target is None:
            return

        minimum_turn_radius = 70  # [m]
        map_resolution = 10  # [m]
        conversion_factor = minimum_turn_radius / map_resolution

        idx_start = (
            int(self.xpos // conversion_factor),
            int(self.ypos // conversion_factor),
        )
        idx_target = (
            int(self.target.xpos // conversion_factor),
            int(self.target.ypos // conversion_factor),
        )

        if idx_start == idx_target:
            self.path = [(self.xpos, self.ypos), (self.target.xpos, self.target.ypos)]
            self.movement_path = list(self.path)  # Store a copy for movement
            return

        self.path, _, _ = calculate_smooth_path(
            idx_start, idx_target, walkable, density_map=grid["weight_grid"], alpha=0.3
        )  # path, step_cost, weight_cost
        self.movement_path = list(self.path)

        if self.path is not None and len(self.path) > 0:
            # Multiply each coordinate in self.path by conversion_factor
            self.path = [
                (x * conversion_factor, y * conversion_factor) for x, y in self.path
            ]
            self.movement_path = list(self.path)  # Store a copy for movement
        else:
            print(
                f"Warning: No path found for drone {self.drone_id} from ({self.xpos}, {self.ypos}) to ({self.target.xpos}, {self.target.ypos})."
            )


class Depot(Point):
    def __init__(self, depot_dict: dict):
        self.depot_id = depot_dict["depot_id"]
        super().__init__(depot_dict["xpos"], depot_dict["ypos"])
        self.capacity = depot_dict["capacity"]
        self.current_drones = []
        self.energy_spent: float = 0.0  # kWh
        self.name = f"Depot {self.depot_id}"
        self.waiting_time = constants.battery_swap_time

    def charge_drone(self, drone: Drone):
        energy_needed = drone.max_battery_level - drone.battery
        self.energy_spent += energy_needed * 0.00444
        if drone.battery < 20:
            print(
                f"WARNING: Drone {drone.drone_id} arrived at depot with battery too low:{drone.battery}% \n \
                  ----------------------------------------------------------------------------------------------"
            )
        drone.battery = drone.max_battery_level


class City:
    def __init__(self, city_dict: dict):
        self.city_name = city_dict["city_name"]
        self.restaurants = city_dict["restaurants"]
        self.depots = city_dict["depots"]
        self.tall_buildings = city_dict["tall_buildings"]
        self.silent_zones = city_dict["silent_zones"]
        self.population = city_dict["population"]

        # Use bounds from population data to set the resolution
        max_x = max([cell["xpos"] for cell in self.population]) + 1
        max_y = max([cell["ypos"] for cell in self.population]) + 1

        self.map = np.zeros((max_x, max_y, 4))
        # 3 pieces of info at each coordinate - restaurant, depot, silent zone
        # each coordinate corresponds to a 1x1 square in the city
        for population_cell in self.population:
            self.map[population_cell["xpos"], population_cell["ypos"], 0] = (
                population_cell["value"]
            )
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
        index = np.random.choice(
            np.arange(self.map.shape[0] * self.map.shape[1]), p=weights
        )
        x = index // self.map.shape[1]
        y = index % self.map.shape[1]
        return x, y

    def reset(self):
        for depot in self.depots:
            depot.current_drones = []
            depot.energy_spent = 0.0
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
    def __init__(self, city: City, logger: Logger):
        self.city = city
        self.logger = logger
        self.order_book = dict()
        self.total_time = constants.time_window
        self.inv_total_time = 1 / self.total_time
        self.drones = [
            drone for depot in self.city.depots for drone in depot.current_drones
        ]
        for drone in self.drones:
            drone.simulation = self
        self.financial_model = financial_model.FinancialModel(self)
        self.dt = 10
        self.mp_interval = constants.mp_interval
        self.mp = MissionPlanning(self)
        self.timestamp = 0
        self.take_orders(dt=constants.initial_orders_time)
        self.weight = -0.1
        self.orders_per_time = []
        self.orders_interval = 300

    def change_order_volume(self, order_volume_ratio: float):
        for restaurant in self.city.restaurants:
            restaurant.mean_nr_orders *= order_volume_ratio

    def take_orders(self, dt=None):
        if dt is None:
            dt = self.dt
        for restaurant in self.city.restaurants:
            if self.timestamp < constants.time_window - constants.min_order_delay:
                orders = restaurant.generate_orders(dt, self.inv_total_time)
                if orders:
                    for order in orders:
                        order["restaurant_id"] = restaurant.restaurant_id
                        order["arrival_time"] = self.calculate_arrival_time()
                        order["restaurant"] = restaurant
                        order["time"] = self.timestamp
                        order["x_delivery_loc"], order["y_delivery_loc"] = (
                            self.city.generate_order_location()
                        )
                        order["status"] = False  # not delivered
                        order_id = len(self.logger.order_log)
                        order["order_id"] = order_id
                        self.order_book[order_id] = Order(order)

                        logorder = copy.deepcopy(order)
                        logorder["order_id"] = order_id
                        del logorder["time"]
                        self.logger.order_log[order["time"]] = logorder

    def take_step(self):
        self.take_orders()

        # if self.timestamp % self.mp_interval == 0:
        # run mission planning
        # self.mp.solve_mission_planning(weight=self.weight)
        # self.mp.basic_heuristic()
        # response = requests.get(ORDERS_URL)
        # orders = response.json()
        orders = database_orders
        print(len(orders))
        if len(orders) < 30:  # placeholder
            for order in orders:

                order["ypos"] = order["location"]["lat"]
                order["xpos"] = order["location"]["lng"]
                transformer = Transformer.from_crs(
                    "EPSG:4326", "EPSG:28992", always_xy=True
                )
                order["xpos"], order["ypos"] = transformer.transform(
                    order["xpos"], order["ypos"]
                )
                order["xpos"] = int((order["xpos"] - 81743.0) / 10)
                order["ypos"] = int((order["ypos"] - 442446.208000008) / 10)
                # Use a unique identifier for each order, e.g., a combination of xpos, ypos, time, pizzas
                order_id = (
                    order["xpos"],
                    order["ypos"],
                    order["initials"],
                    order["restaurant"],
                )

                # print(f"Received order from web: {order}")
                if order_id not in seen_order_ids:
                    if (
                        order["xpos"] < 0
                        or order["xpos"] > legal_order_grid.shape[0] - 1
                        or order["ypos"] < 0
                        or order["ypos"] > legal_order_grid.shape[1] - 1
                    ):
                        print(
                            f"Order at ({order['xpos']}, {order['ypos']}) is not in a the map, skipping."
                        )
                        seen_order_ids.add(order_id)
                        continue
                    if not legal_order_grid[order["ypos"], order["xpos"]]:
                        print(
                            f"Order at ({order['xpos']}, {order['ypos']}) is not in a legal order zone, skipping."
                        )
                        seen_order_ids.add(order_id)
                        continue
                    order_dict = {
                        "order_id": len(self.logger.order_log),
                        "restaurant_id": order["restaurant"],
                        "time": self.timestamp,
                        "s": int(0),
                        "m": int(3),
                        "l": int(0),
                        "arrival_time": 200 + self.timestamp,
                        "restaurant": next(
                            (
                                r
                                for r in self.city.restaurants
                                if r.name == order["restaurant"]
                            ),
                            None,
                        ),
                        "status": False,  # not delivered
                        "x_delivery_loc": float(order["xpos"]),
                        "y_delivery_loc": float(order["ypos"]),
                    }
                    self.order_book[order_id] = Order(order_dict)
                    self.order_book[order_id].initials = order["initials"]
                    print(f"Added order from web: {order}")
                    seen_order_ids.add(order_id)
        if len(self.order_book) > 0:
            for drone in self.drones:
                drone.update_drone(self.dt)
            self.timestamp += self.dt

    def reset(self):
        # reset the simulation
        self.order_book = dict()
        self.timestamp = 0
        self.logger.reset()
        self.city.reset()
        global drone_list
        drone_list = [Drone(drone_dict[i]) for i in range(len(drone_dict))]
        # for drone in drone_list:
        #    drone.depot.current_drones.append(drone)
        self.drones = [drone for drone in drone_list if drone.depot is not None]
        print(f"Resetting simulation with {len(self.drones)} drones.")
        for drone in self.drones:
            drone.simulation = self
        self.mp = MissionPlanning(self)
        self.financial_model = financial_model.FinancialModel(self)

    def calculate_arrival_time(self):
        min_order_time: int = self.timestamp + constants.min_order_delay
        loc: float = ((min_order_time) * 4 + constants.time_window) / 5
        a_trunc, b_trunc = min_order_time, constants.time_window
        a, b = (a_trunc - loc) / constants.scale, (b_trunc - loc) / constants.scale
        order_time: float = stats.truncnorm.rvs(
            a, b, loc=loc, scale=constants.scale, size=1
        )[0]
        order_time = int(np.round(order_time / 300) * 300)
        return order_time


# Helper functions
def load_city_data_from_json(filename):
    """
    Loads city_name, population, restaurants, and silent_zones from a JSON city grid file.
    Returns: city_name, population (as np.array), restaurants (list), silent_zones (list)
    """
    with open(filename, "r") as f:
        city_dict = json.load(f)
    city_name = city_dict["city_name"]
    population = np.array(city_dict["population"])
    restaurants = city_dict["restaurants"]
    silent_zones = city_dict["silent_zones"]
    return city_name, population, restaurants, silent_zones


# -------- SIMULATION SETUP --------

city_name, population_dict, restaurant_dict, silent_zones_dict = (
    load_city_data_from_json("temporary_files/operations/delft_city_grid_10_test.json")
)

restaurants = [Restaurant(r) for r in restaurant_dict]

# make each silent zone a point object:
silent_zones = [
    Point(s["xpos"], s["ypos"]) for s in silent_zones_dict if s.get("value")
]


# Setup Path Planning Grid
# def load_delft_grid(path="pathplanning/data/delft_grid_data_70_symposium.npz"):
def load_delft_grid(path="pathplanning/data/delft_grid_data_70_symposium.npz"):
    data = np.load(path, allow_pickle=True)
    return {
        "weight_grid": data["weight_grid"],
        "obstacle_grid": data["obstacle_grid"],
        "legal_grid": data["legal_order_grid"],
    }


grid = load_delft_grid()
grid_fine = load_delft_grid(path="pathplanning/data/delft_grid_data_10_symposium.npz")
# grid_fine = load_delft_grid()
legal_order_grid = grid_fine["legal_grid"]
walkable = ~grid["obstacle_grid"]

scale_factor = 7
obstacle_grid = grid["obstacle_grid"]
# Use nearest neighbor to preserve original values
obstacle_grid_7x = np.kron(obstacle_grid, np.ones((scale_factor, scale_factor)))
# weight_grid_7x now has the same values as weight_grid, but each cell is expanded to a 7x7 block


# -------- SIMULATION ANIMATION --------


def animate_simulation(sim, steps=100, interval=200, save_path=None):
    city = sim.city
    fig, ax = plt.subplots(figsize=(7, 7))

    img = mpimg.imread("temporary_files/operations/Delft_google_maps.png")
    im = ax.imshow(
        img,
        extent=[-400, city.map.shape[0] * 10 + 100, -50, city.map.shape[1] * 10 + 120],
        zorder=0,
    )
    # fig.canvas.mpl_connect("button_press_event", start_delivery)
    # Plot the population density as a static background
    """im = ax.imshow(
        city.map[:, :, 0].T,
        cmap="YlOrRd",
        alpha=1,
        origin="lower",
        extent=[0, city.map.shape[0] * 10, 0, city.map.shape[1] * 10],
    )"""
    img_legal = ax.imshow(
        legal_order_grid,
        cmap="Blues",
        alpha=0.2,
        origin="lower",
        zorder=15,
        interpolation="none",
    )
    img_legal.set_extent(
        [
            0,
            legal_order_grid.shape[0] * 5960 / 7400 * 10,
            0,
            legal_order_grid.shape[1] * 10 * 7400 / 5960,
        ]
    )
    # Show silent zones as dark gray where silent zone is true
    silent_zone_mask = city.map[:, :, 3].T > 0
    silent_zone_overlay = np.zeros((city.map.shape[1], city.map.shape[0], 4))
    silent_zone_overlay[silent_zone_mask] = [0.2, 0.2, 0.2, 1]
    ax.imshow(
        silent_zone_overlay,
        origin="lower",
        extent=[0, city.map.shape[0] * 10, 0, city.map.shape[1] * 10],
    )

    # Scale all coordinates by 10
    scat_orders = ax.scatter([], [], c="cyan", label="Orders", s=20)
    scat_restaurants = ax.scatter(
        [r.xpos * 10 for r in city.restaurants],
        [r.ypos * 10 for r in city.restaurants],
        c="blue",
        marker="s",
        label="Restaurants",
        s=20,
        edgecolors="black",
    )
    scat_depots = ax.scatter(
        [d.xpos * 10 for d in city.depots],
        [d.ypos * 10 for d in city.depots],
        c="green",
        marker="^",
        label="Depots",
        s=150,
        edgecolors="black",
    )
    scat_drones = ax.scatter(
        [], [], c="lime", label="Drones", s=30, marker="o", zorder=20
    )

    # Create a Line2D object for each drone's path
    path_lines = []
    for _ in sim.drones:
        (line,) = ax.plot(
            [], [], c="magenta", lw=2, alpha=0.8, label="_nolegend_", zorder=10
        )
        path_lines.append(line)

    handles, labels = ax.get_legend_handles_labels()
    custom_handle = Line2D(
        [0],
        [0],
        marker="o",
        color="w",
        markerfacecolor="black",
        markersize=8,
        label="no-fly zones",
    )
    handles.append(custom_handle)
    labels.append("no-fly zones")
    ax.set_xlim(0, city.map.shape[0] * 10)
    ax.set_ylim(0, city.map.shape[1] * 10)
    ax.set_xlabel("X position (m)")
    ax.set_ylabel("Y position (m)")
    ax.legend(handles=handles, labels=labels, loc="upper right")
    title_text = ax.text(
        0.5, 1.01, "", transform=ax.transAxes, ha="center", va="bottom", fontsize=12
    )
    time_text = ax.text(
        0.2,
        0.95,
        "",
        transform=ax.transAxes,
        ha="center",
        va="bottom",
        fontsize=12,
        animated=True,
    )

    order_xs, order_ys = [], []
    drone_xs, drone_ys = [], []
    order_initials_texts = []

    def update(frame):
        sim.take_step()
        # Collect undelivered order locations and IDs
        for txt in order_initials_texts:
            txt.remove()
        order_initials_texts.clear()
        order_xs.clear()
        order_ys.clear()
        for order in sim.order_book.values():
            if (
                not order.status
                and order.arrival_time
                <= sim.timestamp + constants.time_to_consider_order
            ):
                order_xs.append(order.xpos * 10)
                order_ys.append(order.ypos * 10)
                if hasattr(order, "initials"):
                    txt = ax.text(
                        order.xpos * 10,
                        order.ypos * 10 + 5,
                        order.initials,
                        color="black",
                        fontsize=10,
                        ha="center",
                        zorder=20,
                    )
                    order_initials_texts.append(txt)
        scat_orders.set_offsets(np.c_[order_xs, order_ys])

        # Collect drone locations
        drone_xs.clear()
        drone_ys.clear()
        for drone in sim.drones:
            drone_xs.append(drone.xpos * 10)
            drone_ys.append(drone.ypos * 10)
        scat_drones.set_offsets(np.c_[drone_xs, drone_ys])

        # Update each drone's path line
        for i, drone in enumerate(sim.drones):
            if (
                hasattr(drone, "path")
                and drone.path
                and len(drone.path) > 1
                and getattr(drone, "state", None) != "waiting"
                and getattr(drone, "movement_path", None)
            ):
                path_x, path_y = zip(*[(x * 10, y * 10) for x, y in drone.path])
                path_lines[i].set_data(path_x, path_y)
            else:
                path_lines[i].set_data([], [])

        title_text.set_text(f"Simulation Map Animation")
        time_text.set_text(f"Time: {sim.timestamp} s")
        return (
            scat_orders,
            scat_drones,
            scat_restaurants,
            scat_depots,
            *path_lines,
            time_text,
            *order_initials_texts,
        )

    ani = animation.FuncAnimation(
        fig, update, frames=steps, interval=interval, blit=True, repeat=False
    )
    # Uncomment the next line to save the animation as an MP4
    # ani.save("simulation_animation.mp4", writer='ffmpeg', fps=1000//interval)

    plt.show()


def delete_collection():
    coll_ref = db.collection("requests")
    docs = coll_ref.list_documents(page_size=1000)
    deleted = 0
    for doc in docs:
        doc.delete()
        deleted = deleted + 1


# animate_simulation(my_sim, n_steps, interval=10)
# for i in range(n_steps):
#   my_sim.take_step()
# print(my_sim.financial_model.calculate_revenue())

depot_dict = [
    {
        "depot_id": 0,
        "xpos": 264,
        "ypos": 472,
        "capacity": 10,
    }
]
if __name__ == "__main__":
    delete_collection()
    parser = argparse.ArgumentParser()
    col_query = db.collection("requests")
    # Watch the collection query
    query_watch = col_query.on_snapshot(on_snapshot)

    parser.add_argument("--weight", type=float, default=-0.1)
    parser.add_argument("--output", type=str, default="result_weight.txt")
    parser.add_argument("--seed", type=int, required=False, default=1)
    parser.add_argument("--n_drones", type=int, default=n_drones)
    parser.add_argument("--depot_dict", type=str, default=json.dumps(depot_dict))
    args = parser.parse_args()

    n_drones = args.n_drones
    np.random.seed(args.seed)
    depot_dict = json.loads(args.depot_dict)

    depots = [Depot(depot_dict[i]) for i in range(len(depot_dict))]
    n_depots = len(depots)
    # Create drone objects
    if n_depots > 1:
        drone_dict = [
            {"drone_id": i, "depot": 0 if i % 2 == 0 else 1} for i in range(n_drones)
        ]
    else:
        drone_dict = [{"drone_id": i, "depot": 0} for i in range(n_drones)]
    drone_list = [Drone(drone_dict[i]) for i in range(len(drone_dict))]

    # Create the city object
    city_dict = {
        "city_name": city_name,
        "restaurants": restaurants,
        "depots": depots,
        "tall_buildings": [],  # Placeholder, can be populated later
        "silent_zones": silent_zones,
        "population": population_dict,
    }

    my_sim = Simulation(city=City(city_dict), logger=Logger())
    n_steps = int(constants.time_window / my_sim.dt)
    my_sim.change_order_volume(0)
    my_sim.weight = args.weight
    # for i in range(n_steps):
    #    my_sim.take_step()
    # Plot the legal order grid in blue with alpha=0.5
    animate_simulation(my_sim, n_steps * 3000, interval=40)
    # plt.plot(np.linspace(0, constants.time_window, len(my_sim.orders_per_time)), my_sim.orders_per_time, label='Orders per time step')
    # plt.xlabel('Time step')
    # plt.ylabel('Number of orders')
    # plt.title(f'Orders per time step with weight {args.weight}')
    # plt.show()
    # orders = [my_sim.order_book[order_id] for order_id in my_sim.order_book]
    # arrival_times = [order.arrival_time for order in orders if order.status]
    # unique_times, counts = np.unique(arrival_times, return_counts=True)
    # plt.bar(unique_times, counts, width=constants.mp_interval, color='blue', alpha=0.7)
    # plt.xlabel('Arrival Time')
    # plt.ylabel('Number of Orders')
    # plt.title(f'Number of Orders per Arrival Time with weight {args.weight}')
    # plt.show()
    orders_delivered = len(
        [order for order in my_sim.order_book.values() if order.status]
    )
    roi, total_profit, total_costs, total_revenue, initial_costs = (
        my_sim.financial_model.calculate_ROI_single_day()
    )
    daily_profit = my_sim.financial_model.calculate_daily_profit()
    print(f"daily profit: {daily_profit}")
    print(f"total profit: {total_profit}")
    print(f"total costs: {total_costs}")
    print(orders_delivered)
    distnce_travelled = np.sum(
        [drone.distance_travelled for drone in my_sim.drones], axis=0
    )
    total_distance = np.sum(distnce_travelled)
    print(f"Total distance travelled by all drones: {total_distance} m")
    n_pizzas_delivered = sum(
        [
            order.s + order.m + order.l
            for order in my_sim.order_book.values()
            if order.status
        ]
    )
    n_pizzas_placed = sum(
        [order.s + order.m + order.l for order in my_sim.order_book.values()]
    )
    print(
        f"Total number of pizzas delivered: {n_pizzas_delivered} out of {n_pizzas_placed} placed orders"
    )
    with open(args.output, "w") as f:
        f.write(
            f"{args.weight},{args.n_drones},{roi},{total_profit},{total_costs},{total_revenue},{initial_costs},{orders_delivered}\n"
        )
