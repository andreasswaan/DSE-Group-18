import numpy as np
#time related constants
velocity_hori=15
velocity_climb=6
velocity_descent=3
cruise_height=50
waiting_time_restaurant=90
waiting_time_customer=90
loading_time=60
unloading_time_customer=60
buffer_time=60
battery_swap_time=300
peak_order_volume=350
time_window=3600*4 # time window to place orders
min_order_delay=0*60
scale=30*60 # scale for order time distribution
pizza_cooling_time=30*60
time_to_consider_order=60*15
max_waiting_time=60*2 # maximum waiting time at restaurant or customer location
max_extra_travel_time=60*2
TO_land_energy=8
energy_per_metre=0.005
initial_orders_time = 30*60
max_hover_time=60*3
max_orders_per_mission=7
mp_interval=60*2
deliver_time_window=60*10

# financial constants
#initial costs
# per drone cost
cost_per_drone_production=10000
cost_per_spare_parts=1500
cost_per_drone_depot=2724
cost_per_drone_equipment=140

#fixed costs
license_cost_initial=10000
equipment_cost_initial=3000
depot_construction_cost_=34050
#second depot cost
second_depot_construction_cost=20430
second_depot_equipment_cost=1000

#per month costs
#fixed costs
electricity_cost_fixed=113.45
rent_cost = 2738.06
rent_increase_per_meter=-0.85
staff_cost = 11700
licensing_cost = 68
marketing_cost = 8334
# second depot costs
cost_for_second_depot=2131.46
electricity_cost_per_month_second_depot=20.02
staff_cost_second_depot=3300

# per drone cost
drone_insurance_per_month=67
drone_facililities_per_month=19.92
drone_staff_per_month=255

# per delivery cost
# drone_maintenance_per_delivery=0.65
order_reimbursement_per_delivery=0.25
electricity_cost_per_kWh=0.1 # constant for now, we can make it time dependent later
maintenance_cost_per_kWh=0.1 # provisional

# revenue constants
s_price = 10
m_price = 13
l_price = 16
order_fee=0.25  # reimbursement per delivery
order_volume_ratios = [0.786, 0.857, 0.929, 1, 1.494, 1.67, 1.494]

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
        self.waiting_time = waiting_time_customer  # seconds