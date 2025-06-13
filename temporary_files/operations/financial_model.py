import constants
import numpy as np

class FinancialModel:
    def __init__(self, simulation):
        self.simulation = simulation
        self.city = simulation.city
        self.drones = simulation.drones
        self.depots = simulation.city.depots
        self.orders = simulation.order_book
        self.profit = 0
        self.costs = 0
        self.revenue = 0

    def calculate_initial_costs(self):
        # Calculate initial costs based on the number of drones and depots
        num_drones = len(self.drones)
        num_depots = len(self.depots)

        initial_costs = (
            num_drones * (constants.cost_per_drone_production + constants.cost_per_spare_parts +
                           constants.cost_per_drone_depot + constants.cost_per_drone_equipment) +
            constants.depot_construction_cost_ +
            constants.license_cost_initial +
            constants.equipment_cost_initial
        )
        if num_depots > 1:
            initial_costs += (
                constants.second_depot_construction_cost +
                constants.second_depot_equipment_cost
            ) * (num_depots - 1)
        
        return initial_costs
    
    def calculate_costs_per_month(self):
        # Calculate monthly costs
        num_drones = len(self.drones)
        num_depots = len(self.depots)

        monthly_costs = (
            constants.electricity_cost_fixed +
            constants.rent_cost +
            constants.staff_cost +
            constants.licensing_cost +
            constants.marketing_cost +
            num_drones * (constants.drone_insurance_per_month + 
                          constants.drone_facililities_per_month + 
                          constants.drone_staff_per_month)
        )
        if num_depots > 1:
            monthly_costs += (
                constants.cost_for_second_depot + 
                constants.electricity_cost_per_month_second_depot + 
                constants.staff_cost_second_depot
            ) * (num_depots - 1)
        for depot in self.depots:
            monthly_costs += constants.rent_increase_per_meter * np.sqrt((self.city.reso/2-depot.xpos)**2 + (self.city.reso/2-depot.ypos)**2)*6000/ self.city.reso
        return monthly_costs
    
    def calculate_costs_per_delivery(self):
        return constants.order_reimbursement_per_delivery
    
    def calculate_costs_per_kWh(self):
        # Calculate costs per kWh
        return constants.electricity_cost_per_kWh + constants.maintenance_cost_per_kWh
    
    def calculate_revenue(self):
        # Calculate revenue based on the number of orders
        num_s_pizzas = 0
        num_m_pizzas = 0
        num_l_pizzas = 0
        orders = [self.simulation.order_book[order_id] for order_id in self.simulation.order_book if self.simulation.order_book[order_id].status]
        for order in orders:
                num_s_pizzas += order.s
                num_m_pizzas += order.m
                num_l_pizzas += order.l
        order_value = (num_s_pizzas * constants.s_price +
                       num_m_pizzas * constants.m_price +
                       num_l_pizzas * constants.l_price)
        revenue = order_value * constants.order_fee
        return revenue
    
    def calculate_energy_costs(self):
        # Calculate energy costs based on the total energy consumed
        total_energy_consumed = sum(depot.energy_spent for depot in self.depots)
        energy_costs = total_energy_consumed * self.calculate_costs_per_kWh()
        return energy_costs
    
    def calculate_daily_profit(self):
        # Calculate daily profit
        daily_revenue = self.calculate_revenue()
        orders = [self.simulation.order_book[order_id] for order_id in self.simulation.order_book if self.simulation.order_book[order_id].status]
        daily_costs = (
            self.calculate_costs_per_delivery() * len(orders) +  # Costs per delivery
            self.calculate_energy_costs()  # Energy costs
        )
        daily_profit = daily_revenue - daily_costs   
        return daily_profit, daily_costs, daily_revenue
    
    def calculate_weekly_profit(self):
        # Calculate weekly profit
        weekly_revenue = 0
        weekly_costs = 0
        for day in range(7):  # Assuming weekly profit calculation
            self.simulation.change_order_volume(constants.order_volume_ratios[day])
            for t in range(constants.time_window // self.simulation.dt):
                self.simulation.take_step()
            daily_revenue = self.calculate_revenue()
            daily_costs = (
                self.calculate_costs_per_delivery() * len([order for order in self.orders if order.status]) +  # Costs per delivery
                self.calculate_energy_costs()  # Energy costs
            )
            weekly_revenue += daily_revenue
            weekly_costs += daily_costs
            self.simulation.reset()
        weekly_profit = weekly_revenue - weekly_costs
        
        return weekly_profit
    
    def calculate_monthly_profit(self):
        monthly_profit = self.calculate_weekly_profit() * 365 / 12 / 7
        monthly_profit -= self.calculate_costs_per_month()  # Subtract monthly costs
        return monthly_profit
    
    def calculate_ROI(self, time_period=5): #in years
        # Calculate Return on Investment (ROI)
        initial_costs = self.calculate_initial_costs()
        monthly_profit = self.calculate_monthly_profit()
        roi = (monthly_profit * 12 * time_period / initial_costs) * 100
        profit = monthly_profit * 12 * time_period - initial_costs
        return roi, profit
    
    def calculate_ROI_single_day(self,time_period=5):
        # Calculate Return on Investment (ROI) for a single day
        initial_costs = self.calculate_initial_costs()
        daily_profit, daily_cost, daily_revenue = self.calculate_daily_profit()
        monthly_costs = self.calculate_costs_per_month()
        total_costs = initial_costs + monthly_costs * time_period * 12 + daily_cost * time_period * 365
        total_revenue = daily_revenue * 365 * time_period
        total_profit = total_revenue - total_costs
        roi = (total_profit / initial_costs) * 100
        return roi, total_profit, total_costs, total_revenue, initial_costs