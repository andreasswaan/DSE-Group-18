import subprocess
import numpy as np
import os
import matplotlib.pyplot as plt
import constants
import json

output_dir = "operations"
raw_results_dir = os.path.join(output_dir, "results_raw")
os.makedirs(output_dir, exist_ok=True)
os.makedirs(raw_results_dir, exist_ok=True)

n_droness = [10]
weight = -0.1
depot_dict = [{
    'depot_id': 0,
    'xpos': 220,
    'ypos': 340,
    'capacity': 10,
    }]
seeds = range(100,110)  # You can change this list as needed
depot_name = "centre depot"
all_profits = []
all_costs = []
all_revenues = []
all_initial_costs = []
all_orders_delivered = []
all_rois = []

for seed in seeds:
    txt_path = os.path.join(output_dir, f"variable_n_drones_seed{seed}_objective_weight{weight}_{depot_name}(6).txt")
    fig_path = os.path.join(output_dir, f"profit_vs_n_drones_seed{seed}_objective_weight{weight}_{depot_name}(6).pdf")
    profits = []

    with open(txt_path, "w") as f:
        f.write(f"Depot,Objective weight,N_Drones, ROI, Profit, Costs, Revenue, Initial Costs, Orders delivered\n")
        for n_drones in n_droness:
            result_file = os.path.join(raw_results_dir, f"result_n_drones_{n_drones:.2f}_seed{seed}(1).txt")
            # Run simulation_final.py with the given weight and seed
            subprocess.run([
                "python", "temporary_files/operations/simulation_final.py",
                "--n_drones", str(n_drones),
                "--weight", str(weight),
                "--seed", str(seed),
                "--depot_dict", json.dumps(depot_dict),
                "--output", result_file
            ], check=True)
            # Read the result
            with open(result_file, "r") as rf:
                line = rf.readline()
                if not line.strip():
                    continue
                w,n, roi, total_profit, total_costs, total_revenue, initial_costs, orders_delivered = map(float, line.strip().split(","))
                profits.append(total_profit)
                f.write(f"{depot_name},{w},{n}, {roi:.2f}, {total_profit:.2f}, {total_costs:.2f}, {total_revenue:.2f}, {initial_costs:.2f},{orders_delivered}\n")
    all_profits.append(profits)
    all_costs.append(total_costs)
    all_revenues.append(total_revenue)
    all_initial_costs.append(initial_costs)
    all_orders_delivered.append(orders_delivered)
    all_rois.append(roi)

mean_profits = np.mean(all_profits, axis=0)
mean_costs = np.mean(all_costs)
mean_revenues = np.mean(all_revenues)
mean_initial_costs = np.mean(all_initial_costs)
mean_orders_delivered = np.mean(all_orders_delivered)
mean_rois = np.mean(all_rois)

# make text file with the mean values
with open(os.path.join(output_dir, "mean_costs.txt"), "w") as f:
    f.write(f"Mean Costs: {mean_costs:.2f}\n")
    f.write(f"Mean Revenues: {mean_revenues:.2f}\n")
    f.write(f"Mean Initial Costs: {mean_initial_costs:.2f}\n")
    f.write(f"Mean Orders Delivered: {mean_orders_delivered:.2f}\n")
    f.write(f"Mean ROI: {mean_rois:.2f}\n")
    f.write(f"Mean Profits: {mean_profits}\n")

years = np.linspace(0, 5, 100)
# Linear interpolation for profit, cost, revenue
profit_line = (mean_profits / 5) * years - mean_initial_costs * (1 - years / 5)
cost_line = (mean_costs / 5) * years
revenue_line = (mean_revenues / 5) * years

# Profit starts at -initial_cost, ends at mean_profits
profit_line = -mean_initial_costs + (mean_profits + mean_initial_costs) * (years / 5)

# ROI line: (revenue - cost - initial_cost) / initial_cost
roi_line = (revenue_line - cost_line - mean_initial_costs) / mean_initial_costs

# Find break-even point (where ROI crosses 0)
break_even_idx = np.where(np.diff(np.sign(roi_line)))[0]
if break_even_idx.size > 0:
    be_idx = break_even_idx[0]
    be_x = years[be_idx]
    be_y = profit_line[be_idx]
else:
    be_x = be_y = None

plt.figure(figsize=(8, 6))
plt.plot(years, profit_line, label="Profit", color='green')
plt.plot(years, cost_line, label="Total Cost", color='red')
plt.plot(years, revenue_line, label="Total Revenue", color='blue')
plt.plot(years, roi_line * mean_initial_costs, label="ROI × Initial Cost", color='orange', linestyle='--')

if be_x is not None:
    plt.scatter([be_x], [be_y], color='black', zorder=10, label="Break-even (ROI=0)")
    plt.axvline(be_x, color='gray', linestyle=':', alpha=0.7)

plt.xlabel("Time (years)")
plt.ylabel("EUR")
plt.title("Profit, Cost, Revenue, and ROI over 5 Years (Mean Values)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(output_dir, "profit_cost_revenue_roi_over_time.pdf"))
plt.show()

