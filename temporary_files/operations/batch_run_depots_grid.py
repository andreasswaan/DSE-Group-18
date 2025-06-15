import subprocess
import numpy as np
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Needed for 3D plotting
import constants
import json

output_dir = "operations"
raw_results_dir = os.path.join(output_dir, "results_raw")
os.makedirs(output_dir, exist_ok=True)
os.makedirs(raw_results_dir, exist_ok=True)

n_drones = 20
weight = -0.3
capacity = 10

# Define grid parameters
x_min, x_max = 200, 400
y_min, y_max = 300, 500
n_x, n_y = 5, 5  # Number of grid points in x and y

x_grid = np.linspace(x_min, x_max, n_x, dtype=int)
y_grid = np.linspace(y_min, y_max, n_y, dtype=int)

# Create depot configurations for each grid point
depot_dicts = []
depot_positions = []
for x in x_grid:
    for y in y_grid:
        depot_dicts.append([{
            'depot_id': 0,
            'xpos': int(x),
            'ypos': int(y),
            'capacity': capacity,
        }])
        depot_positions.append((int(x), int(y)))

names = [f"depot_{x}_{y}" for x, y in depot_positions]
seeds = [99]  # You can change this list as needed

all_profits = []
all_results = []  # To store all data for further plotting

for seed in seeds:
    txt_path = os.path.join(output_dir, f"variable_depot_grid_seed{seed}_objective_weight{weight}_n_drones_{n_drones}.txt")
    fig_path = os.path.join(output_dir, f"profit_vs_depot_grid_seed{seed}_objective_weight{weight}_n_drones_{n_drones}.png")
    profits = []
    results_this_seed = []

    with open(txt_path, "w") as f:
        f.write(f"Depot,X,Y,Objective weight,N_Drones, ROI, Profit, Costs, Revenue, Initial Costs, Orders Delivered\n")
        for i, depot_dict in enumerate(depot_dicts):
            result_file = os.path.join(raw_results_dir, f"result_depot_{names[i]}_seed{seed}.txt")
            # Run simulation_final.py with the given weight and seed
            subprocess.run([
                "python", "temporary_files/operations/simulation_final.py",
                "--n_drones", str(n_drones),
                "--weight", str(weight),
                "--depot_dict", json.dumps(depot_dict),
                "--seed", str(seed),
                "--output", result_file
            ], check=True)
            # Read the result
            with open(result_file, "r") as rf:
                line = rf.readline()
                if not line.strip():
                    continue
                w, n, roi, total_profit, total_costs, total_revenue, initial_costs, orders_delivered = map(float, line.strip().split(","))
                profits.append(total_profit)
                x, y = depot_positions[i]
                results_this_seed.append({
                    "name": names[i],
                    "x": x,
                    "y": y,
                    "weight": w,
                    "n_drones": n,
                    "roi": roi,
                    "profit": total_profit,
                    "costs": total_costs,
                    "revenue": total_revenue,
                    "initial_costs": initial_costs,
                    "orders_delivered": orders_delivered
                })
                f.write(f"{names[i]},{x},{y},{w},{n},{roi:.2f},{total_profit:.2f},{total_costs:.2f},{total_revenue:.2f},{initial_costs:.2f},{orders_delivered}\n")
    all_profits.append(profits)
    all_results.append(results_this_seed)

    # 3D plot: x, y, profit
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    xs = [r["x"] for r in results_this_seed]
    ys = [r["y"] for r in results_this_seed]
    zs_profit = [r["profit"] for r in results_this_seed]
    sc = ax.scatter(xs, ys, zs_profit, c=zs_profit, cmap='viridis', marker='o')
    ax.set_xlabel("Depot X")
    ax.set_ylabel("Depot Y")
    ax.set_zlabel("Profit (EUR)")
    ax.set_title(f"Profit vs Depot Position (Seed {seed}, Weight {weight}, n_drones {n_drones})")
    fig.colorbar(sc, ax=ax, label="Profit (EUR)")
    plt.savefig(fig_path.replace("profit", "profit"))
    plt.close()

    # 3D plot: x, y, ROI
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111, projection='3d')
    zs_roi = [r["roi"] for r in results_this_seed]
    sc2 = ax2.scatter(xs, ys, zs_roi, c=zs_roi, cmap='plasma', marker='o')
    ax2.set_xlabel("Depot X")
    ax2.set_ylabel("Depot Y")
    ax2.set_zlabel("ROI")
    ax2.set_title(f"ROI vs Depot Position (Seed {seed}, Weight {weight}, n_drones {n_drones})")
    fig2.colorbar(sc2, ax=ax2, label="ROI")
    plt.savefig(fig_path.replace("profit", "roi"))
    plt.close()

# Save all results for further analysis
results_json_path = os.path.join(output_dir, f"all_depot_grid_results_weight_{weight}_n_drones_{n_drones}.json")
with open(results_json_path, "w") as jf:
    json.dump(all_results, jf, indent=2)