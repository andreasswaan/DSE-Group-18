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

weights = np.arange(0.1, -1.2, -0.1)
n_drones = 20
depot_dict = [{
    'depot_id': 0,
    'xpos': 264,
    'ypos': 472,
    'capacity': 10,
    }]
seeds = [0, 1, 2, 3, 4]  # You can change this list as needed
depot_name = "centre depot"
all_profits = []

for seed in seeds:
    txt_path = os.path.join(output_dir, f"variable_objective_weights_seed{seed}_n_drones_{constants.n_drones}_{depot_name}.txt")
    fig_path = os.path.join(output_dir, f"profit_vs_weight_seed{seed}_n_drones_{constants.n_drones}_{depot_name}.png")
    profits = []

    with open(txt_path, "w") as f:
        f.write(f"Depot,Objective weight,N_Drones, ROI, Profit, Costs, Revenue, Initial Costs\n")
        for weight in weights:
            result_file = os.path.join(raw_results_dir, f"result_weight_{weight:.2f}_seed{seed}.txt")
            # Run simulation_final.py with the given weight and seed
            subprocess.run([
                "python", "temporary_files/operations/simulation_final.py",
                "--weight", str(weight),
                "--n_drones", str(n_drones),
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

    # Plot profit vs weight for this seed
    plt.figure()
    plt.plot(weights, profits, marker='o')
    plt.xlabel("Weight")
    plt.ylabel("Profit (EUR)")
    plt.title(f"Profit vs Weight Seed {seed}, Drones {constants.n_drones}, Depot {depot_name}")
    plt.grid(True)
    plt.savefig(fig_path)
    plt.close()

# Compute average profit at each weight over all seeds
all_profits = np.array(all_profits)
avg_profits = np.mean(all_profits, axis=0)

# Plot average profit vs weight
avg_fig_path = os.path.join(output_dir, f"profit_vs_weight_average_n_drones_{constants.n_drones}_{depot_name}.png")
plt.figure()
plt.plot(weights, avg_profits, marker='o')
plt.xlabel("Weight")
plt.ylabel("Average Profit (EUR)")
plt.title(f"Average Profit vs Weight, Drones {constants.n_drones}, Depot {depot_name}")
plt.grid(True)
plt.savefig(avg_fig_path)
plt.close()