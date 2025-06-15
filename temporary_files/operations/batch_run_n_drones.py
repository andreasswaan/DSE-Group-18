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

n_droness = range(6, 14, 2)
weight = -0.3
depot_dict = [{
    'depot_id': 0,
    'xpos': 250,
    'ypos': 450,
    'capacity': 10,
    }]
seeds = [0]  # You can change this list as needed
depot_name = "centre depot"
all_profits = []

for seed in seeds:
    txt_path = os.path.join(output_dir, f"variable_n_drones_seed{seed}_objective_weight{weight}_{depot_name}.txt")
    fig_path = os.path.join(output_dir, f"profit_vs_n_drones_seed{seed}_objective_weight{weight}_{depot_name}.png")
    profits = []

    with open(txt_path, "w") as f:
        f.write(f"Depot,Objective weight,N_Drones, ROI, Profit, Costs, Revenue, Initial Costs\n")
        for n_drones in n_droness:
            result_file = os.path.join(raw_results_dir, f"result_n_drones_{n_drones:.2f}_seed{seed}.txt")
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
                f.write(f"{depot_name}{w},{n}, {roi:.2f}, {total_profit:.2f}, {total_costs:.2f}, {total_revenue:.2f}, {initial_costs:.2f},{orders_delivered}\n")
    all_profits.append(profits)

    # Plot profit vs weight for this seed
    plt.figure()
    plt.plot(n_droness, profits, marker='o')
    plt.xlabel("N_Drones")
    plt.ylabel("Profit (EUR)")
    plt.title(f"Profit vs N_drones Seed {seed}, Objective weight  {weight}, Depot {depot_name}")
    plt.grid(True)
    plt.savefig(fig_path)
    plt.close()

# Compute average profit at each weight over all seeds
all_profits = np.array(all_profits)
avg_profits = np.mean(all_profits, axis=0)

# Plot average profit vs weight
avg_fig_path = os.path.join(output_dir, f"profit_vs_N_drones_average_Objective_weight{weight}_{depot_name}.png")
plt.figure()
plt.plot(n_droness, avg_profits, marker='o')
plt.xlabel("N_Drones")
plt.ylabel("Average Profit (EUR)")
plt.title(f"Average Profit vs Weight (All Seeds), Objective weight {weight}, Depot {depot_name}")
plt.grid(True)
plt.savefig(avg_fig_path)
plt.close()