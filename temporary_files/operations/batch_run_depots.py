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

n_drones = 20
weight = -0.3
depot_dicts = [
    [{
        'depot_id': 0,
        'xpos': 264,
        'ypos': 472,
        'capacity': 10,
    }],
    [{
        'depot_id': 0,
        'xpos': 314,
        'ypos': 234,
        'capacity': 10,
    }],
    [{
        'depot_id': 0,
        'xpos': 300,
        'ypos': 380,
        'capacity': 10,
    }],
    [{
        'depot_id': 0,
        'xpos': 307,
        'ypos': 658,
        'capacity': 10,
    }, {
        'depot_id': 1,
        'xpos': 314,
        'ypos': 234,
        'capacity': 10,
    }],
    [{
        'depot_id': 0,
        'xpos': 264,
        'ypos': 472,
        'capacity': 10,
    }, {
        'depot_id': 1,
        'xpos': 314,
        'ypos': 234,
        'capacity': 10,
    }]
]
names = ['centre depot', 'far depot', 'intermediate depot', 'two depots far', 'depot far and centre']
seeds = [0, 1]  # You can change this list as needed

all_profits = []

for seed in seeds:
    txt_path = os.path.join(output_dir, f"variable_depot_seed{seed}_objective_weight{weight}_n_drones_{n_drones}.txt")
    fig_path = os.path.join(output_dir, f"profit_vs_depot_seed{seed}_objective_weight{weight}_n_drones_{n_drones}.png")
    profits = []

    with open(txt_path, "w") as f:
        for i,depot_dict in enumerate(depot_dicts):
            f.write(f"Depot,Objective weight,N_Drones, ROI, Profit, Costs, Revenue, Initial Costs\n")
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
                w,n, roi, total_profit, total_costs, total_revenue, initial_costs, orders_delivered = map(float, line.strip().split(","))
                profits.append(total_profit)
                f.write(f"{names[i]},{w},{n}, {roi:.2f}, {total_profit:.2f}, {total_costs:.2f}, {total_revenue:.2f}, {initial_costs:.2f},{orders_delivered}\n")
    all_profits.append(profits)

    # Plot profit vs weight for this seed
    plt.figure()
    plt.plot(names, profits, marker='o')
    plt.xlabel("Depot")
    plt.ylabel("Profit (EUR)")
    plt.title(f"Profit vs Depot Seed {seed}, Objective weight  {weight}, n_drones {n_drones}")
    plt.grid(True)
    plt.savefig(fig_path)
    plt.close()

# Compute average profit at each weight over all seeds
all_profits = np.array(all_profits)
avg_profits = np.mean(all_profits, axis=0)

# Plot average profit vs weight
avg_fig_path = os.path.join(output_dir, f"profit_vs_Depot_Objective_weight_{weight}_n_drones_{n_drones}.png")
plt.figure()
plt.plot(names, avg_profits, marker='o')
plt.xlabel("Depot")
plt.ylabel("Average Profit (EUR)")
plt.title(f"Average Profit vs Weight, Objective weight {weight}, n_drones {n_drones}")
plt.grid(True)
plt.savefig(avg_fig_path)
plt.close()