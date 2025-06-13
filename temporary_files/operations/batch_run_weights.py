import subprocess
import numpy as np
import os
import matplotlib.pyplot as plt
import constants

output_dir = "operations"
raw_results_dir = os.path.join(output_dir, "results_raw")
os.makedirs(output_dir, exist_ok=True)
os.makedirs(raw_results_dir, exist_ok=True)

weights = np.arange(0.1, -1.2, -0.1)
seeds = [0, 1, 2, 3, 4]  # You can change this list as needed

all_profits = []

for seed in seeds:
    txt_path = os.path.join(output_dir, f"variable_objective_weights_seed{seed}_time_{constants.time_window // 3600}_n_drones_{constants.n_drones}.txt")
    fig_path = os.path.join(output_dir, f"profit_vs_weight_seed{seed}_time_{constants.time_window // 3600}_n_drones_{constants.n_drones}.png")
    profits = []

    with open(txt_path, "w") as f:
        for weight in weights:
            result_file = os.path.join(raw_results_dir, f"result_weight_{weight:.2f}_seed{seed}.txt")
            # Run simulation_final.py with the given weight and seed
            subprocess.run([
                "python", "temporary_files/operations/simulation_final.py",
                "--weight", str(weight),
                "--seed", str(seed),
                "--output", result_file
            ], check=True)
            # Read the result
            with open(result_file, "r") as rf:
                line = rf.readline()
                if not line.strip():
                    continue
                w,n, profit, costs, revenue = map(float, line.strip().split(","))
                profits.append(profit)
                f.write(f"---------------------Weight: {w}----------------------\n")
                f.write(f"Profit: {profit:.2f} EUR, Costs: {costs:.2f} EUR, Revenue: {revenue:.2f} EUR\n")
    all_profits.append(profits)

    # Plot profit vs weight for this seed
    plt.figure()
    plt.plot(weights, profits, marker='o')
    plt.xlabel("Weight")
    plt.ylabel("Profit (EUR)")
    plt.title(f"Profit vs Weight Seed {seed}, Time {constants.time_window // 3600}h, Drones {constants.n_drones}")
    plt.grid(True)
    plt.savefig(fig_path)
    plt.close()

# Compute average profit at each weight over all seeds
all_profits = np.array(all_profits)
avg_profits = np.mean(all_profits, axis=0)

# Plot average profit vs weight
avg_fig_path = os.path.join(output_dir, f"profit_vs_weight_average_time_{constants.time_window // 3600}_n_drones_{constants.n_drones}.png")
plt.figure()
plt.plot(weights, avg_profits, marker='o')
plt.xlabel("Weight")
plt.ylabel("Average Profit (EUR)")
plt.title(f"Average Profit vs Weight (All Seeds)_ Time {constants.time_window // 3600}h, Drones {constants.n_drones}")
plt.grid(True)
plt.savefig(avg_fig_path)
plt.close()