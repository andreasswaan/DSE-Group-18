import numpy as np
import matplotlib.pyplot as plt

base_path = "operations/variable_n_drones_seed{}_objective_weight-0.1_centre depot(1).txt"
num_seeds = 5

all_weights = []
all_profits = []

for seed in range(num_seeds):
    weights = []
    profits = []
    with open(base_path.format(seed), "r") as f:
        lines = f.readlines()[1:]  # skip header
        for line in lines:
            if not line.strip() or line.startswith("//"):
                continue
            parts = line.strip().split(",")
            if len(parts) < 5:
                continue
            weight = float(parts[1])
            profit = float(parts[3])/1.5
            weights.append(weight)
            profits.append(profit)
    all_weights.append(weights)
    all_profits.append(profits)

# Convert to numpy arrays for easy averaging
all_weights = np.array(all_weights)
all_profits = np.array(all_profits)

# Assume all weights are the same across files
weights = all_weights[0]
avg_profits = np.mean(all_profits, axis=0)

plt.rcParams.update({
    'font.size': 14,          # General text size
    'axes.titlesize': 16,     # Title font size
    'axes.labelsize': 14,     # Axis label font size
    'xtick.labelsize': 12,    # X tick label font size
    'ytick.labelsize': 12,    # Y tick label font size
    'legend.fontsize': 12,    # Legend text
})

fig, axs = plt.subplots(1, 2, figsize=(14, 6))  # 1 row, 2 columns

axs[0].plot(weights, avg_profits, marker='o')
axs[0].set_xlabel("Number of Drones")
axs[0].set_ylabel("profit (EUR)")
axs[0].grid(True)

all_weights = []
all_profits = []

for seed in range(num_seeds):
    weights = []
    profits = []
    with open(base_path.format(seed), "r") as f:
        lines = f.readlines()[1:]  # skip header
        for line in lines:
            if not line.strip() or line.startswith("//"):
                continue
            parts = line.strip().split(",")
            if len(parts) < 5:
                continue
            weight = float(parts[1])
            profit = float(parts[2])/1.5
            weights.append(weight)
            profits.append(profit)
    all_weights.append(weights)
    all_profits.append(profits)

# Convert to numpy arrays for easy averaging
all_weights = np.array(all_weights)
all_profits = np.array(all_profits)

# Assume all weights are the same across files
weights = all_weights[0]
avg_profits = np.mean(all_profits, axis=0)

axs[1].plot(weights, avg_profits, marker='o')
axs[1].set_xlabel("Number of Drones")
axs[1].set_ylabel("ROI (%)")
axs[1].grid(True)

plt.tight_layout()
plt.savefig("profit_roi_combined.svg")
plt.show()