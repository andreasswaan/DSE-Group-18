import numpy as np
import matplotlib.pyplot as plt

base_path = "operations/variable_objective_weights_seed{}_n_drones_10_centre depot.txt"
num_seeds = 5

all_weights = []
all_profits = []

for seed in range(1, num_seeds):
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
            profit = float(parts[6])  - float(parts[5])  # profit = revenue - costs
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
    'axes.labelsize': 18,     # Axis label font size
    'xtick.labelsize': 12,    # X tick label font size
    'ytick.labelsize': 16,    # Y tick label font size
    'legend.fontsize': 12,    # Legend text
})
plt.figure(figsize=(8, 5))
plt.plot(weights, avg_profits, marker='o')
plt.xlabel("w2")
plt.gca().invert_xaxis()  # Invert x-axis to match the original plot
plt.ylabel("Profit (EUR)")
plt.grid(True)
plt.tight_layout()
plt.savefig("operations/profit_vs_weight.pdf")
plt.show()