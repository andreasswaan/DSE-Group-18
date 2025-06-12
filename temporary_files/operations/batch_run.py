import subprocess
import numpy as np
import os
import matplotlib.pyplot as plt

output_dir = "operations"
os.makedirs(output_dir, exist_ok=True)
txt_path = os.path.join(output_dir, "variable_objective_weights.txt")
fig_path = os.path.join(output_dir, "profit_vs_weight.png")

weights = np.arange(0, -1.1, -0.1)
profits = []

with open(txt_path, "w") as f:
    for weight in weights:
        result_file = os.path.join(output_dir, f"result_weight_{weight:.2f}.txt")
        # Run simulation_final.py with the given weight
        subprocess.run([
                        "python", "temporary_files/operations/simulation_final.py",
                        "--weight", str(weight),
                        "--output", result_file
                    ], check=True)
        # Read the result
        with open(result_file, "r") as rf:
            line = rf.readline()
            w, profit, costs, revenue = map(float, line.strip().split(","))
            profits.append(profit)
            f.write(f"---------------------Weight: {w}----------------------\n")
            f.write(f"Profit: {profit:.2f} EUR, Costs: {costs:.2f} EUR, Revenue: {revenue:.2f} EUR\n")

# Plot profit vs weight
plt.figure()
plt.plot(weights, profits, marker='o')
plt.xlabel("Weight")
plt.ylabel("Profit (EUR)")
plt.title("Profit vs Weight")
plt.grid(True)
plt.savefig(fig_path)
plt.close()