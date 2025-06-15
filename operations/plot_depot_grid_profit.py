import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import griddata

# Load the data
with open("operations/all_depot_grid_results_weight_-0.3_n_drones_20.json", "r") as f:
    all_results = json.load(f)

# Flatten the list if needed (your data is [ [ {...}, {...}, ... ] ])
results = all_results[0]

# Extract x, y, profit
xs = np.array([r["x"] for r in results])
ys = np.array([r["y"] for r in results])
zs = np.array([r["profit"] for r in results])

# Create grid for surface
xi = np.linspace(xs.min(), xs.max(), 50)
yi = np.linspace(ys.min(), ys.max(), 50)
xi, yi = np.meshgrid(xi, yi)
zi = griddata((xs, ys), zs, (xi, yi), method='cubic')

# Plot
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# Plot the surface (plane between points)
surf = ax.plot_surface(xi, yi, zi, cmap='viridis', alpha=0.7, edgecolor='none')
# Plot the original points
ax.scatter(xs, ys, zs, color='red', s=40, label='Depot Points')

ax.set_xlabel('Depot X')
ax.set_ylabel('Depot Y')
ax.set_zlabel('Profit (EUR)')
ax.set_title('Profit Surface for Depot Grid')
fig.colorbar(surf, ax=ax, shrink=0.5, aspect=10, label='Profit (EUR)')
ax.legend()
plt.tight_layout()
plt.show()