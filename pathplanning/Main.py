import numpy as np
import matplotlib.pyplot as plt
from Algorithms.Astar_random_plot import a_star_search_8dir, smooth_path
import matplotlib.cm as cm
import matplotlib.colors as mcolors
import matplotlib.cm as cm
from matplotlib.patches import Patch

def load_delft_grid(path="pathplanning/data/delft_grid_data.npz"):
    data = np.load(path, allow_pickle=True)
    return {
        'weight_grid': data["weight_grid"],
        'obstacle_grid': data["obstacle_grid"],
        'restaurant_grid': data["restaurant_grid"],
        'x_coords': data["x_coords"],
        'y_coords': data["y_coords"],
        'bounds': data["bounds"],
        'resolution': data["resolution"].item()
    }

# ---------- 1. Load the Delft grid ----------
grid = load_delft_grid()
walkable = ~grid['obstacle_grid']  # True where traversal is allowed

# ---------- 2. Select a start point (from restaurant locations) ----------
restaurant_coords = np.argwhere(grid['restaurant_grid'])
if len(restaurant_coords) == 0:
    raise ValueError("❌ No restaurant locations found in the grid.")
start = tuple(restaurant_coords[0][::-1])  # flip to (x, y)

# ---------- 3. Select an end point (somewhere near city center) ----------
height, width = grid['weight_grid'].shape
center_x, center_y = width // 2, height // 2

# Try up to 500 times to find a walkable coordinate near the center
np.random.seed(3)
for _ in range(500):
    dx, dy = np.random.randint(-20, 20), np.random.randint(-20, 20)
    x, y = center_x + dx, center_y + dy
    if 0 <= x < width and 0 <= y < height and walkable[y, x]:
        end = (x, y)
        break
else:
    raise ValueError("❌ Could not find a valid endpoint near the center.")

print(f"Start point (restaurant): {start}")
print(f"End point (near center): {end}")

# ---------- 4. Run A* algorithm ----------
path = a_star_search_8dir(start, end, walkable, density_map=grid['weight_grid'])
if not path:
    raise RuntimeError("❌ A* could not find a path.")
smoothed_path = smooth_path(path, walkable)

# ---------- 5. Visualize the result ----------
# Normalize weights for background visualization
weight_grid = grid['weight_grid'].copy()
max_weight = np.nanmax(weight_grid[weight_grid < np.inf])
normalized = np.clip(weight_grid, 0, max_weight) / max_weight

# Apply color map (same as in DelftGrid.py)
cmap = cm.get_cmap('YlOrRd')
rgba = cmap(normalized)  # shape: (H, W, 4)

# Convert to RGB image
img = rgba[:, :, :3].copy()
img[~walkable] = [0, 0, 0]  # mask obstacles in black

for x, y in path:
    img[y, x] = [1, 0, 1]  # magenta for raw A* path
for x, y in smoothed_path:
    img[y, x] = [0, 1, 1]  # cyan for smoothed path

img[start[1], start[0]] = [0, 0, 1]  # blue for start
img[end[1], end[0]] = [0, 1, 0]      # green for end



fig, ax = plt.subplots(figsize=(8, 8))
ax.imshow(img, origin='lower')  

cbar = plt.colorbar(cm.ScalarMappable(norm=mcolors.Normalize(vmin=0.0, vmax=max_weight), cmap=cmap), ax=ax)
cbar.set_label("Navigation Cost (normalized)", fontsize=10)

legend_elements = [
    Patch(facecolor='black', edgecolor='black', label='No-fly zones / obstacles'),
    Patch(facecolor='magenta', edgecolor='black', label='A* path'),
    Patch(facecolor='cyan', edgecolor='black', label='Smoothed path'),
    Patch(facecolor='blue', edgecolor='black', label='Start (restaurant)'),
    Patch(facecolor='green', edgecolor='black', label='End point')
]

ax.legend(handles=legend_elements, loc='upper right', frameon=True)

ax.set_title("A* Path on Weighted Delft Grid")
ax.axis("off")
plt.tight_layout()
plt.show()

