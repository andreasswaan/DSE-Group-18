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

start = tuple([25,480])
# end = tuple([180,585])

print(f"Start point (restaurant): {start}")
print(f"End point (near center): {end}")

# ---------- 4. Run A* algorithm ----------
path = a_star_search_8dir(start, end, walkable, density_map=grid['weight_grid'])
if not path:
    raise RuntimeError("❌ A* could not find a path.")
# smoothed_path = smooth_path(path, walkable)

paths_n = []
alpha_step = 0.1
alphas = np.arange(0.0, 1, alpha_step)
num_runs = 2
for i in range(num_runs):
    np.random.seed(1)
    print(f"------ Run {i+1}/{num_runs} ------")
    start = tuple(restaurant_coords[np.random.randint(len(restaurant_coords))][::-1])
    end = tuple([np.random.randint(0, width), np.random.randint(0, height)])
    
    paths = []
    for i in alphas:
        print(i)
        alpha = i
        paths.append(a_star_search_8dir(start, end, walkable, density_map=grid['weight_grid'], alpha=alpha))
        
    paths_n.append(paths)

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

# Apply a gradient to the line color depending on alpha used for the path in the loop
from matplotlib.colors import to_rgb

# Use purple-to-lime gradient for maximum contrast with each other and the background
start_color = np.array(to_rgb("purple"))    # low alpha
end_color = np.array(to_rgb("lime"))        # high alpha

for paths in paths_n:
    for path_i, alpha in zip(paths, alphas):
        color = (1 - alpha) * start_color + alpha * end_color  # linear interpolation
        for x, y in path_i[0]:
            img[y, x] = color
        print(f"Path with alpha {np.round(alpha,2)} has distance {np.round(path_i[1],3)} and weight {np.round(path_i[2],3)}")

# for x, y in path:
#     img[y, x] = [1, 0, 1]  # blue for raw A* path
# for x, y in smoothed_path:
#     img[y, x] = [0, 1, 1]  # cyan for smoothed path

img[start[1], start[0]] = [0, 0, 0.5]  # dark blue for start
img[end[1], end[0]] = [0, 1, 0]        # green for end

fig, ax = plt.subplots(figsize=(8, 8))
ax.imshow(img, origin='lower')  

# Plot big markers for start and end points
ax.scatter(start[0], start[1], s=120, c='navy', edgecolors='white', linewidths=2, marker='o', label='Start (restaurant)', zorder=10)
ax.scatter(end[0], end[1], s=120, c='lime', edgecolors='black', linewidths=2, marker='*', label='End point', zorder=10)

cbar = plt.colorbar(cm.ScalarMappable(norm=mcolors.Normalize(vmin=0.0, vmax=max_weight), cmap=cmap), ax=ax)
cbar.set_label("Navigation Cost (normalized)", fontsize=10)

legend_elements = [
    Patch(facecolor='black', edgecolor='black', label='No-fly zones / obstacles'),
    Patch(facecolor='purple', edgecolor='black', label='A* path (low alpha)'),
    Patch(facecolor='lime', edgecolor='black', label='A* path (high alpha)'),
    Patch(facecolor='navy', edgecolor='white', label='Start (restaurant)'),
    Patch(facecolor='lime', edgecolor='black', label='End point')
]

ax.legend(handles=legend_elements, loc='upper right', frameon=True)

ax.set_title("A* Path on Weighted Delft Grid")
ax.axis("off")
plt.tight_layout()
plt.show()

# Print Pareto-eque curve
fig2, ax2 = plt.subplots()

for paths in paths_n: 
    distances = [p[1] for p in paths]
    noises = [p[2] for p in paths]
    ax2.plot(distances, noises, marker='o', label='Distance - Noise line')
    
ax2.set_xlabel('Distance')
ax2.set_ylabel('Noise')
ax2.set_title('Path Distance vs Weight at varying Alpha')
ax2.legend()
plt.tight_layout()
plt.show()

