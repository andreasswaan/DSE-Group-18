import numpy as np
import matplotlib.pyplot as plt
from Algorithms.Astar_random_plot import a_star_search_8dir, theta_star_search_8dir, smooth_path
import matplotlib.cm as cm
import matplotlib.colors as mcolors
import matplotlib.cm as cm
from matplotlib.patches import Patch

def load_delft_grid(path="pathplanning/data/delft_grid_data_10_border.npz"):
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
    
# ------------ 0. Fix corner jumps in path ----------    
def fix_corner_jumps(path, walkable):
    """
    For each diagonal step in the path, if both adjacent cardinal cells are not walkable (i.e., a corner cut),
    insert the cell across from the obstacle into the path to force the path to go around the obstacle.
    Returns a new path with the fix applied.
    """
    if not path or len(path) < 3:
        return path

    fixed_path = [path[0]]
    for i in range(1, len(path)):
        x0, y0 = fixed_path[-1]
        x1, y1 = path[i]
        dx, dy = x1 - x0, y1 - y0

        # Check for diagonal move
        if abs(dx) == 1 and abs(dy) == 1:
            # Check if both adjacent cardinal cells are not walkable (corner cut)
            if not walkable[y0, x1] or not walkable[y1, x0]:
                # Insert the cell across from the obstacle before the diagonal move
                # Prefer the cardinal direction that is walkable, or just pick one
                if walkable[y0, x1]:
                    fixed_path.append((x1, y0))
                elif walkable[y1, x0]:
                    fixed_path.append((x0, y1))
        fixed_path.append((x1, y1))
    return fixed_path

# ---------- 1. Load the Delft grid ----------
grid = load_delft_grid()
walkable = ~grid['obstacle_grid']  # True where traversal is allowed

def select_random_point():

    # ---------- 2. Select a start point (from restaurant locations) ----------
    restaurant_coords = np.argwhere(grid['restaurant_grid'])
    if len(restaurant_coords) == 0:
        raise ValueError("❌ No restaurant locations found in the grid.")

    # Only choose restaurant locations that are walkable
    walkable_restaurants = [tuple(coord[::-1]) for coord in restaurant_coords if walkable[tuple(coord)]]
    if not walkable_restaurants:
        raise ValueError("❌ No walkable restaurant locations found in the grid.")
    start = walkable_restaurants[np.random.choice(len(walkable_restaurants))]

    # ---------- 3. Select an end point using the weight grid as a probability density ----------
    weight_grid = grid['weight_grid']
    flat_weights = weight_grid.flatten()
    flat_weights = np.nan_to_num(flat_weights, nan=0.0, posinf=0.0, neginf=0.0)
    flat_weights[flat_weights < 0] = 0
    if np.sum(flat_weights) == 0:
        raise ValueError("❌ All weights are zero in the weight grid.")

    prob_weights = flat_weights / np.sum(flat_weights)

    # Resample until a walkable end point is found
    while True:
        chosen_index = np.random.choice(np.arange(weight_grid.shape[0] * weight_grid.shape[1]), p=prob_weights)
        end_x = chosen_index // weight_grid.shape[1]
        end_y = chosen_index % weight_grid.shape[1]
        end = (end_y, end_x)
        if 0 <= end_y < walkable.shape[0] and 0 <= end_x < walkable.shape[1] and walkable[end]:
            break
    return start, end

# print(f"Start point (restaurant): {start}")
# print(f"End point (sampled from weight grid): {end}")

# ---------- 4. Run A* algorithm ----------
# path = a_star_search_8dir(start, end, walkable, density_map=grid['weight_grid'])
# if not path:
#     raise RuntimeError("❌ A* could not find a path.")
# smoothed_path = smooth_path(path, walkable)

paths_n = []
alpha_step = 0.2
alphas = np.arange(0.0, 1+alpha_step, alpha_step)
num_runs = 1
np.random.seed(3)

for i in range(num_runs):
    start, end = select_random_point()
    print(f"------ Run {i+1}/{num_runs} ------")
    # start = tuple(restaurant_coords[np.random.randint(len(restaurant_coords))][::-1])
    # end = tuple([np.random.randint(0, width), np.random.randint(0, height)])
    start = tuple([200,300])
    end = tuple([250,500])
    start = tuple([160,290])
    end = tuple([275,330])
    start = tuple([180,460])
    end = tuple([270,520])
    end = tuple([211,555])
    # start = (30, 49)
    # end = (32, 76)
    print(f"Start point: {start}, End point: {end}")
    
    paths = []
    for i in alphas:
        print(i)
        alpha = i
        # paths.append(a_star_search_8dir(start, end, walkable, density_map=grid['weight_grid'], alpha=alpha))
        raw_path = a_star_search_8dir(start, end, walkable, density_map=grid['weight_grid'], alpha=alpha)
        fixed_path = fix_corner_jumps(raw_path[0], walkable)
        paths.append((fixed_path, raw_path[1], raw_path[2]))
        # paths.append(theta_star_search_8dir(start, end, walkable, density_map=grid['weight_grid'], alpha=alpha))
        
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

# Use purple-to-lime gradient for alpha within each run, but color each run differently
run_colors = plt.cm.tab10(np.linspace(0, 1, num_runs))  # distinct color for each run

for run_idx, paths in enumerate(paths_n):
    for path_i, alpha in zip(paths, alphas):
        color = run_colors[run_idx][:3]  # use only RGB
        for x, y in path_i[0]:
            img[y, x] = color
            
        # PLOTTING for Theta* paths
        # path_points = path_i[0]
        # # Draw straight lines between consecutive points in the path
        # for i in range(1, len(path_points)):
        #     x0, y0 = path_points[i-1]
        #     x1, y1 = path_points[i]
        #     rr, cc = np.linspace(y0, y1, 100), np.linspace(x0, x1, 100)
        #     img[rr.astype(int), cc.astype(int)] = color

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
cbar.set_label("Population Density (normalized)", fontsize=10)

legend_elements = [
    Patch(facecolor='black', edgecolor='black', label='No-fly zones / obstacles'),
    # Patch(facecolor='purple', edgecolor='black', label='A* path (low alpha)'),
    # Patch(facecolor='lime', edgecolor='black', label='A* path (high alpha)'),
    Patch(facecolor='navy', edgecolor='white', label='Start point'),
    Patch(facecolor='lime', edgecolor='black', label='End point')
]

ax.legend(handles=legend_elements, loc='upper right', frameon=True)

ax.set_title("A* Path on Weighted Delft Grid at Varying Alpha Values")
ax.axis("off")
plt.tight_layout()
plt.show()

# Print Pareto-eque curve
fig2, ax2 = plt.subplots()

# Choose a colormap for alphas
alpha_cmap = plt.cm.viridis
alpha_colors = [alpha_cmap(i / (len(alphas)-1)) for i in range(len(alphas))]

# Plot each path with a different color and label for alpha
for alpha_idx, alpha in enumerate(alphas):
    path_points = paths_n[0][alpha_idx][0]  # paths_n[run][alpha][0] is the path
    if len(path_points) > 1:
        xs, ys = zip(*path_points)
        ax.plot(xs, ys, color=alpha_colors[alpha_idx], linewidth=2,
                label=f'alpha={alpha:.2f}', zorder=6)

# Plot big markers for start and end points
ax.scatter(start[0], start[1], s=120, c='navy', edgecolors='white', linewidths=2, marker='o', label='Start (restaurant)', zorder=10)
ax.scatter(end[0], end[1], s=120, c='lime', edgecolors='black', linewidths=2, marker='*', label='End point', zorder=10)

# To avoid duplicate legend entries, only show each alpha label once
handles, labels = ax.get_legend_handles_labels()
unique = dict()
for h, l in zip(handles, labels):
    if l not in unique and l is not None:
        unique[l] = h
ax.legend(unique.values(), unique.keys(), loc='upper right', frameon=True)

ax2.set_xlabel('Distance [m]')
ax2.set_ylabel('Public disturbance due to noise')
ax2.set_title('Path Distance vs Public Disturbance at Varying Alpha')
ax2.legend()
# plt.tight_layout()
#plt.savefig("pathplanning/figures/pareto_curve_100_seed_3_refined.svg")
plt.show()



