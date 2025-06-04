import numpy as np
import matplotlib.pyplot as plt
from Algorithms.Astar_random_plot_copy import a_star_search_8dir, smooth_path, path_length
import matplotlib.cm as cm
import matplotlib.colors as mcolors
from matplotlib.patches import Patch
import time 

def load_delft_grid(path="pathplanning/data/rough_grid_data.npz"):
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
def select_random_restaurant(restaurant_coords):
    idx = np.random.choice(len(restaurant_coords))
    return tuple(restaurant_coords[idx][::-1])  # Flip to (x, y)

def select_near_center(walkable, center_x, center_y, width, height):
    for _ in range(500):
        dx, dy = np.random.randint(-20, 20), np.random.randint(-20, 20)
        x, y = center_x + dx, center_y + dy
        if 0 <= x < width and 0 <= y < height and walkable[y, x]:
            return (x, y)
    raise ValueError("❌ Could not find a valid endpoint near the center.")

def plot_path(grid, walkable, start, end, path, smoothed_path=None):
    weight_grid = grid['weight_grid'].copy()
    max_weight = np.nanmax(weight_grid[weight_grid < np.inf])
    normalized = np.clip(weight_grid, 0, max_weight) / max_weight
    cmap = cm.get_cmap('YlOrRd')
    rgba = cmap(normalized)
    img = rgba[:, :, :3].copy()
    img[~walkable] = [0, 0, 0]

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(img, origin='lower')

    colors = cm.get_cmap('tab10')

    for idx, (alpha, path) in enumerate(paths):
        color = colors(idx % 10)
        ax.plot([x for x, y in path], [y for x, y in path], color=color, linewidth=2, label=f"A* α={alpha:.2f}")

    ax.plot(start[0], start[1], 'bo', label='Start (restaurant)', markersize=6)
    ax.plot(end[0], end[1], 'go', label='End point', markersize=6)

    cbar = plt.colorbar(cm.ScalarMappable(norm=mcolors.Normalize(vmin=0.0, vmax=max_weight), cmap=cmap), ax=ax)
    cbar.set_label("Navigation Cost (normalized)", fontsize=10)

    ax.legend(loc='upper right', frameon=True)
    ax.axis("off")
    plt.tight_layout()
    plt.show()

# ---------- Main execution ----------
grid = load_delft_grid()
walkable = ~grid['obstacle_grid']
restaurant_coords = np.argwhere(grid['restaurant_grid'])

if len(restaurant_coords) == 0:
    raise ValueError("❌ No restaurant locations found.")

height, width = grid['weight_grid'].shape
center_x, center_y = width // 2, height // 2

np.random.seed(42)  # For reproducibility

alphas = np.linspace(0.1,1.0,10)

  # Make 3 plots
start = (212, 124) #select_random_restaurant(restaurant_coords) (x,y) 
end = (212, 524) #select_near_center(walkable, center_x, center_y, width, height)
print(f"  Start: {start}")
print(f"  End:   {end}")

paths = []

for alpha in alphas: 
    t0 = time.time()
    path = a_star_search_8dir(start, end, walkable, density_map=grid['weight_grid'],alpha=alpha)
    t1= time.time()
    duration = t1 - t0
    if not path:
        print(f"❌ No path found for alpha ={alpha}")
        continue
    paths.append((alpha,path))

    #smoothed_path = smooth_path(path, walkable)
    print(f"  Alpha: {alpha:.2f} || Raw: {path_length(path):.2f} | ⏱ {duration:.4f} seconds")

plot_path(grid, walkable, start, end, paths)


