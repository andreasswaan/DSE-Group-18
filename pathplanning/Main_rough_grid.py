import numpy as np
import matplotlib.pyplot as plt
from Algorithms.Astar_random_plot_copy import a_star_search_8dir, path_length
import matplotlib.cm as cm
import matplotlib.colors as mcolors
from matplotlib.patches import Patch
import time 
from shapely.geometry import LineString, Polygon
#from corners import rounded_corners_path, simplify_path, downsample_path
from skimage import measure
from scipy.interpolate import splprep, splev

def load_delft_grid(path="pathplanning/data/delft_grid_data_70.npz"):
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

def spline_path(path, s=1.5, num=300, k=3):
    """
    Smooth a path using spline interpolation.
    - s: smoothing factor
    - num: number of samples in the output
    - k: spline order (usually 3)
    """
    x, y = zip(*path)
    if len(path) <= 3:
        return path  # can't fit spline with < 4 points
    k = min(k, len(path) - 1)
    tck, u = splprep([x, y], s=s, k=k)
    u_new = np.linspace(0, 1, num=num)
    x_new, y_new = splev(u_new, tck)
    return list(zip(x_new, y_new))

def estimate_curvature(path):
    """Estimate curvature (1/radius) at each point on the path."""
    path = np.array(path)
    dx = np.gradient(path[:, 0])
    dy = np.gradient(path[:, 1])
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    curvature = np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2)**1.5
    radius = 1 / (curvature + 1e-8)
    return radius


def path_is_safe_spline(smoothed_path, obstacle_mask):
    for x, y in smoothed_path:
        xi, yi = int(round(x)), int(round(y))
        if 0 <= xi < obstacle_mask.shape[1] and 0 <= yi < obstacle_mask.shape[0]:
            if obstacle_mask[yi, xi]:
                return False
    return True

def plot_path(grid, walkable, start, end, paths):
    weight_grid = grid['weight_grid'].copy()
    max_weight = np.nanmax(weight_grid[weight_grid < np.inf])
    normalized = np.clip(weight_grid, 0, max_weight) / max_weight
    cmap = cm.get_cmap('YlOrRd')
    rgba = cmap(normalized)
    img = rgba[:, :, :3].copy()
    img[~walkable] = [0, 0, 0]

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(img, origin='lower')

    colors = plt.colormaps['tab10']

    # 1. Plot obstacle polygons for reference (add this here!)
    for poly in obstacle_polygons:
        x, y = poly.exterior.xy
        ax.fill(x, y, color='black', alpha=0.3)


    for idx, (alpha, final_path, smoothed_path, collided) in enumerate(paths):
        color = colors(idx % 10)
        ax.plot([x for x, y in final_path], [y for x, y in final_path],
                linestyle='solid', color=color, linewidth=2.5, label=f"A* α={alpha:.2f} (used)")
        if collided:
            ax.plot([x for x, y in smoothed_path], [y for x, y in smoothed_path],
                    linestyle='dashed', color='red', linewidth=1.5, label=f"A* α={alpha:.2f} (rejected spline)")



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


contours = measure.find_contours((~walkable).astype(float), 0.5)
obstacle_polygons = []
for contour in contours:
    # Convert contour to (x,y) format
    poly = Polygon([(x, y) for y, x in contour])
    if poly.is_valid and poly.area > 1.0:
        obstacle_polygons.append(poly)

restaurant_coords = np.argwhere(grid['restaurant_grid'])

if len(restaurant_coords) == 0:
    raise ValueError("❌ No restaurant locations found.")

height, width = grid['weight_grid'].shape
center_x, center_y = width // 2, height // 2

np.random.seed(42)  # For reproducibility

alphas = 0.1,0.2
# Select random start and end points
start = (212, 324) #select_random_restaurant(restaurant_coords) (x,y) 
end = (212, 524) #select_near_center(walkable, center_x, center_y, width, height)
print(f"  Start: {start}")
print(f"  End:   {end}")

MIN_TURN_RADIUS_M = 60
GRID_RES = grid["resolution"]
MIN_TURN_RADIUS_GRID = MIN_TURN_RADIUS_M / GRID_RES  # e.g., 6 for 10m/cell

alphas = np.linspace(0.1,1.1,10)  # You can extend as needed

start = (30, 40)
end = (32, 54)
print(f"  Start: {start}")
print(f"  End:   {end}")

paths = []
obstacle_mask = np.isinf(grid['weight_grid'])

for alpha in alphas:
    t0 = time.time()
    path = a_star_search_8dir(start, end, walkable, density_map=grid['weight_grid'], alpha=alpha)
    t1 = time.time()
    duration = t1 - t0

    if not path:
        print(f"❌ No path found for alpha={alpha:.2f}")
        continue

    # --- Spline smoothing with radius constraint ---
    s = 1.0
    s_max = 25.0
    s_step = 0.5
    last_smoothed = None
    last_min_radius = None
    while s <= s_max:
        smoothed = spline_path(path, s=s, num=300)
        radii = estimate_curvature(smoothed)
        min_radius = np.min(radii)
        last_smoothed = smoothed
        last_min_radius = min_radius
        print(f"  Alpha: {alpha:.2f} | s={s:.1f} | MinRadius: {min_radius*GRID_RES:.1f} m")
        if min_radius >= MIN_TURN_RADIUS_GRID:
            break
        s += s_step

    # Always use the smoothest spline (even if constraint isn't met)
    final_path = last_smoothed
    collided = False  # Not falling back to A*

    paths.append((alpha, final_path, last_smoothed, collided))
    print(f"  Alpha: {alpha:.2f} || Raw: {path_length(path):.2f} | ⏱ {duration:.4f} seconds")
    print(f"  Final minimum turning radius: {last_min_radius * grid['resolution']:.1f} m" +
          (" (Constraint met)" if last_min_radius >= MIN_TURN_RADIUS_GRID else " (Constraint NOT met)"))

plot_path(grid, walkable, start, end, paths)
