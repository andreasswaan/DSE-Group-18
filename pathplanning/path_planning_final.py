import numpy as np
from pathplanning.Algorithms.Astar_random_plot import a_star_search_8dir
from scipy.interpolate import splprep, splev



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


    
def calculate_smooth_path(start, end, walkable, density_map, MIN_TURN_RADIUS_GRID=1, alpha=0.7):
    
    if not walkable[start[1], start[0]] or not walkable[end[1], end[0]]:
        return [], 0, 0
    path_total = a_star_search_8dir(start, end, walkable, density_map=density_map, alpha=alpha)
    path = path_total[0]  # Extract the path from the tuple
    path = fix_corner_jumps(path, walkable)
    
    if not path:
        print(f"❌ No path found for alpha={alpha:.2f}")
    else:  
        # --- Spline smoothing with radius constraint ---
        s = 0.1
        s_max = 25.0
        s_step = 0.1
        while s <= s_max:
            smoothed = spline_path(path, s=s, num=300)
            radii = estimate_curvature(smoothed)
            min_radius = np.min(radii)
            # print(f"  Alpha: {alpha:.2f} | s={s:.1f} | MinRadius: {min_radius*GRID_RES:.1f} m")
            if min_radius >= MIN_TURN_RADIUS_GRID:
                break
            s += s_step
        
    return smoothed, path_total[1], path_total[2]  # Return the smoothed path, total step cost, and total weight cost

# # ---------- Main execution ----------
# grid = load_delft_grid()
# walkable = ~grid['obstacle_grid']


# contours = measure.find_contours((~walkable).astype(float), 0.5)
# obstacle_polygons = []
# for contour in contours:
#     # Convert contour to (x,y) format
#     poly = Polygon([(x, y) for y, x in contour])
#     if poly.is_valid and poly.area > 1.0:
#         obstacle_polygons.append(poly)

# restaurant_coords = np.argwhere(grid['restaurant_grid'])

# if len(restaurant_coords) == 0:
#     raise ValueError("❌ No restaurant locations found.")

# height, width = grid['weight_grid'].shape
# center_x, center_y = width // 2, height // 2

# np.random.seed(42)  # For reproducibility

# alphas = 0.1,0.2
# # Select random start and end points
# start = (212, 324) #select_random_restaurant(restaurant_coords) (x,y) 
# end = (212, 524) #select_near_center(walkable, center_x, center_y, width, height)
# print(f"  Start: {start}")
# print(f"  End:   {end}")

# MIN_TURN_RADIUS_M = 70
# GRID_RES = grid["resolution"]
# MIN_TURN_RADIUS_GRID = MIN_TURN_RADIUS_M / GRID_RES  # e.g., 6 for 10m/cell

# alphas = np.linspace(0.1,1.1,10)  # You can extend as needed

# start = (30, 49)
# end = (40, 73)
# start = (31, 46)
# end = (36, 65)
# # start = (190, 355)
# # end = (230, 430)
# print(f"  Start: {start}")
# print(f"  End:   {end}")

# paths = []
# obstacle_mask = np.isinf(grid['weight_grid'])

# for alpha in alphas:
#     t0 = time.time()
#     path = a_star_search_8dir(start, end, walkable, density_map=grid['weight_grid'], alpha=alpha)
#     path = fix_corner_jumps(path, walkable)
#     t1 = time.time()
#     duration = t1 - t0

#     if not path:
#         print(f"❌ No path found for alpha={alpha:.2f}")
#         continue

#     # --- Spline smoothing with radius constraint ---
#     s = 0.1
#     s_max = 25.0
#     s_step = 0.1
#     last_smoothed = None
#     last_min_radius = None
#     while s <= s_max:
#         smoothed = spline_path(path, s=s, num=300)
#         radii = estimate_curvature(smoothed)
#         min_radius = np.min(radii)
#         last_smoothed = smoothed
#         last_min_radius = min_radius
#         print(f"  Alpha: {alpha:.2f} | s={s:.1f} | MinRadius: {min_radius*GRID_RES:.1f} m")
#         if min_radius >= MIN_TURN_RADIUS_GRID:
#             break
#         s += s_step

#     # Always use the smoothest spline (even if constraint isn't met)
#     final_path = last_smoothed
#     collided = False  # Not falling back to A*

#     paths.append((alpha, final_path, last_smoothed, collided))
#     print(f"  Alpha: {alpha:.2f} || Raw: {path_length(path):.2f} | ⏱ {duration:.4f} seconds")
#     print(f"  Final minimum turning radius: {last_min_radius * grid['resolution']:.1f} m" +
#           (" (Constraint met)" if last_min_radius >= MIN_TURN_RADIUS_GRID else " (Constraint NOT met)"))

# print(np.round(paths[0][1], 2))
# plot_path(grid, walkable, start, end, paths)
