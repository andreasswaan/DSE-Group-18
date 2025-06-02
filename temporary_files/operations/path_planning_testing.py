import numpy as np
from shapely.geometry import Point, LineString, Polygon
import shapely
import networkx as nx
import matplotlib.pyplot as plt
import math
import pickle
import os

# -------------------
# RANDOM MAP GENERATION
# -------------------

def random_polygon(center, avg_radius, irregularity, spikiness, num_vertices):
    """Generates a random polygon around a center point."""
    # Irregularity: [0,1] indicating variance in angle between vertices
    # Spikiness: [0,1] indicating variance in radius
    angle_steps = []
    lower = (2 * math.pi / num_vertices) * (1 - irregularity)
    upper = (2 * math.pi / num_vertices) * (1 + irregularity)
    sum = 0
    for _ in range(num_vertices):
        tmp = np.random.uniform(lower, upper)
        angle_steps.append(tmp)
        sum += tmp
    k = sum / (2 * math.pi)
    for i in range(num_vertices):
        angle_steps[i] /= k

    points = []
    angle = np.random.uniform(0, 2 * math.pi)
    for i in range(num_vertices):
        r = np.clip(np.random.normal(avg_radius, spikiness * avg_radius), 0.2 * avg_radius, 2 * avg_radius)
        x = center[0] + r * math.cos(angle)
        y = center[1] + r * math.sin(angle)
        points.append((x, y))
        angle += angle_steps[i]
    return Polygon(points)

def generate_random_map(
    map_size=12,
    n_obstacles=5,
    avg_obstacle_size=2,
    buffer_dist=0.2,
    seed=None,
    max_attempts=1000,
    circle_resolution=6, 
):
    """
    Generate a map with a mix of rectangular and circular obstacles, avoiding excessive overlap.
    circle_resolution: number of vertices for circular obstacles (low = less resolved)
    rectangle_vertices: number of vertices for rectangles (should be 4 for true rectangles)
    """
    if seed is not None:
        np.random.seed(seed)
    obstacles = []
    attempts = 0
    while len(obstacles) < n_obstacles and attempts < max_attempts:
        attempts += 1
        # Randomly choose shape
        is_rect = np.random.rand() < 0.5
        center = (np.random.uniform(1, map_size-1), np.random.uniform(1, map_size-1))
        if is_rect:
            # Rectangle (with low resolution)
            width = np.random.uniform(0.7, 1.3) * avg_obstacle_size
            height = np.random.uniform(0.7, 1.3) * avg_obstacle_size * np.random.uniform(0.5, 1.5)
            angle = np.random.uniform(0, 2*np.pi)
            rect = np.array([
                [-width/2, -height/2],
                [width/2, -height/2],
                [width/2, height/2],
                [-width/2, height/2]
            ])
            # Rotate rectangle
            rot = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
            rect = rect @ rot.T + center
            poly = Polygon(rect)
        else:
            # Circle (as low-res polygon)
            radius = np.random.uniform(0.5, 1.2) * avg_obstacle_size / 2
            angles = np.linspace(0, 2*np.pi, circle_resolution, endpoint=False)
            circle_pts = [(center[0] + radius * np.cos(a), center[1] + radius * np.sin(a)) for a in angles]
            poly = Polygon(circle_pts)
        poly = poly.buffer(buffer_dist, resolution=2)
        # Check for overlap
        if all(poly.distance(obs) > buffer_dist * 0.7 and not poly.intersects(obs) for obs in obstacles):
            if poly.is_valid and poly.area > 0.1:
                obstacles.append(poly)
    return obstacles

def generate_test_map_with_block(map_size=12):
    """Generate a map with a single wide rectangular obstacle between start and goal."""
    # Rectangle coordinates: block in the center, wide horizontally
    block_width = map_size * 0.7
    block_height = map_size * 0.2
    block_center = (map_size / 2, map_size / 2)
    x0 = block_center[0] - block_width / 2
    x1 = block_center[0] + block_width / 2
    y0 = block_center[1] - block_height / 2
    y1 = block_center[1] + block_height / 2
    block = Polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
    # Add buffer if desired
    block = block.buffer(BUFFER_DIST, resolution=2)
    return [block]

def generate_checkerboard_map(
    map_size=12,
    grid_size=4,
    obs_size=1.5,
    buffer_dist=0.2,
    noise=0.3,
    area_factor=0.5, 
    seed=None
):
    """
    Generate a map with a checkerboard pattern of obstacles (like one color of a chessboard),
    with small random noise in obstacle positions and spaces between buildings.
    Only the inner (grid_size-1)x(grid_size-1) cells are used for obstacles.
    """
    if seed is not None:
        np.random.seed(seed)
    obstacles = []
    cell_size = map_size / grid_size
    effective_obs_size = min(obs_size, cell_size * area_factor)
    half = effective_obs_size / 2
    # Only use inner cells (skip i=0 and i=grid_size-1, same for j)
    for i in range(1, grid_size-1):
        for j in range(1, grid_size-1):
            if (i + j) % 2 == 0:
                # Center of the cell, with some noise
                cx = (i + 0.5) * cell_size + np.random.uniform(-noise, noise)
                cy = (j + 0.5) * cell_size + np.random.uniform(-noise, noise)
                rect = Polygon([
                    (cx - half, cy - half),
                    (cx + half, cy - half),
                    (cx + half, cy + half),
                    (cx - half, cy + half)
                ])
                rect = rect.buffer(buffer_dist, resolution=2)
                # Only add if inside map bounds (should always be true now)
                if rect.bounds[0] >= 0 and rect.bounds[2] <= map_size and rect.bounds[1] >= 0 and rect.bounds[3] <= map_size:
                    obstacles.append(rect)
    return obstacles

# -------------------
# PARAMETERS
# -------------------

MAP_SIZE = 12
N_OBSTACLES = 10
AVG_OBS_SIZE = 1
BUFFER_DIST = 0.2
CIRCLE_RESOLUTION = 5
SEED = 3


# Generate random obstacles
no_fly_zones = generate_random_map(
    map_size=MAP_SIZE,
    n_obstacles=N_OBSTACLES,
    avg_obstacle_size=AVG_OBS_SIZE,
    buffer_dist=BUFFER_DIST,
    seed=SEED
)

# Start and goal coordinates
start = (1, 1)
goal = (MAP_SIZE-1, MAP_SIZE-2)

# Create visibility graph nodes: start, goal, and obstacle vertices
nodes = [start, goal]
for zone in no_fly_zones:
    nodes.extend(list(zone.exterior.coords)[:-1])  # skip duplicate closing point
    
    
    
# UNCOMMENT for checkerboard map 
nodes = [start, goal]
checkerboard_no_fly_zones = generate_checkerboard_map(
    map_size=MAP_SIZE, grid_size=7, obs_size=1.5, buffer_dist=BUFFER_DIST, noise=1, area_factor=0.8, seed=SEED)
for zone in checkerboard_no_fly_zones:
    nodes.extend(list(zone.exterior.coords)[:-1])
no_fly_zones = checkerboard_no_fly_zones
    

# UNCOMMENT for testing with a fixed map
# nodes = [start, goal]
# test_no_fly_zones = generate_test_map_with_block(MAP_SIZE)
# for zone in test_no_fly_zones:
#     nodes.extend(list(zone.exterior.coords)[:-1])
# no_fly_zones = test_no_fly_zones

# --- Plot checkerboard map before any path finding ---
if __name__ == "__main__":
    fig, ax = plt.subplots(figsize=(8, 8))
    for zone in checkerboard_no_fly_zones:
        x, y = zone.exterior.xy
        ax.fill(x, y, color='brown', alpha=0.7)
    ax.set_xlim(0, MAP_SIZE)
    ax.set_ylim(0, MAP_SIZE)
    ax.set_aspect('equal')
    ax.set_title("Checkerboard Map Preview")
    plt.grid(True)
    plt.show()


# -------------------
# VISIBILITY GRAPH CONSTRUCTION (with caching)
# -------------------

def cache_visibility_graph(nodes, no_fly_zones, cache_file="vis_graph_cache.pkl"):
    # Use a hash of the obstacles for cache key
    obs_hash = hash(tuple([tuple(np.round(np.array(poly.exterior.coords).flatten(), 3)) for poly in no_fly_zones]))
    if os.path.exists(cache_file):
        with open(cache_file, "rb") as f:
            cache = pickle.load(f)
        if obs_hash in cache:
            return cache[obs_hash]
    # Build graph
    G = nx.Graph()
    for i, p1 in enumerate(nodes):
        for j, p2 in enumerate(nodes):
            if i >= j:
                continue
            line = LineString([p1, p2])
            if not any(line.crosses(poly) or line.within(poly) for poly in no_fly_zones):
                dist = math.dist(p1, p2)
                G.add_edge(p1, p2, weight=dist)
    # Save to cache
    if os.path.exists(cache_file):
        with open(cache_file, "rb") as f:
            cache = pickle.load(f)
    else:
        cache = {}
    cache[obs_hash] = G
    with open(cache_file, "wb") as f:
        pickle.dump(cache, f)
    return G

G = cache_visibility_graph(nodes, no_fly_zones)



# -------------------
# A* PATH PLANNING
# -------------------

def heuristic(a, b):
    return math.dist(a, b)

try:
    waypoints = nx.astar_path(G, start, goal, heuristic=heuristic)
except nx.NetworkXNoPath:
    waypoints = None



# -------------------
# PATH SMOOTHING
# -------------------


def round_corner_arc(A, B, C, r, num_arc_points=20):
    """Returns points for a rounded arc at corner B with radius r, tangent to both lines.
    The arc always starts at P (on AB) and ends at Q (on BC).
    The center is computed from P, at 90 degrees to v1 (left/right depending on turn), at distance r."""

    v1 = np.array([A[0] - B[0], A[1] - B[1]])
    v2 = np.array([C[0] - B[0], C[1] - B[1]])
    v1 = v1 / np.linalg.norm(v1)     
    v2 = v2 / np.linalg.norm(v2)
    # External angle (angle between headings)
    theta = np.pi - np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))
    d = r * np.tan(theta / 2)
    # Points on AB and BC at distance d from B
    P = (B[0] + v1[0]*d, B[1] + v1[1]*d)
    Q = (B[0] + v2[0]*d, B[1] + v2[1]*d)
    # Compute the direction for the center: rotate v1 by +/-90 deg
    cross = v1[0]*v2[1] - v1[1]*v2[0]
    if cross < 0:
        # Right turn: rotate v1 by -90 deg
        normal = np.array([v1[1], -v1[0]])
    else:
        # Left turn: rotate v1 by +90 deg
        normal = np.array([-v1[1], v1[0]])
    center = (P[0] + normal[0]*r, P[1] + normal[1]*r)
    # Start and end angles for the arc
    angle1 = np.arctan2(P[1] - center[1], P[0] - center[0])
    angle2 = np.arctan2(Q[1] - center[1], Q[0] - center[0])
    # Choose direction (shortest arc)
    angle_diff = (angle2 - angle1 + np.pi*3) % (2*np.pi) - np.pi
    if angle_diff > 0:
        arc_angles = np.linspace(angle1, angle2, num=num_arc_points)
    else:
        arc_angles = np.linspace(angle1, angle2, num=num_arc_points)[::-1]
    arc_x = center[0] + r * np.cos(arc_angles)
    arc_y = center[1] + r * np.sin(arc_angles)
    return P, Q, arc_x, arc_y

def rounded_corners_path(waypoints, r=0.1, num_arc_points=20):
    """
    Returns:
        - path_points: list of (x, y) tuples for the full rounded-corner path
        - P_list: list of P points (arc starts)
        - Q_list: list of Q points (arc ends)
    """
    if len(waypoints) < 2:
        return [], [], []

    path_points = []
    P_list = []
    Q_list = []

    prev_point = waypoints[0]
    path_points.append(prev_point)

    for i in range(1, len(waypoints)-1):
        A, B, C = waypoints[i-1], waypoints[i], waypoints[i+1]
        try:
            P, Q, arc_x, arc_y = round_corner_arc(A, B, C, r, num_arc_points)
        except Exception:
            # If arc fails, just use the sharp corner as a straight segment
            path_points.append(B)
            prev_point = B
            continue
        # Add straight segment from previous point to P
        if np.hypot(prev_point[0] - P[0], prev_point[1] - P[1]) > 1e-8:
            path_points.append(P)
        # Add arc points from P to Q (excluding duplicate P)
        arc_points = list(zip(arc_x, arc_y))
        if np.hypot(path_points[-1][0] - arc_points[0][0], path_points[-1][1] - arc_points[0][1]) < 1e-8:
            path_points.extend(arc_points[1:])
        else:
            path_points.extend(arc_points)
        P_list.append(P)
        Q_list.append(Q)
        prev_point = Q  # Next straight segment starts from Q

    # Final straight segment from last Q to goal
    if len(waypoints) > 2:
        last = waypoints[-1]
        if np.hypot(prev_point[0] - last[0], prev_point[1] - last[1]) > 1e-8:
            path_points.append(last)
    else:
        path_points.append(waypoints[-1])

    return path_points, P_list, Q_list
    

def mass_spring_relaxation_path(waypoints, no_fly_zones, iterations=1000, k=0.1, j=0.1, obstacle_avoid_radius=0.5, subdiv=20):
    """
    Smooth a path using a mass-spring relaxation method.
    - waypoints: list of (x, y) tuples (start to goal)
    - no_fly_zones: list of shapely Polygon obstacles
    - iterations: number of relaxation steps
    - k: spring constant (smoothness weight)
    - j: obstacle repulsion weight
    - obstacle_avoid_radius: distance at which obstacle repulsion is strongest
    Returns: list of (x, y) tuples (smoothed path)
    """

    points = np.array(waypoints, dtype=float)
    
    # Subdivide path for more control points
    fine_points = [points[0]]
    for i in range(len(points)-1):
        for s in range(1, subdiv+1):
            interp = points[i] + (points[i+1] - points[i]) * (s / (subdiv+1))
            fine_points.append(interp)
    fine_points.append(points[-1])
    fine_points = np.array(fine_points)
    fixed = np.zeros(len(fine_points), dtype=bool)
    fixed[0] = True
    fixed[-1] = True

    for it in range(iterations):
        new_points = fine_points.copy()
        for i in range(1, len(fine_points)-1):
            if fixed[i]:
                continue
            # Spring force: pull towards neighbors
            prev_vec = fine_points[i-1] - fine_points[i]
            next_vec = fine_points[i+1] - fine_points[i]
            normal_vec = prev_vec + next_vec

            # Obstacle repulsion
            obstacle_vec = np.zeros(2)
            min_dist = float('inf')
            for poly in no_fly_zones:
                p = Point(fine_points[i])
                dist = poly.exterior.distance(p)
                if dist < min_dist:
                    min_dist = dist
                    # Get nearest point on obstacle
                    nearest = np.array(poly.exterior.interpolate(poly.exterior.project(p)).coords[0])
                    obstacle_vec = fine_points[i] - nearest
            if min_dist < obstacle_avoid_radius:
                # Repulsion increases as we get closer, rapidly
                repulse = (obstacle_avoid_radius - min_dist) / obstacle_avoid_radius
                obstacle_vec = obstacle_vec / (np.linalg.norm(obstacle_vec) + 1e-8) * repulse * (min_dist**2)
            else:
                obstacle_vec = np.zeros(2)

            # Update position
            new_points[i] += k * normal_vec + j * obstacle_vec

        fine_points = new_points

    return [tuple(pt) for pt in fine_points]
    
    
    
    
# -------------------
# PLOT PATHS AND SCENE
# -------------------
    
    
def plot_scene(path=None, rounded_path=None, P_list=None, Q_list=None, msd_path=None):
    fig, ax = plt.subplots(figsize=(8, 8))

    # Plot actual obstacles and their buffer zones
    for zone in no_fly_zones:
        x, y = zone.exterior.xy
        ax.fill(x, y, color='red', alpha=0.3, label='Buffer' if 'Buffer' not in ax.get_legend_handles_labels()[1] else "")
        original = zone.buffer(-BUFFER_DIST, resolution=2)
        if original.is_empty:
            continue
        if original.geom_type == 'Polygon':
            x0, y0 = original.exterior.xy
            ax.fill(x0, y0, color='brown', alpha=0.7, label='Actual Obstacle' if 'Actual Obstacle' not in ax.get_legend_handles_labels()[1] else "")
        elif original.geom_type == 'MultiPolygon':
            for poly in original.geoms:
                x0, y0 = poly.exterior.xy
                ax.fill(x0, y0, color='brown', alpha=0.7, label='Actual Obstacle' if 'Actual Obstacle' not in ax.get_legend_handles_labels()[1] else "")

    # Plot nodes
    for node in nodes:
        ax.plot(*node, 'ko', markersize=4)

    # Plot visibility edges
    for (u, v) in G.edges:
        ax.plot([u[0], v[0]], [u[1], v[1]], 'lightgray', linewidth=0.8)

    # Plot A* path
    if path:
        px, py = zip(*path)
        ax.plot(px, py, 'b-', linewidth=2, label='A* Path')

    # Plot rounded corners path (orange, one line)
    if rounded_path:
        rx, ry = zip(*rounded_path)
        ax.plot(rx, ry, color='orange', linewidth=2, label='Rounded Corners Path')

    # Optionally plot P and Q points
    # if P_list:
    #     px, py = zip(*P_list)
    #     ax.plot(px, py, 'ro', markersize=7, label='P (arc start)')
    # if Q_list:
    #     qx, qy = zip(*Q_list)
    #     ax.plot(qx, qy, 'go', markersize=7, label='Q (arc end)')

    # Plot mass-spring-damper (relaxation) path
    if msd_path:
        mx, my = zip(*msd_path)
        ax.plot(mx, my, color='cyan', linewidth=2, label='Mass-Spring Path')

    # Start and goal
    ax.plot(*start, 'go', markersize=10, label='Start')
    ax.plot(*goal, 'ro', markersize=10, label='Goal')

    ax.set_title("A* Path (blue), Rounded Corners (orange), Mass-Spring (cyan)")
    ax.legend()
    ax.set_xlim(0, MAP_SIZE)
    ax.set_ylim(0, MAP_SIZE)
    plt.grid(True)
    plt.show()


if waypoints is not None:
    rounded_path, P_list, Q_list = rounded_corners_path(waypoints, r=0.1, num_arc_points=20)
    msd_path = mass_spring_relaxation_path(waypoints, no_fly_zones, iterations=500, k=0.2, j=0.2, obstacle_avoid_radius=0.5, subdiv=20)
else:
    rounded_path, P_list, Q_list = [], [], []
    msd_path = []

# Update plot_scene call to use consistent outputs and plot msd_path
plot_scene(
    path=waypoints,
    rounded_path=rounded_path,
    P_list=P_list,
    Q_list=Q_list,
    msd_path=msd_path
)


