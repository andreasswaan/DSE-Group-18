# pathplanning/Algorithms/Astar_random_plot.py

import numpy as np
import heapq
from math import sqrt

# Richting: 8-richting
DIRECTIONS = [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]

# Standaard kosten voor bevolkingsdichtheid, optioneel mee te geven
DEFAULT_DENSITY_COST = 0.1  # uniform als densities niet meegegeven

# ---------- UTILITIES ----------

def heuristic(a, b):
    return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def path_length(path):
    return sum(heuristic(path[i-1], path[i]) for i in range(1, len(path)))

def is_line_clear(p1, p2, walkable):
    height, width = walkable.shape
    points = np.linspace(p1, p2, num=int(heuristic(p1, p2)*2))
    return all(0 <= int(y) < height and 0 <= int(x) < width and walkable[int(y), int(x)] for x, y in points)

def smooth_path(path, walkable):
    if not path:
        return []
    smoothed = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            if is_line_clear(path[i], path[j], walkable):
                break
            j -= 1
        smoothed.append(path[j])
        i = j
    return smoothed

# ---------- A* ALGORITHM ----------

def a_star_search_8dir(start, end, walkable, density_map=None, 
                       density_cost_map=None, alpha = 0.5):
    """
    start, end: (x, y)
    walkable: boolean 2D array
    density_map: optional 2D array of category letters (e.g. 'A', 'B', 'C')
    density_cost_map: dict mapping letter → cost, e.g. {'A': 0.0, 'B': 0.1, 'C': 0.3}
    """
    height, width = walkable.shape

    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, end), 0, start, []))
    visited = set()

    while open_set:
        _, current_cost, current, path = heapq.heappop(open_set)
        if current in visited:
            continue
        visited.add(current)
        path = path + [current]
        if current == end:
            return path

        x, y = current
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height and walkable[ny, nx]:
                step_cost = sqrt(dx**2 + dy**2) /sqrt (2) #devide over sqrt 2 to normalize

                # Use numeric cost from weight grid (e.g., population density, height, etc.)
                extra_cost = 0.0
                if density_map is not None:
                    extra_cost = float(density_map[ny, nx])
                    if np.isinf(extra_cost):
                        continue  # skip impassable

                total_cost = alpha * step_cost + (1-alpha) * extra_cost #high ALPHA mean density is neglect right ---> shortest path

                heapq.heappush(open_set, (
                    current_cost + total_cost + heuristic((nx, ny), end)*(alpha+0.01),
                    current_cost + total_cost,
                    (nx, ny),
                    path
                ))
    return []


# ---------- TEST ENTRY ----------

if __name__ == "__main__":
    # Alleen voor standalone testen
    import matplotlib.pyplot as plt

    width, height = 50, 50
    np.random.seed(42)

    densities = np.random.choice(['0.0', '0.9', '0.1'], size=(height, width))
    walkable = np.random.rand(height, width) > 0.2
    walkable[0, 0] = True
    walkable[-1, -1] = True

    start = (0, 0)
    end = (width-1, height-1)

    path = a_star_search_8dir(start, end, walkable, densities)
    smoothed = smooth_path(path, walkable)


