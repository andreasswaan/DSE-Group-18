from gurobipy import Model, GRB, quicksum
import numpy as np
import matplotlib.pyplot as plt

# Parameters
n = 10  # number of orders
num_restaurants = 5  # number of restaurants
K = list(range(3))  # Set of drones/vehicles
Q = 5  # Drone capacity
M = 1e4  # big-M

# Sets
P = list(range(1, n + 1))               # Set of pickup nodes
D = list(range(n + 1, 2 * n + 1))       # Set of delivery nodes
N = [0] + P + D + [2 * n + 1]           # Set of all nodes (depot, pickups, deliveries)

# Generate random (x, y) locations for restaurants
np.random.seed(0)
restaurant_ids = list(range(100, 100 + num_restaurants))
restaurant_locations = {rid: (np.random.uniform(0, 10), np.random.uniform(0, 10)) for rid in restaurant_ids}

# Assign each order to a random restaurant
pickup_restaurant = {i: np.random.choice(restaurant_ids) for i in P}

# --- Set depot location to the center of the area instead of (0, 0) ---

# Define area size (matching the random.uniform(0, 10) for other locations)
area_min, area_max = 0, 10
depot_x = (area_min + area_max) / 2
depot_y = (area_min + area_max) / 2

# Generate random delivery locations and set depot at center
locations = {0: (depot_x, depot_y), 2 * n + 1: (depot_x, depot_y)}  # depots at center
for i in P:
    # Pickup node location is the assigned restaurant's location
    locations[i] = restaurant_locations[pickup_restaurant[i]]
for j in D:
    locations[j] = (np.random.uniform(area_min, area_max), np.random.uniform(area_min, area_max))

# Compute Euclidean travel times/distances (assuming speed = 1 unit/time)
def euclidean_dist(a, b):
    return np.hypot(a[0] - b[0], a[1] - b[1])

t_ij = {}
for i in N:
    for j in N:
        if i != j:
            t_ij[i, j] = euclidean_dist(locations[i], locations[j])

# Time windows [eᵢ, lᵢ]
e_i = {i: 0 for i in N}
l_i = {i: 50 for i in N}

# Service time sᵢ
s_i = {i: 2 for i in N}

# Demand qᵢ
q_i = {i: 1 if i in P else (-1 if i in D else 0) for i in N}

# Model
m = Model("DARP")
m.setParam('TimeLimit', 300)  # Set maximum runtime to 30 seconds

# Variables
x_kij = m.addVars(K, N, N, vtype=GRB.BINARY, name="x")
a_ki = m.addVars(K, N, vtype=GRB.CONTINUOUS, name="a")
l_ki = m.addVars(K, N, vtype=GRB.INTEGER, name="l")

# Objective: minimize total travel time
m.setObjective(quicksum(x_kij[k, i, j] * t_ij[i, j] for k in K for i in N for j in N if i != j), GRB.MINIMIZE)

# Each pickup and delivery is visited exactly once
for i in P + D:
    m.addConstr(quicksum(x_kij[k, i, j] for k in K for j in N if j != i) == 1)

# Flow conservation
for k in K:
    for h in N:
        if h != 0 and h != 2 * n + 1:
            m.addConstr(
                quicksum(x_kij[k, i, h] for i in N if i != h) ==
                quicksum(x_kij[k, h, j] for j in N if j != h)
            )

# Depot constraints
for k in K:
    m.addConstr(quicksum(x_kij[k, 0, j] for j in N if j != 0) == 1)
    m.addConstr(quicksum(x_kij[k, i, 2 * n + 1] for i in N if i != 2 * n + 1) == 1)

# Capacity propagation
for k in K:
    for i in N:
        for j in N:
            if i != j:
                m.addConstr(l_ki[k, j] >= l_ki[k, i] + q_i[j] - Q * (1 - x_kij[k, i, j]))

# Capacity bounds
for k in K:
    for i in N:
        m.addConstr(l_ki[k, i] >= 0)
        m.addConstr(l_ki[k, i] <= Q)

# Time propagation
for k in K:
    for i in N:
        for j in N:
            if i != j:
                m.addConstr(a_ki[k, j] >= a_ki[k, i] + s_i[i] + t_ij[i, j] - M * (1 - x_kij[k, i, j]))

# Time windows
for k in K:
    for i in N:
        m.addConstr(a_ki[k, i] >= e_i[i])
        m.addConstr(a_ki[k, i] <= l_i[i])

# Pickup before delivery
for k in K:
    for i in P:
        m.addConstr(a_ki[k, i + n] >= a_ki[k, i] + s_i[i])

# Pickup and delivery by the same drone
for i in P:
    for k in K:
        m.addConstr(
            quicksum(x_kij[k, i, j] for j in N if j != i) ==
            quicksum(x_kij[k, j, i + n] for j in N if j != i + n)
        )

# Initial load at depot is 0
for k in K:
    m.addConstr(l_ki[k, 0] == 0)

# Final load at end depot is 0
for k in K:
    m.addConstr(l_ki[k, 2 * n + 1] == 0)

m.optimize()

# Results (routes, times)
if m.status in [GRB.OPTIMAL, GRB.TIME_LIMIT, GRB.SUBOPTIMAL]:
    print("\nBest solution found (may be suboptimal if time limit reached):")
    for k in K:
        print(f"\nDrone {k}:")
        for i in N:
            for j in N:
                var = x_kij[k, i, j]
                # Only print arcs used in the best solution found
                if i != j and var.X > 0.5:
                    print(f"{i} -> {j}, arrival at {a_ki[k, i].X:.1f}")
elif m.status == GRB.INFEASIBLE:
    print("Model is infeasible. Computing IIS...")
    m.computeIIS()
    m.write("model.ilp")
    print("IIS written to model.ilp")
else:
    print("No optimal solution found.")

# Plot node locations
plt.figure(figsize=(8, 8))
colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown']

# Plot depots, pickups, deliveries
for v in N:
    x_coord, y_coord = locations[v]
    if v == 0:
        plt.scatter(x_coord, y_coord, c='black', marker='s', s=100, label='Depot')
        plt.text(x_coord, y_coord, 'Depot', fontsize=9, ha='right')
    elif v == 2 * n + 1:
        plt.scatter(x_coord, y_coord, c='black', marker='*', s=120, label='End Depot')
        plt.text(x_coord, y_coord, 'End', fontsize=9, ha='right')
    elif v in P:
        # Show which restaurant this pickup belongs to
        rest_id = pickup_restaurant[v]
        plt.scatter(x_coord, y_coord, c='green', marker='o', s=60, label='Pickup' if v == 1 else "")
        plt.text(x_coord, y_coord, f'P{v}\nR{rest_id-99}', fontsize=8, ha='right')
    elif v in D:
        plt.scatter(x_coord, y_coord, c='orange', marker='^', s=60, label='Delivery' if v == n+1 else "")
        plt.text(x_coord, y_coord, f'D{v}', fontsize=9, ha='right')

# Plot restaurant locations (optional, for visualization)
for idx, (rid, (x, y)) in enumerate(restaurant_locations.items()):
    plt.scatter(x, y, c='magenta', marker='P', s=100, label='Restaurant' if idx == 0 else "")
    plt.text(x, y, f'R{rid-99}', fontsize=9, ha='left')

# Plot routes for each drone
for k in K:
    route_edges = []
    for i in N:
        for j in N:
            if i != j and x_kij[k, i, j].X > 0.5:
                route_edges.append((i, j))
    for idx, (i, j) in enumerate(route_edges):
        x_vals = [locations[i][0], locations[j][0]]
        y_vals = [locations[i][1], locations[j][1]]
        # Only add label for the first edge of each drone
        plt.plot(x_vals, y_vals, color=colors[k % len(colors)], linewidth=2, alpha=0.7,
                 label=f'Drone {k}' if idx == 0 else "")

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Optimal Drone Routes (Best Found)')
handles, labels = plt.gca().get_legend_handles_labels()
by_label = dict(zip(labels, handles))
plt.legend(by_label.values(), by_label.keys())
plt.grid(True)
plt.show()
