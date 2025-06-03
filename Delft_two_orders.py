import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rasterio
from pyproj import Transformer
import itertools

# === LOAD DSM ===
DSM_PATH = "/Users/feliceverbeek/Documents/GitHub/DSE-Group-18/Images/DSM_Delft.tif"
with rasterio.open(DSM_PATH) as src:
    dsm = src.read(1)
    transform = src.transform
    width = src.width
    height = src.height
    x_min, y_max = transform[2], transform[5]
    pixel_width, pixel_height = transform[0], -transform[4]
    x_max = x_min + width * pixel_width
    y_min = y_max - height * pixel_height
    extent = [x_min, x_max, y_min, y_max]

# Clean DSM
dsm = np.where(dsm > 1000, np.nan, dsm)

# === FIXED LOCATIONS ===
x_rest, y_rest = 83500.0, 448000.0
x_depot, y_depot = 82500.0, 447000.0
position = np.array([x_depot, y_depot])

# === RANDOM ORDERS ===
rng = np.random.default_rng()
order_weights = [0.5, 1.0, 1.5, 2.0, 2.5]

def generate_random_order():
    return {
        'x': rng.uniform(x_min + 100, x_max - 100),
        'y': rng.uniform(y_min + 100, y_max - 100),
        'weight': rng.choice(order_weights)
    }

orders = [generate_random_order(), generate_random_order()]

# === DRONE STATE ===
speedup = 2
time_step = 1
v_vertical = 5.0
v_horizontal = 10.0
max_altitude = 40.0
safety_margin = 3.0
altitude = 0.0
state = 'ascend'
total_time = 0
avoiding = False
detour = None
battery = 100.0
min_battery = 20.0

# === BATTERY DEPLETION FUNCTION ===
def get_depletion_rate(weight):
    base_rate = 0.05
    return base_rate * (1 + 0.2 * (weight / 0.5))

def max_range(weight):
    rate = get_depletion_rate(weight)
    return ((battery - min_battery) / rate) * v_horizontal

# === PATH STRATEGY ===
def total_energy(path):
    pos = np.array([x_depot, y_depot])
    total_weight = sum(order['weight'] for order in path if 'x' in order and 'weight' in order)
    total = 0
    for stop in path:
        tgt = np.array([stop['x'], stop['y']])
        dist = np.linalg.norm(tgt - pos)
        rate = get_depletion_rate(total_weight)
        total += rate * dist / v_horizontal
        if 'weight' in stop:
            total_weight -= stop['weight']
        pos = tgt
    return total

rest = {'x': x_rest, 'y': y_rest}
depot = {'x': x_depot, 'y': y_depot, 'weight': 0}
optA = [rest, orders[0], orders[1], depot]
optB = [rest, orders[0], rest, orders[1], depot]

best_path = optA if total_energy(optA) < total_energy(optB) else optB

# === ROUTING ===
targets = best_path
step_index = 0
current_target = np.array([targets[step_index]['x'], targets[step_index]['y']])

# === PLOT SETUP ===
fig, ax = plt.subplots(figsize=(10, 10))
im = ax.imshow(dsm, cmap='terrain', extent=extent, origin='upper', vmin=0, vmax=30, zorder=0)
cbar = plt.colorbar(im, ax=ax, label='Elevation (m)')

ax.plot(x_rest, y_rest, 'bo', label="Restaurant")
ax.plot(x_depot, y_depot, 'bs', label="Depot")
order1_dot = ax.plot(orders[0]['x'], orders[0]['y'], 'yo', label="Customer 1")[0]
order2_dot = ax.plot(orders[1]['x'], orders[1]['y'], 'co', label="Customer 2")[0]

drone_dot, = ax.plot([], [], 'ro', markersize=6, label="Drone")
time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, bbox=dict(facecolor='white', alpha=0.8))
ax.set_xlim(x_min, x_max)
ax.set_ylim(y_min, y_max)
ax.legend()

# === HEIGHT FUNCTION ===
def get_dsm_height(x, y):
    col, row = ~transform * (x, y)
    row = int(np.clip(row, 0, dsm.shape[0] - 1))
    col = int(np.clip(col, 0, dsm.shape[1] - 1))
    return dsm[row, col] if not np.isnan(dsm[row, col]) else 0.0

# === UPDATE FUNCTION ===
def update(frame):
    global position, altitude, total_time, avoiding, detour, state, current_target, step_index, battery, orders, targets, best_path

    dt = time_step * speedup
    total_time += dt

    if battery <= min_battery:
        state = 'return'
        current_target = np.array([x_depot, y_depot])

    if state == 'ascend':
        altitude += v_vertical * dt
        if altitude >= max_altitude:
            altitude = max_altitude
            state = 'travel'

    elif state == 'travel':
        direction = (detour if avoiding else current_target) - position
        dist = np.linalg.norm(direction)

        if dist > 1e-2:
            step = (direction / dist) * v_horizontal * dt
            probe = position + step
            dsm_h = get_dsm_height(probe[0], probe[1])

            if dsm_h + safety_margin > altitude:
                perp = np.array([-direction[1], direction[0]])
                perp /= np.linalg.norm(perp)
                detour = position + perp * 30
                avoiding = True
            else:
                avoiding = False
                detour = None

        if dist < v_horizontal * dt:
            position = (detour if avoiding else current_target).copy()
            step_index += 1

            if step_index >= len(targets):
                orders = [generate_random_order(), generate_random_order()]
                rest = {'x': x_rest, 'y': y_rest}
                optA = [rest, orders[0], orders[1], depot]
                optB = [rest, orders[0], rest, orders[1], depot]
                best_path = optA if total_energy(optA) < total_energy(optB) else optB
                targets = best_path
                step_index = 0
                order1_dot.set_data(orders[0]['x'], orders[0]['y'])
                order2_dot.set_data(orders[1]['x'], orders[1]['y'])
                battery = 100.0
            else:
                # After reaching a customer or restaurant
                weight_remaining = sum(order['weight'] for order in targets[step_index:] if 'weight' in order)
                next_point = np.array([targets[step_index]['x'], targets[step_index]['y']])
                dist_to_next = np.linalg.norm(next_point - position)
                if battery - min_battery < get_depletion_rate(weight_remaining) * dist_to_next / v_horizontal:
                    if weight_remaining == 0:
                        current_target = np.array([x_depot, y_depot])
                        state = 'descend'
                        return drone_dot, time_text

            current_target = np.array([targets[step_index]['x'], targets[step_index]['y']])
            state = 'descend'
        else:
            position += (direction / dist) * v_horizontal * dt
            weight_remaining = sum(order['weight'] for order in targets[step_index:] if 'weight' in order)
            battery -= get_depletion_rate(weight_remaining) * dt

    elif state == 'descend':
        altitude -= v_vertical * dt
        if altitude <= 0:
            altitude = 0
            state = 'ascend'

    drone_dot.set_data(position[0], position[1])
    time_text.set_text(f"Time: {int(total_time)}s\nState: {state}\nAlt: {altitude:.1f} m\nBattery: {battery:.1f}%")
    return drone_dot, time_text

ani = FuncAnimation(fig, update, frames=10000, interval=50, blit=True)
plt.title("Drone Delivery Optimized (2 Orders, Battery & Avoidance)")
plt.xlabel("X (RD m)")
plt.ylabel("Y (RD m)")
plt.tight_layout()
plt.show()
