
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rasterio
from pyproj import Transformer

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

# === DRONE SETUP ===
speedup = 2
time_step = 1
v_vertical = 5.0
v_horizontal = 10.0
max_altitude = 10.0
safety_margin = 3.0  # meters above DSM surface
altitude = 0.0
state = 'ascend'
total_time = 0
avoiding = False
detour = None

# Set origin (restaurant) near DSM center
x_rest = (x_min + x_max) / 2
y_rest = (y_min + y_max) / 2
position = np.array([x_rest, y_rest])

# Generate initial target
rng = np.random.default_rng()
def new_random_target():
    angle = rng.uniform(0, 2 * np.pi)
    r = rng.uniform(200, 500)
    dx, dy = r * np.cos(angle), r * np.sin(angle)
    return np.array([x_rest + dx, y_rest + dy])

target = new_random_target()

# === PLOT SETUP ===
fig, ax = plt.subplots(figsize=(10, 10))
im = ax.imshow(dsm, cmap='terrain', extent=extent, origin='upper', vmin=0, vmax=30, zorder=0)
cbar = plt.colorbar(im, ax=ax, label='Elevation (m)')  # Add colorbar for elevation

drone_dot, = ax.plot([], [], 'ro', markersize=6, label="Drone")
rest_dot, = ax.plot(x_rest, y_rest, 'bo', markersize=6, label="Restaurant")
target_dot, = ax.plot([], [], 'yo', markersize=6, label="Customer")
time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)

ax.set_xlim(x_min, x_max)
ax.set_ylim(y_min, y_max)
ax.legend()

def get_dsm_height(x, y):
    col, row = ~transform * (x, y)
    row = int(np.clip(row, 0, dsm.shape[0] - 1))
    col = int(np.clip(col, 0, dsm.shape[1] - 1))
    return dsm[row, col] if not np.isnan(dsm[row, col]) else 0.0

# === ANIMATION LOOP ===
def update(frame):
    global position, altitude, target, state, total_time, avoiding, detour

    dt = time_step * speedup
    total_time += dt

    if state == 'ascend':
        altitude += v_vertical * dt
        if altitude >= max_altitude:
            altitude = max_altitude
            state = 'travel'

    elif state == 'travel':
        direction = (detour if avoiding else target) - position
        dist = np.linalg.norm(direction)

        # Look ahead
        if dist > 1e-2:
            step = (direction / dist) * v_horizontal * dt
            probe = position + step
            dsm_h = get_dsm_height(probe[0], probe[1])

            if dsm_h + safety_margin > altitude:
                # Avoid left (perpendicular vector)
                perp = np.array([-direction[1], direction[0]])
                perp /= np.linalg.norm(perp)
                detour = position + perp * 30
                avoiding = True
            else:
                avoiding = False
                detour = None

        if dist < v_horizontal * dt:
            position = (detour if avoiding else target).copy()
            state = 'descend'
        else:
            position += (direction / dist) * v_horizontal * dt

    elif state == 'descend':
        altitude -= v_vertical * dt
        if altitude <= 0:
            altitude = 0
            state = 'return'

    elif state == 'return':
        direction = np.array([x_rest, y_rest]) - position
        dist = np.linalg.norm(direction)
        if dist < v_horizontal * dt:
            position = np.array([x_rest, y_rest])
            state = 'ascend'
            target[:] = new_random_target()
        else:
            position += (direction / dist) * v_horizontal * dt

    drone_dot.set_data(position[0], position[1])
    target_dot.set_data(target[0], target[1])
    time_text.set_text(f"Time: {int(total_time)}s\nState: {state}\nAlt: {altitude:.1f} m")
    return drone_dot, target_dot, time_text

ani = FuncAnimation(fig, update, frames=1000, interval=50, blit=True)
plt.title("Drone Simulation With Basic Obstacle Avoidance (Left Step)")
plt.xlabel("X (RD m)")
plt.ylabel("Y (RD m)")
plt.tight_layout()
plt.show()
