import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from globals import main_dir
from scipy.optimize import curve_fit


# Define S-curve (sigmoid) function
def s_curve(t, a, b, c, d):
    return a / (1 + np.exp(-b * (t - c))) + d


# Simulation with thrusts as inputs (for optimization)
def simulate_transition(
    thrusts_h,
    thrusts_v,
    target_altitude=40,
    max_pitch_deg=18,
    T_end=20,
    dt=0.01,
    return_all=False,
    silent=False,
):
    g = 9.81
    rho = 1.225
    mass = 10.0
    wing_area = 1.5
    CL0 = 0.3
    CL_alpha = 5.7
    CD0 = 0.02
    k = 0.03
    stall_speed = 11.0
    eta = 0.8
    rotor_area = 0.2

    time = np.arange(0, T_end + dt, dt)
    # node_times = np.linspace(0, T_end, 6)
    node_times_h = np.linspace(0, T_end, len(thrusts_h))
    node_times_v = np.linspace(0, T_end, len(thrusts_v))

    vx = np.zeros_like(time)
    vz = np.zeros_like(time)
    alt = np.zeros_like(time)
    pitch = np.clip(
        np.linspace(0, np.deg2rad(max_pitch_deg), len(time)),
        0,
        np.deg2rad(max_pitch_deg),
    )
    x = np.zeros_like(time)
    ngsy = np.zeros_like(time)

    # h_nodes = thrusts[:6]
    # v_nodes = thrusts[6:]
    h_nodes = thrusts_h
    v_nodes = thrusts_v
    print(h_nodes.shape, v_nodes.shape, node_times.shape)
    h_interp = interp1d(node_times_h, h_nodes, kind="linear", fill_value="extrapolate")
    v_interp = interp1d(node_times_v, v_nodes, kind="linear", fill_value="extrapolate")

    horizontal_thrust = h_interp(time)
    vertical_thrust = v_interp(time)

    energy_vertical = 0.0
    energy_horizontal = 0.0

    reached_target_altitude = False
    max_speed = 18
    constraint_violations = []

    for i in range(1, len(time)):
        if not reached_target_altitude and alt[i - 1] >= target_altitude:
            reached_target_altitude = True
            vertical_thrust[i:] = 0

        if reached_target_altitude and abs(vz[i - 1]) < 0.1:
            time = time[: i + 1]
            vx = vx[: i + 1]
            vz = vz[: i + 1]
            alt = alt[: i + 1]
            x = x[: i + 1]
            pitch = pitch[: i + 1]
            ngsy = ngsy[: i + 1]
            horizontal_thrust = horizontal_thrust[: i + 1]
            vertical_thrust = vertical_thrust[: i + 1]
            break

        flight_path_angle = np.arctan2(vz[i - 1], vx[i - 1] + 1e-4)
        alpha = np.clip(pitch[i] - flight_path_angle, np.deg2rad(-5), np.deg2rad(15))

        CL = CL0 + CL_alpha * alpha
        CD = CD0 + k * CL**2

        speed = np.sqrt(vx[i - 1] ** 2 + vz[i - 1] ** 2)
        lift = 0.5 * rho * speed**2 * wing_area * CL
        drag = 0.5 * rho * speed**2 * wing_area * CD

        Tv = vertical_thrust[i]
        Th = horizontal_thrust[i]

        ngsy[i - 1] = (lift * np.cos(alpha) - drag * np.sin(alpha) + Tv) / (
            mass * g * np.cos(pitch[i])
        )

        Fx = (
            -Tv * np.sin(pitch[i])
            + Th * np.cos(pitch[i])
            - drag * np.cos(flight_path_angle)
            - lift * np.sin(flight_path_angle)
        )
        Fz = (
            Th * np.sin(pitch[i])
            + Tv * np.cos(pitch[i])
            - mass * g
            + lift * np.cos(flight_path_angle)
            - drag * np.sin(flight_path_angle)
        )

        az = Fz / mass
        vz[i] = vz[i - 1] + az * dt
        alt[i] = alt[i - 1] + vz[i] * dt

        ax = Fx / mass
        vx[i] = vx[i - 1] + ax * dt
        x[i] = x[i - 1] + vx[i] * dt

        if vz[i] < -0.01:
            constraint_violations.append(("vz_neg", time[i]))
        if vx[i] > max_speed:
            constraint_violations.append(("vx_exceed", time[i]))

        vx[i] = np.clip(vx[i], -max_speed, max_speed)
        vz[i] = np.clip(vz[i], -max_speed, max_speed)

        if Tv > 0:
            # v_induced = np.sqrt(Tv / (2 * rho * rotor_area))
            # P_vert = (Tv * v_induced) / eta
            P_vert = 40.518 * Tv - 645.22

        else:
            P_vert = 0

        v_eff_horizontal = abs(vx[i])
        P_horiz = (Th * v_eff_horizontal) / eta

        energy_vertical += P_vert * dt
        energy_horizontal += P_horiz * dt

    E_total = energy_vertical + energy_horizontal

    print(f"\n🔋 Estimated Energy Consumption:")
    print(f" - Vertical System: {energy_vertical / 1000:.4f} kJ")
    print(f" - Horizontal System: {energy_horizontal / 1000:.4f} kJ")
    print(f" - Total Energy: {E_total / 1000:.2f} kJ")

    alt = alt + 20


initial_thrusts_hor = [0, 25, 26, 28, 31, 36]
initial_thrusts_ver = [100, 95, 20, 0, 0, 0]
T_end = 20
node_times = np.linspace(0, T_end, len(initial_thrusts_ver))
node_times_hor = np.linspace(0, T_end, len(initial_thrusts_hor))

# Fit S-curve to vertical thrust nodes
popt, _ = curve_fit(s_curve, node_times, initial_thrusts_ver, p0=[100, -0.5, 10, 0])
popt_hor, _ = curve_fit(
    s_curve,
    node_times,
    initial_thrusts_hor,
    p0=[100, -0.5, 10, 0],
    bounds=([0, -np.inf, 0, -np.inf], [np.inf, np.inf, np.inf, np.inf]),
)

# Generate smooth time for plotting
t_smooth = np.linspace(0, T_end, 200)
vthrust_smooth = s_curve(t_smooth, *popt)

# Use the fitted S-curve to generate a smooth vertical thrust profile for all t_smooth points
vertical_thrust_profile = s_curve(t_smooth, *popt)  # smooth profile for all points
# Interpolate horizontal thrusts to match t_smooth
h_interp = interp1d(
    node_times_hor, initial_thrusts_hor, kind="linear", fill_value="extrapolate"
)
horizontal_thrust_profile = h_interp(t_smooth)

# Combine for simulate_transition: [h_profile] + [v_profile]
# smooth_thrusts = list(horizontal_thrust_profile) + list(vertical_thrust_profile)

# Simulate transition with the smooth thrust profile (all points used)
simulate_transition(
    horizontal_thrust_profile, vertical_thrust_profile, silent=False, T_end=T_end
)


data_S_curve = np.load(os.path.join("S-curve_transition_data.npz"))
time = data_S_curve["time"]
alt = data_S_curve["alt"]
vx = data_S_curve["vx"]
vz = data_S_curve["vz"]

data_6_nodes = np.load(os.path.join("notes", "6-nodes_transition_data.npz"))
time_6 = data_6_nodes["time"]
alt_6 = data_6_nodes["alt"]
vx_6 = data_6_nodes["vx"]
vz_6 = data_6_nodes["vz"]

plt.figure(figsize=(12, 10))

# Define consistent colors
S_color =  '#1f77b4'
L_color =  '#ff7f0e' 
stall_color = "#2c0404"

plt.subplot(2, 1, 1)
plt.plot(time, alt, label="Altitude (Continuous thrust profile)", color=S_color)
plt.plot(
    time_6,
    alt_6,
    label="Altitude (piecewise linear thrust profile)",
    color=L_color,
    linestyle="--",
)
plt.ylabel("Altitude [m]", fontsize=16, labelpad=10)
plt.tick_params(labelsize=14)
plt.grid()
plt.legend(fontsize=13)

plt.subplot(2, 1, 2)
plt.plot(time, vx, label="Vx (Continuous thrust profile)", color=S_color)
plt.plot(
    time_6,
    vx_6,
    label="Vx (piecewise linear thrust profile)",
    color=L_color,
    linestyle="--",
)
plt.plot(time, vz, label="Vz (Continuous thrust profile)", color=S_color)
plt.plot(
    time_6,
    vz_6,
    label="Vz (piecewise linear thrust profile)",
    color=L_color,
    linestyle="--",
)
plt.axhline(11, color=stall_color, linestyle=":", label="Stall Speed")
plt.ylabel("Velocity [m/s]", fontsize=16, labelpad=10)
plt.xlabel("Time [s]", fontsize=16, labelpad=10)
plt.tick_params(labelsize=15)
plt.grid()
plt.legend(fontsize=13, loc="upper left")
plt.tight_layout()
plt.savefig(
    os.path.join(main_dir, "prelim_des", "plots", "transition_comparison.pdf"), dpi=300, bbox_inches='tight', format='pdf'
)
plt.show()
