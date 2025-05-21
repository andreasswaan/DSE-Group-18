import os
import numpy as np
import matplotlib.pyplot as plt
from vert_and_hori import iterations, g
from city_density import get_distance_constants


def build_mission_profile(
    sequence,
    distances,
    loitering_times,
    cruise_h,
    cruise_speed,
    payload_masses,
    TO_speed,
    LD_speed,
):
    """
    sequence: str, e.g. "DRCCD"
    distances: dict, e.g. {"DR": 5000, "RC": 5000, "CC": 5000, "CD": 5000}
    loitering_times: list of loitering times per segment (seconds)
    cruise_h: cruise height (m) or list per segment
    cruise_speed: cruise speed (m/s) or list per segment
    payload_masses: list of payload mass per segment (kg)
    TO_speed: takeoff speed (m/s)
    LD_speed: landing speed (m/s)
    """
    profile = []
    for i in range(len(sequence)):
        if i == len(sequence) - 1:
            continue
        TO_letter = sequence[i]
        LD_letter = sequence[i + 1]
        leg_letter = TO_letter + LD_letter

        print(f"leg: {leg_letter}, TO: {TO_letter}, LD: {LD_letter}")

        cruise_distance = distances.get(leg_letter, 0)
        loitering_time = (
            loitering_times[i] if isinstance(loitering_times, list) else loitering_times
        )
        h = cruise_h[i] if isinstance(cruise_h, list) else cruise_h
        speed = cruise_speed[i] if isinstance(cruise_speed, list) else cruise_speed
        payload = (
            payload_masses[i] if isinstance(payload_masses, list) else payload_masses
        )
        profile.append(
            {
                "cruise_distance": cruise_distance * 1000,
                "loitering_time": loitering_time[LD_letter],
                "cruise_h": h,
                "cruise_speed": speed,
                "payload_mass": payload[leg_letter],
                "TO_speed": TO_speed[TO_letter],
                "LD_speed": LD_speed[LD_letter],
            }
        )
    return profile


sequence = "DRCCRCCRCCD"
# distances = {"DR": 5000, "RC": 5000, "CC": 5000, "CD": 5000}
distances = get_distance_constants()["Center"]
loitering_times = {"R": 120, "C": 120, "D": 30}
cruise_h = 200
cruise_speed = 15
payload_masses = {"DR": 0, "RC": 2.5, "CC": 2, "CD": 0, "CR": 0}
TO_speed = {"D": 6, "R": 7, "C": 8}
LD_speed = {"D": 3, "R": 4, "C": 5}
pct_wing_lift_array = np.linspace(0.5, 1.0, 6)
# pct_wing_lift_array = [1]

# print("pct_wing_lift_array:", pct_wing_lift_array)

mission_profile_2 = build_mission_profile(
    sequence,
    distances,
    loitering_times,
    cruise_h,
    cruise_speed,
    payload_masses,
    TO_speed,
    LD_speed,
)

print("Mission Profile 2:")
print(mission_profile_2)
mtow_histories = []

for pct_wing_lift in pct_wing_lift_array:
    print(f"pct_wing_lift: {pct_wing_lift}")
    mtows = iterations(mission_profile_2, pct_wing_lift, PLOT=False)
    mtow_histories.append(mtows)

# Plot MTOW convergence for each pct_wing_lift
plt.figure(figsize=(10, 6))
for i, pct_wing_lift in enumerate(pct_wing_lift_array):
    mtows = np.array(mtow_histories[i]) / g  # Divide all MTOW by g before plotting
    plt.plot(range(1, len(mtows) + 1), mtows, label=f"{pct_wing_lift:.1f}")

plt.xticks(range(1, len(mtows) + 1), range(1, len(mtows) + 1))
plt.xlabel("Iteration [-]")
plt.ylabel("MTOW [kg]")
plt.title("MTOW Convergence for Different frac_wing_lift")
plt.legend(title="frac_wing_lift")
plt.grid(True, linestyle="--", alpha=0.7)
plt.tight_layout()
plot_folder = "src/initial_sizing/plots"
os.makedirs(plot_folder, exist_ok=True)
plt.savefig(os.path.join(plot_folder, "mtow_convergence.svg"), format="svg", dpi=300)
plt.close()
