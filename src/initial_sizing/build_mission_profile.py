from vert_and_hori import iterations, design_payload

def build_mission_profile(
    sequence,
    distances,
    loitering_times,
    cruise_h,
    cruise_speed,
    payload_masses,
    TO_speed,
    LD_speed
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
    for i, seg in enumerate(sequence):
        key = None
        if seg == "D":
            # Depot, usually no cruise, just loiter or ground
            continue
        elif seg == "R":
            key = "DR"
        elif seg == "C":
            key = "RC"
        else:
            key = seg  # fallback

        cruise_distance = distances.get(key, 0)
        loitering_time = loitering_times[i] if isinstance(loitering_times, list) else loitering_times
        h = cruise_h[i] if isinstance(cruise_h, list) else cruise_h
        speed = cruise_speed[i] if isinstance(cruise_speed, list) else cruise_speed
        payload = payload_masses[i] if isinstance(payload_masses, list) else payload_masses

        profile.append({
            "cruise_distance": cruise_distance,
            "loitering_time": loitering_time,
            "cruise_h": h,
            "cruise_speed": speed,
            "payload_mass": payload,
            "TO_speed": TO_speed,
            "LD_speed": LD_speed,
        })
    return profile


sequence = "DRCCD"
distances = {"DR": 5000, "RC": 5000, "CC": 5000, "CD": 5000}
loitering_times = {"R": 120, "C": 30, "D": 0}
cruise_h = 200
cruise_speed = 15
payload_masses = {"D": 0, "R": 2.5, "C": 0}
TO_speed = {"D": 6, "R": 7, "C": 8}
LD_speed = {"D": 3, "R": 4, "C": 5}

mission_profile_2 = build_mission_profile(
    sequence, distances, loitering_times, cruise_h, cruise_speed, payload_masses, TO_speed, LD_speed
)

print("Mission Profile 2:")
print(mission_profile_2)

iterations(design_payload, mission_profile_2)
