from prelim_des.drone import Drone
from prelim_des.mission import Mission, MissionPhase
import yaml

def create_mission():
    return Mission([
        MissionPhase("Takeoff", 60, 0, 0, "hover"),
        MissionPhase("Climb", 120, 5, 100, "transition"),
        MissionPhase("Cruise", 600, 15, 100, "cruise"),
        MissionPhase("Descent", 120, 5, 0, "transition"),
        MissionPhase("Landing", 60, 0, 0, "hover"),
    ])

def load_config():
    with open("config/baseline_config.yaml") as f:
        return yaml.safe_load(f)

def main():
    mission = create_mission()
    config = load_config()
    drone = Drone(config, mission)

    drone.mission_sizing()

    print("=== Subsystem Weights ===")
    weights = drone.class_2_weight_estimate()
    for name, w in weights.items():
        print(f"{name}: {w:.2f} kg")

if __name__ == "__main__":
    main()