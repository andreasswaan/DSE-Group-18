from mission import Mission


mission = Mission(
    "DRCCRCCRCCD",
    {"R": 120, "C": 120, "D": 30},
    {"DR": 0, "RC": 2.5, "CC": 2, "CD": 0, "CR": 0},
    {"R": 7, "C": 8, "D": 6},
    {"R": 4, "C": 5, "D": 3},
    {"DR": 1, "RC": 5, "CC": 5, "CD": 5},
)
print("Mission phases:", mission.phases_str)
print(mission.phases)
print("Mission phase objects:", mission.phases_dict)
