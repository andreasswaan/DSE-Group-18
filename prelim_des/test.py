from mission import Mission


mission = Mission("DRCCRCCRCCD")
print("Mission phases:", mission.phases_str)
print(mission.phases_obj())
print("Mission phase objects:", mission.phases_dict)
