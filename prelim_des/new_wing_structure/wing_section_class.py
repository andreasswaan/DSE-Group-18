from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from prelim_des.drone import Drone
from prelim_des.new_wing_structure.wing_boom_class import WingBoom


class WingSection:
    y_pos: float
    "distance must be in m"
    moment_x: float
    "applied moment must be in N/m"
    moment_z: float
    "applied moment must be in N/m"
    shear_x: float
    "applied shear must be in N"
    shear_z: float
    "applied shear must be in N"
    normal_force: float
    "applied force must be in N"
    torsion: float
    "applied torsion must be in Nm"
    booms: list[WingBoom]

    analysis_moment_x: float
    analysis_moment_z: float
    analysis_shear_x: float
    analysis_shear_z: float
    analysis_normal_force: float

    def __init__(self, drone, name: str = None, length=0):
        self.drone = drone
        self.booms = []
        self.moment_x = 0.0
        self.moment_z = 0.0
        self.shear_x = 0.0
        self.shear_z = 0.0
        self.torsion = 0.0
        self.normal_force = 0.0
        self.analysis_moment_x = 0.0
        self.analysis_moment_z = 0.0
        self.analysis_shear_x = 0.0
        self.analysis_shear_z = 0.0
        self.analysis_normal_force = 0.0
        self.name = name
        self.length = length
        self.top_flange_ids = []
        self.right_flange_ids = []
        self.bottom_flange_ids = []
        self.left_flange_ids = []

        pass

    @property
    def Ixx(self):
        Ixx = 0
        for _, boom in enumerate(self.booms):
            Ixx += boom.z_pos**2 * boom.area
        return Ixx

    @property
    def Izz(self):
        Izz = 0
        for _, boom in enumerate(self.booms):
            Izz += boom.x_pos**2 * boom.area
        return Izz

    @property
    def Ixz(self):
        Ixz = 0
        for _, boom in enumerate(self.booms):
            Ixz += boom.z_pos * boom.x_pos * boom.area
        return Ixz

    @property
    def weight(self):
        weight = 0
        for _, boom in enumerate(self.booms):
            weight += boom.weight(self.length)
        return weight

    def add_boom(self, boom: WingBoom):
        boom.section = self
        self.booms.append(boom)


if __name__ == "__main__":
    from prelim_des.drone import Drone
    drone = Drone()
    section = WingSection(drone)
    boom_1 = WingBoom(-1.000, -1.732, 0.002863)
    boom_2 = WingBoom(-0.347, -1.970, 0.002084)
    boom_3 = WingBoom(0.347, -1.970, 0.002084)
    boom_4 = WingBoom(1.000, -1.732, 0.002863)

    boom_5 = WingBoom(-1.000, 1.732, 0.002863)
    boom_6 = WingBoom(-0.347, 1.970, 0.002084)
    boom_7 = WingBoom(0.347, 1.970, 0.002084)
    boom_8 = WingBoom(1.000, 1.732, 0.002863)
    booms = [boom_1, boom_2, boom_3, boom_4, boom_5, boom_6, boom_7, boom_8]
    for _, boom in enumerate(booms):
        print(boom)
        section.add_boom(boom)
    section.moment_x = 800000
    section.shear_z = 150000
    print(section.Ixx)
    for _, boom in enumerate(section.booms):
        print(boom.shear_flow_delta)
