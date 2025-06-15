import math
import numpy as np

# Earlier inputs


# Inputs
Fexy = 0  # external force for the XY plane in N
Fez = 0  # external force for the Z axis in N
Mez = 0  # external moment for the Z axis in Nm
Mey = 0  # external moment for the Y axis in Nm
Mex = 0  # external moment for the X axis in Nm
m = 10  # drone mass in kg
zdot = 3  # Drone vertical velocity in m/s
z2dot = 1  # Drone vertical acceleration in m/s^2
Iz = 100  # moment of inertia around Z axis ...
psi2dot = 0.2  # angular acceleration around Z axis  rad/s^2 !!!
Rt_fx = 2  # moment arm of the vertical thruster in m
Rt_fy = 2  # moment arm of the vertical thruster in m
Rt_bx = 2  # moment arm of the vertical thruster in m
Rt_by = 2  # moment arm of the vertical thruster in m
Rdx = 1  # moment arm of the Drag for the X axis in  m
rho = 1.2  # air density in kg/m^3
Dv = 0.3  # vertical propeller diameter in m
Dh = 0.3  # horizontal propeller diameter in m
Ct = 0.11  # propeller efficiency constant 0.11-0.15
Cq = 0.03  # aerodynamic drag for blade rotation constant 0.03
Cd = 1  # drag coefficient for the drone
S = 10  # area of the drone in the XY plane in m^2
g = 9.81
max_rpm = 6000


choice = int(
    input(
        "What stability analysis case?:\n 1. All engines \n 2.Engine 1-2 failure \n 3. Engine 3-4 failure \n 4.Engine 5-6 failure \n 5.Engine 7-8 failure"
    )
)


# Fuction
def vertical_stability_state(
    Fexy,
    Fez,
    Mez,
    Mey,
    Mex,
    m,
    zdot,
    z2dot,
    Iz,
    psi2dot,
    Rt_fx,
    Rt_fy,
    Rt_bx,
    Rt_by,
    Rdx,
    rho,
    Dv,
    Dh,
    Ct,
    Cq,
    Cd,
    S,
    g,
    choice,
):
    Kth = (Ct * rho * Dh**4) / (4 * (math.pi) ** 2)
    Kt = (Ct * rho * Dv**4) / (4 * (math.pi) ** 2)
    Kd = 0.5 * Cd * rho * S
    Ktau = (Cq * rho * Dv**5) / (4 * (math.pi) ** 2)
    A = (m * g + Fez - Kd * zdot * abs(zdot) - m * z2dot) / (Kt)
    B = (Iz * psi2dot - Mez) / (Ktau)
    C = -(Mey + Kd * zdot * abs(zdot) * Rdx) / (Kt)
    D = -(Mex) / (Kt)
    E = math.sqrt(Fexy / Kth)
    if choice == 1:
        M_1 = np.array(
            [
                [1, 1, 1, 1],
                [1, -1, -1, 1],
                [Rt_fx, Rt_fx, -Rt_bx, -Rt_bx],
                [Rt_fy, -Rt_fy, Rt_by, -Rt_by],
            ]
        )
        # print("M_1:", M_1)
        M_3 = np.array([[A], [B], [C], [D]])
        # print("M_3:", M_3)
        M_1_reversed = np.linalg.pinv(M_1)
        # print("M_1_reversed:", M_1_reversed)
        M_2 = M_1_reversed @ M_3
        # print("M_2:", M_2)
        w1 = M_2[0][0]
        w3 = M_2[1][0]
        w5 = M_2[2][0]
        w7 = M_2[3][0]
    if choice == 2:
        M_1 = np.array([[1, 1, 1], [Rt_fx, -Rt_bx, -Rt_bx], [-Rt_fy, Rt_by, -Rt_by]])
        M_3 = np.array([[A], [C], [D]])
        M_2 = np.linalg.pinv(M_1) @ M_3
        w1 = 0
        w3 = M_2[0][0]
        w5 = M_2[1][0]
        w7 = M_2[2][0]
    if choice == 3:
        M_1 = np.array([[1, 1, 1], [Rt_fx, -Rt_bx, -Rt_bx], [Rt_fy, Rt_by, -Rt_by]])
        M_3 = np.array([[A], [C], [D]])
        M_2 = np.linalg.pinv(M_1) @ M_3
        w1 = M_2[0][0]
        w3 = 0
        w5 = M_2[1][0]
        w7 = M_2[2][0]
    if choice == 4:
        M_1 = np.array([[1, 1, 1], [Rt_fx, Rt_fx, -Rt_bx], [Rt_fy, -Rt_fy, -Rt_by]])
        M_3 = np.array([[A], [C], [D]])
        M_2 = np.linalg.pinv(M_1) @ M_3
        w1 = M_2[0][0]
        w3 = M_2[1][0]
        w5 = 0
        w7 = M_2[2][0]
    if choice == 5:
        M_1 = np.array([[1, 1, 1], [Rt_fx, Rt_fx, -Rt_bx], [Rt_fy, -Rt_fy, Rt_by]])
        M_3 = np.array([[A], [C], [D]])
        M_2 = np.linalg.pinv(M_1) @ M_3
        w1 = M_2[0][0]
        w3 = M_2[1][0]
        w5 = M_2[2][0]
        w7 = 0
    wh = E
    if w1 >= 0:
        w1 = math.sqrt(w1) * 2 * math.pi / 60
    else:
        w1 = -math.sqrt(-w1) * 2 * math.pi / 60
    if w3 >= 0:
        w3 = math.sqrt(w3) * 2 * math.pi / 60
    else:
        w3 = -math.sqrt(-w3) * 2 * math.pi / 60
    if w5 >= 0:
        w5 = math.sqrt(w5) * 2 * math.pi / 60
    else:
        w5 = -math.sqrt(-w5) * 2 * math.pi / 60
    if w7 >= 0:
        w7 = math.sqrt(w7) * 2 * math.pi / 60
    else:
        w7 = -math.sqrt(-w7) * 2 * math.pi / 60
    wh = wh * 2 * math.pi / 60
    return (w1, w3, w5, w7, wh)


print(
    vertical_stability_state(
        Fexy,
        Fez,
        Mez,
        Mey,
        Mex,
        m,
        zdot,
        z2dot,
        Iz,
        psi2dot,
        Rt_fx,
        Rt_fy,
        Rt_bx,
        Rt_by,
        Rdx,
        rho,
        Dv,
        Dh,
        Ct,
        Cq,
        Cd,
        S,
        g,
        choice,
    )
)
