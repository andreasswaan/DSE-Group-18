import math

# Inputs
Fexy = 0  # external force for the XY plane in N
Fez = 0  # external force for the Z axis in N
Mez = 0  # external moment for the Z axis in Nm
Mey = 0  # external moment for the Y axis in Nm
Mex = 0  # external moment for the X axis in Nm
m = 10  # drone mass in kg
zdot = 0  # Drone vertical velocity in m/s
z2dot = 0  # Drone vertical acceleration in m/s^2
Iz = 100  # moment of inertia around Z axis ...
psi2dot = 0  # angular acceleration around Z axis  rad/s^2
Rt = 2  # moment arm of the vertical thruster in m
Rdx = 1  # moment arm of the Drag for the X axis in  m
rho = 1.2  # air density in kg/m^3
Dv = 0.5  # vertical propeller diameter in m
Dh = 0.5  # horizontal propeller diameter in m
Ct = 0.05  # propeller efficiency constant
Cq = 0.01  # aerodynamic drag for blade rotation constant
Cd = 0.5  # drag coefficient for the drone
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
    Rt,
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
    A = (m * g + Fez - Kd * zdot * abs(zdot) - m * z2dot) / (2 * Kt)
    B = (Iz * psi2dot - Mez) / (2 * Ktau)
    C = -(Mey + Kd * zdot * abs(zdot) * Rdx) / (math.sqrt(2) * Rt * Kt)
    D = -(Mex) / (math.sqrt(2) * Rt * Kt)
    E = math.sqrt(Fexy / Kth)
    if choice == 1:
        w1 = math.sqrt((A + B + C + D) / 4)
        w2 = w1
        w3 = math.sqrt((A - B + C - D) / 4)
        w4 = w3
        w5 = math.sqrt((A - B - C + D) / 4)
        w6 = w5
        w7 = math.sqrt((A + B - C - D) / 4)
        w8 = w7
        wh = E
    if choice == 2:
        w1 = 0
        w2 = w1
        w3 = math.sqrt((A + C) / 2)
        w4 = w3
        w5 = math.sqrt((A + D) / 2)
        w6 = w5
        w7 = math.sqrt(-(C + D) / 2)
        w8 = w7
        wh = E
    if choice == 3:
        w1 = math.sqrt((A + C) / 2)
        w2 = w1
        w3 = 0
        w4 = w3
        w5 = math.sqrt((D - C) / 2)
        w6 = w5
        w7 = math.sqrt((A - D) / 2)
        w8 = w7
        wh = E
    if choice == 4:
        w1 = math.sqrt((A + D) / 2)
        w2 = w1
        w3 = math.sqrt((C - D) / 2)
        w4 = w3
        w5 = 0
        w6 = w5
        w7 = math.sqrt((A - C) / 2)
        w8 = w7
        wh = E
    if choice == 5:
        w1 = math.sqrt((C + D) / 2)
        w2 = w1
        w3 = math.sqrt((A - D) / 2)
        w4 = w3
        w5 = math.sqrt((A - C) / 2)
        w6 = w5
        w7 = 0
        w8 = w7
        wh = E
    return (
        math.sqrt(2) * w1 / (2 * math.pi / 60),
        w3 * math.sqrt(2) / (2 * math.pi / 60),
        w5 * math.sqrt(2) / (2 * math.pi / 60),
        w7 * math.sqrt(2) / (2 * math.pi / 60),
        wh / (2 * math.pi / 60),
    )


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
        Rt,
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
