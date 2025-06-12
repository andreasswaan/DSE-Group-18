from __future__ import annotations
from typing import TYPE_CHECKING
import math
import matplotlib.pyplot as plt
import numpy as np
import tomllib

from prelim_des.internals import layout_var
from prelim_des.internals import layout_const
from prelim_des.idealized_structure import get_fuselage_dimensions
from prelim_des.internals import get_cg_groups


if TYPE_CHECKING:
    from prelim_des.drone import Drone


# from prelim_des.elems import Wing, Fuselage, LandingGear
# from prelim_des.drone import Drone

with open("prelim_des/config.toml", "rb") as f:
    data = tomllib.load(f)


def item_input_sort():
    items = list(layout_var.items())
    sorted_items = sorted(items, key=lambda item: item[1]["x"])
    W_list = [item[1]["m"] for item in sorted_items]
    W_list = [p * 9.81 for p in W_list]
    X_list = [item[1]["x"] for item in sorted_items]
    X_list = [p / 1000 for p in X_list]
    X_reverse = X_list[::-1]
    W_reverse = W_list[::-1]
    return X_list, W_list, X_reverse, W_reverse


# MAIN
def main_horizontal_stability(
    drone: Drone,
    graph=False,
):
    X_fuselage = (
        get_cg_groups(layout_const, drone.fuselage, drone.wing, drone.tail)["fuselage"][
            "x"
        ]
    ) / 1000
    Cl_alpha_h = data["config"]["horizontal_sc"]["Cl_alpha_h"]  # tail Cl alpha
    Cm_ac = data["config"]["horizontal_sc"]["Cm_ac"]  # ac moment constant
    Cl_h = data["config"]["horizontal_sc"]["Cl_h"]  # tail cl
    Cl_alpha_tailless = drone.aero.cl_alpha_slope()
    Cl_tailless = drone.aero.cl_alpha(drone.aero.AOA_cruise)

    Dxw = drone.wing.c_root / 2  # m , from LEMAC to wing CG
    drone_thickness = get_fuselage_dimensions(case=2)[1]

    W_fuselage = drone.fuselage.weight  # kg, fuselage weight
    W_wing = drone.wing.weight  # kg, wing weight
    S_M = data["config"]["horizontal_sc"]["S_M"]  # safety margin
    LEMAC_In = data["config"]["horizontal_sc"][
        "LEMAC_In"
    ]  # m, front limit of wing positioning
    LEMAC_Out = data["config"]["horizontal_sc"][
        "LEMAC_Out"
    ]  # m, back limit of wing positioning
    Vh_V = data["config"]["horizontal_sc"]["Vh_V"]
    taper_ratio_tail = data["config"]["horizontal_sc"][
        "taper_ratio_tail"
    ]  # horizontal tail
    Aspect_ratio_tail = data["config"]["horizontal_sc"][
        "Aspect_ratio_tail"
    ]  # horizontal tail
    d_e_d_alpha = data["config"]["horizontal_sc"][
        "d_e_d_alpha"
    ]  # downwash effect something

    V_g = data["config"]["horizontal_sc"]["V_g"]
    V = data["config"]["mission"]["cruise_speed"]

    drone_thickness = data["config"]["mission"]["cruise_speed"]

    mac = drone.wing.mac
    L_fuselage = drone.fuselage.length  # m, fuselage length
    X_ac = 0.25 * mac
    Lh = L_fuselage
    S_wing = drone.wing.S  # m^2, wing area

    LEMAC_In = LEMAC_In * L_fuselage
    LEMAC_Out = LEMAC_Out * L_fuselage

    X_list, W_list, X_reverse, W_reverse = item_input_sort()

    x_cg_start, x_cg_end, wing_position, tail_area_ratio = find_wing_position(
        W_fuselage,
        X_fuselage,
        W_wing,
        L_fuselage,
        mac,
        Dxw,
        LEMAC_In,
        LEMAC_Out,
        S_M,
        X_ac,
        Cl_alpha_h,
        Cl_alpha_tailless,
        d_e_d_alpha,
        Lh,
        Vh_V,
        Cm_ac,
        Cl_h,
        Cl_tailless,
        X_list,
        W_list,
        X_reverse,
        W_reverse,
    )

    W_EOM, X_EOM = cg_shift(W_fuselage, X_fuselage, W_wing, (wing_position + Dxw))
    X_cg = final_X_cg(W_EOM, X_EOM, W_list, X_list)

    # print("most infront x_cg:",x_cg_start,"\n","most back x_cg:",x_cg_end,"\n","wing position:",wing_position,"\n","tail_area_ratio:",tail_area_ratio,)
    b_h, c_h_small, c_h_big = horizontal_tail_area_sizing(
        tail_area_ratio, S_wing, taper_ratio_tail, Aspect_ratio_tail
    )

    # Extra initial values

    X_LEMAC = wing_position
    X_wing = X_LEMAC + Dxw  # m

    b_v, c_v_small, c_v_big = vertical_tail_sizing(
        L_fuselage, V_g, X_cg, V, Aspect_ratio_tail, taper_ratio_tail, drone_thickness
    )

    if graph:
        main_cg_range_with_graph(
            W_fuselage,
            X_fuselage,
            W_wing,
            X_wing,
            X_LEMAC,
            L_fuselage,
            mac,
            X_list,
            W_list,
            X_reverse,
            W_reverse,
        )
        main_cg_range_with_wing_variation_with_plot(
            W_fuselage,
            X_fuselage,
            W_wing,
            L_fuselage,
            mac,
            Dxw,
            LEMAC_In,
            LEMAC_Out,
            X_list,
            W_list,
            X_reverse,
            W_reverse,
        )

        y_tail, x_stab, x_control = stab_cont_lines(
            S_M,
            mac,
            X_ac,
            Cl_alpha_h,
            Cl_alpha_tailless,
            d_e_d_alpha,
            Lh,
            Vh_V,
            Cm_ac,
            L_fuselage,
            Cl_h,
            Cl_tailless,
            wing_position,
        )
        stab_cont_lines_plot(
            y_tail, x_stab, x_control, x_cg_start, x_cg_end, tail_area_ratio
        )
    return (
        x_cg_start,
        x_cg_end,
        wing_position,
        b_h,
        c_h_small,
        c_h_big,
        b_v,
        c_v_small,
        c_v_big,
    )


# CG range graph
def cg_shift(W_old, X_old, W_item, X_item):
    return W_old + W_item, (W_old * X_old + W_item * X_item) / (W_old + W_item)


def vertical_tail_sizing(
    L_fuselage, V_g, X_cg, V, aspect_ratio, v_taper_ratio, drone_thickness
):
    S_lat = drone_thickness * L_fuselage
    S = (
        (2 * X_cg - L_fuselage)
        * V_g**2
        * S_lat
        / (math.pi * V**2 * (L_fuselage - X_cg))
    ) / aspect_ratio
    b = math.sqrt(S * aspect_ratio)
    c_small = 2 * v_taper_ratio / (1 + v_taper_ratio) * math.sqrt(S / aspect_ratio)
    c_big = 2 / (1 + v_taper_ratio) * math.sqrt(S / aspect_ratio)
    if b <= 0 or c_small <= 0 or c_big <= 0:
        b = 0
        c_small = 0
        c_big = 0
        print("error: vertical tail model failure")
    return c_small, c_big, b


def horizontal_tail_area_sizing(area_ratio, S_wing, t, A):
    S = S_wing * area_ratio
    b = math.sqrt(S * A)
    c_small = 2 * t / (1 + t) * math.sqrt(S / A)
    c_big = 2 / (1 + t) * math.sqrt(S / A)
    return b, c_small, c_big


def cg_diagram(W_EOM, X_EOM, W_list_0, X_list_0):
    W_0 = W_EOM
    X_0 = X_EOM
    W_list = [W_EOM]
    X_list = [X_EOM]
    for i in range(0, len(X_list_0)):
        W_item = W_list_0[i]
        X_item = X_list_0[i]
        W_final, X_final = cg_shift(W_0, X_0, W_item, X_item)
        X_list.append(X_final)
        W_list.append(W_final)
        W_0 = W_final
        X_0 = X_final
    return X_list, W_list


def final_X_cg(W_EOM, X_EOM, W_list_0, X_list_0):
    W_0 = W_EOM
    X_0 = X_EOM
    for i in range(0, len(X_list_0)):
        W_item = W_list_0[i]
        X_item = X_list_0[i]
        W_final, X_final = cg_shift(W_0, X_0, W_item, X_item)
        W_0 = W_final
        X_0 = X_final
    return X_0


def cg_diagram_graph(X_list, W_list, X_reverse, W_reverse):
    plt.plot(X_list, W_list, marker="o")
    plt.plot(X_reverse, W_reverse, marker="o")
    plt.grid(True)
    plt.xlabel("X_CG / MAC [m]")
    plt.ylabel("Mass [kg]")
    plt.title("CG range diagram")
    plt.show()


def main_cg_range_with_graph(
    W_fuselage,
    X_fuselage,
    W_wing,
    X_wing,
    LEMAC,
    L_fuselage,
    mac,
    X_list,
    W_list,
    X_reverse,
    W_reverse,
):  # add values from nose to back of drone

    W_EOM, X_EOM = cg_shift(W_fuselage, X_fuselage, W_wing, X_wing)

    X_list, W_list = cg_diagram(W_EOM, X_EOM, W_list, X_list)
    X_reverse, W_reverse = cg_diagram(W_EOM, X_EOM, W_reverse, X_reverse)
    cg_diagram_graph(
        [(x - LEMAC) / mac for x in X_list],
        W_list,
        [(x - LEMAC) / mac for x in X_reverse],
        W_reverse,
    )
    cg_range = [min(X_list), max(X_reverse)]


def main_cg_range_with_wing_variation_with_plot(
    W_fuselage,
    X_fuselage,
    W_wing,
    L_fuselage,
    mac,
    Dxw,
    LEMAC_In,
    LEMAC_Out,
    X_list_0,
    W_list_0,
    X_reverse_0,
    W_reverse_0,
):  # add values from nose to back of drone
    X_min = []
    X_max = []
    X_wings = []
    cg_range = []

    for X_wing_LEMAC in np.arange(LEMAC_In, L_fuselage - LEMAC_Out, 0.01):
        X_wing_CG = X_wing_LEMAC + Dxw
        W_EOM, X_EOM = cg_shift(W_fuselage, X_fuselage, W_wing, X_wing_CG)

        X_list, W_list = cg_diagram(W_EOM, X_EOM, W_list_0, X_list_0)
        X_reverse, W_reverse = cg_diagram(W_EOM, X_EOM, W_reverse_0, X_reverse_0)
        X_min.append(((min(X_list) - X_wing_LEMAC) / mac))
        X_max.append(((max(X_reverse) - X_wing_LEMAC) / mac))
        X_wings.append(X_wing_LEMAC / L_fuselage)
        cg_range.append(
            ((max(X_reverse) - X_wing_LEMAC) / mac)
            - ((min(X_list) - X_wing_LEMAC) / mac)
        )

    fig, axs = plt.subplots(1, 2)
    axs[0].plot(X_min, X_wings, marker="o")
    axs[0].plot(X_max, X_wings, marker="o")
    axs[0].set_title("CG range for wing position")
    axs[0].set(xlabel="X_cg / MAC", ylabel="X_lemac / L_fuselage")
    axs[0].grid(True)
    axs[1].plot(X_wings, cg_range)
    axs[1].set_title("CG range for wing position")
    axs[1].set(xlabel="X_lemac / L_fuselage", ylabel="X_cg / MAC")
    axs[1].grid(True)

    # Add more grid points by setting custom ticks

    for ax in axs:
        # Create finer ticks based on axis limits
        x_min, x_max = ax.get_xlim()
        y_min, y_max = ax.get_ylim()
        ax.set_xticks(np.linspace(x_min, x_max, 5))  # 5 ticks on x-axis
        ax.set_yticks(np.linspace(y_min, y_max, 5))  # 5 ticks on y-axis
        ax.tick_params(axis="x", labelrotation=45)

    plt.subplots_adjust(wspace=0.4, hspace=0.4)
    plt.show()


def main_cg_range_with_wing_variation_without_plot(
    W_fuselage,
    X_fuselage,
    W_wing,
    L_fuselage,
    mac,
    Dxw,
    LEMAC_In,
    LEMAC_Out,
    X_list_0,
    W_list_0,
    X_reverse_0,
    W_reverse_0,
):  # add values from nose to back of drone
    X_min = []
    X_max = []
    X_wings = []
    cg_range = []

    for X_wing_LEMAC in np.arange(LEMAC_In, L_fuselage - LEMAC_Out, 0.01):
        X_wing_CG = X_wing_LEMAC + Dxw
        W_EOM, X_EOM = cg_shift(W_fuselage, X_fuselage, W_wing, X_wing_CG)

        X_list, W_list = cg_diagram(W_EOM, X_EOM, W_list_0, X_list_0)
        X_reverse, W_reverse = cg_diagram(W_EOM, X_EOM, W_reverse_0, X_reverse_0)
        X_min.append(((min(X_list) - X_wing_LEMAC) / mac))
        X_max.append(((max(X_reverse) - X_wing_LEMAC) / mac))
        X_wings.append(X_wing_LEMAC / L_fuselage)
        cg_range.append(
            ((max(X_reverse) - X_wing_LEMAC) / mac)
            - ((min(X_list) - X_wing_LEMAC) / mac)
        )
    return X_min, X_max, X_wings


# Scissor Plot
def stab_cont_lines(
    S_M,
    mac,
    X_ac,
    Cl_alpha_h,
    Cl_alpha_tailless,
    d_e_d_alpha,
    Lh,
    Vh_V,
    Cm_ac,
    L_fuselage,
    Cl_h,
    Cl_tailless,
    X_LEMAC,
):
    y_tail = []
    x_stab = []
    x_control = []
    for y_t in np.arange(0, 1, 0.001):
        x_c = (
            X_ac / mac
            - Cm_ac / Cl_tailless
            + Cl_h / Cl_tailless * y_t * Lh / mac * (Vh_V) ** 2
        )
        x_control.append(x_c)
        x_s = (
            X_ac / mac
            + Cl_alpha_h
            / Cl_alpha_tailless
            * (1 - d_e_d_alpha)
            * y_t
            * Lh
            / mac
            * (Vh_V) ** 2
            - S_M
        )
        x_stab.append(x_s)
        y_tail.append(y_t)
    return y_tail, x_stab, x_control


def stab_cont_lines_plot(
    y_tail, x_stab, x_control, x_cg_start, x_cg_end, tail_area_ratio
):
    plt.plot(x_stab, y_tail, marker="o")
    plt.plot(x_control, y_tail, marker="o")
    plt.hlines(
        y=tail_area_ratio, xmin=x_cg_start, xmax=x_cg_end, color="blue", linewidth=2
    )
    plt.grid(True)
    plt.xlabel("X_cg / MAC")
    plt.ylabel("Sh/S")
    plt.title("Tail sizing")
    plt.show()


def find_wing_position(
    W_fuselage,
    X_fuselage,
    W_wing,
    L_fuselage,
    mac,
    Dxw,
    LEMAC_In,
    LEMAC_Out,
    S_M,
    X_ac,
    Cl_alpha_h,
    Cl_alpha_tailless,
    d_e_d_alpha,
    Lh,
    Vh_V,
    Cm_ac,
    Cl_h,
    Cl_tailless,
    X_list,
    W_list,
    X_reverse,
    W_reverse,
):
    X_min, X_max, X_wings = main_cg_range_with_wing_variation_without_plot(
        W_fuselage,
        X_fuselage,
        W_wing,
        L_fuselage,
        mac,
        Dxw,
        LEMAC_In,
        LEMAC_Out,
        X_list,
        W_list,
        X_reverse,
        W_reverse,
    )
    X_wings = [x * L_fuselage for x in X_wings]
    Y_g = []
    X_in_g = []
    X_fi_g = []
    X_wing_final = []
    X_min_final = []
    X_max_final = []
    Tail_Ratio_final = []
    y_control = 9999999999
    x_min = 9999999999
    x_max = 9999999999
    for X_LEMAC in X_wings:
        Y_g, X_fi_g, X_in_g = stab_cont_lines(
            S_M,
            mac,
            X_ac,
            Cl_alpha_h,
            Cl_alpha_tailless,
            d_e_d_alpha,
            Lh,
            Vh_V,
            Cm_ac,
            L_fuselage,
            Cl_h,
            Cl_tailless,
            X_LEMAC,
        )

        x_cg_min = X_min[X_wings.index(X_LEMAC)]
        x_cg_max = X_max[X_wings.index(X_LEMAC)]

        for t in Y_g:
            index = Y_g.index(t)
            if (
                x_cg_min >= X_in_g[index]
                and X_fi_g[index] >= x_cg_max
                and t < y_control
            ):
                y_control = t
                x_min = x_cg_min
                x_max = x_cg_max
        Tail_Ratio_final.append(y_control)
        X_wing_final.append(X_LEMAC)
        X_min_final.append(x_min)
        X_max_final.append(x_max)
    final_index = Tail_Ratio_final.index(min(Tail_Ratio_final))
    return (
        X_min_final[final_index],
        X_max_final[final_index],
        X_wing_final[final_index],
        Tail_Ratio_final[final_index],
    )


if __name__ == "__main__":
    from prelim_des.drone import Drone
    from prelim_des.mission import Mission
    from prelim_des.performance import Performance

    mission = Mission("DRCCRCCRCCD")
    drone = Drone()
    perf = Performance(drone, mission)
    drone.perf = perf
    drone.class_1_weight_estimate()
    drone.wing.S = perf.wing_area(drone.OEW)
    drone.class_2_weight_estimate(transition=True)

    main_horizontal_stability(
        drone,
        graph=True,
    )
