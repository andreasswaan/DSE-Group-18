import math
import matplotlib.pyplot as plt
import numpy as np


# CG range graph
def cg_shift(W_old, X_old, W_item, X_item):
    return W_old + W_item, (W_old * X_old + W_item * X_item) / (W_old + W_item)


def item_input_sort():
    N = 1
    W_list = []
    X_list = []
    while N == 1:
        W = float(input("Weight input kg:"))
        X = float(input("Distance from nose m:"))
        W_list.append(W)
        X_list.append(X)
        choice = int(input("1. continue / 2. end :"))
        if choice == 2:
            N = 0
    X_reverse = X_list[::-1]
    W_reverse = W_list[::-1]
    return X_list, W_list, X_reverse, W_reverse


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
    Vh,
    V,
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
            + Cl_h / Cl_tailless * y_t * Lh / mac * (Vh / V) ** 2
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
            * (Vh / V) ** 2
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
    Vh,
    V,
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
            Vh,
            V,
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


# INITIAL VALUES
W_fuselage = 15  # kg
X_fuselage = 5  # m
W_wing = 5  # kg
L_fuselage = 10  # m
mac = 2.5  # m
Dxw = 0.2  # m , from LEMAC to wing CG
LEMAC_In = 2  # m, front limit of wing positioning
LEMAC_Out = 3  # m, back limit of wing positioning
S_M = 0.05  # safety margin
X_ac = 0.25 * mac  # m , ac position from lemac
Cl_alpha_h = 4  # tail Cl alpha
Cl_alpha_tailless = 5  # no tail Cl alpha
d_e_d_alpha = 0.5  # downwash effect something
Lh = L_fuselage  # tail arm
Vh = 15  # tail stream velocity , m/s
V = 15  # tail stream velocity, m/s
Cm_ac = -0.1  # ac moment constant
Cl_h = -0.2  # tail cl
Cl_tailless = 0.3  # tailess aircraft cl

# MAIN

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
    Vh,
    V,
    Cm_ac,
    Cl_h,
    Cl_tailless,
    X_list,
    W_list,
    X_reverse,
    W_reverse,
)

print(
    "most infront x_cg:",
    x_cg_start,
    "\n",
    "most back x_cg:",
    x_cg_end,
    "\n",
    "wing position:",
    wing_position,
    "\n",
    "tail_area_ratio:",
    tail_area_ratio,
)

# Extra initial values

X_LEMAC = wing_position
X_wing = X_LEMAC + Dxw  # m


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
    Vh,
    V,
    Cm_ac,
    L_fuselage,
    Cl_h,
    Cl_tailless,
    wing_position,
)
stab_cont_lines_plot(y_tail, x_stab, x_control, x_cg_start, x_cg_end, tail_area_ratio)
