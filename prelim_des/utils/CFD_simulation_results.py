import numpy as np

import matplotlib.pyplot as plt

AOA = [0, 3, 6, 9, 12]

mixed_rc640_tc320_sweep_0_lift = [
    0.3330991,
    0.567915078,
    0.787545788,
    0.939034661,
    1.020562657,
]
mixed_rc640_tc320_sweep_10_lift = [
    0.339876903,
    0.576202935,
    0.79296803,
    0.97944731,
    1.060442052,
]

sweep_10_lift = [0.346303227, 0.594094893, 0.828717369, 1.004040832, 1.046416998]
no_sweep_lift = [0.349473123, 0.589010506, 0.800520993, 0.941433828, 0.974042951]

tc120_lift = [0.348450043, 0.583965495, 0.765859446, 0.870095166, 0.957614988]
tc320_lift = [0.347680225, 0.587492963, 0.811471378, 0.97615614, 0.990863822]

rc440_lift = [0.362297113, 0.598998281, 0.816417143, 0.930523445, 0.92740638]
rc640_lift = [0.324726239, 0.562986486, 0.780789794, 0.921569625, 0.979597372]

base_lift = [0.353540632, 0.590278464, 0.813790614, 0.948213011, 0.979108506]


def average_cl_alpha():
    output_cl_alpha = []
    for i in range(len(AOA)):
        output = (
            sweep_10_lift[i]
            + no_sweep_lift[i]
            + tc120_lift[i]
            + tc320_lift[i]
            + rc440_lift[i]
            + rc640_lift[i]
            + base_lift[i]
        ) / 9
        output_cl_alpha.append(output)
    return output_cl_alpha


def root_chord_cl_alpha(root_chord):
    output_cl_alpha = []
    if root_chord > 640 or root_chord < 440:
        raise ValueError("root chord out of bounds", root_chord)
    for i in range(len(AOA)):
        ratio = (root_chord - 0.440) / 0.200
        output = abs(rc640_lift[i] - rc440_lift[i]) * ratio + min(
            [rc640_lift[i], rc440_lift[i]]
        )
        output_cl_alpha.append(output)
    return output_cl_alpha


def tip_chord_cl_alpha(tip_chord):
    output_cl_alpha = []
    if tip_chord > 320 or tip_chord < 120:
        raise ValueError("tip chord out of bounds", tip_chord)
    for i in range(len(AOA)):
        ratio = (tip_chord - 0.120) / 0.200
        output = abs(tc320_lift[i] - tc120_lift[i]) * ratio + min(
            [tc120_lift[i], tc320_lift[i]]
        )
        output_cl_alpha.append(output)
    return output_cl_alpha


# Plotting Cl vs AOA
if __name__ == "__main__":
    cl_alpha = tip_chord_cl_alpha(0.220)
    plt.plot(AOA, cl_alpha, marker="o", label="Tip Chord Cl")
    plt.plot(AOA, tc320_lift, marker="o", label="rc640_lift")
    plt.plot(AOA, tc120_lift, marker="o", label="rc440_lift")

    plt.xlabel("Angle of Attack (deg)")
    plt.ylabel("Average $C_l$")
    plt.title("Average $C_l$ vs Angle of Attack")
    plt.legend()

    plt.grid(True)
    plt.show()

    # Estimate Cl at AOA = 8 using linear interpolation

    aoa_query = 8
    cl_at_8 = np.interp(aoa_query, AOA, cl_alpha)
    print(f"Estimated average Cl at AOA={aoa_query}: {cl_at_8:.4f}")
