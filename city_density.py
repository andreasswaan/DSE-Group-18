import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
from scipy.spatial.distance import cdist
import matplotlib.patches as mpatches

# City parameters
city_radius = 2.5  # km
num_samples = 1000
PLOT = False

# Grid for plotting
grid_size = 200
x = np.linspace(-city_radius, city_radius, grid_size)
y = np.linspace(-city_radius, city_radius, grid_size)
X, Y = np.meshgrid(x, y)
pos = np.dstack((X, Y))

# Define restaurant density: peak at center
restaurant_mu = [0, 0]
restaurant_sigma = [[0.8**2, 0], [0, 0.8**2]]
restaurant_rv = multivariate_normal(restaurant_mu, restaurant_sigma)
restaurant_density = restaurant_rv.pdf(pos)

# Define customer density: ring around center (radius ≈ 2 km)
customer_mu = [0, 0]
customer_sigma = [[1.5**2, 0], [0, 1.5**2]]
customer_rv = multivariate_normal(customer_mu, customer_sigma)
customer_density = (
    customer_rv.pdf(pos)
    * (cdist(pos.reshape(-1, 2), [[0, 0]]).reshape(grid_size, grid_size) + 0.2) ** 0.3
)


def sample_points(density, num_samples):
    flat_density = density.ravel()
    flat_density /= flat_density.sum()
    indices = np.random.choice(len(flat_density), size=num_samples, p=flat_density)
    coords = np.column_stack(np.unravel_index(indices, density.shape))
    coords = np.column_stack([x[coords[:, 1]], y[coords[:, 0]]])
    return coords


depot_locations = {
    "Center": np.array([0.0, 0.0]),
    "Outside": np.array([2.0 * np.cos(np.deg2rad(45)), 2.0 * np.sin(np.deg2rad(45))]),
}


def get_distance_constants(PLOT=False):
    output_dict = {}
    for depot_label, depot in depot_locations.items():
        # Sample points
        restaurants = sample_points(restaurant_density, num_samples)
        customers = sample_points(customer_density, num_samples)
        customers_2 = sample_points(customer_density, num_samples)  # for CC distance

        # Compute distances
        RC = np.linalg.norm(restaurants - customers, axis=1)
        CD = np.linalg.norm(customers - depot, axis=1)
        DR = np.linalg.norm(restaurants - depot, axis=1)
        CC_all = np.linalg.norm(customers - customers_2, axis=1)
        CC = CC_all[CC_all <= 2]
        output_dict[depot_label] = {
            "RC": RC.mean(),
            "CD": CD.mean(),
            "DR": DR.mean(),
            "CC": CC.mean(),
        }

        if PLOT:
            # Plotting densities
            fig, ax = plt.subplots(1, 2, figsize=(14, 6))
            im0 = ax[0].imshow(
                restaurant_density,
                extent=(-city_radius, city_radius, -city_radius, city_radius),
                origin="lower",
                cmap="Reds",
            )
            ax[0].set_title("Restaurant Density")
            ax[0].set_xlabel("km")
            ax[0].set_ylabel("km")
            ax[0].plot(depot[0], depot[1], "ko", markersize=10, label="Depot")
            red_patch = mpatches.Patch(color="red", label="Restaurant Density")
            ax[0].legend(
                handles=[red_patch, mpatches.Patch(color="black", label="Depot")],
                loc="upper right",
            )

            im1 = ax[1].imshow(
                customer_density,
                extent=(-city_radius, city_radius, -city_radius, city_radius),
                origin="lower",
                cmap="Blues",
            )
            ax[1].set_title("Customer Density")
            ax[1].set_xlabel("km")
            ax[1].set_ylabel("km")
            ax[1].plot(depot[0], depot[1], "ko", markersize=10, label="Depot")
            blue_patch = mpatches.Patch(color="blue", label="Customer Density")
            ax[1].legend(
                handles=[blue_patch, mpatches.Patch(color="black", label="Depot")],
                loc="upper right",
            )

            plt.suptitle(f"Depot Location: {depot_label}")
            plt.tight_layout(rect=[0, 0, 1, 0.96])
            plt.show()

            # Plotting distance histograms
            fig, ax = plt.subplots(2, 2, figsize=(14, 10))
            ax[0, 0].hist(RC, bins=30, color="orange", edgecolor="black")
            ax[0, 0].set_title(f"RC Distance (avg: {RC.mean():.2f} km)")

            ax[0, 1].hist(CD, bins=30, color="green", edgecolor="black")
            ax[0, 1].set_title(f"CD Distance (avg: {CD.mean():.2f} km)")

            ax[1, 0].hist(DR, bins=30, color="purple", edgecolor="black")
            ax[1, 0].set_title(f"DR Distance (avg: {DR.mean():.2f} km)")

            ax[1, 1].hist(CC, bins=30, color="brown", edgecolor="black")
            ax[1, 1].set_title(f"CC Distance ≤ 2 km (avg: {CC.mean():.2f} km)")

            for a in ax.ravel():
                a.set_xlabel("Distance (km)")
                a.set_ylabel("Frequency")

            plt.suptitle(f"Depot Location: {depot_label}")
            plt.tight_layout(rect=[0, 0, 1, 0.96])
            plt.show()

    return output_dict


if __name__ == "__main__":
    distances = get_distance_constants()
    print(distances)
