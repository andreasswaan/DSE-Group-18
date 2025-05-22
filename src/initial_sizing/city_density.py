import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
from scipy.spatial.distance import cdist
import matplotlib.patches as mpatches

class CityDensity:
    def __init__(self, city_radius=3, num_samples=10000, grid_size=200):
        self.city_radius = city_radius
        self.num_samples = num_samples
        self.grid_size = grid_size

        # Grid for plotting
        self.x = np.linspace(-city_radius, city_radius, grid_size)
        self.y = np.linspace(-city_radius, city_radius, grid_size)
        self.X, self.Y = np.meshgrid(self.x, self.y)
        self.pos = np.dstack((self.X, self.Y))

        # --- Mask for circle ---
        self.circle_mask = self.X**2 + self.Y**2 <= city_radius**2

        # Densities
        self.restaurant_density = self._make_restaurant_density()
        self.customer_density = self._make_customer_density()

        # Depot locations
        self.depot_locations = {
            "Center": np.array([0.0, 0.0]),
            "Outside": np.array([2.0 * np.cos(np.deg2rad(45)), 2.0 * np.sin(np.deg2rad(45))]),
        }

    def _make_restaurant_density(self):
        mu = [0, 0]
        sigma = [[0.8**2, 0], [0, 0.8**2]]
        rv = multivariate_normal(mu, sigma)
        density = rv.pdf(self.pos)
        density[~self.circle_mask] = 0
        return density

    def _make_customer_density(self):
        mu = [0, 0]
        sigma = [[1.5**2, 0], [0, 1.5**2]]
        rv = multivariate_normal(mu, sigma)
        density = (
            rv.pdf(self.pos)
            * (cdist(self.pos.reshape(-1, 2), [[0, 0]]).reshape(self.grid_size, self.grid_size) + 0.2) ** 0.3
        )
        density[~self.circle_mask] = 0
        return density

    def sample_points(self, density, num_samples):
        flat_density = density.ravel()
        flat_density /= flat_density.sum()
        indices = np.random.choice(len(flat_density), size=num_samples, p=flat_density)
        coords = np.column_stack(np.unravel_index(indices, density.shape))
        coords = np.column_stack([self.x[coords[:, 1]], self.y[coords[:, 0]]])
        return coords

    def get_distance_constants(self, PLOT=False):
        output_dict = {}
        for depot_label, depot in self.depot_locations.items():
            # Sample points
            restaurants = self.sample_points(self.restaurant_density, self.num_samples)
            customers = self.sample_points(self.customer_density, self.num_samples)
            customers_2 = self.sample_points(self.customer_density, self.num_samples)  # for CC distance

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
                "CR": RC.mean(),
            }

            if PLOT:
                # Plotting densities
                fig, ax = plt.subplots(1, 2, figsize=(14, 6))
                im0 = ax[0].imshow(
                    self.restaurant_density,
                    extent=(-self.city_radius, self.city_radius, -self.city_radius, self.city_radius),
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
                    self.customer_density,
                    extent=(-self.city_radius, self.city_radius, -self.city_radius, self.city_radius),
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
    city = CityDensity()
    distances = city.get_distance_constants(PLOT=True)
    print(distances)
