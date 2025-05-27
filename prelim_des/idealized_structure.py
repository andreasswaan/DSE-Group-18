import numpy as np
import matplotlib.pyplot as plt


class Boom:
    def __init__(self, x: float, y: float, area: float):
        self.x = x  # [m]
        self.y = y  # [m]
        self.area = area  # [m^2]


class IdealizedSection:
    def __init__(self, booms: list[Boom]):
        self.booms = booms
        self.centroid_x, self.centroid_y = self.calc_centroid()
        self.Ixx, self.Iyy, self.Ixy = self.calc_moments()

    def calc_centroid(self):
        A_total = sum(b.area for b in self.booms)
        x_c = sum(b.area * b.x for b in self.booms) / A_total
        y_c = sum(b.area * b.y for b in self.booms) / A_total
        return x_c, y_c

    def calc_moments(self):
        x_c, y_c = self.centroid_x, self.centroid_y
        Ixx = sum(b.area * (b.y - y_c) ** 2 for b in self.booms)
        Iyy = sum(b.area * (b.x - x_c) ** 2 for b in self.booms)
        Ixy = sum(b.area * (b.x - x_c) * (b.y - y_c) for b in self.booms)
        return Ixx, Iyy, Ixy

    def bending_stress(self, Mx: float, My: float) -> list[float]:
        x_c, y_c = self.centroid_x, self.centroid_y
        Ixx, Iyy, Ixy = self.Ixx, self.Iyy, self.Ixy
        denom = Ixx * Iyy - Ixy**2
        stresses = []
        for b in self.booms:
            y = b.y - y_c
            x = b.x - x_c
            sigma = (My * Ixx * x - Mx * Ixy * x + Mx * Iyy * y - My * Ixy * y) / denom
            stresses.append(sigma)
        return stresses

    def plot_section(self):
        x = [b.x for b in self.booms]
        y = [b.y for b in self.booms]
        plt.figure(figsize=(6, 4))
        plt.scatter(x, y, c="red")
        plt.axhline(self.centroid_y, color="blue", linestyle="--", label="Centroid y")
        plt.axvline(self.centroid_x, color="green", linestyle="--", label="Centroid x")
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.title("Idealized Section")
        plt.legend()
        plt.axis("equal")
        plt.grid(True)
        plt.show()


def create_rectangular_section(
    width: float, height: float, n_booms: int, boom_area: float
) -> IdealizedSection:
    booms = []
    perim = 2 * (width + height)
    spacing = perim / n_booms
    current = 0.0
    for i in range(n_booms):
        if current < width:
            x, y = current, 0
        elif current < width + height:
            x, y = width, current - width
        elif current < 2 * width + height:
            x, y = width - (current - width - height), height
        else:
            x, y = 0, height - (current - 2 * width - height)
        booms.append(Boom(x, y, boom_area))
        current += spacing
    return IdealizedSection(booms)


if __name__ == "__main__":
    width = 0.2  # m
    height = 0.12  # m
    n_booms = 8
    boom_area = 1e-5  # m^2

    section = create_rectangular_section(width, height, n_booms, boom_area)
    print(f"Centroid: ({section.centroid_x:.4f}, {section.centroid_y:.4f}) m")
    print(f"Ixx = {section.Ixx:.6e} m^4")
    print(f"Iyy = {section.Iyy:.6e} m^4")
    print(f"Ixy = {section.Ixy:.6e} m^4")

    Mx = 10.0  # Nm
    My = 5.0  # Nm
    stresses = section.bending_stress(Mx, My)
    for i, sigma in enumerate(stresses):
        print(f"Boom {i+1}: Stress = {sigma:.2f} Pa")

    section.plot_section()
