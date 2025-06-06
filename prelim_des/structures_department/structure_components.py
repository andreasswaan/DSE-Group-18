from matplotlib import pyplot as plt
from matplotlib import ticker
from numpy import np

from prelim_des.constants import g
from prelim_des.drone import Drone

from .section_geometry import Boom, IdealizedSection


# === STRUCTURES ASSEMBLY ===

class ConnectorStructure:
    def __init__(
        self,
        length: float,
        n_sections: int,
        root_section: IdealizedSection,
        start_point: tuple[float, float, float],  # (x, y, z) at point load
        end_point: tuple[float, float, float],  # (x, y, z) at wing surface
    ):
        self.length = length
        self.n_sections = n_sections
        self.root_section = root_section
        self.start_point = np.array(start_point)
        self.end_point = np.array(end_point)
        self.sections = self.generate_sections()

    def generate_sections(self) -> list[tuple[np.ndarray, IdealizedSection]]:
        sections = []
        for i in range(self.n_sections):
            frac = i / (self.n_sections - 1)
            pos = self.start_point * (1 - frac) + self.end_point * frac
            # No scaling of cross-section for now, but you could add tapering if needed
            section = IdealizedSection(
                [
                    Boom(
                        x=boom.x,
                        y=boom.y,
                        area=boom.area,
                        boom_type=boom.type,
                        material=boom.material,
                    )
                    for boom in self.root_section.booms
                ]
            )
            sections.append((pos, section))
        return sections


class FuselageStructure:
    def __init__(
        self,
        length: float,
        root_section: IdealizedSection,
        n_sections: int,
        taper_ratio: float = 1.0,  # For constant-diameter, keep at 1.0
    ):
        self.length = length
        self.width = root_section.width
        self.height = root_section.height
        self.n_sections = n_sections
        self.taper_ratio = taper_ratio
        self.root_section = root_section
        self.dz = length / (n_sections - 1)
        self.sections = self.generate_sections()

    def generate_sections(self) -> list[tuple[float, IdealizedSection]]:
        """
        Returns a list of (x_position, IdealizedSection) from nose to tail.
        """
        sections = []
        for i in range(self.n_sections):
            x = (i / (self.n_sections - 1)) * self.length
            scale = 1 - (1 - self.taper_ratio) * (i / (self.n_sections - 1))
            scaled_booms = [
                Boom(
                    x=boom.x * scale,
                    y=boom.y * scale,
                    area=boom.area,
                    boom_type=boom.type,
                    material=boom.material,
                )
                for boom in self.root_section.booms
            ]
            section = IdealizedSection(scaled_booms)
            sections.append((x, section))
        return sections

    def compute_weight_distribution(self) -> list[float]:
        """
        Returns a list of weight per section [N] for the fuselage,
        based on actual structural mass distribution.
        """
        dz = self.dz
        weight_per_section = []
        for _, section in self.sections:
            w_z = section.mass(dz) * g  # [N] for this section
            weight_per_section.append(w_z)
        return weight_per_section

    def compute_bending_moments_with_point_loads(
        self,
        point_loads: list[dict],
    ) -> tuple[list[float], list[float]]:
        """
        Returns two lists: (Mz_per_section, My_per_section)
        - Mz: bending moment about z-axis (from vertical forces and point moments)
        - My: bending moment about y-axis (from drag and point moments)
        Includes effects of point loads and point moments.
        """
        Mz_list = []
        My_list = []
        section_positions = [x for x, _ in self.sections]
        for i, x in enumerate(section_positions):
            Mz = 0
            My = 0
            for pl in point_loads:
                if pl["x"] >= x:
                    arm = pl["x"] - x
                    Pz = pl.get("Pz", 0)
                    Px = pl.get("Px", 0)
                    # Bending about z-axis from vertical force
                    Mz += Pz * arm
                    # Bending about y-axis from drag force
                    My += Px * arm
                    # Add any direct moments applied at this point
                    if abs(pl["x"] - x) < 1e-6:  # at this section
                        Mz += pl.get("Mz", 0)
                        My += pl.get("My", 0)
            Mz_list.append(Mz)
            My_list.append(My)
        return Mz_list, My_list

    def compute_bending_stresses(
        self, Mz_per_section: list[float], My_per_section: list[float]
    ) -> list[list[float]]:
        """
        Returns a list of [stress at each boom] for each section.
        """
        stresses_per_section = []
        for i, (x_pos, section) in enumerate(self.sections):
            Mz = Mz_per_section[i] if Mz_per_section else 0
            My = My_per_section[i] if My_per_section else 0
            # Pass both moments to the section's bending_stress method
            stresses = section.bending_stress(Mx=Mz, My=0)
            stresses_per_section.append(stresses)
        return stresses_per_section
    
    import numpy as np

    def draw_moment_arrow(self, ax, origin, axis='z', radius=0.1, direction=1, color='purple', lw=2):
        # origin: (x, y, z)
        # axis: 'z' for moment about z, 'y' for moment about y
        # direction: 1 for CCW, -1 for CW (relative to axis)
        theta = np.linspace(0, np.pi/1.5, 30)  # arc angle
        if axis == 'z':
            x = origin[0] + radius * np.cos(theta)
            y = origin[1] + direction * radius * np.sin(theta)
            z = np.full_like(x, origin[2])
            # Arrowhead
            ax.quiver(
                x[-1], y[-1], z[-1],
                -0.05 * np.sin(theta[-1]), 0.05 * np.cos(theta[-1]), 0,
                color=color, linewidth=lw
            )
        elif axis == 'y':
            x = origin[0] + radius * np.cos(theta)
            y = np.full_like(x, origin[1])
            z = origin[2] + direction * radius * np.sin(theta)
            ax.quiver(
                x[-1], y[-1], z[-1],
                -0.05 * np.sin(theta[-1]), 0, 0.05 * np.cos(theta[-1]),
                color=color, linewidth=lw
            )
        ax.plot(x, y, z, color=color, linewidth=lw)

    

    def plot_3d_fuselage(
        self,
        stresses_per_section: list[list[float]],
        point_loads: list[dict] = None,
        weight_per_section: list[float] = None,
        arrow_scale: float = 1.0,
    ):
        import matplotlib.colors as mcolors
        from matplotlib import cm

        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.view_init(azim=210)

        # Flatten all stresses for color normalization
        all_stresses = [
            s for section_stresses in stresses_per_section for s in section_stresses
        ]
        norm = mcolors.Normalize(vmin=min(all_stresses), vmax=max(all_stresses))
        cmap = plt.colormaps["viridis"]

        for i, (x_pos, section) in enumerate(self.sections):
            stresses = stresses_per_section[i]
            for boom, stress in zip(section.booms, stresses):
                color = cmap(norm(stress))
                # x: fuselage length, y: sideways, z: upwards
                ax.scatter(
                    x_pos,  # x: fuselage length
                    boom.x,  # y: sideways
                    boom.y,  # z: upwards
                    color=color,
                    s=boom.area * 1e6 * 20,
                )

        # --- Plot point load arrows ---
        if point_loads:
            for pl in point_loads:
                # Default to 0 if not specified
                Px = pl.get("Px", 0)
                Py = pl.get("Py", 0)
                Pz = pl.get("Pz", 0)
                # Plot drag arrow if Px is nonzero
                if Px != 0:
                    ax.quiver(
                        pl["x"], pl["y"], pl["z"],
                        Px / arrow_scale, 0, 0,
                        color="green",  # or another color for drag
                        arrow_length_ratio=0.2,
                        linewidth=3,
                        alpha=0.9,
                        label="Drag"  # Only label the first time if needed
                    )
                # Plot vertical force arrow as before
                if Pz != 0:
                    ax.quiver(
                        pl["x"], pl["y"], pl["z"],
                        0, 0, Pz / arrow_scale,
                        color="red",
                        arrow_length_ratio=0.2,
                        linewidth=3,
                        alpha=0.9,
                        label="Lift - Weight"
                    )
                    
             # --- Plot moment (rotating) arrows if present ---
        for pl in point_loads:
            # Draw Mz (about z-axis)
            if abs(pl.get("Mz", 0)) > 1e-6:
                self.draw_moment_arrow(
                    ax,
                    (pl["x"], pl["y"], pl["z"]),
                    axis='z',
                    radius=0.1,
                    direction=np.sign(pl["Mz"]),
                    color='purple',
                    lw=3,
                )
            # Draw My (about y-axis)
            if abs(pl.get("My", 0)) > 1e-6:
                self.draw_moment_arrow(
                    ax,
                    (pl["x"], pl["y"], pl["z"]),
                    axis='y',
                    radius=0.1,
                    direction=np.sign(pl["My"]),
                    color='orange',
                    lw=3,
                )

        if weight_per_section:
            arrow_scale = (
                max(weight_per_section) / 0.1 if max(weight_per_section) != 0 else 1.0
            )
            for i, (x_pos, section) in enumerate(self.sections):
                x_c = np.mean([boom.x for boom in section.booms])
                y_c = np.mean([boom.y for boom in section.booms])
                weight = weight_per_section[i]
                ax.quiver(
                    x_pos,
                    x_c,
                    y_c,
                    0,
                    0,
                    -weight / arrow_scale,
                    color="blue",
                    arrow_length_ratio=0.2,
                    linewidth=2,
                    alpha=0.7,
                    label="Weight" if i == 0 else None,
                )

        ax.set_title("Fuselage Structure (Booms colored by Bending Stress)")
        ax.set_xlabel("x [m] (fuselage length)")
        ax.set_ylabel("y [m] (sideways)")
        ax.set_zlabel("z [m] (upwards)")
        ax.grid(True)

        mappable = cm.ScalarMappable(norm=norm, cmap=cmap)
        mappable.set_array(all_stresses)
        cbar = plt.colorbar(mappable, ax=ax, pad=0.1)
        cbar.set_label("Bending Stress [Pa]")

        ax.xaxis.set_major_locator(ticker.MaxNLocator(4))
        ax.yaxis.set_major_locator(ticker.MaxNLocator(4))
        ax.zaxis.set_major_locator(ticker.MaxNLocator(4))

        ax.set_xlim(-0.3, 1.5)
        ax.set_ylim(-0.8, 0.8)
        ax.set_zlim(-0.5, 0.5)

        plt.tight_layout()
        plt.show()

    def compute_weight_distribution(self) -> list[float]:
        """
        Returns a list of weight per section [N] for the fuselage,
        based on actual structural mass distribution.
        """
        dz = self.dz
        weight_per_section = []
        for _, section in self.sections:
            w_z = section.mass(dz) * g  # [N] for this section
            weight_per_section.append(w_z)
        return weight_per_section

    def mass(self) -> float:
        """
        Returns the total structural mass of the fuselage [kg].
        """
        dz = self.length / (self.n_sections - 1)
        return sum(section.mass(dz) for _, section in self.sections)

    def compute_bending_moments_with_distributed_and_point_loads(
        self,
        distributed_loads: list[float],
        point_loads: list[dict],
    ) -> tuple[list[float], list[float]]:
        """
        Returns two lists: (Mz_per_section, My_per_section)
        - Mz: bending moment about z-axis (from vertical loads)
        Includes effects of distributed loads (e.g., fuselage weight) and point loads.
        """
        Mz_list = []
        My_list = []
        section_positions = [x for x, _ in self.sections]
        n = len(section_positions)
        for i, x in enumerate(section_positions):
            Mz = 0.0
            # Distributed loads (from i to end)
            for j in range(i, n):
                x_j = section_positions[j]
                arm = x_j - x
                Mz += distributed_loads[j] * arm
            # Point loads
            for pl in point_loads:
                if pl["x"] >= x:
                    arm = pl["x"] - x
                    Pz = pl.get("Pz", 0)
                    Mz += Pz * arm
            Mz_list.append(Mz)
            My_list.append(0.0)  # Not used, but kept for compatibility
        return Mz_list, My_list


class WingStructure:
    def __init__(
        self,
        n_sections: int,
        root_section: IdealizedSection,
        drone: Drone,
    ):
        self.span = float(drone.wing.span)
        self.n_sections = n_sections
        self.taper_ratio = float(drone.wing.taper)
        self.root_section = root_section
        self.dy = self.span / 2 / (n_sections - 1)
        self.sections = self.generate_sections()
        self.total_weight = None

    def generate_sections(self) -> list[tuple[float, IdealizedSection]]:
        """
        Returns a list of (y_position, IdealizedSection) from root to tip.
        """
        sections = []

        for i in range(self.n_sections):
            y = (i / (self.n_sections - 1)) * (
                self.span / 2
            )  # Half-span (symmetric wing)
            scale = 1 - (1 - self.taper_ratio) * (i / (self.n_sections - 1))

            scaled_booms = [
                Boom(
                    x=boom.x * scale,
                    y=boom.y * scale,
                    area=boom.area * (scale**2),
                    boom_type=boom.type,
                    material=boom.material,
                )
                for boom in self.root_section.booms
            ]
            section = IdealizedSection(scaled_booms)
            sections.append((y, section))

        return sections

    def compute_total_weight(self) -> float:
        """
        Computes total structural weight of the wing based on boom areas, material densities, and span.
        Assumes the total wing (not just half).
        """
        dy = self.dy
        weight_per_section = []
        total_weight = 0.0
        for _, section in self.sections:
            w_y = section.mass(dy) * g  # [N] for this section
            weight_per_section.append(w_y)
        return weight_per_section  # Full span (you model only half)

    def compute_weight_distribution(self) -> list[float]:
        """
        Returns a list of weight per section [N] for half-wing.
        """
        # if self.total_weight is None:
        #     raise ValueError(
        #         "Set self.total_weight before computing weight distribution."

        dz = self.dz
        weight_per_section = []
        for _, section in self.sections:
            w_z = section.mass(dz) * g  # [N] for this section
            weight_per_section.append(w_z)
        return weight_per_section

    def compute_net_vertical_load(
        self, lift_per_section: list[float], weight_per_section: list[float]
    ) -> list[float]:
        """
        Computes net vertical force per section (lift - weight).
        """
        return [L - W for L, W in zip(lift_per_section, weight_per_section)]

    def compute_shear_forces(self, net_vertical_load: list[float]) -> list[float]:
        """
        Returns list of shear forces [N] at each section along span.
        """
        shear_forces = []
        for i in range(len(net_vertical_load)):
            shear = sum(net_vertical_load[i:])
            shear_forces.append(shear)
        return shear_forces

    def compute_bending_moments(self, net_vertical_load: list[float]) -> list[float]:
        """
        Returns a list of bending moments at each section (about x-axis).
        """
        moments_x = []
        for i, (y_pos, _) in enumerate(self.sections):
            moment = 0
            for j in range(i, len(self.sections)):
                y_out, _ = self.sections[j]
                arm = y_out - y_pos
                moment += net_vertical_load[j] * arm
            moments_x.append(moment)
        return moments_x

    def compute_bending_moments_with_point_loads(
        self, net_vertical_load: list[float], point_loads: list[dict]
    ) -> list[float]:
        """
        Returns a list of bending moments at each section (about x-axis),
        including effects of point loads (e.g., payload, landing gear).
        point_loads: list of dicts, each with {"y": position, "Pz": load}
        """
        moments_x = []
        section_positions = [y for y, _ in self.sections]
        for i, y in enumerate(section_positions):
            moment = 0
            # Distributed loads (as before)
            for j in range(i, len(self.sections)):
                y_out, _ = self.sections[j]
                arm = y_out - y
                moment += net_vertical_load[j] * arm
            # Add point loads
            for pl in point_loads:
                if pl["y"] >= y:
                    arm = pl["y"] - y
                    Pz = pl.get("Pz", 0)
                    moment += Pz * arm
            moments_x.append(moment)
        return moments_x

    def compute_bending_stresses(
        self, net_vertical_load: list[float]
    ) -> tuple[list[float], list[list[float]]]:
        moments_x = self.compute_bending_moments(net_vertical_load)
        stresses_per_section = []
        for i, (y_pos, section) in enumerate(self.sections):
            stresses = section.bending_stress(Mx=moments_x[i], My=0)
            stresses_per_section.append(stresses)
        return moments_x, stresses_per_section

    def compute_vertical_deflections(
        self, net_vertical_load: list[float]
    ) -> list[float]:
        """
        Returns the vertical deflection at each section along the span.
        Assumes constant EI (uses root section Ixx and material E).
        """
        E = self.root_section.booms[0].material.E  # Pa
        Ixx = self.root_section.Ixx  # m^4
        dy = self.dy

        moments_x = self.compute_bending_moments(net_vertical_load)

        # Integrate twice to get deflection (Euler-Bernoulli, discrete)
        theta = [0.0]  # slope at root
        for i in range(1, len(moments_x)):
            dtheta = moments_x[i - 1] * dy / (E * Ixx)
            theta.append(theta[-1] + dtheta)
        w = [0.0]  # deflection at root
        for i in range(1, len(theta)):
            dw = theta[i - 1] * dy
            w.append(w[-1] + dw)
        return w

    def plot_3d_wing(
        self,
        stresses_per_section: list[list[float]],
        lift_per_section: list[float] = None,
        weight_per_section: list[float] = None,
        point_loads: list[dict] = None,
        drag_per_section: list[float] = None,
        connector_sections: list[dict] = None,
        connector_stresses_per_section: list[list[float]] = None,
        arrow_scale: float = 1.0,
    ):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

        from matplotlib import cm
        import matplotlib.colors as mcolors

        # --- Flatten all stresses for color normalization ---
        all_stresses = [
            stress
            for section_stresses in stresses_per_section
            for stress in section_stresses
        ]
        if connector_stresses_per_section:
            all_stresses += [
                stress
                for section_stresses in connector_stresses_per_section
                for stress in section_stresses
            ]
        min_stress = min(all_stresses)
        max_stress = max(all_stresses)
        norm = mcolors.Normalize(vmin=min_stress, vmax=max_stress)
        cmap = plt.colormaps["viridis"]

        # --- Plot booms colored by stress ---
        for i, (y_pos, section) in enumerate(self.sections):
            stresses = stresses_per_section[i]
            for boom, stress in zip(section.booms, stresses):
                color = cmap(norm(stress))
                x = boom.x
                z = boom.y
                ax.scatter(
                    x,
                    y_pos,
                    z,
                    color=color,
                    s=boom.area * 1e6 * 20,
                )

        # --- Add load arrows (lift) ---
        arrow_scale = None
        if lift_per_section:
            arrow_scale = (
                max(lift_per_section) / 0.1 if max(lift_per_section) != 0 else 1.0
            )

            for i, (y_pos, section) in enumerate(self.sections):
                x_c = np.mean([boom.x for boom in section.booms])
                upper_boom = max(section.booms, key=lambda b: b.y)
                z_c = upper_boom.y
                lift = lift_per_section[i]
                ax.quiver(
                    x_c,
                    y_pos,
                    z_c,
                    0,
                    0,
                    lift / arrow_scale,
                    color="red",
                    arrow_length_ratio=0.2,
                    linewidth=2,
                    alpha=0.7,
                    label="Lift" if i == 0 else None,
                )

        # --- Add load arrows (weight, downward) ---
        if weight_per_section:
            # Use the same arrow_scale as lift for direct comparison
            if arrow_scale is None:
                arrow_scale = (
                    max(weight_per_section) / 0.1
                    if max(weight_per_section) != 0
                    else 1.0
                )

            for i, (y_pos, section) in enumerate(self.sections):
                x_c = np.mean([boom.x for boom in section.booms])
                lower_boom = min(section.booms, key=lambda b: b.y)
                z_c = lower_boom.y
                weight = weight_per_section[i]
                ax.quiver(
                    x_c,
                    y_pos,
                    z_c,
                    0,
                    0,
                    -weight / arrow_scale,
                    color="blue",
                    arrow_length_ratio=0.2,
                    linewidth=2,
                    alpha=0.7,
                    label="Weight" if i == 0 else None,
                )

        # --- Add drag arrows (if provided) ---
        if drag_per_section:
            arrow_scale = (
                max(drag_per_section) / 0.1 if max(drag_per_section) != 0 else 1.0
            )
            for i, (y_pos, section) in enumerate(self.sections):
                # Place drag arrow at mean chordwise position (x), at y=y_pos, z=mean boom height
                x_c = np.mean([boom.x for boom in section.booms])
                z_c = np.mean([boom.y for boom in section.booms])
                drag = drag_per_section[i]
                ax.quiver(
                    x_c,
                    y_pos,
                    z_c,
                    drag / arrow_scale,
                    0,
                    0,  # Arrow in +x (chordwise) direction
                    color="green",
                    arrow_length_ratio=0.2,
                    linewidth=2,
                    alpha=0.7,
                    label="Drag" if i == 0 else None,
                )

        if point_loads:
            for pl in point_loads:
                x = pl.get("x", 0)
                y = pl["y"]
                z = pl.get("z", 0)
                Pz = pl.get("Pz", 0)
                ax.quiver(
                    x,
                    y,
                    z,  # base at (x, y, z)
                    0,
                    0,
                    Pz / arrow_scale,  # direction: vertical, scaled for visibility
                    color="red",
                    arrow_length_ratio=0.2,
                    linewidth=3,
                    alpha=0.9,
                    label="Point Load",
                )

        # --- Plot connectors ---
        if connector_sections and connector_stresses_per_section:
            for conn, stresses in zip(
                connector_sections, connector_stresses_per_section
            ):
                x = conn.get("x", 0)
                y = conn.get("y", 0)
                z = conn.get("z", 0)
                section = conn.get("section", None)
                if section:
                    for boom, stress in zip(section.booms, stresses):
                        color = cmap(norm(stress))
                        ax.scatter(
                            x + boom.x,
                            y,
                            z + boom.y,
                            color=color,
                            s=boom.area * 1e6 * 20,
                            marker="o",
                            alpha=0.8,
                        )

        ax.set_title("3D Wing Structure (Color-coded by Local Stress)")
        ax.set_xlabel("x [m] (chordwise)")
        ax.set_ylabel("y [m] (spanwise)")
        ax.set_zlabel("z [m] (vertical)")
        ax.grid(True)

        # Add color bar for stress
        mappable = cm.ScalarMappable(norm=norm, cmap=cmap)
        mappable.set_array(all_stresses)
        cbar = plt.colorbar(mappable, ax=ax, pad=0.1)
        cbar.set_label("Bending Stress [Pa]")

        # Add legend for arrows
        handles = []
        if lift_per_section:
            handles.append(plt.Line2D([0], [0], color="red", lw=2, label="Lift"))
        if drag_per_section:
            handles.append(plt.Line2D([0], [0], color="green", lw=2, label="Drag"))
        if weight_per_section:
            handles.append(plt.Line2D([0], [0], color="blue", lw=2, label="Weight"))
        if connector_sections:
            handles.append(
                plt.Line2D(
                    [0],
                    [0],
                    color="magenta",
                    marker="D",
                    linestyle="",
                    label="Connector",
                )
            )
        if handles:
            ax.legend(handles=handles, loc="upper left")

        ax.xaxis.set_major_locator(ticker.MaxNLocator(4))
        ax.yaxis.set_major_locator(ticker.MaxNLocator(4))
        ax.zaxis.set_major_locator(ticker.MaxNLocator(4))

        ax.set_xlim(-1, 1)
        ax.set_ylim(0, self.span / 2)
        ax.set_zlim(-0.2, 0.2)

        plt.tight_layout()
        plt.show()

    def plot_deformed_wing(self, vertical_deflections: list[float]):
        import matplotlib.colors as mcolors
        from matplotlib import cm

        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

        # Normalize deflections for coloring
        min_defl = min(vertical_deflections)
        max_defl = max(vertical_deflections)
        norm = mcolors.Normalize(vmin=min_defl, vmax=max_defl)
        cmap = plt.colormaps["plasma"]

        for i, (y_pos, section) in enumerate(self.sections):
            dz = vertical_deflections[i]
            color = cmap(norm(dz))
            for boom in section.booms:
                x = boom.x
                y = y_pos
                z = boom.y + dz  # add deflection to original z
                ax.scatter(x, y, z, color=color, s=boom.area * 1e6 * 20)

        ax.set_title(
            "Vertical Deflection of Wing Structure (Booms colored by Deflection)"
        )
        ax.set_xlabel("x [m] (chordwise)")
        ax.set_ylabel("y [m] (spanwise)")
        ax.set_zlabel("z [m] (vertical)")
        ax.grid(True)

        # Add color bar for deflection
        mappable = cm.ScalarMappable(norm=norm, cmap=cmap)
        mappable.set_array(vertical_deflections)
        cbar = plt.colorbar(mappable, ax=ax, pad=0.1)
        cbar.set_label("Vertical Deflection [m]")

        ax.xaxis.set_major_locator(ticker.MaxNLocator(4))
        ax.yaxis.set_major_locator(ticker.MaxNLocator(4))
        ax.zaxis.set_major_locator(ticker.MaxNLocator(4))

        ax.set_xlim(-1, 1)
        ax.set_ylim(0, self.span / 2)
        ax.set_zlim(-0.2, 0.2)

        plt.tight_layout()
        plt.show()