import matplotlib.pyplot as plt
from matplotlib import ticker
import numpy as np


# Plotting functions for fuselage and wing structures

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