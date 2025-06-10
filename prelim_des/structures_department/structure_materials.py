from prelim_des.utils.import_toml import load_toml


# === CONFIG & MATERIALS ===

toml = load_toml()

class Material:
    def __init__(
        self,
        name: str,
        E: float,
        G: float,
        density: float,
        yield_strength: float,
        uts: float,
        tau_max: float,
        epsilon_max: float,
    ):
        self.name = name
        self.E = E * 1e9  # GPa → Pa
        self.G = G * 1e9  # GPa → Pa
        self.density = density * 1000  # g/cm³ → kg/m³
        self.yield_strength = yield_strength * 1e6  # MPa → Pa
        self.uts = uts * 1e6  # MPa → Pa
        self.tau_max = tau_max * 1e6  # MPa → Pa
        self.epsilon_max = epsilon_max  # Unitless

    def __repr__(self):
        return f"Material({self.name}, E={self.E:.2e} Pa, ρ={self.density} kg/m³)"


def load_materials(toml: dict) -> dict[str, Material]:
    material_dict = {}
    all_materials = toml["config"]["material"]
    for mat_name, props in all_materials.items():
        # Use only mat_E_1 if present, else mat_E, else mat_E_2
        if "mat_E_1" in props:
            E = props["mat_E_1"]
        elif "mat_E" in props:
            E = props["mat_E"]
        elif "mat_E_2" in props:
            E = props["mat_E_2"]
        else:
            raise ValueError(f"Material {mat_name} missing modulus E")

        # Use only mat_delta_y_1 if present, else mat_delta_y, else mat_delta_y_2
        if "mat_delta_y_1" in props:
            yield_strength = props["mat_delta_y_1"]
        elif "mat_delta_y" in props:
            yield_strength = props["mat_delta_y"]
        elif "mat_delta_y_2" in props:
            yield_strength = props["mat_delta_y_2"]
        else:
            yield_strength = 0  # or raise error

        material = Material(
            name=mat_name,
            E=E,
            G=props.get("mat_G", 0),
            density=props["mat_rho"],
            yield_strength=yield_strength,
            uts=props.get("mat_delta_uts", 0),
            tau_max=props.get("mat_tau_max", 0),
            epsilon_max=props.get("mat_epsilon_max", 0),
        )
        material_dict[mat_name] = material
    return material_dict


materials = load_materials(toml)