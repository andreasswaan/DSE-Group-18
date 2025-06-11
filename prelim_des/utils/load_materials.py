from __future__ import annotations
from structure import Material

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