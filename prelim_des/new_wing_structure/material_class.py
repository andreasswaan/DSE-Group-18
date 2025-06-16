from prelim_des.utils.import_toml import load_toml

toml = load_toml()


class Material:

    def __init__(self, material: str):
        self.material = material
        self.density = toml["config"]["material"][material]["mat_rho"] * 1000
        self.mat_stress_uts = (
            toml["config"]["material"][material]["mat_delta_uts"] * 10**6
        )
        self.E = toml["config"]["material"][material]["mat_E"] * 10**9

        self.mat_tau_max = toml["config"]["material"][material]["mat_E"] * 10**6
