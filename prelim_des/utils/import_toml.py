import toml
import os
from globals import main_dir

file_path = os.path.join(main_dir, "prelim_des/config.toml")


def load_toml() -> dict:
    with open(file_path, "r") as f:
        config = toml.load(f)

    return config


if __name__ == "__main__":
    config = load_toml()
    print(config)
