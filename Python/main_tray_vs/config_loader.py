import os
import json

DEFAULT_CONFIG = {
    "robot": {
        "ip": "192.168.125.1",
        "socket_port": 1025,
        "rws_url": "http://192.168.125.1",
        "rws_user": "Zebastian",
        "rws_password": "robotics"
    },
    "pc": {
        "ip": "192.168.125.201"
    },
    "placement": {
        "slot_source": "camera",
        "fixed_slot_name": "slot_1",
        "fixed_slot_coords": [302.0, -508.0, 248.0]
    },
    "mug_heights": {
        "upward_z": 55.0,
        "downward_z": 45.0
    }
}


def load_config(config_path=None):
    """Load configuration from cell_config.json with safe fallbacks."""
    if config_path is None:
        here = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(here, "cell_config.json")
        if not os.path.exists(config_path):
            parent = os.path.dirname(here)
            config_path = os.path.join(parent, "cell_config.json")

    config = {k: dict(v) if isinstance(v, dict) else v for k, v in DEFAULT_CONFIG.items()}
    if os.path.exists(config_path):
        try:
            with open(config_path, "r", encoding="utf-8") as f:
                loaded = json.load(f)
                for k, v in loaded.items():
                    if isinstance(v, dict) and k in config and isinstance(config[k], dict):
                        config[k].update(v)
                    else:
                        config[k] = v
        except Exception as e:
            print(f"[CONFIG WARNING] Could not read {config_path}: {e}. Using defaults.")
    return config
