import os
from pathlib import Path

ROOT_DIR = os.path.join(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir), os.pardir
)
CAN_DBC_DIR = os.path.join(ROOT_DIR, "dbc")
DEFAULT_GENERATED_DIR = os.path.join(ROOT_DIR, "build")
REG_PATH = Path(os.path.join(DEFAULT_GENERATED_DIR, "generated/enum_registry.json"))
