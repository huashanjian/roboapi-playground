"""Local RoboAPI and calibrated rig configuration; safe to import without Isaac."""
import os
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]

def path(name, default):
    return Path(os.environ.get(name, str(default))).expanduser().resolve()

WORK = path('ROBOAPI_WORK', REPO / '.local/roboapi')
RIG_DIR = path('ROBOAPI_RIG_DIR', WORK / 'rig')
CALIBRATION = path('ROBOAPI_CALIBRATION', RIG_DIR / 'camera.json')
ACTUATOR = path('ROBOAPI_ACTUATOR', RIG_DIR / 'actuator.json')
ROBOT_USD = path('ROBOAPI_ROBOT_USD', RIG_DIR / 'franka_robotiq.usd')
PANDA_USD = path('ROBOAPI_PANDA_USD', WORK / 'assets/panda_instanceable.usd')
ASSEMBLY_USD = path('ROBOAPI_ASSEMBLY_USD', RIG_DIR / 'assembly.usdc')
ASSEMBLY_ROOT = os.environ.get('ROBOAPI_ASSEMBLY_ROOT', '/Robot').rstrip('/')
CALIBRATED_CAMERA_PRIM = os.environ.get('ROBOAPI_CAMERA_PRIM', ASSEMBLY_ROOT + '/poke_wrist_camera_link/WristCamera')
TOKEN_FILE = path('ROBOAPI_TOKEN_FILE', WORK / 'credentials/api-key')
BASE_URL = os.environ.get('ROBOAPI_BASE_URL', 'https://api.pokebot.com/v1').rstrip('/')
MODEL = os.environ.get('ROBOAPI_MODEL', 'poke-aurora-preview')
RUN_NAME = os.environ.get('ROBOAPI_RUN_NAME', 'roboapi_full120_10seeds_120s')
GROUP = REPO / 'output' / RUN_NAME
