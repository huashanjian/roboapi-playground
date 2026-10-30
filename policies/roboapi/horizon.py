"""Explicit full-benchmark task selection and simulation-horizon validation."""
import json
import math
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
ACTION_DT = 1 / 30
CHUNK_LENGTH = 40


def load_task_catalog():
    root = REPO / 'robolab/tasks'
    items = json.loads((root / '_metadata/task_metadata.json').read_text())
    names = [item['task_name'] for item in items]
    files = [item['filename'] for item in items]
    actual = {str(path.relative_to(root)) for path in (root / 'benchmark').glob('*.py')}
    if len(items) != 120 or len(set(names)) != 120 or set(files) != actual:
        raise ValueError('Official catalog must cover exactly all 120 benchmark sources')
    return sorted(items, key=lambda item: item['task_name'])


def validate_horizon(seconds, max_chunks):
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError('Episode duration must be finite and positive')
    steps = round(seconds / ACTION_DT)
    if not math.isclose(steps * ACTION_DT, seconds, abs_tol=1e-9):
        raise ValueError('Episode duration must be an integral number of 30 Hz steps')
    if type(max_chunks) is not int or max_chunks * CHUNK_LENGTH < steps:
        raise ValueError('Action chunk budget would end before the simulation timeout')
    return steps


def apply_horizon(cfg, seconds, max_chunks):
    validate_horizon(seconds, max_chunks)
    cfg._original_episode_length_s = cfg.episode_length_s
    cfg.episode_length_s = seconds


def verify_horizon(env, cfg, seconds, max_chunks):
    expected_steps = validate_horizon(seconds, max_chunks)
    dt = cfg.sim.dt * cfg.decimation
    if not math.isclose(dt, ACTION_DT, rel_tol=0, abs_tol=1e-9):
        raise ValueError(f'Unexpected control interval: {dt}')
    if cfg.episode_length_s != seconds or env.max_episode_length != expected_steps:
        raise ValueError('Instantiated environment does not use the requested horizon')
    return {
        'episode_length_s': seconds,
        'original_official_episode_length_s': cfg._original_episode_length_s,
        'max_episode_steps': int(env.max_episode_length),
        'action_dt_s': dt, 'physics_dt_s': cfg.sim.dt,
        'max_chunks': max_chunks, 'chunk_length': CHUNK_LENGTH,
        'timeout_override_applied_before_env_construction': True,
        'passed': True,
    }
