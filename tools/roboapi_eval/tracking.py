#!/usr/bin/env python3
"""Offline tracking diagnostics for completed calibrated Franka rollouts.

Reads an immutable snapshot of the official result ledger, then reads only its
completed episodes. No API, Isaac, runtime adapter imports or protocol changes.
Only the selected diagnostic JSON is written. This is not an acceptance audit.

Examples:
  python -m tools.roboapi_eval.tracking --run OUTPUT --max-episodes 1
  python -m tools.roboapi_eval.tracking --run OUTPUT --task BananaInBowlTask --require-episodes 10
  python -m tools.roboapi_eval.tracking --run OUTPUT --require-episodes 30
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import h5py
import numpy as np
from scipy.spatial.transform import Rotation


METRICS = (
    'position_error_m', 'orientation_error_deg',
    'policy_step_translation_m', 'policy_step_rotation_deg',
    'policy_within_chunk_translation_m', 'policy_between_chunk_translation_m',
    'policy_within_chunk_rotation_deg', 'policy_between_chunk_rotation_deg',
    'actual_step_translation_m', 'actual_step_rotation_deg',
    'chunk_first_target_from_observation_translation_m',
    'chunk_first_target_from_observation_rotation_deg',
    'gripper_open_command',
)


def read(path):
    return json.loads(path.read_text())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rotation(wxyz):
    return Rotation.from_quat(np.asarray(wxyz)[..., [1, 2, 3, 0]])


def orientation_error(a, b):
    """SO(3) geodesic degrees, invariant to quaternion normalization/sign."""
    return (rotation(a).inv() * rotation(b)).magnitude() * 180 / np.pi


def distribution(values):
    values = np.asarray(values, dtype=float)
    if values.size == 0:
        return {'count': 0}
    return {
        'count': int(values.size), 'mean': float(values.mean()),
        'rms': float(np.sqrt(np.mean(values * values))),
        'min': float(values.min()), 'median': float(np.median(values)),
        'p90': float(np.percentile(values, 90)), 'p95': float(np.percentile(values, 95)),
        'p99': float(np.percentile(values, 99)), 'max': float(values.max()),
    }


def summarize(arrays):
    result = {name: distribution(arrays[name]) for name in METRICS}
    p = arrays['position_error_m']
    g = arrays['gripper_open_command']
    result['position_error_below_threshold_fraction'] = {
        str(threshold): float(np.mean(p < threshold)) if p.size else None
        for threshold in (.001, .005, .01, .02, .05)}
    result['gripper_command_fractions'] = {
        'closed_exactly_0': float(np.mean(g == 0)) if g.size else None,
        'open_exactly_1': float(np.mean(g == 1)) if g.size else None,
        'strict_interior_0_to_1': float(np.mean((g > 0) & (g < 1))) if g.size else None,
        'interior_0_01_to_0_99': float(np.mean((g > .01) & (g < .99))) if g.size else None,
    }
    return result


def trajectory_arrays(actions, actual, policy, chunk_starts, native_observations):
    """Calculate metrics on executed rows only; transitions never cross episodes."""
    actions = np.asarray(actions, dtype=float)
    actual = np.asarray(actual, dtype=float)
    policy = np.asarray(policy, dtype=float)
    n = len(actions)
    if actions.shape != (n, 8) or actual.shape != (n, 7) or policy.shape != (n, 7) or n == 0:
        raise ValueError('Expected nonempty executed actions (N,8), actual (N,7), policy (N,7).')
    if not all(np.isfinite(x).all() for x in (actions, actual, policy)):
        raise ValueError('Nonfinite action, actual pose or policy pose.')
    if any(np.any(np.linalg.norm(x, axis=1) < 1e-8) for x in
           (actions[:, 3:7], actual[:, 3:], policy[:, 3:])):
        raise ValueError('Invalid zero-norm pose quaternion.')
    starts = np.asarray(chunk_starts, dtype=int)
    native = np.asarray(native_observations, dtype=float)
    if (starts.ndim != 1 or not starts.size or starts[0] != 0 or
        np.any(np.diff(starts) <= 0) or starts[-1] >= n or native.shape != (len(starts), 7)):
        raise ValueError('Chunk boundaries and native observations do not align with executed actions.')
    if not np.isfinite(native).all() or np.any(np.linalg.norm(native[:, 3:], axis=1) < 1e-8):
        raise ValueError('Invalid native observation pose.')
    between = np.zeros(n-1, dtype=bool)
    between[starts[1:] - 1] = True
    jump_pos = np.linalg.norm(np.diff(policy[:, :3], axis=0), axis=1)
    jump_rot = orientation_error(policy[:-1, 3:], policy[1:, 3:]) if n > 1 else np.zeros(0)
    return {
        'position_error_m': np.linalg.norm(actions[:, :3] - actual[:, :3], axis=1),
        'orientation_error_deg': orientation_error(actions[:, 3:7], actual[:, 3:]),
        'policy_step_translation_m': jump_pos,
        'policy_step_rotation_deg': jump_rot,
        'policy_within_chunk_translation_m': jump_pos[~between],
        'policy_between_chunk_translation_m': jump_pos[between],
        'policy_within_chunk_rotation_deg': jump_rot[~between],
        'policy_between_chunk_rotation_deg': jump_rot[between],
        'actual_step_translation_m': np.linalg.norm(np.diff(actual[:, :3], axis=0), axis=1),
        'actual_step_rotation_deg': orientation_error(actual[:-1, 3:], actual[1:, 3:]) if n > 1 else np.zeros(0),
        'chunk_first_target_from_observation_translation_m': np.linalg.norm(actions[starts, :3] - native[:, :3], axis=1),
        'chunk_first_target_from_observation_rotation_deg': orientation_error(actions[starts, 3:7], native[:, 3:]),
        'gripper_open_command': actions[:, -1],
    }


def maximum(values):
    return float(np.max(values)) if np.size(values) else 0.


def load_episode(run, row):
    task = row['task_name']
    episode = int(row.get('episode', row['run']))
    ep = run / task
    api = ep / f'run_{episode}_api'
    official_path = ep / f'log_{episode}_env0.json'
    hdf5_path = ep / f'run_{episode}.hdf5'
    summary = read(api/'summary.json')
    official = read(official_path)
    cfg = read(ep/'env_cfg.json')
    if cfg['recorders']['record_ee_pose']['ee_body_name'] != 'tool0':
        raise ValueError('Recorded end-effector is not tool0.')
    if summary['status'] == 'running':
        raise ValueError('Ledger episode API summary is still running; retry after closure.')
    chunks = [read(path) for path in sorted(api.glob('chunk_*.json'))]
    observations = [read(path) for path in sorted(api.glob('observation_*.json'))]
    if len(chunks) != len(observations) or not chunks:
        raise ValueError('Missing/misaligned action chunks and request observations.')
    with h5py.File(hdf5_path, 'r') as file:
        g = file['data/demo_0']
        actions = g['actions'][:].astype(float)
        actual = np.c_[g['ee_pose/position'][:], g['ee_pose/orientation'][:]].astype(float)
    n = len(actions)
    if not (n == row['episode_step'] == official['final_step'] == summary['executed_steps']):
        raise ValueError('HDF5/official ledger/log/API executed-step counts differ.')
    if (type(row['success']) is not bool or row['success'] != official['success'] or
        row['dt'] != official['dt'] or abs(n*row['dt'] - row['duration']) > 1e-6):
        raise ValueError('Official ledger/log success or timing mismatch.')
    logged = np.concatenate([c['sim_actions'] for c in chunks])
    policy_all = np.concatenate([c['arm_wxyz'] for c in chunks])
    grip = np.concatenate([c['gripper_open'] for c in chunks])
    events = [json.loads(line) for line in (api/'events.jsonl').read_text().splitlines() if line.strip()]
    receipts = [event for event in events if event.get('event') == 'execution_feedback']
    if len(receipts) != len(chunks):
        raise ValueError('Execution receipt count differs from accepted chunk count.')
    executed_counts = []
    for index, (chunk, receipt) in enumerate(zip(chunks, receipts, strict=True)):
        count = receipt['executed_steps']
        if (receipt['request_id'] != chunk['request_id'] or type(count) is not int or
            not 0 < count <= len(chunk['sim_actions']) or
            receipt['completed'] != (count == len(chunk['sim_actions'])) or
            (index < len(chunks)-1 and count != len(chunk['sim_actions']))):
            raise ValueError('Execution receipts do not describe full chunks and one optional final prefix.')
        executed_counts.append(count)
    if sum(executed_counts) != n:
        raise ValueError('Execution feedback count differs from completed HDF5/official steps.')
    starts_all = np.cumsum([0]+executed_counts[:-1])
    if len(logged) < n or policy_all.shape != (len(logged), 7) or len(grip) != len(logged):
        raise ValueError('Insufficient or malformed returned trajectory rows.')
    if starts_all[-1] >= n:
        raise ValueError('A returned chunk has no executed rows; completion needs separate review.')
    native = np.asarray([o['native_tool_pose_wxyz'] for o in observations])
    state = np.asarray([o['pose_wxyz'] for o in observations])
    arrays = trajectory_arrays(actions, actual, policy_all[:n], starts_all, native)
    base = Rotation.from_euler('z', -90, degrees=True).as_matrix()
    expected_target_r = Rotation.from_matrix(base.T @ rotation(policy_all[:, 3:]).as_matrix())
    expected_state_r = Rotation.from_matrix(base @ rotation(native[:, 3:]).as_matrix())
    checks = {
        'logged_action_vs_executed_HDF5_max_abs_error': maximum(np.abs(logged[:n]-actions)),
        'continuous_gripper_vs_API_float32_max_abs_error': maximum(np.abs(grip.astype(np.float32)-logged[:, -1])),
        'target_left_rotation_position_max_error_m': maximum(np.linalg.norm(policy_all[:, :3]@base-logged[:, :3], axis=1)),
        'target_left_rotation_orientation_max_error_deg': maximum((expected_target_r.inv()*rotation(logged[:, 3:7])).magnitude()*180/np.pi),
        'observation_left_rotation_position_max_error_m': maximum(np.linalg.norm(native[:, :3]@base.T-state[:, :3], axis=1)),
        'observation_left_rotation_orientation_max_error_deg': maximum((expected_state_r.inv()*rotation(state[:, 3:])).magnitude()*180/np.pi),
        'request_native_vs_prior_poststep_position_max_error_m': maximum(np.linalg.norm(native[1:, :3]-actual[starts_all[1:]-1, :3], axis=1)),
        'request_native_vs_prior_poststep_orientation_max_error_deg': maximum(orientation_error(native[1:, 3:], actual[starts_all[1:]-1, 3:])) if len(native)>1 else 0.,
        'continuous_gripper_in_unit_interval': bool(np.all((actions[:, -1] >= 0) & (actions[:, -1] <= 1))),
        'all_action_frames_are_robot_root_tool0': all(c['sim_actions_frame'] == 'robot-root tool0' for c in chunks),
    }
    checks_passed = all(value is True if isinstance(value, bool) else value <= (
        0 if name.startswith(('logged_action_', 'continuous_gripper_')) else 1e-4 if name.endswith('_deg') else 1e-6)
        for name, value in checks.items())
    return {
        'task': task, 'episode': episode, 'seed': row['seed'],
        'official_success': row['success'], 'official_score': row['score'],
        'executed_steps': n, 'duration_s': row['duration'], 'action_dt_s': row['dt'],
        'returned_action_rows': len(logged), 'unused_final_action_rows': len(logged)-n,
        'chunk_count': len(chunks), 'final_chunk_executed_rows': executed_counts[-1],
        'observation_poststep_boundaries_checked': len(native)-1,
        'artifact_sha256': {str(p.relative_to(run)): digest(p) for p in
                           (hdf5_path, official_path, api/'summary.json')},
        'delivery_and_frame_checks': checks, 'delivery_and_frame_checks_passed': bool(checks_passed),
        'metrics': summarize(arrays),
    }, arrays


def pool(episodes):
    return {name: np.concatenate([values[name] for values in episodes]) if episodes else np.zeros(0)
            for name in METRICS}


def build_report(run, selected_tasks=None, max_episodes=None, require_episodes=None):
    ledger_path = run/'episode_results.jsonl'
    raw = ledger_path.read_bytes()
    rows = []
    notes = []
    lines = raw.splitlines()
    for index, line in enumerate(lines):
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except ValueError:
            if index == len(lines)-1 and not raw.endswith(b'\n'):
                notes.append('Incomplete trailing ledger line deferred; only complete JSON records selected.')
                continue
            raise ValueError(f'Invalid ledger JSON at line {index+1}.')
        if selected_tasks is None or row['task_name'] in selected_tasks:
            rows.append(row)
    pairs = [(r['task_name'], r['seed']) for r in rows]
    if len(pairs) != len(set(pairs)):
        raise ValueError('Duplicate official task/seed in selected ledger snapshot.')
    if max_episodes is not None:
        rows = rows[:max_episodes]
    if require_episodes is not None and len(rows) != require_episodes:
        raise ValueError(f'Expected exactly {require_episodes} completed selected episodes, found {len(rows)}.')
    if not rows:
        raise ValueError('No completed episodes available in selected ledger snapshot.')
    episodes, by_task_arrays, by_task_episodes, failures = [], {}, {}, []
    for row in rows:
        result, arrays = load_episode(run, row)
        episodes.append(result)
        by_task_arrays.setdefault(result['task'], []).append(arrays)
        by_task_episodes.setdefault(result['task'], []).append(result)
        if not result['delivery_and_frame_checks_passed']:
            failures.append({'task': result['task'], 'seed': result['seed'], 'error': 'delivery_or_frame_check_failed'})
    all_arrays = pool([a for values in by_task_arrays.values() for a in values])
    manifest_path = run/'manifest.json'
    metadata = read(run/'run_metadata.json')
    manifest_hash = digest(manifest_path)
    if manifest_hash != metadata['manifest_sha256']:
        failures.append({'error': 'manifest_hash_differs_from_run_metadata'})
    tasks = {}
    for task, items in by_task_episodes.items():
        tasks[task] = {'episodes': len(items), 'seeds': [e['seed'] for e in items],
                       'official_successes': sum(e['official_success'] for e in items),
                       'executed_steps': sum(e['executed_steps'] for e in items),
                       'metrics': summarize(pool(by_task_arrays[task]))}
    return {
        'schema_version': 1, 'generated_utc': datetime.now(timezone.utc).isoformat(),
        'scope': 'Read-only offline diagnostics of completed official ledger episodes; not an acceptance audit or causal attribution.',
        'run_directory': str(run), 'script_sha256': digest(Path(__file__)),
        'provenance': {'manifest_sha256': manifest_hash, 'frozen_manifest_sha256': metadata['manifest_sha256'],
                       'code_snapshot_sha256': metadata.get('code_snapshot_sha256'),
                       'ledger_snapshot_sha256': hashlib.sha256(raw).hexdigest()},
        'selection': {'tasks': selected_tasks, 'max_episodes': max_episodes, 'required_episode_count': require_episodes,
                       'completed_episodes_in_report': len(episodes), 'task_counts': dict(Counter(e['task'] for e in episodes))},
        'metric_definitions': {
            'position_error_m': 'Euclidean distance between current command xyz and the post-step measured tool0 xyz in robot-root coordinates.',
            'orientation_error_deg': 'SO(3) geodesic angle between current command and post-step measured tool0, invariant to q and -q.',
            'policy_step_translation_m': 'Euclidean difference of consecutive executed accepted/decoded API xyz targets in policy coordinates; includes chunk boundaries but never episode boundaries.',
            'policy_step_rotation_deg': 'SO(3) difference of consecutive executed accepted/decoded API target orientations; includes chunk boundaries but never episode boundaries.',
            'within_vs_between_chunk': 'Between-chunk jump is from the final executed target of one chunk to the first target of the next. Within-chunk excludes these edges.',
            'chunk_first_target_from_observation': 'Distance/angle from the measured pose supplied to the request to the first target subsequently executed from that response.',
            'continuous_grip': 'Commanded opening ratio, 0 closed and 1 open; strict interior reports all values 0<g<1, with an additional 0.01<g<0.99 fraction to exclude values very near endpoints.',
            'aggregation': 'Exact quantiles over all individual valid rows/edges pooled within each task and overall; not means of episode quantiles. Longer episodes contribute more samples.',
            'quantile_method': 'numpy.percentile default linear interpolation; median is numpy.median.',
            'censoring': 'Only executed action rows included in tracking/jump metrics; unused final returned action rows are disclosed and excluded.',
        },
        'aggregate': {'episodes': len(episodes), 'executed_steps': sum(e['executed_steps'] for e in episodes),
                       'official_successes': sum(e['official_success'] for e in episodes),
                       'metrics': summarize(all_arrays)},
        'tasks': tasks, 'episodes': episodes,
        'limitations': [
            'Exact command delivery is not exact physical pose tracking. Empty-load preflight hold accuracy does not establish rollout tracking accuracy.',
            'Command versus post-step error is the residual after one 33.3ms control interval and includes normal servo lag, targets, motion, contacts and control limits; no causal attribution or retuning follows from these statistics.',
            'Only complete officially scored episodes are selected. Preserved interrupted attempts are excluded here and must remain disclosed separately in the acceptance/reliability report.',
            'Success remains the unchanged official task result. Trajectory metrics are diagnostic and do not replace success conditions.',
            'The first executed action has no preceding executed command; per-step jump count is N-1 per episode. No transition across episode resets is invented.',
            'Gripper fractions describe commands, not measured jaw gap, grasp force or delayed/quantized gripper sensor state.',
            'Coordinate self-consistency does not establish the physical validity of the flange and camera mounting.',
            'This script does not audit raw HTTP inputs, sessions, camera calibration, frozen runtime code or interruption completeness; use python -m tools.roboapi_eval.audit_full120_worker for those checks.',
        ],
        'notes': notes, 'failures': failures,
        'delivery_and_frame_checks_passed': not failures,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True, type=Path)
    parser.add_argument('--task', action='append', help='Repeat to select specific official task names.')
    parser.add_argument('--max-episodes', type=int, help='Select only the first N completed ledger episodes, for initial validation.')
    parser.add_argument('--require-episodes', type=int, help='Refuse output unless exactly N selected completed episodes exist.')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.max_episodes is not None and args.max_episodes < 1:
        parser.error('--max-episodes must be positive')
    result = build_report(args.run, args.task, args.max_episodes, args.require_episodes)
    output = args.output or args.run/'validation/tracking_summary.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'output': str(output), 'selection': result['selection'],
                       'checks_passed': result['delivery_and_frame_checks_passed'],
                       'tracking': {k: result['aggregate']['metrics'][k]
                                    for k in ('position_error_m', 'orientation_error_deg',
                                              'policy_step_translation_m', 'gripper_command_fractions')}}, indent=2))
    raise SystemExit(0 if result['delivery_and_frame_checks_passed'] else 1)


if __name__ == '__main__':
    main()
