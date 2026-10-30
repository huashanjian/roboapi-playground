"""CPU/offline regression tests; fixtures are synthetic, not experiment results."""
import importlib.util
import json
from pathlib import Path

import h5py
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from tools.roboapi_eval import tracking as m


def pose_at(x):
    return np.array([x, 0, 0, 1, 0, 0, 0], dtype=float)


def test_quaternion_sign_and_known_rotation():
    q = np.array([[1., 0, 0, 0]])
    assert m.orientation_error(q, -q)[0] == 0
    r = Rotation.from_euler('z', 90, degrees=True).as_quat()[[3, 0, 1, 2]][None]
    assert m.orientation_error(q, r)[0] == pytest.approx(90)


def test_executed_edges_boundaries_and_pooling():
    poses = np.stack([pose_at(x) for x in (0., .1, .3, .6)])
    actions = np.c_[poses, [.0, .2, .8, 1.]]
    measured = poses.copy()
    measured[:, 0] += [.0, .01, .02, .03]
    measured[:, 3:] *= -1
    a = m.trajectory_arrays(actions, measured, poses, [0, 2], poses[[0, 1]])
    np.testing.assert_allclose(a['position_error_m'], [0, .01, .02, .03])
    np.testing.assert_allclose(a['orientation_error_deg'], 0)
    np.testing.assert_allclose(a['policy_within_chunk_translation_m'], [.1, .3])
    np.testing.assert_allclose(a['policy_between_chunk_translation_m'], [.2])
    assert m.summarize(a)['gripper_command_fractions']['strict_interior_0_to_1'] == .5
    # Unequal episode lengths: quantile of all samples, not average of medians.
    b = {k: np.array([1.]) for k in m.METRICS}
    pooled = m.pool([a, b])
    assert len(pooled['policy_step_translation_m']) == 4  # 3 + 1; no invented reset edge.
    assert m.summarize(pooled)['position_error_m']['median'] == pytest.approx(.02)
    assert m.summarize(pooled)['position_error_m']['median'] != pytest.approx((.015+1)/2)


def make_fixture(tmp_path):
    task = 'SyntheticTask'
    ep = tmp_path/task
    api = ep/'run_0_api'
    api.mkdir(parents=True)
    native = np.stack([pose_at(x) for x in (0, .01, .02, .03, .04, 100)])
    base = Rotation.from_euler('z', -90, degrees=True)
    policy = native.copy()
    policy[:, :3] = base.apply(native[:, :3])
    policy[:, 3:] = (base * m.rotation(native[:, 3:])).as_quat()[:, [3, 0, 1, 2]]
    actions = np.c_[native, [.2, .5, .8, 0, 1, .5]].astype(np.float32)
    actual = native[:5].copy()
    actual[:, 1] += .001
    actual[:, 3:] *= -1
    def write(path, obj):
        path.write_text(json.dumps(obj)+'\n')
    chunks = []
    for index, start in enumerate((0, 3)):
        chunk = {'sim_actions': actions[start:start+3].tolist(), 'arm_wxyz': policy[start:start+3].tolist(),
                 'gripper_open': actions[start:start+3, -1].tolist(), 'sim_actions_frame': 'robot-root tool0',
                 'request_id': f'request-{index}'}
        chunks.append(chunk)
        write(api/f'chunk_{index:04}.json', chunk)
        obs = native[0] if index == 0 else actual[2]
        obs_state = np.r_[base.apply(obs[:3]), (base*m.rotation(obs[3:])).as_quat()[[3, 0, 1, 2]]]
        write(api/f'observation_{index:04}.json', {'native_tool_pose_wxyz': obs.tolist(), 'pose_wxyz': obs_state.tolist()})
    receipts = [{'event': 'execution_feedback', 'request_id': 'request-0', 'executed_steps': 3, 'completed': True},
                {'event': 'execution_feedback', 'request_id': 'request-1', 'executed_steps': 2, 'completed': False}]
    (api/'events.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in receipts))
    with h5py.File(ep/'run_0.hdf5', 'w') as f:
        g = f.create_group('data/demo_0')
        g.create_dataset('actions', data=actions[:5])
        g.create_dataset('ee_pose/position', data=actual[:, :3])
        g.create_dataset('ee_pose/orientation', data=actual[:, 3:])
    write(ep/'env_cfg.json', {'recorders': {'record_ee_pose': {'ee_body_name': 'tool0'}}})
    write(api/'summary.json', {'status': 'closed', 'executed_steps': 5})
    official = {'success': True, 'final_step': 5, 'dt': 1/30}
    write(ep/'log_0_env0.json', official)
    row = {'task_name': task, 'episode': 0, 'run': 0, 'seed': 7, 'episode_step': 5,
           'dt': 1/30, 'duration': 5/30, 'success': True, 'score': 1.}
    write(tmp_path/'episode_results.jsonl', row)
    write(tmp_path/'manifest.json', {'fixture': 'synthetic'})
    write(tmp_path/'run_metadata.json', {'manifest_sha256': m.digest(tmp_path/'manifest.json')})
    return row, api


def test_completed_success_partial_final_chunk_and_no_unused_jump(tmp_path):
    row, api = make_fixture(tmp_path)
    report = m.build_report(tmp_path, require_episodes=1)
    assert report['delivery_and_frame_checks_passed']
    ep = report['episodes'][0]
    assert ep['official_success'] is True
    assert ep['unused_final_action_rows'] == 1
    assert ep['final_chunk_executed_rows'] == 2
    assert ep['metrics']['policy_step_translation_m']['count'] == 4
    assert ep['metrics']['policy_step_translation_m']['max'] == pytest.approx(.01)
    assert ep['metrics']['position_error_m']['median'] == pytest.approx(.001)
    assert ep['delivery_and_frame_checks']['logged_action_vs_executed_HDF5_max_abs_error'] == 0
    assert report['aggregate']['metrics']['gripper_command_fractions']['strict_interior_0_to_1'] == .6


def test_command_delivery_mutation_does_not_hide_physical_tracking(tmp_path):
    row, api = make_fixture(tmp_path)
    path = api/'chunk_0000.json'
    chunk = json.loads(path.read_text())
    chunk['sim_actions'][0][0] += .1
    path.write_text(json.dumps(chunk))
    report = m.build_report(tmp_path)
    assert report['delivery_and_frame_checks_passed'] is False
    assert report['failures'][0]['error'] == 'delivery_or_frame_check_failed'
    assert report['episodes'][0]['metrics']['position_error_m']['median'] == pytest.approx(.001)


def test_receipt_mismatch_rejected(tmp_path):
    row, api = make_fixture(tmp_path)
    lines = (api/'events.jsonl').read_text().splitlines()
    receipt = json.loads(lines[-1])
    receipt['executed_steps'] = 1
    lines[-1] = json.dumps(receipt)
    (api/'events.jsonl').write_text('\n'.join(lines)+'\n')
    with pytest.raises(ValueError, match='feedback count'):
        m.build_report(tmp_path)


def test_count_gate_duplicate_and_incomplete_ledger(tmp_path):
    row, api = make_fixture(tmp_path)
    with pytest.raises(ValueError, match='Expected exactly 10'):
        m.build_report(tmp_path, require_episodes=10)
    ledger = tmp_path/'episode_results.jsonl'
    original = ledger.read_text()
    ledger.write_text(original+'{"task_name":')
    report = m.build_report(tmp_path)
    assert report['selection']['completed_episodes_in_report'] == 1
    assert len(report['notes']) == 1
    ledger.write_text(original+original)
    with pytest.raises(ValueError, match='Duplicate'):
        m.build_report(tmp_path)
