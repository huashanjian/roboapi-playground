"""Offline tool adapter checks; use pytest --noconftest."""
import json
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from policies.roboapi.client import (
    RoboAPIInferenceClient, policy_to_tool_pose, tool_to_policy_pose,
)


def _matrix(pose):
    pose = np.asarray(pose)
    result = np.broadcast_to(np.eye(4), (*pose.shape[:-1], 4, 4)).copy()
    result[..., :3, :3] = Rotation.from_quat(pose[..., [4, 5, 6, 3]].reshape(-1, 4)).as_matrix().reshape(*pose.shape[:-1], 3, 3)
    result[..., :3, 3] = pose[..., :3]
    return result


def test_identity_tool_pose_retains_required_base_quaternion_rotation():
    tool = np.array([.3, .1, .4, 1, 0, 0, 0])
    before = tool.copy()
    h = np.sqrt(.5)
    np.testing.assert_allclose(tool_to_policy_pose(tool), [.1, -.3, .4, h, 0, 0, -h])
    np.testing.assert_allclose(policy_to_tool_pose(tool_to_policy_pose(tool)), tool, atol=1e-15)
    np.testing.assert_array_equal(tool, before)
    assert not np.shares_memory(tool, tool_to_policy_pose(tool))


def test_arbitrary_batched_poses_match_left_matrix_composition_and_roundtrip():
    rotations = Rotation.from_euler('xyz', [[20, -35, 71], [-90, 7, 123], [30, 90, -44], [180, -30, 2]], degrees=True)
    positions = np.array([[.42, -.3, .6], [-.15, .25, .45], [.12, .33, .5], [0, 0, .4]])
    q = rotations.as_quat()[:, [3, 0, 1, 2]]
    q[-1] *= -1  # Opposite quaternion sign remains the same physical orientation.
    tool = np.column_stack([positions, q]).reshape(2, 2, 7)
    base = np.eye(4)
    base[:3, :3] = Rotation.from_euler('z', -90, degrees=True).as_matrix()
    policy = tool_to_policy_pose(tool)
    np.testing.assert_allclose(_matrix(policy), base @ _matrix(tool), atol=1e-14)
    np.testing.assert_allclose(policy_to_tool_pose(policy), tool, atol=1e-14)


def test_tool_mount_is_already_in_native_pose_and_is_not_applied_twice():
    flange = np.eye(4)
    flange[:3, :3] = Rotation.from_euler('xyz', [13, 42, -75], degrees=True).as_matrix()
    flange[:3, 3] = [.35, .15, .32]
    mount = np.eye(4)
    mount[:3, :3] = Rotation.from_euler('z', 90, degrees=True).as_matrix()
    tool = flange @ mount
    q = Rotation.from_matrix(tool[:3, :3]).as_quat()[[3, 0, 1, 2]]
    policy = tool_to_policy_pose(np.r_[tool[:3, 3], q])
    np.testing.assert_allclose(_matrix(policy), np.linalg.inv(mount) @ flange @ mount, atol=1e-14)


@pytest.mark.parametrize('pose', [[0, 0, 0, 0, 0, 0, 0], [0, 0, np.nan, 1, 0, 0, 0], [0, 0, 0, 1, 0, 0]])
def test_frame_helpers_reject_invalid_poses(pose):
    with pytest.raises(ValueError):
        tool_to_policy_pose(pose)
    with pytest.raises(ValueError):
        policy_to_tool_pose(pose)


class FakePolicy:
    def __init__(self):
        self.events = []
        self.received = []
        self.session_status = {'id': None, 'status': 'closed'}
        self.fail_infer = False
        self.session_count = 0

    def connect(self):
        self.events.append('connect')

    def begin_session(self):
        self.session_count += 1
        self.session_status = {'id': f'offline-{self.session_count}', 'status': 'open'}
        self.events.append('begin')

    def infer(self, pose, gripper, image, instruction):
        self.events.append('infer')
        self.received.append((pose.copy(), gripper, image.copy(), instruction))
        if self.fail_infer:
            raise RuntimeError('private exception detail')
        return SimpleNamespace(
            arm=np.tile([.3, .1, .4, 1, 0, 0, 0], (3, 1)),
            gripper=np.array([.2, .5, .8]), action_dt=1 / 30,
            request_id='offline-request', session_id=self.session_status['id'], session_turn=1,
            request_ms=12., infer_ms=13.,
        )

    def record_execution(self, chunk, steps, *, completed):
        self.events.append(('feedback', steps, completed))

    def end_session(self, reason):
        self.events.append(('end', reason))
        self.session_status['status'] = 'closed'

    def close(self):
        self.events.append('close')


def _observation():
    return {'image_obs': {'wrist_camera': np.zeros((1, 3, 4, 3), dtype=np.uint8)},
            'proprio_obs': {'tcp_pos': np.array([[.1, .2, .3]]),
                            'tcp_quat': np.array([[1., 0, 0, 0]]),
                            'gripper_open': np.array([[.75]])}}


def _client(tmp_path, **kwargs):
    policy = FakePolicy()
    return RoboAPIInferenceClient('unused', policy_client=policy, output_dir=tmp_path, **kwargs), policy


def test_continuous_gripper_and_tool_frames_are_preserved_in_actions_and_logs(tmp_path):
    client, policy = _client(tmp_path)
    actions = []
    for i in range(3):
        actions.append(client.infer(_observation() if i == 0 else {}, 'pick')['action'])
        assert client.state['executed_steps'] == i
        client.on_step_executed()
    actions = np.asarray(actions)
    np.testing.assert_allclose(actions[:, 7], [.2, .5, .8], atol=1e-7)
    np.testing.assert_allclose(actions[:, :3], np.tile([-.1, .3, .4], (3, 1)), atol=1e-7)
    h = np.sqrt(.5)
    np.testing.assert_allclose(actions[:, 3:7], np.tile([h, 0, 0, h], (3, 1)), atol=1e-7)
    np.testing.assert_allclose(policy.received[0][0], [.2, -.1, .3, h, 0, 0, -h])
    assert policy.received[0][1] == .75
    assert policy.events.count('infer') == 1
    assert policy.events[-1] == ('feedback', 3, True)
    logged = json.loads((tmp_path / 'run_0_api/chunk_0000.json').read_text())
    assert logged['sim_actions_frame'] == 'robot-root tool0'
    np.testing.assert_allclose(logged['sim_actions'], actions)
    np.testing.assert_allclose(logged['arm_wxyz'], np.tile([.3, .1, .4, 1, 0, 0, 0], (3, 1)))
    obs = json.loads((tmp_path / 'run_0_api/observation_0000.json').read_text())
    np.testing.assert_allclose(obs['native_tool_pose_wxyz'], [.1, .2, .3, 1, 0, 0, 0])
    assert 'native_hand_pose_wxyz' not in obs
    client.close()


@pytest.mark.parametrize('executed', [0, 1, 2])
def test_partial_chunk_only_reports_acknowledged_execution_before_session_close(tmp_path, executed):
    client, policy = _client(tmp_path)
    for _ in range(executed):
        client.infer(_observation(), 'pick')
        client.on_step_executed()
    client.infer(_observation(), 'pick')  # Offered action has not succeeded in env.step.
    with pytest.raises(RuntimeError, match='acknowledgement'):
        client.infer(_observation(), 'pick')
    client.reset()
    feedback = ('feedback', executed, False)
    assert feedback in policy.events
    assert policy.events.index(feedback) < policy.events.index(('end', 'episode_reset'))
    assert client.state['executed_steps'] == executed
    client.prepare_episode(1, 'pick')
    assert policy.session_count == 2 and client.state['executed_steps'] == 0
    client.close()


def test_budget_preserves_continuous_last_action_and_does_not_infer_again(tmp_path):
    client, policy = _client(tmp_path, max_chunks=1)
    for _ in range(3):
        last = client.infer(_observation(), 'pick')['action']
        client.on_step_executed()
    assert client.budget_exhausted
    np.testing.assert_array_equal(client.infer({}, 'pick')['action'], last)
    assert last[-1] == pytest.approx(.8)
    client.on_step_executed()
    assert client.state['hold_steps'] == 1 and policy.events.count('infer') == 1
    client.close()


def test_inference_errors_remain_terminal_without_retries_or_private_exception_logging(tmp_path):
    client, policy = _client(tmp_path)
    policy.fail_infer = True
    with pytest.raises(RuntimeError):
        client.infer(_observation(), 'pick')
    with pytest.raises(RuntimeError, match='episode failed'):
        client.infer(_observation(), 'pick')
    client.close()
    assert policy.events.count('infer') == 1
    summary = (tmp_path / 'run_0_api/summary.json').read_text()
    assert 'private exception detail' not in summary
    assert json.loads(summary)['errors'] == [{'phase': 'inference', 'type': 'RuntimeError'}]


def test_http_archival_keeps_separate_transport_latency_and_excludes_auth(tmp_path, monkeypatch):
    client, _ = _client(tmp_path)
    client.begin_episode(0)
    clock = [100.]
    monkeypatch.setattr('policies.roboapi.client.time.perf_counter', lambda: clock[0])
    original_write = client._write_json
    def archive(name, value):
        clock[0] += 40
        original_write(name, value)
    monkeypatch.setattr(client, '_write_json', archive)
    request = SimpleNamespace(method='POST', url=SimpleNamespace(path='/v1/chat/completions'),
                              content=b'{"messages": []}', headers={'Authorization': 'PRIVATE'})
    client._request_hook(request)
    def read():
        clock[0] += .7
    response = SimpleNamespace(request=request, read=read, status_code=200, json=lambda: {'choices': []})
    client._response_hook(response)
    assert client.state['api_http_latency_s'] == pytest.approx([.7])
    assert client.state['api_latency_s'] == []
    assert 'PRIVATE' not in (tmp_path / 'run_0_api/http_0001_request.json').read_text()
    assert json.loads((tmp_path / 'run_0_api/http_0001_response.json').read_text())['http_status'] == 200
    client.close()
