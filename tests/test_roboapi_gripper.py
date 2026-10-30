"""CPU ActionTerm checks with synthetic calibration; no Isaac application."""
import importlib.util
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest
torch = pytest.importorskip("torch")

MODULE = Path(__file__).resolve().parents[1] / 'policies/roboapi/gripper_action.py'


@pytest.fixture
def gripper_module(monkeypatch):
    # Avoid Isaac imports/GPU initialization, retaining actual torch CPU buffers.
    managers = ModuleType('isaaclab.managers')
    class ActionTerm:
        def __init__(self, cfg, env):
            self._env = env
            self._asset = env.scene[cfg.asset_name]
            self.num_envs, self.device = env.num_envs, env.device
    managers.ActionTerm, managers.ActionTermCfg = ActionTerm, object
    utils = ModuleType('isaaclab.utils')
    utils.configclass = lambda cls: cls
    monkeypatch.setitem(sys.modules, 'isaaclab.managers', managers)
    monkeypatch.setitem(sys.modules, 'isaaclab.utils', utils)
    spec = importlib.util.spec_from_file_location('gripper_cpu_test', MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    cfg = {
        'arm': {'delay_s': 0., 'tau_s': 0.},
        'gripper': {'delay_s': .1, 'tau_s': .06, 'open_raw': 3., 'closed_raw': 229.,
                    'max_close_raw_per_s': 340., 'max_open_raw_per_s': 320.,
                    'command_period_s': .05, 'sensor_period_s': .15, 'sensor_quantum_raw': 1.}}
    import tempfile
    with tempfile.TemporaryDirectory() as folder:
        file = Path(folder) / 'actuator.json'
        file.write_text(json.dumps(cfg))
        module.ACTUATOR_FILE = str(file)
        yield module


class FakeAsset:
    def __init__(self, n):
        self.data = SimpleNamespace(joint_pos=torch.zeros((n, 5), dtype=torch.float32))
        self.commands = []
    def find_joints(self, names):
        assert names == ['positive_outer_joint']
        return [2], names
    def set_joint_position_target(self, target, joint_ids):
        assert joint_ids == [2]
        self.commands.append(target.clone())


def make_term(module, n=1, now=0.):
    asset = FakeAsset(n)
    env = SimpleNamespace(scene={'robot': asset}, num_envs=n, device='cpu',
                          sim=SimpleNamespace(current_time=now))
    cfg = SimpleNamespace(asset_name='robot', actuator_path=module.ACTUATOR_FILE)
    return module.CalibratedGripperAction(cfg, env), env, asset


def test_sensor_is_sampled_measured_joint_and_process_actions_never_advances_time(gripper_module):
    term, env, asset = make_term(gripper_module)
    term.process_actions(torch.tensor([[.2]]))
    assert term._responses[0].last == pytest.approx(-1/240) and not asset.commands
    assert term.observe().item() == 1.
    asset.data.joint_pos[0, 2] = -.79 * .5
    env.sim.current_time = .04
    assert term.observe().item() == 1.  # First post-settle sample is at 50 ms.
    env.sim.current_time = .05
    assert term.observe().item() == pytest.approx(.5)
    term.apply_actions()
    assert term.processed_actions.item() == 0.  # Initial command delay remains active.
    assert term.observe().item() == pytest.approx(.5)  # Target is not reported as measured opening.


@pytest.mark.parametrize('value', [-.001, 1.001, float('nan'), float('inf')])
def test_rejects_invalid_continuous_actions(gripper_module, value):
    term, _, _ = make_term(gripper_module)
    with pytest.raises(ValueError):
        term.process_actions(torch.tensor([[value]]))
    assert term.raw_actions.item() == 1.


def test_reset_clears_pending_commands_and_handles_global_or_rewound_clock(gripper_module):
    term, env, asset = make_term(gripper_module, n=2, now=10.)
    term.process_actions(torch.tensor([[.2], [.8]]))
    for tick in range(80):
        env.sim.current_time = 10. + tick/240
        term.apply_actions()
    previous_peer = term._responses[1]
    term.reset(torch.tensor([0]))
    assert term._responses[1] is previous_peer
    assert term._responses[0].last == pytest.approx(env.sim.current_time - 1/240)
    assert all(raw == 3. for _, raw in term._responses[0].grip)
    assert term._responses[0].next_sensor == pytest.approx(env.sim.current_time + .05)
    assert term.raw_actions[0].item() == 1. and term.processed_actions[0].item() == 0.
    term.apply_actions()  # Ordinary env reset keeps the global clock advancing.
    env.sim.current_time = 0.
    term.reset()
    term.observe()
    term.apply_actions()  # Full simulation reset can rewind the global clock safely.
    assert all(response.last == 0. for response in term._responses)


def test_reset_preserves_encoder_phase_and_frozen_snapshot(gripper_module):
    term, env, asset = make_term(gripper_module, now=10.)
    term.reset()
    term.observe()
    response = term._responses[0]
    assert response.next_sensor == pytest.approx(10.05)
    assert response.last == pytest.approx(10. - 1/240)
    assert response is not term._settled_controller.response
    asset.data.joint_pos[0, 2] = -.395
    env.sim.current_time = 10.05
    assert term.observe().item() == pytest.approx(.5)
    term.process_actions(torch.tensor([[.2]]))
    term.apply_actions()
    assert term._settled_controller.response.sensor_time == 1.
    env.sim.current_time = 27.
    term.reset()
    assert term._responses[0].next_sensor == pytest.approx(27.05)
    assert term._responses[0].sensor_raw == 3.
