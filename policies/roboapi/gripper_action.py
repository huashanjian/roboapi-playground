"""Run the assembly gripper actuator and encoder model at every physics tick."""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import torch
from isaaclab.managers import ActionTerm, ActionTermCfg
from isaaclab.utils import configclass

from policies.roboapi.gripper_response import (
    INITIAL_SETTLE_S, PHYSICS_DT, MAX_ANGLE, restored_controller, ActuatorResponse,
)

from policies.roboapi.config import ACTUATOR

ACTUATOR_FILE = str(ACTUATOR)


class CalibratedGripperAction(ActionTerm):
    """Continuous calibrated travel; preserve assembly delay, limits and sampling."""

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._joint_ids, _ = self._asset.find_joints(['positive_outer_joint'])
        if len(self._joint_ids) != 1:
            raise RuntimeError('Expected exactly one assembly Robotiq master joint')
        self._raw_actions = torch.ones((self.num_envs, 1), device=self.device)
        self._processed_actions = torch.zeros_like(self._raw_actions)
        self._response_cfg = json.loads(Path(cfg.actuator_path).read_text())
        # Prewarm one second of open-gripper response history before capture.
        # This retains command and encoder phase across environment resets.
        settled = ActuatorResponse(np.zeros(0), 1., self._response_cfg)
        for tick in range(round(INITIAL_SETTLE_S / PHYSICS_DT)):
            now = tick * PHYSICS_DT
            settled.observe(now, 0., MAX_ANGLE)
            settled.step(now, np.zeros(0), 1.)
        # Sample the encoder at the post-settle time.
        settled.observe(INITIAL_SETTLE_S, 0., MAX_ANGLE)
        self._settled_controller = SimpleNamespace(response=settled, command_time=None)
        now = float(self._env.sim.current_time)
        self._responses = [self._restored_response(now) for _ in range(self.num_envs)]

    def _restored_response(self, now):
        # Deepcopy the settled state and shift all four
        # clocks plus queued command deadlines, preserving their relative phase.
        return restored_controller(self._settled_controller, INITIAL_SETTLE_S, now).response

    @property
    def action_dim(self):
        return 1

    @property
    def raw_actions(self):
        return self._raw_actions

    @property
    def processed_actions(self):
        return self._processed_actions

    def process_actions(self, actions):
        if not torch.isfinite(actions).all() or ((actions < 0) | (actions > 1)).any():
            raise ValueError('Calibrated gripper action must be finite calibrated travel in [0,1]')
        self._raw_actions[:] = actions

    def _measured_angles(self):
        return np.clip(-self._asset.data.joint_pos[:, self._joint_ids[0]].detach().cpu().numpy(),
                       0., MAX_ANGLE)

    def observe(self):
        now = float(self._env.sim.current_time)
        measured = self._measured_angles()
        values = [response.observe(now, float(measured[i]), MAX_ANGLE)
                  for i, response in enumerate(self._responses)]
        return torch.tensor(values, dtype=torch.float32, device=self.device).unsqueeze(1)

    def apply_actions(self):
        # Sample measured travel before commanding on each physics tick.
        now = float(self._env.sim.current_time)
        measured = self._measured_angles()
        requested = self._raw_actions.detach().cpu().numpy()[:, 0]
        for i, response in enumerate(self._responses):
            response.observe(now, float(measured[i]), MAX_ANGLE)
            _, travel = response.step(now, np.zeros(0), float(requested[i]))
            self._processed_actions[i, 0] = -response.angle(travel, MAX_ANGLE)
        self._asset.set_joint_position_target(self._processed_actions, joint_ids=self._joint_ids)

    def reset(self, env_ids=None):
        if env_ids is None:
            indices = list(range(self.num_envs))
        elif isinstance(env_ids, slice):
            indices = list(range(self.num_envs))[env_ids]
        else:
            indices = torch.as_tensor(env_ids).cpu().tolist()
        now = float(self._env.sim.current_time)
        for i in indices:
            self._responses[i] = self._restored_response(now)
            self._raw_actions[i] = 1.
            self._processed_actions[i] = 0.


@configclass
class CalibratedGripperActionCfg(ActionTermCfg):
    class_type = CalibratedGripperAction
    asset_name: str = 'robot'
    actuator_path: str = ACTUATOR_FILE


def observed_gripper_open(env):
    return env.action_manager.get_term('gripper_action').observe()
