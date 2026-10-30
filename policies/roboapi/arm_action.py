"""Official Franka high-PD IK with arm-only compensation for the assembly tool.

The original RoboLab high-PD arm has gravity disabled. The copied tool keeps
its assembly gravity, mass and inertia. Apply its gravity wrench through the
tool0 Jacobian to the seven arm joints, leaving all gripper efforts untouched.
This uses robot model/proprioception only, never task-object state.
"""
import torch
from isaaclab.envs.mdp.actions.task_space_actions import DifferentialInverseKinematicsAction
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from isaaclab.utils import configclass


class PayloadCompensatedIKAction(DifferentialInverseKinematicsAction):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        if not self._asset.is_fixed_base or self._num_joints != 7 or self._body_name != 'tool0':
            raise ValueError('Payload compensation requires the fixed-base seven-joint Franka tool0 action')
        self._payload_ids = [i for i, name in enumerate(self._asset.body_names)
                             if not name.startswith('panda_link')]
        if len(self._payload_ids) != 9:
            raise ValueError('Expected the nine unchanged assembly tool/gripper/camera rigid bodies')
        self._payload_masses = self._asset.root_physx_view.get_masses()[:, self._payload_ids].to(self.device)
        self._gravity_w = torch.tensor(env.cfg.sim.gravity, dtype=torch.float32, device=self.device)
        self._last_payload_effort = torch.zeros((self.num_envs, 7), device=self.device)

    @property
    def jacobian_w(self):
        # PhysX provides each link's COM spatial Jacobian. Both our IK target
        # and payload wrench are expressed at the tool0 actor/flange origin.
        # v_actor = v_com + omega x (p_actor - p_com).
        jacobian = super().jacobian_w.clone()
        offset = (self._asset.data.body_link_pos_w[:, self._body_idx]
                  - self._asset.data.body_com_pos_w[:, self._body_idx])
        angular = jacobian[:, 3:, :].transpose(1, 2)
        jacobian[:, :3, :] += torch.cross(
            angular, offset[:, None, :].expand_as(angular), dim=-1).transpose(1, 2)
        return jacobian

    def apply_actions(self):
        super().apply_actions()
        tool_position = self._asset.data.body_link_pos_w[:, self._body_idx]
        centers = self._asset.data.body_com_pos_w[:, self._payload_ids]
        forces = self._payload_masses.unsqueeze(-1) * self._gravity_w
        moments = torch.cross(centers - tool_position.unsqueeze(1), forces, dim=-1)
        wrench = torch.cat((forces.sum(dim=1), moments.sum(dim=1)), dim=-1)
        self._last_payload_effort = -torch.einsum('nkj,nk->nj', self.jacobian_w, wrench)
        self._asset.set_joint_effort_target(self._last_payload_effort, joint_ids=self._joint_ids)

    def reset(self, env_ids=None):
        super().reset(env_ids)
        if hasattr(self, '_last_payload_effort'):
            rows = env_ids if env_ids is not None else slice(None)
            self._last_payload_effort[rows] = 0.
            self._asset.set_joint_effort_target(self._last_payload_effort[rows],
                joint_ids=self._joint_ids, env_ids=env_ids)


@configclass
class PayloadCompensatedIKActionCfg(DifferentialInverseKinematicsActionCfg):
    class_type = PayloadCompensatedIKAction
