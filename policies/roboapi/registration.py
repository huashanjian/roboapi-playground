"""Official tasks with Franka arm, copied assembly Robotiq and assembly camera."""
from pathlib import Path

from policies.roboapi.horizon import load_task_catalog
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.controllers import DifferentialIKControllerCfg
from isaaclab.managers import ObservationGroupCfg, ObservationTermCfg, SceneEntityCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import subtract_frame_transforms

from policies.roboapi.camera import make_franka_camera_cfg
from policies.roboapi.arm_action import PayloadCompensatedIKActionCfg
from policies.roboapi.gripper_action import CalibratedGripperActionCfg, observed_gripper_open
from robolab.constants import TASK_DIR
from robolab.core.environments.factory import auto_discover_and_create_cfgs
from robolab.core.observations.observation_utils import generate_image_obs_from_cameras, generate_obs_cfg
from robolab.robots.franka_high_pd import FrankaCfg as FrankaHighPDCfg
from robolab.variations.backgrounds import HomeOfficeBackgroundCfg
from robolab.variations.camera import EgocentricMirroredCameraCfg
from robolab.variations.lighting import SphereLightCfg

from policies.roboapi.config import ROBOT_USD

DEFAULT_ROBOT = str(ROBOT_USD)


def tool_pose(env, asset_cfg):
    robot = env.scene[asset_cfg.name]
    body = asset_cfg.body_ids[0]
    return subtract_frame_transforms(robot.data.root_pos_w, robot.data.root_quat_w,
        robot.data.body_link_pos_w[:, body], robot.data.body_link_quat_w[:, body])


def tool_pos(env, asset_cfg):
    return tool_pose(env, asset_cfg)[0]


def tool_quat(env, asset_cfg):
    return tool_pose(env, asset_cfg)[1]


@configclass
class ProprioObservationCfg(ObservationGroupCfg):
    tcp_pos = ObservationTermCfg(func=tool_pos,
        params={'asset_cfg': SceneEntityCfg('robot', body_names=['tool0'])})
    tcp_quat = ObservationTermCfg(func=tool_quat,
        params={'asset_cfg': SceneEntityCfg('robot', body_names=['tool0'])})
    gripper_open = ObservationTermCfg(func=observed_gripper_open)

    def __post_init__(self):
        self.enable_corruption = False
        self.concatenate_terms = False


@configclass
class RoboAPIActionsCfg:
    arm_action = PayloadCompensatedIKActionCfg(
        asset_name='robot', joint_names=['panda_joint[1-7]'], body_name='tool0',
        controller=DifferentialIKControllerCfg(command_type='pose', use_relative_mode=False, ik_method='dls'),
        scale=1., body_offset=None)
    gripper_action = CalibratedGripperActionCfg()


def register(tasks, calibration_path=None, robot_usd=None, record_viewport=False):
    # Resolve through the frozen catalog without importing unrelated scenes.
    # Factory class-name lookup otherwise imports every task even for one worker.
    task_sources = {item['task_name']: item['filename'] for item in load_task_catalog()}
    task_paths = [str(Path(TASK_DIR) / task_sources[name]) for name in tasks]
    wrist_cfg = make_franka_camera_cfg(**(
        {'calibration_path': calibration_path} if calibration_path else {}))
    robot_cfg = FrankaHighPDCfg().robot.copy()
    robot_cfg.spawn.usd_path = robot_usd or DEFAULT_ROBOT
    # Builder preserves assembly gripper physics and configures the arm separately.
    robot_cfg.spawn.rigid_props = None
    robot_cfg.spawn.articulation_props = None
    robot_cfg.init_state.joint_pos.pop('panda_finger_joint.*')
    robot_cfg.init_state.joint_pos.update({
        f'{side}_{part}_joint': 0. for side in ('positive', 'negative')
        for part in ('outer', 'inner', 'tip')})
    robot_cfg.actuators.pop('panda_hand')
    robot_cfg.actuators['robotiq'] = ImplicitActuatorCfg(
        joint_names_expr=['positive_outer_joint'], stiffness=None, damping=None)

    @configclass
    class FrankaRobotiqCfg:
        robot = robot_cfg
        wrist_camera = wrist_cfg
        frames = FrameTransformerCfg(prim_path='{ENV_REGEX_NS}/robot/panda_link0',
            target_frames=[
                FrameTransformerCfg.FrameCfg(prim_path='{ENV_REGEX_NS}/robot/tool0',
                    name='end_effector'),
            ])
        ee_frame = FrameTransformerCfg(prim_path='{ENV_REGEX_NS}/robot/panda_link0',
            target_frames=[FrameTransformerCfg.FrameCfg(
                prim_path='{ENV_REGEX_NS}/robot/tool0', name='end_effector')])

    FrankaRobotiqCfg.ee_recorder_bodies = {'ee_pose': 'tool0'}
    FrankaRobotiqCfg.table_fixture = FrankaHighPDCfg.table_fixture

    @configclass
    class WristImages:
        wrist_camera = wrist_cfg.copy()

    ImageObs = generate_image_obs_from_cameras([WristImages])
    groups = {'image_obs': ImageObs(), 'proprio_obs': ProprioObservationCfg()}
    if record_viewport:
        ViewObs = generate_image_obs_from_cameras([EgocentricMirroredCameraCfg])
        groups['viewport_cam'] = ViewObs()
    Obs = generate_obs_cfg(groups)
    return auto_discover_and_create_cfgs(
        task_dir=TASK_DIR, task_subdirs=None, tasks=task_paths,
        pattern='*.py', env_prefix='', env_postfix='',
        robot_cfg=FrankaRobotiqCfg, observations_cfg=Obs(), actions_cfg=RoboAPIActionsCfg(),
        camera_cfg=[EgocentricMirroredCameraCfg] if record_viewport else [],
        lighting_cfg=SphereLightCfg, background_cfg=HomeOfficeBackgroundCfg,
        contact_gripper={'gripper': '{ENV_REGEX_NS}/robot/RobotiqLinkage/positive_tip'},
        gripper_closure_cfg={'gripper': {'asset': 'robot',
            'joint': 'positive_outer_joint', 'open': 0., 'closed': -.79}},
        dt=1/240, decimation=8, render_interval=8, seed=0)
