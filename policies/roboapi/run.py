"""Run RoboAPI evaluation with a calibrated Franka/Robotiq assembly.

Only the public image, TCP pose, gripper opening and instruction reach the API.
The official RoboLab episode loop and task success conditions are reused.
"""
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import subprocess
import signal
import time
import traceback

from policies.roboapi.config import BASE_URL, MODEL, ACTUATOR, TOKEN_FILE, ROBOT_USD, CALIBRATION

import cv2  # Must precede Isaac imports.
from isaaclab.app import AppLauncher
from robolab.eval.runner import add_common_eval_args, run_evaluation


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base-url', default=BASE_URL)
    p.add_argument('--model', default=MODEL)
    p.add_argument('--token-file', default=str(TOKEN_FILE))
    p.add_argument('--calibration-path')
    p.add_argument('--robot-usd', default=str(ROBOT_USD), help='Built Franka with assembly Robotiq and camera USD asset')
    p.add_argument('--max-chunks', type=int, default=90)
    p.add_argument('--episode-length-s', type=float, default=120.,
                   help='Per-rollout simulation horizon; independent of HTTP timeout.')
    p.add_argument('--timeout-s', type=float, default=120.)
    p.add_argument('--preflight-only', action='store_true',
                   help='Render, hold, and check TCP motion; no API request or benchmark score.')
    p.add_argument('--catalog-only', action='store_true',
                   help='Load and serialize official task configs; no environment rollout or API call.')
    add_common_eval_args(p)
    p.set_defaults(task=None,
                   num_runs=10, instruction_type=['default'], video_mode='sensor')
    AppLauncher.add_app_launcher_args(p)
    return p


def write_json(path, data):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2, default=str)+'\n')


def main():
    args = parser().parse_args()
    from policies.roboapi.horizon import load_task_catalog, validate_horizon, apply_horizon, verify_horizon
    catalog = load_task_catalog()
    if args.task is None:
        args.task = [item['task_name'] for item in catalog]
    if not set(args.task).issubset({item['task_name'] for item in catalog}):
        raise ValueError('Only the 120 official benchmark tasks are allowed')
    validate_horizon(args.episode_length_s, args.max_chunks)
    if args.num_envs != 1:
        raise ValueError('This session adapter requires --num-envs 1')
    if args.enable_gt_state:
        raise ValueError('Ground-truth observations are prohibited for this evaluation')
    if not (args.preflight_only or args.catalog_only) and not Path(args.token_file).is_file():
        raise FileNotFoundError('API credential file is missing')
    args.enable_cameras = True
    # USD schema definitions must be registered before SchemaRegistry is first
    # initialized. Enabling this only after AppLauncher starts can be too late.
    args.kit_args = (args.kit_args or '') + ' --enable omni.usd.schema.omni_lens_distortion'
    args.kit_args += ' --/app/settings/persistent=false --/persistent/rtx/modes/rt/enabled=true --/persistent/rtx/modes/rt2/enabled=true --/persistent/rtx/rendermode=RealTimePathTracing'
    args.kit_args += ' --/rtx/rendermode=RealTimePathTracing --/rtx-defaults/rendermode=RealTimePathTracing --/rtx/post/aa/op=3 --/rtx/post/dlss/execMode=2 --/rtx-transient/dlssg/enabled=false'
    launcher = AppLauncher(args)
    app = launcher.app
    # Kit installs native signal handlers; let Python finally blocks close the
    # API session and flush evidence on a requested interruption.
    def interrupted(signum, frame):
        raise KeyboardInterrupt(f'Signal {signum}')
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    try:
        import numpy as np
        import torch
        from PIL import Image
        import robolab.constants as constants
        from policies.roboapi.camera import (camera_manifest, capture_render_settings, configure_render_settings, RENDER_SETTINGS, runtime_camera_check)
        from policies.roboapi.client import RoboAPIInferenceClient, tool_to_policy_pose, policy_to_tool_pose
        from policies.roboapi.registration import register
        from robolab.core.environments.runtime import create_env
        from robolab.eval.episode import run_episode

        render_snapshot = capture_render_settings()
        original_create_env = create_env
        def calibrated_env(*args_, **kwargs_):
            result = original_create_env(*args_, **kwargs_)
            env, cfg = result
            horizon_evidence = verify_horizon(env, cfg, args.episode_length_s, args.max_chunks)
            write_json(Path(constants.get_output_dir())/'evaluation_runtime.json', horizon_evidence)
            effective_render = configure_render_settings(render_snapshot)
            write_json(Path(constants.get_output_dir())/'render_runtime.json',
                effective_render)
            return result
        create_env = calibrated_env
        # run_evaluation imports create_env inside its function body, so patch
        # the defining module (a runner-module attribute would be ignored).
        import robolab.core.environments.runtime as runtime_module
        runtime_module.create_env = calibrated_env
        original_apply_render = runtime_module._apply_render_settings
        def configure_simulation(env_cfg, renderer, rendering_mode):
            apply_horizon(env_cfg, args.episode_length_s, args.max_chunks)
            original_apply_render(env_cfg, renderer, rendering_mode)
            env_cfg.sim.render.carb_settings.update(RENDER_SETTINGS)
            # Enable scene CCD while preserving the supplied rigid-body flags.
            env_cfg.sim.physx.enable_ccd = True
            # Keep the native 255/255 maxima so the finger bodies can use
            # their authored velocity solver iteration count of 8.
            env_cfg.sim.physx.max_position_iteration_count = 255
            env_cfg.sim.physx.max_velocity_iteration_count = 255
        runtime_module._apply_render_settings = configure_simulation

        constants.ENABLE_SUBTASK_PROGRESS_CHECKING = args.enable_subtask
        constants.RECORD_IMAGE_DATA = False
        registered_configs = register(args.task, args.calibration_path, args.robot_usd,
            record_viewport=args.video_mode in ('all', 'viewport'))

        if not args.output_folder_name:
            args.output_folder_name = constants.get_timestamp() + '_roboapi'
        output = Path(constants.PACKAGE_DIR) / 'output' / args.output_folder_name
        output.mkdir(parents=True, exist_ok=True)
        manifest = {
            'robolab_commit': subprocess.check_output(['git','rev-parse','HEAD'],
                cwd=constants.PACKAGE_DIR, text=True).strip(),
            'packages': {name: importlib.metadata.version(name) for name in
                         ('isaacsim', 'isaaclab', 'torch', 'numpy', 'robolab')},
            'model': args.model, 'base_url': args.base_url, 'session_enabled': True,
            'tasks': args.task, 'seeds': list(range(args.seed + args.run_index_offset,
                args.seed + args.run_index_offset + args.num_runs)),
            'robot': 'Franka arm with calibrated Robotiq linkage and wrist camera',
            'robot_profile': {'arm_stiffness': 400., 'arm_damping': 80.,
                'arm_gravity_disabled': True, 'gripper_gravity': 'unchanged assembly USD',
                'tool_payload_compensation': 'tool0 Jacobian transpose of assembly payload gravity wrench; seven arm joints only',
                'arm_controller_source': 'robolab/robots/franka_high_pd.py'},
            'robot_usd_override': args.robot_usd,
            'state_and_action_frame': 'policy base; flange centre, zero tool translation',
            'simulator_action_frame': 'Franka base; assembly tool0 at flange with native mount Rz(+90deg)',
            'frame_mapping': {'policy_from_base': 'Rz(-90deg)',
                'flange_from_tool0': 'Rz(+90deg)' ,
                'formula': 'T_policy_tool=Rz(-90deg) @ T_base_tool0',
                'basis': 'Calibrated flange mounting; tool0, gripper and camera form one assembly'},
            'quaternion_order': {'internal': 'wxyz', 'API_wire': 'xyzw'},
            'gripper': {'observation': 'assembly measured master joint, 150ms sampled and raw3..229 quantized travel',
                'action': 'continuous API travel through unchanged assembly ActuatorResponse to positive_outer_joint'},
            'camera': camera_manifest(**({'calibration_path': args.calibration_path}
                if args.calibration_path else {})),
            'physics_dt_s': 1/240, 'action_dt_s': 1/30,
            'physics_scene_ccd': True,
            'physics_iteration_clamps': {'min_position': 1, 'max_position': 255,
                'min_velocity': 0, 'max_velocity': 255},
            'device': args.device,
            'renderer_cli_bootstrap': args.renderer,
            'isaaclab_bootstrap_preset': args.rendering_type or 'balanced',
            'rendering_policy': 'Restore native preset values and assembly explicit settings after every create_env',
            'auxiliary_viewport_camera': args.video_mode in ('all', 'viewport'),
            'max_chunks': args.max_chunks, 'official_task_timeouts': False,
            'episode_length_s': args.episode_length_s, 'api_timeout_s': args.timeout_s,
            'task_catalog_sha256': hashlib.sha256((Path(constants.TASK_DIR)/'_metadata/task_metadata.json').read_bytes()).hexdigest(),
            'official_task_code_sha256': {item['filename']: hashlib.sha256((Path(constants.TASK_DIR)/item['filename']).read_bytes()).hexdigest() for item in catalog},
            'official_task_timeouts_s': {item['task_name']: float(item['episode_s']) for item in catalog},
            'layout': 'official benchmark fixed layouts; seed does not randomize objects',
            'inference_retries': 0, 'ground_truth_to_policy': False,
            'preflight_only': args.preflight_only,
            'catalog_only': args.catalog_only,
            'calibrated_rig': True,
            'asset_manifest': json.loads((Path(args.robot_usd).parent/'asset_manifest.json').read_text()),
            'render_settings': configure_render_settings(render_snapshot),
            'gripper_actuator': json.loads(ACTUATOR.read_text()),
        }
        rig_data = [ACTUATOR, Path(args.calibration_path or CALIBRATION)]
        manifest['rig_data_sha256'] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                                        for path in rig_data}
        manifest['adapter_sha256'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in Path(__file__).parent.glob('*.py')}
        manifest_path = output/'manifest.json'
        if manifest_path.exists() and ((output/'episode_results.jsonl').exists()
                                       or (output/'run_metadata.json').exists()):
            previous = json.loads(manifest_path.read_text())
            protocol_keys = ('robolab_commit', 'packages', 'model',
                'base_url', 'camera', 'robot', 'robot_usd_override', 'max_chunks',
                'episode_length_s', 'api_timeout_s', 'task_catalog_sha256', 'official_task_code_sha256',
                'physics_dt_s', 'action_dt_s', 'physics_scene_ccd', 'physics_iteration_clamps', 'session_enabled', 'device',
                'frame_mapping', 'renderer_cli_bootstrap', 'isaaclab_bootstrap_preset', 'auxiliary_viewport_camera',
                'rig_data_sha256', 'asset_manifest', 'render_settings')
            changed = [key for key in protocol_keys if previous.get(key) != manifest[key]]
            changed += [name for name in manifest['adapter_sha256']
                if previous.get('adapter_sha256', {}).get(name) != manifest['adapter_sha256'][name]]
            if changed:
                raise ValueError(f'Existing output has a different protocol ({changed}); use a new output folder')
            # Preserve the provenance of completed episodes on resume.
        else:
            write_json(manifest_path, manifest)

        if args.catalog_only:
            configs = {cls._task_name: cls().to_dict() for cls in registered_configs.values()}
            if set(configs) != set(args.task):
                raise RuntimeError('Registered tasks differ from the requested catalog')
            write_json(output/'official_task_configs.json', configs)
            print(f'CATALOG VERIFIED: {len(configs)} task configs; zero policy calls', flush=True)
            return

        if args.preflight_only:
            task = args.task[0]
            constants.set_output_dir(str(output/task))
            env, cfg = create_env(task, device=args.device, seed=args.seed,
                num_envs=1, instruction_type=['default'], policy=args.model,
                renderer=args.renderer, rendering_mode=args.rendering_type)
            preflight_writers = []
            try:
                obs, _ = env.reset()
                obs, _ = env.reset()
                from policies.roboapi.preflight import runtime_setup_check
                setup_check = runtime_setup_check(env)
                write_json(output/'runtime_setup.json', setup_check)
                if not setup_check['passed']:
                    raise RuntimeError(f"Calibrated setup readback failed: {setup_check['failures']}")
                write_json(output/'runtime_camera.json', runtime_camera_check(env))
                from robolab.core.utils.video_utils import VideoWriter
                wrist_writer = VideoWriter(str(output/'gripper_camera_preflight.mp4'), 30)
                preflight_writers.append(wrist_writer)
                viewport_writer = None
                if 'viewport_cam' in obs:
                    viewport_writer = VideoWriter(str(output/'gripper_camera_preflight_viewport.mp4'), 30)
                    preflight_writers.append(viewport_writer)
                original_preflight_step = env.step
                def recorded_preflight_step(action):
                    result = original_preflight_step(action)
                    frame_obs = result[0]
                    wrist_writer.write(frame_obs['image_obs']['wrist_camera'][0].cpu().numpy().astype(np.uint8))
                    if viewport_writer is not None:
                        viewport_writer.write(frame_obs['viewport_cam']['egocentric_mirrored_camera'][0].cpu().numpy().astype(np.uint8))
                    return result
                env.step = recorded_preflight_step
                def flat(value):
                    return value[0].detach().cpu().numpy()
                Image.fromarray(flat(obs['image_obs']['wrist_camera']).astype(np.uint8)).save(
                    output/'wrist_initial.png')
                if 'viewport_cam' in obs:
                    Image.fromarray(flat(obs['viewport_cam']['egocentric_mirrored_camera']).astype(np.uint8)).save(
                        output/'viewport_initial.png')
                p0 = flat(obs['proprio_obs']['tcp_pos']).copy()
                q0 = flat(obs['proprio_obs']['tcp_quat']).copy()
                def sim_action(pos, quat):
                    hand = policy_to_tool_pose(tool_to_policy_pose(np.r_[pos, quat]))
                    return torch.tensor([[*hand, 1.]], device=env.device)
                hold = sim_action(p0, q0)
                hold_started = time.perf_counter()
                for _ in range(30):
                    obs, *_ = env.step(hold)
                hold_wall_s = time.perf_counter() - hold_started
                p1 = flat(obs['proprio_obs']['tcp_pos'])
                Image.fromarray(flat(obs['image_obs']['wrist_camera']).astype(np.uint8)).save(
                    output/'wrist_after_hold.png')
                checks = []
                from scipy.spatial.transform import Rotation
                q_rot = (Rotation.from_rotvec([0., 0., np.deg2rad(5.)]) *
                    Rotation.from_quat(q0[[1,2,3,0]])).as_quat()[[3,0,1,2]]
                targets = [('x_plus_1cm', p0 + [0.01, 0., 0.], q0),
                    ('y_minus_1cm', p0 + [0., -0.01, 0.], q0),
                    ('z_plus_1cm', p0 + [0., 0., 0.01], q0),
                    ('yaw_plus_5deg', p0, q_rot), ('restore', p0, q0)]
                for name, pos, quat in targets:
                    action = sim_action(pos, quat)
                    started = time.perf_counter()
                    for _ in range(60):
                        obs, *_ = env.step(action)
                    measured_pos = flat(obs['proprio_obs']['tcp_pos'])
                    measured_quat = flat(obs['proprio_obs']['tcp_quat'])
                    angle = (Rotation.from_quat(measured_quat[[1,2,3,0]]).inv() *
                        Rotation.from_quat(np.asarray(quat)[[1,2,3,0]])).magnitude()
                    checks.append({'name': name, 'target_tcp': pos.tolist(),
                        'measured_tcp': measured_pos.tolist(),
                        'position_error_m': float(np.linalg.norm(measured_pos-pos)),
                        'orientation_error_rad': float(angle),
                        'wall_60_steps_s': time.perf_counter()-started})
                    print('PREFLIGHT '+json.dumps(checks[-1]), flush=True)
                from policies.roboapi.preflight import gripper_checks
                obs, gripper_result = gripper_checks(env, obs, hold, output)
                passed = gripper_result['passed'] and all(c['position_error_m'] < .002 and
                    c['orientation_error_rad'] < np.deg2rad(1.) for c in checks)
                write_json(output/'preflight.json', {
                    'api_requests': 0, 'task': task, 'seed': args.seed,
                    'native_flange_start': p0.tolist(), 'native_flange_after_1s_hold': p1.tolist(),
                    'policy_flange_start': tool_to_policy_pose(np.r_[p0,q0]).tolist(),
                    'hold_position_error_m': float(np.linalg.norm(p1-p0)),
                    'hold_wall_30_steps_s': hold_wall_s,
                    'tcp_quat_wxyz': q0.tolist(),
                    'gripper_open': flat(obs['proprio_obs']['gripper_open']).tolist(),
                    'camera_rgb_shape': list(obs['image_obs']['wrist_camera'].shape),
                    'camera_rgb_std': float(obs['image_obs']['wrist_camera'].float().std()),
                    'dynamic_checks': checks, 'dynamic_checks_passed': passed,
                    'gripper_validation': gripper_result,
                    'benchmark_result': None,
                })
                if not passed:
                    raise RuntimeError('TCP tracking preflight failed; inspect preflight.json')
            finally:
                for writer in preflight_writers:
                    writer.release()
                env.close()
            return

        def make_client(cli):
            return RoboAPIInferenceClient(base_url=cli.base_url, token_file=cli.token_file,
                model=cli.model, timeout_s=cli.timeout_s,
                max_chunks=cli.max_chunks, action_dt=1/30)

        def seeded_episode(**kwargs):
            env, client, episode = kwargs['env'], kwargs['client'], kwargs['episode']
            seed = args.seed + episode
            env.seed(seed)
            original_step = env.step
            def counted_step(action):
                result = original_step(action)
                client.on_step_executed()
                return result
            env.step = counted_step
            started = time.perf_counter()
            try:
                result = run_episode(**kwargs)
                results, msgs, timing = result
                for r in results:
                    r['episode_metadata'] = {'seed': seed,
                        'layout_randomized': False, 'model': args.model,
                        'episode_length_limit_s': args.episode_length_s}
                write_json(Path(constants.get_output_dir())/f'run_{episode}_seed.json',
                    {'seed': seed, 'wall_episode_s': time.perf_counter()-started,
                     'results': results, 'timing': timing})
                return results, msgs, timing
            except BaseException as exc:
                write_json(Path(constants.get_output_dir())/f'run_{episode}_error.json',
                    {'seed': seed, 'error_type': type(exc).__name__, 'error': str(exc),
                     'wall_episode_s': time.perf_counter()-started,
                     'policy_success': None})
                raise
            finally:
                env.step = original_step

        run_evaluation(args, policy=args.model+'_franka_robotiq', client_factory=make_client,
                       episode_runner=seeded_episode)
    except BaseException:
        # Kit's shutdown can terminate the process before an exception reaches
        # Python's top-level handler; print it before closing the application.
        traceback.print_exc()
        raise
    finally:
        app.close()


if __name__ == '__main__':
    main()
