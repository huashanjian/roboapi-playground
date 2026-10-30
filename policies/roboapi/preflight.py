"""Calibrated-physics readback and physical checks without policy calls.

Call runtime_setup_check only after the first environment reset, when PhysX and
the articulation tensors exist. All diagnostics are read-only.
"""
import json
from pathlib import Path
import numpy as np
import torch
from PIL import Image
from scipy.spatial.transform import Rotation

PAD_CENTERS = {
    'positive_tip': [-.021577547961249346, -1.257285475730896e-8, .030557617017547334],
    'negative_tip': [.0215775386480236, 4.656612873077393e-10, .030557606763635947],
}
from policies.roboapi.config import RIG_DIR

ASSET_MANIFEST = RIG_DIR / 'asset_manifest.json'
MIMIC_SIGNS = {'positive_outer_joint': 1., 'positive_inner_joint': 1.,
               'positive_tip_joint': -1., 'negative_outer_joint': -1.,
               'negative_inner_joint': -1., 'negative_tip_joint': 1.}


def _pose(position, quat_wxyz):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(np.asarray(quat_wxyz)[[1, 2, 3, 0]]).as_matrix()
    result[:3, 3] = position
    return result


def _linkage_tip_poses(master_angle, manifest):
    """Independent FK from the assembly's authored joint frames, in tool0."""
    poses = {'tool0': np.eye(4)}
    for side in ('positive', 'negative'):
        for part in ('outer', 'inner', 'tip'):
            body = side + '_' + part
            name = body + '_joint'
            attrs = manifest['gripper_joint_properties'][name]
            parent = side + '_outer' if part == 'tip' else 'tool0'
            local0 = _pose(attrs['physics:localPos0'], attrs['physics:localRot0'])
            local1 = _pose(attrs['physics:localPos1'], attrs['physics:localRot1'])
            turn = np.eye(4)
            turn[:3, :3] = Rotation.from_rotvec([0., master_angle * MIMIC_SIGNS[name], 0.]).as_matrix()
            poses[body] = poses[parent] @ local0 @ turn @ np.linalg.inv(local1)
    return {name: poses[name] for name in PAD_CENTERS}


def _pad_points(tip_poses):
    centers, faces = {}, {}
    for name, local in PAD_CENTERS.items():
        center = np.asarray(local, dtype=float)
        face = center + [-.004 if name == 'positive_tip' else .004, 0., 0.]
        centers[name] = (tip_poses[name] @ np.r_[center, 1.])[:3]
        faces[name] = (tip_poses[name] @ np.r_[face, 1.])[:3]
    return centers, faces


def _json_value(value):
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if hasattr(value, 'GetReal'):
        return [float(value.GetReal()), *map(float, value.GetImaginary())]
    try:
        return [_json_value(v) for v in value]
    except TypeError:
        return str(value)


def runtime_setup_check(env):
    """Validate effective assembly assembly after reset, without stepping.

    USD authored values and live PhysX drive tensors are recorded separately:
    PhysX converts angular gains to radian units when loading USD. Unspecified
    ImplicitActuator gains must retain those loaded defaults, not be reauthored
    using the USD numeric values as radians.
    """
    import omni.usd
    from pxr import Usd, UsdPhysics, UsdShade
    from policies.roboapi.camera import runtime_camera_check

    result = {'passed': False, 'failures': [], 'checks': {}}
    def check(name, passed, details):
        result['checks'][name] = {'passed': bool(passed), 'details': details}
        if not passed:
            result['failures'].append(name)
    def same(actual, expected):
        if isinstance(expected, (int, float, list)) and not isinstance(expected, bool):
            try:
                return bool(np.allclose(actual, expected, rtol=1e-6, atol=1e-8))
            except (ValueError, TypeError):
                return actual == expected
        return actual == expected

    robot = env.scene['robot']
    stage = omni.usd.get_context().get_stage()
    view = robot.root_physx_view
    body_paths = dict(zip(robot.body_names, view.link_paths[0]))
    root_path = body_paths['tool0'].rsplit('/', 1)[0]
    manifest = json.loads(ASSET_MANIFEST.read_text())
    expected_bodies = {Path(p).name for p in manifest['rigid_bodies']}
    expected_joints = set(MIMIC_SIGNS) | {f'panda_joint{i}' for i in range(1, 8)}
    check('topology', set(robot.body_names) == expected_bodies and
          set(robot.joint_names) == expected_joints and not any(
              'panda_hand' in str(p.GetPath()) or 'panda_finger' in p.GetName() or
              p.GetName() in ('panda_leftfinger', 'panda_rightfinger')
              for p in Usd.PrimRange(stage.GetPrimAtPath(root_path))),
          {'body_names': robot.body_names, 'joint_names': robot.joint_names,
           'expected_bodies': sorted(expected_bodies), 'root_path': root_path})

    joint_paths = dict(zip(robot.joint_names, view.dof_paths[0]))
    mimic_details = {}
    for name, sign in MIMIC_SIGNS.items():
        prim = stage.GetPrimAtPath(joint_paths[name])
        schemas = list(prim.GetMetadata('apiSchemas').GetAppliedItems())
        attr = lambda n: _json_value(prim.GetAttribute(n).Get())
        refs = [str(p) for p in prim.GetRelationship('physxMimicJoint:rotY:referenceJoint').GetTargets()]
        is_master = name == 'positive_outer_joint'
        valid = ('PhysicsDriveAPI:angular' in schemas if is_master else
                 'PhysxMimicJointAPI:rotY' in schemas and
                 'PhysicsDriveAPI:angular' not in schemas and
                 attr('physxMimicJoint:rotY:gearing') == -sign and
                 attr('physxMimicJoint:rotY:offset') == 0. and
                 attr('physxMimicJoint:rotY:referenceJointAxis') == 'rotY' and
                 refs == [joint_paths['positive_outer_joint']])
        mimic_details[name] = {'passed': bool(valid), 'schemas': schemas,
                              'reference_joint': refs, 'gearing': attr('physxMimicJoint:rotY:gearing')}
    check('master_and_five_mimics', all(v['passed'] for v in mimic_details.values()), mimic_details)

    gains = {'stiffness': view.get_dof_stiffnesses()[0].detach().cpu().numpy(),
             'damping': view.get_dof_dampings()[0].detach().cpu().numpy(),
             'effort_limit': view.get_dof_max_forces()[0].detach().cpu().numpy()}
    defaults = {'stiffness': robot.data.default_joint_stiffness[0].cpu().numpy(),
                'damping': robot.data.default_joint_damping[0].cpu().numpy()}
    actuator = robot.actuators['robotiq']
    master_index = robot.joint_names.index('positive_outer_joint')
    drive_details = {name: {key: float(values[robot.joint_names.index(name)])
                           for key, values in gains.items()} for name in MIMIC_SIGNS}
    drive_pass = actuator.is_implicit_model and list(actuator.joint_names) == ['positive_outer_joint']
    actuated_gripper_names = [joint for value in robot.actuators.values()
                             for joint in value.joint_names if joint in MIMIC_SIGNS]
    drive_pass &= actuated_gripper_names == ['positive_outer_joint']
    drive_pass &= actuator.cfg.stiffness is None and actuator.cfg.damping is None
    drive_pass &= actuator.cfg.effort_limit is None and actuator.cfg.effort_limit_sim is None
    drive_pass &= all(np.isclose(gains[key][master_index], defaults[key][master_index])
                      for key in defaults)
    drive_pass &= np.isclose(gains['effort_limit'][master_index], 1.2)
    drive_pass &= all(abs(drive_details[name][key]) < 1e-10
                      for name in MIMIC_SIGNS if name != 'positive_outer_joint'
                      for key in ('stiffness', 'damping'))
    check('live_gripper_drives', drive_pass,
          {'physx_radian_units': drive_details,
           'authored_master_joint': manifest['gripper_joint_properties']['positive_outer_joint'],
           'actuated_joint_names': {name: list(value.joint_names) for name, value in robot.actuators.items()},
           'master_preserves_loaded_defaults': {key: float(values[master_index]) for key, values in defaults.items()}})

    assembly = Usd.Stage.Open(manifest['assembly_asset'])
    if assembly is None:
        raise ValueError('Cannot open the configured tool assembly')
    assembly_root = manifest['assembly_root']
    mismatches, compared = [], 0
    physical_prefixes = ('physics:', 'physxRigidBody:', 'physxCollision:', 'physxMaterial:',
                         'physxMimicJoint:', 'drive:')
    # Compare assembly body, mass, collision and drive settings after all spawning
    # and actuator initialization. Initial velocities and drive targets vary.
    for subtree in ('tool0', 'RobotiqLinkage', 'poke_ee_link', 'poke_wrist_camera_link'):
        for original in Usd.PrimRange(assembly.GetPrimAtPath(assembly_root + '/' + subtree)):
            runtime_path = str(original.GetPath()).replace(assembly_root, root_path, 1)
            runtime = stage.GetPrimAtPath(runtime_path)
            for attribute in original.GetAttributes():
                name = attribute.GetName()
                if not attribute.HasAuthoredValueOpinion() or not name.startswith(physical_prefixes):
                    continue
                if name in ('physics:velocity', 'physics:angularVelocity') or name.endswith(('targetPosition', 'targetVelocity')):
                    continue
                expected = _json_value(attribute.Get())
                actual = _json_value(runtime.GetAttribute(name).Get()) if runtime else None
                compared += 1
                if not same(actual, expected):
                    mismatches.append({'prim': runtime_path, 'attribute': name,
                                       'assembly': expected, 'runtime': actual})
    check('body_collision_and_drive_attributes', not mismatches,
          {'compared_attribute_count': compared, 'mismatches': mismatches})

    gravity = {name: _json_value(stage.GetPrimAtPath(path).GetAttribute('physxRigidBody:disableGravity').Get())
               for name, path in body_paths.items()}
    check('arm_only_gravity_compensation', all(value is True if name.startswith('panda_link') else
          value in (None, False) for name, value in gravity.items()), gravity)
    pads = {}
    for side in ('positive', 'negative'):
        pad = stage.GetPrimAtPath(root_path + '/RobotiqLinkage/' + side + '_tip/PadCollider')
        material, relation = UsdShade.MaterialBindingAPI(pad).ComputeBoundMaterial('physics')
        material_prim = material.GetPrim()
        values = {name: _json_value(material_prim.GetAttribute('physics:' + name).Get())
                  for name in ('staticFriction', 'dynamicFriction', 'restitution')}
        values['material_path'] = str(material.GetPath())
        values['contact_offset_m'] = _json_value(pad.GetAttribute('physxCollision:contactOffset').Get())
        values['rest_offset_m'] = _json_value(pad.GetAttribute('physxCollision:restOffset').Get())
        pads[side] = values
    check('pad_material_and_contact_offsets', all(
        same(v['staticFriction'], 1.) and same(v['dynamicFriction'], .8) and
        same(v['restitution'], 0.) and same(v['contact_offset_m'], .0005) and
        same(v['rest_offset_m'], 0.) for v in pads.values()), pads)

    root = stage.GetPrimAtPath(root_path)
    solver = {name: _json_value(root.GetAttribute('physxArticulation:' + name).Get())
              for name in ('solverPositionIterationCount', 'solverVelocityIterationCount', 'enabledSelfCollisions')}
    check('articulation_solver', solver == {'solverPositionIterationCount': 32,
          'solverVelocityIterationCount': 1, 'enabledSelfCollisions': False}, solver)
    context = env.sim.get_physics_context()
    scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    if len(scenes) != 1:
        check('single_physics_scene', False, [str(p.GetPath()) for p in scenes])
    scene = scenes[0]
    scene_attrs = {a.GetName(): _json_value(a.Get()) for a in scene.GetAttributes()
                   if a.GetName().startswith('physxScene:') and
                   ('Iteration' in a.GetName() or a.GetName() in ('physxScene:enableCCD',
                    'physxScene:enableGPUDynamics', 'physxScene:broadphaseType', 'physxScene:solverType'))}
    clamps = {name: getattr(env.cfg.sim.physx, name) for name in (
        'min_position_iteration_count', 'max_position_iteration_count',
        'min_velocity_iteration_count', 'max_velocity_iteration_count')}
    scene_details = {'device': str(env.device), 'gpu_dynamics_enabled': context.is_gpu_dynamics_enabled(),
        'broadphase_type': context.get_broadphase_type(), 'solver_type': context.get_solver_type(),
        'scene_attributes': scene_attrs, 'configured_iteration_clamps': clamps}
    cpu_valid = not str(env.device).startswith('cpu') or (
        scene_details['gpu_dynamics_enabled'] is False and scene_details['broadphase_type'] == 'MBP')
    actual_clamps = {name: scene.GetAttribute('physxScene:' + usd_name).Get()
                    for name, usd_name in (
                        ('min_position_iteration_count', 'minPositionIterationCount'),
                        ('max_position_iteration_count', 'maxPositionIterationCount'),
                        ('min_velocity_iteration_count', 'minVelocityIterationCount'),
                        ('max_velocity_iteration_count', 'maxVelocityIterationCount'))}
    scene_details['actual_iteration_clamps'] = actual_clamps
    check('physics_scene', cpu_valid and scene.GetAttribute('physxScene:enableCCD').Get() is True and
          actual_clamps == clamps and
          clamps['min_position_iteration_count'] <= 32 <= clamps['max_position_iteration_count'] and
          clamps['min_velocity_iteration_count'] <= 1 and clamps['max_velocity_iteration_count'] >= 8,
          scene_details)
    timing = {'physics_dt_s': float(env.sim.get_physics_dt()), 'control_dt_s': float(env.step_dt),
              'decimation': int(env.cfg.decimation), 'render_interval': int(env.cfg.sim.render_interval)}
    check('timing', np.isclose(timing['physics_dt_s'], 1/240) and
          np.isclose(timing['control_dt_s'], 1/30) and timing['decimation'] == 8 and
          timing['render_interval'] == 8, timing)
    try:
        camera = runtime_camera_check(env)
        check('camera', camera['passed'], camera)
    except Exception as error:
        check('camera', False, {'exception': repr(error)})
    result['passed'] = not result['failures']
    return result


def gripper_checks(env, obs, hold, output):
    manifest = json.loads(ASSET_MANIFEST.read_text())
    robot = env.scene['robot']
    joint_ids, joint_names = robot.find_joints(['(positive|negative)_(outer|inner|tip)_joint'])
    bodies = {name: robot.find_bodies(name)[0][0] for name in PAD_CENTERS}
    tool_id = robot.find_bodies('tool0')[0][0]
    results = []
    for label, travel in [('open', 1.), ('half', .5), ('closed', 0.), ('reopen', 1.)]:
        action = hold.clone()
        action[:, -1] = travel
        for _ in range(60):
            obs, *_ = env.step(action)
        image = obs['image_obs']['wrist_camera'][0].cpu().numpy().astype(np.uint8)
        Image.fromarray(image).save(output/f'wrist_gripper_{label}.png')
        tool_q = robot.data.body_link_quat_w[0, tool_id].cpu().numpy()
        tool_p = robot.data.body_link_pos_w[0, tool_id].cpu().numpy()
        tool_inv = np.linalg.inv(_pose(tool_p, tool_q))
        tips = {}
        for name in PAD_CENTERS:
            idx = bodies[name]
            quat = robot.data.body_link_quat_w[0, idx].cpu().numpy()
            pos = robot.data.body_link_pos_w[0, idx].cpu().numpy()
            tips[name] = tool_inv @ _pose(pos, quat)
        centers, faces = _pad_points(tips)
        delta = centers['positive_tip'] - centers['negative_tip']
        joints = robot.data.joint_pos[0, joint_ids].cpu().numpy()
        joint_values = dict(zip(joint_names, joints.tolist()))
        master = joint_values['positive_outer_joint']
        mimic_errors = {name: joint_values[name] - master * sign
                        for name, sign in MIMIC_SIGNS.items() if name != 'positive_outer_joint'}
        expected_centers, expected_faces = _pad_points(_linkage_tip_poses(master, manifest))
        geometry_errors = {name: float(np.linalg.norm(faces[name] - expected_faces[name]))
                           for name in faces}
        gap = float(np.linalg.norm(faces['positive_tip'] - faces['negative_tip']))
        expected_gap = float(np.linalg.norm(expected_faces['positive_tip'] - expected_faces['negative_tip']))
        observation = float(obs['proprio_obs']['gripper_open'][0, 0])
        response = env.action_manager.get_term('gripper_action')._responses[0]
        results.append({'phase': label, 'requested_travel': travel,
            'measured_sampled_travel': observation,
            'joints_rad': joint_values,
            'mimic_residuals_rad': mimic_errors,
            'max_mimic_residual_rad': max(abs(v) for v in mimic_errors.values()),
            'pad_centers_tool0_m': {k: v.tolist() for k, v in centers.items()},
            'inner_face_centers_tool0_m': {k: v.tolist() for k, v in faces.items()},
            'expected_inner_face_centers_tool0_m': {k: v.tolist() for k, v in expected_faces.items()},
            'pad_geometry_error_m': geometry_errors,
            'max_pad_geometry_error_m': max(geometry_errors.values()),
            'inner_face_center_distance_m': gap,
            'expected_inner_face_center_distance_m': expected_gap,
            'pad_center_delta_in_tool_m': delta.tolist(),
            'pad_center_separation_m': float(np.linalg.norm(delta)),
            'actuator_raw': response.raw, 'actuator_raw_goal': response.raw_goal,
            'settle_sim_seconds': 2.})
    actual = [r['measured_sampled_travel'] for r in results]
    finite = all(np.isfinite(list(r['joints_rad'].values())).all() for r in results)
    encoder_pass = actual[0] > .97 and abs(actual[1]-.5) < .04 and actual[2] < .04 and actual[3] > .97
    mimic_pass = all(r['max_mimic_residual_rad'] < .01 for r in results)
    geometry_pass = all(r['max_pad_geometry_error_m'] < .002 for r in results)
    gaps = [r['inner_face_center_distance_m'] for r in results]
    stroke_pass = gaps[0] > gaps[1] > gaps[2] and abs(gaps[0] - gaps[3]) < .002
    passed = finite and encoder_pass and mimic_pass and geometry_pass and stroke_pass
    return obs, {'passed': bool(passed), 'checks': results,
                 'criteria': {'finite': bool(finite), 'encoder': bool(encoder_pass),
                              'mimic_residual_below_0_01_rad': bool(mimic_pass),
                              'geometry_error_below_2mm': bool(geometry_pass),
                              'monotonic_close_and_repeatable_open': bool(stroke_pass)},
                 'observation_check': 'Calibrated sampled/quantized joint travel after 2 seconds each phase',
                 'pad_gap_note': 'Distance between the two assembly pad inner-face centers, including original joint-frame calibration; independent assembly FK evaluated at measured master angle'}
