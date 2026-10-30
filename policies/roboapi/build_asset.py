"""Build a standalone Franka/Robotiq robot from calibrated rig assets.

Run with Python providing USD and NumPy:
    python -m policies.roboapi.build_asset

Replace the Panda hand with the complete tool assembly at link7 +107 mm,
rotated +90 degrees around Z. Preserve its geometry, mass, inertia, collision,
drive and mimic properties. Position the wrist camera using its calibrated
optical transform with ROS-to-USD axes and a 0.01..5 metre clipping range.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


from policies.roboapi.config import (
    RIG_DIR as WORK, PANDA_USD as PANDA, ASSEMBLY_USD as ASSEMBLY,
    CALIBRATION, ACTUATOR, ASSEMBLY_ROOT, ROBOT_USD,
)
TARGET_ROOT = '/Franka'
GRIPPER_JOINTS = ['positive_outer_joint', 'positive_inner_joint', 'positive_tip_joint',
                  'negative_outer_joint', 'negative_inner_joint', 'negative_tip_joint']
GRIPPER_BODIES = ['positive_outer', 'positive_inner', 'positive_tip',
                  'negative_outer', 'negative_inner', 'negative_tip']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def value_json(value):
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    if isinstance(value, Sdf.AssetPath):
        return {'path': value.path, 'resolved_path': value.resolvedPath}
    if isinstance(value, Sdf.Path):
        return str(value)
    if isinstance(value, (Gf.Quatf, Gf.Quatd, Gf.Quath)):
        return [float(value.GetReal()), *map(float, value.GetImaginary())]
    try:
        return [value_json(item) for item in value]
    except TypeError:
        return str(value)


def matrix(prim):
    """Conventional column-vector homogeneous matrix."""
    return np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T


def rigid(matrix_value):
    result = np.array(matrix_value, dtype=float, copy=True)
    u, _, vt = np.linalg.svd(result[:3, :3])
    result[:3, :3] = u @ vt
    return result


def frame(position, rotation):
    q = Gf.Quatd(float(rotation.GetReal()), Gf.Vec3d(*map(float, rotation.GetImaginary())))
    q.Normalize()
    result = np.asarray(Gf.Matrix4d(1).SetRotate(q)).T.copy()
    result[:3, 3] = position
    return result


def joint_frame(prim, side):
    return frame(prim.GetAttribute(f'physics:localPos{side}').Get() or (0, 0, 0),
                 prim.GetAttribute(f'physics:localRot{side}').Get() or Gf.Quatf(1))


def set_transform(prim, transform):
    for attribute in list(prim.GetAttributes()):
        if attribute.GetName().startswith('xformOp:') or attribute.GetName() == 'xformOpOrder':
            prim.RemoveProperty(attribute.GetName())
    xf = UsdGeom.Xformable(prim)
    xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Matrix4d(transform.T.tolist()))
    xf.SetResetXformStack(False)


def add_api_token(prim, token):
    tokens = list(prim.GetMetadata('apiSchemas').GetAppliedItems()) if prim.HasMetadata('apiSchemas') else []
    if token not in tokens:
        tokens.append(token)
    prim.SetMetadata('apiSchemas', Sdf.TokenListOp.CreateExplicit(tokens))


def flatten_without_instances(stage):
    stage.SetEditTarget(stage.GetSessionLayer())
    for _ in range(8):
        instances = [prim for prim in stage.Traverse() if prim.IsInstanceable()]
        if not instances:
            return stage.Flatten()
        for prim in instances:
            prim.SetInstanceable(False)
    raise RuntimeError('nested USD instances did not expand')


def remap_path(path, mapping):
    for old, new in sorted(mapping.items(), key=lambda pair: len(pair[0]), reverse=True):
        if path.HasPrefix(Sdf.Path(old)):
            return path.ReplacePrefix(Sdf.Path(old), Sdf.Path(new))
    return path


def remap_targets(stage, mapping, removed=()):
    for prim in stage.Traverse():
        for relation in prim.GetRelationships():
            targets = relation.GetTargets()
            if targets:
                converted = [remap_path(target, mapping) for target in targets]
                converted = [target for target in converted if not any(target.HasPrefix(Sdf.Path(p)) for p in removed)]
                relation.SetTargets(converted)
        for attribute in prim.GetAttributes():
            connections = attribute.GetConnections()
            if connections:
                attribute.SetConnections([remap_path(target, mapping) for target in connections])


def authored(prim, *, exclude=()):
    return {attribute.GetName(): value_json(attribute.Get()) for attribute in prim.GetAttributes()
            if attribute.HasAuthoredValueOpinion() and not any(attribute.GetName().startswith(p) for p in exclude)}


def gripper_poses(assembly, angle):
    """Exact assembly local joint frames, with the assembly mimic signs."""
    q = {'positive_outer_joint': -angle, 'positive_inner_joint': -angle,
         'positive_tip_joint': angle, 'negative_outer_joint': angle,
         'negative_inner_joint': angle, 'negative_tip_joint': -angle}
    poses = {ASSEMBLY_ROOT + '/tool0': np.eye(4)}
    for name in GRIPPER_JOINTS:
        joint = assembly.GetPrimAtPath(ASSEMBLY_ROOT + '/RobotiqLinkage/' + name)
        parent = str(joint.GetRelationship('physics:body0').GetTargets()[0])
        child = str(joint.GetRelationship('physics:body1').GetTargets()[0])
        turn = np.eye(4)
        c, s = math.cos(q[name]), math.sin(q[name])
        turn[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]
        poses[child] = poses[parent] @ joint_frame(joint, 0) @ turn @ np.linalg.inv(joint_frame(joint, 1))
    return poses


def pad_measurements(assembly, angle):
    poses = gripper_poses(assembly, angle)
    centers, inner_faces, local_centers = {}, {}, {}
    for side in ('positive', 'negative'):
        body_path = ASSEMBLY_ROOT + '/RobotiqLinkage/' + side + '_tip'
        pad = assembly.GetPrimAtPath(body_path + '/PadCollider')
        local = np.asarray(UsdGeom.Xformable(pad).GetLocalTransformation()).T
        local_centers[side] = local[:3, 3].tolist()
        world = poses[body_path] @ local
        centers[side] = world[:3, 3].tolist()
        # Unit Cube, scale contains the complete pad dimensions.
        inner_faces[side] = (world @ np.array([-.5 if side == 'positive' else .5, 0, 0, 1]))[:3].tolist()
    return {'closing_angle_rad': angle, 'normalized_open': 1 - angle / .79,
            'pad_centers_tool0_m': centers, 'pad_centers_tip_local_m': local_centers,
            'pad_midpoint_tool0_m': ((np.array(centers['positive']) + centers['negative']) / 2).tolist(),
            'inner_face_centers_tool0_m': inner_faces,
            'inner_face_center_distance_m': float(np.linalg.norm(np.array(inner_faces['positive']) - inner_faces['negative']))}


def build(args):
    args.output.parent.mkdir(parents=True, exist_ok=True)
    panda = Usd.Stage.Open(str(args.panda))
    assembly = Usd.Stage.Open(str(args.assembly))
    assembly_layer = assembly.Flatten()
    panda_layer = flatten_without_instances(panda)
    out_layer = Sdf.Layer.CreateAnonymous('franka_robotiq.usda')
    assert Sdf.CopySpec(panda_layer, panda.GetDefaultPrim().GetPath(), out_layer, Sdf.Path(TARGET_ROOT))
    stage = Usd.Stage.Open(out_layer)
    stage.SetDefaultPrim(stage.GetPrimAtPath(TARGET_ROOT))
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    removed = [TARGET_ROOT + '/panda_hand', TARGET_ROOT + '/panda_leftfinger',
               TARGET_ROOT + '/panda_rightfinger', TARGET_ROOT + '/panda_link7/panda_hand_joint']
    for path in removed:
        if not stage.RemovePrim(path):
            raise RuntimeError('expected Panda hand prim absent: ' + path)
    mapping = {'/panda': TARGET_ROOT, ASSEMBLY_ROOT: TARGET_ROOT}
    for name in ('tool0', 'poke_ee_link', 'poke_wrist_camera_link', 'RobotiqLinkage'):
        old, new = ASSEMBLY_ROOT + '/' + name, TARGET_ROOT + '/' + name
        assert Sdf.CopySpec(assembly_layer, Sdf.Path(old), out_layer, Sdf.Path(new))
        mapping[old] = new
    UsdGeom.Scope.Define(stage, TARGET_ROOT + '/ToolMaterials')
    for name in ('Materials', 'Appearance', 'LinkageContact', 'PolicyContact'):
        old, new = '/World/' + name, TARGET_ROOT + '/ToolMaterials/' + name
        if assembly.GetPrimAtPath(old):
            assert Sdf.CopySpec(assembly_layer, Sdf.Path(old), out_layer, Sdf.Path(new))
            mapping[old] = new
    UsdGeom.Scope.Define(stage, TARGET_ROOT + '/rig_joints')
    for name in ('tool0_poke_ee_link', 'poke_ee_link_poke_wrist_camera_link'):
        old, new = ASSEMBLY_ROOT + '/joints/' + name, TARGET_ROOT + '/rig_joints/' + name
        assert Sdf.CopySpec(assembly_layer, Sdf.Path(old), out_layer, Sdf.Path(new))
        mapping[old] = new
    remap_targets(stage, mapping, removed)

    mount = np.eye(4)
    mount[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    mount[:3, 3] = [0, 0, .107]
    link7_pose = rigid(matrix(stage.GetPrimAtPath(TARGET_ROOT + '/panda_link7')))
    tool_pose = link7_pose @ mount
    set_transform(stage.GetPrimAtPath(TARGET_ROOT + '/tool0'), tool_pose)
    poses = gripper_poses(assembly, 0.)
    for name in GRIPPER_BODIES:
        set_transform(stage.GetPrimAtPath(TARGET_ROOT + '/RobotiqLinkage/' + name),
                      tool_pose @ poses[ASSEMBLY_ROOT + '/RobotiqLinkage/' + name])
    # Keep camera-link mass and geometry; define its attachment using the
    # calibrated optical transform.
    calibration = json.loads(args.calibration.read_text())
    camera_tf = np.asarray(calibration['T_tcp_camera_optical'], dtype=float)
    set_transform(stage.GetPrimAtPath(TARGET_ROOT + '/poke_ee_link'), tool_pose)
    set_transform(stage.GetPrimAtPath(TARGET_ROOT + '/poke_wrist_camera_link'), tool_pose @ camera_tf)
    camera_joint = stage.GetPrimAtPath(TARGET_ROOT + '/rig_joints/poke_ee_link_poke_wrist_camera_link')
    camera_joint.GetAttribute('physics:localPos0').Set(Gf.Vec3f(*camera_tf[:3, 3]))
    camera_rotation = Gf.Matrix4d(camera_tf.T.tolist()).ExtractRotationQuat()
    camera_joint.GetAttribute('physics:localRot0').Set(Gf.Quatf(camera_rotation))
    camera = UsdGeom.Camera(stage.GetPrimAtPath(TARGET_ROOT + '/poke_wrist_camera_link/WristCamera'))
    # Isaac Sim Camera initializes an XformPrim view with reset_xform_properties
    # disabled. It requires canonical TRS ops; the equivalent single matrix op
    # is rejected even though USD itself accepts it.
    camera_xform = UsdGeom.Xformable(camera.GetPrim())
    for attribute in list(camera.GetPrim().GetAttributes()):
        if attribute.GetName().startswith('xformOp:'):
            camera.GetPrim().RemoveProperty(attribute.GetName())
    camera_xform.ClearXformOpOrder()
    camera_xform.AddTranslateOp().Set(Gf.Vec3d(0))
    camera_xform.AddOrientOp().Set(Gf.Quatf(0., Gf.Vec3f(1., 0., 0.)))
    camera_xform.AddScaleOp().Set(Gf.Vec3f(1.))
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(.01, 5.))
    fixed = UsdPhysics.FixedJoint.Define(stage, TARGET_ROOT + '/panda_link7/tool0_fixed_mount')
    fixed.CreateBody0Rel().SetTargets([Sdf.Path(TARGET_ROOT + '/panda_link7')])
    fixed.CreateBody1Rel().SetTargets([Sdf.Path(TARGET_ROOT + '/tool0')])
    fixed.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, .107))
    fixed.CreateLocalPos1Attr().Set(Gf.Vec3f(0))
    fixed.CreateLocalRot0Attr().Set(Gf.Quatf(math.sqrt(.5), Gf.Vec3f(0, 0, math.sqrt(.5))))
    fixed.CreateLocalRot1Attr().Set(Gf.Quatf(1))
    fixed.CreateCollisionEnabledAttr().Set(False)
    flange = UsdGeom.Xform.Define(stage, TARGET_ROOT + '/panda_link7/panda_link8')
    flange_tf = np.eye(4)
    flange_tf[2, 3] = .107
    set_transform(flange.GetPrim(), flange_tf)

    root = stage.GetDefaultPrim()
    add_api_token(root, 'PhysxArticulationAPI')
    for name, value, value_type in [('enabledSelfCollisions', False, Sdf.ValueTypeNames.Bool),
                                  ('solverPositionIterationCount', 32, Sdf.ValueTypeNames.Int),
                                  ('solverVelocityIterationCount', 1, Sdf.ValueTypeNames.Int)]:
        root.CreateAttribute('physxArticulation:' + name, value_type).Set(value)
    for index in range(8):
        link = stage.GetPrimAtPath(TARGET_ROOT + f'/panda_link{index}')
        add_api_token(link, 'PhysxRigidBodyAPI')
        link.CreateAttribute('physxRigidBody:disableGravity', Sdf.ValueTypeNames.Bool).Set(True)
        link.CreateAttribute('physxRigidBody:maxDepenetrationVelocity', Sdf.ValueTypeNames.Float).Set(5.)
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            for attribute_name in ('physics:velocity', 'physics:angularVelocity'):
                attribute = prim.GetAttribute(attribute_name)
                if attribute and attribute.HasAuthoredValueOpinion():
                    attribute.Set(Gf.Vec3f(0))
        if prim.GetName() in GRIPPER_JOINTS:
            for attribute_name in ('state:angular:physics:position', 'state:angular:physics:velocity'):
                attribute = prim.GetAttribute(attribute_name)
                if attribute:
                    attribute.Set(0.)

    # Validate exact copy of geometry and physical settings, allowing only the
    # declared initial-state and runtime-camera transformations above.
    equality = []
    excluded = ('xformOp', 'physics:velocity', 'physics:angularVelocity', 'state:angular:physics:')
    for name in ('tool0', 'poke_ee_link', 'poke_wrist_camera_link', 'RobotiqLinkage'):
        for original in Usd.PrimRange(assembly.GetPrimAtPath(ASSEMBLY_ROOT + '/' + name)):
            target_path = remap_path(original.GetPath(), mapping)
            target = stage.GetPrimAtPath(target_path)
            assert target, str(target_path)
            if original.GetTypeName() == 'Camera':
                skip = (*excluded, 'clippingRange')
            elif original.HasAPI(UsdPhysics.RigidBodyAPI):
                skip = excluded
            elif original.IsA(UsdPhysics.Joint):
                skip = ('state:angular:physics:',)
            else:
                skip = ()
            if authored(original, exclude=skip) != authored(target, exclude=skip):
                raise RuntimeError('assembly attributes changed unexpectedly: ' + str(target_path))
            original_api = original.GetMetadata('apiSchemas')
            if original_api != target.GetMetadata('apiSchemas'):
                raise RuntimeError('assembly API schema tokens changed: ' + str(target_path))
            equality.append(str(target_path))
    unresolved = []
    for prim in stage.Traverse():
        for relation in prim.GetRelationships():
            for target in relation.GetTargets():
                if not stage.GetPrimAtPath(target.GetPrimPath()):
                    unresolved.append({'prim': str(prim.GetPath()), 'relationship': relation.GetName(), 'target': str(target)})
        for attribute in prim.GetAttributes():
            for target in attribute.GetConnections():
                if not stage.GetPropertyAtPath(target):
                    unresolved.append({'prim': str(prim.GetPath()), 'connection': str(target)})
    if unresolved:
        raise RuntimeError('dangling relationships/connections: ' + json.dumps(unresolved))
    bodies = [str(prim.GetPath()) for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.RigidBodyAPI)]
    joints = [str(prim.GetPath()) for prim in stage.Traverse() if prim.IsA(UsdPhysics.RevoluteJoint)]
    roots = [str(prim.GetPath()) for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
    assert len(bodies) == 17, bodies
    assert len(joints) == 13, joints
    assert roots == [TARGET_ROOT], roots
    actual_camera = np.linalg.inv(matrix(stage.GetPrimAtPath(TARGET_ROOT + '/tool0'))) @ matrix(camera.GetPrim())
    camera_error = float(np.max(np.abs(actual_camera - camera_tf @ np.diag([1, -1, -1, 1]))))
    assert camera_error < 1e-10, camera_error
    stage.GetRootLayer().Export(str(args.output))
    reopened = Usd.Stage.Open(str(args.output))
    assert reopened and len(list(reopened.GetUsedLayers())) == 2  # file + empty session, no asset layers
    manifest = {
        'schema_version': 2, 'builder': str(Path(__file__).resolve()), 'builder_sha256': sha(__file__),
        'output_asset': str(args.output), 'output_sha256': sha(args.output),
        'input_assets': {str(path.resolve()): sha(path) for path in (args.panda, args.assembly, args.calibration, args.actuator)},
        'assembly_asset': str(args.assembly.resolve()), 'assembly_root': ASSEMBLY_ROOT,
        'default_prim': TARGET_ROOT, 'standalone_geometry': True,
        'articulation_roots': roots, 'rigid_bodies': bodies, 'revolute_joints': joints,
        'verified_prim_count': len(equality),
        'unresolved_relationships_or_shader_connections': unresolved,
        'control_body': 'tool0', 'control_body_path': TARGET_ROOT + '/tool0',
        'camera_path': str(camera.GetPath()),
        'contact_sensor_body': TARGET_ROOT + '/RobotiqLinkage/positive_tip',
        'gripper_main_joint': 'positive_outer_joint', 'gripper_joint_names': GRIPPER_JOINTS,
        'gripper_initial_joint_positions_rad': {name: 0. for name in GRIPPER_JOINTS},
        'gripper_closed_joint_positions_rad': {name: sign * .79 for name, sign in zip(GRIPPER_JOINTS, [-1, -1, 1, 1, 1, -1])},
        'gripper_main_joint_open_closed_rad': [0., -.79],
        'mimic_equation': 'q_mimic + gearing*q_reference + offset = 0; all five reference positive_outer_joint/rotY',
        'gripper_joint_properties': {name: authored(assembly.GetPrimAtPath(ASSEMBLY_ROOT + '/RobotiqLinkage/' + name), exclude=('state:',)) for name in GRIPPER_JOINTS},
        'mount': {'parent_body': 'panda_link7', 'flange_translation_m': [0, 0, .107],
                  'flange_from_tool0_rotation': 'Rz(+90deg)', 'T_link7_tool0': mount.tolist(),
                  'panda_hand_minus45_rotation_retained': False,
                  'basis': 'Flange-centred tool mount: link7 +107 mm, Rz(+90deg)'},
        'arm_gravity_compensation': {'bodies': [f'panda_link{i}' for i in range(8)], 'disable_gravity': True,
                                    'tool_gravity_properties_preserved': True},
        'articulation_settings': {'enabled_self_collisions': False, 'solver_position_iteration_count': 32,
                                 'solver_velocity_iteration_count': 1, 'basis': 'Calibrated assembly root articulation settings'},
        'finger_body_solver_iterations': {'position': 32, 'velocity': 8},
        'camera_runtime_corrections': {'T_tool0_camera_optical': camera_tf.tolist(),
            'optical_to_USD_camera': np.diag([1., -1., -1., 1.]).tolist(), 'clipping_range_m': [.01, 5.],
            'reason': 'Use FK(tool0) @ calibrated optical transform and a 5 metre far clip.',
            'max_matrix_error': camera_error, 'lens_attributes_copied_exactly': True},
        'initial_state': 'Open q=0 rest reconstructed from exact assembly local joint frames; saved simulation snapshot velocities cleared.',
        'pad_geometry': [pad_measurements(assembly, a) for a in (0., .395, .79)],
        'gripper_response_parameters': json.loads(args.actuator.read_text())['gripper'],
        'integration_requirements': [
            'Use only positive_outer_joint as an actuated gripper DOF; preserve five PhysxMimicJointAPI:rotY instances without extra drives.',
            'Disable spawn-wide rigid/articulation overrides; per-arm gravity compensation and assembly gripper settings are authored here.',
            'Main joint target is continuous -(1-g)*0.79 after assembly ActuatorResponse; preserve assembly delayed/quantized gripper sensing.',
            'All policy poses use base-frame left multiplication Rz(-90deg) applied to tool0 pose; tool0 already includes the physical flange +90deg mounting.',
            'Enable omni.usd.schema.omni_lens_distortion before composing the asset; spawn=None camera uses the authored optics.',
            'Physics 240Hz and control30Hz match assembly setup; offline asset checks do not replace a PhysX articulation/closure/collision preflight.',
        ],
    }
    args.manifest.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--panda', type=Path, default=PANDA)
    parser.add_argument('--assembly', type=Path, default=ASSEMBLY)
    parser.add_argument('--calibration', type=Path, default=CALIBRATION)
    parser.add_argument('--actuator', type=Path, default=ACTUATOR)
    parser.add_argument('--output', type=Path, default=ROBOT_USD)
    parser.add_argument('--manifest', type=Path, default=WORK / 'asset_manifest.json')
    args = parser.parse_args()
    manifest = build(args)
    print(json.dumps({key: manifest[key] for key in ('output_asset', 'output_sha256', 'default_prim',
        'control_body', 'camera_path', 'gripper_main_joint', 'verified_prim_count')}, indent=2))


if __name__ == '__main__':
    main()
