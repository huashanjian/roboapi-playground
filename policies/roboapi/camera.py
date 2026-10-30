"""Calibrated wrist fisheye camera on the Franka/Robotiq assembly.

The asset builder attaches the camera to tool0 using the calibrated optical
transform and ROS-to-USD axes. The sensor reads the existing camera prim without
respawning or rotating it. Authored fisheye attributes are retained, with a
0.01..5 metre clipping range. No image postprocessing is applied.

Load the lens schema and RT2 startup feature gates before AppLauncher initializes
USD. Renderer behavior can differ between Isaac Sim releases.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

from policies.roboapi.optics import (
    DEFAULT_CALIBRATION, LENS_SCHEMA, LENS_SCHEMA_EXTENSION, CALIBRATED_CAMERA_PRIM,
    _ensure_lens_schema, _calibrated_camera, _calibration_paths,
)

CAMERA_NAME = "wrist_camera"
CAMERA_PRIM = "{ENV_REGEX_NS}/robot/poke_wrist_camera_link/WristCamera"
WIDTH, HEIGHT = 640, 360
ROS_TO_USD = np.diag([1.0, -1.0, -1.0, 1.0])
CLIPPING_RANGE_M = (0.01, 5.0)
REQUIRED_RENDER_FEATURES = {
    "/persistent/rtx/modes/rt/enabled": True,
    "/persistent/rtx/modes/rt2/enabled": True,
}
RENDER_FEATURE_EVIDENCE = "https://forums.developer.nvidia.com/t/enable-new-rtx-realtime-2-0-in-simulationapp/342829/9"
RENDER_SETTINGS = {
    "/rtx/rendermode": "RealTimePathTracing",
    "/rtx-defaults/rendermode": "RealTimePathTracing",
    "/rtx/post/aa/op": 3,
    "/rtx-defaults/post/aa/op": 3,
    "/rtx/post/dlss/execMode": 2,
    "/rtx-defaults/post/dlss/execMode": 2,
    "/rtx-transient/dlssg/enabled": False,
}


def render_startup_args():
    """Isaac 5.1 RT2 feature flags; pass BEFORE constructing AppLauncher.

    Setting app.settings.persistent=false keeps these process-local and prevents
    writing the feature choices into the user's saved Kit configuration.
    NVIDIA's Isaac 5.x recipe requires both the RT and RT2 gates enabled.
    """
    return ["--/app/settings/persistent=false",
            "--/persistent/rtx/modes/rt/enabled=true",
            "--/persistent/rtx/modes/rt2/enabled=true",
            "--/persistent/rtx/rendermode=RealTimePathTracing",
            *[f"--{key}={str(value).lower() if isinstance(value, bool) else value}"
              for key, value in RENDER_SETTINGS.items()]]


def _render_preset_keys():
    """Read all installed IsaacLab preset keys so their overrides can be undone."""
    import isaaclab
    import tomllib

    # Wheels keep apps inside the import package; assembly/editable installs keep
    # them beside assembly/ at the repository root. Prefer the nearest location.
    package = Path(isaaclab.__file__).resolve().parent
    candidates = [base / "apps/rendering_modes"
                  for base in (package, *list(package.parents)[:4])]
    directory = next((path for path in candidates
                      if path.is_dir() and any(path.glob("*.kit"))), None)
    if directory is None:
        raise RuntimeError(f"Cannot find IsaacLab renderer presets; checked: {candidates}")
    def flatten(mapping, prefix=""):
        for key, value in mapping.items():
            path = prefix + "/" + key
            if isinstance(value, dict):
                yield from flatten(value, path)
            else:
                yield path
    return sorted({key for path in directory.glob("*.kit")
                   for key in flatten(tomllib.loads(path.read_text()))})


def capture_render_settings():
    """Call after AppLauncher, BEFORE the first create_env/SimulationContext.

    Preserve this installed Isaac version's native defaults for every key that
    an IsaacLab quality preset can overwrite. The calibrated render profile must remain independent of the chosen
    IsaacLab performance/balanced/quality preset.
    """
    import carb

    settings = carb.settings.get_settings()
    return {"native_defaults_before_isaaclab_preset": {
        key: settings.get(key) for key in _render_preset_keys()},
        "render_profile_settings": dict(RENDER_SETTINGS)}


def configure_render_settings(snapshot=None):
    """Call after EACH create_env, before reset/render captures policy images.

    With a startup snapshot, undo IsaacLab preset overrides, then apply the
    assembly's explicit renderer/DLSS settings. Without a snapshot this only sets
    explicit assembly settings and reports that preset restoration was not done.
    Returns the effective readback for recording in the run manifest.
    """
    import carb

    settings = carb.settings.get_settings()
    features = {key: settings.get(key) for key in REQUIRED_RENDER_FEATURES}
    if features != REQUIRED_RENDER_FEATURES:
        raise RuntimeError(
            "RTX Real-Time 2.0 feature gates were not enabled before app startup. "
            f"Pass render_startup_args() to AppLauncher; current flags: {features}")
    restored = {}
    if snapshot is not None:
        restored = snapshot["native_defaults_before_isaaclab_preset"]
        for key, value in restored.items():
            if value is None:
                settings.destroy_item(key)
            else:
                settings.set(key, value)
    for key, value in RENDER_SETTINGS.items():
        settings.set(key, value)
    readback = {key: settings.get(key) for key in RENDER_SETTINGS}
    if readback != RENDER_SETTINGS:
        raise RuntimeError(f"Calibrated renderer settings did not persist: {readback}")
    return {"render_profile_readback": readback,
            "renderer_feature_gates_readback": features,
            "isaaclab_preset_restored": snapshot is not None,
            "native_preset_values_restored": restored,
            "cross_version_pixel_equivalence_claimed": False}


def _calibration(path):
    raw = json.loads(Path(path).read_text())
    transform = np.asarray(raw["T_tcp_camera_optical"], dtype=float)
    intrinsic = np.asarray(raw["K_1280x720"], dtype=float).copy()
    distortion = np.asarray(raw["D_effective"], dtype=float)
    if transform.shape != (4, 4) or intrinsic.shape != (3, 3) or distortion.shape != (4,):
        raise ValueError("Unexpected assembly calibration dimensions")
    if not all(np.isfinite(v).all() for v in (transform, intrinsic, distortion)):
        raise ValueError("Calibrated camera calibration contains nonfinite values")
    if not np.allclose(transform[3], [0, 0, 0, 1]) or not np.allclose(
            transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-7):
        raise ValueError("Calibrated camera extrinsic is not a rigid transform")
    intrinsic[:2] *= 0.5
    return raw, intrinsic, distortion, transform


def make_franka_camera_cfg(calibration_path=DEFAULT_CALIBRATION, *, assembly_scene=None):
    """Use the calibrated camera already present in the calibrated robot asset."""
    from isaaclab.sensors import TiledCameraCfg

    _ensure_lens_schema()
    calibration, scene = _calibration_paths(calibration_path, assembly_scene)
    _calibration(calibration)
    assembly_stage, assembly = _calibrated_camera(scene)
    if LENS_SCHEMA not in assembly.GetAppliedSchemas():
        raise RuntimeError("Calibrated lens schema is not recognized; enable its extension at app startup")
    return TiledCameraCfg(prim_path=CAMERA_PRIM, height=HEIGHT, width=WIDTH,
                          data_types=["rgb"], update_period=0.0, spawn=None)


def camera_manifest(calibration_path=DEFAULT_CALIBRATION, *, assembly_scene=None):
    calibration, scene = _calibration_paths(calibration_path, assembly_scene)
    raw, intrinsic, distortion, transform = _calibration(calibration)
    stage, assembly = _calibrated_camera(scene)
    lens = {}
    for attribute in assembly.GetAttributes():
        key, value = attribute.GetName(), attribute.Get()
        if key.startswith("omni:lensdistortion:"):
            lens[key] = value if isinstance(value, (str, int, float, bool)) else list(value)
    return {
        "camera_name": CAMERA_NAME, "target_camera_prim": CAMERA_PRIM,
        "calibrated_camera_prim": CALIBRATED_CAMERA_PRIM,
        "calibration_path": str(calibration),
        "calibration_sha256": hashlib.sha256(calibration.read_bytes()).hexdigest(),
        "assembly_scene": str(scene),
        "assembly_scene_sha256": hashlib.sha256(scene.read_bytes()).hexdigest(),
        "width": WIDTH, "height": HEIGHT, "K_640x360": intrinsic.tolist(),
        "D_effective": distortion.tolist(), "T_tool0_camera_optical": transform.tolist(),
        "T_tool0_camera_usd": (transform @ ROS_TO_USD).tolist(),
        "camera_reference_frame": "assembly tool0 shared by the copied Robotiq mechanism",
        "control_point": "zero-offset tool0 at flange center",
        "camera_pose": "FK(assembly tool0) @ T_tcp_camera_optical @ diag(1,-1,-1,1)",
        "camera_alignment": "Use calibrated tool0 optical transform; no nominal TCP correction",
        "camera_pose_construction": "Builder uses the calibrated optical transform instead of a nominal USD camera transform",
        "mounting": "Complete assembly tool0/Robotiq/camera assembly mounted on true Franka flange by builder; no separate Panda finger-axis correction",
        "render_model": "assembly USD native OmniLensDistortionFthetaAPI",
        "render_lens_attributes": lens, "lens_schema_extension": LENS_SCHEMA_EXTENSION,
        "lens_schema_must_load_before_usd_registry": True,
        "render_clipping_range_m": list(CLIPPING_RANGE_M),
        "camera_exposure": float(assembly.GetAttribute("exposure").Get()),
        "f_stop": float(assembly.GetAttribute("fStop").Get()),
        "focus_distance": float(assembly.GetAttribute("focusDistance").Get()),
        "image_postprocessing": "none",
        "render_settings": dict(RENDER_SETTINGS),
        "isaac51_required_renderer_feature_gates": dict(REQUIRED_RENDER_FEATURES),
        "renderer_startup_args": render_startup_args(),
        "renderer_feature_enablement_reference": RENDER_FEATURE_EVIDENCE,
        "render_version_limitation": "Optical and renderer settings are explicit; pixel identity across Isaac Sim versions is not guaranteed",
        "calibration_status": raw.get("parameter_status"),
    }


def validate_asset_camera(asset_path, calibration_path=DEFAULT_CALIBRATION, *, assembly_scene=None):
    """CPU-only audit of the exported asset's camera against assembly runtime."""
    from pxr import Usd, UsdGeom

    manifest = camera_manifest(calibration_path, assembly_scene=assembly_scene)
    stage = Usd.Stage.Open(str(asset_path))
    if stage is None:
        raise ValueError(f"Cannot open calibrated asset: {asset_path}")
    root = stage.GetDefaultPrim().GetPath()
    tool = stage.GetPrimAtPath(root.AppendPath("tool0"))
    camera = stage.GetPrimAtPath(root.AppendPath("poke_wrist_camera_link/WristCamera"))
    if not tool or not camera or not camera.IsA(UsdGeom.Camera):
        raise ValueError("Calibrated asset must contain tool0 and poke_wrist_camera_link/WristCamera")
    op_order = [op.GetOpName() for op in UsdGeom.Xformable(camera).GetOrderedXformOps()]
    if op_order != ["xformOp:translate", "xformOp:orient", "xformOp:scale"]:
        raise ValueError(f"IsaacLab TiledCamera requires canonical camera TRS operations: {op_order}")
    cache = UsdGeom.XformCache()
    world_tool = np.array(cache.GetLocalToWorldTransform(tool)).T
    world_camera = np.array(cache.GetLocalToWorldTransform(camera)).T
    relative = np.linalg.inv(world_tool) @ world_camera
    expected = np.asarray(manifest["T_tool0_camera_usd"])
    difference = float(np.max(np.abs(relative - expected)))
    if difference > 2e-6:
        raise ValueError(f"Camera does not match assembly runtime tool0 extrinsic: max delta={difference}")
    for key, value in manifest["render_lens_attributes"].items():
        actual = camera.GetAttribute(key).Get()
        if not isinstance(actual, (str, int, float, bool)):
            actual = list(actual)
        if actual != value:
            raise ValueError(f"Copied lens parameter mismatch: {key}")
    if not np.allclose(camera.GetAttribute("clippingRange").Get(), CLIPPING_RANGE_M):
        raise ValueError("Camera clipping must match assembly runtime 0.01..5 m")
    schemas = camera.GetMetadata("apiSchemas")
    if schemas is None or LENS_SCHEMA not in schemas.GetAppliedItems():
        raise ValueError("Copied camera is missing its native lens schema")
    return {"asset_path": str(Path(asset_path).resolve()),
            "asset_sha256": hashlib.sha256(Path(asset_path).read_bytes()).hexdigest(),
            "camera_prim": str(camera.GetPath()), "tool0_prim": str(tool.GetPath()),
            "camera_xform_op_order": op_order,
            "maximum_transform_element_error": difference,
            "lens_attributes_match": True, "clipping_range_matches": True,
            "passed": True}


def runtime_camera_check(env, calibration_path=DEFAULT_CALIBRATION, *,
                         assembly_scene=None, position_tolerance_m=2e-4,
                         rotation_tolerance_rad=2e-3):
    """Read-only camera check after reset; no render or physics step is taken.

    Check the actual native lens schema/attributes and both USD-relative and
    measured articulation-body camera mounts. The latter detects a disconnected
    or incorrectly constrained camera even when its initial USD pose is correct.
    Small tolerances accommodate float32 PhysX fixed-joint constraint residuals.
    """
    import isaaclab.sim as sim_utils
    from pxr import UsdGeom
    from scipy.spatial.transform import Rotation

    manifest = camera_manifest(calibration_path, assembly_scene=assembly_scene)
    expected = np.asarray(manifest["T_tool0_camera_usd"])
    sensor = env.scene[CAMERA_NAME]
    if (sensor.cfg.width, sensor.cfg.height) != (WIDTH, HEIGHT):
        raise ValueError("Runtime camera resolution differs from assembly 640x360")
    cameras = sim_utils.find_matching_prims(sensor.cfg.prim_path)
    if len(cameras) != env.num_envs:
        raise ValueError(f"Expected {env.num_envs} wrist cameras, found {len(cameras)}")
    robot = env.scene["robot"]
    indices = {name: robot.body_names.index(name)
               for name in ("tool0", "poke_wrist_camera_link")}
    positions = robot.data.body_link_pos_w.detach().cpu().numpy()
    orientations = robot.data.body_link_quat_w.detach().cpu().numpy()
    def body_pose(row, name):
        pose = np.eye(4)
        pose[:3, 3] = positions[row, indices[name]]
        pose[:3, :3] = Rotation.from_quat(
            orientations[row, indices[name]], scalar_first=True).as_matrix()
        return pose
    def error(actual):
        delta = np.linalg.inv(expected) @ actual
        return (float(np.linalg.norm(actual[:3, 3] - expected[:3, 3])),
                float(Rotation.from_matrix(delta[:3, :3]).magnitude()))
    results = []
    for camera in cameras:
        path = camera.GetPath()
        robot_path = path.GetParentPath().GetParentPath()
        row = env.scene.env_prim_paths.index(str(robot_path.GetParentPath()))
        stage = camera.GetStage()
        tool = stage.GetPrimAtPath(robot_path.AppendPath("tool0"))
        if not camera.IsA(UsdGeom.Camera) or LENS_SCHEMA not in camera.GetAppliedSchemas():
            raise ValueError(f"Runtime camera has no recognized native Ftheta schema: {path}")
        for key, value in manifest["render_lens_attributes"].items():
            actual = camera.GetAttribute(key).Get()
            if not isinstance(actual, (str, int, float, bool)):
                actual = list(actual)
            if actual != value:
                raise ValueError(f"Runtime assembly lens attribute changed: {path} {key}")
        if not np.allclose(camera.GetAttribute("clippingRange").Get(), CLIPPING_RANGE_M):
            raise ValueError(f"Runtime camera clipping differs from assembly: {path}")
        for key, value in (("exposure", manifest["camera_exposure"]),
                           ("fStop", manifest["f_stop"]),
                           ("focusDistance", manifest["focus_distance"])):
            if not np.isclose(camera.GetAttribute(key).Get(), value):
                raise ValueError(f"Runtime camera optical setting changed: {path} {key}")
        xform = UsdGeom.Xformable(camera)
        if xform.GetResetXformStack():
            raise ValueError(f"Camera unexpectedly resets its parent transform: {path}")
        local = np.array(xform.GetLocalTransformation()).T
        cache = UsdGeom.XformCache()
        usd_relative = (np.linalg.inv(np.array(cache.GetLocalToWorldTransform(tool)).T)
                        @ np.array(cache.GetLocalToWorldTransform(camera)).T)
        measured_relative = (np.linalg.inv(body_pose(row, "tool0"))
                             @ body_pose(row, "poke_wrist_camera_link") @ local)
        usd_position, usd_rotation = error(usd_relative)
        measured_position, measured_rotation = error(measured_relative)
        if (max(usd_position, measured_position) > position_tolerance_m or
                max(usd_rotation, measured_rotation) > rotation_tolerance_rad):
            raise ValueError(
                f"Runtime camera mounting mismatch at {path}: "
                f"USD=({usd_position}m,{usd_rotation}rad), "
                f"PhysX=({measured_position}m,{measured_rotation}rad)")
        results.append({"prim_path": str(path), "environment_index": row,
                        "applied_schemas": camera.GetAppliedSchemas(),
                        "xform_op_order": [op.GetOpName() for op in xform.GetOrderedXformOps()],
                        "usd_mount_position_error_m": usd_position,
                        "usd_mount_rotation_error_rad": usd_rotation,
                        "physx_mount_position_error_m": measured_position,
                        "physx_mount_rotation_error_rad": measured_rotation,
                        "lens_attributes_match": True, "optical_settings_match": True})
    return {"passed": True, "width": WIDTH, "height": HEIGHT,
            "position_tolerance_m": position_tolerance_m,
            "rotation_tolerance_rad": rotation_tolerance_rad, "cameras": results}


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--validate-asset", type=Path)
    args = parser.parse_args()
    result = validate_asset_camera(args.validate_asset) if args.validate_asset else camera_manifest()
    print(json.dumps(result, indent=2))
