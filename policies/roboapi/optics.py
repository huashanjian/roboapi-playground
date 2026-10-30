"""Validate calibrated assembly optics and load the native fisheye schema."""
from pathlib import Path
from policies.roboapi.config import (
    CALIBRATION as DEFAULT_CALIBRATION, ASSEMBLY_USD, CALIBRATED_CAMERA_PRIM,
)
WIDTH, HEIGHT = 640, 360
LENS_SCHEMA = 'OmniLensDistortionFthetaAPI'
LENS_SCHEMA_EXTENSION = 'omni.usd.schema.omni_lens_distortion'

def _ensure_lens_schema():
    """Load the schema omitted by IsaacLab 2.3's headless rendering experience.

    Merely authoring apiSchemas and custom attributes is insufficient: without
    its registered USD definition, RTX silently renders the pinhole defaults.
    """
    import omni.kit.app
    from pxr import Usd

    manager = omni.kit.app.get_app().get_extension_manager()
    if not manager.is_extension_enabled(LENS_SCHEMA_EXTENSION):
        manager.set_extension_enabled_immediate(LENS_SCHEMA_EXTENSION, True)
    if Usd.SchemaRegistry().FindAppliedAPIPrimDefinition(LENS_SCHEMA) is None:
        raise RuntimeError(f"Required native lens schema is not registered: {LENS_SCHEMA}")

def _calibration_paths(calibration_path, assembly_scene=None):
    calibration = Path(calibration_path).expanduser().resolve(strict=True)
    scene = Path(assembly_scene or ASSEMBLY_USD)
    return calibration, scene.expanduser().resolve(strict=True)

def _calibrated_camera(assembly_scene):
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(assembly_scene))
    if stage is None:
        raise ValueError(f"Cannot open tool assembly: {assembly_scene}")
    prim = stage.GetPrimAtPath(CALIBRATED_CAMERA_PRIM)
    if not prim or not prim.IsA(UsdGeom.Camera):
        raise ValueError(f"Missing calibrated assembly camera: {CALIBRATED_CAMERA_PRIM}")
    if prim.GetAttribute("omni:lensdistortion:model").Get() != "ftheta":
        raise ValueError("Tool assembly must contain the calibrated ftheta lens model")
    dimensions = (
        prim.GetAttribute("omni:lensdistortion:ftheta:nominalWidth").Get(),
        prim.GetAttribute("omni:lensdistortion:ftheta:nominalHeight").Get(),
    )
    if dimensions != (WIDTH, HEIGHT):
        raise ValueError(f"Calibrated lens dimensions {dimensions} do not match {(WIDTH, HEIGHT)}")
    return stage, prim  # Keep the stage alive while using its prim.
