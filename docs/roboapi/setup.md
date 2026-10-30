# Configuration and calibrated assembly

Use Python 3.11, Isaac Sim 5.1 and Isaac Lab 2.3.2.post1. Install the repository
with the `isaac51` extra according to the upstream guide. A separate lightweight
Python environment can run the offline checks. Do not install a second Isaac
stack into an existing shared simulator environment.

`policies/roboapi/config.py` reads the `ROBOAPI_*` environment variables shown in
`.env.example`. Source the chosen configuration before importing Python modules.
Relative paths resolve against the working directory. The endpoint and model
are normal deployment settings; the API key is read only from `ROBOAPI_TOKEN_FILE`.
Keep that file owner-readable only (mode 0600), under an owner-only directory
(mode 0700). `.env.local`, `.local`, credentials and evaluation outputs are ignored
by Git and Docker. HTTP archives contain request bodies and images, never the
Authorization header; treat output directories as private observations.

## Required assets

### Download the rig assets

The [v1.0.0 rig release](https://github.com/huashanjian/roboapi-playground/releases/tag/roboapi-rig-v1.0.0)
provides `assembly.usdc`, `camera.json` and `actuator.json`, together with a
portable manifest, per-file SHA-256 checksums and coordinate-system notes.
Run these commands from the repository root:

```bash
mkdir -p .local/roboapi/downloads .local/roboapi/rig
cd .local/roboapi/downloads
curl --fail --location --output roboapi-rig-v1.0.0.tar.gz \
  https://github.com/huashanjian/roboapi-playground/releases/download/roboapi-rig-v1.0.0/roboapi-rig-v1.0.0.tar.gz
curl --fail --location --output SHA256SUMS \
  https://github.com/huashanjian/roboapi-playground/releases/download/roboapi-rig-v1.0.0/SHA256SUMS
sha256sum --check SHA256SUMS
tar -xzf roboapi-rig-v1.0.0.tar.gz -C ../rig
(cd ../rig && sha256sum --check SHA256SUMS)
cd ../../..
```

The archive extracts directly into the rig directory expected by `.env.example`.
Its `bundle_manifest.json` describes the distributed inputs; the builder creates
a separate `asset_manifest.json` for the assembled Franka robot.

The Panda model remains a separate prerequisite. Obtain a local copy of
`ISAACLAB_NUCLEUS_DIR/Robots/FrankaEmika/panda_instanceable.usd`, the model used by
upstream RoboLab, together with its referenced `Materials/` and `Props/` files.
Set `ROBOAPI_PANDA_USD` in `.env.local` to that copy. Preserve the model's relative
directory structure and original license terms. The bundle does not supply
Panda geometry, benchmark scenes or the simulator installation.

### Input contract

The builder consumes a Panda instanceable USD, a calibrated tool assembly USD,
a camera JSON and an actuator JSON. Asset resolution requires all referenced
Panda geometry to be available locally. `ROBOAPI_ASSEMBLY_ROOT` identifies the
input assembly root without requiring a particular scene or arm name.

Under the assembly root the builder expects `tool0`, `poke_ee_link`,
`poke_wrist_camera_link`, `RobotiqLinkage` and their fixed joints/materials.
The linkage has one master joint, `positive_outer_joint`, and five mimic joints.
The input camera is selected by `ROBOAPI_CAMERA_PRIM`; its optics must declare
`OmniLensDistortionFthetaAPI`, model `ftheta`, nominal width 640 and height 360.

The camera JSON contains `K_1280x720` (3x3), `D_effective` (four coefficients),
and `T_tcp_camera_optical` (4x4 rigid transform). The input intrinsic matrix is
scaled by 0.5 for the 640x360 image; the camera retains its authored native fisheye
attributes. No remapping, cropping or image rotation is applied. Camera exposure
and physical mounting come from the supplied assembly and calibration.

The actuator JSON contains `arm.delay_s`, `arm.tau_s` and these `gripper` fields:
`delay_s`, `tau_s`, `open_raw`, `closed_raw`, `command_period_s`, `sensor_period_s`,
`sensor_quantum_raw`, and either `max_raw_per_s` or directional
`max_close_raw_per_s` / `max_open_raw_per_s`. The adapter uses this response for
the gripper only; Franka arm IK does not pass through the actuator response model.
Calibration values are deployment inputs and are not bundled experiment records.

The builder exports a standalone Franka/Robotiq USD and an `asset_manifest.json`
next to it. Keep them together. Its version 2 schema records the configured
assembly asset and root explicitly; the runtime does not infer an installation
path or require a particular input arm scene. The manifest records input hashes, joint maps,
optical transforms, mass properties and verification checks. The preflight checks
runtime physics and camera readback in addition to the offline asset checks.

## Renderer

The launcher loads the lens schema before USD composition and enables RTX
Real-Time 2.0 (`RealTimePathTracing`). It restores renderer defaults overwritten
by Isaac Lab presets, with DLSS settings fixed by `camera.py`. Missing native
schema or renderer support is an error. Rendering across different Isaac Sim
versions is not guaranteed to produce identical pixels.
