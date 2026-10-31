# Configuration and calibrated assembly

## From a fresh clone to your first episode

Run the commands in this guide in Bash, from the repository root unless stated
otherwise.

### 1. Install the simulator environment

```bash
git clone https://github.com/huashanjian/roboapi-playground.git
cd roboapi-playground
```

Use Python 3.11, Isaac Sim 5.1 and Isaac Lab 2.3.2.post1. Follow the
[upstream installation guide](https://github.com/NVLabs/RoboLab/tree/b6cfcf6ccd2113c4cd1216d35b33024a7d5691da#installation)
for this checkout, selecting the `isaac51` extra. The simulator must provide the
native `OmniLensDistortionFthetaAPI` schema. A separate lightweight Python
environment can run the [offline checks](../../README.md#tests). Do not install
a second Isaac stack into an existing shared simulator environment.

### 2. Install the assets and configure local paths

```bash
cp -n .env.example .env.local
```

Install the rig bundle and Panda model using [Required assets](#required-assets)
below. Obtain RoboLab's benchmark assets using the upstream installation guide.
In `.env.local`, set `ROBOAPI_PANDA_USD` to the local Panda model and
`ROBOAPI_PYTHON` to the simulator environment's Python executable. Building the
robot requires USD (`pxr`) and NumPy in that environment. Keep the rig paths
aligned with the installed files.

The default model is `poke-aurora-preview` at `https://api.pokebot.com/v1`.
Set `ROBOAPI_MODEL` and `ROBOAPI_BASE_URL` in `.env.local` to change them.

`policies/roboapi/config.py` reads the `ROBOAPI_*` environment variables shown in
`.env.example`. Source the chosen configuration before importing Python modules.
Relative paths resolve against the working directory. The API key is read only
from `ROBOAPI_TOKEN_FILE`. `.env.local`, `.local`, credentials and evaluation
outputs are ignored by Git and Docker. HTTP archives contain request bodies and images, never the
Authorization header; treat output directories as private observations.

### 3. Load the configuration and build the robot

```bash
set -a
source .env.local
set +a
"$ROBOAPI_PYTHON" -m policies.roboapi.build_asset
```

This writes the assembled robot to `ROBOAPI_ROBOT_USD` and its
`asset_manifest.json` to `ROBOAPI_RIG_DIR`. Keep those files together; when
changing the robot output directory, also place the manifest beside the USD
with the builder's `--manifest` option.

### 4. Check the simulator without calling the model

```bash
policies/roboapi/run.sh --task BananaOnPlateTask --num-runs 1 \
  --preflight-only --output-folder-name roboapi_preflight
```

No API key is needed for this check. It checks mechanics, control and rendering,
and writes `output/roboapi_preflight/preflight.json` with wrist-camera recordings
in the same directory. Resolve reported failures before running an episode.

### 5. Add your API key and run one episode

Create an owner-only directory for the API key:

```bash
mkdir -p "$(dirname "$ROBOAPI_TOKEN_FILE")"
chmod 700 "$(dirname "$ROBOAPI_TOKEN_FILE")"
```

Save your API key in the file named by `ROBOAPI_TOKEN_FILE`, then make the file
owner-readable only and run the episode:

```bash
chmod 600 "$ROBOAPI_TOKEN_FILE"
policies/roboapi/run.sh --task BananaOnPlateTask --num-runs 1 \
  --output-folder-name roboapi_single
```

Outputs are saved under `output/roboapi_single/`. The launcher uses
`ROBOAPI_PYTHON`; reload `.env.local` before starting a new process after changing
its settings. See [Protocol and evaluation](evaluation.md) for the observations,
actions and evaluation conditions, or [Complete evaluation tools](evaluation.md#complete-evaluation-tools)
to run all 120 tasks.

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
