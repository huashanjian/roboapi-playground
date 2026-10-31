# RoboLab with RoboAPI

**English** | [简体中文](README.zh-CN.md)

[![RoboAPI offline checks](https://github.com/huashanjian/roboapi-playground/actions/workflows/roboapi.yml/badge.svg?branch=feat%2Froboapi-eval)](https://github.com/huashanjian/roboapi-playground/actions/workflows/roboapi.yml?query=branch%3Afeat%2Froboapi-eval)
[![Rig asset release](https://img.shields.io/github/v/release/huashanjian/roboapi-playground?label=rig%20assets)](https://github.com/huashanjian/roboapi-playground/releases/latest)
[![Code license](https://img.shields.io/github/license/huashanjian/roboapi-playground)](LICENSE)

**Run and evaluate RoboAPI robot policies in NVIDIA RoboLab.**

Connect [PokeBot RoboAPI](https://developers.pokebot.com/docs/roboapi/latest) to the
[NVIDIA RoboLab](https://github.com/NVLabs/RoboLab) simulation benchmark with a
Franka arm, a calibrated Robotiq gripper assembly and a wrist fisheye camera.

**[Run your first task →](#quick-start)**

[Integration](#integration) · [Documentation](#documentation) · [FAQ](#faq) · [Tests](#tests)

## Integration

- **Evaluate policies from wrist observations.** The model receives the task instruction, one wrist RGB image, measured flange pose and measured gripper opening. Object state and privileged task observations are excluded.
- **Use a calibrated robot assembly.** The integration combines a Franka arm with a Robotiq linkage, native fisheye optics and a calibrated gripper response model.
- **Run RoboLab's 120 tasks.** The adapter reuses the official task definitions, episode loop and success predicates. This configuration uses a uniform 120 seconds of simulated time and at most 90 action chunks per episode; see [evaluation conditions](docs/roboapi/evaluation.md#evaluation-conditions).

## Quick start

### 1. Prepare the environment and assets

Clone this repository, then run the commands below from its root directory in Bash:

```bash
git clone --branch feat/roboapi-eval https://github.com/huashanjian/roboapi-playground.git
cd RoboLab
```

| Requirement | Setup |
| --- | --- |
| Python and simulator | Python 3.11, Isaac Sim 5.1 and Isaac Lab 2.3.2.post1. Follow the [upstream installation guide](https://github.com/NVLabs/RoboLab/tree/b6cfcf6ccd2113c4cd1216d35b33024a7d5691da#installation) for this checkout, using the `isaac51` extra. |
| Camera support | The simulator must provide the native `OmniLensDistortionFthetaAPI` schema. |
| Tool assembly and calibration | Install the [v1.0.0 rig asset bundle](https://github.com/huashanjian/roboapi-playground/releases/tag/roboapi-rig-v1.0.0) using the [asset download instructions](docs/roboapi/setup.md#download-the-rig-assets). |
| Panda model | Supply a local `panda_instanceable.usd` with its referenced `Materials/` and `Props/` files. Preserve their relative directory structure. |

### 2. Configure and build the robot

Create your local configuration:

```bash
cp -n .env.example .env.local
```

Edit `.env.local` to set `ROBOAPI_PANDA_USD` to your Panda model and
`ROBOAPI_PYTHON` to your simulator environment's Python executable. The build
command needs USD (`pxr`) and NumPy in that environment. Keep the rig paths
aligned with the files installed in step 1.

The default model is `poke-aurora-preview` at `https://api.pokebot.com/v1`.
Set `ROBOAPI_MODEL` and `ROBOAPI_BASE_URL` to change them. Before running a model
episode, save your API key in the file named by `ROBOAPI_TOKEN_FILE`, with file
permissions `0600` and parent directory permissions `0700`.

Load the configuration and build the assembled robot:

```bash
set -a
source .env.local
set +a
"$ROBOAPI_PYTHON" -m policies.roboapi.build_asset
```

### 3. Check the simulator setup

Check mechanics, control and rendering without an API key or model call:

```bash
policies/roboapi/run.sh --task BananaInBowlTask --num-runs 1 \
  --preflight-only --output-folder-name roboapi_preflight
```

The check writes its report to `output/roboapi_preflight/preflight.json` and
records the wrist camera view in the same directory. Resolve any reported
failure before running a model episode.

### 4. Run one evaluated episode

With your API key in place, run:

```bash
policies/roboapi/run.sh --task BananaInBowlTask --num-runs 1 \
  --output-folder-name roboapi_single
```

Outputs are saved under `output/roboapi_single/`. Local configuration, installed
assets, credentials and evaluation outputs are excluded from version control
and Docker builds.

## Documentation

| I want to… | Read |
| --- | --- |
| Configure the environment, assets or camera | [Setup and calibration](docs/roboapi/setup.md) |
| Understand coordinates, arm control and gripper behavior | [Control model](docs/roboapi/control.md) |
| Understand observations, actions and evaluation conditions | [Protocol and evaluation](docs/roboapi/evaluation.md) |
| Evaluate all 120 tasks | [Complete evaluation tools](docs/roboapi/evaluation.md#complete-evaluation-tools) |
| Change the model, endpoint or local paths | [Configuration variables](.env.example) |

## FAQ

### Do I need a physical robot?

The evaluation runs in Isaac Sim. It uses the local Panda model, calibrated
tool assembly and RoboLab task assets described in the setup instructions.

### What is included in the rig asset bundle?

The bundle supplies the tool assembly, camera calibration and actuator
calibration. Obtain the Panda geometry, benchmark scenes and simulator
separately; see [required assets](docs/roboapi/setup.md#required-assets).

### What can I check before calling the model?

The [offline tests](#tests) run without Isaac Sim or model calls. The simulator
preflight above checks the assembled robot and rendered camera with no API key
or model call. It requires the simulator and installed assets.

### Can I change the model or API endpoint?

Yes. Set `ROBOAPI_MODEL` and `ROBOAPI_BASE_URL` in `.env.local`, then reload the
configuration before starting a run. The endpoint must implement the
[RoboAPI model API contract](https://developers.pokebot.com/docs/roboapi/latest/model-contract).

### How do I run the full evaluation?

Follow [complete evaluation tools](docs/roboapi/evaluation.md#complete-evaluation-tools)
to prepare the task catalog, start workers and collect results. The default
protocol runs 120 tasks with seeds 0 through 9, for 1,200 task and seed combinations.

### Which conditions should accompany reported results?

Report the Franka/Robotiq assembly, wrist camera configuration, uniform
120 second simulated horizon and limit of 90 action chunks. These differ from
upstream RoboLab's standard Panda gripper, camera and task time limits.
The repository contains no evaluation scores or comparative results.

## Tests

Run the offline checks in a separate Python environment:

```bash
python -m pip install -r requirements-roboapi-tests.txt
scripts/test_roboapi.sh
```

These checks run without Isaac Sim. With PyTorch installed they also exercise
the gripper action buffers on CPU. A simulator preflight remains necessary for
a new installation or calibrated asset.

## License

The code is licensed under [Apache 2.0](LICENSE). RoboLab's upstream history and
NVIDIA copyright notices are retained. Supplied robot, tool and benchmark assets
retain their respective licenses; see [third party notices](THIRD_PARTY_NOTICES.md).

## Star History

<a href="https://www.star-history.com/?repos=huashanjian%2Froboapi-playground&amp;type=date&amp;legend=top-left">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/chart?repos=huashanjian/roboapi-playground&amp;type=date&amp;theme=dark&amp;legend=top-left" />
    <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/chart?repos=huashanjian/roboapi-playground&amp;type=date&amp;legend=top-left" />
    <img alt="GitHub star history for huashanjian/roboapi-playground" src="https://api.star-history.com/chart?repos=huashanjian/roboapi-playground&amp;type=date&amp;legend=top-left" width="800" />
  </picture>
</a>
