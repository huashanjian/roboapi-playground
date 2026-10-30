# RoboLab with RoboAPI

Run vision-based robot policies through [PokeBot RoboAPI](https://developers.pokebot.com/docs/roboapi/latest)
on the [NVIDIA RoboLab](https://github.com/NVLabs/RoboLab) benchmark.
The integration uses a Franka arm, a calibrated Robotiq linkage and a single wrist
fisheye camera. It reuses RoboLab's task definitions, episode loop and success predicates.

The policy receives an instruction, wrist RGB image, measured flange pose and
measured gripper opening. Object state and privileged task observations are not sent.
The default model is `poke-aurora-preview` at `https://api.pokebot.com/v1`;
both values are configurable.

## Setup

Install RoboLab's Isaac Sim 5.1 / Isaac Lab 2.3.2 environment using the
[upstream installation guide](https://github.com/NVLabs/RoboLab/tree/b6cfcf6ccd2113c4cd1216d35b33024a7d5691da#installation).
The calibrated camera requires the native `OmniLensDistortionFthetaAPI` schema.

Copy [.env.example](.env.example) to `.env.local` and set paths to your Panda USD,
tool assembly USD, camera calibration and actuator calibration. Keep credentials
in the configured private token file. These assets and credentials are local
installation inputs and are excluded from version control and Docker builds.

```bash
set -a
source .env.local
set +a
# Run with a Python environment providing USD (pxr) and NumPy.
python -m policies.roboapi.build_asset
# Verify mechanics, control and rendering without calling the model.
policies/roboapi/run.sh --task BananaInBowlTask --num-runs 1 \
  --preflight-only --output-folder-name roboapi_preflight
# Run one evaluated episode.
policies/roboapi/run.sh --task BananaInBowlTask --num-runs 1 \
  --output-folder-name roboapi_single
```

See [configuration and rig requirements](docs/roboapi/setup.md),
[protocol and evaluation](docs/roboapi/evaluation.md) and
[control model](docs/roboapi/control.md).

## Checks

```bash
python -m pip install -r requirements-roboapi-tests.txt
scripts/test_roboapi.sh
```

These checks run without Isaac Sim. With PyTorch installed they also exercise
the gripper action buffers on CPU. A simulator preflight remains necessary for
a new installation or calibrated asset.

## License

RoboLab's upstream history and NVIDIA copyright notices are retained. See
[LICENSE](LICENSE) and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
Supplied robot, tool and benchmark assets retain their respective licenses.
