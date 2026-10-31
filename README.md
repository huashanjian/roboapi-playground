# RoboAPI × RoboLab

**English** | [简体中文](README.zh-CN.md)

[![RoboAPI offline checks](https://github.com/huashanjian/roboapi-playground/actions/workflows/roboapi.yml/badge.svg?branch=feat%2Froboapi-eval)](https://github.com/huashanjian/roboapi-playground/actions/workflows/roboapi.yml?query=branch%3Afeat%2Froboapi-eval)
[![Code license](https://img.shields.io/github/license/huashanjian/roboapi-playground)](LICENSE)

Run [RoboAPI](https://developers.pokebot.com/docs/roboapi/latest) policies in
[NVIDIA RoboLab](https://github.com/NVLabs/RoboLab). This fork adds a RoboAPI
adapter, a calibrated Franka/Robotiq assembly and a wrist fisheye camera.

[Demos](#demos) · [Try it out](#try-it-out) · [How it works](#how-it-works) · [Documentation](#documentation)

## Demos

### Putting electronic devices away

> Put the electronic devices in the grey bin

![The robot moves the grey bin, picks up the mouse and places it inside.](docs/media/roboapi/electronics-preview.gif)

The robot moves the bin, then picks up the mouse and places it inside.

<table width="100%">
  <tr>
    <th width="50%">Stacking three cubes</th>
    <th width="50%">Turning a mug upright</th>
  </tr>
  <tr>
    <td width="50%"><img src="docs/media/roboapi/cube-stacking-preview.gif" alt="The robot stacks three Rubik's cubes, including its grasp adjustments." width="100%" /></td>
    <td width="50%"><img src="docs/media/roboapi/mug-upright-preview.gif" alt="The robot grasps a sideways white mug and turns its opening upwards." width="100%" /></td>
  </tr>
  <tr>
    <td>“Stack the rubiks cubes in a tower”<br />Preview at 3× replay speed.</td>
    <td>“Make sure all the white mugs are upright so that the opening is facing upwards.”</td>
  </tr>
</table>

Selected simulation rollouts. Recording durations reflect replay time and exclude
API waiting time. The external camera shown here is for viewing; the policy
receives the wrist camera image.

## Try it out

Start with **Python 3.11, Isaac Sim 5.1 and Isaac Lab 2.3.2.post1**.
Follow the [setup guide](docs/roboapi/setup.md#from-a-fresh-clone-to-your-first-episode)
to clone this branch, install the [calibrated rig assets](https://github.com/huashanjian/roboapi-playground/releases/tag/roboapi-rig-v1.0.0),
configure `.env.local` and build the robot.

From the repository root, first check the robot, gripper and camera without
calling the model:

```bash
policies/roboapi/run.sh --task BananaInBowlTask --num-runs 1 \
  --preflight-only --output-folder-name roboapi_preflight
```

Then, with your API key configured, run one episode:

```bash
policies/roboapi/run.sh --task BananaInBowlTask --num-runs 1 \
  --output-folder-name roboapi_single
```

The preflight report is at `output/roboapi_preflight/preflight.json`.
The episode writes to `output/roboapi_single/`. To try the featured task, replace
`BananaInBowlTask` with `ElectronicsInBinTask`.

## How it works

RoboLab supplies the task definitions, episode loop and success checks. The
adapter connects that loop to RoboAPI:

1. **Observe:** send the task instruction, one wrist RGB image, measured tool pose
   and gripper opening. Task object states are not sent to the model.
2. **Act:** execute the returned pose and gripper action chunks on the simulated
   Franka/Robotiq robot.
3. **Observe again:** capture the updated wrist view and robot state for the next
   request, until the episode ends.

The calibrated assembly connects the camera view, tool coordinates and gripper
response used by the policy to their simulated counterparts. The
[control notes](docs/roboapi/control.md) explain the coordinate transforms,
tool load compensation and gripper dynamics.

The runner supports RoboLab's **120 task definitions**, with a uniform
120 seconds of simulated time and at most 90 action chunks per episode.
See [evaluation conditions](docs/roboapi/evaluation.md#evaluation-conditions)
for the differences from the upstream robot and time limits.

## Documentation

| To… | Start here |
| --- | --- |
| Install the simulator, configure assets and run a first episode | [Setup and calibration](docs/roboapi/setup.md) |
| Change the model, API endpoint or local paths | [Configuration variables](.env.example) |
| Understand coordinates, arm control and gripper behavior | [Control model](docs/roboapi/control.md) |
| Inspect observations, action chunks and evaluation conditions | [Protocol and evaluation](docs/roboapi/evaluation.md) |
| Run all 120 tasks and collect results | [Complete evaluation tools](docs/roboapi/evaluation.md#complete-evaluation-tools) |

Run outputs include model and camera settings, task and seed information,
returned action chunks and execution logs.

<details>
<summary>Common setup questions</summary>

**Do I need a physical robot?** No. These tasks run in Isaac Sim.

**What comes with the rig bundle?** The tool assembly and camera/actuator
calibration. Obtain the Panda geometry, benchmark scenes and simulator
separately; see [required assets](docs/roboapi/setup.md#required-assets).

**Can I change the model?** Set `ROBOAPI_MODEL` and `ROBOAPI_BASE_URL` in
`.env.local`. The endpoint must implement the
[RoboAPI model API contract](https://developers.pokebot.com/docs/roboapi/latest/model-contract).

**The simulator check fails. Where should I start?** Read
`output/roboapi_preflight/preflight.json`, then check the asset paths and native
fisheye support against the [setup guide](docs/roboapi/setup.md).

</details>

## Tests

Run the offline checks in a separate Python environment:

```bash
python -m pip install -r requirements-roboapi-tests.txt
scripts/test_roboapi.sh
```

These checks run without Isaac Sim or model calls. With PyTorch installed they
also exercise the gripper action buffers on CPU.

## Acknowledgements and license

Built on [NVIDIA RoboLab](https://github.com/NVLabs/RoboLab), using the
[RoboAPI](https://developers.pokebot.com/docs/roboapi/latest) policy interface.
The code is licensed under [Apache 2.0](LICENSE). Upstream history and NVIDIA
copyright notices are retained. Robot, tool and benchmark assets retain their
respective licenses; see [third party notices](THIRD_PARTY_NOTICES.md).

## Star History

<a href="https://www.star-history.com/?repos=huashanjian%2Froboapi-playground&amp;type=date&amp;legend=top-left">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/chart?repos=huashanjian/roboapi-playground&amp;type=date&amp;theme=dark&amp;legend=top-left" />
    <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/chart?repos=huashanjian/roboapi-playground&amp;type=date&amp;legend=top-left" />
    <img alt="GitHub star history for huashanjian/roboapi-playground" src="https://api.star-history.com/chart?repos=huashanjian/roboapi-playground&amp;type=date&amp;legend=top-left" width="800" />
  </picture>
</a>
