# roboapi-playground

**English** | [简体中文](README.zh-CN.md)

[![RoboAPI offline checks](https://github.com/huashanjian/roboapi-playground/actions/workflows/roboapi.yml/badge.svg?branch=feat%2Froboapi-eval)](https://github.com/huashanjian/roboapi-playground/actions/workflows/roboapi.yml?query=branch%3Afeat%2Froboapi-eval)
[![Code license](https://img.shields.io/github/license/huashanjian/roboapi-playground)](LICENSE)

Try [RoboAPI](https://developers.pokebot.com/docs/roboapi/latest) policies on
[NVIDIA RoboLab](https://github.com/NVLabs/RoboLab) manipulation tasks with a
Franka/Robotiq robot and a wrist fisheye camera, using the supplied rig calibration.

This playground calls RoboAPI as an external policy service. The code here
connects its public API to RoboLab, assembles the simulation rig and saves
rollout logs. RoboLab's task definitions, episode loop and success checks are retained.

[Demos](#demos) · [Try it out](#try-it-out) · [Run the demo tasks](#run-the-demo-tasks) · [Codebase tour](#codebase-tour)

[Integration notes](#integration-notes) · [Inspect a rollout](#inspecting-a-rollout) · [Extend the integration](#extending-the-integration) · [Documentation](#documentation)

<a id="demos"></a>

## 🎬 Demos

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

<a id="try-it-out"></a>

## 🛝 Try it out!

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
The episode writes to `output/roboapi_single/`. More tasks are listed in the
[RoboLab task catalog](robolab/tasks/README.md).

Keep `--task` and `--num-runs` explicit for a short trial. If both are omitted,
a normal evaluation targets all 120 tasks with 10 episodes per task; completed
episodes in the same output directory are skipped.

<a id="run-the-demo-tasks"></a>

## 🧾 Run the demo tasks

With the setup complete and `.env.local` loaded, these commands run one new
episode of each featured task, using seed 0 and your configured model:

```bash
policies/roboapi/run.sh --task ElectronicsInBinTask --num-runs 1 --seed 0 \
  --model "$ROBOAPI_MODEL" --output-folder-name electronics_example
policies/roboapi/run.sh --task Stack3RubiksCubeTask --num-runs 1 --seed 0 \
  --model "$ROBOAPI_MODEL" --output-folder-name stacking_example
policies/roboapi/run.sh --task ReorientWhiteMugsTask --num-runs 1 --seed 0 \
  --model "$ROBOAPI_MODEL" --output-folder-name mug_example
```

The outputs go to the three named directories under `output/`. Seeds control
the episode RNG; these tasks retain their fixed RoboLab object layouts.
Use a new output name when changing the model or calibration.

The GIFs' original seed, resolved model version and episode traces have not been
published. These commands run the same tasks with your current configuration;
they do not specify the original recordings' conditions.

<a id="codebase-tour"></a>

## 🗺️ Codebase tour

The simulation adapter sends RoboLab observations to the RoboAPI service, maps the
returned actions into simulator coordinates and reports their execution:

<p align="center">
  <img src="docs/media/roboapi/integration-flow.png" alt="The simulation adapter sends RoboLab observations and execution receipts to the external RoboAPI policy service, then maps and executes returned action chunks in the simulator." width="960" />
</p>

[`client.py`](policies/roboapi/client.py) connects the following observations and
actions. Object states and privileged task observations stay in the simulator.

| Value | Representation |
| --- | --- |
| Wrist image | One 360 × 640 × 3 `uint8` RGB image, encoded as a PNG data URL |
| Instruction | The task's text instruction |
| Measured state | Tool position in metres, orientation and gripper opening ratio |
| Returned action chunk | K × 8 values: `[x, y, z, qx, qy, qz, qw, gripper]` on the API wire |

The [RoboAPI model API contract](https://developers.pokebot.com/docs/roboapi/latest/model-contract)
specifies **absolute** tool poses and an opening ratio (`0` closed, `1` open).
K is chosen by the server. Inside this repository's adapter, poses and archived
`sim_actions` use quaternion order **WXYZ**; `transport.py` converts to/from the
API's **XYZW** order.

| File | What to look for |
| --- | --- |
| [`run.py`](policies/roboapi/run.py) | Preflight, episode execution and run manifests |
| [`client.py`](policies/roboapi/client.py) | Observation/action frame mapping, action chunks and execution feedback |
| [`transport.py`](policies/roboapi/transport.py) | Model discovery, HTTP requests and session/response validation |
| [`build_asset.py`](policies/roboapi/build_asset.py) | Assembling the Franka, Robotiq and camera USD |
| [`registration.py`](policies/roboapi/registration.py) | Environment registration and the tool0 IK configuration |
| [`camera.py`](policies/roboapi/camera.py) | Fisheye optics, rendering settings and runtime checks |
| [`arm_action.py`](policies/roboapi/arm_action.py) | Arm IK, tool0 Jacobian correction and tool load compensation |
| [`gripper_response.py`](policies/roboapi/gripper_response.py) | Gripper delay, response limits and measurement sampling |

<a id="design-notes"></a>
<a id="integration-notes"></a>

## 🧩 Using RoboAPI in RoboLab

These notes explain how this repository uses RoboAPI's public interface and
loads the supplied calibration into the simulator. For the service's interface
definitions, see the RoboAPI [model API](https://developers.pokebot.com/docs/roboapi/latest/model-contract)
and [embodiment documentation](https://developers.pokebot.com/docs/roboapi/latest/hardware-adapters).

### Matching API poses to the simulated tool

In this simulation setup, RoboAPI pose targets are mapped to `tool0` at the
flange centre, rather than the fingertips. The local assembly builder places the
supplied tool, gripper and camera 107 mm along `panda_link7`'s local Z axis,
with a +90° Z rotation. The simulator's measured `tool0` pose already includes
that mounting transform.

The API policy frame and this robot's base frame share an origin but use
different axes. The adapter's [`tool_to_policy_pose`](policies/roboapi/client.py)
converts between them by left-multiplying the 4 × 4 pose transform:

```text
T_policy = Rz(-90°) @ T_robot_tool0
T_robot_tool0 = Rz(+90°) @ T_policy
```

For example, position `(0.3, 0.1, 0.4)` m becomes `(0.1, -0.3, 0.4)` m.
Orientation rotates with it. When checking a returned action, apply the inverse
frame mapping; an additional mounting rotation would rotate the tool twice.
The [frame tests](tests/test_roboapi_client.py) exercise both directions and
this mounting case.

On the simulation side, [`arm_action.py`](policies/roboapi/arm_action.py) shifts
the PhysX Jacobian from the body's centre of mass to the controlled `tool0`
origin and compensates for the attached tool's gravitational load. These are
local controller operations; their equations and joint settings are in the
[control notes](docs/roboapi/control.md).

### Executing returned chunks and reporting progress

This runner requests a chunk, executes its rows in order, then makes another
request with a fresh observation. At its **30 Hz** control rate, a 12-row
response, for example, spans 0.4 seconds of simulated time. Each row advances
eight **240 Hz** physics steps. The adapter uses the length returned by the
service. Its synchronous API call pauses simulated time, so replay speed
does not include network latency.

[`run.py`](policies/roboapi/run.py) acknowledges an action only after `env.step`
returns successfully. At the chunk boundary, `client.py` records the executed
count; `transport.py` sends it with the next observation using RoboAPI's execution
receipt format. If the episode ends partway through a chunk, the receipt records
the executed prefix as interrupted. When reading a rollout, compare the returned
chunk with this receipt to see how much actually ran.

### Loading the camera and gripper calibration

The supplied rig includes the wrist-camera mounting transform and native
F-theta lens settings. [`camera.py`](policies/roboapi/camera.py) uses the
assembly's `OmniLensDistortionFthetaAPI` optics and reads the calibration file.
For the 640 × 360 input, it scales the first two rows of `K_1280x720` by 0.5;
distortion coefficients and extrinsics are unchanged. The adapter sends the
rendered image without cropping, rotation or a separate undistortion pass.

For the gripper, [`gripper_response.py`](policies/roboapi/gripper_response.py)
loads command delay, target update cadence, first-order response and directional
speed limits from the actuator calibration. The **v1.0.0 rig bundle** supplies
a 100 ms command delay, 50 ms target updates and 150 ms encoder sampling,
with one raw encoder count of quantization.

The adapter reports the simulated master joint's measured position after encoder
sampling and quantization. When inspecting a grasp, compare that measurement
with the requested opening: response lag, contact and sampling can make them
differ. The observation sent to RoboAPI contains the measured opening ratio.

<a id="runs-and-evaluation"></a>

## 🧾 Runs and evaluation

Under `output/<run-name>/`, `manifest.json` records the code and package versions,
requested model and endpoint, task and seed lists, camera settings and rig
metadata. `episode_results.jsonl` stores episode outcomes.

Each episode also writes files under `<task>/run_<n>_api/`:

| Files | Contents |
| --- | --- |
| `observation_*.png` and `observation_*.json` | Wrist images, instructions, measured poses and gripper opening |
| `chunk_*.json` | Returned pose/gripper chunks, mapped simulator actions and request/session identifiers |
| `events.jsonl` and `summary.json` | Execution feedback, completed-step counts, timings, errors and the server-resolved model |

Preflight writes to its own output directory. Start with `preflight.json`,
`wrist_initial.png` and `runtime_camera.json` when checking an installation.

The runner supports **120 RoboLab task definitions**, with a uniform
**120-second simulated horizon** and at most **90 action chunks** per episode.
The demo GIFs show selected rollouts; aggregate success rates and model
comparisons are not included in this repository. The custom robot, camera and
time limits are described in [evaluation conditions](docs/roboapi/evaluation.md#evaluation-conditions).
Use the [complete evaluation tools](docs/roboapi/evaluation.md#complete-evaluation-tools)
to schedule all tasks and collect results.

<a id="inspecting-a-rollout"></a>

## 🔬 Inspecting a rollout

After running the electronics example, open
`output/electronics_example/ElectronicsInBinTask/run_0_api/observation_0000.png`.
This is the wrist image sent on the first query. The matching JSON contains
`pose_wxyz`, `native_tool_pose_wxyz`, `gripper_open` and the instruction.
`chunk_0000.json` holds the response to that observation, if the query succeeded.

From the repository root, inspect that episode with Python's standard library:

```bash
python - output/electronics_example/ElectronicsInBinTask/run_0_api <<'PY'
import json
import sys
from pathlib import Path

episode = Path(sys.argv[1])
summary = json.loads((episode / "summary.json").read_text())
for key in ("model", "resolved_model", "status", "executed_steps", "errors"):
    print(f"{key}: {summary.get(key)}")
chunks = sorted(episode.glob("chunk_*.json"))
if chunks:
    path = chunks[0]
    chunk = json.loads(path.read_text())
    obs = json.loads((episode / path.name.replace("chunk_", "observation_")).read_text())
    print("Measured opening before chunk:", obs["gripper_open"])
    print("First requested opening:", chunk["gripper_open"][0])
    print("Returned actions:", len(chunk["sim_actions"]))
    print("Simulated seconds per action:", chunk["action_dt_s"])
    print("Request ID:", chunk["request_id"])
else:
    print("No action chunk was saved; check status and errors above.")
PY
```

Find the same `request_id` in `events.jsonl`: its `execution_feedback` entry
reports how many actions ran and whether the chunk completed. The next numbered
observation shows the measured state after that chunk, when another query occurs.
`summary.json`'s `status` describes the API client lifecycle; task success is
recorded separately in the run's `episode_results.jsonl`.

<a id="extending-the-integration"></a>

## 🛠️ Extending the integration

**Try another model.** Set `ROBOAPI_MODEL` and, if needed, `ROBOAPI_BASE_URL` in
`.env.local`, then reload it with `set -a; source .env.local; set +a` before
launching a new process. The endpoint must implement the
[RoboAPI model API contract](https://developers.pokebot.com/docs/roboapi/latest/model-contract).
An alias is resolved when the session starts; the session pins that version.
Compare `manifest.json`'s requested `model` with `summary.json`'s `resolved_model`.

**Change the camera or gripper calibration.** Point `ROBOAPI_CALIBRATION` and
`ROBOAPI_ACTUATOR` at your calibration files. A different camera assembly also
needs `ROBOAPI_ASSEMBLY_USD` and `ROBOAPI_CAMERA_PRIM` to match. After reloading
the configuration, rebuild with `"$ROBOAPI_PYTHON" -m policies.roboapi.build_asset`
and repeat the preflight command with a fresh output name. Check the wrist image,
runtime camera transform and `gripper_validation` before starting a new episode.
The [asset input contract](docs/roboapi/setup.md#input-contract) lists the JSON
fields and USD structure; editing a JSON path alone does not update an already built robot.

**Change the adapter.** Start at `client.py`'s `_extract_observation` and
`_unpack_response` for data mapping, `transport.py` for the session protocol, and
`registration.py` for simulator observations and control. Keep quaternion order,
coordinate frames and the post-step acknowledgement consistent across those
interfaces. Run the [offline checks](#tests): `test_roboapi_client.py` covers
frame conversion and chunk execution, `test_roboapi_transport.py` covers sessions
and receipts, and `test_roboapi_response.py` / `test_roboapi_gripper.py` cover
gripper response and measured feedback. Changes to rendered views or physical
tracking also need a simulator preflight and a new task run.

<a id="troubleshooting"></a>

## 🩹 Troubleshooting

| Symptom | Check first |
| --- | --- |
| Wrist view looks wrong | Compare preflight `wrist_initial.png` and `runtime_camera.json` with the episode's `observation_*.png`. Check camera paths, native lens support and renderer settings in [`camera.py`](policies/roboapi/camera.py). |
| Arm approaches the wrong location | Compare the native and policy poses in `observation_*.json` with `sim_actions` in `chunk_*.json`. Check the frame mapping in [`client.py`](policies/roboapi/client.py) and the preflight TCP tracking results. |
| Gripper releases or keeps correcting | Compare the returned opening commands in `chunk_*.json` with measured opening in `observation_*.json`. Inspect preflight `gripper_validation`, then the [response model](policies/roboapi/gripper_response.py) and [simulated joint control](policies/roboapi/gripper_action.py). |
| Simulator runs, but API calls fail | Check `ROBOAPI_TOKEN_FILE` and the endpoint/model settings in [`.env.example`](.env.example). Read the episode's `summary.json` and any `http_*_response.json` files; [`transport.py`](policies/roboapi/transport.py) handles session and response validation. |

<details>
<summary>Environment and asset questions</summary>

**Do I need a physical robot?** No. These tasks run in Isaac Sim.

**What comes with the rig bundle?** The tool assembly and camera/actuator
calibration. Obtain the Panda geometry, benchmark scenes and simulator
separately; see [required assets](docs/roboapi/setup.md#required-assets).

</details>

## Tests

Run the offline checks in a separate Python environment:

```bash
python -m pip install -r requirements-roboapi-tests.txt
scripts/test_roboapi.sh
```

These checks run without Isaac Sim or model calls. With PyTorch installed they
also exercise the gripper action buffers on CPU.

## Documentation

- [Installation, local configuration and calibrated assets](docs/roboapi/setup.md)
- [Coordinate frames, arm and gripper control](docs/roboapi/control.md)
- [RoboAPI protocol and evaluation](docs/roboapi/evaluation.md)
- [RoboLab task catalog](robolab/tasks/README.md)

<a id="acknowledgements-and-license"></a>

## 🙏 Acknowledgements and license

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
