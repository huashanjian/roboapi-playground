# RoboAPI protocol and evaluation

The session transport follows the [RoboAPI model API contract](https://developers.pokebot.com/docs/roboapi/latest/model-contract).
The configured model may be an alias: the session pins its server-resolved version.
Both the requested name and resolved version are archived and audited separately;
subsequent inference responses must retain the pinned version.
Each episode discovers the configured model, creates a session and sends serial
`chat/completions` requests. Each request contains the current instruction,
measured state and one RGB PNG data URL. After a successful simulation step, the
adapter acknowledges one executed action. The next request echoes the assistant
tool call and an execution receipt. A partial final chunk is marked interrupted.
Session closure is best effort with a one-second timeout; local execution does
not wait for a model timeout during shutdown. Inference requests are never retried.

State and actions use `[x,y,z,qx,qy,qz,qw,gripper]` on the wire. Positions are
metres at the flange/tool0 origin, targets are absolute, and gripper travel is
continuous from 0 (closed) to 1 (open). Internally quaternions use WXYZ. The
transport validates turn identity, tool-call identity, finite action rows and
nonzero quaternions. For compatibility, gripper overshoot up to 0.01 is clipped;
larger overshoot fails the session. Complete returned chunks are executed without
truncating, padding or imposing a fixed replanning interval.

The simulator runs at 240 Hz with 30 Hz control. Inference pauses simulated time;
no additional hardware inter-chunk wait is introduced. The model only sees wrist
RGB, instruction, measured tool pose and sampled gripper opening. Viewport video
is optional and is not sent to the model.

## Evaluation conditions

The runner uses the 120 official RoboLab task definitions and success predicates,
with an explicit uniform 120-second simulated horizon and at most 90 chunks.
These conditions differ from the upstream per-task time limits and standard
Panda gripper/camera. Report the custom rig and horizon when comparing results.
The upstream task files, evaluator and episode loop remain unchanged.

The default complete evaluation repeats each task with seeds 0 through 9.
Official fixed layouts are retained; seeds do not imply object-pose randomization.
HTTP timeout and simulated episode duration are separate settings. Infrastructure
interruptions are unscored and preserved; already scored outcomes are never rerun.
A result is complete only after 1,200 unique task/seed slots pass the audit.
No evaluation scores or comparative experiment results are bundled.

## Complete evaluation tools

After configuring the rig, collect the preflight and task catalog into a new
run group. The preparation step verifies these and freezes task assignments,
source hashes and asset hashes before scheduling any model calls.

```bash
policies/roboapi/run.sh --task BananaInBowlTask --num-runs 1 --preflight-only \
  --output-folder-name "$ROBOAPI_RUN_NAME/preflight_final"
policies/roboapi/run.sh --catalog-only --output-folder-name "$ROBOAPI_RUN_NAME/catalog"
python -m tools.roboapi_eval.fetch_full120_assets
python -m tools.roboapi_eval.full120_validate_assets
python -m tools.roboapi_eval.prepare --workers 1
python -m tools.roboapi_eval.full120_supervisor
python -m tools.roboapi_eval.full120_status
```

Asset tools require USD and the installed Isaac MDL search paths. They fetch
only recursively referenced official assets, validate LFS hashes and preserve
authored USD. One missing texture can be supplemented from an identical upstream
texture; the validation report records this. Some official remote material URIs
remain external and can change independently of local hashes.

Run `python -m tools.roboapi_eval.audit_full120_worker --help` for worker audit
options, then `python -m tools.roboapi_eval.full120_collect_results` after all
workers pass final acceptance. Keep output directories private; they contain
images, action sequences and session evidence.
