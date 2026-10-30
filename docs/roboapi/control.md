# Arm, tool and gripper control

The integration inherits RoboLab's Franka high-PD joint controller (stiffness
400, damping 80) and absolute-pose differential IK with damped least squares.
The controlled body is `tool0`, with no extra TCP translation. The builder mounts
the complete tool/gripper/camera assembly at `panda_link7 + [0,0,0.107]` metres,
rotated +90 degrees around Z. A left Rz(-90 degrees) maps robot-base poses to the
policy base; actions use its inverse. The mount must not be applied twice.

## Tool load compensation

`arm_action.py` adds two operations to the inherited controller. PhysX returns a
spatial Jacobian at each body's centre of mass; the adapter shifts its linear
part to the tool0 actor origin using `v_actor = v_com + omega x (p_actor-p_com)`.
It then computes the gravity wrench of the nine tool/gripper/camera rigid bodies
from their USD masses, live centres of mass and the simulation gravity vector.

The applied seven-joint effort is `tau = -J_tool0.T @ [sum(m*g), sum((c-p) x m*g)]`.
The arm has gravity disabled; the attached bodies retain their supplied gravity,
mass and inertia. Compensation acts on the seven arm joints only and uses robot
model state. It does not use task objects or a model-generated torque value.
This is an explicit control extension relative to upstream RoboLab.

Tool payload configuration is distinct from camera/TCP calibration, consistent
with the [RoboAPI embodiment contract](https://developers.pokebot.com/docs/roboapi/latest/hardware-adapters).
Deleting this compensation would change joint efforts and tracking behavior.

## Gripper and physics

The Robotiq master joint moves from 0 (open) to -0.79 radians (closed); the other
five joints retain their mimic constraints without added drives. Commands pass
through the calibrated delay, command cadence, first-order response and
directional speed limits at every 240 Hz physics tick. Observations sample the
measured master joint, apply the calibrated encoder cadence/quantization and
normalize its travel. They do not echo the requested opening.

A one-second open-gripper response history is prewarmed before each episode.
Resets copy that snapshot and shift its queued deadlines and sensor phase to the
current simulation clock. Per-environment histories are independent.

The builder preserves tool mesh, collision, mass, inertia, drive and mimic
properties. The root articulation uses 32/1 solver iterations, moving finger
bodies retain 32/8, and the scene enables CCD with iteration maxima 255/255.
Self-collision is disabled. Runtime preflight checks both the authored asset and
the effective simulated settings before model evaluation.
