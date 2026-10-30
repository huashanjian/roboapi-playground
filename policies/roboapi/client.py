"""Serial RoboAPI inference with measured observations and execution receipts.

Internal poses use WXYZ; transport uses XYZW. The tool mount is already in the
native body pose; only Rz(-90deg) maps the robot base into the policy frame.
Gripper commands and measured observations are continuous opening ratios.
"""

from __future__ import annotations

import base64
import json
import math
import time
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

from policies.roboapi.config import BASE_URL, MODEL
from robolab.constants import get_output_dir
from robolab.core.task.instruction import instruction_text
from robolab.eval.base_client import InferenceClient


def _validated_pose(pose_wxyz: np.ndarray) -> np.ndarray:
    pose = np.asarray(pose_wxyz, dtype=np.float64)
    if pose.ndim < 1 or pose.shape[-1] != 7:
        raise ValueError("pose must have shape (..., 7)")
    if not np.isfinite(pose).all():
        raise ValueError("pose must contain only finite values")
    norms = np.linalg.norm(pose[..., 3:7], axis=-1)
    if (norms < 1e-8).any() or not np.isfinite(norms).all():
        raise ValueError("quaternion must have a finite nonzero norm")
    return pose


def tool_to_policy_pose(pose_wxyz: np.ndarray) -> np.ndarray:
    """Map ``(..., 7)`` native tool0 poses into policy base.

    ``T_policy = Rz(-90 deg) @ T_native_tool0``. The tool mounting rotation
    is already part of the body pose and must not be applied a second time.
    The flange center is unchanged; no local translation is introduced.
    """
    pose = _validated_pose(pose_wxyz)
    result = np.empty_like(pose)
    result[..., :3] = pose[..., [1, 0, 2]] * [1, -1, 1]
    w, x, y, z = np.moveaxis(pose[..., 3:], -1, 0)
    result[..., 3:] = np.stack([w + z, x + y, y - x, z - w], axis=-1) * np.sqrt(0.5)
    return result


def policy_to_tool_pose(pose_wxyz: np.ndarray) -> np.ndarray:
    """Inverse of :func:`tool_to_policy_pose`, using only left Rz(+90 deg)."""
    pose = _validated_pose(pose_wxyz)
    result = np.empty_like(pose)
    result[..., :3] = pose[..., [1, 0, 2]] * [-1, 1, 1]
    w, x, y, z = np.moveaxis(pose[..., 3:], -1, 0)
    result[..., 3:] = np.stack([w - z, x - y, y + x, z + w], axis=-1) * np.sqrt(0.5)
    return result


class RoboAPIInferenceClient(InferenceClient):
    open_loop_horizon = None

    def __init__(
        self,
        token_file: str,
        *,
        base_url: str = BASE_URL,
        model: str = MODEL,
        timeout_s: float = 120.0,
        action_dt: float = 1 / 30,
        max_chunks: int = 90,
        session_enabled: bool = True,
        output_dir: str | Path | None = None,
        policy_client: Any = None,
    ) -> None:
        super().__init__()
        if type(max_chunks) is not int or max_chunks < 1:
            raise ValueError("max_chunks must be a positive integer")
        if not math.isclose(action_dt, 1 / 30, rel_tol=0, abs_tol=1e-9):
            raise ValueError("the RoboAPI runner must execute at 30 Hz")
        if policy_client is None:
            from policies.roboapi.transport import SessionPolicyClient

            policy_client = SessionPolicyClient(
                base_url, token_file, model, timeout_s=timeout_s,
                action_dt=action_dt, session_enabled=session_enabled,
            )
        self.policy = policy_client
        self.model = model
        self.action_dt = action_dt
        self.max_chunks = max_chunks
        self.session_enabled = session_enabled
        self._output_root = Path(output_dir) if output_dir is not None else None
        self._episode_dir: Path | None = None
        self._episode_open = False
        self._chunk = None
        self._chunk_actions = None
        self._chunk_executed = 0
        self._feedback_recorded = True
        self._pending_action = False
        self._pending_hold = False
        self._last_action = None
        self._http_index = 0
        self._request_paths: dict[int, Path] = {}
        self._request_started: dict[int, float] = {}
        self._stats: dict[str, Any] = {}
        self.budget_exhausted = False

    @property
    def state(self) -> dict[str, Any]:
        """Public diagnostics; no simulator task state is read by the client."""
        return {**self._stats, "budget_exhausted": self.budget_exhausted,
                "pending_step": self._pending_action}

    @property
    def episode_dir(self) -> Path | None:
        return self._episode_dir

    def _write_json(self, name: str, value: Any) -> None:
        if self._episode_dir is None:
            return
        path = self._episode_dir / name
        path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n")

    def _event(self, event: str, **fields: Any) -> None:
        if self._episode_dir is None:
            return
        with (self._episode_dir / "events.jsonl").open("a") as handle:
            handle.write(json.dumps({"event": event, "time": time.time(), **fields}) + "\n")

    def _save_stats(self) -> None:
        self._write_json("summary.json", self.state)

    def _archive_images(self, value: Any, stem: str) -> Any:
        """Preserve request bodies without duplicating base64 images in JSON."""
        if isinstance(value, dict):
            return {key: self._archive_images(item, f"{stem}_{index}")
                    for index, (key, item) in enumerate(value.items())}
        if isinstance(value, list):
            return [self._archive_images(item, f"{stem}_{index}")
                    for index, item in enumerate(value)]
        if isinstance(value, str) and value.startswith("data:image/"):
            header, encoded = value.split(",", 1)
            extension = "png" if "image/png" in header else "jpg"
            path = self._episode_dir / f"{stem}.{extension}"
            path.write_bytes(base64.b64decode(encoded, validate=True))
            return {"archived_image": path.name, "original_media_type": header[5:].split(";")[0]}
        return value

    def _request_hook(self, request: Any) -> None:
        self._http_index += 1
        stem = f"http_{self._http_index:04d}"
        self._request_paths[id(request)] = Path(stem)
        body = json.loads(request.content) if request.content else None
        self._write_json(f"{stem}_request.json", {
            "method": request.method, "path": request.url.path,
            "body": self._archive_images(body, stem + "_image"),
        })
        # Start after request/image archival so network timing excludes disk I/O.
        self._request_started[id(request)] = time.perf_counter()

    def _response_hook(self, response: Any) -> None:
        response.read()
        finished = time.perf_counter()
        started = self._request_started.pop(id(response.request), None)
        http_latency_s = finished - started if started is not None else None
        if response.request.url.path.endswith("/chat/completions") and http_latency_s is not None:
            self._stats["api_http_latency_s"].append(http_latency_s)
        stem = self._request_paths.pop(id(response.request), Path(f"http_{self._http_index:04d}"))
        try:
            body = response.json()
        except ValueError:
            body = {"invalid_json": True}
        self._write_json(f"{stem}_response.json", {
            "http_status": response.status_code,
            "http_latency_s": http_latency_s,
            "body": self._archive_images(body, str(stem) + "_response_image"),
        })

    def begin_episode(self, episode_idx: int) -> None:
        if self._episode_open:
            raise RuntimeError("reset the previous RoboAPI episode before starting another")
        super().begin_episode(episode_idx)
        root = self._output_root if self._output_root is not None else Path(get_output_dir())
        self._episode_dir = root / f"run_{episode_idx}_api"
        self._episode_dir.mkdir(parents=True, exist_ok=True)
        self._http_index = 0
        self._request_paths.clear()
        self._request_started.clear()
        self._stats = {
            "episode": episode_idx, "model": self.model,
            "session_enabled": self.session_enabled, "max_chunks": self.max_chunks,
            "action_dt_s": self.action_dt, "requests": 0, "responses": 0,
            "executed_steps": 0, "hold_steps": 0, "chunks_completed": 0,
            "api_latency_s": [], "api_http_latency_s": [],
            "latency_definitions": {
                "api_latency_s": "client inference wall time including image encoding and request/response archival",
                "api_http_latency_s": "HTTP transport and response body read, excluding archive writes",
            },
            "status": "starting", "errors": [],
            "observation_frame": "policy base flange-center metres, quaternion WXYZ",
            "api_action_frame": "absolute policy base flange-center metres, quaternion WXYZ",
            "action_frame": "absolute robot-root tool0 metres, quaternion WXYZ",
            "native_observation_frame": "robot-root tool0 metres, quaternion WXYZ",
            "policy_from_tool": "Rz(-90deg) @ T_native_tool0; tool0 already mounted at flange Rz(+90deg)",
            "gripper": "API and simulator continuous opening ratio: 0=closed, 1=open",
        }
        self._episode_open = True
        self.budget_exhausted = False
        try:
            self.policy.connect()
            http = getattr(self.policy, "_http", None)
            if http is not None:
                http.event_hooks = {"request": [self._request_hook], "response": [self._response_hook]}
            if self.session_enabled:
                self.policy.begin_session()
            self._stats["resolved_model"] = (getattr(self.policy, "session_status", None) or {}).get("model", self.model)
            self._stats["status"] = "running"
            self._stats["session_id"] = (getattr(self.policy, "session_status", None) or {}).get("id")
        except Exception as exc:
            self._error("begin_episode", exc)
            raise
        finally:
            self._save_stats()

    def _error(self, phase: str, exc: Exception) -> None:
        # Exception messages can contain third-party request data; never archive them.
        error = {"phase": phase, "type": type(exc).__name__}
        self._stats.setdefault("errors", []).append(error)
        self._stats["status"] = "failed"
        self._event("error", **error)
        self._save_stats()

    def _extract_observation(self, raw_obs: Any, *, env_id: int = 0) -> dict:
        if env_id != 0:
            raise ValueError("RoboAPIInferenceClient supports one environment; use env_id=0")
        proprio = raw_obs["proprio_obs"]
        pos = np.asarray(self._to_numpy(proprio["tcp_pos"], env_id), dtype=float).reshape(3)
        quat = np.asarray(self._to_numpy(proprio["tcp_quat"], env_id), dtype=float).reshape(4)
        grip = float(np.asarray(self._to_numpy(proprio["gripper_open"], env_id)).reshape(1)[0])
        pose = np.concatenate([pos, quat])
        if not np.all(np.isfinite(pose)) or not math.isfinite(grip) or not 0 <= grip <= 1:
            raise ValueError("non-finite TCP observation or gripper outside [0, 1]")
        norm = np.linalg.norm(quat)
        if norm < 1e-8:
            raise ValueError("zero-norm TCP quaternion")
        pose[3:] /= norm
        image = np.asarray(self._to_numpy(raw_obs["image_obs"]["wrist_camera"], env_id))
        if image.ndim != 3 or image.shape[-1] not in (3, 4):
            raise ValueError("wrist_camera must have HxWx3/4 pixels")
        image = image[..., :3]
        if image.dtype != np.uint8:
            raise ValueError("wrist_camera must contain uint8 RGB pixels")
        return {"pose": tool_to_policy_pose(pose), "native_tool_pose": pose,
                "gripper": grip, "image": image}

    def _pack_request(self, extracted_obs: dict, instruction: Any) -> tuple:
        return (extracted_obs["pose"], extracted_obs["gripper"],
                extracted_obs["image"], instruction_text(instruction))

    def _query_server(self, request: tuple) -> Any:
        return self.policy.infer(*request)

    def _unpack_response(self, response: Any) -> np.ndarray:
        arm = np.asarray(response.arm, dtype=np.float64)
        grip = np.asarray(response.gripper, dtype=np.float64)
        if arm.ndim != 2 or arm.shape[1] != 7 or not len(arm) or grip.shape != (len(arm),):
            raise ValueError("RoboAPI must return a nonempty coordinated pose/gripper chunk")
        if not np.isfinite(arm).all() or not np.isfinite(grip).all() or ((grip < 0) | (grip > 1)).any():
            raise ValueError("invalid action chunk")
        if not math.isclose(float(response.action_dt), self.action_dt, rel_tol=0, abs_tol=1e-9):
            raise ValueError("action chunk timestep does not match simulation control timestep")
        norms = np.linalg.norm(arm[:, 3:7], axis=1)
        if (norms < 1e-8).any():
            raise ValueError("zero-norm action quaternion")
        arm = arm.copy()
        arm[:, 3:7] /= norms[:, None]
        tool = policy_to_tool_pose(arm)
        return np.column_stack([tool, grip]).astype(np.float32)

    def _build_visualization(self, extracted_obs: dict) -> np.ndarray:
        return extracted_obs["image"]

    def infer(self, obs: Any, instruction: Any, *, env_id: int = 0) -> dict:
        if env_id != 0:
            raise ValueError("RoboAPIInferenceClient supports one environment; use env_id=0")
        if self._pending_action:
            raise RuntimeError("previous action has no successful env.step acknowledgement")
        if not self._episode_open:
            self.begin_episode(self._eval_episode_idx)
        if self._stats["status"] == "failed":
            raise RuntimeError("RoboAPI episode failed; reset before another inference")
        needs_chunk = self._chunk is None or self._chunk_executed == len(self._chunk_actions)
        if needs_chunk and self._stats["requests"] >= self.max_chunks:
            self.budget_exhausted = True
            self._stats["status"] = "budget_exhausted"
            self._pending_action = self._pending_hold = True
            return {"action": self._last_action.copy(), "viz": None}
        viz = None
        if needs_chunk:
            # The policy consumes observations only at replanning boundaries.
            # Avoid GPU -> CPU image copies while executing cached actions.
            extracted = self._extract_observation(obs, env_id=env_id)
            viz = extracted["image"]
            index = self._stats["requests"]
            Image.fromarray(extracted["image"]).save(self._episode_dir / f"observation_{index:04d}.png")
            self._write_json(f"observation_{index:04d}.json", {
                "pose_wxyz": extracted["pose"].tolist(), "gripper_open": extracted["gripper"],
                "pose_frame": "policy base flange center",
                "native_tool_pose_wxyz": extracted["native_tool_pose"].tolist(),
                "instruction": instruction_text(instruction),
            })
            self._stats["requests"] += 1
            start = time.perf_counter()
            try:
                self._chunk = self._query_server(self._pack_request(extracted, instruction))
                self._chunk_executed = 0
                self._feedback_recorded = False
                self._chunk_actions = self._unpack_response(self._chunk)
                self._stats["responses"] += 1
                self._stats["api_latency_s"].append(time.perf_counter() - start)
                self._write_json(f"chunk_{index:04d}.json", {
                    "arm_wxyz": self._chunk.arm.tolist(), "gripper_open": self._chunk.gripper.tolist(),
                    "arm_wxyz_frame": "policy base flange center",
                    "sim_actions_frame": "robot-root tool0",
                    "sim_actions": self._chunk_actions.tolist(), "action_dt_s": self._chunk.action_dt,
                    "request_id": self._chunk.request_id, "session_id": self._chunk.session_id,
                    "session_turn": self._chunk.session_turn,
                    "request_ms": self._chunk.request_ms, "infer_ms": self._chunk.infer_ms,
                })
                self._save_stats()
            except Exception as exc:
                self._error("inference", exc)
                raise
        self._last_action = self._chunk_actions[self._chunk_executed].copy()
        self._pending_action = True
        self._pending_hold = False
        return {"action": self._last_action.copy(), "viz": viz}

    def on_step_executed(self, *, env_id: int = 0) -> None:
        """Acknowledge exactly one action after a successful simulator step."""
        if env_id != 0:
            raise ValueError("RoboAPIInferenceClient only supports env_id=0")
        if not self._pending_action:
            raise RuntimeError("no RoboAPI action is awaiting a step acknowledgement")
        self._pending_action = False
        if self._pending_hold:
            self._stats["hold_steps"] += 1
            self._pending_hold = False
        else:
            self._chunk_executed += 1
            self._stats["executed_steps"] += 1
            if self._chunk_executed == len(self._chunk_actions):
                self._record_feedback(completed=True)
                self._stats["chunks_completed"] += 1
                if self._stats["requests"] >= self.max_chunks:
                    self.budget_exhausted = True
                    self._stats["status"] = "budget_exhausted"
                # Keep per-step acknowledgements in memory; persist at chunk
                # boundaries and during reset/error instead of every tick.
                self._save_stats()

    def _record_feedback(self, *, completed: bool) -> None:
        if self._feedback_recorded or self._chunk is None:
            return
        try:
            if self.session_enabled:
                self.policy.record_execution(self._chunk, self._chunk_executed, completed=completed)
            self._feedback_recorded = True
            self._event("execution_feedback", request_id=self._chunk.request_id,
                        executed_steps=self._chunk_executed, completed=completed)
        except Exception as exc:
            self._error("execution_feedback", exc)
            raise

    def _finish_episode(self, reason: str) -> None:
        if not self._episode_open:
            return
        try:
            self._record_feedback(completed=False)
        except Exception:
            pass
        try:
            if self.session_enabled:
                self.policy.end_session(reason)
                status = getattr(self.policy, "session_status", None) or {}
                self._stats["session_close_status"] = status.get("status")
                if status.get("status") == "unknown":
                    self._error("session_close", RuntimeError("unconfirmed session close"))
        except Exception as exc:
            self._error("session_close", exc)
        finally:
            self._episode_open = False
            self._stats["end_reason"] = reason
            if self._stats["status"] == "running":
                self._stats["status"] = "closed"
            self._save_stats()
            self.policy.close()

    def reset(self, *, env_id: int | None = None) -> None:
        if env_id not in (None, 0):
            raise ValueError("RoboAPIInferenceClient only supports env_id=0")
        self._finish_episode("episode_reset")
        self._chunk = self._chunk_actions = None
        self._chunk_executed = 0
        self._feedback_recorded = True
        self._pending_action = self._pending_hold = False
        self._last_action = None
        self.budget_exhausted = False
        super().reset(env_id=None)

    def close(self) -> None:
        self._finish_episode("client_close")
        self.policy.close()
