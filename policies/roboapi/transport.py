"""RoboAPI session protocol for absolute 30 Hz pose actions.

Contract: https://developers.pokebot.com/docs/roboapi/latest/model-contract
Only the current observation and pending execution receipt are sent each turn.
A failed request ends inference for the session; it is never retried.
"""
from __future__ import annotations

import base64
from dataclasses import dataclass
import io
import json
from pathlib import Path
import re
import time

import httpx
import numpy as np
from PIL import Image


class PolicyUnavailable(RuntimeError):
    """Redacted protocol or transport failure."""


@dataclass(frozen=True)
class ActionChunk:
    arm: np.ndarray  # (T, 7), xyz + WXYZ, absolute flange pose in policy base
    gripper: np.ndarray  # (T,), 0 closed / 1 open
    action_dt: float
    infer_ms: float
    instruction: str
    request_id: str
    request_ms: float
    session_id: str
    session_turn: int

    def __len__(self):
        return len(self.arm)


class SessionPolicyClient:
    """Synchronous client; one inference followed by one execution receipt."""

    def __init__(self, url, token_file, model, *, timeout_s=120., action_dt=1/30,
                 session_enabled=True, http_transport=None):
        if not session_enabled:
            raise ValueError('RoboAPI requires session transport')
        if not np.isclose(action_dt, 1/30, rtol=0, atol=1e-9):
            raise ValueError('RoboAPI actions require a 30 Hz timestep')
        self.url = url.rstrip('/') + '/'
        self.token_file, self.model = Path(token_file), model
        self.timeout_s, self.action_dt = timeout_s, action_dt
        self._http_transport = http_transport
        self._http = None
        self._id = None
        self._resolved_model = None
        self._turn = 1
        self._messages = []
        self._pending = []
        self._unreported = None
        self._assistant = None
        self._status = 'closed'

    @property
    def session_status(self):
        return {'id': self._id, 'model': self._resolved_model, 'next_turn': self._turn,
                'messages': self._messages, 'status': self._status}

    def _new_http(self, timeout):
        token = self.token_file.read_text(encoding='utf-8').strip()
        if not token or '\n' in token or '\r' in token:
            raise PolicyUnavailable('Invalid API credential file')
        return httpx.Client(base_url=self.url, headers={
            'Authorization': 'Bearer ' + token, 'Accept': 'application/json',
            'Content-Type': 'application/json'}, timeout=timeout, trust_env=False,
            transport=self._http_transport)

    @staticmethod
    def _request(http, method, path, body=None, timeout=None):
        try:
            options = {} if timeout is None else {'timeout': timeout}
            response = http.request(method, path, json=body, **options)
            if response.status_code >= 400:
                raise PolicyUnavailable(f'RoboAPI HTTP status {response.status_code}')
            data = response.json()
            if not isinstance(data, dict):
                raise ValueError('Expected an object')
            return response, data
        except PolicyUnavailable:
            raise
        except (httpx.HTTPError, ValueError, TypeError):
            # Error bodies and exception strings can include private request data.
            raise PolicyUnavailable('RoboAPI transport or response failure') from None

    def connect(self):
        if self._http is not None:
            return
        try:
            self._http = self._new_http(self.timeout_s)
            _, data = self._request(self._http, 'GET', 'models')
            if self.model not in {m.get('id') for m in data.get('data', []) if isinstance(m, dict)}:
                raise PolicyUnavailable('Configured model is unavailable')
        except Exception:
            self.close()
            raise PolicyUnavailable('RoboAPI model discovery failed') from None

    def begin_session(self):
        if self._status != 'closed':
            raise PolicyUnavailable('Close the current session before starting another')
        self.connect()
        self._pending, self._messages = [], []
        self._unreported = self._assistant = None
        self._id = None
        self._resolved_model = None
        try:
            _, data = self._request(self._http, 'POST', 'sessions', {'model': self.model})
            sid, turn, messages = data.get('id'), data.get('next_turn', 1), data.get('messages')
            if (not isinstance(sid, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', sid)
                    or type(turn) is not int or turn < 1 or not isinstance(messages, list)
                    or not isinstance(data.get('model'), str) or not data['model']):
                raise PolicyUnavailable('Invalid session response')
            self._resolved_model = data['model']
            self._id, self._turn, self._messages = sid, turn, messages
            self._status = 'open'
        except Exception:
            self._status = 'unknown'
            raise

    def infer(self, pose, gripper, image, instruction):
        if self._status != 'open' or self._unreported is not None:
            raise PolicyUnavailable('Session unavailable or execution feedback missing')
        pose, image = np.asarray(pose, dtype=float), np.asarray(image)
        if (pose.shape != (7,) or not np.isfinite(pose).all() or np.linalg.norm(pose[3:]) < 1e-8
                or not np.isfinite(gripper) or not 0 <= gripper <= 1):
            raise ValueError('Invalid measured pose or gripper ratio')
        if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError('Expected uint8 RGB pixels')
        encoded = io.BytesIO()
        Image.fromarray(image).save(encoded, format='PNG')
        q = pose[3:] / np.linalg.norm(pose[3:])
        state = np.r_[pose[:3], q[[1, 2, 3, 0]], gripper].astype(np.float32).tolist()
        observation = {'role': 'user', 'content': [
            {'type': 'text', 'text': json.dumps({'instruction': instruction, 'state': state})},
            {'type': 'image_url', 'image_url': {'url': 'data:image/png;base64,' +
             base64.b64encode(encoded.getvalue()).decode('ascii')}}]}
        body = {'model': self.model, 'messages': self._pending + [observation],
                'metadata': {'session_id': self._id, 'turn': str(self._turn)}}
        started = time.perf_counter()
        try:
            response, data = self._request(self._http, 'POST', 'chat/completions', body)
            request_ms = (time.perf_counter() - started) * 1000
            rid = response.headers.get('X-Request-ID', '')
            choice = data['choices'][0]
            assistant = choice['message']
            calls = assistant['tool_calls']
            metadata = data['metadata']
            if (not re.fullmatch(r'[a-f0-9]{32}', rid) or assistant.get('role') != 'assistant'
                    or assistant.get('content') is not None or choice.get('finish_reason') != 'tool_calls'
                    or len(calls) != 1 or calls[0].get('type') != 'function'
                    or calls[0].get('id') != 'call_' + rid
                    or calls[0]['function'].get('name') != 'execute_actions'
                    or data.get('model') != self._resolved_model or metadata.get('session_id') != self._id or metadata.get('turn') != str(self._turn)
                    or metadata.get('next_turn') != str(self._turn + 1)):
                raise ValueError('Response identity or tool call mismatch')
            actions = np.asarray(json.loads(calls[0]['function']['arguments'])['output'], dtype=float)
            if actions.ndim != 2 or actions.shape[1] != 8 or len(actions) < 1 or not np.isfinite(actions).all():
                raise ValueError('Invalid action rows')
            norms = np.linalg.norm(actions[:, 3:7], axis=1)
            if (norms < 1e-8).any() or not np.isfinite(norms).all():
                raise ValueError('Invalid action quaternion')
            if ((actions[:, 7] < -.01) | (actions[:, 7] > 1.01)).any():
                raise ValueError('Gripper outside calibrated travel')
            arm = np.column_stack([actions[:, :3], actions[:, [6, 3, 4, 5]] / norms[:, None]])
            chunk = ActionChunk(arm, np.clip(actions[:, 7], 0, 1), self.action_dt,
                                (time.perf_counter()-started)*1000, instruction,
                                rid, request_ms, self._id, self._turn)
            self._messages.extend(self._pending + [
                {'role': 'user', 'content': observation['content'][0]['text']}, assistant])
            self._messages = self._messages[-64:]
            self._pending = []
            self._assistant = assistant
            self._unreported = self._turn
            self._turn += 1
            return chunk
        except Exception:
            # The server may have accepted the turn. Never repeat it or infer again.
            self._status = 'unknown'
            raise PolicyUnavailable('RoboAPI inference failed; session outcome unknown') from None

    def record_execution(self, chunk, executed_steps, *, completed):
        if (chunk.session_id != self._id or chunk.session_turn != self._unreported
                or type(executed_steps) is not int or not 0 <= executed_steps <= len(chunk)):
            raise ValueError('Execution receipt does not match the pending chunk')
        self._pending.extend([self._assistant, {'role': 'tool', 'tool_call_id': 'call_' + chunk.request_id,
            'content': json.dumps({'executed_steps': executed_steps,
                'status': 'completed' if completed and executed_steps == len(chunk) else 'interrupted'})}])
        self._unreported = None

    def end_session(self, reason='stopped'):
        if self._id is None or self._status == 'closed':
            self._status = 'closed'
            return
        try:
            # Keep local shutdown independent of a long model timeout.
            with self._new_http(1.) as http:
                _, data = self._request(http, 'POST', f'sessions/{self._id}/close',
                    {'reason': reason.strip()[:80] or 'stopped', 'messages': self._pending})
            if data.get('id') != self._id or not isinstance(data.get('messages'), list):
                raise ValueError('Invalid close response')
            self._messages, self._pending = data['messages'], []
            self._status = 'closed'
        except Exception:
            self._status = 'unknown'

    def close(self):
        if self._http is not None:
            self._http.close()
            self._http = None
