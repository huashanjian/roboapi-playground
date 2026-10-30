# Copyright (c) 2023 Lifelong Robot Learning
# SPDX-License-Identifier: MIT
# See THIRD_PARTY_NOTICES.md for the full license.
"""Calibrated actuator response with sampled encoder observations.
Gripper inputs and outputs use calibrated travel: 0 closed, 1 open.
Delays, rate limits and sensor quantization retain their raw register units.
"""
from collections import deque
import math
import numpy as np

class ActuatorResponse:

    def __init__(self, q, g, config):
        self.cfg = config
        self.q = np.array(q, dtype=float).copy()
        self.raw = self.target_raw(g)
        self.arm = deque()
        self.grip = deque()
        self.q_goal = self.q.copy()
        self.raw_goal = self.raw
        self.pending_raw = self.raw
        self.last = None
        self.next_command = None
        self.next_sensor = None
        self.sensor_raw = None
        self.sensor_time = None

    @staticmethod
    def _next_deadline(deadline, t, period):
        return deadline + (math.floor((t - deadline + 1e-09) / period) + 1) * period

    def target_raw(self, g):
        """Convert normalized travel to the calibrated motor register range."""
        c = self.cfg['gripper']
        return float(c['closed_raw'] - np.clip(g, 0, 1) * (c['closed_raw'] - c['open_raw']))

    def _ratio_from_raw(self, raw):
        c = self.cfg['gripper']
        return float(np.clip((c['closed_raw'] - raw) / (c['closed_raw'] - c['open_raw']), 0, 1))

    def step(self, t, q, g):
        dt = 0.0 if self.last is None else t - self.last
        if dt < 0:
            raise ValueError('Actuator simulation time must be monotonic')
        if self.last is None:
            self.next_command = t
            if self.next_sensor is None:
                self.next_sensor = t
        self.last = t
        a = self.cfg['arm']
        c = self.cfg['gripper']
        self.arm.append((t + a['delay_s'], np.array(q, dtype=float).copy()))
        self.grip.append((t + c['delay_s'], self.target_raw(g)))
        while self.arm and self.arm[0][0] <= t + 1e-09:
            self.q_goal = self.arm.popleft()[1]
        while self.grip and self.grip[0][0] <= t + 1e-09:
            self.pending_raw = self.grip.popleft()[1]
        period = c.get('command_period_s', 0.0)
        if period <= 0 or t + 1e-09 >= self.next_command:
            self.raw_goal = self.pending_raw
            if period > 0:
                self.next_command = self._next_deadline(self.next_command, t, period)
        alpha = lambda tau: 1.0 if tau <= 0 else -math.expm1(-dt / tau)
        self.q += alpha(a['tau_s']) * (self.q_goal - self.q)
        closing = c.get('max_close_raw_per_s', c.get('max_raw_per_s', 1000000000000.0))
        opening = c.get('max_open_raw_per_s', c.get('max_raw_per_s', 1000000000000.0))
        self.raw += float(np.clip(alpha(c['tau_s']) * (self.raw_goal - self.raw), -opening * dt, closing * dt))
        return self.q.copy(), self._ratio_from_raw(self.raw)

    @staticmethod
    def angle(g, max_angle):
        return float((1 - np.clip(g, 0, 1)) * max_angle)

    @staticmethod
    def ratio(angle, max_angle):
        """Continuous geometric conversion, without a sensor sampling side effect."""
        return float(1 - np.clip(angle / max_angle, 0, 1))

    def observe(self, t, angle, max_angle):
        """Sample the measured joint, quantize raw counts, and normalize travel.

        Call every physics tick so sensor cadence is independent of model calls.
        """
        if self.sensor_time is not None and t < self.sensor_time - 1e-09:
            raise ValueError('Sensor simulation time must be monotonic')
        self.sensor_time = t
        c = self.cfg['gripper']
        period = c.get('sensor_period_s', 0.0)
        if self.next_sensor is None:
            self.next_sensor = t
        if self.sensor_raw is None or period <= 0 or t + 1e-09 >= self.next_sensor:
            raw = self.target_raw(self.ratio(angle, max_angle))
            quantum = c.get('sensor_quantum_raw', 0.0)
            if quantum > 0:
                raw = round(raw / quantum) * quantum
            self.sensor_raw = float(np.clip(raw, c['open_raw'], c['closed_raw']))
            if period > 0:
                self.next_sensor = self._next_deadline(self.next_sensor, t, period)
        return self._ratio_from_raw(self.sensor_raw)


from copy import deepcopy

PHYSICS_DT = 1 / 240
INITIAL_SETTLE_S = 1.0
MAX_ANGLE = 0.79

def restored_controller(snapshot, captured_time, current_time):
    """Restore actuator history without rewinding the shared simulation clock."""
    controller = deepcopy(snapshot)
    delta = current_time - captured_time
    if controller.command_time is not None:
        controller.command_time += delta
    if controller.response:
        for name in ('last', 'next_command', 'next_sensor', 'sensor_time'):
            value = getattr(controller.response, name)
            if value is not None:
                setattr(controller.response, name, value + delta)
        for name in ('arm', 'grip'):
            history = getattr(controller.response, name)
            setattr(controller.response, name, type(history)((t + delta, value) for t, value in history))
    return controller
