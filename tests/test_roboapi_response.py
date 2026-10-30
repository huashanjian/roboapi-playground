"""Calibrated delay, slew, encoder and clock-restoration contracts."""
from types import SimpleNamespace

import numpy as np
import pytest

from policies.roboapi.gripper_response import ActuatorResponse, restored_controller


def config(**overrides):
    gripper = dict(delay_s=.1, tau_s=0., open_raw=0., closed_raw=100.,
                   max_close_raw_per_s=100., max_open_raw_per_s=50.,
                   sensor_period_s=.15, sensor_quantum_raw=2.)
    gripper.update(overrides)
    return {'arm': {'delay_s': 0., 'tau_s': 0.}, 'gripper': gripper}


def test_delayed_close_and_directional_slew_use_simulated_time():
    c = ActuatorResponse([], 1., config())
    for i in range(11):
        _, g = c.step(i/100, [], 0.)
    assert g == pytest.approx(.99)  # First delayed close command, one 10ms interval.
    for i in range(11, 61): c.step(i/100, [], 0.)
    assert c.raw == pytest.approx(51.)
    for i in range(61, 161): c.step(i/100, [], 1.)
    assert c.raw == pytest.approx(16.)  # Opening starts only after the queued delay.
    with pytest.raises(ValueError): c.step(0., [], 1.)


def test_encoder_reports_sampled_measured_joint_independently_of_target():
    c = ActuatorResponse([], 1., config())
    assert c.observe(0., 0., 1.) == 1.
    c.step(0., [], 0.)
    assert c.observe(.14, .51, 1.) == 1.
    assert c.observe(.15, .51, 1.) == .48  # round(51/2)*2 = 52 raw counts.
    assert c.observe(.16, .1, 1.) == .48
    assert c.observe(.30, .1, 1.) == .9
    with pytest.raises(ValueError): c.observe(.1, 0., 1.)


def test_restoration_shifts_pending_deadlines_and_keeps_template_immutable():
    c = ActuatorResponse([], 1., config())
    c.observe(1., .2, 1.)
    c.step(1., [], 0.)
    original = SimpleNamespace(response=c, command_time=1.)
    shifted = restored_controller(original, 1., 10.)
    assert shifted.command_time == 10.
    assert shifted.response.last == 10.
    assert shifted.response.next_sensor == pytest.approx(10.15)
    assert shifted.response.grip[0][0] == pytest.approx(10.1)
    shifted.response.step(10.1, [], 1.)
    assert c.last == 1. and c.grip[0][0] == 1.1
    np.testing.assert_equal(c.q, [])
