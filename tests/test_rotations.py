"""Verification of attitude representations and frame transformations."""

import numpy as np
import pytest

from flightsim.integrators import rk4_step
from flightsim.rotations import (
    euler_rates,
    euler_to_dcm,
    euler_to_quat,
    quat_derivative,
    quat_normalize,
    quat_to_dcm,
    quat_to_euler,
)

# A spread of attitudes, avoiding exactly +/-90 deg pitch (gimbal lock).
ATTITUDES = [
    (0.0, 0.0, 0.0),
    (0.3, -0.2, 1.1),
    (-1.2, 0.8, -2.5),
    (np.pi / 2, 0.1, np.pi / 3),
    (2.9, -1.4, 0.05),
]


@pytest.mark.parametrize("angles", ATTITUDES)
def test_dcm_is_proper_rotation(angles):
    C = euler_to_dcm(*angles)
    np.testing.assert_allclose(C @ C.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(C) == pytest.approx(1.0)


@pytest.mark.parametrize("angles", ATTITUDES)
def test_quaternion_is_unit(angles):
    assert np.linalg.norm(euler_to_quat(*angles)) == pytest.approx(1.0)


@pytest.mark.parametrize("angles", ATTITUDES)
def test_euler_quaternion_round_trip(angles):
    np.testing.assert_allclose(quat_to_euler(euler_to_quat(*angles)), angles, atol=1e-12)


@pytest.mark.parametrize("angles", ATTITUDES)
def test_dcm_from_euler_matches_dcm_from_quaternion(angles):
    np.testing.assert_allclose(
        quat_to_dcm(euler_to_quat(*angles)), euler_to_dcm(*angles), atol=1e-12
    )


def test_pure_yaw_matches_hand_derivation():
    psi = 0.4
    c, s = np.cos(psi), np.sin(psi)
    expected = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
    np.testing.assert_allclose(euler_to_dcm(0, 0, psi), expected, atol=1e-15)


def test_pure_pitch_matches_hand_derivation():
    th = 0.4
    c, s = np.cos(th), np.sin(th)
    expected = np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])
    np.testing.assert_allclose(euler_to_dcm(0, th, 0), expected, atol=1e-15)


def test_pure_roll_matches_hand_derivation():
    ph = 0.4
    c, s = np.cos(ph), np.sin(ph)
    expected = np.array([[1, 0, 0], [0, c, s], [0, -s, c]])
    np.testing.assert_allclose(euler_to_dcm(ph, 0, 0), expected, atol=1e-15)


def test_nose_up_pitch_puts_gravity_on_negative_body_x():
    # Pitched 30 deg nose up, part of gravity acts backwards along the body.
    g_body = euler_to_dcm(0, np.radians(30), 0) @ np.array([0, 0, 1.0])
    assert g_body[0] == pytest.approx(-0.5)


def test_constant_yaw_rate_integrates_to_expected_heading():
    """Integrating q_dot under constant r must yield psi = r * t."""
    r, dt, n = 0.2, 0.01, 500
    q = euler_to_quat(0, 0, 0)

    def f(_t, qq):
        return quat_derivative(qq, [0.0, 0.0, r])

    for k in range(n):
        q = quat_normalize(rk4_step(f, k * dt, q, dt))
    assert quat_to_euler(q)[2] == pytest.approx(r * n * dt, abs=1e-9)


def test_quaternion_rates_agree_with_euler_rates():
    angles = np.array([0.3, -0.2, 1.1])
    omega = np.array([0.1, -0.4, 0.25])
    h = 1e-6
    q_next = quat_normalize(euler_to_quat(*angles) + h * quat_derivative(euler_to_quat(*angles), omega))
    numerical = (quat_to_euler(q_next) - angles) / h
    np.testing.assert_allclose(numerical, euler_rates(angles[0], angles[1], omega), atol=1e-5)
