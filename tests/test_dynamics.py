"""Verification of the rigid-body equations of motion against analytical results."""

import numpy as np
import pytest

from flightsim.constants import G0
from flightsim.dynamics import gravity_force_body, rigid_body_derivative
from flightsim.integrators import rk4_step
from flightsim.rotations import quat_normalize
from flightsim.state import ANGULAR_RATE, QUATERNION, make_state

INERTIA = np.array([[0.8, 0.0, -0.1], [0.0, 1.1, 0.0], [-0.1, 0.0, 1.8]])
MASS = 10.0


def integrate(f, x0, dt, n):
    x = np.array(x0, dtype=float)
    for k in range(n):
        x = rk4_step(f, k * dt, x, dt)
        x[QUATERNION] = quat_normalize(x[QUATERNION])
    return x


def test_ballistic_trajectory_matches_closed_form():
    """Gravity only, no rotation: x = v0*t, z = 0.5*g*t^2 (NED, z down)."""
    v0, t_end, dt = 20.0, 3.0, 0.01
    x0 = make_state(velocity_body=(v0, 0, 0))

    def f(_t, x):
        return rigid_body_derivative(x, gravity_force_body(x, MASS), np.zeros(3), MASS, INERTIA)

    x = integrate(f, x0, dt, int(t_end / dt))
    np.testing.assert_allclose(x[0:3], [v0 * t_end, 0.0, 0.5 * G0 * t_end**2], rtol=1e-9)


def test_spin_about_principal_axis_is_steady():
    """Torque-free rotation about the pitch (principal) axis stays constant."""
    x0 = make_state(angular_rate_body=(0.0, 0.7, 0.0))

    def f(_t, x):
        return rigid_body_derivative(x, np.zeros(3), np.zeros(3), MASS, INERTIA)

    x = integrate(f, x0, 0.01, 1000)
    np.testing.assert_allclose(x[ANGULAR_RATE], [0.0, 0.7, 0.0], atol=1e-12)


def test_torque_free_motion_conserves_energy_and_momentum():
    """Tumbling body: rotational KE and inertial |H| must be conserved."""
    x0 = make_state(euler_angles=(0.2, 0.1, -0.3), angular_rate_body=(1.0, 0.5, -0.8))

    def f(_t, x):
        return rigid_body_derivative(x, np.zeros(3), np.zeros(3), MASS, INERTIA)

    def invariants(x):
        w = x[ANGULAR_RATE]
        h = INERTIA @ w
        return 0.5 * w @ h, np.linalg.norm(h)

    ke0, h0 = invariants(x0)
    ke1, h1 = invariants(integrate(f, x0, 0.005, 2000))
    assert ke1 == pytest.approx(ke0, rel=1e-8)
    assert h1 == pytest.approx(h0, rel=1e-8)


def test_constant_body_force_gives_constant_acceleration():
    x0 = make_state()
    x_dot = rigid_body_derivative(x0, (20.0, -10.0, 5.0), np.zeros(3), MASS, INERTIA)
    np.testing.assert_allclose(x_dot[3:6], [2.0, -1.0, 0.5])
