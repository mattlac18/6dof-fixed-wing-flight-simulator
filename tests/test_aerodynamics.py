"""Checks of air-data computation and aerodynamic force/moment signs."""

import numpy as np
import pytest

from flightsim.aerodynamics import aerodynamic_forces_moments, air_data
from flightsim.aircraft import example_aircraft
from flightsim.controls import Controls

AC = example_aircraft()
RHO = 1.225


def test_air_data_known_flow_angles():
    alpha, beta, va = np.radians(5.0), np.radians(-3.0), 30.0
    v_body = va * np.array(
        [np.cos(alpha) * np.cos(beta), np.sin(beta), np.sin(alpha) * np.cos(beta)]
    )
    ad = air_data(v_body)
    assert ad.airspeed == pytest.approx(va)
    assert ad.alpha == pytest.approx(alpha)
    assert ad.beta == pytest.approx(beta)


def test_zero_airspeed_gives_zero_loads():
    f, m = aerodynamic_forces_moments(np.zeros(3), np.zeros(3), RHO, Controls(), AC)
    np.testing.assert_array_equal(f, 0.0)
    np.testing.assert_array_equal(m, 0.0)


def test_symmetric_flight_has_no_lateral_loads():
    f, m = aerodynamic_forces_moments([25.0, 0.0, 2.0], np.zeros(3), RHO, Controls(), AC)
    assert f[1] == pytest.approx(0.0)
    assert m[0] == pytest.approx(0.0)
    assert m[2] == pytest.approx(0.0)


def test_positive_alpha_produces_upward_lift():
    f, _ = aerodynamic_forces_moments([25.0, 0.0, 2.0], np.zeros(3), RHO, Controls(), AC)
    assert f[2] < 0.0  # body z is down, so lift is negative z


def test_positive_elevator_pitches_nose_down():
    v = [25.0, 0.0, 2.0]
    _, m0 = aerodynamic_forces_moments(v, np.zeros(3), RHO, Controls(), AC)
    _, m1 = aerodynamic_forces_moments(v, np.zeros(3), RHO, Controls(elevator=0.1), AC)
    assert m1[1] < m0[1]


def test_pitch_rate_is_damped():
    v = [25.0, 0.0, 2.0]
    _, m0 = aerodynamic_forces_moments(v, np.zeros(3), RHO, Controls(), AC)
    _, m1 = aerodynamic_forces_moments(v, [0.0, 0.5, 0.0], RHO, Controls(), AC)
    assert m1[1] < m0[1]


def test_drag_is_always_positive():
    for alpha in np.radians([-10, -5, 0, 5, 10]):
        v = 25.0 * np.array([np.cos(alpha), 0.0, np.sin(alpha)])
        f, _ = aerodynamic_forces_moments(v, np.zeros(3), RHO, Controls(), AC)
        # Drag is the component of force opposite to the velocity vector.
        assert -(f @ v) / np.linalg.norm(v) > 0.0
