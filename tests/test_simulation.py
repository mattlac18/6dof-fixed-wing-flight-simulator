"""End-to-end checks: trim, simulation driver and integrator accuracy."""

import numpy as np
import pytest

from flightsim import Controls, example_aircraft, make_state, simulate
from flightsim.trim import trim_level_flight

AC = example_aircraft()


@pytest.fixture(scope="module")
def trim():
    return trim_level_flight(AC, airspeed=25.0, altitude=1000.0)


def test_trim_is_physically_reasonable(trim):
    assert 0.0 < np.degrees(trim.alpha) < 10.0
    assert 0.0 < trim.controls.throttle < 1.0
    assert trim.residual < 1e-6


def test_trimmed_aircraft_stays_trimmed(trim):
    result = simulate(AC, trim.state, duration=20.0, dt=0.01, controls=trim.controls)
    assert not result.terminated_early
    assert np.ptp(result.altitude) < 0.1
    assert np.ptp(result.airspeed) < 0.01
    np.testing.assert_allclose(np.linalg.norm(result.states[:, 6:10], axis=1), 1.0)


def test_elevator_doublet_excites_and_recovers(trim):
    base = trim.controls

    def doublet(t):
        de = 0.05 if 1.0 <= t < 2.0 else (-0.05 if 2.0 <= t < 3.0 else 0.0)
        return Controls(elevator=base.elevator + de, throttle=base.throttle)

    result = simulate(AC, trim.state, duration=10.0, dt=0.01, controls=doublet)
    q = result.states[:, 11]
    assert np.max(np.abs(q)) > 0.05  # the input visibly disturbs pitch rate
    assert np.all(np.isfinite(result.states))


def test_simulation_stops_at_ground():
    x0 = make_state(position_ned=(0, 0, -20.0), velocity_body=(1.0, 0, 0))
    result = simulate(AC, x0, duration=30.0, dt=0.01)
    assert result.terminated_early
    assert result.altitude[-1] <= 0.0


def test_rk4_converges_at_fourth_order(trim):
    """Halving dt should cut RK4 error by ~2^4 = 16."""
    x0 = trim.state.copy()
    x0[11] = 0.2  # pitch-rate disturbance so there is dynamics to resolve

    def final(dt):
        return simulate(AC, x0, duration=2.0, dt=dt, controls=trim.controls).states[-1]

    reference = final(0.0025)
    err_coarse = np.linalg.norm(final(0.04) - reference)
    err_fine = np.linalg.norm(final(0.02) - reference)
    assert 10.0 < err_coarse / err_fine < 22.0


def test_invalid_throttle_rejected():
    with pytest.raises(ValueError):
        Controls(throttle=1.5)
