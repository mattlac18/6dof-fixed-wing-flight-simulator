"""Time-domain simulation driver.

:func:`simulate` repeatedly advances the aircraft state with a fixed-step
integrator, renormalising the attitude quaternion after every step, and
returns a :class:`SimulationResult` holding the full time history together
with convenient derived quantities (Euler angles, airspeed, altitude...).
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

from flightsim.aerodynamics import air_data
from flightsim.aircraft import AircraftParameters
from flightsim.atmosphere import MIN_ALTITUDE
from flightsim.controls import Controls
from flightsim.dynamics import aircraft_derivative
from flightsim.integrators import get_integrator
from flightsim.rotations import quat_normalize, quat_to_euler
from flightsim.state import POSITION, QUATERNION, VELOCITY

FloatArray = NDArray[np.float64]
ControlSchedule = Controls | Callable[[float], Controls]


@dataclass
class SimulationResult:
    """Time history produced by :func:`simulate`."""

    time: FloatArray  #: shape (N,) [s]
    states: FloatArray  #: shape (N, 13)
    aircraft: AircraftParameters
    terminated_early: bool = False  #: True if the run stopped at the ground

    @property
    def position_ned(self) -> FloatArray:
        return self.states[:, POSITION]

    @property
    def altitude(self) -> FloatArray:
        return -self.states[:, 2]

    @property
    def euler_angles(self) -> FloatArray:
        """Roll, pitch, yaw [rad], shape (N, 3)."""
        return np.array([quat_to_euler(q) for q in self.states[:, QUATERNION]])

    @property
    def airspeed(self) -> FloatArray:
        return np.linalg.norm(self.states[:, VELOCITY], axis=1)

    @property
    def alpha(self) -> FloatArray:
        """Angle of attack [rad]."""
        return np.array([air_data(v).alpha for v in self.states[:, VELOCITY]])

    @property
    def beta(self) -> FloatArray:
        """Sideslip angle [rad]."""
        return np.array([air_data(v).beta for v in self.states[:, VELOCITY]])


def simulate(
    aircraft: AircraftParameters,
    initial_state: ArrayLike,
    duration: float,
    dt: float = 0.01,
    controls: ControlSchedule = Controls(),
    integrator: str = "rk4",
    stop_at_ground: bool = True,
) -> SimulationResult:
    """Simulate the aircraft from ``initial_state`` for ``duration`` seconds.

    Parameters
    ----------
    aircraft:
        Airframe parameters.
    initial_state:
        13-element state vector, e.g. from :func:`flightsim.state.make_state`.
    duration:
        Simulated time [s].
    dt:
        Fixed integration step [s].
    controls:
        Either a constant :class:`Controls` or a function ``t -> Controls``
        (e.g. to apply an elevator doublet).
    integrator:
        Name of the step function in :mod:`flightsim.integrators`.
    stop_at_ground:
        End the run early if altitude reaches 0 m.
    """
    if duration <= 0 or dt <= 0:
        raise ValueError("duration and dt must be positive.")

    step = get_integrator(integrator)
    schedule = controls if callable(controls) else (lambda _t, c=controls: c)

    n_steps = int(round(duration / dt))
    time = np.linspace(0.0, n_steps * dt, n_steps + 1)
    states = np.empty((n_steps + 1, len(initial_state)))
    states[0] = np.asarray(initial_state, dtype=float)
    states[0, QUATERNION] = quat_normalize(states[0, QUATERNION])

    terminated_early = False
    for k in range(n_steps):
        t = time[k]
        u = schedule(t)  # controls held constant over one step (zero-order hold)
        x_next = step(lambda tt, xx: aircraft_derivative(tt, xx, u, aircraft), t, states[k], dt)
        x_next[QUATERNION] = quat_normalize(x_next[QUATERNION])
        states[k + 1] = x_next

        h = -x_next[2]
        if (stop_at_ground and h <= 0.0) or h < MIN_ALTITUDE:
            time, states = time[: k + 2], states[: k + 2]
            terminated_early = True
            break

    return SimulationResult(time, states, aircraft, terminated_early)
