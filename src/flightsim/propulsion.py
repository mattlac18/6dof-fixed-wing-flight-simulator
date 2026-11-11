"""Propulsion model.

Deliberately simple: thrust acts along the body x-axis through the centre of
gravity (so it produces no moment), scales linearly with throttle, and scales
with air density relative to sea level. It is a placeholder that keeps the
interface stable while a propeller/engine model is developed.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from flightsim.aircraft import AircraftParameters
from flightsim.constants import RHO0_ISA
from flightsim.controls import Controls


def propulsion_forces_moments(
    density: float,
    controls: Controls,
    aircraft: AircraftParameters,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Return propulsive ``(force, moment)`` in body axes [N], [N*m]."""
    thrust = controls.throttle * aircraft.max_thrust * (density / RHO0_ISA)
    return np.array([thrust, 0.0, 0.0]), np.zeros(3)
