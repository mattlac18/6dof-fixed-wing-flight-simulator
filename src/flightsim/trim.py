"""Trim solver for steady, wings-level, constant-altitude flight.

"Trim" means finding the angle of attack, elevator deflection and throttle
for which the aircraft is in equilibrium: no linear or angular acceleration.
For wings-level, level flight the pitch angle equals the angle of attack
(flight-path angle is zero), so the unknowns are ``(alpha, elevator,
throttle)`` and the equations are ``u_dot = w_dot = q_dot = 0``.

Starting a simulation from a trimmed state is what makes responses to
control inputs meaningful: otherwise the aircraft immediately begins to
pitch and oscillate on its own.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray
from scipy.optimize import fsolve

from flightsim.aircraft import AircraftParameters
from flightsim.controls import Controls
from flightsim.dynamics import aircraft_derivative
from flightsim.state import make_state


@dataclass(frozen=True)
class TrimResult:
    """Equilibrium condition for level flight."""

    airspeed: float  #: [m/s]
    altitude: float  #: [m]
    alpha: float  #: angle of attack = pitch angle [rad]
    controls: Controls
    state: NDArray[np.float64]  #: ready-to-simulate 13-element state
    residual: float  #: max |acceleration| at the solution (should be ~0)


def _level_state(airspeed: float, altitude: float, alpha: float) -> NDArray[np.float64]:
    return make_state(
        position_ned=(0.0, 0.0, -altitude),
        velocity_body=(airspeed * np.cos(alpha), 0.0, airspeed * np.sin(alpha)),
        euler_angles=(0.0, alpha, 0.0),
    )


def trim_level_flight(
    aircraft: AircraftParameters,
    airspeed: float,
    altitude: float,
    tol: float = 1e-6,
) -> TrimResult:
    """Solve for steady, wings-level flight at the given airspeed and altitude.

    Raises
    ------
    RuntimeError
        If no physically valid trim is found (e.g. throttle outside [0, 1],
        which means the aircraft cannot fly level at that speed).
    """

    def residuals(z: NDArray[np.float64]) -> NDArray[np.float64]:
        alpha, elevator, throttle = z
        x = _level_state(airspeed, altitude, alpha)
        # Throttle is clipped only to satisfy Controls validation while the
        # solver explores; validity is checked after convergence.
        u = Controls(elevator=elevator, throttle=float(np.clip(throttle, 0.0, 1.0)))
        x_dot = aircraft_derivative(0.0, x, u, aircraft)
        return np.array([x_dot[3], x_dot[5], x_dot[11]])  # u_dot, w_dot, q_dot

    solution, _info, ier, msg = fsolve(residuals, x0=[0.05, 0.0, 0.5], full_output=True)
    alpha, elevator, throttle = (float(v) for v in solution)
    if ier != 1 or not 0.0 <= throttle <= 1.0:
        raise RuntimeError(
            f"Trim failed at Va={airspeed} m/s, h={altitude} m "
            f"(alpha={np.degrees(alpha):.2f} deg, throttle={throttle:.3f}): {msg}"
        )

    controls = Controls(elevator=elevator, throttle=throttle)
    residual = float(np.max(np.abs(residuals(solution))))
    if residual > tol:
        raise RuntimeError(f"Trim residual {residual:.2e} exceeds tolerance {tol:.0e}.")

    return TrimResult(
        airspeed=airspeed,
        altitude=altitude,
        alpha=alpha,
        controls=controls,
        state=_level_state(airspeed, altitude, alpha),
        residual=residual,
    )
