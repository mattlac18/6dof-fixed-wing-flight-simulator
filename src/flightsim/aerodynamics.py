"""Aerodynamic forces and moments.

Model
-----
A linear stability-and-control-derivative model, valid for small angles of
attack and sideslip (attached flow, no stall):

* ``CL = CL_0 + CL_alpha*alpha + CL_q*q_hat + CL_de*de``
* ``CD = CD_0 + k*CL^2`` (parabolic drag polar)
* ``Cm = Cm_0 + Cm_alpha*alpha + Cm_q*q_hat + Cm_de*de``
* ``CY, Cl, Cn`` are linear in ``beta``, ``p_hat``, ``r_hat``, ``da``, ``dr``

where ``p_hat = p*b/(2Va)``, ``q_hat = q*c/(2Va)``, ``r_hat = r*b/(2Va)``.
Lift and drag act in the stability frame and are rotated into body axes
through the angle of attack.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

from flightsim.aircraft import AircraftParameters
from flightsim.controls import Controls

FloatArray = NDArray[np.float64]

#: Below this airspeed [m/s] aerodynamic forces are set to zero to avoid
#: dividing by zero in the non-dimensional rates and flow angles.
MIN_AIRSPEED = 1e-3


@dataclass(frozen=True)
class AirData:
    """Airspeed and flow angles derived from body-axis velocity."""

    airspeed: float  #: Va [m/s]
    alpha: float  #: angle of attack [rad]
    beta: float  #: sideslip angle [rad]


def air_data(velocity_body: ArrayLike) -> AirData:
    """Compute airspeed, angle of attack and sideslip (no wind)."""
    u, v, w = np.asarray(velocity_body, dtype=float)
    va = float(np.sqrt(u * u + v * v + w * w))
    if va < MIN_AIRSPEED:
        return AirData(va, 0.0, 0.0)
    return AirData(va, float(np.arctan2(w, u)), float(np.arcsin(v / va)))


def aerodynamic_forces_moments(
    velocity_body: ArrayLike,
    angular_rate_body: ArrayLike,
    density: float,
    controls: Controls,
    aircraft: AircraftParameters,
) -> tuple[FloatArray, FloatArray]:
    """Return aerodynamic ``(force, moment)`` in body axes [N], [N*m]."""
    ad = air_data(velocity_body)
    if ad.airspeed < MIN_AIRSPEED:
        return np.zeros(3), np.zeros(3)

    p, q, r = np.asarray(angular_rate_body, dtype=float)
    va, alpha, beta = ad.airspeed, ad.alpha, ad.beta
    ac = aircraft
    b, c, S = ac.wing_span, ac.mean_chord, ac.wing_area

    p_hat = p * b / (2 * va)
    q_hat = q * c / (2 * va)
    r_hat = r * b / (2 * va)
    de, da, dr = controls.elevator, controls.aileron, controls.rudder

    CL = ac.CL_0 + ac.CL_alpha * alpha + ac.CL_q * q_hat + ac.CL_de * de
    CD = ac.CD_0 + ac.induced_drag_factor * CL**2
    Cm = ac.Cm_0 + ac.Cm_alpha * alpha + ac.Cm_q * q_hat + ac.Cm_de * de
    CY = ac.CY_beta * beta + ac.CY_p * p_hat + ac.CY_r * r_hat + ac.CY_da * da + ac.CY_dr * dr
    Cl = ac.Cl_beta * beta + ac.Cl_p * p_hat + ac.Cl_r * r_hat + ac.Cl_da * da + ac.Cl_dr * dr
    Cn = ac.Cn_beta * beta + ac.Cn_p * p_hat + ac.Cn_r * r_hat + ac.Cn_da * da + ac.Cn_dr * dr

    qbar_s = 0.5 * density * va**2 * S
    lift, drag, side = qbar_s * CL, qbar_s * CD, qbar_s * CY

    # Rotate lift and drag from the stability frame into body axes.
    ca, sa = np.cos(alpha), np.sin(alpha)
    force = np.array([-drag * ca + lift * sa, side, -drag * sa - lift * ca])
    moment = np.array([qbar_s * b * Cl, qbar_s * c * Cm, qbar_s * b * Cn])
    return force, moment
