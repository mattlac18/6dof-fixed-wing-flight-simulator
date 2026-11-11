"""Nonlinear 6-DOF rigid-body equations of motion.

Assumptions: rigid body, constant mass and inertia, flat non-rotating Earth
(NED is inertial), no wind. In body axes:

* Translational:  ``v_dot = F / m - omega x v``
* Rotational:     ``omega_dot = J^-1 (M - omega x (J omega))``
* Attitude:       ``q_dot = 0.5 * Omega(omega) q``
* Navigation:     ``p_ned_dot = C_bn^T v``

The equations are split into two layers:

* :func:`rigid_body_derivative` is pure mechanics: give it total forces and
  moments and it returns the state derivative. It knows nothing about
  aerodynamics, so it can be verified against analytical solutions.
* :func:`aircraft_derivative` assembles gravity, aerodynamic and propulsive
  loads for a specific aircraft and calls the rigid-body layer.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray

from flightsim.aerodynamics import aerodynamic_forces_moments
from flightsim.aircraft import AircraftParameters
from flightsim.atmosphere import isa
from flightsim.constants import G0
from flightsim.controls import Controls
from flightsim.propulsion import propulsion_forces_moments
from flightsim.rotations import quat_derivative, quat_to_dcm
from flightsim.state import ANGULAR_RATE, POSITION, QUATERNION, STATE_SIZE, VELOCITY, altitude

FloatArray = NDArray[np.float64]


def rigid_body_derivative(
    x: ArrayLike,
    force_body: ArrayLike,
    moment_body: ArrayLike,
    mass: float,
    inertia: ArrayLike,
) -> FloatArray:
    """Time derivative of the state for given total body-axis loads.

    Parameters
    ----------
    x:
        13-element state vector (see :mod:`flightsim.state`).
    force_body, moment_body:
        Total external force [N] and moment about the CG [N*m], body axes.
    mass:
        Aircraft mass [kg].
    inertia:
        3x3 body-axis inertia tensor [kg*m^2].
    """
    x = np.asarray(x, dtype=float)
    J = np.asarray(inertia, dtype=float)
    v = x[VELOCITY]
    q = x[QUATERNION]
    w = x[ANGULAR_RATE]

    x_dot = np.empty(STATE_SIZE)
    x_dot[POSITION] = quat_to_dcm(q).T @ v
    x_dot[VELOCITY] = np.asarray(force_body) / mass - np.cross(w, v)
    x_dot[QUATERNION] = quat_derivative(q, w)
    x_dot[ANGULAR_RATE] = np.linalg.solve(J, np.asarray(moment_body) - np.cross(w, J @ w))
    return x_dot


def gravity_force_body(x: ArrayLike, mass: float) -> FloatArray:
    """Weight vector expressed in body axes [N]."""
    q = np.asarray(x, dtype=float)[QUATERNION]
    return quat_to_dcm(q) @ np.array([0.0, 0.0, mass * G0])


def aircraft_derivative(
    t: float,
    x: ArrayLike,
    controls: Controls,
    aircraft: AircraftParameters,
) -> FloatArray:
    """Full aircraft state derivative: gravity + aerodynamics + propulsion.

    ``t`` is unused today but kept so the signature matches what ODE
    integrators expect, and so time-varying effects (e.g. gusts) can be
    added later without changing callers.
    """
    x = np.asarray(x, dtype=float)
    rho = isa(altitude(x)).density

    f_aero, m_aero = aerodynamic_forces_moments(
        x[VELOCITY], x[ANGULAR_RATE], rho, controls, aircraft
    )
    f_prop, m_prop = propulsion_forces_moments(rho, controls, aircraft)
    f_grav = gravity_force_body(x, aircraft.mass)

    return rigid_body_derivative(
        x,
        f_aero + f_prop + f_grav,
        m_aero + m_prop,
        aircraft.mass,
        aircraft.inertia_matrix,
    )
