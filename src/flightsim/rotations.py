"""Attitude representations and conversions between reference frames.

Conventions
-----------
* Earth frame: North-East-Down (NED), treated as inertial (flat Earth).
* Body frame: x forward, y out the right wing, z down.
* Euler angles use the aerospace 3-2-1 sequence: yaw ``psi`` about z, then
  pitch ``theta`` about the new y, then roll ``phi`` about the new x.
* Quaternions are scalar-first, ``q = [q0, q1, q2, q3]``, and describe the
  orientation of the body frame relative to NED.
* ``C_bn`` denotes the direction cosine matrix (DCM) that transforms vector
  components from NED to body axes:  ``v_body = C_bn @ v_ned``.

Formulas follow Stevens, Lewis & Johnson, *Aircraft Control and Simulation*,
3rd ed., Chapter 1.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray

FloatArray = NDArray[np.float64]


def euler_to_dcm(phi: float, theta: float, psi: float) -> FloatArray:
    """Return ``C_bn`` (NED -> body) for 3-2-1 Euler angles in radians."""
    cph, sph = np.cos(phi), np.sin(phi)
    cth, sth = np.cos(theta), np.sin(theta)
    cps, sps = np.cos(psi), np.sin(psi)
    return np.array(
        [
            [cth * cps, cth * sps, -sth],
            [sph * sth * cps - cph * sps, sph * sth * sps + cph * cps, sph * cth],
            [cph * sth * cps + sph * sps, cph * sth * sps - sph * cps, cph * cth],
        ]
    )


def euler_to_quat(phi: float, theta: float, psi: float) -> FloatArray:
    """Convert 3-2-1 Euler angles [rad] to a unit quaternion (scalar first)."""
    cph, sph = np.cos(phi / 2), np.sin(phi / 2)
    cth, sth = np.cos(theta / 2), np.sin(theta / 2)
    cps, sps = np.cos(psi / 2), np.sin(psi / 2)
    return np.array(
        [
            cph * cth * cps + sph * sth * sps,
            sph * cth * cps - cph * sth * sps,
            cph * sth * cps + sph * cth * sps,
            cph * cth * sps - sph * sth * cps,
        ]
    )


def quat_to_dcm(q: ArrayLike) -> FloatArray:
    """Return ``C_bn`` (NED -> body) for a unit quaternion ``q``."""
    q0, q1, q2, q3 = np.asarray(q, dtype=float)
    return np.array(
        [
            [q0**2 + q1**2 - q2**2 - q3**2, 2 * (q1 * q2 + q0 * q3), 2 * (q1 * q3 - q0 * q2)],
            [2 * (q1 * q2 - q0 * q3), q0**2 - q1**2 + q2**2 - q3**2, 2 * (q2 * q3 + q0 * q1)],
            [2 * (q1 * q3 + q0 * q2), 2 * (q2 * q3 - q0 * q1), q0**2 - q1**2 - q2**2 + q3**2],
        ]
    )


def quat_to_euler(q: ArrayLike) -> FloatArray:
    """Convert a unit quaternion to 3-2-1 Euler angles ``[phi, theta, psi]`` [rad].

    Pitch is returned in ``[-pi/2, pi/2]``; roll and yaw in ``(-pi, pi]``.
    Near ``theta = +/-90 deg`` roll and yaw become ill-defined (gimbal lock);
    this is a property of Euler angles, not of the quaternion.
    """
    q0, q1, q2, q3 = np.asarray(q, dtype=float)
    phi = np.arctan2(2 * (q0 * q1 + q2 * q3), q0**2 - q1**2 - q2**2 + q3**2)
    theta = np.arcsin(np.clip(2 * (q0 * q2 - q1 * q3), -1.0, 1.0))
    psi = np.arctan2(2 * (q1 * q2 + q0 * q3), q0**2 + q1**2 - q2**2 - q3**2)
    return np.array([phi, theta, psi])


def quat_normalize(q: ArrayLike) -> FloatArray:
    """Return ``q`` scaled to unit length.

    Numerical integration slowly lets the quaternion norm drift away from 1;
    renormalising after each step removes that drift.
    """
    q = np.asarray(q, dtype=float)
    return q / np.linalg.norm(q)


def quat_derivative(q: ArrayLike, omega_body: ArrayLike) -> FloatArray:
    """Quaternion kinematics: ``q_dot = 0.5 * Omega(omega) @ q``.

    Parameters
    ----------
    q:
        Current attitude quaternion (scalar first).
    omega_body:
        Body angular velocity ``[p, q, r]`` in rad/s.
    """
    p, qr, r = np.asarray(omega_body, dtype=float)
    omega_matrix = np.array(
        [
            [0.0, -p, -qr, -r],
            [p, 0.0, r, -qr],
            [qr, -r, 0.0, p],
            [r, qr, -p, 0.0],
        ]
    )
    return 0.5 * omega_matrix @ np.asarray(q, dtype=float)


def euler_rates(phi: float, theta: float, omega_body: ArrayLike) -> FloatArray:
    """Euler angle rates ``[phi_dot, theta_dot, psi_dot]`` from body rates.

    Singular at ``theta = +/-90 deg``. The simulator integrates quaternions
    instead; this function exists for analysis and cross-checking.
    """
    p, q, r = np.asarray(omega_body, dtype=float)
    sph, cph = np.sin(phi), np.cos(phi)
    tth, cth = np.tan(theta), np.cos(theta)
    return np.array(
        [
            p + (q * sph + r * cph) * tth,
            q * cph - r * sph,
            (q * sph + r * cph) / cth,
        ]
    )
