"""Layout of the 13-element aircraft state vector.

The integrator works with a flat NumPy array. This module is the single
source of truth for which element means what, so no other module has to
hard-code indices.

====== ======================== ========== =======
Index  Quantity                 Frame      Units
====== ======================== ========== =======
0-2    position x_N, y_N, z_N   NED        m
3-5    velocity u, v, w         body       m/s
6-9    quaternion q0..q3        NED->body  -
10-12  angular rate p, q, r     body       rad/s
====== ======================== ========== =======
"""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray

from flightsim.rotations import euler_to_quat

STATE_SIZE = 13

POSITION = slice(0, 3)
VELOCITY = slice(3, 6)
QUATERNION = slice(6, 10)
ANGULAR_RATE = slice(10, 13)


def make_state(
    position_ned: ArrayLike = (0.0, 0.0, 0.0),
    velocity_body: ArrayLike = (0.0, 0.0, 0.0),
    euler_angles: ArrayLike = (0.0, 0.0, 0.0),
    angular_rate_body: ArrayLike = (0.0, 0.0, 0.0),
) -> NDArray[np.float64]:
    """Build a state vector from human-friendly quantities.

    Parameters
    ----------
    position_ned:
        ``[x_N, y_N, z_N]`` in metres. Note ``z_N`` is *down*, so an aircraft
        at 1000 m altitude has ``z_N = -1000``.
    velocity_body:
        ``[u, v, w]`` in m/s, in body axes.
    euler_angles:
        ``[phi, theta, psi]`` (roll, pitch, yaw) in radians.
    angular_rate_body:
        ``[p, q, r]`` in rad/s.
    """
    x = np.zeros(STATE_SIZE)
    x[POSITION] = position_ned
    x[VELOCITY] = velocity_body
    x[QUATERNION] = euler_to_quat(*np.asarray(euler_angles, dtype=float))
    x[ANGULAR_RATE] = angular_rate_body
    return x


def altitude(x: ArrayLike) -> float:
    """Altitude above the reference plane [m] (``h = -z_N``)."""
    return -float(np.asarray(x)[2])
