"""Fixed-step numerical integrators.

A fixed step is used (rather than an adaptive one) so that results are
reproducible and step-size convergence studies are straightforward. SciPy's
adaptive ``solve_ivp`` can be added alongside for comparison.
"""

from __future__ import annotations

from collections.abc import Callable

import numpy as np  # noqa: F401  (used by the NDArray type alias)
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]
Derivative = Callable[[float, FloatArray], FloatArray]


def euler_step(f: Derivative, t: float, x: FloatArray, dt: float) -> FloatArray:
    """One forward-Euler step (1st order). Mainly useful as a baseline."""
    return x + dt * f(t, x)


def rk4_step(f: Derivative, t: float, x: FloatArray, dt: float) -> FloatArray:
    """One classical 4th-order Runge-Kutta step (local error O(dt^5))."""
    k1 = f(t, x)
    k2 = f(t + dt / 2, x + dt / 2 * k1)
    k3 = f(t + dt / 2, x + dt / 2 * k2)
    k4 = f(t + dt, x + dt * k3)
    return x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


INTEGRATORS: dict[str, Callable[[Derivative, float, FloatArray, float], FloatArray]] = {
    "euler": euler_step,
    "rk4": rk4_step,
}


def get_integrator(name: str) -> Callable[[Derivative, float, FloatArray, float], FloatArray]:
    """Look up an integrator step function by name."""
    try:
        return INTEGRATORS[name]
    except KeyError:
        raise ValueError(
            f"Unknown integrator {name!r}. Choose from {sorted(INTEGRATORS)}."
        ) from None


__all__ = ["euler_step", "rk4_step", "get_integrator", "INTEGRATORS"]
