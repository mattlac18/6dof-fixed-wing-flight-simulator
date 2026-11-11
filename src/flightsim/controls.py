"""Pilot / autopilot control inputs.

Sign conventions (matching the aerodynamic derivatives in ``aircraft.py``):

* ``elevator`` > 0: trailing edge down -> nose-down pitching moment.
* ``aileron``  > 0: produces a positive (right-wing-down) rolling moment.
* ``rudder``   > 0: trailing edge left  -> nose-left yawing moment.
* ``throttle`` in [0, 1]: fraction of maximum thrust.

Deflections are in radians.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Controls:
    """Control-surface deflections [rad] and throttle setting [0-1]."""

    elevator: float = 0.0
    aileron: float = 0.0
    rudder: float = 0.0
    throttle: float = 0.0

    def __post_init__(self) -> None:
        if not 0.0 <= self.throttle <= 1.0:
            raise ValueError(f"Throttle must be in [0, 1], got {self.throttle}.")
