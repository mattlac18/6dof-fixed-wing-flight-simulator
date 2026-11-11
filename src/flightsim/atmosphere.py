"""International Standard Atmosphere (ISA) model.

Provides temperature, pressure, density and speed of sound as a function of
geopotential altitude, from 610 m below sea level up to 20 km. This covers the
troposphere (linear temperature lapse) and the lower stratosphere
(isothermal), which is far beyond the operating envelope of most fixed-wing
aircraft this simulator targets.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from flightsim.constants import (
    G0,
    GAMMA_AIR,
    H_TROPOPAUSE,
    LAPSE_RATE_TROPOSPHERE,
    P0_ISA,
    R_AIR,
    T0_ISA,
)

MIN_ALTITUDE = -610.0
MAX_ALTITUDE = 20_000.0

_T_TROPOPAUSE = T0_ISA - LAPSE_RATE_TROPOSPHERE * H_TROPOPAUSE
_P_TROPOPAUSE = P0_ISA * (_T_TROPOPAUSE / T0_ISA) ** (G0 / (R_AIR * LAPSE_RATE_TROPOSPHERE))


@dataclass(frozen=True)
class AtmosphereState:
    """Air properties at a given altitude."""

    temperature: float  #: [K]
    pressure: float  #: [Pa]
    density: float  #: [kg/m^3]
    speed_of_sound: float  #: [m/s]


def isa(altitude: float) -> AtmosphereState:
    """Return ISA air properties at ``altitude`` metres.

    Raises
    ------
    ValueError
        If ``altitude`` is outside the supported range.
    """
    h = float(altitude)
    if not MIN_ALTITUDE <= h <= MAX_ALTITUDE:
        raise ValueError(
            f"Altitude {h:.1f} m is outside the ISA model range "
            f"[{MIN_ALTITUDE:.0f}, {MAX_ALTITUDE:.0f}] m."
        )

    if h <= H_TROPOPAUSE:
        temperature = T0_ISA - LAPSE_RATE_TROPOSPHERE * h
        pressure = P0_ISA * (temperature / T0_ISA) ** (G0 / (R_AIR * LAPSE_RATE_TROPOSPHERE))
    else:
        temperature = _T_TROPOPAUSE
        pressure = _P_TROPOPAUSE * np.exp(-G0 * (h - H_TROPOPAUSE) / (R_AIR * temperature))

    density = pressure / (R_AIR * temperature)
    speed_of_sound = np.sqrt(GAMMA_AIR * R_AIR * temperature)
    return AtmosphereState(temperature, pressure, density, float(speed_of_sound))
