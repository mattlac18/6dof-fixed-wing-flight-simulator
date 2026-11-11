"""Physical constants used throughout the simulator (SI units).

Keeping these in one place avoids "magic numbers" scattered across modules
and guarantees that every module uses identical values.
"""

#: Standard gravitational acceleration [m/s^2].
G0 = 9.80665

#: Specific gas constant for dry air [J/(kg*K)].
R_AIR = 287.05287

#: Ratio of specific heats for air [-].
GAMMA_AIR = 1.4

# --- International Standard Atmosphere (ISA) sea-level reference values ---

#: Sea-level standard temperature [K].
T0_ISA = 288.15

#: Sea-level standard pressure [Pa].
P0_ISA = 101_325.0

#: Sea-level standard density [kg/m^3].
RHO0_ISA = 1.225

#: Temperature lapse rate in the troposphere [K/m].
LAPSE_RATE_TROPOSPHERE = 0.0065

#: Geopotential altitude of the tropopause [m].
H_TROPOPAUSE = 11_000.0
