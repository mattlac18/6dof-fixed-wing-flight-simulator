"""Aircraft physical and aerodynamic parameters.

An :class:`AircraftParameters` instance fully describes one airframe:
mass properties, reference geometry, propulsion and the non-dimensional
stability and control derivatives used by :mod:`flightsim.aerodynamics`.

Derivative naming: ``C<coef>_<variable>``, e.g. ``CL_alpha`` is dCL/dalpha
(per radian) and ``Cm_q`` is dCm/d(q*c/(2*Va)).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class AircraftParameters:
    """All parameters needed to simulate one fixed-wing aircraft (SI units)."""

    name: str

    # --- Mass properties ---
    mass: float  #: [kg]
    Jx: float  #: roll moment of inertia [kg*m^2]
    Jy: float  #: pitch moment of inertia [kg*m^2]
    Jz: float  #: yaw moment of inertia [kg*m^2]
    Jxz: float  #: product of inertia [kg*m^2]

    # --- Reference geometry ---
    wing_area: float  #: S [m^2]
    wing_span: float  #: b [m]
    mean_chord: float  #: c [m]
    oswald_efficiency: float  #: e, used for induced drag [-]

    # --- Propulsion ---
    max_thrust: float  #: static sea-level thrust at full throttle [N]

    # --- Longitudinal aerodynamics ---
    CL_0: float
    CL_alpha: float
    CL_q: float
    CL_de: float
    CD_0: float  #: parasite (zero-lift) drag coefficient
    Cm_0: float
    Cm_alpha: float
    Cm_q: float
    Cm_de: float

    # --- Lateral-directional aerodynamics ---
    CY_beta: float
    CY_p: float
    CY_r: float
    CY_da: float
    CY_dr: float
    Cl_beta: float
    Cl_p: float
    Cl_r: float
    Cl_da: float
    Cl_dr: float
    Cn_beta: float
    Cn_p: float
    Cn_r: float
    Cn_da: float
    Cn_dr: float

    @property
    def aspect_ratio(self) -> float:
        """Wing aspect ratio ``b^2 / S``."""
        return self.wing_span**2 / self.wing_area

    @property
    def induced_drag_factor(self) -> float:
        """``k`` in the drag polar ``CD = CD_0 + k * CL^2``."""
        return 1.0 / (np.pi * self.oswald_efficiency * self.aspect_ratio)

    @property
    def inertia_matrix(self) -> NDArray[np.float64]:
        """Body-axis inertia tensor (assumes x-z plane of symmetry)."""
        return np.array(
            [
                [self.Jx, 0.0, -self.Jxz],
                [0.0, self.Jy, 0.0],
                [-self.Jxz, 0.0, self.Jz],
            ]
        )


def example_aircraft() -> AircraftParameters:
    """A small (~13.5 kg) UAV used for examples and tests.

    Values are adapted from the Aerosonde UAV model in Beard & McLain,
    *Small Unmanned Aircraft: Theory and Practice* (2012), Chapter 4 and
    Appendix E. The propulsion model is simplified to a constant maximum
    thrust. Check the values against the source before using them for
    quantitative validation.
    """
    return AircraftParameters(
        name="Example small UAV (Aerosonde-like)",
        mass=13.5,
        Jx=0.8244,
        Jy=1.135,
        Jz=1.759,
        Jxz=0.1204,
        wing_area=0.55,
        wing_span=2.8956,
        mean_chord=0.18994,
        oswald_efficiency=0.9,
        max_thrust=50.0,
        CL_0=0.28,
        CL_alpha=3.45,
        CL_q=0.0,
        CL_de=-0.36,
        CD_0=0.03,
        Cm_0=-0.02338,
        Cm_alpha=-0.38,
        Cm_q=-3.6,
        Cm_de=-0.5,
        CY_beta=-0.98,
        CY_p=0.0,
        CY_r=0.0,
        CY_da=0.0,
        CY_dr=-0.17,
        Cl_beta=-0.12,
        Cl_p=-0.26,
        Cl_r=0.14,
        Cl_da=0.08,
        Cl_dr=0.105,
        Cn_beta=0.25,
        Cn_p=0.022,
        Cn_r=-0.35,
        Cn_da=0.06,
        Cn_dr=-0.032,
    )
