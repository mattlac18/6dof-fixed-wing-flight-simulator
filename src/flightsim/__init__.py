"""flightsim: a nonlinear 6-DOF fixed-wing flight dynamics simulator.

The package is organised as a pipeline:

    rotations / state      ->  how the aircraft's state is represented
    atmosphere             ->  air properties at the current altitude
    aircraft               ->  physical and aerodynamic parameters
    aerodynamics/propulsion->  forces and moments acting on the aircraft
    dynamics               ->  equations of motion (state derivative)
    simulation             ->  time integration of the equations of motion
    plotting               ->  visualisation of simulation results

The most commonly used names are re-exported here for convenience.
"""

from flightsim.aircraft import AircraftParameters, example_aircraft
from flightsim.controls import Controls
from flightsim.simulation import SimulationResult, simulate
from flightsim.state import make_state

__all__ = [
    "AircraftParameters",
    "Controls",
    "SimulationResult",
    "example_aircraft",
    "make_state",
    "simulate",
]

__version__ = "0.1.0"
