"""Example: trim the aircraft, apply an elevator doublet, and plot the response.

Run from the repository root (after ``pip install -e .``):

    python examples/elevator_doublet.py

Figures are written to ``results/``.
"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # write files without needing a display

import numpy as np

from flightsim import Controls, example_aircraft, simulate
from flightsim.plotting import plot_time_history, plot_trajectory_3d
from flightsim.trim import trim_level_flight

RESULTS_DIR = Path(__file__).resolve().parents[1] / "results"


def main() -> None:
    aircraft = example_aircraft()
    trim = trim_level_flight(aircraft, airspeed=25.0, altitude=1000.0)
    print(f"Trim at {trim.airspeed} m/s, {trim.altitude} m:")
    print(f"  alpha    = {np.degrees(trim.alpha):6.2f} deg")
    print(f"  elevator = {np.degrees(trim.controls.elevator):6.2f} deg")
    print(f"  throttle = {trim.controls.throttle:6.3f}")

    def doublet(t: float) -> Controls:
        """+3 deg for 1 s, then -3 deg for 1 s, starting at t = 2 s."""
        delta = np.radians(3.0)
        de = delta if 2.0 <= t < 3.0 else (-delta if 3.0 <= t < 4.0 else 0.0)
        return Controls(elevator=trim.controls.elevator + de, throttle=trim.controls.throttle)

    result = simulate(aircraft, trim.state, duration=60.0, dt=0.01, controls=doublet)

    RESULTS_DIR.mkdir(exist_ok=True)
    plot_time_history(result).savefig(RESULTS_DIR / "elevator_doublet_time_history.png", dpi=120)
    plot_trajectory_3d(result).savefig(RESULTS_DIR / "elevator_doublet_trajectory.png", dpi=120)
    print(f"Saved figures to {RESULTS_DIR}")


if __name__ == "__main__":
    main()
