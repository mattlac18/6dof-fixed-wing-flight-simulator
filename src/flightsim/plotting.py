"""Standard plots of a :class:`~flightsim.simulation.SimulationResult`."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.figure import Figure

from flightsim.simulation import SimulationResult


def plot_time_history(result: SimulationResult) -> Figure:
    """Airspeed, flow angles, attitude, rates and altitude versus time."""
    t = result.time
    euler = np.degrees(result.euler_angles)
    rates = np.degrees(result.states[:, 10:13])

    fig, axes = plt.subplots(3, 2, figsize=(12, 9), sharex=True)
    fig.suptitle(result.aircraft.name)

    axes[0, 0].plot(t, result.airspeed)
    axes[0, 0].set_ylabel("Airspeed [m/s]")

    axes[0, 1].plot(t, result.altitude)
    axes[0, 1].set_ylabel("Altitude [m]")

    axes[1, 0].plot(t, np.degrees(result.alpha), label="alpha")
    axes[1, 0].plot(t, np.degrees(result.beta), label="beta")
    axes[1, 0].set_ylabel("Flow angles [deg]")
    axes[1, 0].legend()

    for i, name in enumerate(["roll", "pitch", "yaw"]):
        axes[1, 1].plot(t, euler[:, i], label=name)
    axes[1, 1].set_ylabel("Attitude [deg]")
    axes[1, 1].legend()

    for i, name in enumerate(["p", "q", "r"]):
        axes[2, 0].plot(t, rates[:, i], label=name)
    axes[2, 0].set_ylabel("Body rates [deg/s]")
    axes[2, 0].legend()

    pos = result.position_ned
    axes[2, 1].plot(t, pos[:, 0], label="North")
    axes[2, 1].plot(t, pos[:, 1], label="East")
    axes[2, 1].set_ylabel("Position [m]")
    axes[2, 1].legend()

    for ax in axes[-1]:
        ax.set_xlabel("Time [s]")
    for ax in axes.flat:
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_trajectory_3d(result: SimulationResult) -> Figure:
    """3-D flight path (North, East, Altitude)."""
    pos = result.position_ned
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(projection="3d")
    ax.plot(pos[:, 0], pos[:, 1], result.altitude)
    ax.set_xlabel("North [m]")
    ax.set_ylabel("East [m]")
    ax.set_zlabel("Altitude [m]")
    ax.set_title(result.aircraft.name)
    fig.tight_layout()
    return fig
