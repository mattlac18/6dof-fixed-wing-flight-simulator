# Changelog

All notable changes to this project are recorded here.

## [0.1.0] - 2026-11-10

The first working version. The repository goes from an empty scaffold to a running, tested 6-DOF simulator.

### Added

- **`flightsim` package** (`src/flightsim/`):
  - `rotations.py`: Euler angle, quaternion and DCM conversions, plus quaternion kinematics.
  - `state.py`: 13-element state vector layout and the `make_state()` builder.
  - `atmosphere.py`: ISA model from −610 m to 20 km.
  - `controls.py`: `Controls` dataclass with documented sign conventions.
  - `aircraft.py`: `AircraftParameters` and an Aerosonde-like example UAV.
  - `aerodynamics.py`: linear stability-derivative model with a parabolic drag polar.
  - `propulsion.py`: simple throttle-proportional thrust model.
  - `dynamics.py`: nonlinear rigid-body equations of motion.
  - `integrators.py`: forward-Euler and RK4 fixed-step integrators.
  - `simulation.py`: `simulate()` driver and `SimulationResult`.
  - `trim.py`: wings-level, constant-altitude trim solver.
  - `plotting.py`: time-history and 3-D trajectory plots.
- **Test suite** (`tests/`): 49 tests covering rotations, atmosphere, rigid-body dynamics, aerodynamics, trim and the simulation loop. They check results against analytical solutions, conservation laws and the expected RK4 convergence rate.
- **`pyproject.toml`**: makes the project an installable package and configures pytest.
- **`examples/elevator_doublet.py`**: end-to-end demo that produces plots in `results/`.
- **`docs/architecture.md`**: data flow, design principles and a reference for every file.

### Changed

- **`README.md`**: rewritten with modelling assumptions, conventions, usage examples, a roadmap and the verification strategy.
- **`requirements.txt`**: added minimum versions for reproducibility.
- **`.gitignore`**: now ignores packaging build output and generated figures.

### Removed

- `.gitkeep` placeholders from `src/`, `tests/` and `docs/`, which now contain real files.
