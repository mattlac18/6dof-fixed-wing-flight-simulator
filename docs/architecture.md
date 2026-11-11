# Architecture

This document explains how the simulator is organised: what each file is for, how data flows between modules, and why the code is split the way it is.

## Data Flow

One simulation step goes through these layers:

```
                    ┌──────────────┐
  Controls(t) ─────►│              │
                    │  dynamics.   │      ┌─────────────┐
  state x ─────────►│  aircraft_   ├─────►│ integrators │──► next state x
                    │  derivative  │ x_dot│  (RK4 step) │
                    └──────┬───────┘      └─────────────┘
                           │ uses
     ┌──────────────┬──────┴───────┬───────────────┬──────────────┐
     ▼              ▼              ▼               ▼              ▼
 atmosphere   aerodynamics    propulsion      gravity        rigid_body_
 (density)    (F, M)          (F, M)          (F)            derivative
                                                             (pure mechanics)
```

`simulation.simulate` runs this loop for the requested duration, renormalising the quaternion after each step, and returns a `SimulationResult`. `trim.trim_level_flight` uses the same `aircraft_derivative` to find an equilibrium state to start from.

## Design Principles

- **Mechanics is kept separate from modelling.** `rigid_body_derivative` takes total forces and moments and knows nothing about aircraft. This means it can be checked against exact analytical solutions (ballistic flight, torque-free tumbling) before any aerodynamics is involved.
- **The state layout is defined in one place.** Only `state.py` knows which array index is which quantity. Every other module uses its named slices (`VELOCITY`, `QUATERNION` and so on).
- **Parameters are data.** Each airframe is an `AircraftParameters` instance. The physics code contains no aircraft-specific numbers.
- **Each model sits behind a stable interface.** Aerodynamics and propulsion each return `(force, moment)` in body axes, so a model can be replaced with a higher-fidelity one without changing the dynamics code.
- **Every module has its own tests.**

## File Reference

### Package: `src/flightsim/`

| File | Purpose |
|---|---|
| `__init__.py` | Marks `flightsim` as a package and re-exports the most-used names (`simulate`, `Controls`, `example_aircraft`, `make_state`). |
| `constants.py` | Physical constants: gravity, gas constant and ISA sea-level values. Defined once so every module uses identical values. |
| `rotations.py` | Attitude maths: Euler angles ↔ quaternions ↔ direction cosine matrices, quaternion kinematics (`q̇ = ½Ω(ω)q`) and Euler-angle rates. Documents the frame conventions. |
| `state.py` | Defines the 13-element state vector layout (position, velocity, quaternion, body rates) and `make_state()`, which builds a state from human-friendly inputs such as Euler angles. |
| `atmosphere.py` | International Standard Atmosphere from −610 m to 20 km: temperature, pressure, density and speed of sound. |
| `controls.py` | The `Controls` dataclass (elevator, aileron, rudder, throttle) and its sign conventions. Rejects throttle values outside [0, 1]. |
| `aircraft.py` | The `AircraftParameters` dataclass (mass, inertia, geometry, thrust and stability/control derivatives) and `example_aircraft()`, a small UAV based on the Aerosonde model in Beard & McLain. |
| `aerodynamics.py` | Air data (airspeed, α, β) and aerodynamic forces/moments from a linear derivative model with a parabolic drag polar. |
| `propulsion.py` | A placeholder thrust model: throttle × max thrust × density ratio, along body x, producing no moment. |
| `dynamics.py` | The 6-DOF equations of motion. `rigid_body_derivative` covers pure rigid-body mechanics; `aircraft_derivative` adds up gravity, aerodynamic and propulsive loads and calls it. |
| `integrators.py` | Fixed-step forward-Euler and classical RK4 step functions, looked up by name. |
| `simulation.py` | `simulate()` runs the time loop, applies constant or time-varying controls and stops at the ground. `SimulationResult` stores the time history and computes derived quantities (Euler angles, airspeed, α, β, altitude). |
| `trim.py` | `trim_level_flight()` solves for the α, elevator and throttle that give steady, wings-level, constant-altitude flight. |
| `plotting.py` | Standard Matplotlib figures: a six-panel time history and a 3-D trajectory. |

### Tests: `tests/`

| File | What it verifies |
|---|---|
| `test_rotations.py` | DCMs are proper rotations; Euler↔quaternion round-trips; the two DCM routes agree; single-axis matrices match hand derivations; quaternion integration under constant yaw rate gives ψ = r·t; quaternion and Euler-rate kinematics agree. |
| `test_atmosphere.py` | ISA values at sea level, 11 km and 20 km match standard tables; density decreases with altitude; out-of-range altitudes raise errors. |
| `test_dynamics.py` | Gravity-only flight matches the closed-form parabola; a spin about a principal axis stays steady; torque-free tumbling conserves kinetic energy and angular momentum. |
| `test_aerodynamics.py` | Air data recovers known α and β; zero airspeed gives zero loads; symmetric flight gives no lateral loads; lift, elevator, pitch-damping and drag all have the correct signs. |
| `test_simulation.py` | The trim solution is reasonable and holds for 20 s; an elevator doublet excites the response without blowing up; the simulation stops at the ground; RK4 error shrinks at 4th order; invalid throttle is rejected. |

### Other Files

| File | Purpose |
|---|---|
| `pyproject.toml` | Package metadata and dependencies. Makes `flightsim` installable (`pip install -e .`) and lets pytest import it from `src/`. |
| `requirements.txt` | Development environment dependencies with minimum versions. |
| `examples/elevator_doublet.py` | Runnable demo: trim, apply a ±3° elevator doublet, simulate 60 s and save plots to `results/`. |
| `docs/architecture.md` | This document. |
| `CHANGELOG.md` | A record of notable changes to the project. |

## Known Limitations

These come from the current modelling assumptions and are listed on the roadmap:

- The aerodynamics are linear: there is no stall and the model is only valid for small α and β.
- There is no wind or turbulence. Airspeed equals ground speed.
- Thrust is a constant-max-thrust placeholder with no propeller or engine dynamics.
- Control surfaces move instantly (no actuator dynamics).
- Flat, non-rotating Earth: valid for short flights at low speed.
- The example aircraft's parameters come from a textbook and haven't been checked against flight data.
