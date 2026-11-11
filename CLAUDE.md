# CLAUDE.md

Guidelines for Claude when working in this repository.

## Project

`flightsim` is a nonlinear 6-DOF fixed-wing flight dynamics simulator in Python 3.13 (NumPy, SciPy, Matplotlib). The package lives in `src/flightsim/`, tests in `tests/`. Read `README.md` for modelling assumptions and conventions, and `docs/architecture.md` for data flow and the role of each file.

## Git and attribution

- **Do not add Claude as an author or co-author.** No `Co-Authored-By: Claude ...` trailers in commit messages and no "Generated with Claude Code" lines in PR descriptions, commits, code, or docs. Matthew Lac is the sole author.
- Commit or push only when asked. Never force-push or rewrite history without explicit confirmation.
- Commit messages: short imperative subject line (≤ 72 chars), blank line, then a body explaining what changed and why.

## Commands

```bash
pip install -r requirements.txt && pip install -e .   # setup (inside .venv)
pytest                                                # full test suite (~10 s)
pytest tests/test_dynamics.py -k energy               # single test
python examples/elevator_doublet.py                   # demo, writes to results/
```

Run `pytest` after any change to `src/flightsim/` and make sure it passes before calling the work done.

## Conventions (must be preserved)

- **SI units everywhere; angles in radians internally.** Degrees only at user-facing input/display boundaries.
- **Frames:** NED earth frame (altitude `h = -z_N`); body frame x forward, y right wing, z down.
- **Attitude:** unit quaternion, scalar first, NED → body. Euler angles (3-2-1: yaw, pitch, roll) only for input and display.
- **Control signs:** +elevator = nose-down moment, +aileron = right-wing-down, +rudder = nose-left, throttle in [0, 1].
- **State layout is defined only in `state.py`.** Use its named slices (`VELOCITY`, `QUATERNION`, ...) — never hard-code indices elsewhere.

## Design rules

- Keep mechanics separate from modelling: `rigid_body_derivative` takes total forces/moments and knows nothing about aircraft.
- Physics code contains no aircraft-specific numbers; airframes are `AircraftParameters` data.
- Force/moment models (aerodynamics, propulsion, future wind, etc.) return `(force, moment)` in body axes so they can be swapped without touching `dynamics.py`.
- Physical constants come from `constants.py`; don't redefine them.
- Prefer clear, documented NumPy code over cleverness. Docstrings should state units and frames for inputs and outputs.

## Testing and verification

- Every new module or model gets its own tests in `tests/`.
- Verify against something independent: analytical solutions, conservation laws, convergence rates, reference/textbook values, or sign/sanity checks. A test that only re-runs the implementation is not enough.
- Don't loosen a tolerance or delete a test to make it pass — find the cause and explain it.

## Docs upkeep

When behaviour, files, or features change, update the relevant docs in the same change:
- `CHANGELOG.md` (Keep a Changelog style, under an `Unreleased` or new version heading)
- `README.md` roadmap checkboxes, test count, and structure tree
- `docs/architecture.md` file reference and known limitations

## Don'ts

- Don't commit generated figures in `results/` (git-ignored), `.venv/`, or caches.
- Don't add new dependencies without asking; if approved, add them to both `pyproject.toml` and `requirements.txt`.
- Don't change public APIs (`simulate`, `Controls`, `make_state`, `trim_level_flight`, `SimulationResult`) without flagging it.
