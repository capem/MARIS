# MARIS

A Python framework for ship maneuvering simulation and autopilot development with a runnable simulation CLI, data schemas, MMG 3-DOF model implementation, force modules, SciPy-based integration, PID autopilot, and visualization utilities.

Current status:
- Core types, validation, IO loaders, and CLI are fully implemented and operational.
- MMG 3-DOF model with force modules (hull, propulsion, rudder, wind, current) is the default model with proper force aggregation.
- SciPy-based ODE integration (solve_ivp) provides robust numerical integration.
- PID-based autopilot outputs actuator rate commands that are integrated to absolute controls in the runner.
- Environment providers support both static and time-varying environmental conditions (wind, current, sea state).
- Comprehensive logging with CSV/JSONL per-tick output and summary JSON.
- 2D visualization (matplotlib) generates PNG trajectory plots from simulation results.
- The CLI enforces the MMG3DOF model and proper environment providers with no dummy/placeholder implementations.

Requirements
- Python 3.10+
- Windows/macOS/Linux
- Dependencies (declared in pyproject.toml):
  - numpy==2.1.3
  - scipy>=1.14.0
  - jsonschema<5
  - matplotlib==3.*

SciPy-based integration
- ODE integration is handled by SciPy's `scipy.integrate.solve_ivp` with RK45 method by default.
- Integration is executed per fixed-step segment [t_k, t_{k+1}], holding control and environment constant across the segment.
- The simulation runner handles state vector conversion and force aggregation for the solver.
- Robust error handling and termination conditions are implemented to detect numerical issues.
- Legacy custom integrators (RK4 and semi-implicit Euler) have been completely replaced by SciPy's more robust methods.

Install (using uv)
1) Ensure uv is installed (https://github.com/astral-sh/uv):
   # Windows (PowerShell): iwr https://astral.sh/uv/install.ps1 -UseBasicParsing | iex
   # Linux/macOS (bash):   curl -LsSf https://astral.sh/uv/install.sh | sh

2) Create/sync the project environment from pyproject + uv.lock:
   # This will create .venv if missing and sync dependencies exactly
   uv sync

   # Optionally activate the venv for your shell:
   .venv\Scripts\Activate.ps1    # Windows PowerShell
   # or: source .venv/bin/activate  # Linux/macOS

   # If you prefer editable install semantics, uv sync already installs the local project.

Repository layout
- cli/
  - maris_cli.py — Command line entry point (installed as "maris")
- maris/
  - core/ — Core types, exceptions, validation utilities
  - io/ — Ship & scenario loaders with JSON Schema validation; results writers (CSV, JSONL, JSON)
  - control/ — PID autopilot implementation and rate-integrating control provider
  - environment/ — Environment providers for static and time-varying environmental conditions
  - forces/ — Force modules for wind, current, hull, propulsion, and rudder
  - models/mmg3dof/ — MMG 3-DOF model implementation with force aggregation
  - sim/ — SimulationRunner orchestrating the main loop with SciPy integration
  - viz/ — Visualization utilities including plot2d.py for trajectory visualization
- spec/
  - schemas/ — JSON Schemas for ship and scenario definitions
  - examples/ — Example ship (KVLCC2) and scenario (turning circle) files
- runs/ — Sample outputs directory for simulation results
  - example_turn/ — Example turning circle simulation results

Quick start: run a scenario and generate a plot
1) Ensure environment is synced with dependencies:
   ```
   uv sync
   ```

2) Run a simulation scenario (without activating the environment):
   ```
   uv run maris --ship spec/examples/ship_kvlcc2.json --scenario spec/examples/scenario_turning_circle.json --out-dir runs/example1
   ```

3) Run a simulation and generate a 2D plot automatically after completion:
   ```
   uv run maris --ship spec/examples/ship_kvlcc2.json --scenario spec/examples/scenario_turning_circle.json --out-dir runs/example1 --plot2d
   ```

4) Optional flags:
   ```
   --plot-csv runs/example1/results.csv       # Plot a specific CSV file
   --png runs/example1/trajectory.png         # Specify custom PNG output path
   ```

Outputs
- `runs/exampleX/results.csv`     — Per-tick CSV with fields: t,x,y,psi,u,v,r,rpm,rudder_angle,wind_speed,wind_dir_from,current_speed,current_dir_to,X,Y,N,rpm_cmd,rudder_cmd
- `runs/exampleX/results.jsonl`   — JSONL format with per-tick state, control, and force data
- `runs/exampleX/summary.json`    — End-of-run summary with status, reason, end time, ticks, and time step
- `runs/exampleX/plot2d.png`      — Trajectory visualization (generated if --plot2d is passed or using plot utility separately)

Smoke test
- A simple smoke test verifies the simulation pipeline and output generation:
  ```
  uv run python -m cli.smoke_run_example
  ```
- The test runs the example turning circle scenario with the KVLCC2 ship model
- Outputs are written to `runs/example_turn_smoke/`
- The test script asserts the presence of basic artifacts and validates summary fields
- This provides a quick verification that the core simulation functionality is working correctly

Behavior notes
- The CLI enforces the use of the MMG3DOFModel and will surface any import or parameter errors.
- The MMG3DOF model properly aggregates forces from all modules (hull, propulsion, rudder, wind, current).
- The model includes simplified 3-DOF dynamics with added-mass terms and coriolis forces.
- Environment providers automatically select between static and time-varying implementations based on scenario configuration.
- Time-varying environments support sinusoidal wind gusts and tidal current variations.
- SciPy's solve_ivp integration provides robust numerical stability for the simulation.
- The example ship parameters are based on KVLCC2 but may still need further tuning against benchmark data.

Conventions
- Internal units: SI system
  - Length: meters (m)
  - Time: seconds (s)
  - Mass: kilograms (kg)
  - Force: newtons (N)
  - Angles: radians (rad)
- Coordinate systems:
  - Internal frame: LOCAL_ENU (x-east, y-north, z-up)
  - Ship-fixed frame: x-forward, y-starboard, z-down
  - External IO may include CRS (Coordinate Reference System) metadata in JSON, per spec/schemas/*
- State variables:
  - Position: (x, y) in LOCAL_ENU frame
  - Heading: psi (ψ) - angle between north and ship's forward direction
  - Velocities: (u, v, r) - surge, sway, yaw rate in ship-fixed frame

Roadmap (near-term tasks)
- ✅ Make MMG3DOF path the default and surface import/parameter errors instead of silent fallback
- ✅ Implement SciPy-based integration with robust error handling
- ✅ Implement proper force aggregation in the MMG3DOF model
- ✅ Replace dummy classes with proper environment providers supporting static and time-varying conditions
- Tune model coefficients and validate basic maneuvers (turning circle, zig-zag) with KVLCC2 benchmark data
- Add swept path calculation and Under-Keel Clearance (UKC) analysis
- Implement GeoJSON export and overlay capabilities in the 2D plotter
- Add Monte Carlo simulation runner for uncertainty analysis
- Implement metrics aggregation for statistical analysis of simulation results
- Improve documentation with detailed model descriptions and usage examples
- Add comprehensive test suite with pytest for IO validation and deterministic behavior

Troubleshooting

### Simulation Issues
- If you encounter issues with the simulation:
  1) Ensure environment is properly synced with all dependencies: 
     ```
     uv sync
     ```
  2) Verify the package is correctly installed:
     ```
     uv pip show maris
     ```
  3) Run in verbose mode to see detailed error messages:
     ```
     uv run python -c "from cli.maris_cli import main; main()" --ship spec/examples/ship_kvlcc2.json --scenario spec/examples/scenario_turning_circle.json --out-dir runs/example1 --plot2d
     ```
  4) Check the simulation output files (especially summary.json) for termination reasons or errors

### Visualization Issues
- For Matplotlib backend issues in headless environments:
  - The CLI saves directly to PNG when --plot2d is used; no display is required
  - If you encounter font or rendering issues, try setting the Matplotlib backend explicitly:
    ```
    export MPLBACKEND=Agg  # Linux/macOS
    $env:MPLBACKEND="Agg"  # Windows PowerShell
    ```

### Model Implementation
- The MMG3DOF model is enforced by default in the CLI with no dummy fallbacks
- Environment providers are automatically selected based on scenario configuration
- If you need to debug model behavior, examine the force components in the JSONL output
- For numerical instabilities, check the solver parameters in `maris/sim/runner.py`
