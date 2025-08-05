# MARIS

**A comprehensive Python framework for ship maneuvering simulation and autopilot development with advanced marine engineering capabilities, thruster systems, and professional-grade simulation features.

## 🚀 Key Features

### Advanced Physics Modeling
- **MMG 3-DOF Model**: Industry-standard ship dynamics with proper force aggregation
- **Enhanced Force Modules**: Realistic physics with cross-coupling, efficiency curves, and stall effects
- **Environmental Integration**: Apparent wind, shallow water effects, current interactions
- **SciPy Integration**: Robust numerical integration with solve_ivp and RK45 method

### Harbor Maneuvering System
- **Bow/Stern Thrusters**: Precise positioning and docking capabilities
- **360° Control**: Variable force magnitude and direction
- **Realistic Operations**: Power curves, environmental effects, operational modes

### Professional Simulation Platform
- **Comprehensive Logging**: CSV/JSONL per-tick output and summary JSON
- **Force Component Tracking**: Individual module force analysis
- **Performance Monitoring**: Real-time simulation metrics
- **2D Visualization**: Matplotlib-based trajectory plotting with PNG output
- **CLI Interface**: Production-ready command-line tool with robust error handling

### Current Implementation Status
- ✅ Core types, validation, IO loaders, and CLI fully operational
- ✅ MMG 3-DOF model with enhanced force modules (hull, propulsion, rudder, wind, current)
- ✅ SciPy-based ODE integration with robust numerical stability
- ✅ PID autopilot with actuator rate commands and control integration
- ✅ Environment providers for static and time-varying conditions
- ✅ Enhanced physics with cross-coupling and efficiency modeling
- ✅ Thruster system for harbor maneuvering operations

## 📋 Requirements
- 🐍 Python 3.10+
- 💻 Windows/macOS/Linux
- 📦 Dependencies (declared in pyproject.toml):
  - numpy==2.1.3
  - scipy>=1.14.0
  - jsonschema<5
  - matplotlib==3.*

## ⚙️ SciPy-based Integration
- 🧮 ODE integration is handled by SciPy's `scipy.integrate.solve_ivp` with RK45 method by default.
- ⏱️ Integration is executed per fixed-step segment [t_k, t_{k+1}], holding control and environment constant across the segment.
- 🔄 The simulation runner handles state vector conversion and force aggregation for the solver.
- 🛡️ Robust error handling and termination conditions are implemented to detect numerical issues.
- 🆕 Legacy custom integrators (RK4 and semi-implicit Euler) have been completely replaced by SciPy's more robust methods.

## 🚀 Install (using uv)

1) Ensure uv is installed (https://github.com/astral-sh/uv):
   ```
   # Windows (PowerShell)
   iwr https://astral.sh/uv/install.ps1 -UseBasicParsing | iex

   # Linux/macOS (bash)
   curl -LsSf https://astral.sh/uv/install.sh | sh
   ```

2) Create/sync the project environment:
   ```
   # This will create .venv if missing and sync dependencies
   uv sync

   # Optionally activate the venv:
   .venv\Scripts\Activate.ps1    # Windows PowerShell
   source .venv/bin/activate     # Linux/macOS
   ```

Note: uv sync automatically performs an editable install of the local project

## 📁 Repository Layout
- 💻 cli/
  - maris_cli.py — Command line entry point (installed as "maris")
- 🔧 maris/
  - core/ — Core types, exceptions, validation utilities
  - io/ — Ship & scenario loaders with JSON Schema validation; results writers (CSV, JSONL, JSON)
  - control/ — PID autopilot implementation and rate-integrating control provider
  - environment/ — Environment providers for static and time-varying environmental conditions
  - forces/ — Force modules for wind, current, hull, propulsion, and rudder
  - models/mmg3dof/ — MMG 3-DOF model implementation with force aggregation
  - sim/ — SimulationRunner orchestrating the main loop with SciPy integration
  - viz/ — Visualization utilities including plot2d.py for trajectory visualization
- 📋 spec/
  - schemas/ — JSON Schemas for ship and scenario definitions
  - examples/ — Example ship (KVLCC2) and scenario (turning circle) files
- 📊 runs/ — Sample outputs directory for simulation results
  - example_turn/ — Example turning circle simulation results

## ⚡ Quick Start Examples

### 🛠️ Basic Setup
1) Ensure environment is synced with dependencies:
   ```
   uv sync
   ```

### 🌊 Standard Simulations

2) Run a basic simulation scenario (without activating the environment):
   ```
   uv run maris --ship spec/examples/ship_kvlcc2.json --scenario spec/examples/scenario_turning_circle.json --out-dir runs/example1
   ```

3) Run a simulation and generate a 2D plot automatically after completion:
   ```
   uv run maris --ship spec/examples/ship_kvlcc2.json --scenario spec/examples/scenario_turning_circle.json --out-dir runs/example1 --plot2d
   ```

### 🎆 Enhanced Simulation Demonstrations

4) Cross-coupling physics demonstration:
   ```
   uv run maris --ship spec/examples/ship_enhanced_demo.json \
                --scenario spec/examples/scenario_cross_coupling.json \
                --out-dir runs/demo --plot2d
   ```

5) Harbor entry with thrusters:
   ```
   uv run maris --ship spec/examples/ship_harbor_vessel.json \
                --scenario spec/examples/scenario_harbor_entry.json \
                --out-dir runs/harbor --plot2d
   ```

6) Shallow water effects:
   ```
   uv run maris --ship spec/examples/ship_enhanced_demo.json \
                --scenario spec/examples/scenario_shallow_water.json \
                --out-dir runs/shallow --plot2d
   ```

7) Rudder stall behavior:
   ```
   uv run maris --ship spec/examples/ship_enhanced_demo.json \
                --scenario spec/examples/scenario_rudder_stall.json \
                --out-dir runs/stall --plot2d
   ```

8) Propeller efficiency curves:
   ```
   uv run maris --ship spec/examples/ship_enhanced_demo.json \
                --scenario spec/examples/scenario_propeller_efficiency.json \
                --out-dir runs/efficiency --plot2d
   ```

9) Thruster capabilities demo:
   ```
   uv run maris --ship spec/examples/ship_harbor_vessel.json \
                --scenario spec/examples/scenario_thruster_demo.json \
                --out-dir runs/thrusters --plot2d
   ```

### CLI Options
```
--plot-csv runs/example1/results.csv       # Plot a specific CSV file
--png runs/example1/trajectory.png         # Specify custom PNG output path
```

## 📊 Outputs
- 📈 `runs/exampleX/results.csv`     — Per-tick CSV with fields: t,x,y,psi,u,v,r,rpm,rudder_angle,wind_speed,wind_dir_from,current_speed,current_dir_to,X,Y,N,rpm_cmd,rudder_cmd
- 📋 `runs/exampleX/results.jsonl`   — JSONL format with per-tick state, control, and force data
- 📄 `runs/exampleX/summary.json`    — End-of-run summary with status, reason, end time, ticks, and time step
- 🎨 `runs/exampleX/plot2d.png`      — Trajectory visualization (generated if --plot2d is passed or using plot utility separately)

## ⚙️ Configuration Examples

### 😢 Enhanced Ship Configuration
```json
{
  "identity": { "vessel_id": "enhanced_vessel" },
  "geometry": { "Lpp": 150.0, "B": 25.0, "T": 8.5 },
  
  "mmg": {
    "hull_coeffs": {
      "Xu": -2.5e4, "Xuu": -1.0e4, "Yv": -8.0e5, "Yvv": -4.0e4,
      "Yuv": -2.0e5, "Yur": 8.0e5, "Nuv": -4.0e7, "Nur": -1.5e8
    }
  },
  
  "propulsion": {
    "efficiency_max": 0.68, "wake_fraction": 0.28, "thrust_deduction": 0.19
  },
  
  "rudder": {
    "stall_angle": 0.35, "slipstream_factor": 1.45, "aspect_ratio": 2.1
  },
  
  "thruster_params": {
    "bow_thruster": { "max_force": 120000.0, "position_x": 48.0 },
    "stern_thruster": { "max_force": 80000.0, "position_x": -48.0 }
  }
}
```

### ⚓ Harbor Entry Control Schedule
```json
{
  "control": {
    "mode": "manual",
    "schedule": [
      { "t": 0.0, "rpm": 40.0, "rudder_rad": 0.0, "notes": "approach" },
      { "t": 120.0, "rpm": 15.0, "rudder_rad": 0.2,
        "bow_thruster_force": 50000.0, "bow_thruster_angle": 1.57,
        "notes": "tight_turn_with_bow_thruster" },
      { "t": 180.0, "rpm": 5.0, "rudder_rad": 0.0,
        "bow_thruster_force": 20000.0, "bow_thruster_angle": 0.0,
        "stern_thruster_force": -15000.0, "stern_thruster_angle": -1.57,
        "notes": "docking_with_thrusters" }
    ]
  }
}
```

## Analysis Capabilities

### Force Component Analysis
```python
# Monitor individual force components
def on_force_analysis(t, state, analysis):
    components = analysis.get("components", {})
    for name, comp in components.items():
        magnitude = comp.get("magnitude", 0.0)
        print(f"{name}: {magnitude:.0f}N")

callbacks = Callbacks(on_force_analysis=on_force_analysis)
```

### Performance Monitoring
```python
# Access simulation performance metrics
result = runner.run(...)
perf = result.performance_metrics
print(f"Speed: {perf['simulation_speed_ratio']:.1f}x realtime")
print(f"Solver efficiency: {perf['solver_time_fraction']:.2f}")
```

## Use Cases

### Marine Engineering Research
- **Hull Form Optimization**: Cross-coupling coefficient analysis
- **Propeller Design**: Efficiency curve validation
- **Rudder Performance**: Stall behavior characterization

### Harbor Operations
- **Pilot Training**: Realistic docking scenarios
- **Port Design**: Maneuvering space requirements
- **Tugboat Operations**: Assisted docking simulations

### Environmental Studies
- **Shallow Water Navigation**: Depth effect analysis
- **Weather Routing**: Wind/current impact assessment
- **Current Modeling**: Quadratic drag validation

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

## Development Guide

### Custom Force Modules
```python
class CustomForce:
    def compute(self, state, control, env, params):
        # Custom force calculations
        return {"X": force_x, "Y": force_y, "N": moment_n}

# Integrate with MMG model
model = MMG3DOFModel(custom_module=CustomForce())
```

### Advanced Control Systems
```python
class AdvancedController:
    def compute(self, t, state, target):
        # Advanced control logic
        return ControlInput(
            rpm=computed_rpm,
            rudder_angle=computed_rudder,
            bow_thruster_force=computed_bow_force,
            stern_thruster_force=computed_stern_force
        )
```

### Performance Considerations
- **Enhanced Physics**: ~10-15% computational overhead
- **Thruster System**: Minimal additional cost
- **Force Tracking**: ~5% overhead for component analysis
- **Overall**: Maintains real-time+ simulation speeds


## Roadmap (Near-term Tasks)
- ✅ Make MMG3DOF path the default and surface import/parameter errors instead of silent fallback
- ✅ Implement SciPy-based integration with robust error handling
- ✅ Implement proper force aggregation in the MMG3DOF model
- ✅ Replace dummy classes with proper environment providers supporting static and time-varying conditions
- ✅ Enhanced physics with cross-coupling and efficiency modeling
- ✅ Thruster system for harbor maneuvering operations
- Tune model coefficients and validate basic maneuvers (turning circle, zig-zag) with KVLCC2 benchmark data
- Add swept path calculation and Under-Keel Clearance (UKC) analysis
- Implement GeoJSON export and overlay capabilities in the 2D plotter
- Add Monte Carlo simulation runner for uncertainty analysis
- Implement metrics aggregation for statistical analysis of simulation results
- Improve documentation with detailed model descriptions and usage examples
- Add comprehensive test suite with pytest for IO validation and deterministic behavior


---

## Troubleshooting

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
