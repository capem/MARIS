# Enhanced Force Modules

MARIS now features significantly enhanced force modules that provide realistic marine engineering physics beyond the original "placeholder" implementations.

## Overview

The enhanced force modules include:
- **Current Forces**: Quadratic drag, depth effects, configurable parameters
- **Hull Forces**: MMG-style cross-coupling, velocity-dependent coefficients
- **Propulsion Forces**: Efficiency curves, advance ratio effects, wake interaction
- **Rudder Forces**: Stall effects, propeller slipstream interaction
- **Wind Forces**: Apparent wind calculation, proper coordinate transformations

## Current Force Module

### Features
- **Linear + Quadratic Drag**: Realistic resistance modeling
- **Depth Effects**: Shallow water drag increase
- **Environmental Factors**: Current speed and direction effects
- **Configurable Parameters**: Ship-specific drag coefficients

### Configuration
```json
{
  "current_params": {
    "kx_linear": 4.0e4,
    "ky_linear": 8.0e4,
    "kx_quad": 2.0e3,
    "ky_quad": 4.0e3
  }
}
```

### Physics Model
```
X = depth_factor * (kx_linear * rel_u + kx_quad * rel_u * |rel_u|)
Y = depth_factor * (ky_linear * rel_v + ky_quad * rel_v * |rel_v|)
N = Y * lever_arm
```

## Hull Force Module

### Features
- **MMG-Style Cross-Coupling**: Realistic ship behavior
- **Primary Coefficients**: Xu, Xuu, Yv, Yvv, Nr, Nrr
- **Cross-Coupling Terms**: Yuv, Yur, Nuv, Nur, Xvr
- **Advanced Terms**: Xvv, Xrr, Yvr, Nvv

### Configuration
```json
{
  "mmg": {
    "hull_coeffs": {
      "Xu": -2.5e4, "Xuu": -1.0e4,
      "Yv": -8.0e5, "Yvv": -4.0e4,
      "Nr": -2.0e8, "Nrr": -1.0e7,
      "Yuv": -2.0e5, "Yur": 8.0e5,
      "Nuv": -4.0e7, "Nur": -1.5e8,
      "Xvr": -2.0e5, "Xvv": -8.0e3,
      "Xrr": -8.0e5, "Yvr": -4.0e5,
      "Nvv": -8.0e6
    }
  }
}
```

### Physics Model
```
X = Xu*u + Xuu*u*|u| + Xvr*v*r + Xvv*v*|v| + Xrr*r*|r|
Y = Yv*v + Yvv*v*|v| + Yuv*u*v + Yur*u*r + Yvr*v*r
N = Nr*r + Nrr*r*|r| + Nuv*u*v + Nur*u*r + Nvv*v*|v|
```

## Propulsion Force Module

### Features
- **Advance Ratio Effects**: Propeller efficiency curves
- **Wake Fraction**: Realistic inflow velocity
- **Thrust Deduction**: Hull-propeller interaction
- **Efficiency Modeling**: Parabolic efficiency curves

### Configuration
```json
{
  "propulsion": {
    "diameter": 5.5,
    "wake_fraction": 0.28,
    "thrust_deduction": 0.19,
    "efficiency_max": 0.68,
    "map": {
      "T0": 0.0, "T1": 0.0, "T2": 8.0e4,
      "steer_gain": 0.025, "x_prop": -75.0
    }
  }
}
```

### Physics Model
```
J = Va / (n * D)  // Advance ratio
efficiency = f(J, J_optimal)  // Efficiency curve
thrust_effective = thrust_base * efficiency * (1 - thrust_deduction)
```

## Rudder Force Module

### Features
- **Stall Effects**: Nonlinear lift curve with post-stall behavior
- **Propeller Slipstream**: Enhanced flow velocity at rudder
- **Induced Drag**: Aspect ratio dependent drag
- **Angle-Dependent Drag**: Realistic drag characteristics

### Configuration
```json
{
  "rudder": {
    "area": 45.0,
    "x_rudder": -70.0,
    "a_lift": 6.28,
    "c_drag": 0.085,
    "stall_angle": 0.35,
    "slipstream_factor": 1.45,
    "aspect_ratio": 2.1
  }
}
```

### Physics Model
```
// Linear region (δ ≤ stall_angle)
Cl = a_lift * δ

// Post-stall region (δ > stall_angle)
Cl = Cl_max * stall_factor * sign(δ)

// Enhanced drag
Cd = c_drag_0 + Cl²/(π*AR) + angle_dependent_drag
```

## Wind Force Module

### Features
- **Apparent Wind**: Vessel motion effects included
- **Coordinate Transformations**: Proper world-to-body frame conversion
- **Environmental Integration**: True wind to apparent wind calculation

### Configuration
```json
{
  "wind_params": {
    "cx": 0.85,
    "cy": 1.05,
    "lever_coeff": 0.48
  }
}
```

### Physics Model
```
// Apparent wind calculation
apparent_wind = true_wind - vessel_velocity  // in world frame

// Transform to body frame
wx = cos(-ψ) * aw_x - sin(-ψ) * aw_y
wy = sin(-ψ) * aw_x + cos(-ψ) * aw_y

// Forces
X = -cx * ρ * |w| * wx
Y = -cy * ρ * |w| * wy
N = Y * lever_arm
```

## Enhanced Simulation Features

### Force Component Tracking
The simulation runner now tracks individual force components:
```json
{
  "components": {
    "hull": {"X": -55000, "Y": 0, "N": 0},
    "propulsion": {"X": 3360, "Y": 3, "N": 168},
    "rudder": {"X": -765, "Y": 3219, "N": 144833},
    "wind": {"X": 1323, "Y": 0, "N": 0},
    "current": {"X": -25500, "Y": 0, "N": 0}
  }
}
```

### Performance Monitoring
```json
{
  "performance": {
    "total_simulation_time": 2.45,
    "solver_time": 1.82,
    "solver_calls": 750,
    "simulation_speed_ratio": 61.2
  }
}
```

### Enhanced Logging
- Force magnitude and direction
- Individual component contributions
- Motion analysis (speed, drift angle)
- Performance metrics

## Example Scenarios

### Cross-Coupling Demonstration
```bash
uv run maris --ship spec/examples/ship_enhanced_demo.json \
             --scenario spec/examples/scenario_cross_coupling.json \
             --out-dir runs/cross_coupling --plot2d
```

### Shallow Water Effects
```bash
uv run maris --ship spec/examples/ship_enhanced_demo.json \
             --scenario spec/examples/scenario_shallow_water.json \
             --out-dir runs/shallow_water --plot2d
```

### Rudder Stall Behavior
```bash
uv run maris --ship spec/examples/ship_enhanced_demo.json \
             --scenario spec/examples/scenario_rudder_stall.json \
             --out-dir runs/rudder_stall --plot2d
```

### Propeller Efficiency
```bash
uv run maris --ship spec/examples/ship_enhanced_demo.json \
             --scenario spec/examples/scenario_propeller_efficiency.json \
             --out-dir runs/propeller_efficiency --plot2d
```

## Migration from Basic Models

Existing scenarios will continue to work with enhanced physics automatically. To leverage new features:

1. **Add enhanced parameters** to ship configurations
2. **Use new scenario types** for specific demonstrations
3. **Enable force component tracking** in callbacks
4. **Monitor performance metrics** for optimization

The enhanced force modules provide sophisticated marine engineering capabilities while maintaining full backward compatibility with existing MARIS configurations.
