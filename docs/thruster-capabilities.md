# Bow/Stern Thruster Capabilities

MARIS now supports bow and stern thrusters for realistic harbor maneuvering and precise vessel positioning operations.

## Overview

The thruster system provides:
- **Bow Thrusters**: Forward-mounted lateral thrusters for precise maneuvering
- **Stern Thrusters**: Aft-mounted thrusters for positioning control
- **360° Control**: Variable force magnitude and direction
- **Realistic Physics**: Power curves, efficiency effects, environmental impacts
- **Harbor Operations**: Complete docking and undocking capabilities

## Control System

### Enhanced ControlInput
```python
@dataclass(frozen=True)
class ControlInput:
    rpm: float                              # Main engine RPM
    rudder_angle: float                     # Rudder angle [rad]
    throttle: Optional[float] = None        # Throttle 0-1
    bow_thruster_force: Optional[float] = None    # [N], + = starboard
    bow_thruster_angle: Optional[float] = None    # [rad], 0 = starboard
    stern_thruster_force: Optional[float] = None  # [N], + = starboard  
    stern_thruster_angle: Optional[float] = None  # [rad], 0 = starboard
```

### Thruster Angles
- **0.0 rad (0°)**: To starboard (right)
- **π/2 rad (90°)**: Forward
- **π rad (180°)**: To port (left)
- **-π/2 rad (-90°)**: Aft (backward)

## Ship Configuration

### Thruster Parameters
```json
{
  "thruster_params": {
    "bow_thruster": {
      "max_force": 120000.0,      // Maximum force [N]
      "position_x": 48.0,         // Position from CG [m]
      "efficiency": 0.82,         // Efficiency factor
      "power_curve_exp": 1.4      // Power curve exponent
    },
    "stern_thruster": {
      "max_force": 80000.0,       // Typically smaller than bow
      "position_x": -48.0,        // Negative = aft of CG
      "efficiency": 0.78,         // Slightly lower efficiency
      "power_curve_exp": 1.5
    }
  }
}
```

### Default Positions
- **Bow Thruster**: 40% of Lpp forward of center of gravity
- **Stern Thruster**: 40% of Lpp aft of center of gravity

## Physics Model

### Force Calculation
```python
# Effective force with power curve
force_ratio = force_magnitude / max_force
power_factor = force_ratio ** power_curve_exp
effective_force = force_magnitude * efficiency * (1.0 - 0.2 * power_factor)

# Force components in body frame
force_x = effective_force * cos(thruster_angle) * force_sign
force_y = effective_force * sin(thruster_angle) * force_sign

# Yaw moment from position
yaw_moment = -force_y * position_x
```

### Environmental Effects

#### Current Impact
```python
if current_speed > 1.0:
    current_factor = max(0.7, 1.0 - 0.1 * (current_speed - 1.0))
    force_x *= current_factor
    force_y *= current_factor
```

#### Propeller Wash (Stern Thruster)
```python
if main_rpm > 10.0:
    wash_factor = max(0.8, 1.0 - 0.002 * main_rpm)
    force_x *= wash_factor
    force_y *= wash_factor
```

## Control Scheduling

### Manual Control Provider
```python
from maris.control.manual import ManualControlProvider

schedule = [
    {
        "t": 0.0,
        "rpm": 20.0,
        "rudder_rad": 0.0,
        "notes": "approach"
    },
    {
        "t": 120.0,
        "rpm": 15.0,
        "rudder_rad": 0.2,
        "bow_thruster_force": 50000.0,
        "bow_thruster_angle": 1.57,  # Forward
        "notes": "tight_turn_with_bow_thruster"
    },
    {
        "t": 180.0,
        "rpm": 5.0,
        "rudder_rad": 0.0,
        "bow_thruster_force": 20000.0,    # Starboard
        "bow_thruster_angle": 0.0,
        "stern_thruster_force": -15000.0, # Port
        "stern_thruster_angle": -1.57,    # Aft
        "notes": "docking_with_thrusters"
    }
]

provider = ManualControlProvider(schedule)
```

## Harbor Maneuvering Operations

### 1. Approach Phase
- **Traditional Control**: Main engine + rudder
- **Speed Reduction**: Gradual RPM decrease
- **Course Corrections**: Small rudder adjustments

### 2. Harbor Entry
- **Bow Thruster Activation**: Assist tight turns
- **Combined Control**: Engine + rudder + bow thruster
- **Precise Navigation**: Navigate narrow channels

### 3. Docking Phase
- **Lateral Positioning**: Bow thruster for sideways movement
- **Stern Control**: Stern thruster for aft positioning
- **Combined Thrusters**: Coordinated bow/stern operation

### 4. Final Positioning
- **Pure Thruster Control**: Engine stopped, thrusters only
- **Precise Movements**: Small force adjustments
- **Station Keeping**: Maintain position against wind/current

## Example Scenarios

### Harbor Entry Sequence
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
        "notes": "docking_with_thrusters" },
      { "t": 210.0, "rpm": 0.0, "rudder_rad": 0.0,
        "bow_thruster_force": 10000.0, "bow_thruster_angle": 1.57,
        "stern_thruster_force": -10000.0, "stern_thruster_angle": -1.57,
        "notes": "precise_positioning" }
    ]
  }
}
```

### Thruster Demonstration
```bash
# Run thruster capabilities demo
uv run maris --ship spec/examples/ship_harbor_vessel.json \
             --scenario spec/examples/scenario_thruster_demo.json \
             --out-dir runs/thruster_demo --plot2d

# Run harbor entry simulation
uv run maris --ship spec/examples/ship_harbor_vessel.json \
             --scenario spec/examples/scenario_harbor_entry.json \
             --out-dir runs/harbor_entry --plot2d
```

## Force Component Analysis

### Thruster Force Verification
```python
# Example test results
Bow thruster starboard: X=39495N, Y=0N, N=0N⋅m
Bow thruster forward: X=0N, Y=39495N, N=-1579792N⋅m
Stern thruster port: X=-29737N, Y=0N, N=0N⋅m
Stern thruster aft: X=0N, Y=-29737N, N=-1189490N⋅m
```

### Force Component Tracking
```json
{
  "components": {
    "hull": {"X": -55000, "Y": 0, "N": 0},
    "propulsion": {"X": 3360, "Y": 3, "N": 168},
    "rudder": {"X": -765, "Y": 3219, "N": 144833},
    "bow_thruster": {"X": 16834, "Y": 0, "N": 0},
    "stern_thruster": {"X": -11386, "Y": 0, "N": 0}
  }
}
```

## Typical Thruster Specifications

### Commercial Vessels
- **Bow Thruster**: 100-300 kN (22,000-67,000 lbf)
- **Stern Thruster**: 60-80% of bow thruster capacity
- **Position**: ±40-45% Lpp from center of gravity

### Harbor Tugs
- **Bow Thruster**: 200-500 kN (45,000-112,000 lbf)
- **Multiple Thrusters**: Often 2-3 thrusters per end
- **High Power**: Up to 2-3 MW per thruster

### Offshore Vessels
- **Dynamic Positioning**: 4-8 thrusters total
- **Azimuth Thrusters**: 360° rotatable pods
- **High Precision**: ±1m position accuracy

## Integration with Existing Systems

### Backward Compatibility
- Existing scenarios work without modification
- Thruster parameters default to None (no thruster forces)
- Traditional control (RPM + rudder) remains primary

### Enhanced Capabilities
- Add thruster_params to ship configuration
- Include thruster commands in control schedules
- Monitor thruster forces in simulation output

## Advanced Features

### Power Curve Modeling
Thrusters exhibit realistic power consumption characteristics:
- **Linear Region**: Efficient operation at low forces
- **Quadratic Region**: Increasing power demand at high forces
- **Maximum Limits**: Force saturation at rated capacity

### Environmental Compensation
- **Current Effects**: Reduced efficiency in strong currents
- **Propeller Interaction**: Stern thruster affected by main engine wash
- **Wind Loading**: Thruster forces work against environmental loads

### Operational Modes
1. **Transit Mode**: Thrusters off, traditional control
2. **Maneuvering Mode**: Combined engine/rudder/thruster control
3. **Docking Mode**: Primary thruster control, minimal engine use
4. **Station Keeping**: Pure thruster positioning control

The thruster system transforms MARIS into a comprehensive harbor maneuvering simulator suitable for pilot training, port design, and maritime operations research.
