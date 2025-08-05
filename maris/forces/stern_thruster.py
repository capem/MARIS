from __future__ import annotations

import math
from typing import Dict

from ..core.types import ControlInput, EnvironmentSample, VesselParams, VesselState


class SternThrusterForce:
    """
    Stern thruster force module for harbor maneuvering.
    Features:
    - Configurable thruster position along vessel length
    - Variable force magnitude and direction control
    - Realistic power limitations and efficiency effects
    - Integration with vessel dynamics for proper moment generation
    """

    def __init__(
        self,
        max_force: float = 100000.0,  # Maximum thruster force [N] (typically smaller than bow)
        position_x: float | None = None,  # Position from CG [m], defaults to -0.4*Lpp
        efficiency: float = 0.80,  # Thruster efficiency factor (slightly lower than bow)
        power_curve_exp: float = 1.5,  # Power curve exponent for realistic response
    ) -> None:
        self.max_force = max_force
        self.position_x = position_x
        self.efficiency = efficiency
        self.power_curve_exp = power_curve_exp

    def compute(
        self,
        state: VesselState,
        control: ControlInput,
        env: EnvironmentSample,
        params: VesselParams,
    ) -> Dict[str, float]:
        # Get thruster parameters from vessel params if available
        tp = params.thruster_params or {} if hasattr(params, 'thruster_params') else {}
        stern_params = tp.get("stern_thruster", {})
        
        max_force = float(stern_params.get("max_force", self.max_force))
        efficiency = float(stern_params.get("efficiency", self.efficiency))
        
        # Default position: aft of CG (typically 40% of Lpp aft)
        position_x = self.position_x
        if position_x is None:
            position_x = float(stern_params.get("position_x", -0.4 * params.Lpp))
        
        # Get thruster commands
        thruster_force = control.stern_thruster_force or 0.0
        thruster_angle = control.stern_thruster_angle or 0.0
        
        # Limit thruster force to maximum capability
        force_magnitude = min(abs(thruster_force), max_force)
        force_sign = 1.0 if thruster_force >= 0 else -1.0
        
        # Apply efficiency and power curve effects
        # Power curve: higher forces require disproportionately more power
        force_ratio = force_magnitude / max_force if max_force > 0 else 0.0
        power_factor = force_ratio ** self.power_curve_exp
        effective_force = force_magnitude * efficiency * (1.0 - 0.2 * power_factor)
        
        # Calculate force components in body frame
        # Thruster angle: 0 = to starboard, π/2 = forward, -π/2 = aft
        force_x = effective_force * math.cos(thruster_angle) * force_sign
        force_y = effective_force * math.sin(thruster_angle) * force_sign
        
        # Environmental effects on thruster efficiency
        # Reduce efficiency in high current and consider propeller wash effects
        current_speed = float(env.current_speed)
        if current_speed > 1.0:  # Significant current
            current_factor = max(0.7, 1.0 - 0.1 * (current_speed - 1.0))
            force_x *= current_factor
            force_y *= current_factor
        
        # Propeller wash effect: stern thruster efficiency reduced when main engine running
        main_rpm = abs(control.rpm)
        if main_rpm > 10.0:  # Main engine running
            wash_factor = max(0.8, 1.0 - 0.002 * main_rpm)  # Reduce efficiency with RPM
            force_x *= wash_factor
            force_y *= wash_factor
        
        # Generate yaw moment from lateral force at stern position
        # Negative position_x means aft of CG
        moment_arm = position_x
        yaw_moment = -force_y * moment_arm  # Right-hand rule: +Y force at -X creates +N moment
        
        return {"X": force_x, "Y": force_y, "N": yaw_moment}
