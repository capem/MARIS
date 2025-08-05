from __future__ import annotations

import math
from typing import Dict

from ..core.types import ControlInput, EnvironmentSample, VesselParams, VesselState


class BowThrusterForce:
    """
    Bow thruster force module for harbor maneuvering.
    Features:
    - Configurable thruster position along vessel length
    - Variable force magnitude and direction control
    - Realistic power limitations and efficiency effects
    - Integration with vessel dynamics for proper moment generation
    """

    def __init__(
        self,
        max_force: float = 150000.0,  # Maximum thruster force [N]
        position_x: float | None = None,  # Position from CG [m], defaults to 0.4*Lpp
        efficiency: float = 0.85,  # Thruster efficiency factor
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
        bow_params = tp.get("bow_thruster", {})
        
        max_force = float(bow_params.get("max_force", self.max_force))
        efficiency = float(bow_params.get("efficiency", self.efficiency))
        
        # Default position: forward of CG (typically 40% of Lpp forward)
        position_x = self.position_x
        if position_x is None:
            position_x = float(bow_params.get("position_x", 0.4 * params.Lpp))
        
        # Get thruster commands
        thruster_force = control.bow_thruster_force or 0.0
        thruster_angle = control.bow_thruster_angle or 0.0
        
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
        # Reduce efficiency in high current (thruster working against flow)
        current_speed = float(env.current_speed)
        if current_speed > 1.0:  # Significant current
            current_factor = max(0.7, 1.0 - 0.1 * (current_speed - 1.0))
            force_x *= current_factor
            force_y *= current_factor
        
        # Generate yaw moment from lateral force at bow position
        # Positive position_x means forward of CG
        moment_arm = position_x
        yaw_moment = -force_y * moment_arm  # Right-hand rule: +Y force at +X creates -N moment
        
        return {"X": force_x, "Y": force_y, "N": yaw_moment}
