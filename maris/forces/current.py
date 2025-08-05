from __future__ import annotations

import math
from typing import Dict

from ..core.types import ControlInput, EnvironmentSample, VesselParams, VesselState


class CurrentForce:
    """
    Enhanced hydrodynamic force model from ambient current.
    Features:
    - Proper relative water velocity calculation in body frame
    - Linear and quadratic drag components for realistic resistance
    - Depth-dependent effects for shallow water
    - Configurable drag coefficients and reference areas
    - Yaw moment from lateral force with proper lever arm
    """

    def __init__(
        self,
        kx_linear: float = 5e4,
        ky_linear: float = 1e5,
        kx_quad: float = 2e3,
        ky_quad: float = 5e3,
        lever_coeff: float = 0.5,
        depth_effect: bool = True,
        min_depth_ratio: float = 1.2,
    ) -> None:
        self.kx_linear = kx_linear
        self.ky_linear = ky_linear
        self.kx_quad = kx_quad
        self.ky_quad = ky_quad
        self.lever_coeff = lever_coeff
        self.depth_effect = depth_effect
        self.min_depth_ratio = min_depth_ratio

    def compute(
        self,
        state: VesselState,
        control: ControlInput,
        env: EnvironmentSample,
        params: VesselParams,
    ) -> Dict[str, float]:
        # Get current parameters from vessel params if available
        cp = params.current_params or {}
        kx_linear = float(cp.get("kx_linear", self.kx_linear))
        ky_linear = float(cp.get("ky_linear", self.ky_linear))
        kx_quad = float(cp.get("kx_quad", self.kx_quad))
        ky_quad = float(cp.get("ky_quad", self.ky_quad))

        # Current to-direction vector in world (ENU)
        Vc = max(0.0, float(env.current_speed))
        theta_to = float(env.current_dir_to)
        cw_x = Vc * math.cos(theta_to)
        cw_y = Vc * math.sin(theta_to)

        # Rotate world current into body frame using -psi
        c = math.cos(-state.psi)
        s = math.sin(-state.psi)
        c_body_x = c * cw_x - s * cw_y  # surge component of current in body frame
        c_body_y = s * cw_x + c * cw_y  # sway component of current in body frame

        # Relative water velocity = vessel velocity minus water (current) velocity in body frame
        rel_u = state.u - c_body_x
        rel_v = state.v - c_body_y

        # Depth effect factor for shallow water (increases drag)
        depth_factor = 1.0
        if self.depth_effect and hasattr(env, "depth") and env.depth is not None:
            depth = float(env.depth)
            if depth > 0:
                depth_ratio = depth / params.T
                if depth_ratio < self.min_depth_ratio:
                    # Shallow water effect: increase drag as depth decreases
                    depth_factor = 1.0 + (self.min_depth_ratio - depth_ratio) * 0.5

        # Enhanced drag model: linear + quadratic components
        # Linear drag (viscous effects)
        X_linear = -kx_linear * rel_u
        Y_linear = -ky_linear * rel_v

        # Quadratic drag (pressure/form drag)
        X_quad = -kx_quad * rel_u * abs(rel_u)
        Y_quad = -ky_quad * rel_v * abs(rel_v)

        # Total forces with depth effects
        X = depth_factor * (X_linear + X_quad)
        Y = depth_factor * (Y_linear + Y_quad)
        N = Y * (self.lever_coeff * params.Lpp)

        return {"X": X, "Y": Y, "N": N}
