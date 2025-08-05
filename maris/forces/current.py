from __future__ import annotations

import math
from typing import Dict

from ..core.types import VesselState, ControlInput, EnvironmentSample, VesselParams


class CurrentForce:
    """
    Placeholder force from ambient current.
    - Compute relative water velocity as vessel body velocity minus current resolved in body frame.
    - Linear drag proportional to relative velocity; yaw moment via lever arm ~ Lpp/2.
    """
    def __init__(self, kx: float = 5e4, ky: float = 1e5, lever_coeff: float = 0.5) -> None:
        self.kx = kx
        self.ky = ky
        self.lever_coeff = lever_coeff

    def compute(self, state: VesselState, control: ControlInput, env: EnvironmentSample, params: VesselParams) -> Dict[str, float]:
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

        # Drag opposes relative motion
        X = -self.kx * rel_u
        Y = -self.ky * rel_v
        N = Y * (self.lever_coeff * params.Lpp)

        return {"X": X, "Y": Y, "N": N}