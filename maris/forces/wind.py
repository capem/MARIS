from __future__ import annotations

import math
from typing import Dict

from ..core.types import ControlInput, EnvironmentSample, VesselParams, VesselState


class WindForce:
    """
    Enhanced aerodynamic force model with proper coordinate transformations.
    Features:
      - Apparent wind calculation including vessel motion effects
      - Proper coordinate transformation from world to body frame
      - Quadratic drag proportional to apparent wind speed squared
      - Lift neglected; yaw moment proportional to lateral force lever arm ~ Lpp/2
    Tunables can later be moved into params.wind_params.
    """

    def __init__(
        self,
        cx: float = 0.8,
        cy: float = 1.0,
        area_ref: float | None = None,
        lever_coeff: float = 0.5,
    ) -> None:
        self.cx = cx
        self.cy = cy
        self.area_ref = area_ref
        self.lever_coeff = lever_coeff

    def compute(
        self,
        state: VesselState,
        control: ControlInput,
        env: EnvironmentSample,
        params: VesselParams,
    ) -> Dict[str, float]:
        rho_air = 1.225  # kg/m^3
        A = (
            self.area_ref if self.area_ref is not None else params.B * params.T
        )  # crude side area estimate

        # True wind vector in world frame (ENU)
        Vw = max(0.0, float(env.wind_speed))
        theta_from = float(env.wind_dir_from)
        # Convert from "direction wind is coming from" to "direction wind is going to"
        theta_to = theta_from + math.pi
        wind_world_x = Vw * math.cos(theta_to)  # east component
        wind_world_y = Vw * math.sin(theta_to)  # north component

        # Vessel velocity in world frame (ENU)
        c = math.cos(state.psi)
        s = math.sin(state.psi)
        vessel_world_x = state.u * c - state.v * s  # east velocity
        vessel_world_y = state.u * s + state.v * c  # north velocity

        # Apparent wind = true wind - vessel velocity (in world frame)
        apparent_wind_world_x = wind_world_x - vessel_world_x
        apparent_wind_world_y = wind_world_y - vessel_world_y

        # Transform apparent wind from world frame to body frame using -psi
        c_body = math.cos(-state.psi)
        s_body = math.sin(-state.psi)
        wx = (
            c_body * apparent_wind_world_x - s_body * apparent_wind_world_y
        )  # surge component
        wy = (
            s_body * apparent_wind_world_x + c_body * apparent_wind_world_y
        )  # sway component

        # Forces quadratic with coefficients
        X = -0.5 * rho_air * self.cx * A * wx * abs(wx)
        Y = -0.5 * rho_air * self.cy * A * wy * abs(wy)
        N = Y * (
            self.lever_coeff * params.Lpp
        )  # lateral force creates yaw moment around CG

        return {"X": X, "Y": Y, "N": N}
