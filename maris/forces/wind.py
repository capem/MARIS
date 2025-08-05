from __future__ import annotations

import math
from typing import Dict

from ..core.types import ControlInput, EnvironmentSample, VesselParams, VesselState


class WindForce:
    """
    Very simple placeholder aerodynamic force model to yield non-zero values.
    Assumptions:
      - Quadratic drag proportional to apparent wind speed squared.
      - Lift neglected; yaw moment proportional to lateral force lever arm ~ Lpp/2.
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

        # Apparent wind in North-East; convert to body-fixed approx. ignoring vessel motion for placeholder
        Vw = max(0.0, float(env.wind_speed))
        # Direction-from to direction-to: wind coming from angle -> flow to +pi offset
        theta_from = float(env.wind_dir_from)
        # Project wind along ship axes (body x approximated by global x when psi ~ 0; keep simple here)
        # For placeholder, resolve in body assuming psi ~ 0
        wx = Vw * math.cos(theta_from + math.pi)  # to-direction
        wy = Vw * math.sin(theta_from + math.pi)

        # Forces quadratic with coefficients
        X = -0.5 * rho_air * self.cx * A * wx * abs(wx)
        Y = -0.5 * rho_air * self.cy * A * wy * abs(wy)
        N = Y * (
            self.lever_coeff * params.Lpp
        )  # lateral force creates yaw moment around CG

        return {"X": X, "Y": Y, "N": N}
