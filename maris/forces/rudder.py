from __future__ import annotations

import math
from typing import Dict

from ..core.types import ControlInput, EnvironmentSample, VesselParams, VesselState


class RudderForce:
    """
    Simplified rudder lift/drag model producing lateral force and yaw moment.
    """

    def __init__(self) -> None:
        pass

    def compute(
        self,
        state: VesselState,
        control: ControlInput,
        env: EnvironmentSample,
        params: VesselParams,
    ) -> Dict[str, float]:
        rp = params.rudder_params or {}
        S = float(rp.get("area", max(1.0, 0.02 * params.Lpp * params.T)))
        a_lift = float(rp.get("a_lift", 6.28))  # ~2*pi per rad
        c_drag = float(rp.get("c_drag", 0.08))
        x_r = float(rp.get("x_rudder", -0.45 * params.Lpp))
        eff = float(
            rp.get("eff_factor", 1.2)
        )  # slightly >1 to amplify placeholder effect

        rho = (
            params.rho_water
            if math.isfinite(getattr(params, "rho_water", 1025.0))
            else 1025.0
        )

        # Relative inflow speed at rudder: use surge speed plus small sway-induced component magnitude
        V = max(0.0, (state.u**2 + 0.25 * state.v**2) ** 0.5)
        q = 0.5 * rho * V * V

        delta = control.rudder_angle
        Cl = a_lift * delta
        Cd = c_drag * (1.0 + abs(delta))  # increase drag with deflection

        Y = eff * q * S * Cl
        X = -eff * q * S * Cd  # drag opposes motion
        N = -Y * x_r

        return {"X": X, "Y": Y, "N": N}
