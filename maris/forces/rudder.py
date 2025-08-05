from __future__ import annotations

import math
from typing import Dict

from ..core.types import ControlInput, EnvironmentSample, VesselParams, VesselState


class RudderForce:
    """
    Enhanced rudder hydrodynamic model with realistic lift/drag characteristics.
    Features:
    - Nonlinear lift curve with stall effects
    - Propeller slipstream interaction
    - Angle-dependent drag characteristics
    - Configurable rudder geometry and coefficients
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
        c_drag_0 = float(rp.get("c_drag", 0.08))  # base drag coefficient
        x_r = float(rp.get("x_rudder", -0.45 * params.Lpp))

        # Enhanced rudder parameters
        stall_angle = float(rp.get("stall_angle", 0.35))  # ~20 degrees in radians
        slipstream_factor = float(rp.get("slipstream_factor", 1.5))  # propeller effect
        aspect_ratio = float(rp.get("aspect_ratio", 2.0))  # typical rudder AR

        rho = (
            params.rho_water
            if math.isfinite(getattr(params, "rho_water", 1025.0))
            else 1025.0
        )

        # Enhanced inflow velocity calculation with propeller effects
        # Base velocity from hull motion
        V_hull = max(0.0, (state.u**2 + 0.25 * state.v**2) ** 0.5)

        # Propeller slipstream effect (simplified)
        # Assume propeller increases flow velocity at rudder
        prop_params = params.prop_params or {}
        rpm = abs(control.rpm)
        if rpm > 1e-6:
            # Simplified slipstream velocity increase
            V_slip = (
                slipstream_factor * V_hull * min(1.0, rpm / 50.0)
            )  # saturate at high RPM
            V_effective = max(V_hull, V_slip)
        else:
            V_effective = V_hull

        q = 0.5 * rho * V_effective * V_effective

        delta = control.rudder_angle
        delta_abs = abs(delta)

        # Enhanced lift coefficient with stall effects
        if delta_abs <= stall_angle:
            # Linear region
            Cl = a_lift * delta
        else:
            # Post-stall region: reduced lift
            Cl_max = a_lift * stall_angle
            stall_factor = 1.0 - 0.5 * (delta_abs - stall_angle) / stall_angle
            stall_factor = max(0.3, stall_factor)  # maintain some lift post-stall
            Cl = math.copysign(Cl_max * stall_factor, delta)

        # Enhanced drag coefficient
        # Base drag + induced drag + angle-dependent drag
        induced_drag_factor = Cl * Cl / (math.pi * aspect_ratio)  # induced drag
        angle_drag = c_drag_0 * (delta_abs / stall_angle) ** 2  # angle-dependent drag
        Cd = c_drag_0 + induced_drag_factor + angle_drag

        # Forces and moments
        Y = q * S * Cl
        X = -q * S * Cd  # drag opposes motion
        N = -Y * x_r

        return {"X": X, "Y": Y, "N": N}
