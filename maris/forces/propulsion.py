from __future__ import annotations

from typing import Dict

from ..core.types import ControlInput, EnvironmentSample, VesselParams, VesselState


class PropulsionForce:
    """
    Enhanced propulsor model with realistic thrust characteristics.
    Features:
    - Advance ratio effects on propeller efficiency
    - Wake fraction and thrust deduction modeling
    - Propeller-rudder interaction effects
    - Configurable thrust mapping with efficiency curves
    Thrust map can be provided either as flat keys (T0,T1,T2,steer_gain,x_prop) or
    under a nested "map" key in ship JSON. Units: N for thrust, RPM for input.
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
        pp = params.prop_params or {}
        # Support nested map dict as written in example ship JSON
        mp = pp.get("map", {}) if isinstance(pp.get("map", {}), dict) else {}

        def _g(key: str, default: float) -> float:
            if key in pp:
                return float(pp.get(key))  # flat form
            return float(mp.get(key, default))  # nested form

        # Basic thrust coefficients
        T0 = _g("T0", 0.0)
        T1 = _g("T1", 0.0)
        T2 = _g("T2", 50.0)  # default small if not provided
        steer_gain = _g("steer_gain", 0.01)
        x_prop = _g("x_prop", -0.5 * params.Lpp)

        # Enhanced propeller parameters
        diameter = _g("diameter", 0.1 * params.Lpp)  # typical prop diameter
        wake_fraction = _g("wake_fraction", 0.3)  # typical wake fraction
        thrust_deduction = _g("thrust_deduction", 0.2)  # typical thrust deduction
        efficiency_max = _g("efficiency_max", 0.7)  # maximum propeller efficiency

        rpm = control.rpm

        # Calculate advance ratio for efficiency effects
        if abs(rpm) > 1e-6:  # avoid division by zero
            # Effective advance speed considering wake
            Va = state.u * (1.0 - wake_fraction)
            # Advance ratio J = Va / (n * D)
            n = abs(rpm) / 60.0  # convert RPM to rev/s
            J = Va / (n * diameter) if n > 1e-6 else 0.0

            # Efficiency curve (simplified parabolic model)
            # Peak efficiency at J ≈ 0.7, drops off at higher/lower J
            J_opt = 0.7
            if J <= J_opt:
                efficiency = efficiency_max * (J / J_opt) if J_opt > 0 else 0.0
            else:
                # Efficiency drops after optimal advance ratio
                efficiency = efficiency_max * max(0.0, 1.0 - 0.5 * (J - J_opt))
        else:
            efficiency = 0.0

        # Base thrust from RPM mapping
        thrust_base = T0 + T1 * rpm + T2 * rpm * abs(rpm)  # sign via rpm

        # Apply efficiency and thrust deduction
        thrust_effective = thrust_base * efficiency * (1.0 - thrust_deduction)

        # Forces and moments
        X = thrust_effective
        Y = (
            steer_gain * thrust_effective * control.rudder_angle
        )  # rudder-steering effect
        N = -Y * x_prop  # yaw moment from lateral force at prop location

        return {"X": X, "Y": Y, "N": N}
