from __future__ import annotations

from typing import Dict

from ..core.types import VesselState, ControlInput, EnvironmentSample, VesselParams


class PropulsionForce:
    """
    Simplified propulsor model producing surge force (placeholder).
    Thrust map can be provided either as flat keys (T0,T1,T2,steer_gain,x_prop) or
    under a nested "map" key in ship JSON. Units: N for thrust, RPM for input.
    """
    def __init__(self) -> None:
        pass

    def compute(self, state: VesselState, control: ControlInput, env: EnvironmentSample, params: VesselParams) -> Dict[str, float]:
        pp = params.prop_params or {}
        # Support nested map dict as written in example ship JSON
        mp = pp.get("map", {}) if isinstance(pp.get("map", {}), dict) else {}
        def _g(key: str, default: float) -> float:
            if key in pp:
                return float(pp.get(key))  # flat form
            return float(mp.get(key, default))  # nested form

        T0 = _g("T0", 0.0)
        T1 = _g("T1", 0.0)
        T2 = _g("T2", 50.0)  # default small if not provided
        steer_gain = _g("steer_gain", 0.01)
        x_prop = _g("x_prop", -0.5 * params.Lpp)

        rpm = control.rpm
        thrust = T0 + T1 * rpm + T2 * rpm * abs(rpm)  # sign via rpm

        X = thrust
        Y = steer_gain * thrust * control.rudder_angle  # small lateral from rudder-steering effect
        N = -Y * x_prop  # yaw moment from lateral force at prop location

        return {"X": X, "Y": Y, "N": N}