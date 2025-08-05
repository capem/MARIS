from __future__ import annotations

from typing import Dict

from ..core.types import VesselState, ControlInput, EnvironmentSample, VesselParams


class HullForce:
    """
    Simplified hydrodynamic hull forces and moment (placeholder):
      - Linear + quadratic damping opposing motion in surge/sway/yaw.
      - Minimal cross-coupling: surge depends on u, sway on v, yaw on r.
    Expected hull_params keys (defaults used if absent):
      Xu, Xuu, Yv, Yvv, Nr, Nrr
    Units:
      X, Y [N]; N [N*m]
    """
    def __init__(self) -> None:
        pass

    def compute(self, state: VesselState, control: ControlInput, env: EnvironmentSample, params: VesselParams) -> Dict[str, float]:
        hp = params.hull_params or {}
        # Lighter surge damping to allow acceleration forward
        Xu = float(hp.get("Xu", -5e4))
        # Quadratic damping opposes |u|*u; keep magnitude moderate
        Xuu = float(hp.get("Xuu", -5e3))
        # Keep lateral and yaw damping stronger but finite
        Yv = float(hp.get("Yv", -1e6))
        Yvv = float(hp.get("Yvv", -5e4))
        Nr = float(hp.get("Nr", -1e8))
        Nrr = float(hp.get("Nrr", -5e6))

        u = state.u
        v = state.v
        r = state.r

        # Damping opposes motion: signs handled by velocity terms
        X = Xu * u + Xuu * u * abs(u)
        Y = Yv * v + Yvv * v * abs(v)
        N = Nr * r + Nrr * r * abs(r)

        return {"X": X, "Y": Y, "N": N}
