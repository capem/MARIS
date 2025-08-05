from __future__ import annotations

from typing import Dict

from ..core.types import ControlInput, EnvironmentSample, VesselParams, VesselState


class HullForce:
    """
    Enhanced MMG-style hydrodynamic hull forces and moment.
    Features:
      - Linear and quadratic damping in all DOF
      - Cross-coupling terms for realistic ship behavior (Yuv, Nur, etc.)
      - Velocity-dependent coefficients for better accuracy
      - Support for both simplified and full MMG coefficient sets
    Expected hull_params keys (defaults used if absent):
      Primary: Xu, Xuu, Yv, Yvv, Nr, Nrr
      Cross-coupling: Yuv, Yur, Nuv, Nur, Xvr
      Advanced: Xvv, Xrr, Yvr, Nvv, Nrr
    Units:
      X, Y [N]; N [N*m]
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
        hp = params.hull_params or {}

        # Primary damping coefficients
        Xu = float(hp.get("Xu", -5e4))
        Xuu = float(hp.get("Xuu", -5e3))
        Yv = float(hp.get("Yv", -1e6))
        Yvv = float(hp.get("Yvv", -5e4))
        Nr = float(hp.get("Nr", -1e8))
        Nrr = float(hp.get("Nrr", -5e6))

        # Cross-coupling coefficients (MMG-style)
        Yuv = float(hp.get("Yuv", -2e5))  # sway force due to surge-sway coupling
        Yur = float(hp.get("Yur", 1e6))  # sway force due to surge-yaw coupling
        Nuv = float(hp.get("Nuv", -5e7))  # yaw moment due to surge-sway coupling
        Nur = float(hp.get("Nur", -2e8))  # yaw moment due to surge-yaw coupling
        Xvr = float(hp.get("Xvr", -2e5))  # surge force due to sway-yaw coupling

        # Additional quadratic cross-coupling terms
        Xvv = float(hp.get("Xvv", -1e4))  # surge force due to sway velocity squared
        Xrr = float(hp.get("Xrr", -1e6))  # surge force due to yaw rate squared
        Yvr = float(hp.get("Yvr", -5e5))  # sway force due to sway-yaw coupling
        Nvv = float(hp.get("Nvv", -1e7))  # yaw moment due to sway velocity squared

        u = state.u
        v = state.v
        r = state.r

        # Enhanced MMG-style force calculations with cross-coupling
        # Surge force (X)
        X = (
            Xu * u
            + Xuu * u * abs(u)  # primary surge damping
            + Xvr * v * r  # cross-coupling: sway-yaw
            + Xvv * v * abs(v)  # sway velocity effect
            + Xrr * r * abs(r)
        )  # yaw rate effect

        # Sway force (Y)
        Y = (
            Yv * v
            + Yvv * v * abs(v)  # primary sway damping
            + Yuv * u * v  # cross-coupling: surge-sway
            + Yur * u * r  # cross-coupling: surge-yaw
            + Yvr * v * r
        )  # sway-yaw coupling

        # Yaw moment (N)
        N = (
            Nr * r
            + Nrr * r * abs(r)  # primary yaw damping
            + Nuv * u * v  # cross-coupling: surge-sway
            + Nur * u * r  # cross-coupling: surge-yaw
            + Nvv * v * abs(v)
        )  # sway velocity effect

        return {"X": X, "Y": Y, "N": N}
