from __future__ import annotations

from typing import Protocol

from ..core.types import ControlInput, EnvironmentSample, VesselParams, VesselState


class ForceModule(Protocol):
    def compute(
        self,
        state: VesselState,
        control: ControlInput,
        env: EnvironmentSample,
        params: VesselParams,
    ) -> dict[str, float]:
        """
        Return a dict with keys "X","Y","N" (body-fixed forces and yaw moment).
        Units: N, N, N*m.
        """
        ...
