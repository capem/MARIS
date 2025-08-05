from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from ..core.types import ControlInput, VesselParams, VesselState


@dataclass
class WaypointTarget:
    heading_rad: float
    speed_ms: float


class WaypointProvider:
    """Simple provider that returns a fixed heading and speed target."""

    def __init__(self, heading_rad: float, speed_ms: float) -> None:
        """
        Initialize waypoint provider with fixed target.

        Args:
            heading_rad: Target heading in radians
            speed_ms: Target speed in m/s
        """
        self._target = WaypointTarget(heading_rad=heading_rad, speed_ms=speed_ms)

    def desired(self, t: float, state: VesselState) -> WaypointTarget:
        """
        Return the fixed target.

        Args:
            t: Current simulation time [s] (unused)
            state: Current vessel state (unused)

        Returns:
            Fixed WaypointTarget
        """
        return self._target


class RateIntegratingControlProvider:
    """
    ControlProvider that consumes rate commands (rudder_rate [rad/s], rpm_rate [RPM/s])
    from an autopilot and integrates them into absolute commands with saturation applied
    by SimulationRunner. If no autopilot is attached, it returns the current absolute command.
    """

    def __init__(
        self,
        vessel_params: VesselParams,
        initial_rpm: float = 0.0,
        initial_rudder: float = 0.0,
        autopilot=None,
        target_provider: Optional[WaypointProvider] = None,
    ) -> None:
        self._rpm = max(vessel_params.rpm_min, min(vessel_params.rpm_max, initial_rpm))
        self._rudder = max(
            vessel_params.rudder_min, min(vessel_params.rudder_max, initial_rudder)
        )
        self._ap = autopilot
        self._tp = target_provider

    @property
    def target(self) -> Optional[WaypointTarget]:
        return (
            self._tp.desired(0.0, None) if self._tp else None
        )  # not used directly by runner

    def current(self) -> ControlInput:
        return ControlInput(rpm=self._rpm, rudder_angle=self._rudder, notes="abs")

    def compute(
        self, t: float, state: VesselState, target: Optional[WaypointTarget]
    ) -> ControlInput:
        if self._ap is None or self._tp is None:
            # No autopilot; return current absolute
            return ControlInput(rpm=self._rpm, rudder_angle=self._rudder, notes="abs")
        tgt = self._tp.desired(t, state)
        rudder_rate, rpm_rate = self._ap.update(t, state, tgt.heading_rad, tgt.speed_ms)
        # Runner will integrate when notes == "rate"
        return ControlInput(rpm=rpm_rate, rudder_angle=rudder_rate, notes="rate")
