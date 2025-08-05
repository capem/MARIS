from __future__ import annotations

from typing import List, Mapping, Optional

from ..core.types import ControlInput, VesselState


class ManualControlProvider:
    """
    Control provider that follows a predefined schedule of control commands.
    Supports traditional controls (RPM, rudder) and enhanced controls (thrusters, throttle).
    """

    def __init__(self, control_schedule: List[Mapping[str, float]]) -> None:
        """
        Initialize manual control provider with a schedule.
        
        Args:
            control_schedule: List of control commands with timing
                Each entry should have 't' (time) and control parameters like:
                - rpm, rudder_rad (traditional)
                - bow_thruster_force, bow_thruster_angle (bow thruster)
                - stern_thruster_force, stern_thruster_angle (stern thruster)
                - throttle (optional throttle control)
        """
        self.schedule = sorted(control_schedule, key=lambda x: x.get("t", 0.0))
        self._current_index = 0

    @property
    def target(self) -> None:
        """Manual control has no target."""
        return None

    def current(self) -> ControlInput:
        """Return current control input (used for initialization)."""
        if not self.schedule:
            return ControlInput(rpm=0.0, rudder_angle=0.0)
        
        first_cmd = self.schedule[0]
        return self._create_control_input(first_cmd)

    def compute(
        self, t: float, state: VesselState, target: Optional[object] = None
    ) -> ControlInput:
        """
        Compute control input based on current time and schedule.
        
        Args:
            t: Current simulation time [s]
            state: Current vessel state (unused for manual control)
            target: Target (unused for manual control)
            
        Returns:
            ControlInput with interpolated or held control values
        """
        if not self.schedule:
            return ControlInput(rpm=0.0, rudder_angle=0.0)

        # Find the appropriate control command for current time
        current_cmd = self._find_current_command(t)
        return self._create_control_input(current_cmd)

    def _find_current_command(self, t: float) -> Mapping[str, float]:
        """Find the current control command based on time."""
        if not self.schedule:
            return {"t": 0.0, "rpm": 0.0, "rudder_rad": 0.0}

        # If before first command, use first command
        if t <= self.schedule[0].get("t", 0.0):
            return self.schedule[0]

        # If after last command, use last command
        if t >= self.schedule[-1].get("t", 0.0):
            return self.schedule[-1]

        # Find the command to use (step function - hold until next command)
        for i in range(len(self.schedule) - 1):
            current_time = self.schedule[i].get("t", 0.0)
            next_time = self.schedule[i + 1].get("t", 0.0)
            
            if current_time <= t < next_time:
                return self.schedule[i]

        # Fallback to last command
        return self.schedule[-1]

    def _create_control_input(self, cmd: Mapping[str, float]) -> ControlInput:
        """Create ControlInput from command dictionary."""
        # Traditional controls
        rpm = float(cmd.get("rpm", 0.0))
        rudder_angle = float(cmd.get("rudder_rad", 0.0))
        throttle = cmd.get("throttle")
        
        # Thruster controls
        bow_thruster_force = cmd.get("bow_thruster_force")
        bow_thruster_angle = cmd.get("bow_thruster_angle")
        stern_thruster_force = cmd.get("stern_thruster_force")
        stern_thruster_angle = cmd.get("stern_thruster_angle")
        
        # Notes for special handling
        notes = cmd.get("notes")

        return ControlInput(
            rpm=rpm,
            rudder_angle=rudder_angle,
            throttle=throttle,
            bow_thruster_force=bow_thruster_force,
            bow_thruster_angle=bow_thruster_angle,
            stern_thruster_force=stern_thruster_force,
            stern_thruster_angle=stern_thruster_angle,
            notes=notes,
        )
