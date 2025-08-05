from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from ..core.types import VesselState, ControlInput, VesselParams


@dataclass
class PIDConfig:
    kp: float
    ki: float
    kd: float
    u_min: float
    u_max: float


@dataclass
class RatePIDAutopilot:
    """Outputs actuator RATES: rudder_rate [rad/s], rpm_rate [RPM/s]."""
    heading: PIDConfig
    speed: PIDConfig

    # Integrator states
    _e_int_heading: float = 0.0
    _e_int_speed: float = 0.0
    _last_heading_err: Optional[float] = None
    _last_speed_err: Optional[float] = None
    _last_t: Optional[float] = None

    def reset(self) -> None:
        self._e_int_heading = 0.0
        self._e_int_speed = 0.0
        self._last_heading_err = None
        self._last_speed_err = None
        self._last_t = None

    def _pid_step(self, t: float, err: float, cfg: PIDConfig, last_err: Optional[float], e_int: float, last_t: Optional[float]) -> tuple[float, float, float]:
        if last_t is None:
            dt = 0.0
            de = 0.0
        else:
            dt = max(1e-9, t - last_t)
            de = (err - (last_err or 0.0)) / dt
        e_int_new = e_int + err * dt
        u = cfg.kp * err + cfg.ki * e_int_new + cfg.kd * de
        # saturate
        u_sat = max(cfg.u_min, min(cfg.u_max, u))
        # simple anti-windup (clamp)
        if u != u_sat:
            e_int_new = e_int  # freeze integrator on saturation
        return u_sat, e_int_new, err

    def update(self, t: float, state: VesselState, target_heading_rad: float, target_speed_ms: float) -> tuple[float, float]:
        # heading error wrapped to [-pi, pi]
        import math
        dpsi = target_heading_rad - state.psi
        dpsi = (dpsi + math.pi) % (2.0 * math.pi) - math.pi
        sog = math.hypot(state.u, state.v)

        rudder_rate, self._e_int_heading, self._last_heading_err = self._pid_step(
            t, dpsi, self.heading, self._last_heading_err, self._e_int_heading, self._last_t
        )
        speed_err = target_speed_ms - sog
        rpm_rate, self._e_int_speed, self._last_speed_err = self._pid_step(
            t, speed_err, self.speed, self._last_speed_err, self._e_int_speed, self._last_t
        )
        self._last_t = t
        return rudder_rate, rpm_rate