from __future__ import annotations

import math
from typing import Optional

from .types import VesselState, ControlInput, SimulationConfig, VesselParams
from .exceptions import ConfigError, NumericalInstability


def validate_params(params: VesselParams) -> None:
    if not math.isfinite(params.m) or params.m <= 0:
        raise ConfigError("Invalid mass m")
    if not math.isfinite(params.Iz) or params.Iz <= 0:
        raise ConfigError("Invalid yaw inertia Iz")
    if params.rpm_min >= params.rpm_max:
        raise ConfigError("rpm_min must be < rpm_max")
    if params.rudder_min >= params.rudder_max:
        raise ConfigError("rudder_min must be < rudder_max")


def validate_config(cfg: SimulationConfig) -> None:
    if not math.isfinite(cfg.dt) or cfg.dt <= 0:
        raise ConfigError("dt must be > 0")
    if not math.isfinite(cfg.t_end) or cfg.t_end <= 0:
        raise ConfigError("t_end must be > 0")
    if not math.isfinite(cfg.t0):
        raise ConfigError("t0 must be finite")
    if cfg.output_decimation <= 0 or cfg.stream_decimation <= 0:
        raise ConfigError("decimation factors must be positive integers")


def validate_initial_state(state: VesselState, params: VesselParams) -> None:
    check_finite_state(state)
    # Additional sanity checks can be added (e.g., speed limits vs design), kept minimal for scaffolding.


def check_finite_state(state: VesselState) -> None:
    for name, val in (
        ("t", state.t),
        ("x", state.x),
        ("y", state.y),
        ("psi", state.psi),
        ("u", state.u),
        ("v", state.v),
        ("r", state.r),
    ):
        if not math.isfinite(val):
            raise NumericalInstability(f"Non-finite state component {name}={val}")


def check_bounds_control(control: ControlInput, params: VesselParams) -> None:
    if not math.isfinite(control.rpm):
        raise NumericalInstability("Control rpm not finite")
    if not math.isfinite(control.rudder_angle):
        raise NumericalInstability("Control rudder_angle not finite")
    if control.rpm < params.rpm_min or control.rpm > params.rpm_max:
        raise NumericalInstability(f"Control rpm out of bounds [{params.rpm_min},{params.rpm_max}]")
    if control.rudder_angle < params.rudder_min or control.rudder_angle > params.rudder_max:
        raise NumericalInstability("Control rudder_angle out of bounds")


def check_finite_derivatives(du: float, dv: float, dr: float) -> None:
    for name, val in (("du", du), ("dv", dv), ("dr", dr)):
        if not math.isfinite(val):
            raise NumericalInstability(f"Non-finite derivative {name}={val}")


def detect_numerical_issue(state_before: VesselState, state_after: VesselState) -> None:
    # Basic guard: NaN/Inf and absurd jumps (placeholder thresholds)
    check_finite_state(state_after)
    dx = abs(state_after.x - state_before.x)
    dy = abs(state_after.y - state_before.y)
    if dx > 1e6 or dy > 1e6:
        raise NumericalInstability("Unrealistic position jump detected")


def apply_termination_bounds(state: VesselState, cfg: SimulationConfig) -> Optional[str]:
    bounds = cfg.termination_bounds or {}
    if "x_min" in bounds and state.x < bounds["x_min"]:
        return "x below bound"
    if "x_max" in bounds and state.x > bounds["x_max"]:
        return "x above bound"
    if "y_min" in bounds and state.y < bounds["y_min"]:
        return "y below bound"
    if "y_max" in bounds and state.y > bounds["y_max"]:
        return "y above bound"
    return None