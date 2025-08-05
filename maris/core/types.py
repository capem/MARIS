from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Optional


@dataclass(frozen=True)
class VesselState:
    """Internal state in SI units and radians."""
    t: float
    x: float
    y: float
    psi: float  # heading [rad], CCW from +x
    u: float    # surge [m/s]
    v: float    # sway [m/s]
    r: float    # yaw rate [rad/s]


@dataclass(frozen=True)
class ControlInput:
    """Actuator commands in internal units."""
    rpm: float                 # commanded propeller RPM
    rudder_angle: float        # δ [rad], positive to port (CCW)
    throttle: Optional[float] = None  # 0..1 optional, if used
    notes: Optional[str] = None


@dataclass(frozen=True)
class EnvironmentSample:
    """Point sample used for force evaluations."""
    wind_speed: float          # [m/s]
    wind_dir_from: float       # [rad], direction wind is coming from
    current_speed: float       # [m/s]
    current_dir_to: float      # [rad], direction current flowing to
    sea_state: Optional[int] = None
    extras: Optional[Mapping[str, Any]] = None


@dataclass(frozen=True)
class ForceResult:
    """Total body-fixed forces and moment, with optional components per module."""
    X: float
    Y: float
    N: float
    components: Optional[Mapping[str, Mapping[str, float]]] = None  # e.g., {"hull": {"X":..,"Y":..,"N":..}, ...}


@dataclass(frozen=True)
class Derivatives:
    """Time derivatives of state; kinematics/dynamics combined."""
    dx: float
    dy: float
    dpsi: float
    du: float
    dv: float
    dr: float


@dataclass(frozen=True)
class VesselParams:
    """Subset needed for Phase 1; populated by Ship JSON loader with SI normalization."""
    m: float
    Iz: float
    X_u_dot: float
    Y_v_dot: float
    N_r_dot: float
    Lpp: float
    B: float
    T: float
    rho_water: float
    rpm_min: float
    rpm_max: float
    rudder_min: float      # [rad]
    rudder_max: float      # [rad]
    hull_params: Mapping[str, Any]
    prop_params: Mapping[str, Any]
    rudder_params: Mapping[str, Any]
    wind_params: Optional[Mapping[str, Any]] = None
    current_params: Optional[Mapping[str, Any]] = None
    metadata: Optional[Mapping[str, Any]] = None


@dataclass(frozen=True)
class SimulationConfig:
    """Simulation configuration derived from Scenario JSON."""
    t0: float
    t_end: float
    dt: float
    seed: Optional[int] = None
    integrator: str = "rk4"
    output_decimation: int = 1
    stream_decimation: int = 1
    autopilot: Optional[Mapping[str, Any]] = None
    env: Optional[Mapping[str, Any]] = None
    termination_bounds: Optional[Mapping[str, float]] = None
    notes: Optional[str] = None