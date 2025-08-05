from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Iterable, Mapping, Optional, Protocol

import numpy as np
from scipy.integrate import solve_ivp

from ..core.types import (
    VesselState,
    ControlInput,
    EnvironmentSample,
    Derivatives,
    SimulationConfig,
    VesselParams,
)
from ..core.validation import (
    validate_config,
    validate_params,
    validate_initial_state,
    check_bounds_control,
    detect_numerical_issue,
    apply_termination_bounds,
)


# Protocols for dependency inversion (interfaces only, no implementations here)
class VesselModel(Protocol):
    def forces(self, state: VesselState, control: ControlInput, env: EnvironmentSample, params: VesselParams) -> Mapping[str, Any]: ...
    def f(self, t: float, state: VesselState, control: ControlInput, env: EnvironmentSample, params: VesselParams) -> Derivatives: ...
    # Optional helpers for vector state mapping
    def unpack_state_vector(self, y: "np.ndarray | list[float]") -> VesselState: ...
    def pack_state_derivative(self, der: Derivatives) -> "np.ndarray | list[float]": ...
    def get_initial_state_vector(self, initial: VesselState) -> "np.ndarray | list[float]": ...


class ControlProvider(Protocol):
    def current(self) -> ControlInput: ...
    def compute(self, t: float, state: VesselState, target: Optional["Target"]) -> ControlInput: ...
    @property
    def target(self) -> Optional["Target"]: ...


class EnvironmentProvider(Protocol):
    def sample(self, t: float, state: Optional[VesselState] = None) -> EnvironmentSample: ...


class Target(Protocol):
    def desired(self, t: float, state: VesselState) -> "Target": ...


@dataclass
class Callbacks:
    on_step: Optional[Callable[[float, VesselState, ControlInput, EnvironmentSample, Derivatives, Mapping[str, Any]], None]] = None
    on_tick_logged: Optional[Callable[[int], None]] = None
    on_stream_send: Optional[Callable[[int], None]] = None


@dataclass
class RunResult:
    status: str
    reason: Optional[str]
    end_time: float
    ticks: int
    dt: float

    def summary(self) -> Mapping[str, Any]:
        return {
            "status": self.status,
            "reason": self.reason,
            "end_time": self.end_time,
            "ticks": self.ticks,
            "dt": self.dt,
        }

    def iter_records(self) -> Iterable[Mapping[str, Any]]:
        # Placeholder for streaming of written records via attached writer, if any.
        return ().__iter__()


def _make_segment_fun(model: VesselModel, control_k: ControlInput, env_k: EnvironmentSample, params: VesselParams):
    def fun(t: float, y: "np.ndarray") -> "np.ndarray":
        # Ensure y is a numpy array
        y_arr = np.asarray(y, dtype=float)
        state = model.unpack_state_vector(y_arr.tolist() if hasattr(y_arr, "tolist") else y_arr)
        der = model.f(t, state, control_k, env_k, params)
        # Ensure derivative is a numpy float array
        return np.asarray(model.pack_state_derivative(der), dtype=float)
    return fun


class SimulationRunner:
    """Fixed-step simulation loop now using SciPy solve_ivp per segment."""

    def run(
        self,
        initial_state: VesselState,
        config: SimulationConfig,
        vessel_params: VesselParams,
        model: VesselModel,
        control_provider: ControlProvider,
        env_provider: EnvironmentProvider,
        writer: Optional["TickWriter"] = None,
        summary_writer: Optional["SummaryWriter"] = None,
        callbacks: Optional[Callbacks] = None,
    ) -> RunResult:
        # Pre-run validation
        validate_config(config)
        validate_params(vessel_params)
        validate_initial_state(initial_state, vessel_params)

        t = config.t0
        k = 0
        # vector state for solver
        y = np.asarray([
            initial_state.x, initial_state.y, initial_state.psi,
            initial_state.u, initial_state.v, initial_state.r,
        ], dtype=float)
        term_reason: Optional[str] = None

        # Control integration state (for rate-based autopilot outputs)
        rpm_cmd = 0.0
        rudder_cmd = 0.0
        try:
            _c0 = control_provider.current()
            rpm_cmd = _c0.rpm
            rudder_cmd = _c0.rudder_angle
        except Exception:
            pass

        # Solver options (can be later moved into config)
        method = "RK45"
        rtol = 1e-6
        atol = 1e-9
        max_step = np.inf

        # Main loop
        min_advance = max(1e-9, 1e-6 * config.dt)
        while t < config.t_end + 1e-12:
            # Reconstruct VesselState for control/env sampling and logging
            state = model.unpack_state_vector(y)
            target = getattr(control_provider, "target", None)
            up = control_provider.compute(t, state, target)

            if getattr(up, "notes", None) == "rate":
                rpm_cmd = max(vessel_params.rpm_min, min(vessel_params.rpm_max, rpm_cmd + up.rpm * config.dt))
                rudder_cmd = max(vessel_params.rudder_min, min(vessel_params.rudder_max, rudder_cmd + up.rudder_angle * config.dt))
                control = ControlInput(rpm=rpm_cmd, rudder_angle=rudder_cmd, notes="abs")
            else:
                control = up
            check_bounds_control(control, vessel_params)
            env = env_provider.sample(t, state)

            # Force/model evaluation for callbacks/logging
            forces = model.forces(state, control, env, vessel_params)
            deriv = model.f(t, state, control, env, vessel_params)

            if callbacks and callbacks.on_step:
                callbacks.on_step(t, state, control, env, deriv, forces)

            # Build per-segment fun and integrate to t_next
            t_next = min(t + config.dt, config.t_end)
            fun = _make_segment_fun(model, control, env, vessel_params)

            # Handle zero-length segment
            if t_next <= t + 1e-15:
                y_out = np.asarray(y, dtype=float)
            else:
                # Integrate without t_eval; take final state for robustness
                sol = solve_ivp(
                    fun,
                    (t, t_next),
                    y,
                    method=method,
                    rtol=rtol,
                    atol=atol,
                    max_step=max_step,
                    vectorized=False,
                )

                if not sol.success or sol.y is None:
                    term_reason = f"solver_failed: {getattr(sol, 'message', 'no_message')}"
                    # If solver provided a last time, use it for stagnation detection below
                    last_t = sol.t[-1] if hasattr(sol, "t") and getattr(sol, "t", None) is not None and len(sol.t) else t
                    # If no progress, break; else advance t to last_t and break
                    if last_t - t < min_advance:
                        t = t  # no change
                    else:
                        t = last_t
                    break

                y_arr = np.asarray(sol.y, dtype=float)
                if y_arr.size == 0:
                    term_reason = "solver_no_output"
                    break
                y_out = y_arr[:, -1] if y_arr.ndim == 2 else y_arr
                # Also sanity-check time advancement from solver if available
                if hasattr(sol, "t") and getattr(sol, "t", None) is not None and len(sol.t):
                    if sol.t[-1] - t < min_advance and t_next - t >= min_advance:
                        term_reason = "solver_stagnation_internal"
                        break

            y = y_out
            # Basic runtime checks against previous state
            state_next = model.unpack_state_vector(y)
            detect_numerical_issue(state, state_next)

            # Logging (decimated)
            if writer and (k % config.output_decimation) == 0:
                writer.write_tick({
                    "t": t,
                    "x": state.x,
                    "y": state.y,
                    "psi": state.psi,
                    "u": state.u,
                    "v": state.v,
                    "r": state.r,
                    "rpm": control.rpm,
                    "rudder_angle": control.rudder_angle,
                    "wind_speed": env.wind_speed,
                    "wind_dir_from": env.wind_dir_from,
                    "current_speed": env.current_speed,
                    "current_dir_to": env.current_dir_to,
                    "X": float(forces.get("X", 0.0)) if isinstance(forces, dict) else 0.0,
                    "Y": float(forces.get("Y", 0.0)) if isinstance(forces, dict) else 0.0,
                    "N": float(forces.get("N", 0.0)) if isinstance(forces, dict) else 0.0,
                    "rpm_cmd": rpm_cmd,
                    "rudder_cmd": rudder_cmd,
                })
                if callbacks and callbacks.on_tick_logged:
                    callbacks.on_tick_logged(k)

            if callbacks and callbacks.on_stream_send and (k % config.stream_decimation) == 0:
                callbacks.on_stream_send(k)

            # Advance time, state and tick counter
            # Detect solver stagnation: ensure time progressed to (or very near) t_next
            if t_next - t < min_advance:
                term_reason = "solver_stagnation_no_time_advance"
                break
            t = t_next
            k += 1

            # Termination
            reason = apply_termination_bounds(state_next, config)
            if reason:
                term_reason = reason
                break

        status = "completed" if t >= config.t_end and term_reason is None else "terminated"
        summary = {
            "status": status,
            "reason": term_reason if status == "terminated" else "completed",
            "end_time": t,
            "ticks": k,
            "dt": config.dt,
        }
        if summary_writer:
            summary_writer.write_summary(summary)

        return RunResult(status=status, reason=term_reason, end_time=t, ticks=k, dt=config.dt)


# IO writer protocols (kept local to avoid circular deps)
class TickWriter(Protocol):
    def write_tick(self, record: Mapping[str, Any]) -> None: ...


class SummaryWriter(Protocol):
    def write_summary(self, summary: Mapping[str, Any]) -> None: ...