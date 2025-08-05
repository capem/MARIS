from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional, Protocol
import math

from ...core.types import VesselState, ControlInput, EnvironmentSample, Derivatives, VesselParams
from ...core.validation import check_bounds_control
from ...forces.wind import WindForce
from ...forces.current import CurrentForce
from ...forces.hull import HullForce
from ...forces.propulsion import PropulsionForce
from ...forces.rudder import RudderForce


class ForceModule(Protocol):
    def compute(self, state: VesselState, control: ControlInput, env: EnvironmentSample, params: VesselParams) -> Mapping[str, float]: ...


@dataclass
class MMG3DOFModel:
    """MMG 3-DOF model skeleton.
    Responsibilities:
      - Aggregate forces from hull, propulsion, rudder, wind, current modules
      - Provide f(t, state, control, env, params) mapping forces to Derivatives
    Dynamics here remain placeholder but include basic kinematics to show motion.
    """
    hull: Optional[ForceModule] = None
    propulsion: Optional[ForceModule] = None
    rudder: Optional[ForceModule] = None
    wind: Optional[ForceModule] = None
    current: Optional[ForceModule] = None

    def __post_init__(self):
        # Provide default force models if none supplied
        if self.hull is None:
            self.hull = HullForce()
        if self.propulsion is None:
            self.propulsion = PropulsionForce()
        if self.rudder is None:
            self.rudder = RudderForce()
        if self.wind is None:
            self.wind = WindForce()
        if self.current is None:
            self.current = CurrentForce()

    def _sum_forces(self, state: VesselState, control: ControlInput, env: EnvironmentSample, params: VesselParams) -> Dict[str, Any]:
        components: Dict[str, Dict[str, float]] = {}
        total_X = total_Y = total_N = 0.0

        for name, mod in (
            ("hull", self.hull),
            ("propulsion", self.propulsion),
            ("rudder", self.rudder),
            ("wind", self.wind),
            ("current", self.current),
        ):
            if mod is None:
                continue
            out = dict(mod.compute(state, control, env, params))
            X = float(out.get("X", 0.0))
            Y = float(out.get("Y", 0.0))
            N = float(out.get("N", 0.0))
            components[name] = {"X": X, "Y": Y, "N": N}
            total_X += X
            total_Y += Y
            total_N += N

        return {"X": total_X, "Y": total_Y, "N": total_N, "components": components}

    # Public interface expected by SimulationRunner/Integrator
    def forces(self, state: VesselState, control: ControlInput, env: EnvironmentSample, params: VesselParams) -> Mapping[str, Any]:
        check_bounds_control(control, params)
        return self._sum_forces(state, control, env, params)

    def f(self, t: float, state: VesselState, control: ControlInput, env: EnvironmentSample, params: VesselParams) -> Derivatives:
        # Kinematics
        c = math.cos(state.psi)
        s = math.sin(state.psi)
        dx = state.u * c - state.v * s
        dy = state.u * s + state.v * c
        dpsi = state.r

        # Aggregate forces/moment
        F = self._sum_forces(state, control, env, params)
        X = float(F["X"])
        Y = float(F["Y"])
        N = float(F["N"])

        # Simplified 3-DOF dynamics using added-mass terms (placeholders):
        # (m - X_u_dot) * du - (m) * v * r = X
        # (m - Y_v_dot) * dv + (m) * u * r = Y
        # (Iz - N_r_dot) * dr = N
        m_eff_x = max(1e-6, params.m - params.X_u_dot)
        m_eff_y = max(1e-6, params.m - params.Y_v_dot)
        Iz_eff = max(1e-6, params.Iz - params.N_r_dot)

        coriolis_x = -params.m * state.v * state.r
        coriolis_y = params.m * state.u * state.r

        du = (X + coriolis_x) / m_eff_x
        dv = (Y + coriolis_y) / m_eff_y
        dr = N / Iz_eff

        return Derivatives(dx=dx, dy=dy, dpsi=dpsi, du=du, dv=dv, dr=dr)

    # --- Vector mapping helpers for SciPy integration ---
    def unpack_state_vector(self, y: "list[float] | tuple[float, ...]") -> VesselState:
        x, y_pos, psi, u, v, r = y
        # time t is not stored in vector; caller provides current t separately
        return VesselState(t=0.0, x=float(x), y=float(y_pos), psi=float(psi), u=float(u), v=float(v), r=float(r))

    def pack_state_derivative(self, der: Derivatives) -> "list[float]":
        return [float(der.dx), float(der.dy), float(der.dpsi), float(der.du), float(der.dv), float(der.dr)]

    def get_initial_state_vector(self, initial: VesselState) -> "list[float]":
        return [float(initial.x), float(initial.y), float(initial.psi), float(initial.u), float(initial.v), float(initial.r)]