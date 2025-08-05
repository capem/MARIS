from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping

from jsonschema import Draft202012Validator

from ..core.exceptions import ConfigError, SchemaError
from ..core.types import SimulationConfig, VesselState


def _load_json(path: Path) -> Mapping[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception as e:
        raise ConfigError(f"Failed to read JSON {path}: {e}") from e


def _load_schema(schema_path: Path) -> Mapping[str, Any]:
    try:
        return json.loads(schema_path.read_text(encoding="utf-8"))
    except Exception as e:
        raise SchemaError(f"Failed to read schema {schema_path}: {e}") from e


def _validate(
    data: Mapping[str, Any], schema: Mapping[str, Any], schema_name: str
) -> None:
    try:
        Draft202012Validator(schema).validate(data)
    except Exception as e:
        raise SchemaError(f"{schema_name} validation failed: {e}") from e


def load(
    path: str | Path, schema_path: str | Path = "spec/schemas/scenario.schema.json"
) -> tuple[SimulationConfig, VesselState, Mapping[str, Any]]:
    """Load Scenario JSON, validate against schema, normalize to SI/radians and LOCAL_ENU for core."""
    p = Path(path)
    schema = _load_schema(Path(schema_path))
    data = _load_json(p)
    _validate(data, schema, "Scenario")

    sim = data["simulation"]
    init = data["initial_conditions"]["state"]
    outputs = data.get("outputs", {})
    per_tick = outputs.get("per_tick", {}) if isinstance(outputs, dict) else {}
    websocket = outputs.get("websocket", {}) if isinstance(outputs, dict) else {}

    cfg = SimulationConfig(
        t0=float(sim.get("t0", 0.0)),
        t_end=float(sim["t_end"]),
        dt=float(sim["t_step"]),
        seed=sim.get("seed"),
        integrator=str(sim.get("integrator", "rk4")),
        output_decimation=int(per_tick.get("decimation", 1) or 1),
        stream_decimation=int(websocket.get("decimation", 1) or 1),
        autopilot=(data.get("control", {}) or {}).get("autopilot"),
        env=data.get("environment"),
        termination_bounds=data.get("termination_bounds"),
        notes=data.get("notes"),
    )

    # Normalize initial state; incoming angles are in radians per schema (psi_rad)
    state = VesselState(
        t=float(sim.get("t0", 0.0)),
        x=float(init["x"]),
        y=float(init["y"]),
        psi=float(init["psi_rad"]),
        u=float(init.get("u", 0.0)),
        v=float(init.get("v", 0.0)),
        r=float(init.get("r", 0.0)),
    )

    env_cfg = data.get("environment", {}) or {}
    return cfg, state, env_cfg
