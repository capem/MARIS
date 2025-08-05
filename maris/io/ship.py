from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping

from jsonschema import Draft202012Validator

from ..core.exceptions import ConfigError, SchemaError
from ..core.types import VesselParams


def _load_json(path: Path) -> Mapping[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception as e:
        raise ConfigError(
            f"Failed to read JSON {path}: {e}",
            config_path=path,
        ) from e


def _load_schema(schema_path: Path) -> Mapping[str, Any]:
    try:
        return json.loads(schema_path.read_text(encoding="utf-8"))
    except Exception as e:
        raise SchemaError(
            f"Failed to read schema {schema_path}: {e}",
            schema_path=schema_path,
        ) from e


def _validate(
    data: Mapping[str, Any], schema: Mapping[str, Any], schema_name: str
) -> None:
    try:
        Draft202012Validator(schema).validate(data)
    except Exception as e:
        raise SchemaError(
            f"{schema_name} validation failed: {e}",
            schema_name=schema_name,
            validation_error=e,
        ) from e


def _angle_to_rad(val: float, units: str | None) -> float:
    if units == "deg":
        import math

        return val * math.pi / 180.0
    return val


def load(
    path: str | Path, schema_path: str | Path = "spec/schemas/ship.schema.json"
) -> VesselParams:
    """Load Ship Definition JSON, validate, and normalize to internal SI/radians."""
    path = Path(path)
    schema_path = Path(schema_path)

    data = _load_json(path)
    schema = _load_schema(schema_path)
    _validate(data, schema, "Ship")

    # Units policy
    angles_unit = (data.get("units_policy", {}) or {}).get("angles", "rad")

    # Geometry
    geom = data["geometry"]
    Lpp = float(geom["Lpp"])
    B = float(geom["B"])
    T = float(geom["T"])

    # Mass properties
    mp = data["mass_properties"]
    m = float(mp["displacement"])
    Iz = float(mp["Iz"])
    added_mass = mp.get("added_mass", {})
    X_u_dot = float(
        added_mass.get("X_u_dot", data["mmg"]["added_mass"].get("X_u_dot", 0.0))
    )
    Y_v_dot = float(
        added_mass.get("Y_v_dot", data["mmg"]["added_mass"].get("Y_v_dot", 0.0))
    )
    N_r_dot = float(
        added_mass.get("N_r_dot", data["mmg"]["added_mass"].get("N_r_dot", 0.0))
    )

    # Limits
    limits = data["limits"]
    rpm_min = float(limits["rpm"]["min"])
    rpm_max = float(limits["rpm"]["max"])
    rud = limits["rudder"]
    rudder_min = float(
        rud.get("min_rad", _angle_to_rad(rud.get("min", 0.0), angles_unit))
    )
    rudder_max = float(
        rud.get("max_rad", _angle_to_rad(rud.get("max", 0.0), angles_unit))
    )

    # Environment defaults
    rho_water = float((data.get("environment", {}) or {}).get("rho_water", 1025.0))

    # Params payloads
    mmg = data.get("mmg", {})
    hull_params = mmg.get("hull_coeffs", {})
    prop_params = data.get("propulsion", {})
    rudder_params = data.get("rudder", {})
    wind_params = data.get("wind_params")
    current_params = data.get("current_params")
    thruster_params = data.get("thruster_params")

    return VesselParams(
        m=m,
        Iz=Iz,
        X_u_dot=X_u_dot,
        Y_v_dot=Y_v_dot,
        N_r_dot=N_r_dot,
        Lpp=Lpp,
        B=B,
        T=T,
        rho_water=rho_water,
        rpm_min=rpm_min,
        rpm_max=rpm_max,
        rudder_min=rudder_min,
        rudder_max=rudder_max,
        hull_params=hull_params,
        prop_params=prop_params,
        rudder_params=rudder_params,
        wind_params=wind_params,
        current_params=current_params,
        thruster_params=thruster_params,
        metadata={"source": str(path), "schema_version": data.get("schema_version")},
    )
