from __future__ import annotations

from pathlib import Path
from typing import Any, Optional


class SchemaError(Exception):
    """Raised when JSON schema validation fails or schema version is unsupported."""

    def __init__(
        self,
        message: str,
        schema_path: Optional[str | Path] = None,
        schema_name: Optional[str] = None,
        validation_error: Optional[Exception] = None,
    ):
        super().__init__(message)
        self.schema_path = str(schema_path) if schema_path else None
        self.schema_name = schema_name
        self.validation_error = validation_error

    def __str__(self) -> str:
        parts = [super().__str__()]
        if self.schema_name:
            parts.append(f"Schema: {self.schema_name}")
        if self.schema_path:
            parts.append(f"Path: {self.schema_path}")
        return " | ".join(parts)


class ConfigError(Exception):
    """Raised when scenario/ship configuration is invalid or inconsistent."""

    def __init__(
        self,
        message: str,
        config_path: Optional[str | Path] = None,
        field_name: Optional[str] = None,
        field_value: Optional[Any] = None,
    ):
        super().__init__(message)
        self.config_path = str(config_path) if config_path else None
        self.field_name = field_name
        self.field_value = field_value

    def __str__(self) -> str:
        parts = [super().__str__()]
        if self.field_name:
            parts.append(f"Field: {self.field_name}")
            if self.field_value is not None:
                parts.append(f"Value: {self.field_value}")
        if self.config_path:
            parts.append(f"Path: {self.config_path}")
        return " | ".join(parts)


class RuntimeAbort(Exception):
    """Raised to abort a simulation due to external condition or termination policy."""

    def __init__(self, reason: str):
        super().__init__(reason)
        self.reason = reason


class NumericalInstability(Exception):
    """Raised when numerical instability is detected (NaN/Inf or divergence)."""

    def __init__(
        self,
        message: str,
        component: Optional[str] = None,
        value: Optional[float] = None,
        bounds: Optional[tuple[float, float]] = None,
        simulation_time: Optional[float] = None,
    ):
        super().__init__(message)
        self.component = component
        self.value = value
        self.bounds = bounds
        self.simulation_time = simulation_time

    def __str__(self) -> str:
        parts = [super().__str__()]
        if self.component:
            parts.append(f"Component: {self.component}")
        if self.value is not None:
            parts.append(f"Value: {self.value}")
        if self.bounds:
            parts.append(f"Bounds: [{self.bounds[0]}, {self.bounds[1]}]")
        if self.simulation_time is not None:
            parts.append(f"Time: {self.simulation_time}s")
        return " | ".join(parts)
