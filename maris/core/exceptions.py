class SchemaError(Exception):
    """Raised when JSON schema validation fails or schema version is unsupported."""
    pass


class ConfigError(Exception):
    """Raised when scenario/ship configuration is invalid or inconsistent."""
    pass


class RuntimeAbort(Exception):
    """Raised to abort a simulation due to external condition or termination policy."""
    def __init__(self, reason: str):
        super().__init__(reason)
        self.reason = reason


class NumericalInstability(Exception):
    """Raised when numerical instability is detected (NaN/Inf or divergence)."""
    pass