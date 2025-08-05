from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Mapping, Optional, Union

from ..core.types import VesselState, EnvironmentSample


@dataclass
class WindConfig:
    """Wind configuration for environment providers."""
    speed: float = 0.0                    # [m/s]
    dir_from_rad: float = 0.0            # [rad], direction wind is coming from
    gust_factor: float = 1.0             # multiplier for gusts
    gust_period: Optional[float] = None  # [s], period for sinusoidal gusts


@dataclass
class CurrentConfig:
    """Current configuration for environment providers."""
    speed: float = 0.0                   # [m/s]
    dir_to_rad: float = 0.0             # [rad], direction current flowing to
    tidal_amplitude: float = 0.0         # [m/s], amplitude of tidal variation
    tidal_period: Optional[float] = None # [s], period for tidal variation


class StaticEnvironmentProvider:
    """
    Environment provider that returns constant environmental conditions.
    Suitable for steady-state simulations and testing.
    """
    
    def __init__(
        self,
        wind_speed: float = 0.0,
        wind_dir_from: float = 0.0,
        current_speed: float = 0.0,
        current_dir_to: float = 0.0,
        sea_state: Optional[int] = None,
        extras: Optional[Mapping[str, Any]] = None,
    ) -> None:
        """
        Initialize static environment provider.
        
        Args:
            wind_speed: Wind speed [m/s]
            wind_dir_from: Direction wind is coming from [rad]
            current_speed: Current speed [m/s]
            current_dir_to: Direction current is flowing to [rad]
            sea_state: Sea state (0-9 scale), optional
            extras: Additional environment parameters
        """
        self._sample = EnvironmentSample(
            wind_speed=wind_speed,
            wind_dir_from=wind_dir_from,
            current_speed=current_speed,
            current_dir_to=current_dir_to,
            sea_state=sea_state,
            extras=extras,
        )
    
    def sample(self, t: float, state: Optional[VesselState] = None) -> EnvironmentSample:
        """Return the constant environment sample."""
        return self._sample


class TimeVaryingEnvironmentProvider:
    """
    Environment provider that supports time-varying environmental conditions.
    Supports sinusoidal variations, gusts, tidal effects, and custom functions.
    """
    
    def __init__(
        self,
        wind: Optional[WindConfig] = None,
        current: Optional[CurrentConfig] = None,
        sea_state: Optional[int] = None,
        wind_function: Optional[Callable[[float], tuple[float, float]]] = None,
        current_function: Optional[Callable[[float], tuple[float, float]]] = None,
        extras: Optional[Mapping[str, Any]] = None,
    ) -> None:
        """
        Initialize time-varying environment provider.
        
        Args:
            wind: Wind configuration with optional time variations
            current: Current configuration with optional time variations
            sea_state: Sea state (0-9 scale), optional
            wind_function: Custom function returning (speed, dir_from_rad) for given time
            current_function: Custom function returning (speed, dir_to_rad) for given time
            extras: Additional environment parameters
        """
        self._wind = wind or WindConfig()
        self._current = current or CurrentConfig()
        self._sea_state = sea_state
        self._wind_function = wind_function
        self._current_function = current_function
        self._extras = extras
    
    def sample(self, t: float, state: Optional[VesselState] = None) -> EnvironmentSample:
        """
        Sample environment at given time.
        
        Args:
            t: Simulation time [s]
            state: Current vessel state (unused in base implementation)
            
        Returns:
            Environment sample with time-varying conditions
        """
        # Calculate wind conditions
        if self._wind_function:
            wind_speed, wind_dir_from = self._wind_function(t)
        else:
            wind_speed = self._wind.speed
            wind_dir_from = self._wind.dir_from_rad
            
            # Apply gust effects if configured
            if self._wind.gust_period and self._wind.gust_period > 0:
                gust_factor = 1.0 + (self._wind.gust_factor - 1.0) * math.sin(
                    2 * math.pi * t / self._wind.gust_period
                )
                wind_speed *= gust_factor
        
        # Calculate current conditions
        if self._current_function:
            current_speed, current_dir_to = self._current_function(t)
        else:
            current_speed = self._current.speed
            current_dir_to = self._current.dir_to_rad
            
            # Apply tidal effects if configured
            if self._current.tidal_period and self._current.tidal_period > 0:
                tidal_variation = self._current.tidal_amplitude * math.sin(
                    2 * math.pi * t / self._current.tidal_period
                )
                current_speed += tidal_variation
                current_speed = max(0.0, current_speed)  # Ensure non-negative
        
        return EnvironmentSample(
            wind_speed=wind_speed,
            wind_dir_from=wind_dir_from,
            current_speed=current_speed,
            current_dir_to=current_dir_to,
            sea_state=self._sea_state,
            extras=self._extras,
        )


def create_environment_provider_from_config(
    env_cfg: Mapping[str, Any]
) -> Union[StaticEnvironmentProvider, TimeVaryingEnvironmentProvider]:
    """
    Factory function to create environment provider from scenario configuration.
    
    Args:
        env_cfg: Environment configuration from scenario JSON
        
    Returns:
        Appropriate environment provider based on configuration
    """
    wind_cfg = env_cfg.get("wind", {})
    current_cfg = env_cfg.get("current", {})
    
    wind_speed = float(wind_cfg.get("speed", 0.0))
    wind_dir_from = float(wind_cfg.get("dir_from_rad", 0.0))
    current_speed = float(current_cfg.get("speed", 0.0))
    current_dir_to = float(current_cfg.get("dir_to_rad", 0.0))
    
    # Check if time-varying features are requested
    has_time_varying = (
        wind_cfg.get("gust_period") is not None or
        wind_cfg.get("gust_factor", 1.0) != 1.0 or
        current_cfg.get("tidal_period") is not None or
        current_cfg.get("tidal_amplitude", 0.0) != 0.0
    )
    
    if has_time_varying:
        wind = WindConfig(
            speed=wind_speed,
            dir_from_rad=wind_dir_from,
            gust_factor=float(wind_cfg.get("gust_factor", 1.0)),
            gust_period=wind_cfg.get("gust_period"),
        )
        current = CurrentConfig(
            speed=current_speed,
            dir_to_rad=current_dir_to,
            tidal_amplitude=float(current_cfg.get("tidal_amplitude", 0.0)),
            tidal_period=current_cfg.get("tidal_period"),
        )
        return TimeVaryingEnvironmentProvider(
            wind=wind,
            current=current,
            sea_state=env_cfg.get("sea_state"),
            extras=env_cfg.get("extras"),
        )
    else:
        return StaticEnvironmentProvider(
            wind_speed=wind_speed,
            wind_dir_from=wind_dir_from,
            current_speed=current_speed,
            current_dir_to=current_dir_to,
            sea_state=env_cfg.get("sea_state"),
            extras=env_cfg.get("extras"),
        )
