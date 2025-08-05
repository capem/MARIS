from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Mapping, Optional

from maris.control.pid import PIDConfig, RatePIDAutopilot
from maris.control.rate_provider import RateIntegratingControlProvider, WaypointProvider
from maris.core.validation import (
    validate_config,
    validate_initial_state,
    validate_params,
)
from maris.environment.providers import create_environment_provider_from_config
from maris.io.results import (
    CsvTickWriter,
    JsonlTickWriter,
    SummaryJsonWriter,
    TickWriter,
)
from maris.io.scenario import load as load_scenario
from maris.io.ship import load as load_ship
from maris.sim.runner import SimulationRunner
from maris.viz.plot2d import Plot2DOptions, plot_run_csv


def _mkdir(p: Path) -> None:
    p.mkdir(parents=True, exist_ok=True)


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def main(argv: Optional[list[str]] = None) -> None:
    ap = argparse.ArgumentParser(
        prog="maris", description="MARIS ship maneuvering simulation CLI"
    )
    ap.add_argument(
        "--ship", type=Path, required=True, help="Path to ship JSON definition"
    )
    ap.add_argument(
        "--scenario", type=Path, required=True, help="Path to scenario JSON"
    )
    ap.add_argument(
        "--out-dir", type=Path, required=True, help="Output directory for run artifacts"
    )
    ap.add_argument(
        "--plot2d",
        action="store_true",
        help="Generate 2D PNG plot of trajectory after run",
    )
    ap.add_argument(
        "--plot-csv",
        type=Path,
        default=None,
        help="Optional path to a per-tick CSV to plot (defaults to results.csv in out-dir)",
    )
    ap.add_argument(
        "--png",
        type=Path,
        default=None,
        help="Optional output PNG path (defaults to out-dir/plot2d.png)",
    )
    args = ap.parse_args(argv)

    out_dir: Path = args.out_dir
    _mkdir(out_dir)

    # Load inputs via IO layer
    vessel_params = load_ship(args.ship)
    cfg, state0, env_cfg = load_scenario(args.scenario)

    # Basic validation
    validate_params(vessel_params)
    validate_config(cfg)
    validate_initial_state(state0, vessel_params)

    # Create environment provider from scenario configuration
    envp = create_environment_provider_from_config(env_cfg)
    # Autopilot (rate output) and rate-integrating provider with fixed target
    ap = RatePIDAutopilot(
        heading=PIDConfig(kp=0.5, ki=0.0, kd=0.1, u_min=-0.02, u_max=0.02),  # rad/s
        speed=PIDConfig(kp=0.5, ki=0.0, kd=0.0, u_min=-5.0, u_max=5.0),  # RPM/s
    )
    waypoint = WaypointProvider(heading_rad=0.1, speed_ms=0.5)  # small nonzero target
    ctrlp = RateIntegratingControlProvider(
        vessel_params=vessel_params,
        initial_rpm=0.0,
        initial_rudder=0.0,
        autopilot=ap,
        target_provider=waypoint,
    )

    # Writers
    csv_writer = CsvTickWriter(
        out_dir / "results.csv",
        header=[
            "t",
            "x",
            "y",
            "psi",
            "u",
            "v",
            "r",
            "rpm",
            "rudder_angle",
            "wind_speed",
            "wind_dir_from",
            "current_speed",
            "current_dir_to",
            "X",
            "Y",
            "N",
        ],
    )
    jsonl_writer = JsonlTickWriter(out_dir / "results.jsonl")
    summary_writer = SummaryJsonWriter(out_dir / "summary.json")

    # Runner with SciPy-based integration
    runner = SimulationRunner()
    # Enforce MMG3DOF model; do not silently fall back so issues surface early
    from maris.models.mmg3dof.model import MMG3DOFModel

    model = MMG3DOFModel()

    try:
        runner.run(
            initial_state=state0,
            config=cfg,
            vessel_params=vessel_params,
            model=model,
            control_provider=ctrlp,
            env_provider=envp,
            writer=_MuxWriter(csv_writer, jsonl_writer),
            summary_writer=summary_writer,
            callbacks=None,
        )
    finally:
        csv_writer.close()
        jsonl_writer.close()

    # Optional 2D plotting
    if args.plot2d:
        try:
            csv_path = args.plot_csv if args.plot_csv else (out_dir / "results.csv")
            png_out = args.png if args.png else (out_dir / "plot2d.png")
            title = f"Trajectory — {args.ship.name} / {args.scenario.name}"
            plot_run_csv(
                csv_path=csv_path, png_out=png_out, options=Plot2DOptions(title=title)
            )
        except Exception as e:
            # Do not fail the run if plotting fails
            print(f"[warn] plot2d failed: {e}")


class _MuxWriter(TickWriter):
    def __init__(self, *writers: TickWriter) -> None:
        self._writers = writers

    def write_tick(self, record: Mapping[str, Any]) -> None:
        for w in self._writers:
            w.write_tick(record)


if __name__ == "__main__":
    main()
