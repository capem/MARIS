from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

import matplotlib.pyplot as plt
import numpy as np


@dataclass
class Plot2DOptions:
    figsize: tuple[float, float] = (8.0, 8.0)
    equal_axis: bool = True
    show_heading_ticks: bool = True
    heading_tick_stride: int = 50
    color_by_speed: bool = True
    quiver_stride: int = 50
    title: Optional[str] = None
    grid: bool = True
    dpi: int = 120
    background_layers: Optional[list[Path]] = None  # future: GeoJSON/CSV layers
    x_label: str = "x [m]"
    y_label: str = "y [m]"


def _load_csv_columns(path: Path, cols: list[str]) -> dict[str, np.ndarray]:
    with path.open("r", encoding="utf-8", newline="") as fh:
        reader = csv.DictReader(fh)
        data = {c: [] for c in cols}
        for row in reader:
            for c in cols:
                try:
                    data[c].append(float(row.get(c, "nan")))
                except Exception:
                    data[c].append(np.nan)
    return {k: np.asarray(v, dtype=float) for k, v in data.items()}


def plot_run_csv(csv_path: Path, png_out: Optional[Path] = None, options: Optional[Plot2DOptions] = None) -> Optional[plt.Figure]:
    """
    Plot a 2D trajectory from a per-tick CSV produced by the CLI.
    Expects at least columns: t,x,y,psi,u,v,r and optionally rpm,rudder_angle,wind_*,current_*,X,Y,N.
    """
    if options is None:
        options = Plot2DOptions()

    cols = [
        "t", "x", "y", "psi", "u", "v",
        "wind_speed", "wind_dir_from", "current_speed", "current_dir_to",
        "X", "Y", "N",
    ]
    # Always load r for completeness if present
    maybe_cols = ["r", "rpm", "rudder_angle"]
    all_cols = cols + maybe_cols

    data = _load_csv_columns(csv_path, all_cols)
    # Mandatory fields check
    for req in ("x", "y", "psi", "u", "v", "t"):
        if req not in data or data[req].size == 0:
            raise ValueError(f"Missing or empty column '{req}' in {csv_path}")

    x = data["x"]
    y = data["y"]
    psi = data["psi"]
    u = data["u"]
    v = data["v"]
    t = data["t"]

    speed = np.hypot(u, v)

    fig, ax = plt.subplots(figsize=options.figsize, dpi=options.dpi)
    if options.title:
        ax.set_title(options.title)

    if options.color_by_speed:
        sc = ax.scatter(x, y, c=speed, s=8, cmap="viridis", alpha=0.9, edgecolors="none")
        cbar = plt.colorbar(sc, ax=ax, pad=0.01, fraction=0.046)
        cbar.set_label("Speed [m/s]")
        ax.plot(x, y, color="black", linewidth=0.7, alpha=0.4)
    else:
        ax.plot(x, y, color="tab:blue", linewidth=1.2)

    if options.show_heading_ticks:
        stride = max(1, int(options.heading_tick_stride))
        idx = np.arange(0, x.size, stride)
        L = 0.05 * max(1.0, float(np.nanmax(np.hypot(x - np.nanmin(x), y - np.nanmin(y))) or 1.0))
        hx = np.cos(psi[idx]) * L
        hy = np.sin(psi[idx]) * L
        ax.quiver(x[idx], y[idx], hx, hy, angles="xy", scale_units="xy", scale=1.0, width=0.003, color="red", alpha=0.6)

    # Optional wind/current field arrows along track
    if "wind_speed" in data and "wind_dir_from" in data and np.any(np.isfinite(data["wind_speed"])):
        stride = max(1, int(options.quiver_stride))
        idxw = np.arange(0, x.size, stride)
        Vw = np.nan_to_num(data["wind_speed"][idxw], nan=0.0)
        theta_from = np.nan_to_num(data["wind_dir_from"][idxw], nan=0.0)
        wx = np.cos(theta_from + np.pi) * Vw
        wy = np.sin(theta_from + np.pi) * Vw
        ax.quiver(x[idxw], y[idxw], wx, wy, color="tab:orange", alpha=0.5, scale_units="xy", angles="xy", scale=1.0, width=0.002, label="wind")

    if "current_speed" in data and "current_dir_to" in data and np.any(np.isfinite(data["current_speed"])):
        stride = max(1, int(options.quiver_stride))
        idxc = np.arange(0, x.size, stride)
        Vc = np.nan_to_num(data["current_speed"][idxc], nan=0.0)
        theta_to = np.nan_to_num(data["current_dir_to"][idxc], nan=0.0)
        cxv = np.cos(theta_to) * Vc
        cyv = np.sin(theta_to) * Vc
        ax.quiver(x[idxc], y[idxc], cxv, cyv, color="tab:green", alpha=0.5, scale_units="xy", angles="xy", scale=1.0, width=0.002, label="current")

    if options.grid:
        ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)

    ax.set_xlabel(options.x_label)
    ax.set_ylabel(options.y_label)
    if options.equal_axis:
        ax.set_aspect("equal", adjustable="datalim")

    ax.legend(loc="best", fontsize=8, framealpha=0.6)

    if png_out is not None:
        png_out.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(png_out, bbox_inches="tight")
        plt.close(fig)
        return None

    return fig