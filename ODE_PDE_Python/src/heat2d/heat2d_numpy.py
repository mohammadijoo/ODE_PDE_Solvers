#!/usr/bin/env python3
# ------------------------------------------------------------
# 2D Heat Equation (Heat Transfer by Conduction) in Python (NumPy)
# ------------------------------------------------------------
# PDE:
#   ∂T/∂t = α ( ∂²T/∂x² + ∂²T/∂y² )
#
# Method:
#   - 2D explicit finite-difference scheme (FTCS)
#   - Vectorized interior update using NumPy slicing
#   - Dirichlet boundary conditions (fixed temperature on boundaries)
#   - Snapshots saved as PNG heatmaps (Matplotlib)
#   - Additional plots saved (center point vs time, final centerline)
#
# Output folder:
#   output/heat2d_numpy/
# ------------------------------------------------------------

from __future__ import annotations

import math
import csv
from pathlib import Path
from typing import List

import numpy as np

import matplotlib
matplotlib.use("Agg")  # Use a non-GUI backend (works without Tcl/Tk)

import matplotlib.pyplot as plt


def write_csv_two_columns(filename: Path, h1: str, h2: str, c1: List[float], c2: List[float]) -> None:
    """
    Write a simple 2-column CSV file with a header row.
    """
    if len(c1) != len(c2):
        raise ValueError("CSV write error: column sizes do not match.")

    filename.parent.mkdir(parents=True, exist_ok=True)
    with filename.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([h1, h2])
        for a, b in zip(c1, c2):
            w.writerow([a, b])


def save_heatmap_png(out_path: Path, T: np.ndarray, title: str) -> None:
    """
    Save a high-quality heatmap (dpi=300).
    T is expected to be a 2D array shaped (ny, nx).
    """
    fig = plt.figure(figsize=(6.2, 5.0), dpi=300)
    ax = fig.add_subplot(1, 1, 1)

    im = ax.imshow(T, origin="lower", aspect="auto")
    ax.set_title(title)
    ax.set_xlabel("x index")
    ax.set_ylabel("y index")

    # Optional colorbar for interpretability (comment out if you prefer a cleaner look)
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)

    fig.tight_layout()
    fig.savefig(out_path, dpi=300)
    plt.close(fig)


def main() -> int:
    # ------------------------------------------------------------
    # Output directory setup
    # ------------------------------------------------------------
    out_dir = Path("output/heat2d_numpy")
    out_dir.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------
    # Physical parameters
    # ------------------------------------------------------------
    alpha = 1.0  # thermal diffusivity (chosen for demonstrative speed)

    # ------------------------------------------------------------
    # Domain and grid
    # ------------------------------------------------------------
    Lx, Ly = 1.0, 1.0

    # Grid resolution (increase for smoother but slower simulations)
    nx, ny = 81, 81

    dx = Lx / (nx - 1)
    dy = Ly / (ny - 1)

    # ------------------------------------------------------------
    # Time step selection (explicit stability)
    # ------------------------------------------------------------
    dt_stable = 1.0 / (2.0 * alpha * (1.0 / (dx * dx) + 1.0 / (dy * dy)))
    dt = 0.80 * dt_stable

    t_end = 0.20
    nsteps = int(math.ceil(t_end / dt))

    desired_snapshots = 30
    snapshot_every = max(1, nsteps // desired_snapshots)

    # ------------------------------------------------------------
    # Allocate temperature field as a 2D NumPy array
    # ------------------------------------------------------------
    # T[j, i] corresponds to (y_index=j, x_index=i)
    T = np.zeros((ny, nx), dtype=np.float64)

    # ------------------------------------------------------------
    # Boundary conditions (Dirichlet)
    # ------------------------------------------------------------
    T_left = 100.0
    T_right = 0.0
    T_top = 0.0
    T_bottom = 0.0

    # Apply boundary values to the initial condition
    T[:, 0] = T_left
    T[:, -1] = T_right
    T[0, :] = T_bottom
    T[-1, :] = T_top

    # ------------------------------------------------------------
    # Logging (temperature at center over time)
    # ------------------------------------------------------------
    ic = nx // 2
    jc = ny // 2

    time_log: List[float] = []
    center_T_log: List[float] = []

    inv_dx2 = 1.0 / (dx * dx)
    inv_dy2 = 1.0 / (dy * dy)

    # ------------------------------------------------------------
    # Time integration loop (vectorized FTCS)
    # ------------------------------------------------------------
    t = 0.0
    for step in range(nsteps):
        # Log the center temperature at the current time
        time_log.append(t)
        center_T_log.append(float(T[jc, ic]))

        # Create a new array for the next time step
        T_new = T.copy()

        # Compute the discrete Laplacian on the interior using slicing
        Txx = (T[1:-1, 2:] - 2.0 * T[1:-1, 1:-1] + T[1:-1, :-2]) * inv_dx2
        Tyy = (T[2:, 1:-1] - 2.0 * T[1:-1, 1:-1] + T[:-2, 1:-1]) * inv_dy2

        # Explicit FTCS update on the interior
        T_new[1:-1, 1:-1] = T[1:-1, 1:-1] + alpha * dt * (Txx + Tyy)

        # Re-apply Dirichlet boundaries (kept fixed exactly)
        T_new[:, 0] = T_left
        T_new[:, -1] = T_right
        T_new[0, :] = T_bottom
        T_new[-1, :] = T_top

        # Advance solution and time
        T = T_new
        t += dt

        # Save snapshots occasionally
        if step % snapshot_every == 0:
            png_name = out_dir / f"heat_t{step}.png"
            save_heatmap_png(png_name, T, "2D Heat Equation - Temperature Field (NumPy)")
            print(f"Saved snapshot: {png_name}")

    # ------------------------------------------------------------
    # Final plots: centerline temperature and center point vs time
    # ------------------------------------------------------------
    x_vals = np.linspace(0.0, Lx, nx)
    centerline = T[jc, :].copy()

    # Final centerline plot (dpi=300)
    fig = plt.figure(figsize=(6.5, 4.0), dpi=300)
    ax = fig.add_subplot(1, 1, 1)
    ax.plot(x_vals, centerline)
    ax.set_title("Final Centerline Temperature (y = 0.5) (NumPy)")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("T(x, y=0.5)")
    fig.tight_layout()
    fig.savefig(out_dir / "centerline_final.png", dpi=300)
    plt.close(fig)

    # Center point vs time plot (dpi=300)
    fig = plt.figure(figsize=(6.5, 4.0), dpi=300)
    ax = fig.add_subplot(1, 1, 1)
    ax.plot(time_log, center_T_log)
    ax.set_title("Temperature at Plate Center vs Time (NumPy)")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("T(center)")
    fig.tight_layout()
    fig.savefig(out_dir / "center_point_vs_time.png", dpi=300)
    plt.close(fig)

    # ------------------------------------------------------------
    # Save CSV log
    # ------------------------------------------------------------
    write_csv_two_columns(out_dir / "heat2d_log.csv", "t", "T_center", time_log, center_T_log)

    print(f"Heat2D (NumPy) finished. Results are in: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
