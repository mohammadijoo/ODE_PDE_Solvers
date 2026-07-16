#!/usr/bin/env python3
# ------------------------------------------------------------
# 2D Heat Equation (Heat Transfer by Conduction) in Python
# ------------------------------------------------------------
# PDE:
#   ∂T/∂t = α ( ∂²T/∂x² + ∂²T/∂y² )
#
# Method:
#   - 2D explicit finite-difference scheme (FTCS)
#   - Dirichlet boundary conditions (fixed temperature on boundaries)
#   - Snapshots saved as PNG heatmaps using Matplotlib
#   - Additional plots saved (center point vs time, final centerline)
#
# Output folder (relative to where you run the program):
#   output/heat2d/
# ------------------------------------------------------------

from __future__ import annotations

import math
from pathlib import Path
from typing import List

import matplotlib
matplotlib.use("Agg")  # Use a non-GUI backend (works without Tcl/Tk)

import matplotlib.pyplot as plt


def idx(i: int, j: int, nx: int) -> int:
    """
    Flatten 2D indexing (i, j) into a 1D array index.

    Parameters
    ----------
    i : int
        x-index (column)
    j : int
        y-index (row)
    nx : int
        number of columns in the x-direction

    Returns
    -------
    int
        flattened index into a 1D array of length nx*ny
    """
    return j * nx + i


def write_csv_two_columns(
    filename: str,
    h1: str,
    h2: str,
    c1: List[float],
    c2: List[float],
) -> None:
    """
    Write a simple 2-column CSV file with a header row.

    Parameters
    ----------
    filename : str
        output CSV file path
    h1, h2 : str
        column header names
    c1, c2 : list[float]
        column data arrays (must have same length)
    """
    if len(c1) != len(c2):
        raise ValueError("CSV write error: column sizes do not match.")

    out_path = Path(filename)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    with out_path.open("w", encoding="utf-8") as f:
        # Header row
        f.write(f"{h1},{h2}\n")

        # Data rows
        for a, b in zip(c1, c2):
            f.write(f"{a},{b}\n")


def save_heatmap_png(
    out_path: Path,
    T_flat: List[float],
    nx: int,
    ny: int,
    title: str,
) -> None:
    """
    Save a heatmap figure (high quality) to PNG.

    Parameters
    ----------
    out_path : Path
        output PNG filename
    T_flat : list[float]
        temperature field stored as flattened 1D array (nx*ny)
    nx, ny : int
        grid size in x and y
    title : str
        figure title
    """
    # Convert flattened field into a 2D nested list for plotting
    T_2d = [[0.0 for _ in range(nx)] for _ in range(ny)]
    for j in range(ny):
        for i in range(nx):
            T_2d[j][i] = T_flat[idx(i, j, nx)]

    # Create a high-resolution figure suitable for publication-like output
    fig = plt.figure(figsize=(6.0, 5.0), dpi=300)
    ax = fig.add_subplot(1, 1, 1)

    # Plot with origin at lower-left so y increases upward
    im = ax.imshow(T_2d, origin="lower", aspect="auto")
    ax.set_title(title)
    ax.set_xlabel("x index")
    ax.set_ylabel("y index")

    # Keep visuals clean (no extra decorations unless desired)
    # If you want a colorbar, uncomment the next line:
    # fig.colorbar(im, ax=ax)

    fig.tight_layout()
    fig.savefig(out_path, dpi=300)
    plt.close(fig)


def main() -> int:
    # ------------------------------------------------------------
    # Output directory setup
    # ------------------------------------------------------------
    out_dir = Path("output/heat2d")
    out_dir.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------
    # Physical parameters
    # ------------------------------------------------------------
    alpha = 1.0  # thermal diffusivity (chosen for demonstrative speed)

    # ------------------------------------------------------------
    # Domain and grid
    # ------------------------------------------------------------
    Lx = 1.0
    Ly = 1.0

    # Grid resolution (increase for smoother but slower simulations)
    nx = 81
    ny = 81

    dx = Lx / (nx - 1)
    dy = Ly / (ny - 1)

    # ------------------------------------------------------------
    # Time step selection (explicit stability)
    # ------------------------------------------------------------
    # For the 2D explicit heat equation (FTCS), a standard stability constraint is:
    #
    #   dt <= 1 / ( 2*α*( 1/dx^2 + 1/dy^2 ) )
    #
    # We take a conservative fraction of this limit.
    dt_stable = 1.0 / (2.0 * alpha * (1.0 / (dx * dx) + 1.0 / (dy * dy)))
    dt = 0.80 * dt_stable  # conservative choice

    t_end = 0.20  # total simulation time
    nsteps = int(math.ceil(t_end / dt))

    # Save a heatmap snapshot every N steps (avoid generating too many PNGs)
    desired_snapshots = 30
    snapshot_every = max(1, nsteps // desired_snapshots)

    # ------------------------------------------------------------
    # Allocate temperature fields (flattened 2D arrays)
    # ------------------------------------------------------------
    T = [0.0] * (nx * ny)
    T_new = [0.0] * (nx * ny)

    # ------------------------------------------------------------
    # Boundary conditions (Dirichlet)
    # ------------------------------------------------------------
    # Left edge is hot; other edges cold.
    T_left = 100.0
    T_right = 0.0
    T_top = 0.0
    T_bottom = 0.0

    # Apply boundary values to the initial condition
    for j in range(ny):
        T[idx(0, j, nx)] = T_left
        T[idx(nx - 1, j, nx)] = T_right

    for i in range(nx):
        T[idx(i, 0, nx)] = T_bottom
        T[idx(i, ny - 1, nx)] = T_top

    # ------------------------------------------------------------
    # Logging (temperature at center over time)
    # ------------------------------------------------------------
    ic = nx // 2
    jc = ny // 2

    time_log: List[float] = []
    center_T_log: List[float] = []

    # Precompute reciprocal squared spacings for efficiency and clarity
    inv_dx2 = 1.0 / (dx * dx)
    inv_dy2 = 1.0 / (dy * dy)

    # ------------------------------------------------------------
    # Time integration loop
    # ------------------------------------------------------------
    t = 0.0
    for step in range(nsteps):
        # Log the center temperature before updating this step
        time_log.append(t)
        center_T_log.append(T[idx(ic, jc, nx)])

        # Update only the interior nodes (boundaries remain fixed)
        for j in range(1, ny - 1):
            for i in range(1, nx - 1):
                # Second derivative in x (central difference)
                Txx = (
                    T[idx(i + 1, j, nx)]
                    - 2.0 * T[idx(i, j, nx)]
                    + T[idx(i - 1, j, nx)]
                ) * inv_dx2

                # Second derivative in y (central difference)
                Tyy = (
                    T[idx(i, j + 1, nx)]
                    - 2.0 * T[idx(i, j, nx)]
                    + T[idx(i, j - 1, nx)]
                ) * inv_dy2

                # Explicit FTCS update
                T_new[idx(i, j, nx)] = T[idx(i, j, nx)] + alpha * dt * (Txx + Tyy)

        # Re-apply Dirichlet boundaries so they remain fixed exactly
        for j in range(ny):
            T_new[idx(0, j, nx)] = T_left
            T_new[idx(nx - 1, j, nx)] = T_right

        for i in range(nx):
            T_new[idx(i, 0, nx)] = T_bottom
            T_new[idx(i, ny - 1, nx)] = T_top

        # Swap buffers for the next iteration
        T, T_new = T_new, T

        # --------------------------------------------------------
        # Save a heatmap snapshot occasionally
        # --------------------------------------------------------
        if step % snapshot_every == 0:
            png_name = out_dir / f"heat_t{step}.png"
            save_heatmap_png(
                out_path=png_name,
                T_flat=T,
                nx=nx,
                ny=ny,
                title="2D Heat Equation - Temperature Field",
            )
            print(f"Saved snapshot: {png_name}")

        # Advance time
        t += dt

    # ------------------------------------------------------------
    # Final plots: centerline temperature and center point vs time
    # ------------------------------------------------------------

    # Create x-axis values (physical coordinates) and centerline values at y = middle row
    x_vals = [i * dx for i in range(nx)]
    centerline = [T[idx(i, jc, nx)] for i in range(nx)]

    # Plot final centerline temperature (high quality)
    fig = plt.figure(figsize=(6.5, 4.0), dpi=300)
    ax = fig.add_subplot(1, 1, 1)
    ax.plot(x_vals, centerline)
    ax.set_title("Final Centerline Temperature (y = 0.5)")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("T(x, y=0.5)")
    fig.tight_layout()
    fig.savefig(out_dir / "centerline_final.png", dpi=300)
    plt.close(fig)

    # Plot center point temperature vs time (high quality)
    fig = plt.figure(figsize=(6.5, 4.0), dpi=300)
    ax = fig.add_subplot(1, 1, 1)
    ax.plot(time_log, center_T_log)
    ax.set_title("Temperature at Plate Center vs Time")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("T(center)")
    fig.tight_layout()
    fig.savefig(out_dir / "center_point_vs_time.png", dpi=300)
    plt.close(fig)

    # ------------------------------------------------------------
    # Save CSV log of center temperature
    # ------------------------------------------------------------
    try:
        write_csv_two_columns(
            filename=str(out_dir / "heat2d_log.csv"),
            h1="t",
            h2="T_center",
            c1=time_log,
            c2=center_T_log,
        )
    except Exception as e:
        print(str(e))

    print(f"Heat2D finished. Results are in: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
