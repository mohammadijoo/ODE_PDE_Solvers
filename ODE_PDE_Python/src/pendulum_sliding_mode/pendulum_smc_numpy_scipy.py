#!/usr/bin/env python3
# ------------------------------------------------------------
# Cart–Pole Inverted Pendulum with Sliding Mode Control (NumPy + optional SciPy)
# ------------------------------------------------------------
# Requirements:
#   - Total simulation time: 10 seconds
#   - Total frames: 6000  (600 FPS video)
#   - Exactly two disturbances: at t=0.5s and t=5.0s
#   - Disturbance arrow shown for 0.5s only (direction indicator, not stretched)
#   - Dynamics are integrated and used to render frames
#   - Outputs: frames, mp4 (ffmpeg), plots (dpi=300), CSV log
#
# Output folders:
#   output/pendulum_sliding_mode_numpy_scipy/frames/frame_000000.png ... frame_005999.png
#   output/pendulum_sliding_mode_numpy_scipy/pendulum_smc_10s_6000f.mp4 (if ffmpeg is installed)
#   output/pendulum_sliding_mode_numpy_scipy/*.png plots
#   output/pendulum_sliding_mode_numpy_scipy/cartpole_log.csv
# ------------------------------------------------------------

from __future__ import annotations

import argparse
import csv
import math
import os
import shutil
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Tuple

import numpy as np

# Force a non-GUI Matplotlib backend to avoid Tk/Tcl dependency issues on some systems.
import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt


# ------------------------------------------------------------
# Small math helpers
# ------------------------------------------------------------
def clamp(x: float, lo: float, hi: float) -> float:
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def sat(z: float) -> float:
    return clamp(z, -1.0, 1.0)


# ------------------------------------------------------------
# CSV writer
# ------------------------------------------------------------
def write_csv(filename: Path, header: List[str], cols: List[np.ndarray]) -> None:
    if not cols:
        raise ValueError("CSV: no columns provided.")
    n = int(cols[0].shape[0])
    for c in cols:
        if int(c.shape[0]) != n:
            raise ValueError("CSV: column size mismatch.")

    filename.parent.mkdir(parents=True, exist_ok=True)
    with filename.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        for i in range(n):
            w.writerow([float(c[i]) for c in cols])


# ------------------------------------------------------------
# Pole model: uniform rod + lumped bob at the top
# ------------------------------------------------------------
@dataclass
class PoleModel:
    L: float = 1.0
    m_rod: float = 0.1
    m_bob: float = 0.15

    m_total: float = 0.0
    l_com: float = 0.0
    I_pivot: float = 0.0
    I_com: float = 0.0
    inertia_factor: float = 0.0

    def compute_derived(self) -> None:
        self.m_total = self.m_rod + self.m_bob
        self.l_com = (self.m_rod * (self.L * 0.5) + self.m_bob * self.L) / self.m_total
        self.I_pivot = (1.0 / 3.0) * self.m_rod * self.L * self.L + self.m_bob * self.L * self.L
        self.I_com = self.I_pivot - self.m_total * self.l_com * self.l_com
        self.inertia_factor = 1.0 + self.I_com / (self.m_total * self.l_com * self.l_com)


@dataclass
class CartPoleParams:
    M: float = 1.2
    pole: PoleModel = field(default_factory=PoleModel)  # avoid mutable dataclass default
    g: float = 9.81
    cart_damping: float = 0.10
    pole_damping: float = 0.03


@dataclass
class ControlParams:
    lambda_theta: float = 10.0
    lambda_x: float = 1.5
    alpha: float = 0.55

    k: float = 110.0
    phi: float = 0.05

    u_max: float = 70.0

    hold_kp: float = 8.0
    hold_kd: float = 10.0
    theta_gate: float = 0.20


@dataclass
class TrackLimits:
    x_max: float = 1.6


@dataclass
class Disturbance:
    t1: float = 0.5
    t2: float = 5.0
    arrow_duration: float = 0.5
    duration: float = 0.5
    tau_amp: float = 3.3

    @staticmethod
    def half_sine(local_t: float, duration: float) -> float:
        return math.sin(math.pi * local_t / duration)

    def tau_ext(self, t: float) -> float:
        if self.t1 <= t <= self.t1 + self.duration:
            return +self.tau_amp * self.half_sine(t - self.t1, self.duration)
        if self.t2 <= t <= self.t2 + self.duration:
            return -self.tau_amp * self.half_sine(t - self.t2, self.duration)
        return 0.0

    def bob_force_equivalent(self, t: float, L: float) -> float:
        return self.tau_ext(t) / L

    def arrow_visible(self, t: float) -> bool:
        if self.t1 <= t <= self.t1 + self.arrow_duration:
            return True
        if self.t2 <= t <= self.t2 + self.arrow_duration:
            return True
        return False


# ------------------------------------------------------------
# Dynamics and control (NumPy state y = [x, xdot, theta, thetadot])
# ------------------------------------------------------------
def dynamics(p: CartPoleParams, y: np.ndarray, u_cart: float, tau_ext: float) -> np.ndarray:
    x, xdot, theta, thetadot = float(y[0]), float(y[1]), float(y[2]), float(y[3])

    m = p.pole.m_total
    l = p.pole.l_com

    total_mass = p.M + m
    polemass_length = m * l

    theta_w = wrap_to_pi(theta)
    sin_t = math.sin(theta_w)
    cos_t = math.cos(theta_w)

    F_damped = u_cart - p.cart_damping * xdot

    temp = (F_damped + polemass_length * thetadot * thetadot * sin_t) / total_mass
    denom = l * (p.pole.inertia_factor - (m * cos_t * cos_t) / total_mass)

    theta_ddot = (p.g * sin_t - cos_t * temp) / denom
    theta_ddot -= p.pole_damping * thetadot
    theta_ddot += tau_ext / p.pole.I_pivot

    x_ddot = temp - polemass_length * theta_ddot * cos_t / total_mass

    return np.array([xdot, x_ddot, thetadot, theta_ddot], dtype=np.float64)


def sliding_surface(c: ControlParams, y: np.ndarray) -> float:
    x, xdot, theta, thetadot = float(y[0]), float(y[1]), float(y[2]), float(y[3])
    theta_w = wrap_to_pi(theta)
    return thetadot + c.lambda_theta * theta_w + c.alpha * (xdot + c.lambda_x * x)


def sliding_surface_dot_nominal(p: CartPoleParams, c: ControlParams, y: np.ndarray, u_cart: float) -> float:
    d = dynamics(p, y, u_cart=u_cart, tau_ext=0.0)
    xdot = float(y[1])
    thetadot = float(y[3])
    return float(d[3]) + c.lambda_theta * thetadot + c.alpha * (float(d[1]) + c.lambda_x * xdot)


def compute_control(p: CartPoleParams, c: ControlParams, y: np.ndarray) -> float:
    sval = sliding_surface(c, y)
    desired_sdot = -c.k * sat(sval / c.phi)

    sdot0 = sliding_surface_dot_nominal(p, c, y, u_cart=0.0)
    sdot1 = sliding_surface_dot_nominal(p, c, y, u_cart=1.0)
    a = (sdot1 - sdot0)
    b = sdot0

    if abs(a) < 1e-8:
        u_smc = 0.0
    else:
        u_smc = (desired_sdot - b) / a

    x, xdot, theta = float(y[0]), float(y[1]), float(y[2])
    theta_w = wrap_to_pi(theta)
    gate = clamp(1.0 - (abs(theta_w) / c.theta_gate), 0.0, 1.0)
    u_hold = gate * (-c.hold_kp * x - c.hold_kd * xdot)

    u_total = u_smc + u_hold
    return clamp(u_total, -c.u_max, c.u_max)


def rk4_step(p: CartPoleParams, y: np.ndarray, dt: float, u: float, tau: float) -> np.ndarray:
    k1 = dynamics(p, y, u, tau)
    k2 = dynamics(p, y + 0.5 * dt * k1, u, tau)
    k3 = dynamics(p, y + 0.5 * dt * k2, u, tau)
    k4 = dynamics(p, y + dt * k3, u, tau)

    y_next = y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    y_next[2] = wrap_to_pi(float(y_next[2]))
    return y_next


# ------------------------------------------------------------
# Plot saving (dpi=300)
# ------------------------------------------------------------
def save_plots(
    out_dir: Path,
    t: np.ndarray,
    x: np.ndarray,
    theta: np.ndarray,
    u: np.ndarray,
    F_eq: np.ndarray,
    tau_ext: np.ndarray,
    s: np.ndarray,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    def save_one(y, title, xlabel, ylabel, filename):
        fig = plt.figure(figsize=(6.5, 4.0), dpi=300)
        ax = fig.add_subplot(1, 1, 1)
        ax.plot(t, y)
        ax.set_title(title)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        fig.tight_layout()
        fig.savefig(out_dir / filename, dpi=300)
        plt.close(fig)

    save_one(x, "Cart Position x(t)", "time (s)", "x (m)", "cart_position.png")
    save_one(theta, "Pole Angle theta(t) (0=upright)", "time (s)", "theta (rad)", "pole_angle.png")
    save_one(u, "Control Force u(t) (SMC)", "time (s)", "u (N)", "control_force.png")
    save_one(tau_ext, "External Disturbance Torque tau_ext(t)", "time (s)", "tau_ext (N*m)", "disturbance_torque.png")
    save_one(F_eq, "Equivalent Bob Force (for arrow direction) F_eq(t) = tau/L", "time (s)", "F_eq (N)", "equivalent_bob_force.png")
    save_one(s, "Sliding Surface s(t)", "time (s)", "s", "sliding_surface.png")


# ------------------------------------------------------------
# MP4 encoding via ffmpeg
# ------------------------------------------------------------
def encode_mp4_with_ffmpeg(frames_dir: Path, fps: int, out_mp4: Path) -> None:
    if shutil.which("ffmpeg") is None:
        print("ffmpeg not found on PATH; MP4 will not be created.")
        return

    cmd = [
        "ffmpeg",
        "-y",
        "-framerate",
        str(fps),
        "-i",
        str(frames_dir / "frame_%06d.png"),
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        str(out_mp4),
    ]

    print("Encoding MP4...")
    try:
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"MP4 created: {out_mp4}")
    except subprocess.CalledProcessError:
        print("ffmpeg encoding failed.")


# ------------------------------------------------------------
# Rendering helpers (pygame)
# ------------------------------------------------------------
def draw_force_arrow(surface, start: Tuple[float, float], end: Tuple[float, float], color: Tuple[int, int, int]) -> None:
    """
    Draw a constant-length arrow with a triangular head (direction indicator only).
    """
    import pygame

    pygame.draw.line(surface, color, start, end, width=3)

    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return

    ux, uy = dx / length, dy / length
    nx, ny = -uy, ux

    head_len = 14.0
    head_w = 7.0

    tip = end
    base = (end[0] - ux * head_len, end[1] - uy * head_len)

    p1 = tip
    p2 = (base[0] + nx * head_w, base[1] + ny * head_w)
    p3 = (base[0] - nx * head_w, base[1] - ny * head_w)

    pygame.draw.polygon(surface, color, [p1, p2, p3])


# ------------------------------------------------------------
# Integration backends
# ------------------------------------------------------------
def simulate_rk4(
    plant: CartPoleParams,
    ctrl: ControlParams,
    track: TrackLimits,
    dist: Disturbance,
    video_seconds: float,
    total_frames: int,
    substeps: int,
    y0: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Fixed-step RK4 simulation on an exact frame/substep time grid.
    """
    n_steps = total_frames * substeps
    dt = video_seconds / float(n_steps)

    t_eval = np.linspace(0.0, video_seconds, n_steps + 1, dtype=np.float64)
    Y = np.zeros((n_steps + 1, 4), dtype=np.float64)
    Y[0, :] = y0

    y = y0.copy()
    for k in range(n_steps):
        t = float(t_eval[k])

        tau = dist.tau_ext(t)
        u = compute_control(plant, ctrl, y)

        y = rk4_step(plant, y, dt, u=u, tau=tau)

        # Enforce track bounds explicitly at the physics time scale
        if y[0] > track.x_max:
            y[0] = track.x_max
            if y[1] > 0.0:
                y[1] = 0.0
        if y[0] < -track.x_max:
            y[0] = -track.x_max
            if y[1] < 0.0:
                y[1] = 0.0

        Y[k + 1, :] = y

    return t_eval, Y


def simulate_scipy(
    plant: CartPoleParams,
    ctrl: ControlParams,
    dist: Disturbance,
    video_seconds: float,
    total_frames: int,
    substeps: int,
    y0: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    SciPy solve_ivp simulation evaluated exactly on the physics time grid.

    SciPy is imported only when this path is selected, so the script can run
    in RK4 mode without SciPy installed.
    """
    try:
        from scipy.integrate import solve_ivp
    except ImportError as e:
        raise RuntimeError("SciPy is not installed. Install it or run with --integrator rk4.") from e

    n_steps = total_frames * substeps
    dt = video_seconds / float(n_steps)
    t_eval = np.linspace(0.0, video_seconds, n_steps + 1, dtype=np.float64)

    def rhs(t: float, y: np.ndarray) -> np.ndarray:
        tau = dist.tau_ext(t)
        u = compute_control(plant, ctrl, y)
        return dynamics(plant, y, u_cart=u, tau_ext=tau)

    sol = solve_ivp(
        fun=rhs,
        t_span=(0.0, video_seconds),
        y0=y0.astype(np.float64),
        t_eval=t_eval,
        method="RK45",
        max_step=dt,
        rtol=1e-7,
        atol=1e-9,
    )

    if not sol.success:
        raise RuntimeError(f"solve_ivp failed: {sol.message}")

    Y = sol.y.T.copy()
    Y[:, 2] = np.vectorize(wrap_to_pi)(Y[:, 2])
    return t_eval, Y


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------
def main() -> int:
    parser = argparse.ArgumentParser(description="Cart–Pole SMC (NumPy + optional SciPy integrator).")
    parser.add_argument("--preview", action="store_true", help="Show a live preview window while generating frames.")
    parser.add_argument(
        "--integrator",
        choices=["rk4", "scipy"],
        default="rk4",
        help="Integrator backend: 'rk4' (fixed-step) or 'scipy' (solve_ivp).",
    )
    args = parser.parse_args()

    preview = bool(args.preview)

    # ----------------------------
    # Output folders
    # ----------------------------
    out_dir = Path("output/pendulum_sliding_mode_numpy_scipy")
    frames_dir = out_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    # Clean old frames
    for p in frames_dir.glob("frame_*.png"):
        try:
            p.unlink()
        except OSError:
            pass

    # ----------------------------
    # Simulation settings
    # ----------------------------
    video_seconds = 10.0
    total_frames = 6000
    fps = int(total_frames / video_seconds)

    substeps = 8
    n_steps = total_frames * substeps
    dt_frame = video_seconds / float(total_frames)

    # ----------------------------
    # Plant / controller / disturbance
    # ----------------------------
    plant = CartPoleParams()
    plant.pole.compute_derived()

    ctrl = ControlParams()
    track = TrackLimits()
    dist = Disturbance()

    # Initial state: y = [x, xdot, theta, thetadot]
    y0 = np.array([0.0, 0.0, 0.20, 0.0], dtype=np.float64)

    # ----------------------------
    # Integrate on the physics grid
    # ----------------------------
    if args.integrator == "rk4":
        t_phys, Y = simulate_rk4(plant, ctrl, track, dist, video_seconds, total_frames, substeps, y0)
    else:
        t_phys, Y = simulate_scipy(plant, ctrl, dist, video_seconds, total_frames, substeps, y0)

    # ----------------------------
    # Prepare pygame rendering (headless-safe)
    # ----------------------------
    if not preview:
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

    import pygame

    pygame.init()

    W, H = 1000, 600
    surface = pygame.Surface((W, H))
    screen = None
    clock = pygame.time.Clock()

    if preview:
        screen = pygame.display.set_mode((W, H))
        pygame.display.set_caption("Cart–Pole SMC (NumPy/SciPy, 10s, 6000 frames)")

    # Visual mapping
    pixels_per_meter = 180.0
    origin = (W * 0.5, H * 0.75)

    # Colors
    bg = (20, 20, 20)
    track_color = (170, 170, 170)
    cart_color = (40, 140, 255)
    pole_color = (240, 70, 70)
    bob_color = (255, 220, 60)
    wheel_color = (70, 70, 70)
    arrow_color = (60, 255, 120)

    # Geometry
    cart_w, cart_h = 140, 40
    wheel_r = 14
    pole_len_px = int(plant.pole.L * pixels_per_meter)
    arrow_len_px = 120.0

    # ----------------------------
    # Per-frame logging
    # ----------------------------
    t_log = np.zeros((total_frames,), dtype=np.float64)
    x_log = np.zeros((total_frames,), dtype=np.float64)
    th_log = np.zeros((total_frames,), dtype=np.float64)
    u_log = np.zeros((total_frames,), dtype=np.float64)
    Feq_log = np.zeros((total_frames,), dtype=np.float64)
    tau_log = np.zeros((total_frames,), dtype=np.float64)
    s_log = np.zeros((total_frames,), dtype=np.float64)

    # ----------------------------
    # Render frames
    # ----------------------------
    for frame in range(total_frames):
        tf = frame * dt_frame

        # Frame aligns exactly with every 'substeps' physics steps
        k = frame * substeps
        y = Y[k, :].copy()

        # Keep rendering position within view bounds (does not change integrated history)
        y[0] = clamp(float(y[0]), -track.x_max, track.x_max)

        # Compute disturbance and control at this frame time
        tau = dist.tau_ext(tf)
        u = compute_control(plant, ctrl, y)
        Feq = dist.bob_force_equivalent(tf, plant.pole.L)
        sval = sliding_surface(ctrl, y)

        # Log
        t_log[frame] = tf
        x_log[frame] = float(y[0])
        th_log[frame] = float(y[2])
        u_log[frame] = float(u)
        Feq_log[frame] = float(Feq)
        tau_log[frame] = float(tau)
        s_log[frame] = float(sval)

        # Preview window events
        if screen is not None:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    screen = None
                    preview = False

        # Draw background
        surface.fill(bg)

        # Track line
        track_rect = pygame.Rect(0, 0, W - 100, 4)
        track_rect.center = (origin[0], origin[1] + 25.0)
        pygame.draw.rect(surface, track_color, track_rect)

        # Center marker
        center_rect = pygame.Rect(0, 0, 3, 60)
        center_rect.center = (origin[0], origin[1])
        pygame.draw.rect(surface, (120, 120, 120), center_rect)

        # Cart position
        cart_x_px = origin[0] + float(y[0]) * pixels_per_meter
        cart_y_px = origin[1]

        cart_rect = pygame.Rect(0, 0, cart_w, cart_h)
        cart_rect.center = (cart_x_px, cart_y_px)
        pygame.draw.rect(surface, cart_color, cart_rect, border_radius=6)

        # Wheels
        wheel1 = (cart_x_px - cart_w * 0.30, cart_y_px + cart_h * 0.55)
        wheel2 = (cart_x_px + cart_w * 0.30, cart_y_px + cart_h * 0.55)
        pygame.draw.circle(surface, wheel_color, (int(wheel1[0]), int(wheel1[1])), wheel_r)
        pygame.draw.circle(surface, wheel_color, (int(wheel2[0]), int(wheel2[1])), wheel_r)

        # Pivot point
        pivot = (cart_x_px, cart_y_px - cart_h * 0.5)

        # Pole tip
        theta = float(y[2])
        tip_x = pivot[0] + pole_len_px * math.sin(theta)
        tip_y = pivot[1] - pole_len_px * math.cos(theta)

        pygame.draw.line(surface, pole_color, pivot, (tip_x, tip_y), width=8)
        pygame.draw.circle(surface, bob_color, (int(tip_x), int(tip_y)), 9)

        # Disturbance arrow (constant length)
        if dist.arrow_visible(tf):
            direction = 1.0 if Feq >= 0.0 else -1.0
            start = (tip_x, tip_y - 25.0)
            end = (tip_x + direction * arrow_len_px, tip_y - 25.0)
            draw_force_arrow(surface, start, end, arrow_color)

        # Save frame
        fn = frames_dir / f"frame_{frame:06d}.png"
        pygame.image.save(surface, str(fn))

        # Optional preview
        if screen is not None:
            screen.blit(surface, (0, 0))
            pygame.display.flip()
            clock.tick(60)

        # Progress output
        if frame % 600 == 0:
            print(
                f"Frame {frame}/{total_frames}"
                f"  t={tf:6.2f}"
                f"  x={float(y[0]): .3f}"
                f"  theta={float(y[2]): .3f}"
                f"  u={float(u): .2f}"
                f"  tau={float(tau): .3f}"
            )

    pygame.quit()

    # Encode MP4
    mp4 = out_dir / "pendulum_smc_10s_6000f.mp4"
    encode_mp4_with_ffmpeg(frames_dir, fps, mp4)

    # Save plots and CSV
    print("Saving plots and CSV...")
    save_plots(out_dir, t_log, x_log, th_log, u_log, Feq_log, tau_log, s_log)

    write_csv(
        out_dir / "cartpole_log.csv",
        header=["t", "x", "theta", "u", "F_equiv", "tau_ext", "s"],
        cols=[t_log, x_log, th_log, u_log, Feq_log, tau_log, s_log],
    )

    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
