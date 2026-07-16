#!/usr/bin/env python3
# ------------------------------------------------------------
# Cart–Pole Inverted Pendulum (moving base) with SLIDING MODE CONTROL (SMC)
# ------------------------------------------------------------
# Requirements implemented:
#   - Total simulation time: 10 seconds
#   - Total frames: 6000 (=> 600 FPS video)
#   - Exactly two disturbances: at t=0.5s and t=5.0s
#   - Disturbance shown as an arrow for 0.5s only (direction indicator, not stretched)
#   - Dynamics are integrated and used to render each frame
#   - Outputs: frames, mp4 (ffmpeg), plots (Matplotlib, dpi=300), CSV log
#
# Output folders:
#   output/pendulum_sliding_mode/frames/frame_000000.png ... frame_005999.png
#   output/pendulum_sliding_mode/pendulum_smc_10s_6000f.mp4   (if ffmpeg in PATH)
#   output/pendulum_sliding_mode/*.png plots
#   output/pendulum_sliding_mode/cartpole_log.csv
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
from typing import List, Optional, Tuple

# Force a non-GUI Matplotlib backend to avoid Tk/Tcl dependency issues on some systems.
import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt


# ------------------------------------------------------------
# Small math helpers
# ------------------------------------------------------------
def clamp(x: float, lo: float, hi: float) -> float:
    """Clamp x into the closed interval [lo, hi]."""
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def wrap_to_pi(a: float) -> float:
    """Wrap angle a (radians) into [-pi, +pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def sat(z: float) -> float:
    """Boundary-layer saturation for sliding mode: sat(z) in [-1, 1]."""
    return clamp(z, -1.0, 1.0)


# ------------------------------------------------------------
# CSV writer
# ------------------------------------------------------------
def write_csv(filename: Path, header: List[str], cols: List[List[float]]) -> None:
    """
    Write CSV where each element of cols is one column (same length required).
    """
    if not cols:
        raise ValueError("CSV: no columns provided.")
    n = len(cols[0])
    for c in cols:
        if len(c) != n:
            raise ValueError("CSV: column size mismatch.")

    filename.parent.mkdir(parents=True, exist_ok=True)

    with filename.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        for r in range(n):
            w.writerow([cols[c][r] for c in range(len(cols))])


# ------------------------------------------------------------
# Pole model: uniform rod + lumped bob at the top
# ------------------------------------------------------------
@dataclass
class PoleModel:
    L: float = 1.0        # (m)
    m_rod: float = 0.06   # (kg)
    m_bob: float = 0.04   # (kg)

    m_total: float = 0.0
    l_com: float = 0.0
    I_pivot: float = 0.0
    I_com: float = 0.0
    inertia_factor: float = 0.0  # 1 + I_com/(m*l^2)

    def compute_derived(self) -> None:
        """Compute derived mass and inertia quantities used by the dynamics."""
        self.m_total = self.m_rod + self.m_bob

        # COM from pivot: rod at L/2, bob at L
        self.l_com = (self.m_rod * (self.L * 0.5) + self.m_bob * self.L) / self.m_total

        # Inertia about pivot: rod (1/3)mL^2, bob mL^2
        self.I_pivot = (1.0 / 3.0) * self.m_rod * self.L * self.L + self.m_bob * self.L * self.L

        # Inertia about COM (parallel-axis theorem)
        self.I_com = self.I_pivot - self.m_total * self.l_com * self.l_com

        # Convenience factor used in generalized cart-pole equations
        self.inertia_factor = 1.0 + self.I_com / (self.m_total * self.l_com * self.l_com)


# ------------------------------------------------------------
# Plant parameters
# ------------------------------------------------------------
@dataclass
class CartPoleParams:
    M: float = 1.2  # cart mass (kg)
    pole: PoleModel = field(default_factory=PoleModel)  # avoid mutable dataclass default
    g: float = 9.81

    # Damping improves realism and numerical behavior
    cart_damping: float = 0.10
    pole_damping: float = 0.03


# ------------------------------------------------------------
# State (theta=0 upright)
# Convention: tip_x = x + L*sin(theta), so theta>0 leans to the right.
# ------------------------------------------------------------
@dataclass
class State:
    x: float = 0.0
    xdot: float = 0.0
    theta: float = 0.20
    thetadot: float = 0.0


# ------------------------------------------------------------
# Sliding Mode Control parameters + a small cart-centering term
# ------------------------------------------------------------
@dataclass
class ControlParams:
    # Sliding surface:
    # s = theta_dot + lambda_theta*theta + alpha*(x_dot + lambda_x*x)
    lambda_theta: float = 10.0
    lambda_x: float = 1.5
    alpha: float = 0.55

    # Sliding mode dynamics
    k: float = 110.0
    phi: float = 0.05

    # Actuator saturation
    u_max: float = 70.0  # N

    # Cart centering (only near upright)
    hold_kp: float = 8.0   # N/m
    hold_kd: float = 10.0  # N/(m/s)

    # Gate: when |theta| >= theta_gate, centering ~ 0
    theta_gate: float = 0.20  # rad


# ------------------------------------------------------------
# Track limit: keep cart position within view
# ------------------------------------------------------------
@dataclass
class TrackLimits:
    x_max: float = 1.6

    def enforce(self, s: State) -> None:
        """Clip cart position to [-x_max, x_max] and stop outward velocity at limits."""
        if s.x > self.x_max:
            s.x = self.x_max
            if s.xdot > 0.0:
                s.xdot = 0.0
        if s.x < -self.x_max:
            s.x = -self.x_max
            if s.xdot < 0.0:
                s.xdot = 0.0


# ------------------------------------------------------------
# Disturbance schedule: exactly two disturbances
#   - Pulse 1 starts at t=0.5s
#   - Pulse 2 starts at t=5.0s
# Arrow is shown for 0.5s after each pulse start.
# Disturbance is applied as an external torque about the pivot.
# ------------------------------------------------------------
@dataclass
class Disturbance:
    t1: float = 0.5
    t2: float = 5.0

    arrow_duration: float = 0.5
    duration: float = 0.5

    # Torque amplitude (N*m)
    tau_amp: float = 3.3

    @staticmethod
    def half_sine(local_t: float, duration: float) -> float:
        """Smooth half-sine pulse: 0 -> 1 -> 0 on interval [0, duration]."""
        return math.sin(math.pi * local_t / duration)

    def tau_ext(self, t: float) -> float:
        """External torque about pivot (N*m). Positive torque tends to increase theta."""
        if self.t1 <= t <= self.t1 + self.duration:
            return +self.tau_amp * self.half_sine(t - self.t1, self.duration)
        if self.t2 <= t <= self.t2 + self.duration:
            return -self.tau_amp * self.half_sine(t - self.t2, self.duration)
        return 0.0

    def bob_force_equivalent(self, t: float, L: float) -> float:
        """Equivalent bob force used for arrow direction: F_eq = tau / L."""
        return self.tau_ext(t) / L

    def arrow_visible(self, t: float) -> bool:
        """Arrow is visible for arrow_duration after each disturbance starts."""
        if self.t1 <= t <= self.t1 + self.arrow_duration:
            return True
        if self.t2 <= t <= self.t2 + self.arrow_duration:
            return True
        return False


# ------------------------------------------------------------
# ODE derivative container
# ------------------------------------------------------------
@dataclass
class Deriv:
    x_dot: float
    x_ddot: float
    theta_dot: float
    theta_ddot: float


# ------------------------------------------------------------
# Nonlinear cart–pole dynamics + external torque tau_ext
# ------------------------------------------------------------
def dynamics(p: CartPoleParams, s: State, u_cart: float, tau_ext: float) -> Deriv:
    """
    Compute time-derivatives for cart-pole with external torque.
    """
    m = p.pole.m_total
    l = p.pole.l_com

    total_mass = p.M + m
    polemass_length = m * l

    sin_t = math.sin(s.theta)
    cos_t = math.cos(s.theta)

    # Damped cart force (simple friction-like term)
    F_damped = u_cart - p.cart_damping * s.xdot

    temp = (F_damped + polemass_length * s.thetadot * s.thetadot * sin_t) / total_mass
    denom = l * (p.pole.inertia_factor - (m * cos_t * cos_t) / total_mass)

    theta_ddot = (p.g * sin_t - cos_t * temp) / denom

    # Pole damping (applied to angular rate)
    theta_ddot -= p.pole_damping * s.thetadot

    # External torque about pivot (disturbance)
    theta_ddot += tau_ext / p.pole.I_pivot

    x_ddot = temp - polemass_length * theta_ddot * cos_t / total_mass

    return Deriv(x_dot=s.xdot, x_ddot=x_ddot, theta_dot=s.thetadot, theta_ddot=theta_ddot)


# ------------------------------------------------------------
# RK4 integration step
# ------------------------------------------------------------
def rk4_step(p: CartPoleParams, s: State, dt: float, u_cart: float, tau_ext: float) -> None:
    """Advance the state by one RK4 step of size dt."""
    def add_scaled(a: State, k: Deriv, h: float) -> State:
        """Return a + h*k (componentwise) without mutating the original."""
        return State(
            x=a.x + h * k.x_dot,
            xdot=a.xdot + h * k.x_ddot,
            theta=a.theta + h * k.theta_dot,
            thetadot=a.thetadot + h * k.theta_ddot,
        )

    k1 = dynamics(p, s, u_cart, tau_ext)
    k2 = dynamics(p, add_scaled(s, k1, 0.5 * dt), u_cart, tau_ext)
    k3 = dynamics(p, add_scaled(s, k2, 0.5 * dt), u_cart, tau_ext)
    k4 = dynamics(p, add_scaled(s, k3, dt), u_cart, tau_ext)

    s.x += (dt / 6.0) * (k1.x_dot + 2.0 * k2.x_dot + 2.0 * k3.x_dot + k4.x_dot)
    s.xdot += (dt / 6.0) * (k1.x_ddot + 2.0 * k2.x_ddot + 2.0 * k3.x_ddot + k4.x_ddot)
    s.theta += (dt / 6.0) * (k1.theta_dot + 2.0 * k2.theta_dot + 2.0 * k3.theta_dot + k4.theta_dot)
    s.thetadot += (dt / 6.0) * (k1.theta_ddot + 2.0 * k2.theta_ddot + 2.0 * k3.theta_ddot + k4.theta_ddot)

    # Keep angle bounded for numerical stability and clean plots
    s.theta = wrap_to_pi(s.theta)


# ------------------------------------------------------------
# Sliding surface and SMC control
# ------------------------------------------------------------
def sliding_surface(c: ControlParams, s: State) -> float:
    """Compute the sliding surface s(t)."""
    return (
        s.thetadot
        + c.lambda_theta * s.theta
        + c.alpha * (s.xdot + c.lambda_x * s.x)
    )


def sliding_surface_dot_nominal(p: CartPoleParams, c: ControlParams, s: State, u_cart: float) -> float:
    """
    Estimate sdot under nominal dynamics (ignoring external torque in the control law).
    """
    d = dynamics(p, s, u_cart, tau_ext=0.0)
    return (
        d.theta_ddot
        + c.lambda_theta * s.thetadot
        + c.alpha * (d.x_ddot + c.lambda_x * s.xdot)
    )


def compute_control(p: CartPoleParams, c: ControlParams, s: State) -> float:
    """
    Sliding Mode Control with a boundary layer and a gated cart-centering term.
    """
    # Sliding mode term
    sval = sliding_surface(c, s)
    desired_sdot = -c.k * sat(sval / c.phi)

    # Numeric affine approximation: sdot(u) ≈ a*u + b
    sdot0 = sliding_surface_dot_nominal(p, c, s, u_cart=0.0)
    sdot1 = sliding_surface_dot_nominal(p, c, s, u_cart=1.0)
    a = (sdot1 - sdot0)
    b = sdot0

    if abs(a) < 1e-8:
        u_smc = 0.0
    else:
        u_smc = (desired_sdot - b) / a

    # Gated cart-centering near upright
    theta_abs = abs(s.theta)
    gate = clamp(1.0 - (theta_abs / c.theta_gate), 0.0, 1.0)

    u_hold = gate * (-c.hold_kp * s.x - c.hold_kd * s.xdot)

    u_total = u_smc + u_hold
    return clamp(u_total, -c.u_max, c.u_max)


# ------------------------------------------------------------
# Plot saving (dpi=300)
# ------------------------------------------------------------
def save_plots(
    out_dir: Path,
    t: List[float],
    x: List[float],
    theta: List[float],
    u: List[float],
    F_eq: List[float],
    tau_ext: List[float],
    ssurf: List[float],
) -> None:
    """Save high-quality plots (dpi=300) for key signals."""
    out_dir.mkdir(parents=True, exist_ok=True)

    def save_one(y: List[float], title: str, xlabel: str, ylabel: str, filename: str) -> None:
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
    save_one(ssurf, "Sliding Surface s(t)", "time (s)", "s", "sliding_surface.png")


# ------------------------------------------------------------
# MP4 encoding via ffmpeg
# ------------------------------------------------------------
def encode_mp4_with_ffmpeg(frames_dir: Path, fps: int, out_mp4: Path) -> None:
    """Encode an MP4 from numbered PNG frames using ffmpeg if it is available."""
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
# Rendering (pygame)
# ------------------------------------------------------------
def draw_force_arrow(surface, start: Tuple[float, float], end: Tuple[float, float], color: Tuple[int, int, int]) -> None:
    """
    Draw a constant-length arrow with a triangular head.
    The arrow is intentionally not stretched by magnitude to serve as a direction indicator.
    """
    import pygame

    # Main arrow shaft
    pygame.draw.line(surface, color, start, end, width=3)

    # Arrowhead geometry
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return

    ux = dx / length
    uy = dy / length

    # Perpendicular unit vector
    nx = -uy
    ny = ux

    head_len = 14.0
    head_w = 7.0

    tip = end
    base = (end[0] - ux * head_len, end[1] - uy * head_len)

    p1 = tip
    p2 = (base[0] + nx * head_w, base[1] + ny * head_w)
    p3 = (base[0] - nx * head_w, base[1] - ny * head_w)

    pygame.draw.polygon(surface, color, [p1, p2, p3])


def main() -> int:
    parser = argparse.ArgumentParser(description="Cart–Pole SMC simulation and rendering.")
    parser.add_argument("--preview", action="store_true", help="Show a live preview window while generating frames.")
    args = parser.parse_args()

    preview = bool(args.preview)

    # ----------------------------
    # Output folders
    # ----------------------------
    out_dir = Path("output/pendulum_sliding_mode")
    frames_dir = out_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    # Remove old frames to avoid mixing multiple runs
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
    fps = int(total_frames / video_seconds)  # 600 FPS

    dt_frame = video_seconds / float(total_frames)

    # Integrate with multiple substeps per frame for stability
    substeps = 8
    dt_physics = dt_frame / float(substeps)

    # ----------------------------
    # Plant / controller / disturbance
    # ----------------------------
    plant = CartPoleParams()
    plant.M = 1.2
    plant.pole.L = 1.0
    plant.pole.m_rod = 0.1
    plant.pole.m_bob = 0.15
    plant.pole.compute_derived()

    ctrl = ControlParams()
    track = TrackLimits()
    dist = Disturbance()

    # Initial state
    s = State(theta=0.20)

    # ----------------------------
    # Logging (one entry per frame)
    # ----------------------------
    t_log: List[float] = []
    x_log: List[float] = []
    th_log: List[float] = []
    u_log: List[float] = []
    Feq_log: List[float] = []
    tau_log: List[float] = []
    surf_log: List[float] = []

    # ----------------------------
    # Initialize pygame (headless-safe)
    # ----------------------------
    if not preview:
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

    import pygame

    pygame.init()

    W, H = 1000, 600
    surface = pygame.Surface((W, H))
    screen: Optional["pygame.Surface"] = None
    clock = pygame.time.Clock()

    if preview:
        screen = pygame.display.set_mode((W, H))
        pygame.display.set_caption("Cart–Pole SMC (10s, 6000 frames)")

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

    # Arrow constant size (not scaled by magnitude)
    arrow_len_px = 120.0

    # ----------------------------
    # Main loop
    # ----------------------------
    t = 0.0

    for frame in range(total_frames):
        # Preview window events (allow closing without stopping frame generation)
        if preview:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    preview = False
                    screen = None

        # Integrate dynamics for this frame using substeps
        u_used = 0.0
        tau_used = 0.0
        Feq_used = 0.0
        s_used = 0.0

        for _ in range(substeps):
            tau_ext = dist.tau_ext(t)
            u = compute_control(plant, ctrl, s)
            rk4_step(plant, s, dt_physics, u_cart=u, tau_ext=tau_ext)
            track.enforce(s)

            u_used = u
            tau_used = tau_ext
            Feq_used = dist.bob_force_equivalent(t, plant.pole.L)
            s_used = sliding_surface(ctrl, s)

            t += dt_physics

        tf = float(frame) * dt_frame

        # Log once per frame
        t_log.append(tf)
        x_log.append(s.x)
        th_log.append(s.theta)
        u_log.append(u_used)
        tau_log.append(tau_used)
        Feq_log.append(Feq_used)
        surf_log.append(s_used)

        # ----------------------------
        # Render current integrated state
        # ----------------------------
        surface.fill(bg)

        # Track line
        import pygame
        track_rect = pygame.Rect(0, 0, W - 100, 4)
        track_rect.center = (origin[0], origin[1] + 25.0)
        pygame.draw.rect(surface, track_color, track_rect)

        # Center marker
        center_rect = pygame.Rect(0, 0, 3, 60)
        center_rect.center = (origin[0], origin[1])
        pygame.draw.rect(surface, (120, 120, 120), center_rect)

        # Cart position in pixels
        cart_x_px = origin[0] + s.x * pixels_per_meter
        cart_y_px = origin[1]

        cart_rect = pygame.Rect(0, 0, cart_w, cart_h)
        cart_rect.center = (cart_x_px, cart_y_px)
        pygame.draw.rect(surface, cart_color, cart_rect, border_radius=6)

        # Wheels
        w1 = (cart_x_px - cart_w * 0.30, cart_y_px + cart_h * 0.55)
        w2 = (cart_x_px + cart_w * 0.30, cart_y_px + cart_h * 0.55)
        pygame.draw.circle(surface, wheel_color, (int(w1[0]), int(w1[1])), wheel_r)
        pygame.draw.circle(surface, wheel_color, (int(w2[0]), int(w2[1])), wheel_r)

        # Pivot point at the top center of the cart
        pivot = (cart_x_px, cart_y_px - cart_h * 0.5)

        # Pole tip location
        tip_x = pivot[0] + pole_len_px * math.sin(s.theta)
        tip_y = pivot[1] - pole_len_px * math.cos(s.theta)

        pygame.draw.line(surface, pole_color, pivot, (tip_x, tip_y), width=8)
        pygame.draw.circle(surface, bob_color, (int(tip_x), int(tip_y)), 9)

        # Disturbance arrow shown only for 0.5 s after each disturbance start
        if dist.arrow_visible(tf):
            direction = 1.0 if Feq_used >= 0.0 else -1.0
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

        # Console progress (about once per second at 600 FPS)
        if frame % 600 == 0:
            print(
                f"Frame {frame}/{total_frames}"
                f"  t={tf:6.2f}"
                f"  x={s.x: .3f}"
                f"  theta={s.theta: .3f}"
                f"  u={u_used: .2f}"
                f"  tau={tau_used: .3f}"
            )

    pygame.quit()

    # Encode MP4
    mp4 = out_dir / "pendulum_smc_10s_6000f.mp4"
    encode_mp4_with_ffmpeg(frames_dir, fps, mp4)

    # Save plots and CSV
    print("Saving plots and CSV...")
    save_plots(out_dir, t_log, x_log, th_log, u_log, Feq_log, tau_log, surf_log)

    write_csv(
        out_dir / "cartpole_log.csv",
        header=["t", "x", "theta", "u", "F_equiv", "tau_ext", "s"],
        cols=[t_log, x_log, th_log, u_log, Feq_log, tau_log, surf_log],
    )

    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
