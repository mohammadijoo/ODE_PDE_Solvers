// ------------------------------------------------------------
// Cart–Pole Inverted Pendulum (moving base) with SLIDING MODE CONTROL (SMC)
// Requirements implemented:
//   - Total animation: 10 seconds
//   - Exactly two disturbances: at t=0.5s and t=5.0s
//   - Disturbance shown as an arrow for 0.5s only (direction indicator, not stretched)
//   - Disturbance direction and base compensation direction are consistent:
//       "push right" => pole tips right => cart accelerates right to recover
//   - Dynamics are integrated and used to render each frame
//   - Outputs: frames, optional mp4 (ffmpeg), plots, CSV log
//
// Output folders:
//   output/pendulum_sliding_mode/frames/frame_000000.png ... frame_001799.png
//   output/pendulum_sliding_mode/pendulum_smc_10s_1800f.mp4   (if ffmpeg in PATH)
//   output/pendulum_sliding_mode/*.png plots
//   output/pendulum_sliding_mode/cartpole_log.csv
// ------------------------------------------------------------

use anyhow::{Context, Result};
use image::{ImageBuffer, Rgba};
use imageproc::drawing::{
    draw_filled_circle_mut, draw_filled_rect_mut, draw_line_segment_mut, draw_polygon_mut,
};
use imageproc::rect::Rect;
use minifb::{Key, Window, WindowOptions};
use plotters::prelude::*;
use std::f64::consts::PI;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

// ------------------------------------------------------------
// Small math helpers
// ------------------------------------------------------------
fn clamp(x: f64, lo: f64, hi: f64) -> f64 {
    if x < lo {
        lo
    } else if x > hi {
        hi
    } else {
        x
    }
}

fn wrap_to_pi(mut a: f64) -> f64 {
    while a > PI {
        a -= 2.0 * PI;
    }
    while a < -PI {
        a += 2.0 * PI;
    }
    a
}

// Boundary-layer saturation for sliding mode: sat(z) in [-1, 1]
fn sat(z: f64) -> f64 {
    clamp(z, -1.0, 1.0)
}

// ------------------------------------------------------------
// CSV writer
// ------------------------------------------------------------
fn write_csv(
    filename: &Path,
    header: &[&str],
    cols: &[Vec<f64>],
) -> Result<()> {
    if cols.is_empty() {
        anyhow::bail!("CSV: no columns.");
    }
    let n = cols[0].len();
    for c in cols {
        if c.len() != n {
            anyhow::bail!("CSV: column size mismatch.");
        }
    }

    let mut wtr = csv::Writer::from_path(filename)
        .with_context(|| format!("CSV: cannot open {}", filename.display()))?;

    wtr.write_record(header)?;
    for r in 0..n {
        let row: Vec<String> = cols.iter().map(|c| c[r].to_string()).collect();
        wtr.write_record(row)?;
    }
    wtr.flush()?;
    Ok(())
}

// ------------------------------------------------------------
// Pole model: uniform rod + lumped bob at the top
// ------------------------------------------------------------
#[derive(Clone, Copy)]
struct PoleModel {
    l: f64,       // length (m)
    m_rod: f64,   // rod mass (kg)
    m_bob: f64,   // bob mass (kg)

    m_total: f64,
    l_com: f64,
    i_pivot: f64,
    i_com: f64,
    inertia_factor: f64, // 1 + I_com/(m*l^2)
}

impl PoleModel {
    fn new() -> Self {
        Self {
            l: 1.0,
            m_rod: 0.06,
            m_bob: 0.04,
            m_total: 0.0,
            l_com: 0.0,
            i_pivot: 0.0,
            i_com: 0.0,
            inertia_factor: 0.0,
        }
    }

    fn compute_derived(&mut self) {
        self.m_total = self.m_rod + self.m_bob;

        // COM from pivot: rod at L/2, bob at L
        self.l_com = (self.m_rod * (self.l * 0.5) + self.m_bob * self.l) / self.m_total;

        // Inertia about pivot:
        // rod: (1/3)mL^2, bob: mL^2
        self.i_pivot = (1.0 / 3.0) * self.m_rod * self.l * self.l + self.m_bob * self.l * self.l;

        // Inertia about COM
        self.i_com = self.i_pivot - self.m_total * self.l_com * self.l_com;

        // Inertia factor used in a generalized cart–pole form
        self.inertia_factor = 1.0 + self.i_com / (self.m_total * self.l_com * self.l_com);
    }
}

// ------------------------------------------------------------
// Plant parameters
// ------------------------------------------------------------
#[derive(Clone, Copy)]
struct CartPoleParams {
    m_cart: f64,   // cart mass (kg)
    pole: PoleModel,

    g: f64,

    // Damping for realism and numerical stability
    cart_damping: f64, // N per (m/s)
    pole_damping: f64, // applied to theta_ddot term
}

// ------------------------------------------------------------
// State (theta=0 upright)
// Rendering convention: tip_x = x + L*sin(theta), so theta>0 leans right.
// ------------------------------------------------------------
#[derive(Clone, Copy)]
struct State {
    x: f64,
    xdot: f64,
    theta: f64,
    thetadot: f64,
}

// ------------------------------------------------------------
// Sliding Mode Control parameters + a small cart-centering term
// The centering term is gated: it becomes strong only when |theta| is small.
// ------------------------------------------------------------
#[derive(Clone, Copy)]
struct ControlParams {
    // Sliding surface:
    // s = theta_dot + lambda_theta*theta + alpha*(x_dot + lambda_x*x)
    lambda_theta: f64,
    lambda_x: f64,
    alpha: f64,

    // Sliding mode dynamics
    k: f64,
    phi: f64,

    // Actuator saturation
    u_max: f64, // N

    // Cart centering (only near upright)
    hold_kp: f64,  // N/m
    hold_kd: f64,  // N/(m/s)

    // Gate: when |theta| >= theta_gate, centering is ~0
    theta_gate: f64, // rad
}

// ------------------------------------------------------------
// Track limit: keep the cart in view
// ------------------------------------------------------------
#[derive(Clone, Copy)]
struct TrackLimits {
    x_max: f64, // meters
}

impl TrackLimits {
    fn enforce(&self, s: &mut State) {
        if s.x > self.x_max {
            s.x = self.x_max;
            if s.xdot > 0.0 {
                s.xdot = 0.0;
            }
        }
        if s.x < -self.x_max {
            s.x = -self.x_max;
            if s.xdot < 0.0 {
                s.xdot = 0.0;
            }
        }
    }
}

// ------------------------------------------------------------
// Disturbance schedule (exactly two disturbances)
// Arrow shown for 0.5 s only after each pulse start.
// Modelled as an external torque about the pivot.
// ------------------------------------------------------------
#[derive(Clone, Copy)]
struct Disturbance {
    t1: f64,
    t2: f64,
    arrow_duration: f64,
    duration: f64,
    tau_amp: f64, // N*m
}

impl Disturbance {
    fn half_sine(local_t: f64, duration: f64) -> f64 {
        (PI * local_t / duration).sin()
    }

    fn tau_ext(&self, t: f64) -> f64 {
        if t >= self.t1 && t <= self.t1 + self.duration {
            let local = t - self.t1;
            return self.tau_amp * Self::half_sine(local, self.duration);
        }
        if t >= self.t2 && t <= self.t2 + self.duration {
            let local = t - self.t2;
            return -self.tau_amp * Self::half_sine(local, self.duration);
        }
        0.0
    }

    // Equivalent bob force is used only for arrow direction and logging: F_eq = tau / L.
    fn bob_force_equivalent(&self, t: f64, l: f64) -> f64 {
        self.tau_ext(t) / l
    }

    fn arrow_visible(&self, t: f64) -> bool {
        (t >= self.t1 && t <= self.t1 + self.arrow_duration)
            || (t >= self.t2 && t <= self.t2 + self.arrow_duration)
    }
}

// ------------------------------------------------------------
// ODE derivative
// ------------------------------------------------------------
#[derive(Clone, Copy)]
struct Deriv {
    x_dot: f64,
    x_ddot: f64,
    theta_dot: f64,
    theta_ddot: f64,
}

// ------------------------------------------------------------
// Nonlinear cart–pole dynamics + external torque tau_ext.
// Generalized Gym-like form using inertia_factor.
// Adds damping and an external torque term on theta_ddot.
// ------------------------------------------------------------
fn dynamics(p: &CartPoleParams, s: &State, u_cart: f64, tau_ext: f64) -> Deriv {
    let m = p.pole.m_total;
    let l = p.pole.l_com;

    let total_mass = p.m_cart + m;
    let polemass_length = m * l;

    let sin_t = s.theta.sin();
    let cos_t = s.theta.cos();

    // Cart friction-like damping
    let f_damped = u_cart - p.cart_damping * s.xdot;

    let temp = (f_damped + polemass_length * s.thetadot * s.thetadot * sin_t) / total_mass;

    let denom = l * (p.pole.inertia_factor - (m * cos_t * cos_t) / total_mass);

    let mut theta_ddot = (p.g * sin_t - cos_t * temp) / denom;

    // Pole damping
    theta_ddot -= p.pole_damping * s.thetadot;

    // External torque about pivot (disturbance)
    theta_ddot += tau_ext / p.pole.i_pivot;

    let x_ddot = temp - polemass_length * theta_ddot * cos_t / total_mass;

    Deriv {
        x_dot: s.xdot,
        x_ddot,
        theta_dot: s.thetadot,
        theta_ddot,
    }
}

// ------------------------------------------------------------
// RK4 integration step
// ------------------------------------------------------------
fn rk4_step(p: &CartPoleParams, s: &mut State, dt: f64, u_cart: f64, tau_ext: f64) {
    let add_scaled = |a: State, k: Deriv, h: f64| -> State {
        State {
            x: a.x + h * k.x_dot,
            xdot: a.xdot + h * k.x_ddot,
            theta: a.theta + h * k.theta_dot,
            thetadot: a.thetadot + h * k.theta_ddot,
        }
    };

    let k1 = dynamics(p, s, u_cart, tau_ext);
    let k2 = dynamics(p, &add_scaled(*s, k1, 0.5 * dt), u_cart, tau_ext);
    let k3 = dynamics(p, &add_scaled(*s, k2, 0.5 * dt), u_cart, tau_ext);
    let k4 = dynamics(p, &add_scaled(*s, k3, dt), u_cart, tau_ext);

    s.x += (dt / 6.0) * (k1.x_dot + 2.0 * k2.x_dot + 2.0 * k3.x_dot + k4.x_dot);
    s.xdot += (dt / 6.0) * (k1.x_ddot + 2.0 * k2.x_ddot + 2.0 * k3.x_ddot + k4.x_ddot);
    s.theta += (dt / 6.0) * (k1.theta_dot + 2.0 * k2.theta_dot + 2.0 * k3.theta_dot + k4.theta_dot);
    s.thetadot += (dt / 6.0) * (k1.theta_ddot + 2.0 * k2.theta_ddot + 2.0 * k3.theta_ddot + k4.theta_ddot);

    s.theta = wrap_to_pi(s.theta);
}

// ------------------------------------------------------------
// Sliding surface and SMC control
// ------------------------------------------------------------
fn sliding_surface(c: &ControlParams, s: &State) -> f64 {
    s.thetadot + c.lambda_theta * s.theta + c.alpha * (s.xdot + c.lambda_x * s.x)
}

fn sliding_surface_dot_nominal(p: &CartPoleParams, c: &ControlParams, s: &State, u_cart: f64) -> f64 {
    // Nominal: ignore external torque for the control law
    let d = dynamics(p, s, u_cart, 0.0);

    // sdot = theta_ddot + lambda_theta*theta_dot + alpha*(x_ddot + lambda_x*x_dot)
    d.theta_ddot + c.lambda_theta * s.thetadot + c.alpha * (d.x_ddot + c.lambda_x * s.xdot)
}

fn compute_control(p: &CartPoleParams, c: &ControlParams, s: &State) -> f64 {
    // ----- Sliding Mode Control part -----
    let sval = sliding_surface(c, s);
    let desired_sdot = -c.k * sat(sval / c.phi);

    // Numeric affine approximation: sdot(u) ≈ a*u + b
    let sdot0 = sliding_surface_dot_nominal(p, c, s, 0.0);
    let sdot1 = sliding_surface_dot_nominal(p, c, s, 1.0);
    let a = sdot1 - sdot0;
    let b = sdot0;

    let u_smc = if a.abs() < 1e-8 {
        0.0
    } else {
        (desired_sdot - b) / a
    };

    // ----- Cart-centering term (gated) -----
    // When the pole is near upright, pull cart back to x=0.
    // When pole is far from upright, don't fight the catch maneuver.
    let theta_abs = s.theta.abs();
    let gate = clamp(1.0 - (theta_abs / c.theta_gate), 0.0, 1.0);

    let u_hold = gate * (-c.hold_kp * s.x - c.hold_kd * s.xdot);

    // Combined control + saturation
    clamp(u_smc + u_hold, -c.u_max, c.u_max)
}

// ------------------------------------------------------------
// Rendering helpers (CPU rasterization)
// ------------------------------------------------------------

type Img = ImageBuffer<Rgba<u8>, Vec<u8>>;

fn rgba(r: u8, g: u8, b: u8, a: u8) -> Rgba<u8> {
    Rgba([r, g, b, a])
}

// Draw a constant-size arrow (line + triangle head).
fn draw_force_arrow(img: &mut Img, start: (f32, f32), end: (f32, f32), color: Rgba<u8>) {
    // Main line (slightly thicker for visibility)
    for off in -1..=1 {
        let s = (start.0, start.1 + off as f32);
        let e = (end.0, end.1 + off as f32);
        draw_line_segment_mut(img, s, e, color);
    }

    // Arrowhead geometry
    let dx = end.0 - start.0;
    let dy = end.1 - start.1;
    let len = (dx * dx + dy * dy).sqrt();
    if len < 1e-3 {
        return;
    }

    let ux = dx / len;
    let uy = dy / len;

    // Normal vector (perpendicular)
    let nx = -uy;
    let ny = ux;

    let head_len = 14.0_f32;
    let head_w = 7.0_f32;

    let tip = (end.0, end.1);
    let base = (end.0 - ux * head_len, end.1 - uy * head_len);

    let p1 = (base.0 + nx * head_w, base.1 + ny * head_w);
    let p2 = (base.0 - nx * head_w, base.1 - ny * head_w);

    let poly = vec![
        imageproc::point::Point::new(tip.0 as i32, tip.1 as i32),
        imageproc::point::Point::new(p1.0 as i32, p1.1 as i32),
        imageproc::point::Point::new(p2.0 as i32, p2.1 as i32),
    ];
    draw_polygon_mut(img, &poly, color);
}

// Draw a rotated rectangle as a filled quadrilateral.
// Inputs:
// - center (cx, cy) in pixels
// - width, height in pixels
// - angle in radians (positive rotates clockwise in screen coordinates if y increases downward)
#[allow(dead_code)]
fn draw_rotated_rect(
    img: &mut Img,
    cx: f32,
    cy: f32,
    width: f32,
    height: f32,
    angle_rad: f32,
    color: Rgba<u8>,
) {
    let hw = 0.5 * width;
    let hh = 0.5 * height;

    // Rectangle corners in local coordinates (origin at center)
    let corners = [
        (-hw, -hh),
        ( hw, -hh),
        ( hw,  hh),
        (-hw,  hh),
    ];

    let ca = angle_rad.cos();
    let sa = angle_rad.sin();

    // Rotate and translate corners
    let pts: Vec<imageproc::point::Point<i32>> = corners
        .iter()
        .map(|(x, y)| {
            let xr = ca * x - sa * y;
            let yr = sa * x + ca * y;
            imageproc::point::Point::new((cx + xr) as i32, (cy + yr) as i32)
        })
        .collect();

    draw_polygon_mut(img, &pts, color);
}


// Draw a thicker line segment by rendering several parallel line segments.
// This is used for the pole so it visually remains connected to the bob tip
// at all angles (same endpoints are used for both pole and bob).
fn draw_thick_line_segment(
    img: &mut Img,
    start: (f32, f32),
    end: (f32, f32),
    thickness_px: i32,
    color: Rgba<u8>,
) {
    let dx = end.0 - start.0;
    let dy = end.1 - start.1;

    let len = (dx * dx + dy * dy).sqrt();
    if len < 1e-3 {
        return;
    }

    // Unit normal (perpendicular) direction used for parallel offsets.
    let nx = -dy / len;
    let ny =  dx / len;

    let half = thickness_px.max(1) / 2;

    for k in -half..=half {
        let off = k as f32;
        let s = (start.0 + nx * off, start.1 + ny * off);
        let e = (end.0 + nx * off, end.1 + ny * off);
        draw_line_segment_mut(img, s, e, color);
    }
}

// Convert an RGBA image to a minifb buffer (u32 ARGB).
fn to_minifb_buffer(img: &Img) -> Vec<u32> {
    let mut out = vec![0u32; (img.width() * img.height()) as usize];
    for (i, p) in img.pixels().enumerate() {
        let r = p[0] as u32;
        let g = p[1] as u32;
        let b = p[2] as u32;
        out[i] = (255u32 << 24) | (r << 16) | (g << 8) | b;
    }
    out
}

// ------------------------------------------------------------
// Plot saving (Plotters)
// ------------------------------------------------------------
fn save_line_plot_png(
    filename: &Path,
    title: &str,
    xlabel: &str,
    ylabel: &str,
    x: &[f64],
    y: &[f64],
) -> Result<()> {
    if x.len() != y.len() {
        anyhow::bail!("Plot error: x and y must have the same length.");
    }

    // High resolution output (suitable for ~300 DPI usage when printed).
    let (w, h) = (2400u32, 1800u32);

    // Compute bounds.
    let mut xmin = f64::INFINITY;
    let mut xmax = f64::NEG_INFINITY;
    let mut ymin = f64::INFINITY;
    let mut ymax = f64::NEG_INFINITY;
    for k in 0..x.len() {
        xmin = xmin.min(x[k]);
        xmax = xmax.max(x[k]);
        ymin = ymin.min(y[k]);
        ymax = ymax.max(y[k]);
    }

    // Add a small y padding.
    let ypad = 0.05 * (ymax - ymin).abs().max(1e-9);
    ymin -= ypad;
    ymax += ypad;

    let root = BitMapBackend::new(filename, (w, h)).into_drawing_area();
    root.fill(&RGBColor(255, 255, 255))?;

    let mut chart = ChartBuilder::on(&root)
        .margin(20)
        .caption(title, ("sans-serif", 76))
        .x_label_area_size(110)
        .y_label_area_size(140)
        .build_cartesian_2d(xmin..xmax, ymin..ymax)?;

    chart
        .configure_mesh()
        .x_desc(xlabel)
        .y_desc(ylabel)
        .axis_desc_style(("sans-serif", 60))
        .label_style(("sans-serif", 44))
        .x_labels(10)
        .y_labels(10)
        .x_label_formatter(&|v| format!("{:.1}", v))
        .y_label_formatter(&|v| format!("{:.1}", v))
        .bold_line_style(RGBColor(160, 160, 160).stroke_width(2))
        .light_line_style(RGBColor(220, 220, 220).stroke_width(1))
        .draw()?;

    chart.draw_series(LineSeries::new(
        x.iter().copied().zip(y.iter().copied()),
        RGBColor(30, 90, 200).stroke_width(4),
    ))?;

    root.present()?;
    Ok(())
}

fn save_plots(
    out_dir: &Path,
    t: &[f64],
    x: &[f64],
    theta: &[f64],
    u: &[f64],
    f_eq: &[f64],
    tau_ext: &[f64],
    s_surf: &[f64],
) -> Result<()> {
    save_line_plot_png(
        &out_dir.join("cart_position.png"),
        "Cart Position x(t)",
        "time (s)",
        "x (m)",
        t,
        x,
    )?;
    save_line_plot_png(
        &out_dir.join("pole_angle.png"),
        "Pole Angle theta(t) (0=upright)",
        "time (s)",
        "theta (rad)",
        t,
        theta,
    )?;
    save_line_plot_png(
        &out_dir.join("control_force.png"),
        "Control Force u(t) (SMC)",
        "time (s)",
        "u (N)",
        t,
        u,
    )?;
    save_line_plot_png(
        &out_dir.join("disturbance_torque.png"),
        "External Disturbance Torque tau_ext(t)",
        "time (s)",
        "tau_ext (N*m)",
        t,
        tau_ext,
    )?;
    save_line_plot_png(
        &out_dir.join("equivalent_bob_force.png"),
        "Equivalent Bob Force (for arrow direction) F_eq(t) = tau/L",
        "time (s)",
        "F_eq (N)",
        t,
        f_eq,
    )?;
    save_line_plot_png(
        &out_dir.join("sliding_surface.png"),
        "Sliding Surface s(t)",
        "time (s)",
        "s",
        t,
        s_surf,
    )?;
    Ok(())
}

// ------------------------------------------------------------
// MP4 encoding via ffmpeg
// ------------------------------------------------------------
fn ffmpeg_available() -> bool {
    Command::new("ffmpeg")
        .arg("-version")
        .output()
        .is_ok()
}

fn encode_mp4_with_ffmpeg(frames_dir: &Path, fps: usize, out_mp4: &Path) -> Result<()> {
    if !ffmpeg_available() {
        eprintln!("ffmpeg not found on PATH; MP4 will not be created.");
        return Ok(());
    }

    println!("Encoding MP4...");

    // Pattern used by ffmpeg to read frames.
    let input_pattern = frames_dir.join("frame_%06d.png");

    let status = Command::new("ffmpeg")
        .arg("-y")
        .arg("-framerate")
        .arg(format!("{}", fps))
        .arg("-i")
        .arg(input_pattern.to_string_lossy().as_ref())
        .arg("-c:v")
        .arg("libx264")
        .arg("-pix_fmt")
        .arg("yuv420p")
        .arg(out_mp4.to_string_lossy().as_ref())
        .status()
        .context("Failed to run ffmpeg")?;

    if status.success() {
        println!("MP4 created: {}", out_mp4.display());
    } else {
        eprintln!("ffmpeg encoding failed.");
    }

    Ok(())
}

fn main() -> Result<()> {
    // Optional: --preview to show a window while generating frames.
    let preview = std::env::args().any(|a| a == "--preview");

    // ----------------------------
    // Output folders
    // ----------------------------
    let out_dir = PathBuf::from("output").join("pendulum_sliding_mode");
    let frames_dir = out_dir.join("frames");
    fs::create_dir_all(&frames_dir).context("Failed to create output directories")?;

    // Clean old frames.
    if frames_dir.exists() {
        for entry in fs::read_dir(&frames_dir)? {
            let p = entry?.path();
            if p.is_file() {
                let _ = fs::remove_file(p);
            }
        }
    }

    // ----------------------------
    // Simulation settings
    // ----------------------------
    const VIDEO_SECONDS: f64 = 10.0;

    // Frame count used by the simulation and output naming.
    const TOTAL_FRAMES: usize = 1800;

    let fps: usize = (TOTAL_FRAMES as f64 / VIDEO_SECONDS) as usize;
    let dt_frame: f64 = VIDEO_SECONDS / TOTAL_FRAMES as f64;

    // Physics integration step:
    // Multiple substeps per frame keep the nonlinear dynamics stable.
    const SUBSTEPS: usize = 8;
    let dt_physics: f64 = dt_frame / SUBSTEPS as f64;

    // Screen size (pixels).
    const W: u32 = 1000;
    const H: u32 = 600;

    // ----------------------------
    // Plant / controller / disturbance
    // ----------------------------
    let mut pole = PoleModel::new();
    pole.l = 1.0;
    pole.m_rod = 0.1;
    pole.m_bob = 0.15;
    pole.compute_derived();

    let plant = CartPoleParams {
        m_cart: 1.2,
        pole,
        g: 9.81,
        cart_damping: 0.10,
        pole_damping: 0.03,
    };

    let ctrl = ControlParams {
        lambda_theta: 10.0,
        lambda_x: 1.5,
        alpha: 0.55,
        k: 110.0,
        phi: 0.05,
        u_max: 70.0,
        hold_kp: 8.0,
        hold_kd: 10.0,
        theta_gate: 0.20,
    };

    let track = TrackLimits { x_max: 1.6 };

    let dist = Disturbance {
        t1: 0.5,
        t2: 5.0,
        arrow_duration: 0.5,
        duration: 0.5,
        tau_amp: 3.3,
    };

    // Initial state.
    let mut s = State {
        x: 0.0,
        xdot: 0.0,
        theta: 0.20,
        thetadot: 0.0,
    };

    // ----------------------------
    // Logging (one entry per frame)
    // ----------------------------
    let mut t_log: Vec<f64> = Vec::with_capacity(TOTAL_FRAMES);
    let mut x_log: Vec<f64> = Vec::with_capacity(TOTAL_FRAMES);
    let mut th_log: Vec<f64> = Vec::with_capacity(TOTAL_FRAMES);
    let mut u_log: Vec<f64> = Vec::with_capacity(TOTAL_FRAMES);
    let mut feq_log: Vec<f64> = Vec::with_capacity(TOTAL_FRAMES);
    let mut tau_log: Vec<f64> = Vec::with_capacity(TOTAL_FRAMES);
    let mut surf_log: Vec<f64> = Vec::with_capacity(TOTAL_FRAMES);

    // ----------------------------
    // Optional preview window (does not affect saved frames)
    // ----------------------------
    let mut window = if preview {
        Some(
            Window::new(
                "Cart-Pole SMC (10s, frames)",
                W as usize,
                H as usize,
                WindowOptions::default(),
            )
            .context("Failed to create preview window")?,
        )
    } else {
        None
    };

    if let Some(w) = window.as_mut() {
        // Keep input responsive while generating frames.
        w.set_target_fps(60);
    }

    // ----------------------------
    // Visual mapping
    // ----------------------------
    let pixels_per_meter: f32 = 180.0;
    let origin = (W as f32 * 0.5, H as f32 * 0.75);

    // Colors
    let bg = rgba(20, 20, 20, 255);
    let track_color = rgba(170, 170, 170, 255);
    let cart_color = rgba(40, 140, 255, 255);
    let pole_color = rgba(240, 70, 70, 255);
    let bob_color = rgba(255, 220, 60, 255);
    let wheel_color = rgba(70, 70, 70, 255);
    let arrow_color = rgba(60, 255, 120, 255);
    let center_mark_color = rgba(120, 120, 120, 255);

    // Track line geometry
    let track_w: i32 = (W as i32 - 100).max(1);
    let track_h: i32 = 4;
    let track_x0: i32 = (origin.0 as i32) - track_w / 2;
    let track_y0: i32 = (origin.1 as i32) + 25;

    // Center marker geometry
    let center_mark_w: i32 = 3;
    let center_mark_h: i32 = 60;
    let center_x0: i32 = origin.0 as i32 - center_mark_w / 2;
    let center_y0: i32 = origin.1 as i32 - center_mark_h / 2;

    // Cart geometry
    let cart_w: f32 = 140.0;
    let cart_h: f32 = 40.0;

    // Wheel geometry
    let wheel_r: i32 = 14;

    // Pole geometry (rendered as a rotated rectangle)
    let pole_len_px: f32 = plant.pole.l as f32 * pixels_per_meter;
    let pole_thickness: f32 = 8.0;

    // Bob geometry
    let bob_r: i32 = 9;

    // Arrow constant length (not stretched)
    let arrow_len_px: f32 = 120.0;

    // ----------------------------
    // Main loop: frame generation
    // ----------------------------
    let mut t: f64 = 0.0;

    for frame in 0..TOTAL_FRAMES {
        // Allow user to close preview window during generation.
        if let Some(w) = window.as_mut() {
            if !w.is_open() || w.is_key_down(Key::Escape) {
                window = None;
            }
        }

        // Integrate dynamics for this frame using substeps.
        let mut u_used = 0.0;
        let mut tau_used = 0.0;
        let mut feq_used = 0.0;
        let mut s_used = 0.0;

        for _ in 0..SUBSTEPS {
            let tau_ext = dist.tau_ext(t);
            let u = compute_control(&plant, &ctrl, &s);

            rk4_step(&plant, &mut s, dt_physics, u, tau_ext);

            track.enforce(&mut s);

            u_used = u;
            tau_used = tau_ext;
            feq_used = dist.bob_force_equivalent(t, plant.pole.l);
            s_used = sliding_surface(&ctrl, &s);

            t += dt_physics;
        }

        // Log once per frame.
        let tf = frame as f64 * dt_frame;
        t_log.push(tf);
        x_log.push(s.x);
        th_log.push(s.theta);
        u_log.push(u_used);
        tau_log.push(tau_used);
        feq_log.push(feq_used);
        surf_log.push(s_used);

        // ----------------------------
        // Render using current integrated state
        // ----------------------------
        let mut img: Img = ImageBuffer::from_pixel(W, H, bg);

        // Track
        draw_filled_rect_mut(
            &mut img,
            Rect::at(track_x0, track_y0).of_size(track_w as u32, track_h as u32),
            track_color,
        );

        // Center marker
        draw_filled_rect_mut(
            &mut img,
            Rect::at(center_x0, center_y0).of_size(center_mark_w as u32, center_mark_h as u32),
            center_mark_color,
        );

        // Cart position in pixels
        let cart_x_px = origin.0 + (s.x as f32) * pixels_per_meter;
        let cart_y_px = origin.1;

        // Cart rectangle
        let cart_left = (cart_x_px - cart_w * 0.5) as i32;
        let cart_top = (cart_y_px - cart_h * 0.5) as i32;
        draw_filled_rect_mut(
            &mut img,
            Rect::at(cart_left, cart_top).of_size(cart_w as u32, cart_h as u32),
            cart_color,
        );

        // Wheels
        let w1 = (cart_x_px - cart_w * 0.30, cart_y_px + cart_h * 0.55);
        let w2 = (cart_x_px + cart_w * 0.30, cart_y_px + cart_h * 0.55);
        draw_filled_circle_mut(&mut img, (w1.0 as i32, w1.1 as i32), wheel_r, wheel_color);
        draw_filled_circle_mut(&mut img, (w2.0 as i32, w2.1 as i32), wheel_r, wheel_color);

        // Pivot at top center of cart
        let pivot = (cart_x_px, cart_y_px - cart_h * 0.5);

        // Pole + bob geometry:
        // Tip position computed from the pivot and current angle.
        let theta = s.theta as f32;

        // Tip in floating screen coordinates (y increases downward in image space).
        let tip_x_f = pivot.0 + pole_len_px * theta.sin();
        let tip_y_f = pivot.1 - pole_len_px * theta.cos();

        // Use integer-rounded tip coordinates for BOTH:
        //   1) the pole endpoint
        //   2) the bob center
        //
        // This removes any visible gap caused by float->int rounding differences.
        let tip_x_i = tip_x_f.round() as i32;
        let tip_y_i = tip_y_f.round() as i32;

        // Convenience floating versions used by other drawing elements (e.g., disturbance arrow).
        let tip_x = tip_x_i as f32;
        let tip_y = tip_y_i as f32;

        // Draw the pole as a thick line from pivot to the exact bob center.
        draw_thick_line_segment(
            &mut img,
            (pivot.0, pivot.1),
            (tip_x_i as f32, tip_y_i as f32),
            pole_thickness as i32,
            pole_color,
        );

        // Draw the bob centered exactly at the pole tip.
        draw_filled_circle_mut(&mut img, (tip_x_i, tip_y_i), bob_r, bob_color);
// Disturbance arrow visible only 0.5 s after each disturbance start.
        if dist.arrow_visible(tf) {
            let dir: f32 = if feq_used >= 0.0 { 1.0 } else { -1.0 };
            let start = (tip_x, tip_y - 25.0);
            let end = (tip_x + dir * arrow_len_px, tip_y - 25.0);
            draw_force_arrow(&mut img, start, end, arrow_color);
        }

        // Save frame
        let frame_name = frames_dir.join(format!("frame_{:06}.png", frame));
        img.save(&frame_name)
            .with_context(|| format!("Failed to save {}", frame_name.display()))?;

        // Optional preview
        if let Some(w) = window.as_mut() {
            let buffer = to_minifb_buffer(&img);
            w.update_with_buffer(&buffer, W as usize, H as usize)
                .context("Failed to update preview window")?;
        }

        // Console progress
        if frame % 180 == 0 {
            println!(
                "Frame {}/{}  t={:.2}  x={:.3}  theta={:.3}  u={:.2}  tau={:.3}",
                frame,
                TOTAL_FRAMES,
                tf,
                s.x,
                s.theta,
                u_used,
                tau_used
            );
        }
    }

    // Encode MP4
    let mp4 = out_dir.join("pendulum_smc_10s_1800f.mp4");
    encode_mp4_with_ffmpeg(&frames_dir, fps, &mp4)?;

    // Save plots + CSV
    println!("Saving plots and CSV...");
    save_plots(
        &out_dir,
        &t_log,
        &x_log,
        &th_log,
        &u_log,
        &feq_log,
        &tau_log,
        &surf_log,
    )?;

    write_csv(
        &out_dir.join("cartpole_log.csv"),
        &["t", "x", "theta", "u", "F_equiv", "tau_ext", "s"],
        &[
            t_log.clone(),
            x_log.clone(),
            th_log.clone(),
            u_log.clone(),
            feq_log.clone(),
            tau_log.clone(),
            surf_log.clone(),
        ],
    )?;

    println!("Done. Results are in: {}", out_dir.display());
    Ok(())
}
