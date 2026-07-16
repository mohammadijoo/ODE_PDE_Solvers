# frozen_string_literal: true

# ------------------------------------------------------------
# Cart–Pole Inverted Pendulum (moving base) with SLIDING MODE CONTROL (SMC)
# ------------------------------------------------------------
# Requirements implemented:
#   - Total simulation: 10 seconds
#   - Frame rendering to PNG files (offscreen)
#   - Exactly two disturbances: at t=0.5s and t=5.0s
#   - Disturbance shown as a constant-length arrow for 0.5s only (direction indicator)
#   - Disturbance direction and cart compensation direction are consistent:
#       "push right" => pole tips right => cart accelerates right to recover
#   - Dynamics are integrated and used to render each frame
#   - Outputs:
#       * PNG frames
#       * MP4 (optional, if ffmpeg is on PATH)
#       * High-quality plots (gnuplot)
#       * CSV log
#
# Output folders:
#   output/pendulum_sliding_mode/frames/frame_000000.png ... frame_001799.png
#   output/pendulum_sliding_mode/pendulum_smc_10s_6000f.mp4   (if ffmpeg in PATH)
#   output/pendulum_sliding_mode/*.png plots                  (if gnuplot in PATH)
#   output/pendulum_sliding_mode/cartpole_log.csv
# ------------------------------------------------------------

require "fileutils"
require "chunky_png"
require_relative "../../lib/common/math_utils"
require_relative "../../lib/common/csv_writer"
require_relative "../../lib/common/gnuplot_helpers"

include MathUtils

# ------------------------------------------------------------
# Pole model: uniform rod + lumped bob at the top
# ------------------------------------------------------------
class PoleModel
  attr_accessor :l, :m_rod, :m_bob
  attr_reader :m_total, :l_com, :i_pivot, :i_com, :inertia_factor

  def initialize
    @l = 1.0
    @m_rod = 0.06
    @m_bob = 0.04

    @m_total = 0.0
    @l_com = 0.0
    @i_pivot = 0.0
    @i_com = 0.0
    @inertia_factor = 0.0
  end

  def compute_derived!
    @m_total = @m_rod + @m_bob

    # Center of mass from pivot: rod at L/2, bob at L.
    @l_com = (@m_rod * (@l * 0.5) + @m_bob * @l) / @m_total

    # Inertia about pivot:
    # rod: (1/3)mL^2, bob: mL^2.
    @i_pivot = (1.0 / 3.0) * @m_rod * @l * @l + @m_bob * @l * @l

    # Inertia about COM.
    @i_com = @i_pivot - @m_total * @l_com * @l_com

    # Convenience factor used in the generalized dynamics.
    @inertia_factor = 1.0 + @i_com / (@m_total * @l_com * @l_com)
  end
end

# ------------------------------------------------------------
# Plant parameters
# ------------------------------------------------------------
class CartPoleParams
  attr_accessor :m_cart, :g, :cart_damping, :pole_damping, :pole

  def initialize
    @m_cart = 1.2
    @pole = PoleModel.new
    @g = 9.81

    # Damping for realism and numerical stability.
    @cart_damping = 0.10
    @pole_damping = 0.03
  end
end

# ------------------------------------------------------------
# State (theta = 0 upright)
# Rendering convention: tip_x = x + L*sin(theta), so theta > 0 leans to the right.
# ------------------------------------------------------------
State = Struct.new(:x, :xdot, :theta, :thetadot)

# ------------------------------------------------------------
# Sliding Mode Control parameters + a cart-centering term
# ------------------------------------------------------------
class ControlParams
  attr_accessor :lambda_theta, :lambda_x, :alpha, :k, :phi, :u_max, :hold_kp, :hold_kd, :theta_gate

  def initialize
    @lambda_theta = 10.0
    @lambda_x = 1.5
    @alpha = 0.55

    @k = 110.0
    @phi = 0.05

    @u_max = 70.0

    @hold_kp = 8.0
    @hold_kd = 10.0

    @theta_gate = 0.20
  end
end

# ------------------------------------------------------------
# Track limits: keep cart in view.
# ------------------------------------------------------------
class TrackLimits
  attr_accessor :x_max

  def initialize
    @x_max = 1.6
  end

  def enforce!(s)
    if s.x > @x_max
      s.x = @x_max
      s.xdot = 0.0 if s.xdot > 0.0
    end
    if s.x < -@x_max
      s.x = -@x_max
      s.xdot = 0.0 if s.xdot < 0.0
    end
  end
end

# ------------------------------------------------------------
# Disturbance schedule:
#   - exactly two torque pulses: t=0.5s and t=5.0s
#   - arrow shown for 0.5s after each pulse start (direction only)
#
# Disturbance is applied as an external torque about the pole pivot.
# Direction convention:
#   - "push right" => pole tends to lean right (theta increases) => positive torque
# ------------------------------------------------------------
class Disturbance
  attr_accessor :t1, :t2, :arrow_duration, :duration, :tau_amp

  def initialize
    @t1 = 0.5
    @t2 = 5.0

    @arrow_duration = 0.5
    @duration = 0.5

    @tau_amp = 3.3
  end

  def half_sine(local_t, dur)
    Math.sin(Math::PI * local_t / dur)
  end

  def tau_ext(t)
    if t >= @t1 && t <= (@t1 + @duration)
      local = t - @t1
      return +@tau_amp * half_sine(local, @duration)
    end
    if t >= @t2 && t <= (@t2 + @duration)
      local = t - @t2
      return -@tau_amp * half_sine(local, @duration)
    end
    0.0
  end

  # Equivalent force at the bob for arrow direction only:
  #   F_eq = tau / L
  def bob_force_equivalent(t, pole_len)
    tau_ext(t) / pole_len
  end

  def arrow_visible?(t)
    return true if t >= @t1 && t <= (@t1 + @arrow_duration)
    return true if t >= @t2 && t <= (@t2 + @arrow_duration)
    false
  end
end

# ------------------------------------------------------------
# Derivatives for the ODE system
# ------------------------------------------------------------
Deriv = Struct.new(:x_dot, :x_ddot, :theta_dot, :theta_ddot)

# ------------------------------------------------------------
# Nonlinear cart–pole dynamics + external torque tau_ext.
# Adds damping and disturbance torque.
# ------------------------------------------------------------
def dynamics(plant, s, u_cart, tau_ext)
  m = plant.pole.m_total
  l = plant.pole.l_com

  total_mass = plant.m_cart + m
  polemass_length = m * l

  sin_t = Math.sin(s.theta)
  cos_t = Math.cos(s.theta)

  # Cart friction-like damping.
  f_damped = u_cart - plant.cart_damping * s.xdot

  temp = (f_damped + polemass_length * s.thetadot * s.thetadot * sin_t) / total_mass

  inertia_factor = plant.pole.inertia_factor
  denom = l * (inertia_factor - (m * cos_t * cos_t) / total_mass)

  theta_ddot = (plant.g * sin_t - cos_t * temp) / denom

  # Pole damping.
  theta_ddot -= plant.pole_damping * s.thetadot

  # External torque about pivot (disturbance).
  theta_ddot += tau_ext / plant.pole.i_pivot

  x_ddot = temp - polemass_length * theta_ddot * cos_t / total_mass

  Deriv.new(s.xdot, x_ddot, s.thetadot, theta_ddot)
end

# ------------------------------------------------------------
# RK4 integration step
# ------------------------------------------------------------
def rk4_step!(plant, s, dt, u_cart, tau_ext)
  add_scaled = lambda do |st, k, h|
    State.new(
      st.x + h * k.x_dot,
      st.xdot + h * k.x_ddot,
      st.theta + h * k.theta_dot,
      st.thetadot + h * k.theta_ddot
    )
  end

  k1 = dynamics(plant, s, u_cart, tau_ext)
  k2 = dynamics(plant, add_scaled.call(s, k1, 0.5 * dt), u_cart, tau_ext)
  k3 = dynamics(plant, add_scaled.call(s, k2, 0.5 * dt), u_cart, tau_ext)
  k4 = dynamics(plant, add_scaled.call(s, k3, dt), u_cart, tau_ext)

  s.x        += (dt / 6.0) * (k1.x_dot      + 2.0 * k2.x_dot      + 2.0 * k3.x_dot      + k4.x_dot)
  s.xdot     += (dt / 6.0) * (k1.x_ddot     + 2.0 * k2.x_ddot     + 2.0 * k3.x_ddot     + k4.x_ddot)
  s.theta    += (dt / 6.0) * (k1.theta_dot  + 2.0 * k2.theta_dot  + 2.0 * k3.theta_dot  + k4.theta_dot)
  s.thetadot += (dt / 6.0) * (k1.theta_ddot + 2.0 * k2.theta_ddot + 2.0 * k3.theta_ddot + k4.theta_ddot)

  s.theta = wrap_to_pi(s.theta)
end

# ------------------------------------------------------------
# Sliding surface and SMC control
# ------------------------------------------------------------
def sliding_surface(ctrl, s)
  s.thetadot +
    ctrl.lambda_theta * s.theta +
    ctrl.alpha * (s.xdot + ctrl.lambda_x * s.x)
end

def sliding_surface_dot_nominal(plant, ctrl, s, u_cart)
  # Nominal control design ignores external torque.
  d = dynamics(plant, s, u_cart, 0.0)

  d.theta_ddot +
    ctrl.lambda_theta * s.thetadot +
    ctrl.alpha * (d.x_ddot + ctrl.lambda_x * s.xdot)
end

def compute_control(plant, ctrl, s)
  # Sliding Mode Control part.
  sval = sliding_surface(ctrl, s)
  desired_sdot = -ctrl.k * sat(sval / ctrl.phi)

  # Local affine approximation: sdot(u) ≈ a*u + b.
  sdot0 = sliding_surface_dot_nominal(plant, ctrl, s, 0.0)
  sdot1 = sliding_surface_dot_nominal(plant, ctrl, s, 1.0)
  a = (sdot1 - sdot0)
  b = sdot0

  u_smc =
    if a.abs < 1e-8
      0.0
    else
      (desired_sdot - b) / a
    end

  # Cart-centering term (gated):
  # become active only when the pole is near upright.
  theta_abs = s.theta.abs
  gate = clamp(1.0 - (theta_abs / ctrl.theta_gate), 0.0, 1.0)

  u_hold = gate * (-ctrl.hold_kp * s.x - ctrl.hold_kd * s.xdot)

  u_total = u_smc + u_hold

  clamp(u_total, -ctrl.u_max, ctrl.u_max)
end

# ------------------------------------------------------------
# Rendering utilities (ChunkyPNG)
# ------------------------------------------------------------
module Render2D
  module_function

  def clamp_i(v, lo, hi)
    return lo if v < lo
    return hi if v > hi
    v
  end

  # Filled rectangle.
  def fill_rect(img, x0, y0, w, h, color)
    x1 = x0 + w - 1
    y1 = y0 + h - 1

    x0 = clamp_i(x0, 0, img.width - 1)
    y0 = clamp_i(y0, 0, img.height - 1)
    x1 = clamp_i(x1, 0, img.width - 1)
    y1 = clamp_i(y1, 0, img.height - 1)

    (y0..y1).each do |y|
      (x0..x1).each do |x|
        img[x, y] = color
      end
    end
  end

  # Filled circle.
  def fill_circle(img, cx, cy, r, color)
    r2 = r * r
    y0 = clamp_i(cy - r, 0, img.height - 1)
    y1 = clamp_i(cy + r, 0, img.height - 1)
    x0 = clamp_i(cx - r, 0, img.width - 1)
    x1 = clamp_i(cx + r, 0, img.width - 1)

    (y0..y1).each do |y|
      dy = y - cy
      (x0..x1).each do |x|
        dx = x - cx
        img[x, y] = color if (dx * dx + dy * dy) <= r2
      end
    end
  end

  # Scanline fill for a convex polygon.
  def fill_polygon(img, pts, color)
    ys = pts.map { |p| p[1] }
    y_min = ys.min.floor
    y_max = ys.max.ceil

    y_min = clamp_i(y_min, 0, img.height - 1)
    y_max = clamp_i(y_max, 0, img.height - 1)

    (y_min..y_max).each do |y|
      # Find intersections with polygon edges at this scanline.
      xints = []
      pts.each_with_index do |p1, i|
        p2 = pts[(i + 1) % pts.length]
        y1, y2 = p1[1], p2[1]
        x1, x2 = p1[0], p2[0]

        next if (y1 - y2).abs < 1e-9

        # Check if scanline crosses the edge (half-open to avoid double counting vertices).
        if (y >= [y1, y2].min) && (y < [y1, y2].max)
          t = (y - y1) / (y2 - y1).to_f
          x = x1 + t * (x2 - x1)
          xints << x
        end
      end

      next if xints.length < 2

      xints.sort!
      (0...(xints.length / 2)).each do |k|
        xa = xints[2 * k].floor
        xb = xints[2 * k + 1].ceil

        xa = clamp_i(xa, 0, img.width - 1)
        xb = clamp_i(xb, 0, img.width - 1)

        (xa..xb).each do |x|
          img[x, y] = color
        end
      end
    end
  end

  # Draw a thick line by rendering a rectangle polygon around the segment.
  def thick_segment(img, x0, y0, x1, y1, thickness, color)
    dx = x1 - x0
    dy = y1 - y0
    len = Math.sqrt(dx * dx + dy * dy)
    return if len < 1e-6

    ux = dx / len
    uy = dy / len

    # Perpendicular direction.
    px = -uy
    py = ux

    half = thickness * 0.5

    p1 = [x0 + px * half, y0 + py * half]
    p2 = [x0 - px * half, y0 - py * half]
    p3 = [x1 - px * half, y1 - py * half]
    p4 = [x1 + px * half, y1 + py * half]

    fill_polygon(img, [p1, p2, p3, p4], color)
  end

  # Draw an arrow of constant length (not scaled by force magnitude).
  def draw_arrow(img, start_x, start_y, end_x, end_y, color, thickness: 4, head_len: 16, head_w: 10)
    thick_segment(img, start_x, start_y, end_x, end_y, thickness, color)

    dx = end_x - start_x
    dy = end_y - start_y
    len = Math.sqrt(dx * dx + dy * dy)
    return if len < 1e-6

    ux = dx / len
    uy = dy / len

    # Perpendicular.
    px = -uy
    py = ux

    tip = [end_x, end_y]
    base = [end_x - ux * head_len, end_y - uy * head_len]

    left  = [base[0] + px * head_w, base[1] + py * head_w]
    right = [base[0] - px * head_w, base[1] - py * head_w]

    fill_polygon(img, [tip, left, right], color)
  end
end

# ------------------------------------------------------------
# Plot saving (Gnuplot)
# ------------------------------------------------------------
def save_plots(out_dir, t, x, theta, u, f_eq, tau_ext, s_surf)
  plots = [
    ["cart_position.png", t, x, "Cart Position x(t)", "time (s)", "x (m)"],
    ["pole_angle.png", t, theta, "Pole Angle theta(t) (0=upright)", "time (s)", "theta (rad)"],
    ["control_force.png", t, u, "Control Force u(t) (SMC)", "time (s)", "u (N)"],
    ["disturbance_torque.png", t, tau_ext, "External Disturbance Torque tau_ext(t)", "time (s)", "tau_ext (N*m)"],
    ["equivalent_bob_force.png", t, f_eq, "Equivalent Bob Force F_eq(t) = tau/L", "time (s)", "F_eq (N)"],
    ["sliding_surface.png", t, s_surf, "Sliding Surface s(t)", "time (s)", "s"]
  ]

  plots.each do |fname, xs, ys, title_text, xlabel_text, ylabel_text|
    out_png = File.join(out_dir, fname)
    dat = File.join(out_dir, "tmp_plot_data.dat")

    File.open(dat, "w") do |f|
      xs.length.times { |k| f.puts "#{xs[k]} #{ys[k]}" }
    end

    xmin = xs.min
    xmax = xs.max
    ymin = ys.min
    ymax = ys.max

    script = <<~GP
      #{GnuplotHelpers.common_png_header(out_png, width_px: 2400, height_px: 1600, font: "Arial,28", line_width: 3)}

      set title "#{title_text}" font "Arial,38"
      set xlabel "#{xlabel_text}" font "Arial,34"
      set ylabel "#{ylabel_text}" font "Arial,34"

      set xtics #{GnuplotHelpers.nice_tick_step(xmin, xmax, 10)}
      set ytics #{GnuplotHelpers.nice_tick_step(ymin, ymax, 10)}

      plot "#{GnuplotHelpers.gp_escape(dat)}" using 1:2 with lines linestyle 1
    GP

    GnuplotHelpers.run_gnuplot!(script, out_dir: out_dir, script_name: "plot_#{fname}.gp")
    FileUtils.rm_f(dat)
  end
end

# ------------------------------------------------------------
# MP4 encoding via ffmpeg (optional)
# ------------------------------------------------------------
def encode_mp4_with_ffmpeg(frames_dir, fps, out_mp4)
  null = Gem.win_platform? ? "NUL" : "/dev/null"

  # Check if ffmpeg is available.
  ok = system("ffmpeg -version >#{null} 2>&1")
  unless ok
    warn "ffmpeg not found on PATH; MP4 will not be created."
    return
  end

  cmd = %Q(ffmpeg -y -framerate #{fps} -i "#{frames_dir}/frame_%06d.png" -c:v libx264 -pix_fmt yuv420p "#{out_mp4}" >#{null} 2>&1)
  puts "Encoding MP4..."
  rc = system(cmd)
  if rc
    puts "MP4 created: #{out_mp4}"
  else
    warn "ffmpeg encoding failed."
  end
end

# ------------------------------------------------------------
# Main program
# ------------------------------------------------------------

# Optional flag parsing (currently the renderer is always offscreen).
preview = ARGV.include?("--preview")
puts "Preview mode requested, but this implementation renders offscreen only." if preview

out_dir = "output/pendulum_sliding_mode"
frames_dir = File.join(out_dir, "frames")
FileUtils.mkdir_p(frames_dir)

# Clean old frames.
Dir.glob(File.join(frames_dir, "*.png")).each { |f| FileUtils.rm_f(f) }

# Simulation settings.
video_seconds = 10.0

# Total rendered frames:
#   10 seconds × 600 FPS = 6000 frames
total_frames = 6000
fps = (total_frames / video_seconds).to_i

dt_frame = video_seconds / total_frames.to_f

# Use multiple substeps per frame for stable dynamics.
substeps = 8
dt_physics = dt_frame / substeps.to_f

# Screen.
w = 1000
h = 600

# Plant / controller / disturbance.
plant = CartPoleParams.new
plant.m_cart = 1.2
plant.pole.l = 1.0
plant.pole.m_rod = 0.1
plant.pole.m_bob = 0.15
plant.pole.compute_derived!

ctrl = ControlParams.new
track = TrackLimits.new
dist = Disturbance.new

# Initial state.
s = State.new(0.0, 0.0, 0.20, 0.0)

# Logging (one entry per frame).

# Visual mapping.
pixels_per_meter = 180.0
origin_x = w * 0.5
origin_y = h * 0.75

# Colors (ARGB).
bg          = ChunkyPNG::Color.rgba(20, 20, 20, 255)
track_color = ChunkyPNG::Color.rgba(170, 170, 170, 255)
cart_color  = ChunkyPNG::Color.rgba(40, 140, 255, 255)
pole_color  = ChunkyPNG::Color.rgba(240, 70, 70, 255)
bob_color   = ChunkyPNG::Color.rgba(255, 220, 60, 255)
wheel_color = ChunkyPNG::Color.rgba(70, 70, 70, 255)
arrow_color = ChunkyPNG::Color.rgba(60, 255, 120, 255)
center_mark = ChunkyPNG::Color.rgba(120, 120, 120, 255)

cart_w = 140
cart_h = 40
wheel_r = 14
pole_len_px = (plant.pole.l * pixels_per_meter).to_i
pole_thickness = 8
arrow_len_px = 120

# Main loop.
t = 0.0

# Logging (one entry per frame).
# These arrays are later saved to CSV and used for generating plots.
t_log   = []
x_log   = []
th_log  = []
u_log   = []
tau_log = []
feq_log = []
surf_log = []

(0...total_frames).each do |frame|
  u_used = 0.0
  tau_used = 0.0
  feq_used = 0.0
  s_used = 0.0

  # Integrate physics for this frame.
  substeps.times do
    tau_ext = dist.tau_ext(t)
    u = compute_control(plant, ctrl, s)

    rk4_step!(plant, s, dt_physics, u, tau_ext)
    track.enforce!(s)

    u_used = u
    tau_used = tau_ext
    feq_used = dist.bob_force_equivalent(t, plant.pole.l)
    s_used = sliding_surface(ctrl, s)

    t += dt_physics
  end

  tf = frame * dt_frame

  t_log << tf
  x_log << s.x
  th_log << s.theta
  u_log << u_used
  tau_log << tau_used
  feq_log << feq_used
  surf_log << s_used

  # Render current state to a PNG.
  img = ChunkyPNG::Image.new(w, h, bg)

  # Track line.
  track_x0 = 50
  track_x1 = w - 50
  track_y = (origin_y + 25).to_i
  Render2D.thick_segment(img, track_x0, track_y, track_x1, track_y, 4, track_color)

  # Center marker.
  Render2D.fill_rect(img, (origin_x - 1).to_i, (origin_y - 30).to_i, 3, 60, center_mark)

  # Cart.
  cart_x_px = (origin_x + s.x * pixels_per_meter).to_i
  cart_y_px = origin_y.to_i
  Render2D.fill_rect(img, cart_x_px - cart_w / 2, cart_y_px - cart_h / 2, cart_w, cart_h, cart_color)

  # Wheels.
  w1x = (cart_x_px - cart_w * 0.30).to_i
  w2x = (cart_x_px + cart_w * 0.30).to_i
  wy = (cart_y_px + cart_h * 0.55).to_i
  Render2D.fill_circle(img, w1x, wy, wheel_r, wheel_color)
  Render2D.fill_circle(img, w2x, wy, wheel_r, wheel_color)

  # Pivot at the top center of cart.
  pivot_x = cart_x_px.to_f
  pivot_y = (cart_y_px - cart_h / 2).to_f

  # Tip of pole.
  tip_x = pivot_x + pole_len_px * Math.sin(s.theta)
  tip_y = pivot_y - pole_len_px * Math.cos(s.theta)

  # Pole as a thick segment.
  Render2D.thick_segment(img, pivot_x, pivot_y, tip_x, tip_y, pole_thickness, pole_color)

  # Bob.
  Render2D.fill_circle(img, tip_x.to_i, tip_y.to_i, 9, bob_color)

  # Disturbance arrow shown for 0.5s after each disturbance start.
  if dist.arrow_visible?(tf)
    dir = (feq_used >= 0.0) ? 1.0 : -1.0
    start_x = tip_x
    start_y = tip_y - 25.0
    end_x = tip_x + dir * arrow_len_px
    end_y = tip_y - 25.0

    Render2D.draw_arrow(img, start_x, start_y, end_x, end_y, arrow_color, thickness: 4, head_len: 18, head_w: 10)
  end

  # Save frame.
  fn = File.join(frames_dir, format("frame_%06d.png", frame))
  img.save(fn)

  # Console progress (every 600 frames).
  if (frame % 600).zero?
    puts format(
      "Frame %d/%d  t=%.2f  x=%.3f  theta=%.3f  u=%.2f  tau=%.3f",
      frame, total_frames, tf, s.x, s.theta, u_used, tau_used
    )
  end
end

# Encode MP4.
mp4 = File.join(out_dir, "pendulum_smc_10s_6000f.mp4")
encode_mp4_with_ffmpeg(frames_dir, fps, mp4)

# Save plots + CSV.
puts "Saving plots and CSV..."
save_plots(out_dir, t_log, x_log, th_log, u_log, feq_log, tau_log, surf_log)

CsvWriter.write_csv(
  File.join(out_dir, "cartpole_log.csv"),
  ["t", "x", "theta", "u", "F_equiv", "tau_ext", "s"],
  [t_log, x_log, th_log, u_log, feq_log, tau_log, surf_log]
)

puts "Done."
