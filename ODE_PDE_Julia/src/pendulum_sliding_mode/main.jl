# ------------------------------------------------------------
# Cart–Pole Inverted Pendulum (moving base) with SLIDING MODE CONTROL (SMC)
# ------------------------------------------------------------
# Requirements:
#   - Total simulation: 10 seconds
#   - Total frames: 6000 (=> 600 FPS video)
#   - Exactly two disturbances: at t=0.5s and t=5.0s
#   - Disturbance shown as an arrow for 0.5s only (constant length)
#   - Dynamics are integrated and used to render each frame
#   - Outputs: frames, mp4 (ffmpeg if available), plots, CSV log
#
# Output folders:
#   output/pendulum_sliding_mode/frames/frame_000000.png ... frame_005999.png
#   output/pendulum_sliding_mode/pendulum_smc_10s_6000f.mp4   (if ffmpeg in PATH)
#   output/pendulum_sliding_mode/*.png plots
#   output/pendulum_sliding_mode/cartpole_log.csv
# ------------------------------------------------------------

ENV["GKSwstype"] = "100"

using Printf
using Plots
import Plots: px
import Luxor

gr()

# ------------------------------------------------------------
# Plot styling (high quality static plots)
# ------------------------------------------------------------
default(
    dpi = 300,
    size = (1100, 700),
    linewidth = 3,
    titlefontsize = 20,
    guidefontsize = 18,
    tickfontsize = 14,
    legendfontsize = 14,
    left_margin = 20px,
    right_margin = 20px,
    top_margin = 20px,
    bottom_margin = 20px
)

# ------------------------------------------------------------
# Helpers: 1-decimal tick labels and ≤ 10 ticks
# ------------------------------------------------------------
fmt1(x::Real) = @sprintf("%.1f", float(x))

function ticks_1dp(minv::Real, maxv::Real; n::Int = 10)
    a = float(minv)
    b = float(maxv)

    if !isfinite(a) || !isfinite(b)
        v = [0.0]
        return (v, [fmt1(x) for x in v])
    end

    if a == b
        v = [a]
        return (v, [fmt1(x) for x in v])
    end

    v = collect(range(a, b; length = n))
    return (v, [fmt1(x) for x in v])
end

# ------------------------------------------------------------
# Small math helpers
# ------------------------------------------------------------
clamp(x, lo, hi) = x < lo ? lo : (x > hi ? hi : x)

function wrap_to_pi(a)
    while a > pi
        a -= 2.0 * pi
    end
    while a < -pi
        a += 2.0 * pi
    end
    return a
end

sat(z) = clamp(z, -1.0, 1.0)

# ------------------------------------------------------------
# CSV writer
# ------------------------------------------------------------
function write_csv(filename::AbstractString,
                   header::Vector{String},
                   cols::Vector{Vector{Float64}})
    if isempty(cols)
        error("CSV: no columns.")
    end
    n = length(cols[1])
    for c in cols
        if length(c) != n
            error("CSV: column size mismatch.")
        end
    end

    open(filename, "w") do io
        for i in 1:length(header)
            print(io, header[i])
            if i < length(header)
                print(io, ",")
            end
        end
        print(io, "\n")

        for r in 1:n
            for c in 1:length(cols)
                print(io, cols[c][r])
                if c < length(cols)
                    print(io, ",")
                end
            end
            print(io, "\n")
        end
    end
end

# ------------------------------------------------------------
# Pole model: uniform rod + lumped bob at the top
# ------------------------------------------------------------
mutable struct PoleModel
    L::Float64
    m_rod::Float64
    m_bob::Float64
    m_total::Float64
    l_com::Float64
    I_pivot::Float64
    I_com::Float64
    inertia_factor::Float64
end

function PoleModel(; L=1.0, m_rod=0.10, m_bob=0.15)
    pm = PoleModel(L, m_rod, m_bob, 0.0, 0.0, 0.0, 0.0, 0.0)
    compute_derived!(pm)
    return pm
end

function compute_derived!(pm::PoleModel)
    pm.m_total = pm.m_rod + pm.m_bob
    pm.l_com = (pm.m_rod * (pm.L * 0.5) + pm.m_bob * pm.L) / pm.m_total
    pm.I_pivot = (1.0 / 3.0) * pm.m_rod * pm.L^2 + pm.m_bob * pm.L^2
    pm.I_com = pm.I_pivot - pm.m_total * pm.l_com^2
    pm.inertia_factor = 1.0 + pm.I_com / (pm.m_total * pm.l_com^2)
    return pm
end

# ------------------------------------------------------------
# Plant parameters
# ------------------------------------------------------------
mutable struct CartPoleParams
    M::Float64
    pole::PoleModel
    g::Float64
    cart_damping::Float64
    pole_damping::Float64
end

function CartPoleParams(; M=1.2, pole=PoleModel(), g=9.81,
                        cart_damping=0.10, pole_damping=0.03)
    return CartPoleParams(M, pole, g, cart_damping, pole_damping)
end

# ------------------------------------------------------------
# State (theta=0 upright)
# ------------------------------------------------------------
mutable struct State
    x::Float64
    xdot::Float64
    theta::Float64
    thetadot::Float64
end

function State(; x=0.0, xdot=0.0, theta=0.20, thetadot=0.0)
    return State(x, xdot, theta, thetadot)
end

# ------------------------------------------------------------
# Sliding Mode Control parameters + gated cart-centering term
# ------------------------------------------------------------
struct ControlParams
    lambda_theta::Float64
    lambda_x::Float64
    alpha::Float64
    k::Float64
    phi::Float64
    u_max::Float64
    hold_kp::Float64
    hold_kd::Float64
    theta_gate::Float64
end

function ControlParams(; lambda_theta=10.0, lambda_x=1.5, alpha=0.55,
                       k=110.0, phi=0.05, u_max=70.0,
                       hold_kp=8.0, hold_kd=10.0, theta_gate=0.20)
    return ControlParams(lambda_theta, lambda_x, alpha,
                         k, phi, u_max,
                         hold_kp, hold_kd, theta_gate)
end

# ------------------------------------------------------------
# Track limit: keep the cart in view
# ------------------------------------------------------------
struct TrackLimits
    x_max::Float64
end

function enforce!(lim::TrackLimits, s::State)
    if s.x > lim.x_max
        s.x = lim.x_max
        if s.xdot > 0.0
            s.xdot = 0.0
        end
    elseif s.x < -lim.x_max
        s.x = -lim.x_max
        if s.xdot < 0.0
            s.xdot = 0.0
        end
    end
    return s
end

# ------------------------------------------------------------
# Disturbance schedule (exactly two disturbances)
# ------------------------------------------------------------
struct Disturbance
    t1::Float64
    t2::Float64
    arrow_duration::Float64
    duration::Float64
    tau_amp::Float64
end

function Disturbance(; t1=0.5, t2=5.0, arrow_duration=0.5, duration=0.5, tau_amp=3.3)
    return Disturbance(t1, t2, arrow_duration, duration, tau_amp)
end

half_sine(local_t, duration) = sin(pi * local_t / duration)

function tau_ext(d::Disturbance, t::Float64)
    if t >= d.t1 && t <= d.t1 + d.duration
        tloc = t - d.t1
        return +d.tau_amp * half_sine(tloc, d.duration)
    end
    if t >= d.t2 && t <= d.t2 + d.duration
        tloc = t - d.t2
        return -d.tau_amp * half_sine(tloc, d.duration)
    end
    return 0.0
end

bob_force_equivalent(d::Disturbance, t::Float64, L::Float64) = tau_ext(d, t) / L

function arrow_visible(d::Disturbance, t::Float64)
    return (t >= d.t1 && t <= d.t1 + d.arrow_duration) ||
           (t >= d.t2 && t <= d.t2 + d.arrow_duration)
end

# ------------------------------------------------------------
# ODE derivative
# ------------------------------------------------------------
struct Deriv
    x_dot::Float64
    x_ddot::Float64
    theta_dot::Float64
    theta_ddot::Float64
end

# ------------------------------------------------------------
# Nonlinear cart–pole dynamics + external torque
# ------------------------------------------------------------
function dynamics(p::CartPoleParams, s::State, u_cart::Float64, tau_external::Float64)
    m = p.pole.m_total
    l = p.pole.l_com

    total_mass = p.M + m
    polemass_length = m * l

    sin_t = sin(s.theta)
    cos_t = cos(s.theta)

    F_damped = u_cart - p.cart_damping * s.xdot
    temp = (F_damped + polemass_length * s.thetadot^2 * sin_t) / total_mass

    denom = l * (p.pole.inertia_factor - (m * cos_t^2) / total_mass)

    theta_ddot = (p.g * sin_t - cos_t * temp) / denom
    theta_ddot -= p.pole_damping * s.thetadot
    theta_ddot += tau_external / p.pole.I_pivot

    x_ddot = temp - polemass_length * theta_ddot * cos_t / total_mass

    return Deriv(s.xdot, x_ddot, s.thetadot, theta_ddot)
end

# ------------------------------------------------------------
# RK4 integration step
# ------------------------------------------------------------
function rk4_step!(p::CartPoleParams, s::State, dt::Float64, u_cart::Float64, tau_external::Float64)
    function add_scaled(a::State, k::Deriv, h::Float64)
        return State(
            x        = a.x        + h * k.x_dot,
            xdot     = a.xdot     + h * k.x_ddot,
            theta    = a.theta    + h * k.theta_dot,
            thetadot = a.thetadot + h * k.theta_ddot
        )
    end

    k1 = dynamics(p, s, u_cart, tau_external)
    k2 = dynamics(p, add_scaled(s, k1, 0.5 * dt), u_cart, tau_external)
    k3 = dynamics(p, add_scaled(s, k2, 0.5 * dt), u_cart, tau_external)
    k4 = dynamics(p, add_scaled(s, k3, dt),       u_cart, tau_external)

    s.x        += (dt / 6.0) * (k1.x_dot      + 2.0*k2.x_dot      + 2.0*k3.x_dot      + k4.x_dot)
    s.xdot     += (dt / 6.0) * (k1.x_ddot     + 2.0*k2.x_ddot     + 2.0*k3.x_ddot     + k4.x_ddot)
    s.theta    += (dt / 6.0) * (k1.theta_dot  + 2.0*k2.theta_dot  + 2.0*k3.theta_dot  + k4.theta_dot)
    s.thetadot += (dt / 6.0) * (k1.theta_ddot + 2.0*k2.theta_ddot + 2.0*k3.theta_ddot + k4.theta_ddot)

    s.theta = wrap_to_pi(s.theta)
    return s
end

# ------------------------------------------------------------
# Sliding surface and SMC control
# ------------------------------------------------------------
function sliding_surface(c::ControlParams, s::State)
    return s.thetadot +
           c.lambda_theta * s.theta +
           c.alpha * (s.xdot + c.lambda_x * s.x)
end

function sliding_surface_dot_nominal(p::CartPoleParams, c::ControlParams, s::State, u_cart::Float64)
    d = dynamics(p, s, u_cart, 0.0)
    return d.theta_ddot +
           c.lambda_theta * s.thetadot +
           c.alpha * (d.x_ddot + c.lambda_x * s.xdot)
end

function compute_control(p::CartPoleParams, c::ControlParams, s::State)
    sval = sliding_surface(c, s)
    desired_sdot = -c.k * sat(sval / c.phi)

    sdot0 = sliding_surface_dot_nominal(p, c, s, 0.0)
    sdot1 = sliding_surface_dot_nominal(p, c, s, 1.0)
    a = (sdot1 - sdot0)
    b = sdot0

    u_smc = abs(a) < 1e-8 ? 0.0 : (desired_sdot - b) / a

    theta_abs = abs(s.theta)
    gate = clamp(1.0 - (theta_abs / c.theta_gate), 0.0, 1.0)
    u_hold = gate * (-c.hold_kp * s.x - c.hold_kd * s.xdot)

    u_total = u_smc + u_hold
    return clamp(u_total, -c.u_max, c.u_max)
end

# ------------------------------------------------------------
# Arrow drawing (constant length)
# ------------------------------------------------------------
function draw_force_arrow(start::Luxor.Point, dir::Float64, arrow_len::Float64;
                          head_len::Float64=14.0, head_w::Float64=7.0)
    endpoint = Luxor.Point(start.x + dir * arrow_len, start.y)

    Luxor.line(start, endpoint, :stroke)

    u = Luxor.Point(dir, 0.0)
    n = Luxor.Point(0.0, 1.0)

    tip = endpoint
    base = Luxor.Point(endpoint.x - u.x * head_len, endpoint.y - u.y * head_len)

    p1 = Luxor.Point(base.x + n.x * head_w, base.y + n.y * head_w)
    p2 = Luxor.Point(base.x - n.x * head_w, base.y - n.y * head_w)

    Luxor.poly([tip, p1, p2], :fill)
end

# ------------------------------------------------------------
# Plot saving (static PNGs, 1-decimal ticks)
# ------------------------------------------------------------
function save_plots(out_dir::AbstractString,
                    t::Vector{Float64},
                    x::Vector{Float64},
                    theta::Vector{Float64},
                    u::Vector{Float64},
                    F_eq::Vector{Float64},
                    tau::Vector{Float64},
                    s_surf::Vector{Float64})

    xt = ticks_1dp(minimum(t), maximum(t); n = 10)

    begin
        yt = ticks_1dp(minimum(x), maximum(x); n = 10)
        p = plot(t, x; title="Cart Position x(t)", xlabel="time (s)", ylabel="x (m)",
                 xticks=xt, yticks=yt)
        savefig(p, joinpath(out_dir, "cart_position.png"))
    end

    begin
        yt = ticks_1dp(minimum(theta), maximum(theta); n = 10)
        p = plot(t, theta; title="Pole Angle theta(t) (0=upright)", xlabel="time (s)", ylabel="theta (rad)",
                 xticks=xt, yticks=yt)
        savefig(p, joinpath(out_dir, "pole_angle.png"))
    end

    begin
        yt = ticks_1dp(minimum(u), maximum(u); n = 10)
        p = plot(t, u; title="Control Force u(t) (SMC)", xlabel="time (s)", ylabel="u (N)",
                 xticks=xt, yticks=yt)
        savefig(p, joinpath(out_dir, "control_force.png"))
    end

    begin
        yt = ticks_1dp(minimum(tau), maximum(tau); n = 10)
        p = plot(t, tau; title="External Disturbance Torque τ(t)", xlabel="time (s)", ylabel="τ (N·m)",
                 xticks=xt, yticks=yt)
        savefig(p, joinpath(out_dir, "disturbance_torque.png"))
    end

    begin
        yt = ticks_1dp(minimum(F_eq), maximum(F_eq); n = 10)
        p = plot(t, F_eq; title="Equivalent Bob Force F_eq(t) = τ/L", xlabel="time (s)", ylabel="F_eq (N)",
                 xticks=xt, yticks=yt)
        savefig(p, joinpath(out_dir, "equivalent_bob_force.png"))
    end

    begin
        yt = ticks_1dp(minimum(s_surf), maximum(s_surf); n = 10)
        p = plot(t, s_surf; title="Sliding Surface s(t)", xlabel="time (s)", ylabel="s",
                 xticks=xt, yticks=yt)
        savefig(p, joinpath(out_dir, "sliding_surface.png"))
    end
end

# ------------------------------------------------------------
# MP4 encoding via ffmpeg
# ------------------------------------------------------------
function encode_mp4_with_ffmpeg(frames_dir::AbstractString, fps::Int, out_mp4::AbstractString)
    ok = success(pipeline(`ffmpeg -version`, stdout=devnull, stderr=devnull))
    if !ok
        println("ffmpeg not found on PATH; MP4 will not be created.")
        return
    end

    input_pattern = joinpath(frames_dir, "frame_%06d.png")
    cmd = `ffmpeg -y -framerate $fps -i $input_pattern -c:v libx264 -pix_fmt yuv420p $out_mp4`
    println("Encoding MP4...")
    success(pipeline(cmd, stdout=devnull, stderr=devnull)) || println("ffmpeg encoding failed.")
end

# ------------------------------------------------------------
# Frame renderer (Luxor) - frames are not affected by plot padding settings
# ------------------------------------------------------------
function render_frame_png(filename::AbstractString,
                          plant::CartPoleParams,
                          s::State,
                          tf::Float64,
                          dist::Disturbance,
                          Feq_used::Float64;
                          W::Int=1000,
                          H::Int=600,
                          pixels_per_meter::Float64=180.0,
                          track_y_from_top::Float64=0.75 * H,
                          arrow_len_px::Float64=120.0)

    Luxor.Drawing(W, H, filename)
    Luxor.origin()

    cart_w = 140.0
    cart_h = 40.0
    wheel_r = 14.0
    bob_r = 9.0
    pole_w = 8.0

    pole_len_px = plant.pole.L * pixels_per_meter

    y_nom = track_y_from_top - H/2
    margin = 20.0

    y_min = -H/2 + margin + cart_h/2 + pole_len_px + bob_r
    y_max = H/2 - margin - wheel_r - cart_h*0.55
    y_track = clamp(y_nom, y_min, y_max)

    bg          = (20/255, 20/255, 20/255)
    track_color = (170/255, 170/255, 170/255)
    cart_color  = (40/255, 140/255, 1.0)
    pole_color  = (240/255, 70/255, 70/255)
    bob_color   = (1.0, 220/255, 60/255)
    wheel_color = (70/255, 70/255, 70/255)
    arrow_color = (60/255, 1.0, 120/255)

    Luxor.sethue(bg...)
    Luxor.box(Luxor.Point(0, 0), W, H, :fill)

    Luxor.sethue(track_color...)
    Luxor.box(Luxor.Point(0, y_track + 25.0), W - 100, 4, :fill)

    Luxor.sethue(120/255, 120/255, 120/255)
    Luxor.box(Luxor.Point(0, y_track), 3, 60, :fill)

    cart_x = s.x * pixels_per_meter
    cart_y = y_track

    Luxor.sethue(cart_color...)
    Luxor.box(Luxor.Point(cart_x, cart_y), cart_w, cart_h, :fill)

    Luxor.sethue(wheel_color...)
    Luxor.circle(Luxor.Point(cart_x - cart_w * 0.30, cart_y + cart_h * 0.55), wheel_r, :fill)
    Luxor.circle(Luxor.Point(cart_x + cart_w * 0.30, cart_y + cart_h * 0.55), wheel_r, :fill)

    pivot = Luxor.Point(cart_x, cart_y - cart_h * 0.5)

    Luxor.sethue(pole_color...)
    Luxor.gsave()
    Luxor.translate(pivot)
    Luxor.rotate(s.theta)
    Luxor.box(Luxor.Point(0, -pole_len_px / 2), pole_w, pole_len_px, :fill)
    Luxor.grestore()

    tip_x = pivot.x + pole_len_px * sin(s.theta)
    tip_y = pivot.y - pole_len_px * cos(s.theta)

    Luxor.sethue(bob_color...)
    Luxor.circle(Luxor.Point(tip_x, tip_y), bob_r, :fill)

    if arrow_visible(dist, tf)
        dir = Feq_used >= 0.0 ? 1.0 : -1.0
        Luxor.sethue(arrow_color...)
        start = Luxor.Point(tip_x, tip_y - 25.0)
        draw_force_arrow(start, dir, arrow_len_px)
    end

    Luxor.finish()
    return
end

# ------------------------------------------------------------
# Main program
# ------------------------------------------------------------
function main()
    out_dir = joinpath("output", "pendulum_sliding_mode")
    frames_dir = joinpath(out_dir, "frames")
    mkpath(frames_dir)

    for f in readdir(frames_dir)
        rm(joinpath(frames_dir, f); force=true)
    end

    video_seconds = 10.0
    total_frames = 6000
    fps = Int(round(total_frames / video_seconds))

    dt_frame = video_seconds / total_frames
    substeps = 8
    dt_physics = dt_frame / substeps

    plant = CartPoleParams(
        M = 1.2,
        pole = PoleModel(L=1.0, m_rod=0.10, m_bob=0.15),
        g = 9.81,
        cart_damping = 0.10,
        pole_damping = 0.03
    )

    ctrl = ControlParams()
    track = TrackLimits(1.6)
    dist = Disturbance()

    s = State(theta=0.20)

    t_log   = Vector{Float64}(undef, total_frames)
    x_log   = Vector{Float64}(undef, total_frames)
    th_log  = Vector{Float64}(undef, total_frames)
    u_log   = Vector{Float64}(undef, total_frames)
    Feq_log = Vector{Float64}(undef, total_frames)
    tau_log = Vector{Float64}(undef, total_frames)
    surf_log= Vector{Float64}(undef, total_frames)

    t = 0.0

    for frame in 0:(total_frames - 1)
        u_used = 0.0
        tau_used = 0.0
        Feq_used = 0.0
        s_used = 0.0

        for _ in 1:substeps
            tau_external = tau_ext(dist, t)
            u = compute_control(plant, ctrl, s)

            rk4_step!(plant, s, dt_physics, u, tau_external)
            enforce!(track, s)

            u_used = u
            tau_used = tau_external
            Feq_used = bob_force_equivalent(dist, t, plant.pole.L)
            s_used = sliding_surface(ctrl, s)

            t += dt_physics
        end

        tf = frame * dt_frame

        idx = frame + 1
        t_log[idx] = tf
        x_log[idx] = s.x
        th_log[idx] = s.theta
        u_log[idx] = u_used
        tau_log[idx] = tau_used
        Feq_log[idx] = Feq_used
        surf_log[idx] = s_used

        fn = joinpath(frames_dir, @sprintf("frame_%06d.png", frame))
        render_frame_png(fn, plant, s, tf, dist, Feq_used;
                         W=1000, H=600, pixels_per_meter=180.0,
                         track_y_from_top=0.75 * 600, arrow_len_px=120.0)

        if frame % 600 == 0
            @printf("Frame %d/%d  t=%.2f  x=%.3f  theta=%.3f  u=%.2f  tau=%.3f\n",
                    frame, total_frames, tf, s.x, s.theta, u_used, tau_used)
        end
    end

    mp4 = joinpath(out_dir, "pendulum_smc_10s_6000f.mp4")
    encode_mp4_with_ffmpeg(frames_dir, fps, mp4)

    println("Saving plots and CSV...")
    save_plots(out_dir, t_log, x_log, th_log, u_log, Feq_log, tau_log, surf_log)

    write_csv(joinpath(out_dir, "cartpole_log.csv"),
              ["t","x","theta","u","F_equiv","tau_ext","s"],
              [t_log, x_log, th_log, u_log, Feq_log, tau_log, surf_log])

    println("Done.")
end

main()
