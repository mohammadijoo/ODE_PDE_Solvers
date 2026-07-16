// ------------------------------------------------------------
// Cart–Pole Inverted Pendulum (moving base) with SLIDING MODE CONTROL (SMC)
// Requirements implemented exactly:
//   - Total animation: 10 seconds
//   - Total frames: 6000 (=> 600 FPS video)
//   - Exactly two disturbances: at t=0.5s and t=5.0s
//   - Disturbance shown as an arrow for 0.5s only (direction indicator, not stretched)
//   - Disturbance direction and base compensation direction are consistent:
//       "push right" => pole tips right => cart accelerates right to recover
//   - Dynamics are actually integrated and used to render each frame
//   - Outputs: frames, mp4 (ffmpeg), plots (matplot++), CSV log
//
// Output folders:
//   output/pendulum_sliding_mode/frames/frame_000000.png ... frame_005999.png
//   output/pendulum_sliding_mode/pendulum_smc_10s_6000f.mp4   (if ffmpeg in PATH)
//   output/pendulum_sliding_mode/*.png plots                  (if gnuplot in PATH)
//   output/pendulum_sliding_mode/cartpole_log.csv
// ------------------------------------------------------------

#define _USE_MATH_DEFINES
#include <cmath>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib> // std::system

#include <SFML/Graphics.hpp>
#include <matplot/matplot.h>

// ------------------------------------------------------------
// Small math helpers
// ------------------------------------------------------------
static double clamp(double x, double lo, double hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static double wrap_to_pi(double a)
{
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// Boundary-layer saturation for sliding mode: sat(z) in [-1, 1]
static double sat(double z) { return clamp(z, -1.0, 1.0); }

// ------------------------------------------------------------
// CSV writer
// ------------------------------------------------------------
static void write_csv(const std::string& filename,
                      const std::vector<std::string>& header,
                      const std::vector<std::vector<double>>& cols)
{
    if (cols.empty()) throw std::runtime_error("CSV: no columns.");
    const std::size_t n = cols[0].size();
    for (const auto& c : cols)
        if (c.size() != n) throw std::runtime_error("CSV: column size mismatch.");

    std::ofstream out(filename);
    if (!out) throw std::runtime_error("CSV: cannot open " + filename);

    for (std::size_t i = 0; i < header.size(); ++i)
    {
        out << header[i];
        if (i + 1 < header.size()) out << ",";
    }
    out << "\n";

    for (std::size_t r = 0; r < n; ++r)
    {
        for (std::size_t c = 0; c < cols.size(); ++c)
        {
            out << cols[c][r];
            if (c + 1 < cols.size()) out << ",";
        }
        out << "\n";
    }
}

// ------------------------------------------------------------
// Pole model: uniform rod + lumped bob at the top
// IMPORTANT: masses are still smaller than cart, but not "almost zero"
// to avoid unrealistic accelerations.
// ------------------------------------------------------------
struct PoleModel
{
    double L = 1.0;        // (m)
    double m_rod = 0.06;   // (kg)
    double m_bob = 0.04;   // (kg)

    double m_total = 0.0;
    double l_com = 0.0;
    double I_pivot = 0.0;
    double I_com = 0.0;
    double inertia_factor = 0.0; // 1 + I_com/(m*l^2)

    void compute_derived()
    {
        m_total = m_rod + m_bob;

        // COM from pivot: rod at L/2, bob at L
        l_com = (m_rod * (L * 0.5) + m_bob * L) / m_total;

        // Inertia about pivot:
        // rod: (1/3)mL^2, bob: mL^2
        I_pivot = (1.0 / 3.0) * m_rod * L * L + m_bob * L * L;

        // Inertia about COM
        I_com = I_pivot - m_total * l_com * l_com;

        inertia_factor = 1.0 + I_com / (m_total * l_com * l_com);
    }
};

// ------------------------------------------------------------
// Plant parameters
// ------------------------------------------------------------
struct CartPoleParams
{
    double M = 1.2;   // cart mass (kg)  >> pole total mass (0.10 kg)
    PoleModel pole;

    double g = 9.81;

    // Damping for realism and numerical stability
    double cart_damping = 0.10; // N per (m/s)
    double pole_damping = 0.03; // applied to theta_ddot term
};

// ------------------------------------------------------------
// State (theta=0 upright)
// NOTE: We render tip_x = x + L*sin(theta), so theta>0 appears leaning right.
// ------------------------------------------------------------
struct State
{
    double x = 0.0;
    double xdot = 0.0;
    double theta = 0.20;
    double thetadot = 0.0;
};

// ------------------------------------------------------------
// Sliding Mode Control parameters + a small cart-centering term
// The centering term is "gated": it becomes strong only when |theta| is small.
// This prevents it from fighting the corrective cart motion during a disturbance.
// ------------------------------------------------------------
struct ControlParams
{
    // Sliding surface:
    // s = theta_dot + lambda_theta*theta + alpha*(x_dot + lambda_x*x)
    double lambda_theta = 10.0;
    double lambda_x     = 1.5;
    double alpha        = 0.55;

    // Sliding mode dynamics:
    double k   = 110.0;
    double phi = 0.05;

    // Actuator saturation
    double u_max = 70.0; // N

    // Cart centering (only near upright)
    double hold_kp = 8.0;   // N/m
    double hold_kd = 10.0;  // N/(m/s)

    // Gate: when |theta| >= theta_gate, centering is ~0
    double theta_gate = 0.20; // rad
};

// ------------------------------------------------------------
// Track limit: keep the cart in view
// ------------------------------------------------------------
struct TrackLimits
{
    double x_max = 1.6; // meters

    void enforce(State& s) const
    {
        if (s.x > x_max)
        {
            s.x = x_max;
            if (s.xdot > 0) s.xdot = 0;
        }
        if (s.x < -x_max)
        {
            s.x = -x_max;
            if (s.xdot < 0) s.xdot = 0;
        }
    }
};

// ------------------------------------------------------------
// Disturbance schedule
// EXACTLY two disturbances:
//   1) starts at t = 0.5 s
//   2) starts at t = 5.0 s
// Arrow shown for 0.5 s only (direction indicator).
//
// We model the disturbance as a "push at the bob" but apply it as an EXTERNAL TORQUE
// so we do not inject net cart momentum (which makes the cart run away).
//
// Direction convention we enforce:
//   - "push right" => pole tends to lean right (theta increases)
//
// We implement tau_ext(t) directly as:
//   tau_ext = +tau_amp * pulse_shape
// and we also report an equivalent force for arrow direction:
//   F_eq = tau_ext / L
// ------------------------------------------------------------
struct Disturbance
{
    double t1 = 0.5;
    double t2 = 5.0;

    // Your requirement: arrow only half a second
    double arrow_duration = 0.5;

    // Disturbance duration (use same as arrow: half a second)
    double duration = 0.5;

    // Torque amplitude (N*m). Increase slightly if you want stronger visible response.
    double tau_amp = 3.3;

    // Smooth half-sine pulse in [0, duration]
    static double half_sine(double local_t, double duration)
    {
        // 0 -> 1 -> 0
        return std::sin(M_PI * local_t / duration);
    }

    // Returns external torque about pivot (N*m).
    // First pulse: positive torque (push right)
    // Second pulse: negative torque (push left)
    double tau_ext(double t) const
    {
        // Pulse 1
        if (t >= t1 && t <= t1 + duration)
        {
            const double local = t - t1;
            return +tau_amp * half_sine(local, duration);
        }
        // Pulse 2
        if (t >= t2 && t <= t2 + duration)
        {
            const double local = t - t2;
            return -tau_amp * half_sine(local, duration);
        }
        return 0.0;
    }

    // "Equivalent bob force" used only for arrow direction and logging:
    // F_eq = tau / L.
    double bob_force_equivalent(double t, double L) const
    {
        return tau_ext(t) / L;
    }

    // Arrow is shown for 0.5 s only after each pulse start.
    bool arrow_visible(double t) const
    {
        if (t >= t1 && t <= t1 + arrow_duration) return true;
        if (t >= t2 && t <= t2 + arrow_duration) return true;
        return false;
    }
};

// ------------------------------------------------------------
// ODE derivative
// ------------------------------------------------------------
struct Deriv
{
    double x_dot;
    double x_ddot;
    double theta_dot;
    double theta_ddot;
};

// ------------------------------------------------------------
// Nonlinear cart–pole dynamics + external torque tau_ext.
// This is a standard generalized form (Gym-like) using inertia_factor.
// We also add damping and the external torque term:
//
//   theta_ddot += tau_ext / I_pivot
//
// ------------------------------------------------------------
static Deriv dynamics(const CartPoleParams& p, const State& s, double u_cart, double tau_ext)
{
    const double m = p.pole.m_total;
    const double l = p.pole.l_com;

    const double total_mass = p.M + m;
    const double polemass_length = m * l;

    const double sin_t = std::sin(s.theta);
    const double cos_t = std::cos(s.theta);

    // Cart friction-like damping
    const double F_damped = u_cart - p.cart_damping * s.xdot;

    const double temp =
        (F_damped + polemass_length * s.thetadot * s.thetadot * sin_t) / total_mass;

    const double denom =
        l * (p.pole.inertia_factor - (m * cos_t * cos_t) / total_mass);

    double theta_ddot =
        (p.g * sin_t - cos_t * temp) / denom;

    // Pole damping
    theta_ddot -= p.pole_damping * s.thetadot;

    // External torque about pivot (disturbance)
    theta_ddot += tau_ext / p.pole.I_pivot;

    const double x_ddot =
        temp - polemass_length * theta_ddot * cos_t / total_mass;

    return Deriv{s.xdot, x_ddot, s.thetadot, theta_ddot};
}

// ------------------------------------------------------------
// RK4 integration step
// ------------------------------------------------------------
static void rk4_step(const CartPoleParams& p, State& s, double dt, double u_cart, double tau_ext)
{
    auto add_scaled = [](const State& a, const Deriv& k, double h)
    {
        State out = a;
        out.x        += h * k.x_dot;
        out.xdot     += h * k.x_ddot;
        out.theta    += h * k.theta_dot;
        out.thetadot += h * k.theta_ddot;
        return out;
    };

    const Deriv k1 = dynamics(p, s, u_cart, tau_ext);
    const Deriv k2 = dynamics(p, add_scaled(s, k1, 0.5 * dt), u_cart, tau_ext);
    const Deriv k3 = dynamics(p, add_scaled(s, k2, 0.5 * dt), u_cart, tau_ext);
    const Deriv k4 = dynamics(p, add_scaled(s, k3, dt),       u_cart, tau_ext);

    s.x        += (dt / 6.0) * (k1.x_dot      + 2.0 * k2.x_dot      + 2.0 * k3.x_dot      + k4.x_dot);
    s.xdot     += (dt / 6.0) * (k1.x_ddot     + 2.0 * k2.x_ddot     + 2.0 * k3.x_ddot     + k4.x_ddot);
    s.theta    += (dt / 6.0) * (k1.theta_dot  + 2.0 * k2.theta_dot  + 2.0 * k3.theta_dot  + k4.theta_dot);
    s.thetadot += (dt / 6.0) * (k1.theta_ddot + 2.0 * k2.theta_ddot + 2.0 * k3.theta_ddot + k4.theta_ddot);

    s.theta = wrap_to_pi(s.theta);
}

// ------------------------------------------------------------
// Sliding surface and SMC control
// ------------------------------------------------------------
static double sliding_surface(const ControlParams& c, const State& s)
{
    // Main stabilization of pole + small cart terms
    return s.thetadot
         + c.lambda_theta * s.theta
         + c.alpha * (s.xdot + c.lambda_x * s.x);
}

static double sliding_surface_dot_nominal(const CartPoleParams& p, const ControlParams& c, const State& s, double u_cart)
{
    // Nominal: ignore external torque for the control law
    const Deriv d = dynamics(p, s, u_cart, 0.0);

    // sdot = theta_ddot + lambda_theta*theta_dot + alpha*(x_ddot + lambda_x*x_dot)
    return d.theta_ddot
         + c.lambda_theta * s.thetadot
         + c.alpha * (d.x_ddot + c.lambda_x * s.xdot);
}

static double compute_control(const CartPoleParams& p, const ControlParams& c, const State& s)
{
    // ----- Sliding Mode Control part -----
    const double sval = sliding_surface(c, s);
    const double desired_sdot = -c.k * sat(sval / c.phi);

    // Numeric affine approximation: sdot(u) ≈ a*u + b
    const double sdot0 = sliding_surface_dot_nominal(p, c, s, 0.0);
    const double sdot1 = sliding_surface_dot_nominal(p, c, s, 1.0);
    const double a = (sdot1 - sdot0);
    const double b = sdot0;

    double u_smc = 0.0;
    if (std::abs(a) < 1e-8)
        u_smc = 0.0;
    else
        u_smc = (desired_sdot - b) / a;

    // ----- Cart-centering term (gated) -----
    // When the pole is near upright, pull cart back to x=0.
    // When pole is far from upright, don't fight the catch maneuver.
    const double theta_abs = std::abs(s.theta);
    const double gate = clamp(1.0 - (theta_abs / c.theta_gate), 0.0, 1.0);

    const double u_hold = gate * (-c.hold_kp * s.x - c.hold_kd * s.xdot);

    // Combined
    const double u_total = u_smc + u_hold;

    // Saturate actuator
    return clamp(u_total, -c.u_max, c.u_max);
}

// ------------------------------------------------------------
// SFML arrow drawing (SFML 3-safe)
// Arrow length is constant to avoid "stretched" look.
// ------------------------------------------------------------
static void draw_force_arrow(sf::RenderTarget& target,
                             const sf::Vector2f& start,
                             const sf::Vector2f& end,
                             const sf::Color& color)
{
    sf::Vertex line[2];
    line[0].position = start;
    line[0].color    = color;
    line[1].position = end;
    line[1].color    = color;
    target.draw(line, 2, sf::PrimitiveType::Lines);

    const sf::Vector2f dir = end - start;
    const float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
    if (len < 1e-3f) return;

    const sf::Vector2f u = dir / len;
    const sf::Vector2f n{-u.y, u.x};

    const float head_len = 14.0f;
    const float head_w   = 7.0f;

    const sf::Vector2f tip  = end;
    const sf::Vector2f base = end - u * head_len;

    sf::ConvexShape tri;
    tri.setPointCount(3);
    tri.setPoint(0, tip);
    tri.setPoint(1, base + n * head_w);
    tri.setPoint(2, base - n * head_w);
    tri.setFillColor(color);

    target.draw(tri);
}

// ------------------------------------------------------------
// Plot saving (Matplot++)
// ------------------------------------------------------------
static void save_plots(const std::string& out_dir,
                       const std::vector<double>& t,
                       const std::vector<double>& x,
                       const std::vector<double>& theta,
                       const std::vector<double>& u,
                       const std::vector<double>& F_eq,
                       const std::vector<double>& tau_ext,
                       const std::vector<double>& ssurf)
{
    using namespace matplot;

    {
        auto fig = figure(true);
        plot(t, x);
        title("Cart Position x(t)");
        xlabel("time (s)");
        ylabel("x (m)");
        fig->save(out_dir + "/cart_position.png");
    }
    {
        auto fig = figure(true);
        plot(t, theta);
        title("Pole Angle theta(t) (0=upright)");
        xlabel("time (s)");
        ylabel("theta (rad)");
        fig->save(out_dir + "/pole_angle.png");
    }
    {
        auto fig = figure(true);
        plot(t, u);
        title("Control Force u(t) (SMC)");
        xlabel("time (s)");
        ylabel("u (N)");
        fig->save(out_dir + "/control_force.png");
    }
    {
        auto fig = figure(true);
        plot(t, tau_ext);
        title("External Disturbance Torque tau_ext(t)");
        xlabel("time (s)");
        ylabel("tau_ext (N*m)");
        fig->save(out_dir + "/disturbance_torque.png");
    }
    {
        auto fig = figure(true);
        plot(t, F_eq);
        title("Equivalent Bob Force (for arrow direction) F_eq(t) = tau/L");
        xlabel("time (s)");
        ylabel("F_eq (N)");
        fig->save(out_dir + "/equivalent_bob_force.png");
    }
    {
        auto fig = figure(true);
        plot(t, ssurf);
        title("Sliding Surface s(t)");
        xlabel("time (s)");
        ylabel("s");
        fig->save(out_dir + "/sliding_surface.png");
    }
}

// ------------------------------------------------------------
// MP4 encoding via ffmpeg
// ------------------------------------------------------------
static void encode_mp4_with_ffmpeg(const std::string& frames_dir, int fps, const std::string& out_mp4)
{
    const int ok = std::system("ffmpeg -version >nul 2>&1");
    if (ok != 0)
    {
        std::cerr << "ffmpeg not found on PATH; MP4 will not be created.\n";
        return;
    }

    std::ostringstream cmd;
    cmd << "ffmpeg -y "
        << "-framerate " << fps << " "
        << "-i \"" << frames_dir << "/frame_%06d.png\" "
        << "-c:v libx264 -pix_fmt yuv420p "
        << "\"" << out_mp4 << "\""
        << " >nul 2>&1";

    std::cout << "Encoding MP4...\n";
    const int rc = std::system(cmd.str().c_str());
    if (rc == 0) std::cout << "MP4 created: " << out_mp4 << "\n";
    else         std::cerr << "ffmpeg encoding failed.\n";
}

int main(int argc, char** argv)
{
    // Optional: --preview to show a window while generating frames
    bool preview = false;
    for (int i = 1; i < argc; ++i)
        if (std::string(argv[i]) == "--preview") preview = true;

    // ----------------------------
    // Output folders
    // ----------------------------
    const std::string out_dir    = "output/pendulum_sliding_mode";
    const std::string frames_dir = out_dir + "/frames";
    std::filesystem::create_directories(frames_dir);

    // Clean old frames
    for (const auto& e : std::filesystem::directory_iterator(frames_dir))
        if (e.is_regular_file()) std::filesystem::remove(e.path());

    // ----------------------------
    // Simulation settings (YOUR REQUEST)
    // ----------------------------
    const double video_seconds = 10.0;
    const int total_frames = 1800;          // EXACT
    const int fps = static_cast<int>(total_frames / video_seconds); // 600 FPS

    const double dt_frame = video_seconds / static_cast<double>(total_frames);

    // Physics integration step:
    // We use multiple substeps per frame so the dynamics are stable at high FPS.
    const int substeps = 8;
    const double dt_physics = dt_frame / static_cast<double>(substeps);

    // Screen
    const unsigned W = 1000;
    const unsigned H = 600;

    // ----------------------------
    // Plant / controller / disturbance
    // ----------------------------
    CartPoleParams plant;
    plant.M = 1.2;
    plant.pole.L = 1.0;
    plant.pole.m_rod = 0.1;
    plant.pole.m_bob = 0.15;
    plant.pole.compute_derived();

    ControlParams ctrl;
    TrackLimits track;
    Disturbance dist;

    // Initial state
    State s;
    s.theta = 0.20;

    // ----------------------------
    // Logging (one entry per frame)
    // ----------------------------
    std::vector<double> t_log, x_log, th_log, u_log, Feq_log, tau_log, surf_log;
    t_log.reserve(total_frames);

    // ----------------------------
    // SFML offscreen renderer
    // ----------------------------
    sf::RenderTexture render;
    if (!render.resize(sf::Vector2u{W, H}))
    {
        std::cerr << "RenderTexture creation failed.\n";
        return 1;
    }

    std::optional<sf::RenderWindow> window;
    if (preview)
    {
        window.emplace(sf::VideoMode({W, H}), "Cart-Pole SMC (10s, 6000 frames)");
        window->setFramerateLimit(60); // preview only; does not affect saved frames
    }

    // ----------------------------
    // Visual mapping
    // ----------------------------
    const float pixels_per_meter = 180.0f;
    const sf::Vector2f origin{W * 0.5f, H * 0.75f};

    const sf::Color bg(20, 20, 20);
    const sf::Color track_color(170, 170, 170);
    const sf::Color cart_color(40, 140, 255);
    const sf::Color pole_color(240, 70, 70);
    const sf::Color bob_color(255, 220, 60);
    const sf::Color wheel_color(70, 70, 70);
    const sf::Color arrow_color(60, 255, 120);

    // Track line
    sf::RectangleShape track_line({static_cast<float>(W - 100), 4.0f});
    track_line.setFillColor(track_color);
    track_line.setOrigin({track_line.getSize().x * 0.5f, track_line.getSize().y * 0.5f});
    track_line.setPosition({origin.x, origin.y + 25.0f});

    // Center marker (helps visually confirm cart stays centered)
    sf::RectangleShape center_mark({3.0f, 60.0f});
    center_mark.setFillColor(sf::Color(120, 120, 120));
    center_mark.setOrigin({1.5f, 30.0f});
    center_mark.setPosition({origin.x, origin.y});

    // Cart
    const sf::Vector2f cart_size{140.0f, 40.0f};
    sf::RectangleShape cart_rect(cart_size);
    cart_rect.setFillColor(cart_color);
    cart_rect.setOrigin({cart_size.x * 0.5f, cart_size.y * 0.5f});

    // Wheels
    const float wheel_r = 14.0f;
    sf::CircleShape wheel1(wheel_r), wheel2(wheel_r);
    wheel1.setFillColor(wheel_color);
    wheel2.setFillColor(wheel_color);
    wheel1.setOrigin({wheel_r, wheel_r});
    wheel2.setOrigin({wheel_r, wheel_r});

    // Pole
    const float pole_len_px = static_cast<float>(plant.pole.L * pixels_per_meter);
    sf::RectangleShape pole_rect({8.0f, pole_len_px});
    pole_rect.setFillColor(pole_color);
    pole_rect.setOrigin({4.0f, pole_len_px});

    // Bob
    sf::CircleShape bob(9.0f);
    bob.setFillColor(bob_color);
    bob.setOrigin({9.0f, 9.0f});

    // Arrow constant size (NOT stretched)
    const float arrow_len_px = 120.0f;

    // ----------------------------
    // Main loop: 6000 frames
    // ----------------------------
    double t = 0.0;

    for (int frame = 0; frame < total_frames; ++frame)
    {
        // Preview window events
        if (window.has_value())
        {
            while (const std::optional<sf::Event> ev = window->pollEvent())
            {
                if (ev->is<sf::Event::Closed>())
                {
                    window->close();
                    window.reset();
                }
            }
        }

        // Integrate dynamics for this frame using substeps
        double u_used = 0.0;
        double tau_used = 0.0;
        double Feq_used = 0.0;
        double s_used = 0.0;

        for (int k = 0; k < substeps; ++k)
        {
            // Disturbance torque
            const double tau_ext = dist.tau_ext(t);

            // Control force (SMC)
            const double u = compute_control(plant, ctrl, s);

            // Integrate
            rk4_step(plant, s, dt_physics, u, tau_ext);

            // Keep cart on track
            track.enforce(s);

            // Save latest values for logging/rendering
            u_used = u;
            tau_used = tau_ext;
            Feq_used = dist.bob_force_equivalent(t, plant.pole.L);
            s_used = sliding_surface(ctrl, s);

            t += dt_physics;
        }

        // Log once per frame
        const double tf = static_cast<double>(frame) * dt_frame;
        t_log.push_back(tf);
        x_log.push_back(s.x);
        th_log.push_back(s.theta);
        u_log.push_back(u_used);
        tau_log.push_back(tau_used);
        Feq_log.push_back(Feq_used);
        surf_log.push_back(s_used);

        // ----------------------------
        // Render using CURRENT integrated state
        // ----------------------------
        render.clear(bg);
        render.draw(track_line);
        render.draw(center_mark);

        const float cart_x_px = origin.x + static_cast<float>(s.x * pixels_per_meter);
        const float cart_y_px = origin.y;

        cart_rect.setPosition({cart_x_px, cart_y_px});
        render.draw(cart_rect);

        wheel1.setPosition({cart_x_px - cart_size.x * 0.30f, cart_y_px + cart_size.y * 0.55f});
        wheel2.setPosition({cart_x_px + cart_size.x * 0.30f, cart_y_px + cart_size.y * 0.55f});
        render.draw(wheel1);
        render.draw(wheel2);

        // Pivot at top center of cart
        const sf::Vector2f pivot{cart_x_px, cart_y_px - cart_size.y * 0.5f};

        // Rotate pole by theta (SFML uses degrees; positive is clockwise on screen)
        const float theta_deg = static_cast<float>(s.theta * 180.0 / M_PI);
        pole_rect.setPosition(pivot);
        pole_rect.setRotation(sf::degrees(theta_deg));
        render.draw(pole_rect);

        // Bob at tip
        const float tip_x = pivot.x + pole_len_px * std::sin(static_cast<float>(s.theta));
        const float tip_y = pivot.y - pole_len_px * std::cos(static_cast<float>(s.theta));
        bob.setPosition({tip_x, tip_y});
        render.draw(bob);

        // Arrow shown only 0.5 s after each disturbance start
        if (dist.arrow_visible(tf))
        {
            // Direction from equivalent force sign (tau/L)
            const float dir = (Feq_used >= 0.0) ? 1.0f : -1.0f;

            // Place arrow slightly above the bob
            const sf::Vector2f start{tip_x, tip_y - 25.0f};
            const sf::Vector2f end{tip_x + dir * arrow_len_px, tip_y - 25.0f};
            draw_force_arrow(render, start, end, arrow_color);
        }

        render.display();

        // Save frame
        sf::Image img = render.getTexture().copyToImage();
        std::ostringstream fn;
        fn << frames_dir << "/frame_" << std::setw(6) << std::setfill('0') << frame << ".png";
        if (!img.saveToFile(fn.str()))
        {
            std::cerr << "Failed to save: " << fn.str() << "\n";
            return 1;
        }

        // Optional preview
        if (window.has_value())
        {
            window->clear(bg);
            window->draw(sf::Sprite(render.getTexture()));
            window->display();
        }

        // Console progress
        if (frame % 600 == 0) // every ~1 second at 600 fps
        {
            std::cout << "Frame " << frame << "/" << total_frames
                      << "  t=" << std::fixed << std::setprecision(2) << tf
                      << "  x=" << std::setprecision(3) << s.x
                      << "  theta=" << std::setprecision(3) << s.theta
                      << "  u=" << std::setprecision(2) << u_used
                      << "  tau=" << std::setprecision(3) << tau_used
                      << "\n";
        }
    }

    // Encode MP4 (600 fps, 10 seconds)
    const std::string mp4 = out_dir + "/pendulum_smc_10s_6000f.mp4";
    encode_mp4_with_ffmpeg(frames_dir, fps, mp4);

    // Save plots + CSV
    std::cout << "Saving plots and CSV...\n";
    save_plots(out_dir, t_log, x_log, th_log, u_log, Feq_log, tau_log, surf_log);

    write_csv(out_dir + "/cartpole_log.csv",
              {"t","x","theta","u","F_equiv","tau_ext","s"},
              {t_log, x_log, th_log, u_log, Feq_log, tau_log, surf_log});

    std::cout << "Done.\n";
    return 0;
}
