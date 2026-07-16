# ------------------------------------------------------------
# 2D Heat Equation (Heat Transfer by Conduction) in Julia
# ------------------------------------------------------------
# PDE:
#   ∂T/∂t = α ( ∂²T/∂x² + ∂²T/∂y² )
#
# Method:
#   - 2D explicit finite-difference scheme (FTCS)
#   - Dirichlet boundary conditions (fixed temperature on boundaries)
#   - Snapshots saved as PNG heatmaps
#   - Additional plots saved (center point vs time, final centerline)
#
# Output folder (relative to where you run the program):
#   output/heat2d/
# ------------------------------------------------------------

ENV["GKSwstype"] = "100"

using Printf
using Plots
import Plots: px

gr()

# ------------------------------------------------------------
# Plot styling (high-quality static figures)
# ------------------------------------------------------------
default(
    dpi = 300,
    size = (1000, 700),
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
# Helpers: ticks with exactly 1 decimal digit and ≤ 10 ticks
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
# Minimal CSV writer (two columns)
# ------------------------------------------------------------
function write_csv_two_columns(filename::AbstractString,
                               h1::AbstractString,
                               h2::AbstractString,
                               c1::AbstractVector{<:Real},
                               c2::AbstractVector{<:Real})
    if length(c1) != length(c2)
        error("CSV write error: column sizes do not match.")
    end

    open(filename, "w") do io
        println(io, "$(h1),$(h2)")
        for k in eachindex(c1)
            println(io, "$(c1[k]),$(c2[k])")
        end
    end
end

# ------------------------------------------------------------
# Main program
# ------------------------------------------------------------
function main()
    out_dir = joinpath("output", "heat2d")
    mkpath(out_dir)

    # Physical parameters
    alpha = 1.0

    # Domain and grid
    Lx = 1.0
    Ly = 1.0
    nx = 81
    ny = 81

    dx = Lx / (nx - 1)
    dy = Ly / (ny - 1)

    x = range(0.0, Lx; length = nx)
    y = range(0.0, Ly; length = ny)

    # Time step (explicit stability)
    dt_stable = 1.0 / (2.0 * alpha * (1.0 / (dx^2) + 1.0 / (dy^2)))
    dt = 0.80 * dt_stable

    t_end = 0.20
    nsteps = ceil(Int, t_end / dt)

    desired_snapshots = 30
    snapshot_every = max(1, nsteps ÷ desired_snapshots)

    # Temperature fields (T[j, i] = T(x[i], y[j]))
    T = zeros(Float64, ny, nx)
    T_new = similar(T)

    # Dirichlet boundaries
    T_left = 100.0
    T_right = 0.0
    T_top = 0.0
    T_bottom = 0.0

    T[:, 1] .= T_left
    T[:, end] .= T_right
    T[1, :] .= T_bottom
    T[end, :] .= T_top

    # Center logging
    ic = (nx ÷ 2) + 1
    jc = (ny ÷ 2) + 1

    time_log = Float64[]
    center_T_log = Float64[]

    # 1-decimal, ≤10 ticks for coordinate axes
    xticks_xy = ticks_1dp(0.0, Lx; n = 10)
    yticks_xy = ticks_1dp(0.0, Ly; n = 10)

    # Time integration
    t = 0.0
    for step in 1:nsteps
        push!(time_log, t)
        push!(center_T_log, T[jc, ic])

        @inbounds for j in 2:(ny - 1)
            for i in 2:(nx - 1)
                Txx = (T[j, i + 1] - 2.0 * T[j, i] + T[j, i - 1]) / (dx^2)
                Tyy = (T[j + 1, i] - 2.0 * T[j, i] + T[j - 1, i]) / (dy^2)
                T_new[j, i] = T[j, i] + alpha * dt * (Txx + Tyy)
            end
        end

        # Re-apply boundaries
        T_new[:, 1] .= T_left
        T_new[:, end] .= T_right
        T_new[1, :] .= T_bottom
        T_new[end, :] .= T_top

        T, T_new = T_new, T

        # Snapshot heatmap
        if (step - 1) % snapshot_every == 0
            p = heatmap(
                x, y, T;
                title = "2D Heat Equation - Temperature Field",
                xlabel = "x (m)",
                ylabel = "y (m)",
                aspect_ratio = 1,
                xticks = xticks_xy,
                yticks = yticks_xy
            )

            png_name = joinpath(out_dir, @sprintf("heat_t%06d.png", step - 1))
            savefig(p, png_name)
            println("Saved snapshot: ", png_name)
        end

        t += dt
    end

    # Final centerline (y = middle)
    centerline = vec(T[jc, :])

    begin
        xt = ticks_1dp(0.0, Lx; n = 10)
        yt = ticks_1dp(minimum(centerline), maximum(centerline); n = 10)

        p = plot(
            x, centerline;
            title = "Final Centerline Temperature (y = 0.5 m)",
            xlabel = "x (m)",
            ylabel = "T(x, y=0.5)",
            xticks = xt,
            yticks = yt
        )
        savefig(p, joinpath(out_dir, "centerline_final.png"))
    end

    # Center point vs time
    begin
        xt = ticks_1dp(minimum(time_log), maximum(time_log); n = 10)
        yt = ticks_1dp(minimum(center_T_log), maximum(center_T_log); n = 10)

        p = plot(
            time_log, center_T_log;
            title = "Temperature at Plate Center vs Time",
            xlabel = "time (s)",
            ylabel = "T(center)",
            xticks = xt,
            yticks = yt
        )
        savefig(p, joinpath(out_dir, "center_point_vs_time.png"))
    end

    # CSV log
    write_csv_two_columns(joinpath(out_dir, "heat2d_log.csv"),
                          "t", "T_center",
                          time_log, center_T_log)

    println("Heat2D finished. Results are in: ", out_dir)
end

main()
