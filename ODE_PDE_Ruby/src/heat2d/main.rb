# frozen_string_literal: true

# ------------------------------------------------------------
# 2D Heat Equation (Heat Transfer by Conduction) in Ruby
# ------------------------------------------------------------
# PDE:
#   ∂T/∂t = α ( ∂²T/∂x² + ∂²T/∂y² )
#
# Method:
#   - 2D explicit finite-difference scheme (FTCS)
#   - Dirichlet boundary conditions (fixed temperature on boundaries)
#   - Snapshots saved as PNG heatmaps using Gnuplot
#   - Additional plots saved (center point vs time, final centerline)
#
# Output folder (relative to where you run the program):
#   output/heat2d/
#
# Notes:
#   - Gnuplot must be installed and available on PATH at runtime.
# ------------------------------------------------------------

require "fileutils"
require_relative "../../lib/common/csv_writer"
require_relative "../../lib/common/gnuplot_helpers"

# Flatten 2D indexing (i, j) into a 1D array index.
# nx is the number of columns in the x-direction.
def idx(i, j, nx)
  j * nx + i
end

# Write a pm3d-friendly grid file:
# Each y-row is separated by a blank line so gnuplot can interpret the surface grid.
def write_pm3d_grid(filename, t_field, nx, ny)
  File.open(filename, "w") do |f|
    (0...ny).each do |j|
      (0...nx).each do |i|
        f.puts "#{i} #{j} #{t_field[idx(i, j, nx)]}"
      end
      f.puts
    end
  end
end

# Create a high-quality heatmap plot via gnuplot (pm3d map).
def save_heatmap_png!(out_png, grid_dat, title_text, nx, ny)
  out_dir = File.dirname(out_png)

  # Keep color scale consistent across snapshots for easy comparison.
  script = <<~GP
    #{GnuplotHelpers.common_png_header(out_png, width_px: 2000, height_px: 1800, font: "Arial,30", line_width: 3)}

    unset key
    set title "#{title_text}" font "Arial,38"
    set xlabel "x index" font "Arial,34"
    set ylabel "y index" font "Arial,34"

    # Limit tick count for readability.
    set xtics #{GnuplotHelpers.nice_tick_step(0.0, (nx - 1).to_f, 10)}
    set ytics #{GnuplotHelpers.nice_tick_step(0.0, (ny - 1).to_f, 10)}

    set view map
    set pm3d map
    set palette rgb 33,13,10

    # Temperature range used by the boundary conditions.
    set cbrange [0:100]
    set cblabel "Temperature" font "Arial,30"

    splot "#{GnuplotHelpers.gp_escape(grid_dat)}" using 1:2:3 with pm3d
  GP

  GnuplotHelpers.run_gnuplot!(script, out_dir: out_dir, script_name: "heatmap_plot.gp")
end

# Create a high-quality line plot (x,y) via gnuplot.
def save_line_plot_png!(out_png, x, y, title_text, xlabel_text, ylabel_text)
  out_dir = File.dirname(out_png)
  dat = File.join(out_dir, "tmp_line_data.dat")

  File.open(dat, "w") do |f|
    x.length.times { |k| f.puts "#{x[k]} #{y[k]}" }
  end

  xmin = x.min
  xmax = x.max
  ymin = y.min
  ymax = y.max

  script = <<~GP
    #{GnuplotHelpers.common_png_header(out_png, width_px: 2400, height_px: 1600, font: "Arial,28", line_width: 3)}

    set title "#{title_text}" font "Arial,38"
    set xlabel "#{xlabel_text}" font "Arial,34"
    set ylabel "#{ylabel_text}" font "Arial,34"

    # Limit tick count for readability.
    set xtics #{GnuplotHelpers.nice_tick_step(xmin, xmax, 10)}
    set ytics #{GnuplotHelpers.nice_tick_step(ymin, ymax, 10)}

    plot "#{GnuplotHelpers.gp_escape(dat)}" using 1:2 with lines linestyle 1
  GP

  GnuplotHelpers.run_gnuplot!(script, out_dir: out_dir, script_name: "line_plot.gp")
  FileUtils.rm_f(dat)
end

# ------------------------------------------------------------
# Main program
# ------------------------------------------------------------
out_dir = "output/heat2d"
FileUtils.mkdir_p(out_dir)

# ------------------------------------------------------------
# Physical parameters
# ------------------------------------------------------------
alpha = 1.0 # thermal diffusivity (chosen for demonstrative speed)

# ------------------------------------------------------------
# Domain and grid
# ------------------------------------------------------------
lx = 1.0
ly = 1.0

# Grid resolution (increase for smoother but slower simulations)
nx = 81
ny = 81

dx = lx / (nx - 1).to_f
dy = ly / (ny - 1).to_f

# ------------------------------------------------------------
# Time step selection (explicit stability)
# ------------------------------------------------------------
# For the 2D explicit heat equation (FTCS), a standard stability constraint is:
#
#   dt <= 1 / ( 2*α*( 1/dx^2 + 1/dy^2 ) )
#
# We take a conservative fraction of this limit.
dt_stable = 1.0 / (2.0 * alpha * (1.0 / (dx * dx) + 1.0 / (dy * dy)))
dt = 0.80 * dt_stable

t_end = 0.20
nsteps = (t_end / dt).ceil

# Save a heatmap snapshot every N steps (avoid generating too many PNG files).
desired_snapshots = 30
snapshot_every = [1, (nsteps / desired_snapshots)].max

# ------------------------------------------------------------
# Allocate temperature fields (flattened)
# ------------------------------------------------------------
t_field = Array.new(nx * ny, 0.0)
t_new   = Array.new(nx * ny, 0.0)

# ------------------------------------------------------------
# Boundary conditions (Dirichlet)
# ------------------------------------------------------------
# Left edge is hot; other edges cold.
t_left   = 100.0
t_right  = 0.0
t_top    = 0.0
t_bottom = 0.0

# Apply boundary values to initial condition.
(0...ny).each do |j|
  t_field[idx(0, j, nx)]       = t_left
  t_field[idx(nx - 1, j, nx)]  = t_right
end
(0...nx).each do |i|
  t_field[idx(i, 0, nx)]       = t_bottom
  t_field[idx(i, ny - 1, nx)]  = t_top
end

# ------------------------------------------------------------
# Logging (temperature at center over time)
# ------------------------------------------------------------
ic = nx / 2
jc = ny / 2

time_log = []
center_t_log = []

# ------------------------------------------------------------
# Time integration loop
# ------------------------------------------------------------
t = 0.0

(0...nsteps).each do |step|
  # Log center temperature.
  time_log << t
  center_t_log << t_field[idx(ic, jc, nx)]

  # Update interior nodes only (boundaries remain fixed).
  j = 1
  while j < ny - 1
    i = 1
    while i < nx - 1
      # Second derivative in x (central difference).
      txx = (t_field[idx(i + 1, j, nx)] - 2.0 * t_field[idx(i, j, nx)] + t_field[idx(i - 1, j, nx)]) / (dx * dx)

      # Second derivative in y (central difference).
      tyy = (t_field[idx(i, j + 1, nx)] - 2.0 * t_field[idx(i, j, nx)] + t_field[idx(i, j - 1, nx)]) / (dy * dy)

      # Explicit FTCS update.
      t_new[idx(i, j, nx)] = t_field[idx(i, j, nx)] + alpha * dt * (txx + tyy)

      i += 1
    end
    j += 1
  end

  # Re-apply Dirichlet boundaries to keep them fixed exactly.
  (0...ny).each do |jj|
    t_new[idx(0, jj, nx)]      = t_left
    t_new[idx(nx - 1, jj, nx)] = t_right
  end
  (0...nx).each do |ii|
    t_new[idx(ii, 0, nx)]      = t_bottom
    t_new[idx(ii, ny - 1, nx)] = t_top
  end

  # Swap buffers for next iteration.
  t_field, t_new = t_new, t_field

  # Save snapshot occasionally.
  if (step % snapshot_every).zero?
    grid_dat = File.join(out_dir, format("heat_t%05d.dat", step))
    write_pm3d_grid(grid_dat, t_field, nx, ny)

    png_name = File.join(out_dir, format("heat_t%05d.png", step))
    save_heatmap_png!(png_name, grid_dat, "2D Heat Equation - Temperature Field", nx, ny)

    puts "Saved snapshot: #{png_name}"
    FileUtils.rm_f(grid_dat)
  end

  t += dt
end

# ------------------------------------------------------------
# Final plots: centerline temperature and center point vs time
# ------------------------------------------------------------

# Physical x-coordinate values and centerline values.
x_vals = Array.new(nx, 0.0)
centerline = Array.new(nx, 0.0)
(0...nx).each do |i|
  x_vals[i] = i * dx
  centerline[i] = t_field[idx(i, jc, nx)]
end

save_line_plot_png!(
  File.join(out_dir, "centerline_final.png"),
  x_vals,
  centerline,
  "Final Centerline Temperature (y = 0.5)",
  "x (m)",
  "T(x, y=0.5)"
)

save_line_plot_png!(
  File.join(out_dir, "center_point_vs_time.png"),
  time_log,
  center_t_log,
  "Temperature at Plate Center vs Time",
  "time (s)",
  "T(center)"
)

# ------------------------------------------------------------
# Save CSV log of center temperature
# ------------------------------------------------------------
begin
  CsvWriter.write_two_columns(File.join(out_dir, "heat2d_log.csv"), "t", "T_center", time_log, center_t_log)
rescue StandardError => e
  warn e.message
end

puts "Heat2D finished. Results are in: #{out_dir}"
