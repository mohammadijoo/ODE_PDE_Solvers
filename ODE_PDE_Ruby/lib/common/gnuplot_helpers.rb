# frozen_string_literal: true

require "fileutils"
require_relative "math_utils"

# ------------------------------------------------------------
# Gnuplot helper utilities:
#   - Generates high-quality PNG plots (300-dpi-class, large fonts)
#   - Enforces readable tick formatting:
#       * English digits (C locale)
#       * One decimal place on tick labels
#       * No more than ~10 tick marks (configurable)
# ------------------------------------------------------------
module GnuplotHelpers
  module_function

  # Convert a filesystem path into a gnuplot-friendly form:
  # - use forward slashes (works on Windows too)
  # - escape double quotes
  def gp_escape(path)
    path.to_s.tr("\\", "/").gsub('"', '\"')
  end


  # Compute a "nice" tick step so the number of labeled ticks is not excessive.
  #
  # The returned step is selected from the set:
  #   {1, 2, 2.5, 5} * 10^k
  #
  # max_ticks includes both endpoints in a typical plotting scenario.
  def nice_tick_step(min_val, max_val, max_ticks = 10)
    span = (max_val - min_val).abs
    return 1.0 if span <= 0.0

    target = span / [max_ticks - 1, 1].max.to_f

    # Determine order of magnitude.
    exp = Math.log10(target).floor
    base = 10.0**exp
    frac = target / base

    candidates = [1.0, 2.0, 2.5, 5.0, 10.0]
    chosen = candidates.find { |c| c >= frac } || 10.0

    chosen * base
  end

  # Run gnuplot with an in-memory script, writing to a .gp file for reproducibility.
  #
  # - out_dir: directory to place the .gp script
  # - script_name: filename for the script (e.g., "plot_cart_position.gp")
  def run_gnuplot!(script_text, out_dir:, script_name:)
    FileUtils.mkdir_p(out_dir)
    gp_path = File.join(out_dir, script_name)
    File.write(gp_path, script_text)

    # Use gnuplot from PATH.
    ok = system("gnuplot", gp_path)
    raise "Gnuplot failed. Ensure 'gnuplot' is installed and available on PATH." unless ok
  end

  # Header snippet used in all plot scripts:
  # - pngcairo terminal: anti-aliased text, good fonts
  # - high pixel resolution to approximate 300 dpi for typical figure sizes
  # - large fonts and thick lines for publication-style output
  def common_png_header(output_png, width_px: 2400, height_px: 1600, font: "Arial,28", line_width: 3)
    <<~GP
      # ----------------------------
      # Global style / quality
      # ----------------------------
      set term pngcairo size #{width_px},#{height_px} font "#{font}" linewidth #{line_width} enhanced
      set output "#{gp_escape(output_png)}"

      # Enforce English numeric formatting regardless of OS locale.
      set locale "C"
      set decimalsign "."

      # Tick label formatting: one decimal place.
      set format x "%.1f"
      set format y "%.1f"

      # Padding / margins (screen coordinates keep it consistent).
      # These values ensure there is visible whitespace around the plot region.
      set lmargin at screen 0.10
      set rmargin at screen 0.97
      set bmargin at screen 0.12
      set tmargin at screen 0.92

      # Grid improves readability for analysis plots.
      set grid

      # Disable the legend (key) so plots remain clean and uncluttered.
      # This prevents gnuplot from auto-generating labels like:
      #   "output/.../something.dat" using 1:2
      unset key

      # Use a consistent thick line style.
      set style line 1 lw 5
    GP
  end
end
