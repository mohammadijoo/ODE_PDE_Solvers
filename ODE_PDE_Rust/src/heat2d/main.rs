// ------------------------------------------------------------
// 2D Heat Equation (Heat Transfer by Conduction) in Rust
// ------------------------------------------------------------
// PDE:
//   ∂T/∂t = α ( ∂²T/∂x² + ∂²T/∂y² )
//
// Method:
//   - 2D explicit finite-difference scheme (FTCS)
//   - Dirichlet boundary conditions (fixed temperature on boundaries)
//   - Snapshots saved as PNG heatmaps (high-resolution)
//   - Additional plots saved (center point vs time, final centerline)
//
// Output folder (relative to where you run the program):
//   output/heat2d/
//
// Notes:
//   - Plot images are saved at high pixel resolution (suitable for ~300 DPI usage).
//   - Axis tick labels are formatted in English with one decimal digit.
// ------------------------------------------------------------

use anyhow::{Context, Result};
use plotters::prelude::*;
use std::fs;
use std::path::Path;

// Flatten (i, j) into a single index for a row-major 2D field of size nx * ny.
fn idx(i: usize, j: usize, nx: usize) -> usize {
    j * nx + i
}

// Write a simple two-column CSV file with a header.
fn write_csv_two_columns(
    filename: &str,
    h1: &str,
    h2: &str,
    c1: &[f64],
    c2: &[f64],
) -> Result<()> {
    if c1.len() != c2.len() {
        anyhow::bail!("CSV write error: column sizes do not match.");
    }

    let mut wtr = csv::Writer::from_path(filename)
        .with_context(|| format!("CSV write error: cannot open file: {}", filename))?;

    wtr.write_record([h1, h2])?;
    for k in 0..c1.len() {
        wtr.write_record([c1[k].to_string(), c2[k].to_string()])?;
    }
    wtr.flush()?;
    Ok(())
}

// Map a scalar value in [vmin, vmax] to an RGB color.
// This uses a simple blue→cyan→green→yellow→red gradient for readability.
fn colormap(value: f64, vmin: f64, vmax: f64) -> RGBColor {
    let mut t = (value - vmin) / (vmax - vmin);
    if !t.is_finite() {
        t = 0.0;
    }
    t = t.clamp(0.0, 1.0);

    // Piecewise linear gradient across 4 segments.
    // 0.00: blue   (0,0,255)
    // 0.25: cyan   (0,255,255)
    // 0.50: green  (0,255,0)
    // 0.75: yellow (255,255,0)
    // 1.00: red    (255,0,0)
    let (r, g, b) = if t < 0.25 {
        let a = t / 0.25;
        (0.0, 255.0 * a, 255.0)
    } else if t < 0.50 {
        let a = (t - 0.25) / 0.25;
        (0.0, 255.0, 255.0 * (1.0 - a))
    } else if t < 0.75 {
        let a = (t - 0.50) / 0.25;
        (255.0 * a, 255.0, 0.0)
    } else {
        let a = (t - 0.75) / 0.25;
        (255.0, 255.0 * (1.0 - a), 0.0)
    };

    RGBColor(r.round() as u8, g.round() as u8, b.round() as u8)
}

// Save a heatmap snapshot as a PNG using Plotters.
// The heatmap is rendered as filled rectangles over a (nx, ny) index grid.
fn save_heatmap_png(
    filename: &str,
    t: &[f64],
    nx: usize,
    ny: usize,
    title: &str,
) -> Result<()> {
    // High resolution output (suitable for ~300 DPI usage when printed).
    let (w, h) = (2400u32, 1800u32);

    // Create bitmap backend and drawing area.
    let root = BitMapBackend::new(filename, (w, h)).into_drawing_area();
    root.fill(&RGBColor(255, 255, 255))?;

    // Add padding around the plot area.
    let mut chart = ChartBuilder::on(&root)
        .margin(20)
        .caption(title, ("sans-serif", 76))
        .x_label_area_size(110)
        .y_label_area_size(110)
        .build_cartesian_2d(0f64..(nx as f64), 0f64..(ny as f64))?;

    // Mesh configuration:
    // - Large fonts
    // - Thick grid lines
    // - ≤ 10 tick labels per axis
    // - English tick formatting with one decimal digit
    chart
        .configure_mesh()
        .x_desc("x index")
        .y_desc("y index")
        .axis_desc_style(("sans-serif", 60))
        .label_style(("sans-serif", 44))
        .x_labels(10)
        .y_labels(10)
        .x_label_formatter(&|v| format!("{:.1}", v))
        .y_label_formatter(&|v| format!("{:.1}", v))
        .bold_line_style(RGBColor(160, 160, 160).stroke_width(2))
        .light_line_style(RGBColor(220, 220, 220).stroke_width(1))
        .draw()?;

    // Determine color scale limits (min/max from current field).
    let (mut vmin, mut vmax) = (f64::INFINITY, f64::NEG_INFINITY);
    for &val in t {
        vmin = vmin.min(val);
        vmax = vmax.max(val);
    }
    if (vmax - vmin).abs() < 1e-12 {
        vmax = vmin + 1.0;
    }

    // Draw each cell as a filled rectangle.
    // Using a dense grid is acceptable here because nx, ny are modest.
    chart.draw_series((0..ny).flat_map(|j| {
        (0..nx).map(move |i| {
            let v = t[idx(i, j, nx)];
            let c = colormap(v, vmin, vmax);

            // Rectangle spans [i, i+1] and [j, j+1] in index space.
            Rectangle::new(
                [(i as f64, j as f64), (i as f64 + 1.0, j as f64 + 1.0)],
                c.filled(),
            )
        })
    }))?;

    root.present()?;
    Ok(())
}

// Save a single line plot (x vs y) with consistent high-quality styling.
fn save_line_plot_png(
    filename: &str,
    title: &str,
    xlabel: &str,
    ylabel: &str,
    x: &[f64],
    y: &[f64],
) -> Result<()> {
    if x.len() != y.len() {
        anyhow::bail!("Plot error: x and y must have the same length.");
    }

    // High resolution output.
    let (w, h) = (2400u32, 1800u32);

    // Determine data bounds with a small margin.
    let (mut xmin, mut xmax) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut ymin, mut ymax) = (f64::INFINITY, f64::NEG_INFINITY);
    for k in 0..x.len() {
        xmin = xmin.min(x[k]);
        xmax = xmax.max(x[k]);
        ymin = ymin.min(y[k]);
        ymax = ymax.max(y[k]);
    }

    // Add a 5% padding to y-range to avoid clipping.
    let ypad = 0.05 * (ymax - ymin).abs().max(1e-9);
    ymin -= ypad;
    ymax += ypad;

    let root = BitMapBackend::new(filename, (w, h)).into_drawing_area();
    root.fill(&RGBColor(255, 255, 255))?;

    let mut chart = ChartBuilder::on(&root)
        .margin(20)
        .caption(title, ("sans-serif", 76))
        .x_label_area_size(110)
        .y_label_area_size(110)
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

    // Draw the polyline with a thicker stroke for readability.
    chart.draw_series(LineSeries::new(
        x.iter().copied().zip(y.iter().copied()),
        RGBColor(30, 90, 200).stroke_width(4),
    ))?;

    root.present()?;
    Ok(())
}

fn main() -> Result<()> {
    // ------------------------------------------------------------
    // Output directory setup
    // ------------------------------------------------------------
    let out_dir = Path::new("output").join("heat2d");
    fs::create_dir_all(&out_dir).context("Failed to create output directory")?;

    // ------------------------------------------------------------
    // Physical parameters
    // ------------------------------------------------------------
    let alpha: f64 = 1.0; // thermal diffusivity (chosen for demonstrative speed)

    // ------------------------------------------------------------
    // Domain and grid
    // ------------------------------------------------------------
    let lx: f64 = 1.0;
    let ly: f64 = 1.0;

    // Grid resolution (increase for smoother but slower simulations)
    let nx: usize = 81;
    let ny: usize = 81;

    let dx: f64 = lx / (nx as f64 - 1.0);
    let dy: f64 = ly / (ny as f64 - 1.0);

    // ------------------------------------------------------------
    // Time step selection (explicit stability)
    // ------------------------------------------------------------
    // For the 2D explicit heat equation (FTCS), a standard stability constraint is:
    //
    //   dt <= 1 / ( 2*α*( 1/dx^2 + 1/dy^2 ) )
    //
    // We take a conservative fraction of this limit.
    let dt_stable: f64 = 1.0 / (2.0 * alpha * (1.0 / (dx * dx) + 1.0 / (dy * dy)));
    let dt: f64 = 0.80 * dt_stable;

    let t_end: f64 = 0.20;
    let nsteps: usize = (t_end / dt).ceil() as usize;

    // Save a heatmap snapshot every N steps.
    let desired_snapshots: usize = 30;
    let snapshot_every: usize = (nsteps / desired_snapshots).max(1);

    // ------------------------------------------------------------
    // Allocate temperature fields
    // ------------------------------------------------------------
    // Store T as a flattened 2D field of size nx * ny.
    let mut t_field = vec![0.0_f64; nx * ny];
    let mut t_new = vec![0.0_f64; nx * ny];

    // ------------------------------------------------------------
    // Boundary conditions (Dirichlet)
    // ------------------------------------------------------------
    // Left edge is hot; other edges cold.
    let t_left = 100.0;
    let t_right = 0.0;
    let t_top = 0.0;
    let t_bottom = 0.0;

    // Apply boundary values to initial condition.
    for j in 0..ny {
        t_field[idx(0, j, nx)] = t_left;
        t_field[idx(nx - 1, j, nx)] = t_right;
    }
    for i in 0..nx {
        t_field[idx(i, 0, nx)] = t_bottom;
        t_field[idx(i, ny - 1, nx)] = t_top;
    }

    // ------------------------------------------------------------
    // Logging (temperature at center over time)
    // ------------------------------------------------------------
    let ic: usize = nx / 2;
    let jc: usize = ny / 2;

    let mut time_log: Vec<f64> = Vec::with_capacity(nsteps + 1);
    let mut center_t_log: Vec<f64> = Vec::with_capacity(nsteps + 1);

    // ------------------------------------------------------------
    // Time integration loop
    // ------------------------------------------------------------
    let mut t: f64 = 0.0;

    for step in 0..nsteps {
        // Log center temperature.
        time_log.push(t);
        center_t_log.push(t_field[idx(ic, jc, nx)]);

        // Update only interior nodes (boundaries remain fixed).
        for j in 1..(ny - 1) {
            for i in 1..(nx - 1) {
                // Second derivative in x (central difference).
                let txx = (t_field[idx(i + 1, j, nx)]
                    - 2.0 * t_field[idx(i, j, nx)]
                    + t_field[idx(i - 1, j, nx)])
                    / (dx * dx);

                // Second derivative in y (central difference).
                let tyy = (t_field[idx(i, j + 1, nx)]
                    - 2.0 * t_field[idx(i, j, nx)]
                    + t_field[idx(i, j - 1, nx)])
                    / (dy * dy);

                // Explicit FTCS update.
                t_new[idx(i, j, nx)] = t_field[idx(i, j, nx)] + alpha * dt * (txx + tyy);
            }
        }

        // Re-apply Dirichlet boundaries to keep them fixed exactly.
        for j in 0..ny {
            t_new[idx(0, j, nx)] = t_left;
            t_new[idx(nx - 1, j, nx)] = t_right;
        }
        for i in 0..nx {
            t_new[idx(i, 0, nx)] = t_bottom;
            t_new[idx(i, ny - 1, nx)] = t_top;
        }

        // Swap buffers for the next iteration.
        std::mem::swap(&mut t_field, &mut t_new);

        // --------------------------------------------------------
        // Save a heatmap snapshot occasionally
        // --------------------------------------------------------
        if step % snapshot_every == 0 {
            let png_name = out_dir.join(format!("heat_t{}.png", step));
            save_heatmap_png(
                png_name.to_string_lossy().as_ref(),
                &t_field,
                nx,
                ny,
                "2D Heat Equation - Temperature Field",
            )?;
            println!("Saved snapshot: {}", png_name.display());
        }

        // Advance time.
        t += dt;
    }

    // ------------------------------------------------------------
    // Final plots: centerline temperature and center point vs time
    // ------------------------------------------------------------

    // Create x-axis values (physical coordinates) and centerline values.
    let mut x = vec![0.0_f64; nx];
    let mut centerline = vec![0.0_f64; nx];
    for i in 0..nx {
        x[i] = i as f64 * dx;
        centerline[i] = t_field[idx(i, jc, nx)];
    }

    // Plot final centerline temperature.
    {
        let png = out_dir.join("centerline_final.png");
        save_line_plot_png(
            png.to_string_lossy().as_ref(),
            "Final Centerline Temperature (y = 0.5)",
            "x (m)",
            "T(x, y=0.5)",
            &x,
            &centerline,
        )?;
    }

    // Plot center point temperature vs time.
    {
        let png = out_dir.join("center_point_vs_time.png");
        save_line_plot_png(
            png.to_string_lossy().as_ref(),
            "Temperature at Plate Center vs Time",
            "time (s)",
            "T(center)",
            &time_log,
            &center_t_log,
        )?;
    }

    // ------------------------------------------------------------
    // Save CSV log of center temperature
    // ------------------------------------------------------------
    let csv_path = out_dir.join("heat2d_log.csv");
    if let Err(e) = write_csv_two_columns(
        csv_path.to_string_lossy().as_ref(),
        "t",
        "T_center",
        &time_log,
        &center_t_log,
    ) {
        eprintln!("{}", e);
    }

    println!("Heat2D finished. Results are in: {}", out_dir.display());
    Ok(())
}
