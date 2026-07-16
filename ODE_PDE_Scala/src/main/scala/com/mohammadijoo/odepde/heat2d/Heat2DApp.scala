package com.mohammadijoo.odepde.heat2d

import com.mohammadijoo.odepde.common.{CsvUtils, PlotUtils}

import java.nio.file.{Files, Path, Paths}
import java.util.Locale
import scala.math.{ceil, max}

/**
  * ------------------------------------------------------------
  * 2D Heat Equation (Heat Transfer by Conduction)
  * ------------------------------------------------------------
  *
  * PDE:
  *   ∂T/∂t = α ( ∂²T/∂x² + ∂²T/∂y² )
  *
  * Method:
  *   - 2D explicit finite-difference scheme (FTCS)
  *   - Dirichlet boundary conditions (fixed temperature on boundaries)
  *   - Snapshots saved as PNG heatmaps
  *   - Additional plots saved (center point vs time, final centerline)
  *
  * Output folder (relative to where you run the program):
  *   output/heat2d/
  * ------------------------------------------------------------
  */
object Heat2DApp {

  /**
    * Flattens 2D indexing (i, j) into a 1D array index for row-major storage:
    *   index = j*nx + i
    */
  private def idx(i: Int, j: Int, nx: Int): Int = j * nx + i

  def main(args: Array[String]): Unit = {
    // Ensure plots use English/Latin digits regardless of the Windows display language.
    Locale.setDefault(Locale.US)

    // ------------------------------------------------------------
    // Output directory setup
    // ------------------------------------------------------------
    val outDir: Path = Paths.get("output", "heat2d")
    Files.createDirectories(outDir)

    // ------------------------------------------------------------
    // Physical parameters
    // ------------------------------------------------------------
    val alpha = 1.0 // thermal diffusivity (chosen for demonstrative speed)

    // ------------------------------------------------------------
    // Domain and grid
    // ------------------------------------------------------------
    val Lx = 1.0
    val Ly = 1.0

    // Grid resolution (increase for smoother but slower simulations)
    val nx = 81
    val ny = 81

    val dx = Lx / (nx - 1).toDouble
    val dy = Ly / (ny - 1).toDouble

    // ------------------------------------------------------------
    // Time step selection (explicit stability)
    // ------------------------------------------------------------
    // For the 2D explicit heat equation (FTCS), a standard stability constraint is:
    //
    //   dt <= 1 / ( 2*α*( 1/dx^2 + 1/dy^2 ) )
    //
    // We take a conservative fraction of this limit.
    val dtStable = 1.0 / (2.0 * alpha * (1.0 / (dx * dx) + 1.0 / (dy * dy)))
    val dt = 0.80 * dtStable // conservative choice

    val tEnd = 0.20 // total simulation time
    val nSteps = ceil(tEnd / dt).toInt

    // Save a heatmap snapshot every N steps.
    // Target a reasonable number of snapshots so we do not create thousands of PNG files.
    val desiredSnapshots = 30
    val snapshotEvery = max(1, nSteps / desiredSnapshots)

    // ------------------------------------------------------------
    // Allocate temperature fields
    // ------------------------------------------------------------
    // Temperature is stored as a flattened 2D field of size nx*ny.
    val T = Array.fill[Double](nx * ny)(0.0)
    val Tnew = Array.fill[Double](nx * ny)(0.0)

    // ------------------------------------------------------------
    // Boundary conditions (Dirichlet)
    // ------------------------------------------------------------
    // Left edge is hot; other edges cold.
    val Tleft = 100.0
    val Tright = 0.0
    val Ttop = 0.0
    val Tbottom = 0.0

    // Apply boundary values to initial condition
    var j = 0
    while (j < ny) {
      T(idx(0, j, nx)) = Tleft
      T(idx(nx - 1, j, nx)) = Tright
      j += 1
    }

    var i = 0
    while (i < nx) {
      T(idx(i, 0, nx)) = Tbottom
      T(idx(i, ny - 1, nx)) = Ttop
      i += 1
    }

    // ------------------------------------------------------------
    // Logging (temperature at center over time)
    // ------------------------------------------------------------
    val ic = nx / 2
    val jc = ny / 2

    val timeLog = collection.mutable.ArrayBuffer.empty[Double]
    val centerTLog = collection.mutable.ArrayBuffer.empty[Double]

    // ------------------------------------------------------------
    // Time integration loop
    // ------------------------------------------------------------
    var t = 0.0
    var step = 0

    while (step < nSteps) {
      // Log center temperature
      timeLog += t
      centerTLog += T(idx(ic, jc, nx))

      // Update only the interior nodes (boundaries remain fixed)
      j = 1
      while (j < ny - 1) {
        i = 1
        while (i < nx - 1) {
          // Second derivative in x (central difference)
          val Txx =
            (T(idx(i + 1, j, nx)) - 2.0 * T(idx(i, j, nx)) + T(idx(i - 1, j, nx))) / (dx * dx)

          // Second derivative in y (central difference)
          val Tyy =
            (T(idx(i, j + 1, nx)) - 2.0 * T(idx(i, j, nx)) + T(idx(i, j - 1, nx))) / (dy * dy)

          // Explicit FTCS update
          Tnew(idx(i, j, nx)) = T(idx(i, j, nx)) + alpha * dt * (Txx + Tyy)
          i += 1
        }
        j += 1
      }

      // Re-apply Dirichlet boundaries to Tnew so they remain fixed exactly
      j = 0
      while (j < ny) {
        Tnew(idx(0, j, nx)) = Tleft
        Tnew(idx(nx - 1, j, nx)) = Tright
        j += 1
      }

      i = 0
      while (i < nx) {
        Tnew(idx(i, 0, nx)) = Tbottom
        Tnew(idx(i, ny - 1, nx)) = Ttop
        i += 1
      }

      // Swap buffers for the next iteration by copying references
      // (Scala arrays are mutable references, so we swap element-wise to keep allocations stable)
      var k = 0
      while (k < T.length) {
        val tmp = T(k)
        T(k) = Tnew(k)
        Tnew(k) = tmp
        k += 1
      }

      // --------------------------------------------------------
      // Save a heatmap snapshot occasionally
      // --------------------------------------------------------
      if (step % snapshotEvery == 0) {
        val pngName = outDir.resolve(f"heat_t$step%04d.png")
        PlotUtils.saveHeatmap(
          title = "2D Heat Equation - Temperature Field",
          xlabel = "x index",
          ylabel = "y index",
          values = T,
          nx = nx,
          ny = ny,
          outFile = pngName
        )
        println(s"Saved snapshot: $pngName")
      }

      // Advance time
      t += dt
      step += 1
    }

    // ------------------------------------------------------------
    // Final plots: centerline temperature and center point vs time
    // ------------------------------------------------------------
    // Create x-axis values (physical coordinates) and centerline values
    val x = (0 until nx).map(ii => ii.toDouble * dx)
    val centerline = (0 until nx).map(ii => T(idx(ii, jc, nx)))

    // Plot final centerline temperature
    PlotUtils.saveLinePlot(
      title = "Final Centerline Temperature (y = 0.5)",
      xlabel = "x (m)",
      ylabel = "T(x, y=0.5)",
      x = x,
      y = centerline,
      outFile = outDir.resolve("centerline_final.png")
    )

    // Plot center point temperature vs time
    PlotUtils.saveLinePlot(
      title = "Temperature at Plate Center vs Time",
      xlabel = "time (s)",
      ylabel = "T(center)",
      x = timeLog.toSeq,
      y = centerTLog.toSeq,
      outFile = outDir.resolve("center_point_vs_time.png")
    )

    // ------------------------------------------------------------
    // Save CSV log of center temperature
    // ------------------------------------------------------------
    try {
      CsvUtils.writeTwoColumns(
        file = outDir.resolve("heat2d_log.csv"),
        h1 = "t",
        h2 = "T_center",
        c1 = timeLog.toSeq,
        c2 = centerTLog.toSeq
      )
    } catch {
      case e: Exception =>
        System.err.println(e.getMessage)
    }

    println(s"Heat2D finished. Results are in: $outDir")
  }
}
