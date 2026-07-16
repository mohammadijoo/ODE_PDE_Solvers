// ------------------------------------------------------------
// 2D Heat Equation (Heat Transfer by Conduction) in C#
// ------------------------------------------------------------
// PDE:
//   ∂T/∂t = α ( ∂²T/∂x² + ∂²T/∂y² )
//
// Method:
//   - 2D explicit finite-difference scheme (FTCS)
//   - Dirichlet boundary conditions (fixed temperature on boundaries)
//   - Snapshots saved as PNG heatmaps (high resolution)
//   - Additional plots saved (center point vs time, final centerline)
//   - CSV log saved (center temperature vs time)
//
// Output folder (relative to where you run the program):
//   output/heat2d/
//
// Export quality:
//   - High pixel resolution
//   - PNG is rewritten to embed 300 DPI metadata safely (no cropping)
// ------------------------------------------------------------

using System.Globalization;
using System.Drawing;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using System.Threading;
using ScottPlot;

using DrawingImageFormat = System.Drawing.Imaging.ImageFormat;

internal static class Program
{
    // Flatten 2D indexing (i, j) into a 1D array index where nx is the number of x columns.
    private static int Idx(int i, int j, int nx) => j * nx + i;

    // Write a two-column CSV file with a header row.
    private static void WriteCsvTwoColumns(
        string filename,
        string h1,
        string h2,
        IReadOnlyList<double> c1,
        IReadOnlyList<double> c2)
    {
        if (c1.Count != c2.Count)
            throw new InvalidOperationException("CSV write error: column sizes do not match.");

        var ci = CultureInfo.InvariantCulture;

        using var sw = new StreamWriter(filename);
        sw.WriteLine($"{h1},{h2}");

        for (int k = 0; k < c1.Count; k++)
            sw.WriteLine($"{c1[k].ToString("G17", ci)},{c2[k].ToString("G17", ci)}");
    }

    // Apply export-related plot settings (improves readability for high-resolution images).
    private static void ConfigurePlotForExport(ScottPlot.Plot plt)
    {
        // ScaleFactor increases text/line sizes relative to pixel dimensions.
        // This helps avoid tiny labels on large images.
        plt.ScaleFactor = 2;

        // Keep axes/ticks crisp when ScaleFactor is increased.
        plt.Axes.Hairline(true);
    }

    // Save a ScottPlot plot as a high-resolution PNG, then embed 300 DPI metadata safely.
    // The DPI embedding is done by cloning pixel data 1:1 (no Graphics.DrawImage),
    // which avoids DPI-based scaling/cropping.
    private static void SavePlotPng300Dpi(ScottPlot.Plot plt, string path, int widthPx, int heightPx)
    {
        string? dir = Path.GetDirectoryName(path);
        if (!string.IsNullOrWhiteSpace(dir))
            Directory.CreateDirectory(dir);

        ConfigurePlotForExport(plt);

        // 1) Save the plot PNG (ScottPlot writes the pixels).
        plt.SavePng(path, widthPx, heightPx);

        // 2) Rewrite the PNG with 300 DPI metadata safely (temp file + overwrite).
        //    If rewriting fails (file locked), keep the original image.
        try
        {
            string tmpPath = Path.Combine(
                dir ?? ".",
                Path.GetFileNameWithoutExtension(path) + $".__tmp__{Guid.NewGuid():N}.png");

            const int maxAttempts = 10;
            const int sleepMs = 50;

            for (int attempt = 1; attempt <= maxAttempts; attempt++)
            {
                try
                {
                    using var fs = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.ReadWrite);
                    using var loaded = new Bitmap(fs);

                    // Clone pixel data exactly (no DPI scaling side effects).
                    using var clone = loaded.Clone(
                        new Rectangle(0, 0, loaded.Width, loaded.Height),
                        PixelFormat.Format32bppArgb);

                    // Set DPI metadata (does not change pixels).
                    clone.SetResolution(300f, 300f);

                    clone.Save(tmpPath, DrawingImageFormat.Png);

                    File.Copy(tmpPath, path, overwrite: true);
                    File.Delete(tmpPath);
                    break;
                }
                catch (IOException) when (attempt < maxAttempts)
                {
                    Thread.Sleep(sleepMs);
                }
                catch (ExternalException) when (attempt < maxAttempts)
                {
                    Thread.Sleep(sleepMs);
                }
            }
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine($"Warning: Failed to embed 300 DPI metadata for '{path}'. Keeping original PNG. Details: {ex.Message}");
        }
    }

    public static void Main()
    {
        // ------------------------------------------------------------
        // Output directory setup
        // ------------------------------------------------------------
        string outDir = Path.Combine("output", "heat2d");
        Directory.CreateDirectory(outDir);

        // ------------------------------------------------------------
        // Physical parameters
        // ------------------------------------------------------------
        double alpha = 1.0; // thermal diffusivity

        // ------------------------------------------------------------
        // Domain and grid
        // ------------------------------------------------------------
        double Lx = 1.0;
        double Ly = 1.0;

        int nx = 81;
        int ny = 81;

        double dx = Lx / (nx - 1);
        double dy = Ly / (ny - 1);

        // ------------------------------------------------------------
        // Time step selection (explicit stability)
        // ------------------------------------------------------------
        double dtStable = 1.0 / (2.0 * alpha * (1.0 / (dx * dx) + 1.0 / (dy * dy)));
        double dt = 0.80 * dtStable;

        double tEnd = 0.20;
        int nSteps = (int)Math.Ceiling(tEnd / dt);

        int desiredSnapshots = 30;
        int snapshotEvery = Math.Max(1, nSteps / desiredSnapshots);

        // ------------------------------------------------------------
        // Allocate temperature fields (flattened nx*ny arrays)
        // ------------------------------------------------------------
        double[] T = new double[nx * ny];
        double[] TNew = new double[nx * ny];

        // ------------------------------------------------------------
        // Boundary conditions (Dirichlet)
        // ------------------------------------------------------------
        double TLeft = 100.0;
        double TRight = 0.0;
        double TTop = 0.0;
        double TBottom = 0.0;

        // Apply boundary values to initial condition.
        for (int j = 0; j < ny; j++)
        {
            T[Idx(0, j, nx)] = TLeft;
            T[Idx(nx - 1, j, nx)] = TRight;
        }
        for (int i = 0; i < nx; i++)
        {
            T[Idx(i, 0, nx)] = TBottom;
            T[Idx(i, ny - 1, nx)] = TTop;
        }

        // ------------------------------------------------------------
        // Logging (temperature at center over time)
        // ------------------------------------------------------------
        int ic = nx / 2;
        int jc = ny / 2;

        var timeLog = new List<double>(capacity: nSteps);
        var centerTLog = new List<double>(capacity: nSteps);

        // ------------------------------------------------------------
        // Time integration loop
        // ------------------------------------------------------------
        double t = 0.0;

        for (int step = 0; step < nSteps; step++)
        {
            timeLog.Add(t);
            centerTLog.Add(T[Idx(ic, jc, nx)]);

            // Update only the interior nodes.
            for (int j = 1; j < ny - 1; j++)
            {
                for (int i = 1; i < nx - 1; i++)
                {
                    double Txx =
                        (T[Idx(i + 1, j, nx)] - 2.0 * T[Idx(i, j, nx)] + T[Idx(i - 1, j, nx)]) / (dx * dx);

                    double Tyy =
                        (T[Idx(i, j + 1, nx)] - 2.0 * T[Idx(i, j, nx)] + T[Idx(i, j - 1, nx)]) / (dy * dy);

                    TNew[Idx(i, j, nx)] = T[Idx(i, j, nx)] + alpha * dt * (Txx + Tyy);
                }
            }

            // Re-apply Dirichlet boundaries.
            for (int j = 0; j < ny; j++)
            {
                TNew[Idx(0, j, nx)] = TLeft;
                TNew[Idx(nx - 1, j, nx)] = TRight;
            }
            for (int i = 0; i < nx; i++)
            {
                TNew[Idx(i, 0, nx)] = TBottom;
                TNew[Idx(i, ny - 1, nx)] = TTop;
            }

            // Swap buffers.
            (T, TNew) = (TNew, T);

            // Save heatmap snapshots.
            if (step % snapshotEvery == 0)
            {
                double[,] M = new double[ny, nx];
                for (int y = 0; y < ny; y++)
                    for (int x = 0; x < nx; x++)
                        M[y, x] = T[Idx(x, y, nx)];

                var plt = new ScottPlot.Plot();
                plt.Title("2D Heat Equation - Temperature Field");
                plt.XLabel("x index");
                plt.YLabel("y index");

                plt.Add.Heatmap(M);

                string pngName = Path.Combine(outDir, $"heat_t{step}.png");
                SavePlotPng300Dpi(plt, pngName, widthPx: 2400, heightPx: 1800);

                Console.WriteLine($"Saved snapshot: {pngName}");
            }

            t += dt;
        }

        // ------------------------------------------------------------
        // Final plots
        // ------------------------------------------------------------
        double[] xAxis = new double[nx];
        double[] centerline = new double[nx];

        for (int i = 0; i < nx; i++)
        {
            xAxis[i] = i * dx;
            centerline[i] = T[Idx(i, jc, nx)];
        }

        // Final centerline plot.
        {
            var plt = new ScottPlot.Plot();
            plt.Title("Final Centerline Temperature (y = 0.5)");
            plt.XLabel("x (m)");
            plt.YLabel("T(x, y=0.5)");
            plt.Add.Scatter(xAxis, centerline);

            SavePlotPng300Dpi(plt, Path.Combine(outDir, "centerline_final.png"), 2600, 1600);
        }

        // Center point vs time plot.
        {
            var plt = new ScottPlot.Plot();
            plt.Title("Temperature at Plate Center vs Time");
            plt.XLabel("time (s)");
            plt.YLabel("T(center)");
            plt.Add.Scatter(timeLog.ToArray(), centerTLog.ToArray());

            SavePlotPng300Dpi(plt, Path.Combine(outDir, "center_point_vs_time.png"), 2600, 1600);
        }

        // CSV log.
        try
        {
            WriteCsvTwoColumns(
                filename: Path.Combine(outDir, "heat2d_log.csv"),
                h1: "t",
                h2: "T_center",
                c1: timeLog,
                c2: centerTLog);
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine(ex.Message);
        }

        Console.WriteLine($"Heat2D finished. Results are in: {outDir}");
    }
}
