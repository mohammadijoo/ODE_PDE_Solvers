// ------------------------------------------------------------
// Cart–Pole Inverted Pendulum with Sliding Mode Control (SMC)
// ------------------------------------------------------------
// Requirements:
//   - Total simulation duration: 10 seconds
//   - Total frames: 6000 (600 FPS video)
//   - Exactly two disturbances at t=0.5s and t=5.0s
//   - Disturbance arrow visible for 0.5s only and constant length
//   - Dynamics integrated and used for rendering
//   - Outputs: frames, mp4 (ffmpeg if available), plots, CSV log
//
// Output:
//   output/pendulum_sliding_mode/frames/frame_000000.png ... frame_005999.png
//   output/pendulum_sliding_mode/pendulum_smc_10s_6000f.mp4 (if ffmpeg in PATH)
//   output/pendulum_sliding_mode/*.png plots
//   output/pendulum_sliding_mode/cartpole_log.csv
//
// Optional argument:
//   --preview  (shows a live preview window)
// ------------------------------------------------------------

using System.Diagnostics;
using System.Globalization;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using System.Threading;
using System.Windows.Forms;
using ScottPlot;

// Aliases to avoid type-name ambiguities (ScottPlot defines types with similar names)
using DrawingColor = System.Drawing.Color;
using DrawingImage = System.Drawing.Image;
using DrawingImageFormat = System.Drawing.Imaging.ImageFormat;

internal static class Program
{
    // ------------------------------------------------------------
    // Math helpers
    // ------------------------------------------------------------
    private static double Clamp(double x, double lo, double hi) => (x < lo) ? lo : (x > hi) ? hi : x;

    private static double WrapToPi(double a)
    {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    // Boundary-layer saturation for sliding mode: sat(z) in [-1, 1]
    private static double Sat(double z) => Clamp(z, -1.0, 1.0);

    // ------------------------------------------------------------
    // CSV writer
    // ------------------------------------------------------------
    private static void WriteCsv(string filename, string[] header, List<double>[] cols)
    {
        if (cols.Length == 0) throw new InvalidOperationException("CSV: no columns.");

        int n = cols[0].Count;
        for (int i = 1; i < cols.Length; i++)
            if (cols[i].Count != n)
                throw new InvalidOperationException("CSV: column size mismatch.");

        var ci = CultureInfo.InvariantCulture;

        using var sw = new StreamWriter(filename);
        sw.WriteLine(string.Join(",", header));

        for (int r = 0; r < n; r++)
        {
            string line = string.Join(",", cols.Select(c => c[r].ToString("G17", ci)));
            sw.WriteLine(line);
        }
    }

    // ------------------------------------------------------------
    // Pole model: uniform rod + lumped bob at top
    // ------------------------------------------------------------
    private sealed class PoleModel
    {
        public double L = 1.0;
        public double MRod = 0.06;
        public double MBob = 0.04;

        public double MTotal;
        public double LCom;
        public double IPivot;
        public double ICom;
        public double InertiaFactor;

        public void ComputeDerived()
        {
            MTotal = MRod + MBob;

            // COM from pivot: rod at L/2, bob at L
            LCom = (MRod * (L * 0.5) + MBob * L) / MTotal;

            // Inertia about pivot: rod (1/3)mL^2, bob mL^2
            IPivot = (1.0 / 3.0) * MRod * L * L + MBob * L * L;

            // Inertia about COM
            ICom = IPivot - MTotal * LCom * LCom;

            InertiaFactor = 1.0 + ICom / (MTotal * LCom * LCom);
        }
    }

    // ------------------------------------------------------------
    // Plant parameters
    // ------------------------------------------------------------
    private sealed class CartPoleParams
    {
        public double M = 1.2;
        public PoleModel Pole = new();
        public double G = 9.81;

        // Damping for realism and numerical stability
        public double CartDamping = 0.10;
        public double PoleDamping = 0.03;
    }

    // ------------------------------------------------------------
    // State (theta=0 upright)
    // ------------------------------------------------------------
    private struct State
    {
        public double X;
        public double XDot;
        public double Theta;
        public double ThetaDot;
    }

    // ------------------------------------------------------------
    // Sliding Mode Control parameters + gated cart-centering term
    // ------------------------------------------------------------
    private sealed class ControlParams
    {
        // Sliding surface:
        // s = theta_dot + lambda_theta*theta + alpha*(x_dot + lambda_x*x)
        public double LambdaTheta = 10.0;
        public double LambdaX = 1.5;
        public double Alpha = 0.55;

        // Sliding mode dynamics
        public double K = 110.0;
        public double Phi = 0.05;

        // Actuator saturation
        public double UMax = 70.0;

        // Cart centering (only near upright)
        public double HoldKp = 8.0;
        public double HoldKd = 10.0;

        // Gate: when |theta| >= theta_gate, centering is ~0
        public double ThetaGate = 0.20;
    }

    // ------------------------------------------------------------
    // Track limits
    // ------------------------------------------------------------
    private sealed class TrackLimits
    {
        public double XMax = 1.6;

        public void Enforce(ref State s)
        {
            if (s.X > XMax)
            {
                s.X = XMax;
                if (s.XDot > 0) s.XDot = 0;
            }

            if (s.X < -XMax)
            {
                s.X = -XMax;
                if (s.XDot < 0) s.XDot = 0;
            }
        }
    }

    // ------------------------------------------------------------
    // Disturbance schedule (exactly two disturbances)
    // ------------------------------------------------------------
    private sealed class Disturbance
    {
        public double T1 = 0.5;
        public double T2 = 5.0;

        public double ArrowDuration = 0.5;
        public double Duration = 0.5;

        // Torque amplitude (N*m)
        public double TauAmp = 3.3;

        private static double HalfSine(double localT, double duration)
        {
            // 0 -> 1 -> 0 over [0, duration]
            return Math.Sin(Math.PI * localT / duration);
        }

        public double TauExt(double t)
        {
            // Pulse 1 (positive torque)
            if (t >= T1 && t <= T1 + Duration)
            {
                double local = t - T1;
                return +TauAmp * HalfSine(local, Duration);
            }

            // Pulse 2 (negative torque)
            if (t >= T2 && t <= T2 + Duration)
            {
                double local = t - T2;
                return -TauAmp * HalfSine(local, Duration);
            }

            return 0.0;
        }

        public double BobForceEquivalent(double t, double L) => TauExt(t) / L;

        public bool ArrowVisible(double t)
        {
            if (t >= T1 && t <= T1 + ArrowDuration) return true;
            if (t >= T2 && t <= T2 + ArrowDuration) return true;
            return false;
        }
    }

    // ------------------------------------------------------------
    // Derivative container
    // ------------------------------------------------------------
    private readonly struct Deriv
    {
        public readonly double XDot;
        public readonly double XDDot;
        public readonly double ThetaDot;
        public readonly double ThetaDDot;

        public Deriv(double xDot, double xDDot, double thetaDot, double thetaDDot)
        {
            XDot = xDot;
            XDDot = xDDot;
            ThetaDot = thetaDot;
            ThetaDDot = thetaDDot;
        }
    }

    // ------------------------------------------------------------
    // Nonlinear cart–pole dynamics + external torque
    // ------------------------------------------------------------
    private static Deriv Dynamics(CartPoleParams p, State s, double uCart, double tauExt)
    {
        double m = p.Pole.MTotal;
        double l = p.Pole.LCom;

        double totalMass = p.M + m;
        double poleMassLength = m * l;

        double sinT = Math.Sin(s.Theta);
        double cosT = Math.Cos(s.Theta);

        // Cart friction-like damping
        double fDamped = uCart - p.CartDamping * s.XDot;

        double temp = (fDamped + poleMassLength * s.ThetaDot * s.ThetaDot * sinT) / totalMass;

        double denom = l * (p.Pole.InertiaFactor - (m * cosT * cosT) / totalMass);

        double thetaDDot = (p.G * sinT - cosT * temp) / denom;

        // Pole damping
        thetaDDot -= p.PoleDamping * s.ThetaDot;

        // External torque about pivot (disturbance)
        thetaDDot += tauExt / p.Pole.IPivot;

        double xDDot = temp - poleMassLength * thetaDDot * cosT / totalMass;

        return new Deriv(s.XDot, xDDot, s.ThetaDot, thetaDDot);
    }

    // ------------------------------------------------------------
    // RK4 integration step
    // ------------------------------------------------------------
    private static void Rk4Step(CartPoleParams p, ref State s, double dt, double uCart, double tauExt)
    {
        static State AddScaled(State a, Deriv k, double h)
        {
            a.X += h * k.XDot;
            a.XDot += h * k.XDDot;
            a.Theta += h * k.ThetaDot;
            a.ThetaDot += h * k.ThetaDDot;
            return a;
        }

        Deriv k1 = Dynamics(p, s, uCart, tauExt);
        Deriv k2 = Dynamics(p, AddScaled(s, k1, 0.5 * dt), uCart, tauExt);
        Deriv k3 = Dynamics(p, AddScaled(s, k2, 0.5 * dt), uCart, tauExt);
        Deriv k4 = Dynamics(p, AddScaled(s, k3, dt), uCart, tauExt);

        s.X += (dt / 6.0) * (k1.XDot + 2.0 * k2.XDot + 2.0 * k3.XDot + k4.XDot);
        s.XDot += (dt / 6.0) * (k1.XDDot + 2.0 * k2.XDDot + 2.0 * k3.XDDot + k4.XDDot);
        s.Theta += (dt / 6.0) * (k1.ThetaDot + 2.0 * k2.ThetaDot + 2.0 * k3.ThetaDot + k4.ThetaDot);
        s.ThetaDot += (dt / 6.0) * (k1.ThetaDDot + 2.0 * k2.ThetaDDot + 2.0 * k3.ThetaDDot + k4.ThetaDDot);

        s.Theta = WrapToPi(s.Theta);
    }

    // ------------------------------------------------------------
    // Sliding surface and SMC control
    // ------------------------------------------------------------
    private static double SlidingSurface(ControlParams c, State s)
    {
        return s.ThetaDot
               + c.LambdaTheta * s.Theta
               + c.Alpha * (s.XDot + c.LambdaX * s.X);
    }

    private static double SlidingSurfaceDotNominal(CartPoleParams p, ControlParams c, State s, double uCart)
    {
        // Nominal: ignore external torque for the control law
        Deriv d = Dynamics(p, s, uCart, 0.0);

        // sdot = theta_ddot + lambda_theta*theta_dot + alpha*(x_ddot + lambda_x*x_dot)
        return d.ThetaDDot
               + c.LambdaTheta * s.ThetaDot
               + c.Alpha * (d.XDDot + c.LambdaX * s.XDot);
    }

    private static double ComputeControl(CartPoleParams p, ControlParams c, State s)
    {
        // Sliding Mode Control part
        double sVal = SlidingSurface(c, s);
        double desiredSDot = -c.K * Sat(sVal / c.Phi);

        // Numeric affine approximation: sdot(u) ≈ a*u + b
        double sDot0 = SlidingSurfaceDotNominal(p, c, s, 0.0);
        double sDot1 = SlidingSurfaceDotNominal(p, c, s, 1.0);

        double a = (sDot1 - sDot0);
        double b = sDot0;

        double uSmc = (Math.Abs(a) < 1e-8) ? 0.0 : (desiredSDot - b) / a;

        // Cart-centering term (gated)
        double thetaAbs = Math.Abs(s.Theta);
        double gate = Clamp(1.0 - (thetaAbs / c.ThetaGate), 0.0, 1.0);

        double uHold = gate * (-c.HoldKp * s.X - c.HoldKd * s.XDot);

        // Combine and saturate
        return Clamp(uSmc + uHold, -c.UMax, c.UMax);
    }

    // ------------------------------------------------------------
    // Plot saving (ScottPlot) + embed 300 DPI metadata safely
    // ------------------------------------------------------------
    private static void ConfigurePlotForExport(ScottPlot.Plot plt)
    {
        plt.ScaleFactor = 2;
        plt.Axes.Hairline(true);
    }

    private static void SavePlotPng300Dpi(ScottPlot.Plot plt, string path, int widthPx, int heightPx)
    {
        string? dir = Path.GetDirectoryName(path);
        if (!string.IsNullOrWhiteSpace(dir))
            Directory.CreateDirectory(dir);

        ConfigurePlotForExport(plt);

        plt.SavePng(path, widthPx, heightPx);

        // Rewrite with 300 DPI metadata by cloning pixels 1:1 (avoids cropping)
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

                    using var clone = loaded.Clone(
                        new Rectangle(0, 0, loaded.Width, loaded.Height),
                        PixelFormat.Format32bppArgb);

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

    private static void SavePlots(
        string outDir,
        List<double> t,
        List<double> x,
        List<double> theta,
        List<double> u,
        List<double> fEq,
        List<double> tauExt,
        List<double> sSurf)
    {
        void SaveOne(string fileName, string title, string xlab, string ylab, double[] xs, double[] ys)
        {
            var plt = new ScottPlot.Plot();
            plt.Title(title);
            plt.XLabel(xlab);
            plt.YLabel(ylab);
            plt.Add.Scatter(xs, ys);
            SavePlotPng300Dpi(plt, Path.Combine(outDir, fileName), 2600, 1600);
        }

        SaveOne("cart_position.png", "Cart Position x(t)", "time (s)", "x (m)", t.ToArray(), x.ToArray());
        SaveOne("pole_angle.png", "Pole Angle theta(t) (0 = upright)", "time (s)", "theta (rad)", t.ToArray(), theta.ToArray());
        SaveOne("control_force.png", "Control Force u(t) (SMC)", "time (s)", "u (N)", t.ToArray(), u.ToArray());
        SaveOne("disturbance_torque.png", "External Disturbance Torque tau_ext(t)", "time (s)", "tau_ext (N·m)", t.ToArray(), tauExt.ToArray());
        SaveOne("equivalent_bob_force.png", "Equivalent Bob Force F_eq(t) = tau/L", "time (s)", "F_eq (N)", t.ToArray(), fEq.ToArray());
        SaveOne("sliding_surface.png", "Sliding Surface s(t)", "time (s)", "s", t.ToArray(), sSurf.ToArray());
    }

    // ------------------------------------------------------------
    // ffmpeg encoding (fixed: no redirected pipes + fast preset)
    // ------------------------------------------------------------
    private static bool FfmpegExistsOnPath()
    {
        try
        {
            var psi = new ProcessStartInfo
            {
                FileName = "ffmpeg",
                UseShellExecute = false,
                CreateNoWindow = true
            };
            psi.ArgumentList.Add("-version");

            using var p = Process.Start(psi);
            if (p == null) return false;

            p.WaitForExit();
            return p.ExitCode == 0;
        }
        catch
        {
            return false;
        }
    }

    private static void EncodeMp4WithFfmpeg(string framesDir, int fps, string outMp4)
    {
        if (!FfmpegExistsOnPath())
        {
            Console.Error.WriteLine("ffmpeg not found on PATH; MP4 will not be created.");
            return;
        }

        // Absolute paths are critical when setting WorkingDirectory to the frames folder.
        string framesDirFull = Path.GetFullPath(framesDir);
        string outMp4Full = Path.GetFullPath(outMp4);

        // Ensure output directory exists
        string? outDir = Path.GetDirectoryName(outMp4Full);
        if (!string.IsNullOrWhiteSpace(outDir))
            Directory.CreateDirectory(outDir);

        Console.WriteLine("Encoding MP4...");

        var psi = new ProcessStartInfo
        {
            FileName = "ffmpeg",
            WorkingDirectory = framesDirFull,

            // IMPORTANT: Do not redirect stdout/stderr here, because ffmpeg writes progress continuously.
            // Redirecting and reading incorrectly can cause pipe buffering and stall the process.
            UseShellExecute = false,
            CreateNoWindow = false
        };

        psi.ArgumentList.Add("-y");
        psi.ArgumentList.Add("-hide_banner");
        psi.ArgumentList.Add("-stats");

        psi.ArgumentList.Add("-framerate");
        psi.ArgumentList.Add(fps.ToString(CultureInfo.InvariantCulture));

        psi.ArgumentList.Add("-i");
        psi.ArgumentList.Add("frame_%06d.png");

        // Use H.264 encoding with fast preset.
        psi.ArgumentList.Add("-c:v");
        psi.ArgumentList.Add("libx264");
        psi.ArgumentList.Add("-preset");
        psi.ArgumentList.Add("veryfast");
        psi.ArgumentList.Add("-crf");
        psi.ArgumentList.Add("18");

        psi.ArgumentList.Add("-pix_fmt");
        psi.ArgumentList.Add("yuv420p");

        // Ensure even dimensions (safety for encoders/players)
        psi.ArgumentList.Add("-vf");
        psi.ArgumentList.Add("pad=ceil(iw/2)*2:ceil(ih/2)*2");

        psi.ArgumentList.Add("-movflags");
        psi.ArgumentList.Add("+faststart");

        psi.ArgumentList.Add(outMp4Full);

        using var p = Process.Start(psi);
        if (p == null)
        {
            Console.Error.WriteLine("Failed to start ffmpeg process.");
            return;
        }

        p.WaitForExit();

        if (p.ExitCode == 0)
            Console.WriteLine($"MP4 created: {outMp4Full}");
        else
            Console.Error.WriteLine($"ffmpeg encoding failed (exit code {p.ExitCode}).");
    }

    // ------------------------------------------------------------
    // Drawing helpers
    // ------------------------------------------------------------
    private static void DrawForceArrow(Graphics g, PointF start, PointF end, Pen pen, Brush brush, float headLen = 14f, float headW = 7f)
    {
        g.DrawLine(pen, start, end);

        float dx = end.X - start.X;
        float dy = end.Y - start.Y;
        float len = (float)Math.Sqrt(dx * dx + dy * dy);
        if (len < 1e-3f) return;

        float ux = dx / len;
        float uy = dy / len;

        float nx = -uy;
        float ny = ux;

        PointF tip = end;
        PointF basePt = new(end.X - ux * headLen, end.Y - uy * headLen);

        PointF p1 = new(basePt.X + nx * headW, basePt.Y + ny * headW);
        PointF p2 = new(basePt.X - nx * headW, basePt.Y - ny * headW);

        g.FillPolygon(brush, new[] { tip, p1, p2 });
    }

    private sealed class PreviewWindow : Form
    {
        private readonly PictureBox _pb = new() { Dock = DockStyle.Fill, SizeMode = PictureBoxSizeMode.Zoom };

        public PreviewWindow(int w, int h)
        {
            Text = "Cart-Pole SMC Preview";
            ClientSize = new Size(w, h);
            Controls.Add(_pb);
        }

        public void UpdateFrame(Bitmap bmp)
        {
            DrawingImage? old = _pb.Image;
            _pb.Image = (Bitmap)bmp.Clone();
            old?.Dispose();
            Application.DoEvents();
        }
    }

    [STAThread]
    public static void Main(string[] args)
    {
        bool preview = args.Any(a => a.Equals("--preview", StringComparison.OrdinalIgnoreCase));

        string outDir = Path.Combine("output", "pendulum_sliding_mode");
        string framesDir = Path.Combine(outDir, "frames");
        Directory.CreateDirectory(framesDir);

        foreach (string file in Directory.EnumerateFiles(framesDir, "*.png"))
            File.Delete(file);

        // Simulation settings
        double videoSeconds = 10.0;
        int totalFrames = 6000; // EXACT
        int fps = (int)(totalFrames / videoSeconds); // 600 FPS

        double dtFrame = videoSeconds / totalFrames;

        // Physics integration step: multiple substeps per frame for stability
        int substeps = 8;
        double dtPhysics = dtFrame / substeps;

        // Screen
        int W = 1000;
        int H = 600;

        // Plant/controller/disturbance
        var plant = new CartPoleParams
        {
            M = 1.2,
            Pole =
            {
                L = 1.0,
                MRod = 0.10,
                MBob = 0.15
            }
        };
        plant.Pole.ComputeDerived();

        var ctrl = new ControlParams();
        var track = new TrackLimits();
        var dist = new Disturbance();

        // Initial state
        State s = new()
        {
            X = 0.0,
            XDot = 0.0,
            Theta = 0.20,
            ThetaDot = 0.0
        };

        // Logging (one entry per frame)
        var tLog = new List<double>(totalFrames);
        var xLog = new List<double>(totalFrames);
        var thLog = new List<double>(totalFrames);
        var uLog = new List<double>(totalFrames);
        var fEqLog = new List<double>(totalFrames);
        var tauLog = new List<double>(totalFrames);
        var sSurfLog = new List<double>(totalFrames);

        // Optional preview
        PreviewWindow? previewWindow = null;
        if (preview)
        {
            previewWindow = new PreviewWindow(W, H);
            previewWindow.Show();
        }

        // Visual mapping
        float pixelsPerMeter = 180.0f;
        PointF origin = new(W * 0.5f, H * 0.75f);

        DrawingColor bg = DrawingColor.FromArgb(20, 20, 20);
        DrawingColor trackColor = DrawingColor.FromArgb(170, 170, 170);
        DrawingColor cartColor = DrawingColor.FromArgb(40, 140, 255);
        DrawingColor poleColor = DrawingColor.FromArgb(240, 70, 70);
        DrawingColor bobColor = DrawingColor.FromArgb(255, 220, 60);
        DrawingColor wheelColor = DrawingColor.FromArgb(70, 70, 70);
        DrawingColor arrowColor = DrawingColor.FromArgb(60, 255, 120);

        float trackLineX1 = 50f;
        float trackLineX2 = W - 50f;
        float trackY = origin.Y + 25f;

        float centerMarkX = origin.X;
        float centerMarkY1 = origin.Y - 30f;
        float centerMarkY2 = origin.Y + 30f;

        SizeF cartSize = new(140f, 40f);
        float wheelR = 14f;

        float poleLenPx = (float)(plant.Pole.L * pixelsPerMeter);
        float arrowLenPx = 120f;

        using var trackPen = new Pen(trackColor, 4f);
        using var centerPen = new Pen(DrawingColor.FromArgb(120, 120, 120), 3f);
        using var cartBrush = new SolidBrush(cartColor);
        using var polePen = new Pen(poleColor, 8f) { StartCap = LineCap.Round, EndCap = LineCap.Round };
        using var bobBrush = new SolidBrush(bobColor);
        using var wheelBrush = new SolidBrush(wheelColor);
        using var arrowPen = new Pen(arrowColor, 3f) { StartCap = LineCap.Round, EndCap = LineCap.Round };
        using var arrowBrush = new SolidBrush(arrowColor);

        // Main loop: 6000 frames
        double t = 0.0;

        for (int frame = 0; frame < totalFrames; frame++)
        {
            // Integrate dynamics for this frame using substeps
            double uUsed = 0.0;
            double tauUsed = 0.0;
            double fEqUsed = 0.0;
            double sUsed = 0.0;

            for (int k = 0; k < substeps; k++)
            {
                double tauExt = dist.TauExt(t);
                double u = ComputeControl(plant, ctrl, s);

                Rk4Step(plant, ref s, dtPhysics, u, tauExt);
                track.Enforce(ref s);

                uUsed = u;
                tauUsed = tauExt;
                fEqUsed = dist.BobForceEquivalent(t, plant.Pole.L);
                sUsed = SlidingSurface(ctrl, s);

                t += dtPhysics;
            }

            double tf = frame * dtFrame;

            // Log once per frame
            tLog.Add(tf);
            xLog.Add(s.X);
            thLog.Add(s.Theta);
            uLog.Add(uUsed);
            tauLog.Add(tauUsed);
            fEqLog.Add(fEqUsed);
            sSurfLog.Add(sUsed);

            // Render frame
            using var bmp = new Bitmap(W, H, PixelFormat.Format24bppRgb);
            bmp.SetResolution(300f, 300f);

            using (Graphics g = Graphics.FromImage(bmp))
            {
                g.SmoothingMode = SmoothingMode.AntiAlias;
                g.PixelOffsetMode = PixelOffsetMode.HighQuality;
                g.InterpolationMode = InterpolationMode.HighQualityBicubic;

                g.Clear(bg);

                // Track and center marker
                g.DrawLine(trackPen, trackLineX1, trackY, trackLineX2, trackY);
                g.DrawLine(centerPen, centerMarkX, centerMarkY1, centerMarkX, centerMarkY2);

                // Cart position mapping
                float cartXPx = origin.X + (float)(s.X * pixelsPerMeter);
                float cartYPx = origin.Y;

                // Cart body
                RectangleF cartRect = new(cartXPx - cartSize.Width / 2f, cartYPx - cartSize.Height / 2f, cartSize.Width, cartSize.Height);
                g.FillRectangle(cartBrush, cartRect);

                // Wheels
                float w1x = cartXPx - cartSize.Width * 0.30f;
                float w2x = cartXPx + cartSize.Width * 0.30f;
                float wy = cartYPx + cartSize.Height * 0.55f;

                g.FillEllipse(wheelBrush, w1x - wheelR, wy - wheelR, wheelR * 2f, wheelR * 2f);
                g.FillEllipse(wheelBrush, w2x - wheelR, wy - wheelR, wheelR * 2f, wheelR * 2f);

                // Pivot at top center of cart
                PointF pivot = new(cartXPx, cartYPx - cartSize.Height / 2f);

                // Pole tip position
                float tipX = pivot.X + poleLenPx * (float)Math.Sin(s.Theta);
                float tipY = pivot.Y - poleLenPx * (float)Math.Cos(s.Theta);

                // Pole line and bob
                g.DrawLine(polePen, pivot, new PointF(tipX, tipY));
                float bobR = 9f;
                g.FillEllipse(bobBrush, tipX - bobR, tipY - bobR, bobR * 2f, bobR * 2f);

                // Disturbance arrow visible for 0.5 s only
                if (dist.ArrowVisible(tf))
                {
                    float dir = (fEqUsed >= 0.0) ? 1f : -1f;
                    PointF start = new(tipX, tipY - 25f);
                    PointF end = new(tipX + dir * arrowLenPx, tipY - 25f);
                    DrawForceArrow(g, start, end, arrowPen, arrowBrush);
                }
            }

            // Save frame
            string framePath = Path.Combine(framesDir, $"frame_{frame:D6}.png");
            bmp.Save(framePath, DrawingImageFormat.Png);

            // Optional preview
            if (previewWindow != null && !previewWindow.IsDisposed)
                previewWindow.UpdateFrame(bmp);

            // Console progress
            if (frame % 600 == 0)
            {
                Console.WriteLine(
                    $"Frame {frame}/{totalFrames}  t={tf:F2}  x={s.X:F3}  theta={s.Theta:F3}  u={uUsed:F2}  tau={tauUsed:F3}");
            }
        }

        // Close preview window
        if (previewWindow != null && !previewWindow.IsDisposed)
        {
            previewWindow.Close();
            previewWindow.Dispose();
        }

        // Encode MP4
        string mp4Path = Path.Combine(outDir, "pendulum_smc_10s_6000f.mp4");
        EncodeMp4WithFfmpeg(framesDir, fps, mp4Path);

        // Save plots + CSV
        Console.WriteLine("Saving plots and CSV...");
        SavePlots(outDir, tLog, xLog, thLog, uLog, fEqLog, tauLog, sSurfLog);

        WriteCsv(
            filename: Path.Combine(outDir, "cartpole_log.csv"),
            header: new[] { "t", "x", "theta", "u", "F_equiv", "tau_ext", "s" },
            cols: new[] { tLog, xLog, thLog, uLog, fEqLog, tauLog, sSurfLog });

        Console.WriteLine("Done.");
    }
}
