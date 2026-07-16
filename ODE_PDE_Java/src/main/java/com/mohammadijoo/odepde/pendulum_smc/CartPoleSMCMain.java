package com.mohammadijoo.odepde.pendulum_smc;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.axis.NumberTickUnit;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import javax.imageio.IIOImage;
import javax.imageio.ImageIO;
import javax.imageio.ImageTypeSpecifier;
import javax.imageio.ImageWriteParam;
import javax.imageio.ImageWriter;
import javax.imageio.metadata.IIOMetadata;
import javax.imageio.metadata.IIOMetadataNode;
import javax.imageio.stream.ImageOutputStream;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Polygon;
import java.awt.RenderingHints;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;

import java.nio.file.Files;
import java.nio.file.Path;

import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.text.NumberFormat;

import java.util.Iterator;
import java.util.Locale;

/**
 * ------------------------------------------------------------
 * Cart–Pole Inverted Pendulum (moving base) with SLIDING MODE CONTROL (SMC)
 *
 * Implemented features:
 *   - Fixed total simulation time: 10 seconds
 *   - Frame-by-frame rendering to PNG files
 *   - Exactly two disturbances: at t=0.5s and t=5.0s
 *   - Disturbance arrow shown for 0.5s only (direction indicator, constant length)
 *   - Dynamics are integrated (RK4) and used to render each frame
 *   - Outputs: frames, plots (300 DPI), CSV log, then MP4 (ffmpeg)
 *
 * Output folders:
 *   output/pendulum_sliding_mode/frames/frame_000000.png ...
 *   output/pendulum_sliding_mode/*.png plots
 *   output/pendulum_sliding_mode/cartpole_log.csv
 *   output/pendulum_sliding_mode/pendulum_smc_10s_6000f.mp4   (if ffmpeg in PATH)
 * ------------------------------------------------------------
 */
public class CartPoleSMCMain {

    // Larger typography for large 300-DPI canvases
    private static final Font TITLE_FONT = new Font("SansSerif", Font.PLAIN, 48);
    private static final Font AXIS_LABEL_FONT = new Font("SansSerif", Font.PLAIN, 40);
    private static final Font TICK_LABEL_FONT = new Font("SansSerif", Font.PLAIN, 34);

    // Thicker strokes for visibility
    private static final float PLOT_LINE_STROKE = 6.0f;
    private static final float GRID_STROKE = 2.0f;

    // ------------------------------------------------------------
    // Small math helpers
    // ------------------------------------------------------------
    private static double clamp(double x, double lo, double hi) {
        if (x < lo) return lo;
        if (x > hi) return hi;
        return x;
    }

    private static double wrapToPi(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private static double sat(double z) {
        return clamp(z, -1.0, 1.0);
    }

    private static NumberFormat englishNumberFormat(String pattern) {
        DecimalFormatSymbols sym = DecimalFormatSymbols.getInstance(Locale.US);
        return new DecimalFormat(pattern, sym);
    }

    // ------------------------------------------------------------
    // CSV writer
    // ------------------------------------------------------------
    private static void writeCsv(String filename, String[] header, double[][] cols) throws Exception {
        if (cols.length == 0) throw new IllegalArgumentException("CSV: no columns.");

        int n = cols[0].length;
        for (double[] c : cols) {
            if (c.length != n) throw new IllegalArgumentException("CSV: column size mismatch.");
        }

        try (BufferedWriter out = new BufferedWriter(new FileWriter(filename))) {
            for (int i = 0; i < header.length; i++) {
                out.write(header[i]);
                if (i + 1 < header.length) out.write(",");
            }
            out.newLine();

            for (int r = 0; r < n; r++) {
                for (int c = 0; c < cols.length; c++) {
                    out.write(String.format(Locale.US, "%.12f", cols[c][r]));
                    if (c + 1 < cols.length) out.write(",");
                }
                out.newLine();
            }
        }
    }

    // ------------------------------------------------------------
    // Pole model
    // ------------------------------------------------------------
    private static final class PoleModel {
        double L = 1.0;
        double mRod = 0.06;
        double mBob = 0.04;

        double mTotal;
        double lCom;
        double iPivot;
        double iCom;
        double inertiaFactor;

        void computeDerived() {
            mTotal = mRod + mBob;

            lCom = (mRod * (L * 0.5) + mBob * L) / mTotal;

            iPivot = (1.0 / 3.0) * mRod * L * L + mBob * L * L;
            iCom = iPivot - mTotal * lCom * lCom;

            inertiaFactor = 1.0 + iCom / (mTotal * lCom * lCom);
        }
    }

    private static final class CartPoleParams {
        double M = 1.2;
        PoleModel pole = new PoleModel();
        double g = 9.81;

        double cartDamping = 0.10;
        double poleDamping = 0.03;
    }

    private static final class State {
        double x = 0.0;
        double xDot = 0.0;
        double theta = 0.20;
        double thetaDot = 0.0;
    }

    private static final class ControlParams {
        double lambdaTheta = 10.0;
        double lambdaX = 1.5;
        double alpha = 0.55;

        double k = 110.0;
        double phi = 0.05;

        double uMax = 70.0;

        double holdKp = 8.0;
        double holdKd = 10.0;

        double thetaGate = 0.20;
    }

    private static final class TrackLimits {
        double xMax = 1.6;

        void enforce(State s) {
            if (s.x > xMax) {
                s.x = xMax;
                if (s.xDot > 0) s.xDot = 0;
            }
            if (s.x < -xMax) {
                s.x = -xMax;
                if (s.xDot < 0) s.xDot = 0;
            }
        }
    }

    private static final class Disturbance {
        double t1 = 0.5;
        double t2 = 5.0;

        double arrowDuration = 0.5;
        double duration = 0.5;

        double tauAmp = 3.3;

        private static double halfSine(double localT, double duration) {
            return Math.sin(Math.PI * localT / duration);
        }

        double tauExt(double t) {
            if (t >= t1 && t <= t1 + duration) {
                double local = t - t1;
                return +tauAmp * halfSine(local, duration);
            }
            if (t >= t2 && t <= t2 + duration) {
                double local = t - t2;
                return -tauAmp * halfSine(local, duration);
            }
            return 0.0;
        }

        double bobForceEquivalent(double t, double L) {
            return tauExt(t) / L;
        }

        boolean arrowVisible(double t) {
            if (t >= t1 && t <= t1 + arrowDuration) return true;
            if (t >= t2 && t <= t2 + arrowDuration) return true;
            return false;
        }
    }

    private static final class Deriv {
        double xDot;
        double xDdot;
        double thetaDot;
        double thetaDdot;

        Deriv(double xDot, double xDdot, double thetaDot, double thetaDdot) {
            this.xDot = xDot;
            this.xDdot = xDdot;
            this.thetaDot = thetaDot;
            this.thetaDdot = thetaDdot;
        }
    }

    private static Deriv dynamics(CartPoleParams p, State s, double uCart, double tauExt) {
        double m = p.pole.mTotal;
        double l = p.pole.lCom;

        double totalMass = p.M + m;
        double poleMassLength = m * l;

        double sinT = Math.sin(s.theta);
        double cosT = Math.cos(s.theta);

        double fDamped = uCart - p.cartDamping * s.xDot;

        double temp = (fDamped + poleMassLength * s.thetaDot * s.thetaDot * sinT) / totalMass;

        double denom = l * (p.pole.inertiaFactor - (m * cosT * cosT) / totalMass);

        double thetaDdot = (p.g * sinT - cosT * temp) / denom;

        thetaDdot -= p.poleDamping * s.thetaDot;
        thetaDdot += tauExt / p.pole.iPivot;

        double xDdot = temp - poleMassLength * thetaDdot * cosT / totalMass;

        return new Deriv(s.xDot, xDdot, s.thetaDot, thetaDdot);
    }

    private static void rk4Step(CartPoleParams p, State s, double dt, double uCart, double tauExt) {
        State s1 = copyState(s);

        Deriv k1 = dynamics(p, s1, uCart, tauExt);
        Deriv k2 = dynamics(p, addScaled(s1, k1, 0.5 * dt), uCart, tauExt);
        Deriv k3 = dynamics(p, addScaled(s1, k2, 0.5 * dt), uCart, tauExt);
        Deriv k4 = dynamics(p, addScaled(s1, k3, dt), uCart, tauExt);

        s.x += (dt / 6.0) * (k1.xDot + 2.0 * k2.xDot + 2.0 * k3.xDot + k4.xDot);
        s.xDot += (dt / 6.0) * (k1.xDdot + 2.0 * k2.xDdot + 2.0 * k3.xDdot + k4.xDdot);
        s.theta += (dt / 6.0) * (k1.thetaDot + 2.0 * k2.thetaDot + 2.0 * k3.thetaDot + k4.thetaDot);
        s.thetaDot += (dt / 6.0) * (k1.thetaDdot + 2.0 * k2.thetaDdot + 2.0 * k3.thetaDdot + k4.thetaDdot);

        s.theta = wrapToPi(s.theta);
    }

    private static State copyState(State s) {
        State out = new State();
        out.x = s.x;
        out.xDot = s.xDot;
        out.theta = s.theta;
        out.thetaDot = s.thetaDot;
        return out;
    }

    private static State addScaled(State a, Deriv k, double h) {
        State out = copyState(a);
        out.x += h * k.xDot;
        out.xDot += h * k.xDdot;
        out.theta += h * k.thetaDot;
        out.thetaDot += h * k.thetaDdot;
        return out;
    }

    private static double slidingSurface(ControlParams c, State s) {
        return s.thetaDot
                + c.lambdaTheta * s.theta
                + c.alpha * (s.xDot + c.lambdaX * s.x);
    }

    private static double slidingSurfaceDotNominal(CartPoleParams p, ControlParams c, State s, double uCart) {
        Deriv d = dynamics(p, s, uCart, 0.0);

        return d.thetaDdot
                + c.lambdaTheta * s.thetaDot
                + c.alpha * (d.xDdot + c.lambdaX * s.xDot);
    }

    private static double computeControl(CartPoleParams p, ControlParams c, State s) {
        double sVal = slidingSurface(c, s);
        double desiredSDot = -c.k * sat(sVal / c.phi);

        double sdot0 = slidingSurfaceDotNominal(p, c, s, 0.0);
        double sdot1 = slidingSurfaceDotNominal(p, c, s, 1.0);
        double a = (sdot1 - sdot0);
        double b = sdot0;

        double uSmc = (Math.abs(a) < 1e-8) ? 0.0 : (desiredSDot - b) / a;

        double thetaAbs = Math.abs(s.theta);
        double gate = clamp(1.0 - (thetaAbs / c.thetaGate), 0.0, 1.0);

        double uHold = gate * (-c.holdKp * s.x - c.holdKd * s.xDot);

        double uTotal = uSmc + uHold;
        return clamp(uTotal, -c.uMax, c.uMax);
    }

    private static void savePngWithDpi(BufferedImage image, String filename, int dpi) throws Exception {
        int pixelsPerMeter = (int) Math.round(dpi / 0.0254);

        Iterator<ImageWriter> it = ImageIO.getImageWritersByFormatName("png");
        if (!it.hasNext()) {
            ImageIO.write(image, "png", new File(filename));
            return;
        }

        ImageWriter writer = it.next();
        ImageWriteParam writeParam = writer.getDefaultWriteParam();
        ImageTypeSpecifier typeSpecifier = ImageTypeSpecifier.createFromBufferedImageType(image.getType());
        IIOMetadata metadata = writer.getDefaultImageMetadata(typeSpecifier, writeParam);

        String metaFormat = "javax_imageio_png_1.0";

        try {
            IIOMetadataNode root = new IIOMetadataNode(metaFormat);
            IIOMetadataNode phys = new IIOMetadataNode("pHYs");
            phys.setAttribute("pixelsPerUnitXAxis", Integer.toString(pixelsPerMeter));
            phys.setAttribute("pixelsPerUnitYAxis", Integer.toString(pixelsPerMeter));
            phys.setAttribute("unitSpecifier", "meter");
            root.appendChild(phys);
            metadata.mergeTree(metaFormat, root);
        } catch (Exception ignored) {
        }

        try (ImageOutputStream ios = ImageIO.createImageOutputStream(new File(filename))) {
            writer.setOutput(ios);
            writer.write(null, new IIOImage(image, null, metadata), writeParam);
        } finally {
            writer.dispose();
        }
    }

    private static final class PreviewWindow {
        private final JFrame frame;
        private final ImagePanel panel;
        private volatile boolean open = true;

        PreviewWindow(int w, int h) {
            panel = new ImagePanel(w, h);

            frame = new JFrame("Cart-Pole SMC Preview");
            frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
            frame.getContentPane().add(panel);
            frame.pack();
            frame.setLocationByPlatform(true);
            frame.setVisible(true);

            frame.addWindowListener(new java.awt.event.WindowAdapter() {
                @Override
                public void windowClosed(java.awt.event.WindowEvent e) {
                    open = false;
                }
            });
        }

        boolean isOpen() {
            return open;
        }

        void updateImage(BufferedImage img) {
            panel.setImage(img);
            panel.repaint();
        }
    }

    private static final class ImagePanel extends JPanel {
        private volatile BufferedImage image;

        ImagePanel(int w, int h) {
            setPreferredSize(new Dimension(w, h));
        }

        void setImage(BufferedImage img) {
            this.image = img;
        }

        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            BufferedImage img = image;
            if (img != null) {
                g.drawImage(img, 0, 0, null);
            }
        }
    }

    private static void drawForceArrow(Graphics2D g2, float x0, float y0, float x1, float y1, Color color) {
        g2.setColor(color);
        g2.setStroke(new BasicStroke(5.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
        g2.drawLine(Math.round(x0), Math.round(y0), Math.round(x1), Math.round(y1));

        float dx = x1 - x0;
        float dy = y1 - y0;
        float len = (float) Math.sqrt(dx * dx + dy * dy);
        if (len < 1e-3f) return;

        float ux = dx / len;
        float uy = dy / len;

        float nx = -uy;
        float ny = ux;

        float headLen = 20.0f;
        float headW = 10.0f;

        float tipX = x1;
        float tipY = y1;
        float baseX = x1 - ux * headLen;
        float baseY = y1 - uy * headLen;

        Polygon tri = new Polygon();
        tri.addPoint(Math.round(tipX), Math.round(tipY));
        tri.addPoint(Math.round(baseX + nx * headW), Math.round(baseY + ny * headW));
        tri.addPoint(Math.round(baseX - nx * headW), Math.round(baseY - ny * headW));

        g2.fillPolygon(tri);
    }

    private static BufferedImage renderChart(JFreeChart chart, int width, int height) {
        BufferedImage img = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
        Graphics2D g2 = img.createGraphics();

        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g2.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
        g2.setRenderingHint(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_QUALITY);

        chart.draw(g2, new java.awt.geom.Rectangle2D.Double(0, 0, width, height));
        g2.dispose();

        return img;
    }

    private static JFreeChart createLineChart(String title, String xLabel, String yLabel, double[] x, double[] y) {
        XYSeries series = new XYSeries("data");
        int n = Math.min(x.length, y.length);
        for (int i = 0; i < n; i++) {
            series.add(x[i], y[i]);
        }

        XYSeriesCollection ds = new XYSeriesCollection(series);

        JFreeChart chart = ChartFactory.createXYLineChart(title, xLabel, yLabel, ds);
        chart.setBackgroundPaint(Color.WHITE);
        chart.getXYPlot().setBackgroundPaint(Color.WHITE);

        // Thicker plot line, no point markers
        XYPlot plot = chart.getXYPlot();
        XYLineAndShapeRenderer r = new XYLineAndShapeRenderer(true, false);
        r.setSeriesStroke(0, new BasicStroke(PLOT_LINE_STROKE, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
        plot.setRenderer(r);

        return chart;
    }

    private static void styleXYChart(JFreeChart chart,
                                    double xTickUnit, double yTickUnit,
                                    String xPattern, String yPattern) {

        if (chart.getTitle() != null) {
            chart.getTitle().setFont(TITLE_FONT); // 48 px, plain
        }

        XYPlot plot = chart.getXYPlot();
        plot.setBackgroundPaint(Color.WHITE);

        plot.setDomainGridlinePaint(new Color(210, 210, 210));
        plot.setRangeGridlinePaint(new Color(210, 210, 210));
        plot.setDomainGridlineStroke(new BasicStroke(GRID_STROKE));
        plot.setRangeGridlineStroke(new BasicStroke(GRID_STROKE));
        plot.setDomainGridlinesVisible(true);
        plot.setRangeGridlinesVisible(true);

        NumberAxis xAxis = (NumberAxis) plot.getDomainAxis();
        NumberAxis yAxis = (NumberAxis) plot.getRangeAxis();

        xAxis.setLabelFont(AXIS_LABEL_FONT);
        yAxis.setLabelFont(AXIS_LABEL_FONT);

        xAxis.setTickLabelFont(TICK_LABEL_FONT);
        yAxis.setTickLabelFont(TICK_LABEL_FONT);

        xAxis.setNumberFormatOverride(englishNumberFormat(xPattern));
        yAxis.setNumberFormatOverride(englishNumberFormat(yPattern));

        xAxis.setTickUnit(new NumberTickUnit(xTickUnit));
        yAxis.setTickUnit(new NumberTickUnit(yTickUnit));

        xAxis.setAxisLineStroke(new BasicStroke(3.0f));
        yAxis.setAxisLineStroke(new BasicStroke(3.0f));
        xAxis.setTickMarkStroke(new BasicStroke(2.5f));
        yAxis.setTickMarkStroke(new BasicStroke(2.5f));
    }

    private static double[] minMax(double[] a) {
        double mn = Double.POSITIVE_INFINITY;
        double mx = Double.NEGATIVE_INFINITY;
        for (double v : a) {
            if (v < mn) mn = v;
            if (v > mx) mx = v;
        }
        if (!Double.isFinite(mn) || !Double.isFinite(mx) || Math.abs(mx - mn) < 1e-12) {
            return new double[]{0.0, 1.0};
        }
        return new double[]{mn, mx};
    }

    private static double niceTickUnit(double range, int targetTicks) {
        if (range <= 0.0) return 1.0;

        double raw = range / Math.max(1, targetTicks);
        double exp = Math.floor(Math.log10(raw));
        double base = raw / Math.pow(10.0, exp);

        double nice;
        if (base <= 1.0) nice = 1.0;
        else if (base <= 2.0) nice = 2.0;
        else if (base <= 5.0) nice = 5.0;
        else nice = 10.0;

        return nice * Math.pow(10.0, exp);
    }

    private static void savePlots(
            String outDir,
            double[] t,
            double[] x,
            double[] theta,
            double[] u,
            double[] fEq,
            double[] tauExt,
            double[] sSurf
    ) throws Exception {

        int dpi = 300;

        int w = 2400;
        int h = 1800;

        double[] mmX = minMax(x);
        double[] mmTh = minMax(theta);
        double[] mmU = minMax(u);
        double[] mmTau = minMax(tauExt);
        double[] mmFeq = minMax(fEq);
        double[] mmS = minMax(sSurf);

        double yTickX = niceTickUnit(mmX[1] - mmX[0], 6);
        double yTickTh = niceTickUnit(mmTh[1] - mmTh[0], 6);
        double yTickU = niceTickUnit(mmU[1] - mmU[0], 6);
        double yTickTau = niceTickUnit(mmTau[1] - mmTau[0], 6);
        double yTickFeq = niceTickUnit(mmFeq[1] - mmFeq[0], 6);
        double yTickS = niceTickUnit(mmS[1] - mmS[0], 6);

        double xTickTime = 1.0;

        {
            JFreeChart ch = createLineChart("Cart Position x(t)", "time (s)", "x (m)", t, x);
            styleXYChart(ch, xTickTime, yTickX, "0", "0.###");
            BufferedImage img = renderChart(ch, w, h);
            savePngWithDpi(img, outDir + "/cart_position.png", dpi);
        }
        {
            JFreeChart ch = createLineChart("Pole Angle theta(t) (0=upright)", "time (s)", "theta (rad)", t, theta);
            styleXYChart(ch, xTickTime, yTickTh, "0", "0.###");
            BufferedImage img = renderChart(ch, w, h);
            savePngWithDpi(img, outDir + "/pole_angle.png", dpi);
        }
        {
            JFreeChart ch = createLineChart("Control Force u(t) (SMC)", "time (s)", "u (N)", t, u);
            styleXYChart(ch, xTickTime, yTickU, "0", "0.###");
            BufferedImage img = renderChart(ch, w, h);
            savePngWithDpi(img, outDir + "/control_force.png", dpi);
        }
        {
            JFreeChart ch = createLineChart("External Disturbance Torque tau_ext(t)", "time (s)", "tau_ext (N*m)", t, tauExt);
            styleXYChart(ch, xTickTime, yTickTau, "0", "0.###");
            BufferedImage img = renderChart(ch, w, h);
            savePngWithDpi(img, outDir + "/disturbance_torque.png", dpi);
        }
        {
            JFreeChart ch = createLineChart("Equivalent Bob Force F_eq(t) = tau/L", "time (s)", "F_eq (N)", t, fEq);
            styleXYChart(ch, xTickTime, yTickFeq, "0", "0.###");
            BufferedImage img = renderChart(ch, w, h);
            savePngWithDpi(img, outDir + "/equivalent_bob_force.png", dpi);
        }
        {
            JFreeChart ch = createLineChart("Sliding Surface s(t)", "time (s)", "s", t, sSurf);
            styleXYChart(ch, xTickTime, yTickS, "0", "0.###");
            BufferedImage img = renderChart(ch, w, h);
            savePngWithDpi(img, outDir + "/sliding_surface.png", dpi);
        }
    }

    private static boolean ffmpegOnPath() {
        try {
            Process p = new ProcessBuilder("ffmpeg", "-version")
                    .inheritIO()
                    .start();
            int rc = p.waitFor();
            return rc == 0;
        } catch (Exception e) {
            return false;
        }
    }

    private static void encodeMp4WithFfmpeg(String framesDir, int fps, String outMp4) {
        if (!ffmpegOnPath()) {
            System.err.println("ffmpeg not found on PATH; MP4 will not be created.");
            return;
        }

        try {
            System.out.println("Encoding MP4 (ffmpeg will show progress)...");

            ProcessBuilder pb = new ProcessBuilder(
                    "ffmpeg",
                    "-y",
                    "-hide_banner",
                    "-loglevel", "error",
                    "-stats",
                    "-framerate", Integer.toString(fps),
                    "-i", framesDir + File.separator + "frame_%06d.png",
                    "-c:v", "libx264",
                    "-preset", "veryfast",
                    "-pix_fmt", "yuv420p",
                    outMp4
            );

            pb.inheritIO();

            Process p = pb.start();
            int rc = p.waitFor();

            if (rc == 0) {
                System.out.println("MP4 created: " + outMp4);
            } else {
                System.err.println("ffmpeg encoding failed (exit code " + rc + ").");
            }
        } catch (Exception e) {
            System.err.println("ffmpeg encoding failed: " + e.getMessage());
        }
    }

    public static void main(String[] args) {
        try {
            Locale.setDefault(Locale.US);

            boolean preview = false;
            int totalFrames = 6000;
            boolean noMp4 = false;

            for (int i = 0; i < args.length; i++) {
                if ("--preview".equalsIgnoreCase(args[i])) {
                    preview = true;
                } else if ("--frames".equalsIgnoreCase(args[i]) && i + 1 < args.length) {
                    totalFrames = Integer.parseInt(args[i + 1]);
                    i++;
                } else if ("--no-mp4".equalsIgnoreCase(args[i])) {
                    noMp4 = true;
                }
            }

            String outDir = "output/pendulum_sliding_mode";
            String framesDir = outDir + "/frames";
            Files.createDirectories(Path.of(framesDir));

            try (var stream = Files.list(Path.of(framesDir))) {
                stream.filter(Files::isRegularFile).forEach(p -> {
                    try { Files.deleteIfExists(p); } catch (Exception ignored) {}
                });
            }

            final double videoSeconds = 10.0;
            final int fps = (int) Math.round(totalFrames / videoSeconds);
            final double dtFrame = videoSeconds / (double) totalFrames;

            final int substeps = 8;
            final double dtPhysics = dtFrame / (double) substeps;

            final int W = 1000;
            final int H = 600;

            CartPoleParams plant = new CartPoleParams();
            plant.M = 1.2;
            plant.pole.L = 1.0;
            plant.pole.mRod = 0.1;
            plant.pole.mBob = 0.15;
            plant.pole.computeDerived();

            ControlParams ctrl = new ControlParams();
            TrackLimits track = new TrackLimits();
            Disturbance dist = new Disturbance();

            State s = new State();
            s.theta = 0.20;

            double[] tLog = new double[totalFrames];
            double[] xLog = new double[totalFrames];
            double[] thLog = new double[totalFrames];
            double[] uLog = new double[totalFrames];
            double[] fEqLog = new double[totalFrames];
            double[] tauLog = new double[totalFrames];
            double[] surfLog = new double[totalFrames];

            final PreviewWindow[] previewWindow = new PreviewWindow[1];
            if (preview) {
                SwingUtilities.invokeAndWait(() -> previewWindow[0] = new PreviewWindow(W, H));
            }

            final float pixelsPerMeter = 180.0f;
            final float originX = W * 0.5f;
            final float originY = H * 0.75f;

            final Color bg = new Color(20, 20, 20);
            final Color trackColor = new Color(170, 170, 170);
            final Color cartColor = new Color(40, 140, 255);
            final Color poleColor = new Color(240, 70, 70);
            final Color bobColor = new Color(255, 220, 60);
            final Color wheelColor = new Color(70, 70, 70);
            final Color arrowColor = new Color(60, 255, 120);

            final float cartW = 140.0f;
            final float cartH = 40.0f;
            final float wheelR = 14.0f;
            final float poleW = 8.0f;
            final float poleLenPx = (float) (plant.pole.L * pixelsPerMeter);

            final float arrowLenPx = 120.0f;

            double t = 0.0;

            for (int frame = 0; frame < totalFrames; frame++) {
                double uUsed = 0.0;
                double tauUsed = 0.0;
                double fEqUsed = 0.0;
                double sUsed = 0.0;

                for (int k = 0; k < substeps; k++) {
                    double tauExt = dist.tauExt(t);
                    double u = computeControl(plant, ctrl, s);

                    rk4Step(plant, s, dtPhysics, u, tauExt);
                    track.enforce(s);

                    uUsed = u;
                    tauUsed = tauExt;
                    fEqUsed = dist.bobForceEquivalent(t, plant.pole.L);
                    sUsed = slidingSurface(ctrl, s);

                    t += dtPhysics;
                }

                double tf = frame * dtFrame;
                tLog[frame] = tf;
                xLog[frame] = s.x;
                thLog[frame] = s.theta;
                uLog[frame] = uUsed;
                tauLog[frame] = tauUsed;
                fEqLog[frame] = fEqUsed;
                surfLog[frame] = sUsed;

                BufferedImage img = new BufferedImage(W, H, BufferedImage.TYPE_INT_ARGB);
                Graphics2D g2 = img.createGraphics();

                g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
                g2.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
                g2.setRenderingHint(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_QUALITY);

                g2.setColor(bg);
                g2.fillRect(0, 0, W, H);

                g2.setColor(trackColor);
                int trackY = Math.round(originY + 25.0f);
                g2.fillRect(50, trackY - 2, W - 100, 4);

                g2.setColor(new Color(120, 120, 120));
                g2.fillRect(Math.round(originX - 1.5f), Math.round(originY - 30.0f), 3, 60);

                float cartX = originX + (float) (s.x * pixelsPerMeter);
                float cartY = originY;

                g2.setColor(cartColor);
                g2.fillRoundRect(Math.round(cartX - cartW / 2), Math.round(cartY - cartH / 2),
                        Math.round(cartW), Math.round(cartH), 12, 12);

                g2.setColor(wheelColor);
                float w1x = cartX - cartW * 0.30f;
                float w2x = cartX + cartW * 0.30f;
                float wy = cartY + cartH * 0.55f;
                g2.fillOval(Math.round(w1x - wheelR), Math.round(wy - wheelR), Math.round(2 * wheelR), Math.round(2 * wheelR));
                g2.fillOval(Math.round(w2x - wheelR), Math.round(wy - wheelR), Math.round(2 * wheelR), Math.round(2 * wheelR));

                float pivotX = cartX;
                float pivotY = cartY - cartH * 0.5f;

                g2.setColor(poleColor);

                AffineTransform old = g2.getTransform();
                AffineTransform tx = new AffineTransform();
                tx.translate(pivotX, pivotY);
                tx.rotate(s.theta);
                g2.setTransform(tx);

                g2.fillRoundRect(Math.round(-poleW / 2), Math.round(-poleLenPx),
                        Math.round(poleW), Math.round(poleLenPx), 6, 6);

                g2.setTransform(old);

                float tipX = (float) (pivotX + poleLenPx * Math.sin(s.theta));
                float tipY = (float) (pivotY - poleLenPx * Math.cos(s.theta));

                g2.setColor(bobColor);
                float bobR = 9.0f;
                g2.fillOval(Math.round(tipX - bobR), Math.round(tipY - bobR), Math.round(2 * bobR), Math.round(2 * bobR));

                if (dist.arrowVisible(tf)) {
                    float dir = (fEqUsed >= 0.0) ? 1.0f : -1.0f;
                    float startX = tipX;
                    float startY = tipY - 25.0f;
                    float endX = tipX + dir * arrowLenPx;
                    float endY = tipY - 25.0f;
                    drawForceArrow(g2, startX, startY, endX, endY, arrowColor);
                }

                g2.dispose();

                String fn = framesDir + "/frame_" + String.format(Locale.US, "%06d", frame) + ".png";
                if (!ImageIO.write(img, "png", new File(fn))) {
                    throw new RuntimeException("Failed to save: " + fn);
                }

                if (previewWindow[0] != null && previewWindow[0].isOpen() && (frame % 10 == 0)) {
                    BufferedImage toShow = img;
                    SwingUtilities.invokeLater(() -> {
                        if (previewWindow[0] != null && previewWindow[0].isOpen()) {
                            previewWindow[0].updateImage(toShow);
                        }
                    });
                }

                if (frame % Math.max(1, fps) == 0) {
                    System.out.println(String.format(
                            Locale.US,
                            "Frame %d/%d  t=%.2f  x=%.3f  theta=%.3f  u=%.2f  tau=%.3f",
                            frame, totalFrames, tf, s.x, s.theta, uUsed, tauUsed
                    ));
                }
            }

            System.out.println("Saving plots and CSV...");
            savePlots(outDir, tLog, xLog, thLog, uLog, fEqLog, tauLog, surfLog);

            writeCsv(outDir + "/cartpole_log.csv",
                    new String[]{"t", "x", "theta", "u", "F_equiv", "tau_ext", "s"},
                    new double[][]{tLog, xLog, thLog, uLog, fEqLog, tauLog, surfLog});

            if (!noMp4) {
                String mp4 = outDir + "/pendulum_smc_10s_6000f.mp4";
                encodeMp4WithFfmpeg(framesDir, fps, mp4);
            } else {
                System.out.println("MP4 encoding skipped (--no-mp4).");
            }

            System.out.println("Done.");

        } catch (Exception e) {
            System.err.println("CartPole SMC failed: " + e.getMessage());
            e.printStackTrace();
            System.exit(1);
        }
    }
}
