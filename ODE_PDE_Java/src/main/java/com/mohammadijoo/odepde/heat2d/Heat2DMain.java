package com.mohammadijoo.odepde.heat2d;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.axis.NumberTickUnit;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.PaintScale;
import org.jfree.chart.renderer.xy.XYBlockRenderer;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.data.xy.DefaultXYZDataset;
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

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.RenderingHints;
import java.awt.image.BufferedImage;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import java.nio.file.Files;
import java.nio.file.Path;

import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.text.NumberFormat;

import java.util.Iterator;
import java.util.Locale;

/**
 * ------------------------------------------------------------
 * 2D Heat Equation (Heat Transfer by Conduction) in Java
 * ------------------------------------------------------------
 * PDE:
 *   ∂T/∂t = α ( ∂²T/∂x² + ∂²T/∂y² )
 *
 * Method:
 *   - 2D explicit finite-difference scheme (FTCS)
 *   - Dirichlet boundary conditions (fixed temperature on boundaries)
 *   - Snapshots saved as PNG heatmaps (high resolution + 300 DPI metadata)
 *   - Additional plots saved (center point vs time, final centerline) (300 DPI metadata)
 *
 * Output folder:
 *   output/heat2d/
 * ------------------------------------------------------------
 */
public class Heat2DMain {

    // Larger typography for large 300-DPI canvases
    private static final Font TITLE_FONT = new Font("SansSerif", Font.PLAIN, 48);
    private static final Font AXIS_LABEL_FONT = new Font("SansSerif", Font.PLAIN, 40);
    private static final Font TICK_LABEL_FONT = new Font("SansSerif", Font.PLAIN, 34);

    // Thicker strokes for visibility on high-resolution images
    private static final float PLOT_LINE_STROKE = 6.0f;
    private static final float GRID_STROKE = 2.0f;

    private static int idx(int i, int j, int nx) {
        return j * nx + i;
    }

    private static NumberFormat englishNumberFormat(String pattern) {
        DecimalFormatSymbols sym = DecimalFormatSymbols.getInstance(Locale.US);
        return new DecimalFormat(pattern, sym);
    }

    private static void styleXYChart(JFreeChart chart,
                                     double xTickUnit, double yTickUnit,
                                     String xPattern, String yPattern) {

        if (chart.getTitle() != null) {
            chart.getTitle().setFont(TITLE_FONT); // 48 px, plain
        }

        XYPlot plot = chart.getXYPlot();
        plot.setBackgroundPaint(Color.WHITE);

        // Stronger gridlines and clearer visibility
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

        // Thicker axis lines and tick marks
        xAxis.setAxisLineStroke(new BasicStroke(3.0f));
        yAxis.setAxisLineStroke(new BasicStroke(3.0f));
        xAxis.setTickMarkStroke(new BasicStroke(2.5f));
        yAxis.setTickMarkStroke(new BasicStroke(2.5f));
    }

    private static void writeCsvTwoColumns(
            String filename,
            String h1,
            String h2,
            double[] c1,
            double[] c2
    ) throws IOException {

        if (c1.length != c2.length) {
            throw new IllegalArgumentException("CSV write error: column sizes do not match.");
        }

        try (BufferedWriter out = new BufferedWriter(new FileWriter(filename))) {
            out.write(h1 + "," + h2);
            out.newLine();
            for (int k = 0; k < c1.length; k++) {
                out.write(String.format(Locale.US, "%.12f,%.12f", c1[k], c2[k]));
                out.newLine();
            }
        }
    }

    private static final class ThermalPaintScale implements PaintScale {
        private final double lower;
        private final double upper;

        ThermalPaintScale(double lower, double upper) {
            this.lower = lower;
            this.upper = upper;
        }

        @Override
        public double getLowerBound() {
            return lower;
        }

        @Override
        public double getUpperBound() {
            return upper;
        }

        @Override
        public java.awt.Paint getPaint(double value) {
            double v = value;

            if (v < lower) v = lower;
            if (v > upper) v = upper;

            double t = (upper > lower) ? (v - lower) / (upper - lower) : 0.0;

            // Anchors: blue(0) -> cyan(0.25) -> green(0.5) -> yellow(0.75) -> red(1)
            Color c0, c1;
            double local;

            if (t <= 0.25) {
                c0 = new Color(0, 0, 255);
                c1 = new Color(0, 255, 255);
                local = t / 0.25;
            } else if (t <= 0.50) {
                c0 = new Color(0, 255, 255);
                c1 = new Color(0, 255, 0);
                local = (t - 0.25) / 0.25;
            } else if (t <= 0.75) {
                c0 = new Color(0, 255, 0);
                c1 = new Color(255, 255, 0);
                local = (t - 0.50) / 0.25;
            } else {
                c0 = new Color(255, 255, 0);
                c1 = new Color(255, 0, 0);
                local = (t - 0.75) / 0.25;
            }

            int r = (int) Math.round(c0.getRed() + local * (c1.getRed() - c0.getRed()));
            int g = (int) Math.round(c0.getGreen() + local * (c1.getGreen() - c0.getGreen()));
            int b = (int) Math.round(c0.getBlue() + local * (c1.getBlue() - c0.getBlue()));

            return new Color(clamp255(r), clamp255(g), clamp255(b));
        }

        private static int clamp255(int x) {
            if (x < 0) return 0;
            if (x > 255) return 255;
            return x;
        }
    }

    private static void savePngWithDpi(BufferedImage image, String filename, int dpi) throws IOException {
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

    private static BufferedImage renderChart(JFreeChart chart, int width, int height) {
        BufferedImage img = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
        var g2 = img.createGraphics();

        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g2.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
        g2.setRenderingHint(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_QUALITY);

        chart.draw(g2, new java.awt.geom.Rectangle2D.Double(0, 0, width, height));
        g2.dispose();

        return img;
    }

    private static JFreeChart createHeatmapChart(
            String title,
            double[] T,
            int nx,
            int ny,
            double dx,
            double dy,
            double tMin,
            double tMax
    ) {
        int n = nx * ny;
        double[] xs = new double[n];
        double[] ys = new double[n];
        double[] zs = new double[n];

        int k = 0;
        for (int j = 0; j < ny; j++) {
            double y = j * dy;
            for (int i = 0; i < nx; i++) {
                double x = i * dx;
                xs[k] = x;
                ys[k] = y;
                zs[k] = T[idx(i, j, nx)];
                k++;
            }
        }

        DefaultXYZDataset dataset = new DefaultXYZDataset();
        dataset.addSeries("Temperature", new double[][]{xs, ys, zs});

        NumberAxis xAxis = new NumberAxis("x (m)");
        NumberAxis yAxis = new NumberAxis("y (m)");

        // English digits + reduced tick labels
        xAxis.setNumberFormatOverride(englishNumberFormat("0.0"));
        yAxis.setNumberFormatOverride(englishNumberFormat("0.0"));
        xAxis.setTickUnit(new NumberTickUnit(0.2));
        yAxis.setTickUnit(new NumberTickUnit(0.2));

        // Fonts
        xAxis.setLabelFont(AXIS_LABEL_FONT);
        yAxis.setLabelFont(AXIS_LABEL_FONT);
        xAxis.setTickLabelFont(TICK_LABEL_FONT);
        yAxis.setTickLabelFont(TICK_LABEL_FONT);

        // Thicker axis lines
        xAxis.setAxisLineStroke(new BasicStroke(3.0f));
        yAxis.setAxisLineStroke(new BasicStroke(3.0f));
        xAxis.setTickMarkStroke(new BasicStroke(2.5f));
        yAxis.setTickMarkStroke(new BasicStroke(2.5f));

        XYBlockRenderer renderer = new XYBlockRenderer();
        renderer.setBlockWidth(dx);
        renderer.setBlockHeight(dy);
        renderer.setPaintScale(new ThermalPaintScale(tMin, tMax));

        XYPlot plot = new XYPlot(dataset, xAxis, yAxis, renderer);
        plot.setBackgroundPaint(Color.WHITE);
        plot.setDomainGridlinesVisible(false);
        plot.setRangeGridlinesVisible(false);

        JFreeChart chart = new JFreeChart(title, TITLE_FONT, plot, false);
        chart.setBackgroundPaint(Color.WHITE);

        if (chart.getTitle() != null) {
            chart.getTitle().setFont(TITLE_FONT);
        }

        return chart;
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

        // Thicker plot line and no point markers
        XYPlot plot = chart.getXYPlot();
        XYLineAndShapeRenderer r = new XYLineAndShapeRenderer(true, false);
        r.setSeriesStroke(0, new BasicStroke(PLOT_LINE_STROKE, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
        plot.setRenderer(r);

        return chart;
    }

    private static double[] minMax(double[] a) {
        double mn = Double.POSITIVE_INFINITY;
        double mx = Double.NEGATIVE_INFINITY;

        for (double v : a) {
            if (v < mn) mn = v;
            if (v > mx) mx = v;
        }

        if (!Double.isFinite(mn) || !Double.isFinite(mx)) {
            mn = 0.0;
            mx = 1.0;
        }

        if (Math.abs(mx - mn) < 1e-12) {
            mx = mn + 1.0;
        }

        return new double[]{mn, mx};
    }

    public static void main(String[] args) {
        try {
            Locale.setDefault(Locale.US);

            String outDir = "output/heat2d";
            Files.createDirectories(Path.of(outDir));

            final double alpha = 1.0;

            final double Lx = 1.0;
            final double Ly = 1.0;

            final int nx = 81;
            final int ny = 81;

            final double dx = Lx / (nx - 1);
            final double dy = Ly / (ny - 1);

            final double dtStable = 1.0 / (2.0 * alpha * (1.0 / (dx * dx) + 1.0 / (dy * dy)));
            final double dt = 0.80 * dtStable;

            final double tEnd = 0.20;
            final int nSteps = (int) Math.ceil(tEnd / dt);

            final int desiredSnapshots = 30;
            final int snapshotEvery = Math.max(1, nSteps / desiredSnapshots);

            double[] T = new double[nx * ny];
            double[] Tnew = new double[nx * ny];

            final double Tleft = 100.0;
            final double Tright = 0.0;
            final double Ttop = 0.0;
            final double Tbottom = 0.0;

            for (int j = 0; j < ny; j++) {
                T[idx(0, j, nx)] = Tleft;
                T[idx(nx - 1, j, nx)] = Tright;
            }
            for (int i = 0; i < nx; i++) {
                T[idx(i, 0, nx)] = Tbottom;
                T[idx(i, ny - 1, nx)] = Ttop;
            }

            final int ic = nx / 2;
            final int jc = ny / 2;

            double[] timeLog = new double[nSteps];
            double[] centerTLog = new double[nSteps];

            final int dpi = 300;

            // Keep your current pixel size; increased fonts/strokes make it readable
            final int heatmapW = 2400;
            final int heatmapH = 2000;

            final int plotW = 2400;
            final int plotH = 1800;

            double t = 0.0;

            for (int step = 0; step < nSteps; step++) {
                timeLog[step] = t;
                centerTLog[step] = T[idx(ic, jc, nx)];

                for (int j = 1; j < ny - 1; j++) {
                    for (int i = 1; i < nx - 1; i++) {
                        double Txx =
                                (T[idx(i + 1, j, nx)] - 2.0 * T[idx(i, j, nx)] + T[idx(i - 1, j, nx)]) / (dx * dx);

                        double Tyy =
                                (T[idx(i, j + 1, nx)] - 2.0 * T[idx(i, j, nx)] + T[idx(i, j - 1, nx)]) / (dy * dy);

                        Tnew[idx(i, j, nx)] = T[idx(i, j, nx)] + alpha * dt * (Txx + Tyy);
                    }
                }

                for (int j = 0; j < ny; j++) {
                    Tnew[idx(0, j, nx)] = Tleft;
                    Tnew[idx(nx - 1, j, nx)] = Tright;
                }
                for (int i = 0; i < nx; i++) {
                    Tnew[idx(i, 0, nx)] = Tbottom;
                    Tnew[idx(i, ny - 1, nx)] = Ttop;
                }

                double[] tmp = T;
                T = Tnew;
                Tnew = tmp;

                if (step % snapshotEvery == 0) {
                    double[] mm = minMax(T);

                    JFreeChart chart = createHeatmapChart(
                            "2D Heat Equation - Temperature Field",
                            T, nx, ny, dx, dy,
                            mm[0], mm[1]
                    );

                    BufferedImage img = renderChart(chart, heatmapW, heatmapH);
                    String pngName = outDir + "/heat_t" + step + ".png";
                    savePngWithDpi(img, pngName, dpi);

                    System.out.println("Saved snapshot: " + pngName);
                }

                t += dt;
            }

            double[] x = new double[nx];
            double[] centerline = new double[nx];

            for (int i = 0; i < nx; i++) {
                x[i] = i * dx;
                centerline[i] = T[idx(i, jc, nx)];
            }

            {
                JFreeChart chart = createLineChart(
                        "Final Centerline Temperature (y = 0.5)",
                        "x (m)",
                        "T(x, y=0.5)",
                        x, centerline
                );

                styleXYChart(chart, 0.2, 20.0, "0.0", "0");

                BufferedImage img = renderChart(chart, plotW, plotH);
                savePngWithDpi(img, outDir + "/centerline_final.png", dpi);
            }

            {
                JFreeChart chart = createLineChart(
                        "Temperature at Plate Center vs Time",
                        "time (s)",
                        "T(center)",
                        timeLog, centerTLog
                );

                styleXYChart(chart, 0.05, 20.0, "0.00", "0");

                BufferedImage img = renderChart(chart, plotW, plotH);
                savePngWithDpi(img, outDir + "/center_point_vs_time.png", dpi);
            }

            writeCsvTwoColumns(outDir + "/heat2d_log.csv", "t", "T_center", timeLog, centerTLog);

            System.out.println("Heat2D finished. Results are in: " + outDir);

        } catch (Exception e) {
            System.err.println("Heat2D failed: " + e.getMessage());
            e.printStackTrace();
            System.exit(1);
        }
    }
}
