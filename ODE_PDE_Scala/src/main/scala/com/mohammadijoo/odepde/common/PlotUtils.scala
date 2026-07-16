package com.mohammadijoo.odepde.common

import org.jfree.chart.{ChartFactory, ChartUtils, JFreeChart}
import org.jfree.chart.axis.{NumberAxis, NumberTickUnit}
import org.jfree.chart.plot.{PlotOrientation, XYPlot}
import org.jfree.chart.renderer.PaintScale
import org.jfree.chart.renderer.xy.{XYBlockRenderer, XYLineAndShapeRenderer}
import org.jfree.chart.title.PaintScaleLegend
import org.jfree.chart.ui.{RectangleEdge, RectangleInsets}
import org.jfree.data.xy.{DefaultXYZDataset, XYSeries, XYSeriesCollection}

import java.awt.{BasicStroke, Color, Font}
import java.io.File
import java.nio.file.{Files, Path}
import java.text.{DecimalFormat, DecimalFormatSymbols, NumberFormat}
import java.util.Locale

/**
  * High-quality plot utilities using JFreeChart.
  *
  * Design goals (as requested):
  *  - high-resolution PNG output suitable for ~300 DPI printing
  *  - large fonts for titles and axis labels
  *  - thick plot lines
  *  - ≤ 10 tick labels per axis (by explicit tick spacing)
  *  - ≥ 20 px padding around the chart
  */
object PlotUtils {

  // ----------------------------
  // Locale / numeric formatting
  // ----------------------------
  // Some operating systems use non-Latin numerals in the default locale.
  // To guarantee English/Latin digits in tick labels, force a US-style number format
  // for all axes that we create.
  private val UsNumberFormat: NumberFormat = {
    val symbols = DecimalFormatSymbols.getInstance(Locale.US)
    // Compact, engineering-friendly format without scientific notation for typical ranges.
    new DecimalFormat("0.0", symbols)
  }

  // ----------------------------
  // Global styling choices
  // ----------------------------
  private val TitleFont = new Font("SansSerif", Font.BOLD, 48)
  private val AxisLabelFont = new Font("SansSerif", Font.BOLD, 40)
  private val TickFont = new Font("SansSerif", Font.PLAIN, 32)
  private val LineStroke = new BasicStroke(3.5f)

  // Typical "print-ready" image sizes:
  // - 2400x1800 roughly corresponds to 8x6 inches at 300 DPI
  private val DefaultW = 2400
  private val DefaultH = 1800

  private def ensureParent(file: Path): Unit = {
    val parent = file.getParent
    if (parent != null) Files.createDirectories(parent)
  }

  private def tickUnitForRange(lo: Double, hi: Double): NumberTickUnit = {
    val span = math.abs(hi - lo)
    val unit = if (span <= 1e-12) 1.0 else span / 9.0 // -> at most ~10 ticks
    new NumberTickUnit(unit)
  }

  private def expandRange(lo: Double, hi: Double, frac: Double = 0.05): (Double, Double) = {
    if (math.abs(hi - lo) < 1e-12) (lo - 1.0, hi + 1.0)
    else {
      val m = (hi - lo) * frac
      (lo - m, hi + m)
    }
  }

  // ----------------------------
  // Line plots
  // ----------------------------

  /**
    * Saves a single-series line plot as a high-resolution PNG.
    */
  def saveLinePlot(
      title: String,
      xlabel: String,
      ylabel: String,
      x: Seq[Double],
      y: Seq[Double],
      outFile: Path,
      width: Int = DefaultW,
      height: Int = DefaultH
  ): Unit = {
    require(x.length == y.length, "Plot error: x and y must have the same length.")
    ensureParent(outFile)

    val series = new XYSeries("series")
    var i = 0
    while (i < x.length) {
      series.add(x(i), y(i))
      i += 1
    }
    val dataset = new XYSeriesCollection()
    dataset.addSeries(series)

    val chart = ChartFactory.createXYLineChart(
      title,
      xlabel,
      ylabel,
      dataset,
      PlotOrientation.VERTICAL,
      false,
      false,
      false
    )

    styleLineChart(chart, x, y)

    ChartUtils.saveChartAsPNG(outFile.toFile, chart, width, height)
  }

  private def styleLineChart(chart: JFreeChart, x: Seq[Double], y: Seq[Double]): Unit = {
    chart.getTitle.setFont(TitleFont)
    chart.setPadding(new RectangleInsets(20, 20, 20, 20))

    val plot = chart.getXYPlot
    plot.setBackgroundPaint(Color.WHITE)
    plot.setDomainGridlinePaint(Color.LIGHT_GRAY)
    plot.setRangeGridlinePaint(Color.LIGHT_GRAY)

    val renderer = plot.getRenderer.asInstanceOf[XYLineAndShapeRenderer]
    renderer.setSeriesStroke(0, LineStroke)
    renderer.setDefaultShapesVisible(false)

    // Axis styling
    val xAxis = plot.getDomainAxis.asInstanceOf[NumberAxis]
    val yAxis = plot.getRangeAxis.asInstanceOf[NumberAxis]

    // Force English/Latin digits regardless of the OS/UI language.
    xAxis.setNumberFormatOverride(UsNumberFormat)
    yAxis.setNumberFormatOverride(UsNumberFormat)

    xAxis.setLabelFont(AxisLabelFont)
    yAxis.setLabelFont(AxisLabelFont)
    xAxis.setTickLabelFont(TickFont)
    yAxis.setTickLabelFont(TickFont)

    // Ensure tick numbers use English/Latin digits and a single decimal place
    xAxis.setNumberFormatOverride(UsNumberFormat)
    yAxis.setNumberFormatOverride(UsNumberFormat)

    // Force tick counts to be bounded
    val xMin = x.min
    val xMax = x.max
    val (xLo, xHi) = expandRange(xMin, xMax, frac = 0.0) // keep exact range unless x is degenerate
    xAxis.setRange(xLo, xHi)
    xAxis.setTickUnit(tickUnitForRange(xLo, xHi))

    val yMin = y.min
    val yMax = y.max
    val (yLo, yHi) = expandRange(yMin, yMax, frac = 0.05)
    yAxis.setRange(yLo, yHi)
    yAxis.setTickUnit(tickUnitForRange(yLo, yHi))

    plot.setAxisOffset(new RectangleInsets(20, 20, 20, 20))
  }

  // ----------------------------
  // Heatmaps
  // ----------------------------

  /**
    * Saves a heatmap as a high-resolution PNG using an XYBlockRenderer.
    *
    * @param values Flattened field in row-major order: index = j*nx + i
    * @param nx     number of columns in x-direction
    * @param ny     number of rows in y-direction
    */
  def saveHeatmap(
      title: String,
      xlabel: String,
      ylabel: String,
      values: Array[Double],
      nx: Int,
      ny: Int,
      outFile: Path,
      width: Int = DefaultW,
      height: Int = DefaultH
  ): Unit = {
    require(values.length == nx * ny, "Heatmap error: values length must equal nx*ny.")
    ensureParent(outFile)

    // Build XYZ arrays
    val n = nx * ny
    val xs = new Array[Double](n)
    val ys = new Array[Double](n)
    val zs = new Array[Double](n)

    var k = 0
    var j = 0
    while (j < ny) {
      var i = 0
      while (i < nx) {
        xs(k) = i.toDouble
        ys(k) = j.toDouble
        zs(k) = values(j * nx + i)
        k += 1
        i += 1
      }
      j += 1
    }

    val dataset = new DefaultXYZDataset()
    dataset.addSeries("T", Array(xs, ys, zs))

    val zMin = zs.min
    val zMax = zs.max
    val paintScale: PaintScale = new LinearHSBPaintScale(zMin, zMax)

    val renderer = new XYBlockRenderer()
    renderer.setBlockWidth(1.0)
    renderer.setBlockHeight(1.0)
    renderer.setPaintScale(paintScale)

    val xAxis = new NumberAxis(xlabel)
    val yAxis = new NumberAxis(ylabel)

    // Force English/Latin digits regardless of the OS/UI language.
    xAxis.setNumberFormatOverride(UsNumberFormat)
    yAxis.setNumberFormatOverride(UsNumberFormat)
    xAxis.setLabelFont(AxisLabelFont)
    yAxis.setLabelFont(AxisLabelFont)
    xAxis.setTickLabelFont(TickFont)
    yAxis.setTickLabelFont(TickFont)

    // Ensure tick numbers use English/Latin digits and a single decimal place
    xAxis.setNumberFormatOverride(UsNumberFormat)
    yAxis.setNumberFormatOverride(UsNumberFormat)

    // Keep tick labels bounded to ~10
    xAxis.setRange(-0.5, (nx - 1).toDouble + 0.5)
    yAxis.setRange(-0.5, (ny - 1).toDouble + 0.5)
    xAxis.setTickUnit(new NumberTickUnit(math.max(1.0, (nx - 1).toDouble / 9.0)))
    yAxis.setTickUnit(new NumberTickUnit(math.max(1.0, (ny - 1).toDouble / 9.0)))

    val plot = new XYPlot(dataset, xAxis, yAxis, renderer)
    plot.setBackgroundPaint(Color.WHITE)
    plot.setAxisOffset(new RectangleInsets(20, 20, 20, 20))
    plot.setDomainGridlinesVisible(false)
    plot.setRangeGridlinesVisible(false)

    val chart = new JFreeChart(title, TitleFont, plot, false)
    chart.setPadding(new RectangleInsets(20, 20, 20, 20))

    // Add a color scale legend to the right
    val legendAxis = new NumberAxis("T")
    // Force English/Latin digits on the color scale legend.
    legendAxis.setNumberFormatOverride(UsNumberFormat)
    val legend = new PaintScaleLegend(paintScale, legendAxis)
    legend.setPosition(RectangleEdge.RIGHT)
    legend.setMargin(new RectangleInsets(20, 20, 20, 20))
    legend.getAxis.setLabelFont(AxisLabelFont)
    legend.getAxis.setTickLabelFont(TickFont)
    chart.addSubtitle(legend)

    ChartUtils.saveChartAsPNG(outFile.toFile, chart, width, height)
  }

  /**
    * A simple continuous color scale based on HSB:
    * - low values map to blue-ish hues
    * - high values map to red-ish hues
    *
    * This is intentionally deterministic and does not rely on external colormap libraries.
    */
  final class LinearHSBPaintScale(private val lower: Double, private val upper: Double) extends PaintScale {
    override def getLowerBound: Double = lower
    override def getUpperBound: Double = upper

    override def getPaint(value: Double): java.awt.Paint = {
      if (upper <= lower + 1e-12) return Color.getHSBColor(0.66f, 1.0f, 1.0f)

      val t = ((value - lower) / (upper - lower)).toFloat
      val tt = math.max(0.0f, math.min(1.0f, t))

      // Hue: 0.66 (blue) -> 0.0 (red)
      val hue = (0.66f * (1.0f - tt)) + (0.0f * tt)
      Color.getHSBColor(hue, 1.0f, 1.0f)
    }
  }
}