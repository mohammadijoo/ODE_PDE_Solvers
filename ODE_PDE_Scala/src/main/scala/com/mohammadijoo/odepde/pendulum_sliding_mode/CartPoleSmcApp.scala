package com.mohammadijoo.odepde.pendulum_sliding_mode

import com.mohammadijoo.odepde.common.{CsvUtils, PlotUtils}

import java.awt.geom.{AffineTransform, Ellipse2D, Line2D, Rectangle2D}
import java.awt.{BasicStroke, Color, RenderingHints}
import java.awt.image.BufferedImage
import java.nio.file.{Files, Path, Paths}
import java.util.Locale
import javax.imageio.ImageIO
import javax.swing.{JFrame, JPanel, SwingUtilities}

import scala.math.{Pi, abs, ceil, cos, sin, sqrt}
import scala.util.Try

/**
  * ------------------------------------------------------------
  * Cart–Pole Inverted Pendulum (moving base) with SLIDING MODE CONTROL (SMC)
  *
  * Implemented features:
  *   - Total simulation: 10 seconds
  *   - Default frames: 6000 (=> 600 FPS video)
  *   - Exactly two disturbances: at t=0.5s and t=5.0s
  *   - Disturbance shown as an arrow for 0.5s only (constant length)
  *   - Dynamics are integrated (RK4) and used to render each frame
  *   - Outputs: frames, mp4 (ffmpeg), plots, CSV log
  *
  * Output folders:
  *   output/pendulum_sliding_mode/frames/frame_000000.png ...
  *   output/pendulum_sliding_mode/pendulum_smc_10s_6000f.mp4 (if ffmpeg in PATH)
  *   output/pendulum_sliding_mode/ (PNG plot files)
  *   output/pendulum_sliding_mode/cartpole_log.csv
  * ------------------------------------------------------------
  */
object CartPoleSmcApp {

  // ------------------------------------------------------------
  // Small math helpers
  // ------------------------------------------------------------

  private def clamp(x: Double, lo: Double, hi: Double): Double =
    if (x < lo) lo else if (x > hi) hi else x

  private def wrapToPi(a0: Double): Double = {
    var a = a0
    while (a > Pi) a -= 2.0 * Pi
    while (a < -Pi) a += 2.0 * Pi
    a
  }

  // Boundary-layer saturation for sliding mode: sat(z) in [-1, 1]
  private def sat(z: Double): Double = clamp(z, -1.0, 1.0)

  // ------------------------------------------------------------
  // Pole model: uniform rod + lumped bob at the top
  // ------------------------------------------------------------
  final class PoleModel(
      var L: Double = 1.0,      // (m)
      var mRod: Double = 0.06,  // (kg)
      var mBob: Double = 0.04   // (kg)
  ) {
    var mTotal: Double = 0.0
    var lCom: Double = 0.0
    var iPivot: Double = 0.0
    var iCom: Double = 0.0
    var inertiaFactor: Double = 0.0 // 1 + I_com/(m*l^2)

    def computeDerived(): Unit = {
      mTotal = mRod + mBob

      // Center of mass (COM) from pivot:
      // - rod COM is at L/2
      // - bob is at L
      lCom = (mRod * (L * 0.5) + mBob * L) / mTotal

      // Inertia about pivot:
      // - rod about end: (1/3)mL^2
      // - bob about pivot: mL^2
      iPivot = (1.0 / 3.0) * mRod * L * L + mBob * L * L

      // Inertia about COM (parallel axis theorem)
      iCom = iPivot - mTotal * lCom * lCom

      inertiaFactor = 1.0 + iCom / (mTotal * lCom * lCom)
    }
  }

  // ------------------------------------------------------------
  // Plant parameters
  // ------------------------------------------------------------
  final class CartPoleParams {
    var M: Double = 1.2
    val pole: PoleModel = new PoleModel()

    val g: Double = 9.81

    // Damping for realism and numerical stability
    val cartDamping: Double = 0.10 // N per (m/s)
    val poleDamping: Double = 0.03 // applied to theta_ddot term
  }

  // ------------------------------------------------------------
  // State (theta=0 upright)
  // Convention: theta > 0 means pole leans right.
  // ------------------------------------------------------------
  final class State {
    var x: Double = 0.0
    var xdot: Double = 0.0
    var theta: Double = 0.20
    var thetadot: Double = 0.0
  }

  // ------------------------------------------------------------
  // Sliding Mode Control parameters + cart-centering term
  // ------------------------------------------------------------
  final class ControlParams {
    // Sliding surface:
    // s = theta_dot + lambda_theta*theta + alpha*(x_dot + lambda_x*x)
    val lambdaTheta: Double = 10.0
    val lambdaX: Double = 1.5
    val alpha: Double = 0.55

    // Sliding mode dynamics (boundary layer for chattering reduction)
    val k: Double = 110.0
    val phi: Double = 0.05

    // Actuator saturation
    val uMax: Double = 70.0 // N

    // Cart centering (only when near upright)
    val holdKp: Double = 8.0
    val holdKd: Double = 10.0

    // Gate: when |theta| >= thetaGate, centering is ~0
    val thetaGate: Double = 0.20 // rad
  }

  // ------------------------------------------------------------
  // Track limit: keep the cart in view
  // ------------------------------------------------------------
  final class TrackLimits {
    val xMax: Double = 1.6 // meters

    def enforce(s: State): Unit = {
      if (s.x > xMax) {
        s.x = xMax
        if (s.xdot > 0) s.xdot = 0
      }
      if (s.x < -xMax) {
        s.x = -xMax
        if (s.xdot < 0) s.xdot = 0
      }
    }
  }

  // ------------------------------------------------------------
  // Disturbance schedule (exactly two disturbances)
  // ------------------------------------------------------------
  final class Disturbance {
    val t1: Double = 0.5
    val t2: Double = 5.0

    // Arrow is shown for 0.5 s only after each pulse start
    val arrowDuration: Double = 0.5

    // Disturbance duration (use same as arrow duration)
    val duration: Double = 0.5

    // Torque amplitude (N*m)
    val tauAmp: Double = 3.3

    // Smooth half-sine pulse in [0, duration]
    private def halfSine(localT: Double, dur: Double): Double =
      sin(Pi * localT / dur) // 0 -> 1 -> 0

    /**
      * External torque about pivot (N*m).
      * Pulse 1: positive torque (push right)
      * Pulse 2: negative torque (push left)
      */
    def tauExt(t: Double): Double = {
      if (t >= t1 && t <= t1 + duration) {
        val local = t - t1
        +tauAmp * halfSine(local, duration)
      } else if (t >= t2 && t <= t2 + duration) {
        val local = t - t2
        -tauAmp * halfSine(local, duration)
      } else 0.0
    }

    /**
      * Equivalent bob force (used for arrow direction and logging only).
      * F_eq = tau / L
      */
    def bobForceEquivalent(t: Double, L: Double): Double = tauExt(t) / L

    /**
      * Arrow is visible for a short window after each disturbance starts.
      */
    def arrowVisible(t: Double): Boolean =
      (t >= t1 && t <= t1 + arrowDuration) || (t >= t2 && t <= t2 + arrowDuration)
  }

  // ------------------------------------------------------------
  // ODE derivative container
  // ------------------------------------------------------------
  final case class Deriv(
      xDot: Double,
      xDdot: Double,
      thetaDot: Double,
      thetaDdot: Double
  )

  // ------------------------------------------------------------
  // Nonlinear cart–pole dynamics + external torque tauExt
  // ------------------------------------------------------------
  private def dynamics(p: CartPoleParams, s: State, uCart: Double, tauExt: Double): Deriv = {
    val m = p.pole.mTotal
    val l = p.pole.lCom

    val totalMass = p.M + m
    val poleMassLength = m * l

    val sinT = sin(s.theta)
    val cosT = cos(s.theta)

    // Cart damping term (simple viscous friction model)
    val fDamped = uCart - p.cartDamping * s.xdot

    val temp =
      (fDamped + poleMassLength * s.thetadot * s.thetadot * sinT) / totalMass

    val denom =
      l * (p.pole.inertiaFactor - (m * cosT * cosT) / totalMass)

    var thetaDdot =
      (p.g * sinT - cosT * temp) / denom

    // Pole damping
    thetaDdot -= p.poleDamping * s.thetadot

    // External torque about pivot (disturbance)
    thetaDdot += tauExt / p.pole.iPivot

    val xDdot =
      temp - poleMassLength * thetaDdot * cosT / totalMass

    Deriv(s.xdot, xDdot, s.thetadot, thetaDdot)
  }

  // ------------------------------------------------------------
  // RK4 integration step
  // ------------------------------------------------------------
  private def rk4Step(p: CartPoleParams, s: State, dt: Double, uCart: Double, tauExt: Double): Unit = {
    def addScaled(a: State, k: Deriv, h: Double): State = {
      val out = new State
      out.x = a.x + h * k.xDot
      out.xdot = a.xdot + h * k.xDdot
      out.theta = a.theta + h * k.thetaDot
      out.thetadot = a.thetadot + h * k.thetaDdot
      out
    }

    val k1 = dynamics(p, s, uCart, tauExt)
    val k2 = dynamics(p, addScaled(s, k1, 0.5 * dt), uCart, tauExt)
    val k3 = dynamics(p, addScaled(s, k2, 0.5 * dt), uCart, tauExt)
    val k4 = dynamics(p, addScaled(s, k3, dt), uCart, tauExt)

    s.x += (dt / 6.0) * (k1.xDot + 2.0 * k2.xDot + 2.0 * k3.xDot + k4.xDot)
    s.xdot += (dt / 6.0) * (k1.xDdot + 2.0 * k2.xDdot + 2.0 * k3.xDdot + k4.xDdot)
    s.theta += (dt / 6.0) * (k1.thetaDot + 2.0 * k2.thetaDot + 2.0 * k3.thetaDot + k4.thetaDot)
    s.thetadot += (dt / 6.0) * (k1.thetaDdot + 2.0 * k2.thetaDdot + 2.0 * k3.thetaDdot + k4.thetaDdot)

    s.theta = wrapToPi(s.theta)
  }

  // ------------------------------------------------------------
  // Sliding surface and SMC control
  // ------------------------------------------------------------
  private def slidingSurface(c: ControlParams, s: State): Double =
    s.thetadot + c.lambdaTheta * s.theta + c.alpha * (s.xdot + c.lambdaX * s.x)

  private def slidingSurfaceDotNominal(p: CartPoleParams, c: ControlParams, s: State, uCart: Double): Double = {
    // Nominal derivative: external torque ignored in the control law
    val d = dynamics(p, s, uCart, tauExt = 0.0)

    // sdot = theta_ddot + lambda_theta*theta_dot + alpha*(x_ddot + lambda_x*x_dot)
    d.thetaDdot + c.lambdaTheta * s.thetadot + c.alpha * (d.xDdot + c.lambdaX * s.xdot)
  }

  private def computeControl(p: CartPoleParams, c: ControlParams, s: State): Double = {
    // Sliding Mode Control term
    val sVal = slidingSurface(c, s)
    val desiredSdot = -c.k * sat(sVal / c.phi)

    // Numeric affine approximation: sdot(u) ≈ a*u + b
    val sdot0 = slidingSurfaceDotNominal(p, c, s, uCart = 0.0)
    val sdot1 = slidingSurfaceDotNominal(p, c, s, uCart = 1.0)
    val a = (sdot1 - sdot0)
    val b = sdot0

    val uSmc =
      if (abs(a) < 1e-8) 0.0 else (desiredSdot - b) / a

    // Cart-centering term (gated by |theta|)
    val thetaAbs = abs(s.theta)
    val gate = clamp(1.0 - (thetaAbs / c.thetaGate), 0.0, 1.0)

    val uHold = gate * (-c.holdKp * s.x - c.holdKd * s.xdot)

    // Combine and saturate
    clamp(uSmc + uHold, -c.uMax, c.uMax)
  }

  // ------------------------------------------------------------
  // Rendering helpers (Java2D)
  // ------------------------------------------------------------

  private final case class RenderConfig(
      W: Int = 1000,
      H: Int = 600,
      pixelsPerMeter: Double = 180.0,
      originX: Double = 500.0,
      originY: Double = 450.0, // H * 0.75
      cartW: Double = 140.0,
      cartH: Double = 40.0,
      wheelR: Double = 14.0,
      poleWidth: Double = 8.0,
      bobR: Double = 9.0,
      arrowLenPx: Double = 120.0
  )

  /**
    * Draws a constant-length force arrow.
    */
  private def drawForceArrow(g: java.awt.Graphics2D, startX: Double, startY: Double, endX: Double, endY: Double, color: Color): Unit = {
    g.setColor(color)
    g.setStroke(new BasicStroke(4.0f))

    // Main arrow line
    g.draw(new Line2D.Double(startX, startY, endX, endY))

    // Arrowhead geometry
    val dx = endX - startX
    val dy = endY - startY
    val len = sqrt(dx * dx + dy * dy)
    if (len < 1e-6) return

    val ux = dx / len
    val uy = dy / len

    // Perpendicular unit vector
    val nx = -uy
    val ny = ux

    val headLen = 14.0
    val headW = 7.0

    val tipX = endX
    val tipY = endY
    val baseX = endX - ux * headLen
    val baseY = endY - uy * headLen

    val x1 = baseX + nx * headW
    val y1 = baseY + ny * headW
    val x2 = baseX - nx * headW
    val y2 = baseY - ny * headW

    val poly = new java.awt.Polygon()
    poly.addPoint(tipX.toInt, tipY.toInt)
    poly.addPoint(x1.toInt, y1.toInt)
    poly.addPoint(x2.toInt, y2.toInt)
    g.fill(poly)
  }

  /**
    * Renders one frame to a BufferedImage using the current integrated state.
    */
  private def renderFrame(cfg: RenderConfig, plant: CartPoleParams, s: State, arrowVisible: Boolean, arrowDirSign: Double): BufferedImage = {
    val img = new BufferedImage(cfg.W, cfg.H, BufferedImage.TYPE_INT_ARGB)
    val g = img.createGraphics()

    // Enable anti-aliasing for smoother visuals
    g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON)
    g.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON)

    // Colors
    val bg = new Color(20, 20, 20)
    val trackColor = new Color(170, 170, 170)
    val cartColor = new Color(40, 140, 255)
    val poleColor = new Color(240, 70, 70)
    val bobColor = new Color(255, 220, 60)
    val wheelColor = new Color(70, 70, 70)
    val arrowColor = new Color(60, 255, 120)

    // Background
    g.setColor(bg)
    g.fillRect(0, 0, cfg.W, cfg.H)

    // Track line
    g.setColor(trackColor)
    g.setStroke(new BasicStroke(4.0f))
    val trackY = cfg.originY + 25.0
    g.draw(new Line2D.Double(50.0, trackY, cfg.W - 50.0, trackY))

    // Center marker
    g.setColor(new Color(120, 120, 120))
    g.setStroke(new BasicStroke(3.0f))
    g.draw(new Line2D.Double(cfg.originX, cfg.originY - 30.0, cfg.originX, cfg.originY + 30.0))

    // Cart position (meters -> pixels)
    val cartX = cfg.originX + s.x * cfg.pixelsPerMeter
    val cartY = cfg.originY

    // Cart rectangle
    g.setColor(cartColor)
    g.fill(new Rectangle2D.Double(cartX - cfg.cartW * 0.5, cartY - cfg.cartH * 0.5, cfg.cartW, cfg.cartH))

    // Wheels
    g.setColor(wheelColor)
    val w1x = cartX - cfg.cartW * 0.30
    val w2x = cartX + cfg.cartW * 0.30
    val wy = cartY + cfg.cartH * 0.55
    g.fill(new Ellipse2D.Double(w1x - cfg.wheelR, wy - cfg.wheelR, 2 * cfg.wheelR, 2 * cfg.wheelR))
    g.fill(new Ellipse2D.Double(w2x - cfg.wheelR, wy - cfg.wheelR, 2 * cfg.wheelR, 2 * cfg.wheelR))

    // Pivot at top center of cart
    val pivotX = cartX
    val pivotY = cartY - cfg.cartH * 0.5

    // Pole rendering as a thin rectangle rotated about pivot
    val poleLenPx = plant.pole.L * cfg.pixelsPerMeter
    val poleRect = new Rectangle2D.Double(-cfg.poleWidth * 0.5, -poleLenPx, cfg.poleWidth, poleLenPx)

    val oldTx = g.getTransform
    g.setColor(poleColor)

    val tx = new AffineTransform()
    tx.translate(pivotX, pivotY)
    tx.rotate(s.theta) // positive theta leans right
    g.setTransform(tx)
    g.fill(poleRect)

    // Restore transform for world-space drawing
    g.setTransform(oldTx)

    // Bob at pole tip
    val tipX = pivotX + poleLenPx * sin(s.theta)
    val tipY = pivotY - poleLenPx * cos(s.theta)
    g.setColor(bobColor)
    g.fill(new Ellipse2D.Double(tipX - cfg.bobR, tipY - cfg.bobR, 2 * cfg.bobR, 2 * cfg.bobR))

    // Disturbance arrow: constant length and placed above the bob
    if (arrowVisible) {
      val dir = if (arrowDirSign >= 0.0) 1.0 else -1.0
      val startX = tipX
      val startY = tipY - 25.0
      val endX = tipX + dir * cfg.arrowLenPx
      val endY = tipY - 25.0
      drawForceArrow(g, startX, startY, endX, endY, arrowColor)
    }

    g.dispose()
    img
  }

  private final class PreviewPanel extends JPanel {
    @volatile private var latest: BufferedImage = _

    def updateImage(img: BufferedImage): Unit = {
      latest = img
      repaint()
    }

    override def paintComponent(gr: java.awt.Graphics): Unit = {
      super.paintComponent(gr)
      val img = latest
      if (img != null) {
        gr.drawImage(img, 0, 0, getWidth, getHeight, null)
      }
    }
  }

  // ------------------------------------------------------------
  // MP4 encoding via ffmpeg
  // ------------------------------------------------------------
  private def ffmpegAvailable(): Boolean = {
    Try {
      val p = new ProcessBuilder("ffmpeg", "-version").redirectErrorStream(true).start()
      val rc = p.waitFor()
      rc == 0
    }.getOrElse(false)
  }

  private def encodeMp4WithFfmpeg(framesDir: Path, fps: Int, outMp4: Path): Unit = {
    if (!ffmpegAvailable()) {
      System.err.println("ffmpeg not found on PATH; MP4 will not be created.")
      return
    }

    val pattern = framesDir.resolve("frame_%06d.png").toString

    // ProcessBuilder handles quoting safely by passing arguments as an array.
    val cmd = Seq(
      "ffmpeg",
      "-y",
      "-framerate", fps.toString,
      "-i", pattern,
      "-c:v", "libx264",
      "-pix_fmt", "yuv420p",
      outMp4.toString
    )

    println("Encoding MP4 with ffmpeg...")
    val pb = new ProcessBuilder(cmd: _*)
    pb.redirectErrorStream(true)

    val proc = pb.start()

    // Read and discard output to prevent potential buffer blocking
    val in = proc.getInputStream
    val buf = new Array[Byte](8192)
    while (in.read(buf) != -1) {}
    in.close()

    val rc = proc.waitFor()
    if (rc == 0) println(s"MP4 created: $outMp4")
    else System.err.println("ffmpeg encoding failed.")
  }

  // ------------------------------------------------------------
  // Plot saving
  // ------------------------------------------------------------
  private def savePlots(
      outDir: Path,
      t: Seq[Double],
      x: Seq[Double],
      theta: Seq[Double],
      u: Seq[Double],
      fEq: Seq[Double],
      tauExt: Seq[Double],
      sSurf: Seq[Double]
  ): Unit = {

    PlotUtils.saveLinePlot(
      title = "Cart Position x(t)",
      xlabel = "time (s)",
      ylabel = "x (m)",
      x = t,
      y = x,
      outFile = outDir.resolve("cart_position.png")
    )

    PlotUtils.saveLinePlot(
      title = "Pole Angle theta(t) (0=upright)",
      xlabel = "time (s)",
      ylabel = "theta (rad)",
      x = t,
      y = theta,
      outFile = outDir.resolve("pole_angle.png")
    )

    PlotUtils.saveLinePlot(
      title = "Control Force u(t) (SMC)",
      xlabel = "time (s)",
      ylabel = "u (N)",
      x = t,
      y = u,
      outFile = outDir.resolve("control_force.png")
    )

    PlotUtils.saveLinePlot(
      title = "External Disturbance Torque tau_ext(t)",
      xlabel = "time (s)",
      ylabel = "tau_ext (N*m)",
      x = t,
      y = tauExt,
      outFile = outDir.resolve("disturbance_torque.png")
    )

    PlotUtils.saveLinePlot(
      title = "Equivalent Bob Force (for arrow direction) F_eq(t) = tau/L",
      xlabel = "time (s)",
      ylabel = "F_eq (N)",
      x = t,
      y = fEq,
      outFile = outDir.resolve("equivalent_bob_force.png")
    )

    PlotUtils.saveLinePlot(
      title = "Sliding Surface s(t)",
      xlabel = "time (s)",
      ylabel = "s",
      x = t,
      y = sSurf,
      outFile = outDir.resolve("sliding_surface.png")
    )
  }

  // ------------------------------------------------------------
  // Main
  // ------------------------------------------------------------
  def main(args: Array[String]): Unit = {
    // Ensure plots use English/Latin digits regardless of the Windows display language.
    Locale.setDefault(Locale.US)

    // Optional runtime flags:
    //   --preview         show a window while generating frames
    //   --frames <N>      override number of frames (default: 6000)
    var preview = false
    var totalFrames = 6000

    var ai = 0
    while (ai < args.length) {
      args(ai) match {
        case "--preview" =>
          preview = true
          ai += 1
        case "--frames" if ai + 1 < args.length =>
          totalFrames = args(ai + 1).toInt
          ai += 2
        case _ =>
          ai += 1
      }
    }

    // Ensure headless mode for faster offscreen rendering when preview is disabled
    if (!preview) System.setProperty("java.awt.headless", "true")

    // ----------------------------
    // Output folders
    // ----------------------------
    val outDir: Path = Paths.get("output", "pendulum_sliding_mode")
    val framesDir: Path = outDir.resolve("frames")
    Files.createDirectories(framesDir)

    // Clean old frames
    if (Files.exists(framesDir)) {
      val ds = Files.newDirectoryStream(framesDir)
      try {
        val it = ds.iterator()
        while (it.hasNext) {
          val p = it.next()
          if (Files.isRegularFile(p)) Files.deleteIfExists(p)
        }
      } finally {
        ds.close()
      }
    }

    // ----------------------------
    // Simulation settings
    // ----------------------------
    val videoSeconds = 10.0
    val fps = (totalFrames.toDouble / videoSeconds).round.toInt

    val dtFrame = videoSeconds / totalFrames.toDouble

    // Physics integration step: use multiple substeps per frame for stability
    val substeps = 8
    val dtPhysics = dtFrame / substeps.toDouble

    // Screen / rendering
    val cfg = RenderConfig(
      W = 1000,
      H = 600,
      originX = 1000.0 * 0.5,
      originY = 600.0 * 0.75
    )

    // ----------------------------
    // Plant / controller / disturbance
    // ----------------------------
    val plant = new CartPoleParams
    plant.M = 1.2
    plant.pole.L = 1.0
    plant.pole.mRod = 0.1
    plant.pole.mBob = 0.15
    plant.pole.computeDerived()

    val ctrl = new ControlParams
    val track = new TrackLimits
    val dist = new Disturbance

    // Initial state
    val s = new State
    s.theta = 0.20

    // ----------------------------
    // Logging (one entry per frame)
    // ----------------------------
    val tLog = collection.mutable.ArrayBuffer.empty[Double]
    val xLog = collection.mutable.ArrayBuffer.empty[Double]
    val thLog = collection.mutable.ArrayBuffer.empty[Double]
    val uLog = collection.mutable.ArrayBuffer.empty[Double]
    val fEqLog = collection.mutable.ArrayBuffer.empty[Double]
    val tauLog = collection.mutable.ArrayBuffer.empty[Double]
    val surfLog = collection.mutable.ArrayBuffer.empty[Double]

    tLog.sizeHint(totalFrames)

    // ----------------------------
    // Optional preview window (Swing)
    // ----------------------------
    var previewPanel: Option[PreviewPanel] = None
    var previewFrame: Option[JFrame] = None

    if (preview) {
      val panel = new PreviewPanel
      previewPanel = Some(panel)

      SwingUtilities.invokeLater(() => {
        val frame = new JFrame("Cart-Pole SMC Preview")
        // In Scala, Java static members are accessed from the class that defines them.
        // Swing's close-operation constants are defined on WindowConstants.
        frame.setDefaultCloseOperation(javax.swing.WindowConstants.DISPOSE_ON_CLOSE)
        frame.setSize(cfg.W, cfg.H)
        frame.setResizable(false)
        frame.add(panel)
        frame.setVisible(true)
        previewFrame = Some(frame)
      })
    }

    // ----------------------------
    // Main loop: frame generation
    // ----------------------------
    var t = 0.0

    var frameIdx = 0
    while (frameIdx < totalFrames) {
      // Integrate dynamics for this frame using substeps
      var uUsed = 0.0
      var tauUsed = 0.0
      var fEqUsed = 0.0
      var sUsed = 0.0

      var k = 0
      while (k < substeps) {
        // Disturbance torque at current time
        val tauExtNow = dist.tauExt(t)

        // Control force (SMC)
        val u = computeControl(plant, ctrl, s)

        // Integrate one physics step
        rk4Step(plant, s, dtPhysics, u, tauExtNow)

        // Keep cart on track
        track.enforce(s)

        // Save latest values for logging/rendering
        uUsed = u
        tauUsed = tauExtNow
        fEqUsed = dist.bobForceEquivalent(t, plant.pole.L)
        sUsed = slidingSurface(ctrl, s)

        // Advance continuous time
        t += dtPhysics
        k += 1
      }

      // Log once per frame using the frame time (not the internal integrator time)
      val tf = frameIdx.toDouble * dtFrame
      tLog += tf
      xLog += s.x
      thLog += s.theta
      uLog += uUsed
      tauLog += tauUsed
      fEqLog += fEqUsed
      surfLog += sUsed

      // Render using the current integrated state
      val arrowVis = dist.arrowVisible(tf)
      val img = renderFrame(cfg, plant, s, arrowVis, arrowDirSign = fEqUsed)

      // Save frame to PNG with zero-padded name
      val fn = framesDir.resolve(f"frame_$frameIdx%06d.png")
      ImageIO.write(img, "png", fn.toFile)

      // Optional preview update
      previewPanel.foreach(_.updateImage(img))

      // Console progress every ~1 second
      if (frameIdx % fps == 0) {
        println(f"Frame $frameIdx/$totalFrames  t=$tf%.2f  x=${s.x}%.3f  theta=${s.theta}%.3f  u=$uUsed%.2f  tau=$tauUsed%.3f")
      }

      frameIdx += 1
    }

    // Close preview window if it exists
    previewFrame.foreach(f => SwingUtilities.invokeLater(() => f.dispose()))

    // ----------------------------
    // Encode MP4 (if ffmpeg is available)
    // ----------------------------
    val mp4 = outDir.resolve("pendulum_smc_10s_6000f.mp4")
    encodeMp4WithFfmpeg(framesDir, fps, mp4)

    // ----------------------------
    // Save plots + CSV
    // ----------------------------
    println("Saving plots and CSV...")

    savePlots(outDir, tLog.toSeq, xLog.toSeq, thLog.toSeq, uLog.toSeq, fEqLog.toSeq, tauLog.toSeq, surfLog.toSeq)

    CsvUtils.writeColumns(
      outDir.resolve("cartpole_log.csv"),
      header = Seq("t", "x", "theta", "u", "F_equiv", "tau_ext", "s"),
      cols = Seq(tLog.toSeq, xLog.toSeq, thLog.toSeq, uLog.toSeq, fEqLog.toSeq, tauLog.toSeq, surfLog.toSeq)
    )

    println("Done.")
  }
}
