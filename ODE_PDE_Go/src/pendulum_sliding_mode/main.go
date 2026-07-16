// ------------------------------------------------------------
// Cart–Pole Inverted Pendulum (moving base) with SLIDING MODE CONTROL (SMC)
// ------------------------------------------------------------
// Requirements implemented:
//   - Total simulation/animation time: 10 seconds
//   - Total frames: 6000  (=> 600 FPS video)
//   - Exactly two disturbances: at t=0.5s and t=5.0s
//   - Disturbance shown as an arrow for 0.5s only (direction indicator, constant length)
//   - Dynamics are integrated and used to render each frame
//   - Outputs: frames, mp4 (ffmpeg), plots (Gonum Plot), CSV log
//
// Output folders:
//   output/pendulum_sliding_mode/frames/frame_000000.png ... frame_005999.png
//   output/pendulum_sliding_mode/pendulum_smc_10s_6000f.mp4   (if ffmpeg in PATH)
//   output/pendulum_sliding_mode/*.png plots
//   output/pendulum_sliding_mode/cartpole_log.csv
// ------------------------------------------------------------

package main

import (
	"bufio"
	"encoding/csv"
	"errors"
	"flag"
	"fmt"
	"image"
	"image/color"
	"image/png"
	"log"
	"math"
	"os"
	"os/exec"
	"path/filepath"
	"sort"
	"strings"

	"gonum.org/v1/plot"
	"gonum.org/v1/plot/plotter"
	"gonum.org/v1/plot/vg"
	"gonum.org/v1/plot/vg/draw"
	"gonum.org/v1/plot/vg/vgimg"
)

// ------------------------------------------------------------
// Small math helpers
// ------------------------------------------------------------

// clamp bounds x into [lo, hi].
func clamp(x, lo, hi float64) float64 {
	if x < lo {
		return lo
	}
	if x > hi {
		return hi
	}
	return x
}

// wrapToPi wraps an angle to [-π, π] to avoid numeric drift.
func wrapToPi(a float64) float64 {
	for a > math.Pi {
		a -= 2.0 * math.Pi
	}
	for a < -math.Pi {
		a += 2.0 * math.Pi
	}
	return a
}

// sat implements a boundary-layer saturation used in sliding mode control.
func sat(z float64) float64 { return clamp(z, -1.0, 1.0) }

// ------------------------------------------------------------
// CSV writer
// ------------------------------------------------------------

func writeCSV(filename string, header []string, cols [][]float64) error {
	if len(cols) == 0 {
		return errors.New("CSV: no columns")
	}
	n := len(cols[0])
	for _, c := range cols {
		if len(c) != n {
			return errors.New("CSV: column size mismatch")
		}
	}

	if err := os.MkdirAll(filepath.Dir(filename), 0o755); err != nil {
		return fmt.Errorf("CSV: cannot create directory: %w", err)
	}

	f, err := os.Create(filename)
	if err != nil {
		return fmt.Errorf("CSV: cannot open %s: %w", filename, err)
	}
	defer f.Close()

	w := csv.NewWriter(f)
	defer w.Flush()

	if err := w.Write(header); err != nil {
		return fmt.Errorf("CSV: cannot write header: %w", err)
	}

	for r := 0; r < n; r++ {
		row := make([]string, len(cols))
		for c := range cols {
			row[c] = fmt.Sprintf("%.15g", cols[c][r])
		}
		if err := w.Write(row); err != nil {
			return fmt.Errorf("CSV: cannot write row: %w", err)
		}
	}
	return nil
}

// ------------------------------------------------------------
// Plant model: uniform rod + lumped bob at the top
// ------------------------------------------------------------

type PoleModel struct {
	L    float64 // (m)
	MRod float64 // (kg)
	MBob float64 // (kg)

	// Derived quantities
	MTotal        float64
	LCom          float64
	IPivot        float64
	ICom          float64
	InertiaFactor float64 // 1 + I_com/(m*l^2)
}

func (pm *PoleModel) ComputeDerived() {
	pm.MTotal = pm.MRod + pm.MBob

	// Center of mass from pivot: rod at L/2, bob at L
	pm.LCom = (pm.MRod*(pm.L*0.5) + pm.MBob*pm.L) / pm.MTotal

	// Inertia about pivot: rod (1/3)mL^2, bob mL^2
	pm.IPivot = (1.0/3.0)*pm.MRod*pm.L*pm.L + pm.MBob*pm.L*pm.L

	// Inertia about COM
	pm.ICom = pm.IPivot - pm.MTotal*pm.LCom*pm.LCom

	// Inertia factor commonly used in compact cart–pole dynamics
	pm.InertiaFactor = 1.0 + pm.ICom/(pm.MTotal*pm.LCom*pm.LCom)
}

type CartPoleParams struct {
	M float64 // cart mass (kg)
	G float64 // gravity (m/s^2)

	CartDamping float64
	PoleDamping float64

	Pole PoleModel
}

type State struct {
	X        float64
	XDot     float64
	Theta    float64 // theta=0 is upright
	ThetaDot float64
}

type ControlParams struct {
	// Sliding surface:
	// s = theta_dot + lambda_theta*theta + alpha*(x_dot + lambda_x*x)
	LambdaTheta float64
	LambdaX     float64
	Alpha       float64

	// Sliding mode dynamics:
	K   float64
	Phi float64

	// Actuator saturation
	UMax float64

	// Cart centering (only near upright)
	HoldKp float64
	HoldKd float64

	// Gate: when |theta| >= theta_gate, centering is ~0
	ThetaGate float64
}

type TrackLimits struct {
	XMax float64
}

func (tl TrackLimits) Enforce(s *State) {
	if s.X > tl.XMax {
		s.X = tl.XMax
		if s.XDot > 0 {
			s.XDot = 0
		}
	}
	if s.X < -tl.XMax {
		s.X = -tl.XMax
		if s.XDot < 0 {
			s.XDot = 0
		}
	}
}

// ------------------------------------------------------------
// Disturbance schedule (exactly two disturbances)
// ------------------------------------------------------------

type Disturbance struct {
	T1, T2        float64
	ArrowDuration float64
	Duration      float64
	TauAmp        float64
}

func halfSine(localT, duration float64) float64 {
	return math.Sin(math.Pi * localT / duration)
}

// TauExt returns the external torque about the pole pivot (N*m).
// First pulse: positive torque ("push right")
// Second pulse: negative torque ("push left")
func (d Disturbance) TauExt(t float64) float64 {
	if t >= d.T1 && t <= d.T1+d.Duration {
		local := t - d.T1
		return +d.TauAmp * halfSine(local, d.Duration)
	}
	if t >= d.T2 && t <= d.T2+d.Duration {
		local := t - d.T2
		return -d.TauAmp * halfSine(local, d.Duration)
	}
	return 0.0
}

// BobForceEquivalent converts the torque into an equivalent tip force for
// arrow direction only: F_eq = tau/L.
func (d Disturbance) BobForceEquivalent(t, L float64) float64 {
	return d.TauExt(t) / L
}

func (d Disturbance) ArrowVisible(t float64) bool {
	if t >= d.T1 && t <= d.T1+d.ArrowDuration {
		return true
	}
	if t >= d.T2 && t <= d.T2+d.ArrowDuration {
		return true
	}
	return false
}

// ------------------------------------------------------------
// ODE derivative
// ------------------------------------------------------------

type Deriv struct {
	XDot      float64
	XDDot     float64
	ThetaDot  float64
	ThetaDDot float64
}

// dynamics computes the cart–pole derivatives given control force (cart) and external torque.
func dynamics(p CartPoleParams, s State, uCart, tauExt float64) Deriv {
	m := p.Pole.MTotal
	l := p.Pole.LCom

	totalMass := p.M + m
	poleMassLen := m * l

	sinT := math.Sin(s.Theta)
	cosT := math.Cos(s.Theta)

	// Cart damping (simple viscous term)
	fDamped := uCart - p.CartDamping*s.XDot

	temp := (fDamped + poleMassLen*s.ThetaDot*s.ThetaDot*sinT) / totalMass

	denom := l * (p.Pole.InertiaFactor - (m*cosT*cosT)/totalMass)

	thetaDDot := (p.G*sinT - cosT*temp) / denom

	// Pole damping
	thetaDDot -= p.PoleDamping * s.ThetaDot

	// External torque disturbance about pivot
	thetaDDot += tauExt / p.Pole.IPivot

	xDDot := temp - poleMassLen*thetaDDot*cosT/totalMass

	return Deriv{
		XDot:      s.XDot,
		XDDot:     xDDot,
		ThetaDot:  s.ThetaDot,
		ThetaDDot: thetaDDot,
	}
}

// rk4Step advances the state by one RK4 step.
func rk4Step(p CartPoleParams, s *State, dt, uCart, tauExt float64) {
	addScaled := func(a State, k Deriv, h float64) State {
		out := a
		out.X += h * k.XDot
		out.XDot += h * k.XDDot
		out.Theta += h * k.ThetaDot
		out.ThetaDot += h * k.ThetaDDot
		return out
	}

	k1 := dynamics(p, *s, uCart, tauExt)
	k2 := dynamics(p, addScaled(*s, k1, 0.5*dt), uCart, tauExt)
	k3 := dynamics(p, addScaled(*s, k2, 0.5*dt), uCart, tauExt)
	k4 := dynamics(p, addScaled(*s, k3, dt), uCart, tauExt)

	s.X += (dt / 6.0) * (k1.XDot + 2.0*k2.XDot + 2.0*k3.XDot + k4.XDot)
	s.XDot += (dt / 6.0) * (k1.XDDot + 2.0*k2.XDDot + 2.0*k3.XDDot + k4.XDDot)
	s.Theta += (dt / 6.0) * (k1.ThetaDot + 2.0*k2.ThetaDot + 2.0*k3.ThetaDot + k4.ThetaDot)
	s.ThetaDot += (dt / 6.0) * (k1.ThetaDDot + 2.0*k2.ThetaDDot + 2.0*k3.ThetaDDot + k4.ThetaDDot)

	s.Theta = wrapToPi(s.Theta)
}

// slidingSurface computes the SMC surface value s(x).
func slidingSurface(c ControlParams, s State) float64 {
	return s.ThetaDot +
		c.LambdaTheta*s.Theta +
		c.Alpha*(s.XDot+c.LambdaX*s.X)
}

// slidingSurfaceDotNominal estimates sdot under nominal assumptions (disturbance ignored).
func slidingSurfaceDotNominal(p CartPoleParams, c ControlParams, s State, uCart float64) float64 {
	d := dynamics(p, s, uCart, 0.0)
	return d.ThetaDDot +
		c.LambdaTheta*s.ThetaDot +
		c.Alpha*(d.XDDot+c.LambdaX*s.XDot)
}

// computeControl computes the saturated cart force using a practical numeric affine approximation.
func computeControl(p CartPoleParams, c ControlParams, s State) float64 {
	// Sliding mode: enforce sdot ≈ -k*sat(s/phi)
	sval := slidingSurface(c, s)
	desiredSDot := -c.K * sat(sval/c.Phi)

	// Numeric affine approximation: sdot(u) ≈ a*u + b
	sdot0 := slidingSurfaceDotNominal(p, c, s, 0.0)
	sdot1 := slidingSurfaceDotNominal(p, c, s, 1.0)
	a := (sdot1 - sdot0)
	b := sdot0

	uSmc := 0.0
	if math.Abs(a) >= 1e-8 {
		uSmc = (desiredSDot - b) / a
	}

	// Gated cart-centering term: active only near upright.
	thetaAbs := math.Abs(s.Theta)
	gate := clamp(1.0-thetaAbs/c.ThetaGate, 0.0, 1.0)
	uHold := gate * (-c.HoldKp*s.X - c.HoldKd*s.XDot)

	uTotal := uSmc + uHold
	return clamp(uTotal, -c.UMax, c.UMax)
}

// ------------------------------------------------------------
// Plotting helpers (high-resolution PNG with 300 DPI)
// ------------------------------------------------------------

func limitedTicker(maxLabels int, labelFmt string) plot.Ticker {
	if maxLabels < 2 {
		maxLabels = 2
	}
	return plot.TickerFunc(func(min, max float64) []plot.Tick {
		if math.IsNaN(min) || math.IsNaN(max) || math.IsInf(min, 0) || math.IsInf(max, 0) {
			return nil
		}
		if min == max {
			return []plot.Tick{{Value: min, Label: fmt.Sprintf(labelFmt, min)}}
		}
		step := (max - min) / float64(maxLabels-1)
		ticks := make([]plot.Tick, 0, maxLabels)
		for i := 0; i < maxLabels; i++ {
			v := min + float64(i)*step
			ticks = append(ticks, plot.Tick{Value: v, Label: fmt.Sprintf(labelFmt, v)})
		}
		return ticks
	})
}

func stylePlot(p *plot.Plot) {
	p.Title.TextStyle.Font.Size = vg.Points(22)
	p.Title.Padding = vg.Points(12)

	p.X.Label.TextStyle.Font.Size = vg.Points(18)
	p.Y.Label.TextStyle.Font.Size = vg.Points(18)
	p.X.Label.Padding = vg.Points(10)
	p.Y.Label.Padding = vg.Points(10)

	p.X.LineStyle.Width = vg.Points(2.2)
	p.Y.LineStyle.Width = vg.Points(2.2)
	p.X.Padding = vg.Points(20)
	p.Y.Padding = vg.Points(20)

	p.X.Tick.LineStyle.Width = vg.Points(2.0)
	p.Y.Tick.LineStyle.Width = vg.Points(2.0)
	p.X.Tick.Length = vg.Points(8)
	p.Y.Tick.Length = vg.Points(8)

	p.X.Tick.Label.Font.Size = vg.Points(14)
	p.Y.Tick.Label.Font.Size = vg.Points(14)

	p.X.Tick.Marker = limitedTicker(10, "%.1f")
	p.Y.Tick.Marker = limitedTicker(10, "%.1f")
}

func savePlotPNG(p *plot.Plot, widthIn, heightIn float64, filename string) error {
	if err := os.MkdirAll(filepath.Dir(filename), 0o755); err != nil {
		return fmt.Errorf("cannot create directory: %w", err)
	}
	w := vg.Length(widthIn) * vg.Inch
	h := vg.Length(heightIn) * vg.Inch

	c := vgimg.NewWith(
		vgimg.UseWH(w, h),
		vgimg.UseDPI(300),
	)
	dc := draw.New(c)
	p.Draw(dc)

	f, err := os.Create(filename)
	if err != nil {
		return fmt.Errorf("cannot create png: %w", err)
	}
	defer f.Close()

	bw := bufio.NewWriter(f)
	defer bw.Flush()

	pngc := vgimg.PngCanvas{Canvas: c}
	if _, err := pngc.WriteTo(bw); err != nil {
		return fmt.Errorf("cannot write png: %w", err)
	}
	return nil
}

func saveLinePlot(outDir, filename, title, xlabel, ylabel string, xs, ys []float64) error {
	if len(xs) != len(ys) || len(xs) == 0 {
		return fmt.Errorf("plot data invalid")
	}
	p := plot.New()
	p.Title.Text = title
	p.X.Label.Text = xlabel
	p.Y.Label.Text = ylabel
	stylePlot(p)

	pts := make(plotter.XYs, len(xs))
	for i := range xs {
		pts[i].X = xs[i]
		pts[i].Y = ys[i]
	}
	line, err := plotter.NewLine(pts)
	if err != nil {
		return err
	}
	line.LineStyle.Width = vg.Points(3.0)
	p.Add(line)

	return savePlotPNG(p, 8.0, 6.0, filepath.Join(outDir, filename))
}

func savePlots(outDir string, t, x, theta, u, Feq, tau, s []float64) error {
	if err := saveLinePlot(outDir, "cart_position.png", "Cart Position x(t)", "time (s)", "x (m)", t, x); err != nil {
		return err
	}
	if err := saveLinePlot(outDir, "pole_angle.png", "Pole Angle theta(t) (0 = upright)", "time (s)", "theta (rad)", t, theta); err != nil {
		return err
	}
	if err := saveLinePlot(outDir, "control_force.png", "Control Force u(t) (SMC)", "time (s)", "u (N)", t, u); err != nil {
		return err
	}
	if err := saveLinePlot(outDir, "disturbance_torque.png", "External Disturbance Torque tau_ext(t)", "time (s)", "tau_ext (N*m)", t, tau); err != nil {
		return err
	}
	if err := saveLinePlot(outDir, "equivalent_bob_force.png", "Equivalent Bob Force F_eq(t) = tau/L", "time (s)", "F_eq (N)", t, Feq); err != nil {
		return err
	}
	if err := saveLinePlot(outDir, "sliding_surface.png", "Sliding Surface s(t)", "time (s)", "s", t, s); err != nil {
		return err
	}
	return nil
}

// ------------------------------------------------------------
// Minimal PNG rendering helpers (frame generation)
// ------------------------------------------------------------

func fill(img *image.RGBA, c color.RGBA) {
	b := img.Bounds()
	for y := b.Min.Y; y < b.Max.Y; y++ {
		for x := b.Min.X; x < b.Max.X; x++ {
			img.SetRGBA(x, y, c)
		}
	}
}

func drawRectFilled(img *image.RGBA, cx, cy, w, h float64, c color.RGBA) {
	minX := int(math.Round(cx - w/2))
	maxX := int(math.Round(cx + w/2))
	minY := int(math.Round(cy - h/2))
	maxY := int(math.Round(cy + h/2))

	b := img.Bounds()
	if minX < b.Min.X {
		minX = b.Min.X
	}
	if maxX > b.Max.X {
		maxX = b.Max.X
	}
	if minY < b.Min.Y {
		minY = b.Min.Y
	}
	if maxY > b.Max.Y {
		maxY = b.Max.Y
	}

	for y := minY; y < maxY; y++ {
		for x := minX; x < maxX; x++ {
			img.SetRGBA(x, y, c)
		}
	}
}

func drawCircleFilled(img *image.RGBA, cx, cy, r float64, c color.RGBA) {
	minX := int(math.Floor(cx - r))
	maxX := int(math.Ceil(cx + r))
	minY := int(math.Floor(cy - r))
	maxY := int(math.Ceil(cy + r))

	rsq := r * r
	b := img.Bounds()

	for y := minY; y <= maxY; y++ {
		if y < b.Min.Y || y >= b.Max.Y {
			continue
		}
		for x := minX; x <= maxX; x++ {
			if x < b.Min.X || x >= b.Max.X {
				continue
			}
			dx := (float64(x) + 0.5) - cx
			dy := (float64(y) + 0.5) - cy
			if dx*dx+dy*dy <= rsq {
				img.SetRGBA(x, y, c)
			}
		}
	}
}

// drawThickLine draws a thick line by stamping small circles along the segment.
func drawThickLine(img *image.RGBA, x1, y1, x2, y2, width float64, c color.RGBA) {
	dx := x2 - x1
	dy := y2 - y1
	dist := math.Hypot(dx, dy)
	if dist < 1e-6 {
		drawCircleFilled(img, x1, y1, width/2, c)
		return
	}
	steps := int(dist / 0.8) // small step for smooth thickness
	if steps < 1 {
		steps = 1
	}
	for i := 0; i <= steps; i++ {
		t := float64(i) / float64(steps)
		x := x1 + t*dx
		y := y1 + t*dy
		drawCircleFilled(img, x, y, width/2, c)
	}
}

func drawTriangleFilled(img *image.RGBA, ax, ay, bx, by, cx, cy float64, col color.RGBA) {
	// Simple bounding-box fill with barycentric test.
	minX := int(math.Floor(math.Min(ax, math.Min(bx, cx))))
	maxX := int(math.Ceil(math.Max(ax, math.Max(bx, cx))))
	minY := int(math.Floor(math.Min(ay, math.Min(by, cy))))
	maxY := int(math.Ceil(math.Max(ay, math.Max(by, cy))))

	b := img.Bounds()
	if minX < b.Min.X {
		minX = b.Min.X
	}
	if maxX > b.Max.X {
		maxX = b.Max.X
	}
	if minY < b.Min.Y {
		minY = b.Min.Y
	}
	if maxY > b.Max.Y {
		maxY = b.Max.Y
	}

	edge := func(x0, y0, x1, y1, x, y float64) float64 {
		return (x-x0)*(y1-y0) - (y-y0)*(x1-x0)
	}

	for y := minY; y < maxY; y++ {
		for x := minX; x < maxX; x++ {
			px := float64(x) + 0.5
			py := float64(y) + 0.5

			w0 := edge(bx, by, cx, cy, px, py)
			w1 := edge(cx, cy, ax, ay, px, py)
			w2 := edge(ax, ay, bx, by, px, py)

			if (w0 >= 0 && w1 >= 0 && w2 >= 0) || (w0 <= 0 && w1 <= 0 && w2 <= 0) {
				img.SetRGBA(x, y, col)
			}
		}
	}
}

// drawForceArrow draws a constant-length arrow between start and end.
func drawForceArrow(img *image.RGBA, x1, y1, x2, y2 float64, col color.RGBA) {
	// Shaft
	drawThickLine(img, x1, y1, x2, y2, 4.0, col)

	dx := x2 - x1
	dy := y2 - y1
	length := math.Hypot(dx, dy)
	if length < 1e-6 {
		return
	}

	ux := dx / length
	uy := dy / length

	// Perpendicular for the head width
	nx := -uy
	ny := ux

	headLen := 16.0
	headW := 8.0

	tipX, tipY := x2, y2
	baseX := x2 - ux*headLen
	baseY := y2 - uy*headLen

	leftX := baseX + nx*headW
	leftY := baseY + ny*headW
	rightX := baseX - nx*headW
	rightY := baseY - ny*headW

	drawTriangleFilled(img, tipX, tipY, leftX, leftY, rightX, rightY, col)
}

// listFilesSorted lists files in a directory matching a suffix, sorted by name.
func listFilesSorted(dir, suffix string) ([]string, error) {
	ents, err := os.ReadDir(dir)
	if err != nil {
		return nil, err
	}
	out := make([]string, 0, len(ents))
	for _, e := range ents {
		if e.IsDir() {
			continue
		}
		name := e.Name()
		if strings.HasSuffix(strings.ToLower(name), strings.ToLower(suffix)) {
			out = append(out, filepath.Join(dir, name))
		}
	}
	sort.Strings(out)
	return out, nil
}

// cleanOldFrames removes existing PNG frames to avoid mixing runs.
func cleanOldFrames(framesDir string) error {
	if err := os.MkdirAll(framesDir, 0o755); err != nil {
		return err
	}
	files, err := listFilesSorted(framesDir, ".png")
	if err != nil {
		return err
	}
	for _, f := range files {
		if err := os.Remove(f); err != nil {
			return err
		}
	}
	return nil
}

// encodeMP4WithFFmpeg calls ffmpeg if it is available in PATH.
func encodeMP4WithFFmpeg(framesDir string, fps int, outMP4 string) {
	if _, err := exec.LookPath("ffmpeg"); err != nil {
		log.Printf("ffmpeg not found on PATH; MP4 will not be created.")
		return
	}

	cmd := exec.Command("ffmpeg",
		"-y",
		"-framerate", fmt.Sprintf("%d", fps),
		"-i", filepath.Join(framesDir, "frame_%06d.png"),
		"-c:v", "libx264",
		"-pix_fmt", "yuv420p",
		outMP4,
	)

	// Keep terminal output clean; show only errors if something fails.
	cmd.Stdout = nil
	cmd.Stderr = nil

	log.Printf("Encoding MP4 with ffmpeg...")
	if err := cmd.Run(); err != nil {
		log.Printf("ffmpeg encoding failed: %v", err)
		return
	}
	log.Printf("MP4 created: %s", outMP4)
}

func main() {
	// CLI flag maintained for a typical simulation workflow.
	// This implementation always renders offscreen frames and does not open a GUI.
	preview := flag.Bool("preview", false, "preview window (not used in this Go implementation)")
	flag.Parse()
	_ = preview

	// ----------------------------
	// Output folders
	// ----------------------------
	outDir := filepath.Join("output", "pendulum_sliding_mode")
	framesDir := filepath.Join(outDir, "frames")

	if err := os.MkdirAll(outDir, 0o755); err != nil {
		log.Fatalf("cannot create output dir: %v", err)
	}
	if err := cleanOldFrames(framesDir); err != nil {
		log.Fatalf("cannot clean frames: %v", err)
	}

	// ----------------------------
	// Simulation settings (exact)
	// ----------------------------
	videoSeconds := 10.0
	totalFrames := 6000
	fps := int(float64(totalFrames) / videoSeconds) // 600 FPS
	dtFrame := videoSeconds / float64(totalFrames)

	// Sub-stepping improves numerical stability for high-frame-rate output.
	substeps := 8
	dtPhysics := dtFrame / float64(substeps)

	// Render size
	W := 1000
	H := 600

	// ----------------------------
	// Plant / controller / disturbance
	// ----------------------------
	plant := CartPoleParams{
		M:           1.2,
		G:           9.81,
		CartDamping: 0.10,
		PoleDamping: 0.03,
		Pole: PoleModel{
			L:    1.0,
			MRod: 0.10,
			MBob: 0.15,
		},
	}
	plant.Pole.ComputeDerived()

	ctrl := ControlParams{
		LambdaTheta: 10.0,
		LambdaX:     1.5,
		Alpha:       0.55,
		K:           110.0,
		Phi:         0.05,
		UMax:        70.0,
		HoldKp:      8.0,
		HoldKd:      10.0,
		ThetaGate:   0.20,
	}

	track := TrackLimits{XMax: 1.6}

	dist := Disturbance{
		T1:            0.5,
		T2:            5.0,
		ArrowDuration: 0.5,
		Duration:      0.5,
		TauAmp:        3.3,
	}

	// Initial state
	s := State{
		X:        0.0,
		XDot:     0.0,
		Theta:    0.20,
		ThetaDot: 0.0,
	}

	// ----------------------------
	// Logging (one entry per frame)
	// ----------------------------
	tLog := make([]float64, 0, totalFrames)
	xLog := make([]float64, 0, totalFrames)
	thLog := make([]float64, 0, totalFrames)
	uLog := make([]float64, 0, totalFrames)
	FeqLog := make([]float64, 0, totalFrames)
	tauLog := make([]float64, 0, totalFrames)
	surfLog := make([]float64, 0, totalFrames)

	// ----------------------------
	// Visual mapping
	// ----------------------------
	pixelsPerMeter := 180.0
	originX := float64(W) * 0.5
	originY := float64(H) * 0.75

	bg := color.RGBA{20, 20, 20, 255}
	trackCol := color.RGBA{170, 170, 170, 255}
	cartCol := color.RGBA{40, 140, 255, 255}
	poleCol := color.RGBA{240, 70, 70, 255}
	bobCol := color.RGBA{255, 220, 60, 255}
	wheelCol := color.RGBA{70, 70, 70, 255}
	arrowCol := color.RGBA{60, 255, 120, 255}

	// Geometry
	cartW := 140.0
	cartH := 40.0
	wheelR := 14.0
	poleLenPx := plant.Pole.L * pixelsPerMeter
	arrowLenPx := 120.0

	// Re-use one RGBA buffer for every frame.
	img := image.NewRGBA(image.Rect(0, 0, W, H))

	// ----------------------------
	// Main loop: frames
	// ----------------------------
	t := 0.0
	for frame := 0; frame < totalFrames; frame++ {
		// Integrate dynamics for this frame using substeps
		var (
			uUsed   float64
			tauUsed float64
			FeqUsed float64
			sUsed   float64
		)

		for k := 0; k < substeps; k++ {
			// Disturbance torque
			tauExt := dist.TauExt(t)

			// Control force (SMC)
			u := computeControl(plant, ctrl, s)

			// Integrate
			rk4Step(plant, &s, dtPhysics, u, tauExt)

			// Keep cart on track
			track.Enforce(&s)

			// Latest values for logging/rendering
			uUsed = u
			tauUsed = tauExt
			FeqUsed = dist.BobForceEquivalent(t, plant.Pole.L)
			sUsed = slidingSurface(ctrl, s)

			t += dtPhysics
		}

		// Log once per frame using the frame time base (consistent with rendering)
		tf := float64(frame) * dtFrame
		tLog = append(tLog, tf)
		xLog = append(xLog, s.X)
		thLog = append(thLog, s.Theta)
		uLog = append(uLog, uUsed)
		tauLog = append(tauLog, tauUsed)
		FeqLog = append(FeqLog, FeqUsed)
		surfLog = append(surfLog, sUsed)

		// ----------------------------
		// Render using current integrated state
		// ----------------------------
		fill(img, bg)

		// Track line
		drawRectFilled(img, originX, originY+25.0, float64(W-100), 4.0, trackCol)

		// Center marker
		drawRectFilled(img, originX, originY, 3.0, 60.0, color.RGBA{120, 120, 120, 255})

		// Cart position
		cartX := originX + s.X*pixelsPerMeter
		cartY := originY

		// Cart body
		drawRectFilled(img, cartX, cartY, cartW, cartH, cartCol)

		// Wheels
		drawCircleFilled(img, cartX-cartW*0.30, cartY+cartH*0.55, wheelR, wheelCol)
		drawCircleFilled(img, cartX+cartW*0.30, cartY+cartH*0.55, wheelR, wheelCol)

		// Pivot at top center of cart
		pivotX := cartX
		pivotY := cartY - cartH*0.5

		// Pole tip (theta>0 leans to the right visually)
		tipX := pivotX + poleLenPx*math.Sin(s.Theta)
		tipY := pivotY - poleLenPx*math.Cos(s.Theta)

		// Pole as a thick line
		drawThickLine(img, pivotX, pivotY, tipX, tipY, 8.0, poleCol)

		// Bob
		drawCircleFilled(img, tipX, tipY, 9.0, bobCol)

		// Disturbance arrow shown only 0.5 s after each pulse start
		if dist.ArrowVisible(tf) {
			dir := 1.0
			if FeqUsed < 0.0 {
				dir = -1.0
			}
			startX := tipX
			startY := tipY - 25.0
			endX := tipX + dir*arrowLenPx
			endY := tipY - 25.0
			drawForceArrow(img, startX, startY, endX, endY, arrowCol)
		}

		// Save frame as PNG
		fn := filepath.Join(framesDir, fmt.Sprintf("frame_%06d.png", frame))
		f, err := os.Create(fn)
		if err != nil {
			log.Fatalf("cannot create frame: %v", err)
		}
		bw := bufio.NewWriter(f)
		if err := png.Encode(bw, img); err != nil {
			_ = f.Close()
			log.Fatalf("cannot encode png: %v", err)
		}
		_ = bw.Flush()
		_ = f.Close()

		// Console progress
		if frame%600 == 0 {
			log.Printf("Frame %d/%d  t=%.2f  x=%.3f  theta=%.3f  u=%.2f  tau=%.3f",
				frame, totalFrames, tf, s.X, s.Theta, uUsed, tauUsed)
		}
	}

	// Encode MP4
	mp4 := filepath.Join(outDir, "pendulum_smc_10s_6000f.mp4")
	encodeMP4WithFFmpeg(framesDir, fps, mp4)

	// Plots + CSV
	log.Printf("Saving plots and CSV...")
	if err := savePlots(outDir, tLog, xLog, thLog, uLog, FeqLog, tauLog, surfLog); err != nil {
		log.Fatalf("plot saving failed: %v", err)
	}

	if err := writeCSV(filepath.Join(outDir, "cartpole_log.csv"),
		[]string{"t", "x", "theta", "u", "F_equiv", "tau_ext", "s"},
		[][]float64{tLog, xLog, thLog, uLog, FeqLog, tauLog, surfLog},
	); err != nil {
		log.Fatalf("CSV saving failed: %v", err)
	}

	log.Printf("Done.")
}
