// ------------------------------------------------------------
// 2D Heat Equation (Heat Transfer by Conduction) in Go
// ------------------------------------------------------------
// PDE:
//   ∂T/∂t = α ( ∂²T/∂x² + ∂²T/∂y² )
//
// Method:
//   - 2D explicit finite-difference scheme (FTCS)
//   - Dirichlet boundary conditions (fixed temperature on boundaries)
//   - Heatmap snapshots saved as PNG (high-resolution, ~300 DPI)
//   - Additional plots saved (center point vs time, final centerline)
//
// Output folder (relative to where you run the program):
//   output/heat2d/
//
// Notes:
//   - Plots are generated with Gonum Plot (pure Go).
//   - PNG images are rendered using a 300 DPI canvas.
// ------------------------------------------------------------

package main

import (
	"bufio"
	"encoding/csv"
	"fmt"
	"log"
	"math"
	"os"
	"path/filepath"

	"gonum.org/v1/plot"
	"gonum.org/v1/plot/palette/moreland"
	"gonum.org/v1/plot/plotter"
	"gonum.org/v1/plot/vg"
	"gonum.org/v1/plot/vg/draw"
	"gonum.org/v1/plot/vg/vgimg"
)

// idx flattens 2D indexing (i, j) into a 1D array index.
// nx is the number of columns (x-direction).
func idx(i, j, nx int) int {
	return j*nx + i
}

// writeCSVTwoColumns saves two equal-length vectors to a CSV file with a header row.
func writeCSVTwoColumns(filename, h1, h2 string, c1, c2 []float64) error {
	if len(c1) != len(c2) {
		return fmt.Errorf("CSV write error: column sizes do not match")
	}

	if err := os.MkdirAll(filepath.Dir(filename), 0o755); err != nil {
		return fmt.Errorf("CSV write error: cannot create directory: %w", err)
	}

	f, err := os.Create(filename)
	if err != nil {
		return fmt.Errorf("CSV write error: cannot open file: %w", err)
	}
	defer f.Close()

	w := csv.NewWriter(f)
	defer w.Flush()

	if err := w.Write([]string{h1, h2}); err != nil {
		return fmt.Errorf("CSV write error: cannot write header: %w", err)
	}

	for k := range c1 {
		row := []string{
			fmt.Sprintf("%.15g", c1[k]),
			fmt.Sprintf("%.15g", c2[k]),
		}
		if err := w.Write(row); err != nil {
			return fmt.Errorf("CSV write error: cannot write row: %w", err)
		}
	}
	return nil
}

// limitedTicker returns a tick generator that:
// - produces at most maxLabels tick labels,
// - formats numeric labels with a given fmt (e.g., "%.1f").
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
			ticks = append(ticks, plot.Tick{
				Value: v,
				Label: fmt.Sprintf(labelFmt, v),
			})
		}
		return ticks
	})
}

// stylePlot applies consistent plot styling:
// - large fonts (labels + title),
// - thick axes and tick marks,
// - at most 10 tick labels per axis,
// - substantial padding so data and labels are clear.
func stylePlot(p *plot.Plot) {
	// Title and labels
	p.Title.TextStyle.Font.Size = vg.Points(22)
	p.Title.Padding = vg.Points(12)

	p.X.Label.TextStyle.Font.Size = vg.Points(18)
	p.Y.Label.TextStyle.Font.Size = vg.Points(18)
	p.X.Label.Padding = vg.Points(10)
	p.Y.Label.Padding = vg.Points(10)

	// Axis lines and padding
	p.X.LineStyle.Width = vg.Points(2.2)
	p.Y.LineStyle.Width = vg.Points(2.2)
	p.X.Padding = vg.Points(20)
	p.Y.Padding = vg.Points(20)

	// Tick styling and tick label formatting
	p.X.Tick.LineStyle.Width = vg.Points(2.0)
	p.Y.Tick.LineStyle.Width = vg.Points(2.0)
	p.X.Tick.Length = vg.Points(8)
	p.Y.Tick.Length = vg.Points(8)

	p.X.Tick.Label.Font.Size = vg.Points(14)
	p.Y.Tick.Label.Font.Size = vg.Points(14)

	p.X.Tick.Marker = limitedTicker(10, "%.2f")
	p.Y.Tick.Marker = limitedTicker(10, "%.1f")
}

// savePlotPNG renders a plot to a high-resolution PNG using a 300 DPI raster canvas.
// widthIn and heightIn are in inches (e.g., 8x6 inches).
func savePlotPNG(p *plot.Plot, widthIn, heightIn float64, filename string) error {
	if err := os.MkdirAll(filepath.Dir(filename), 0o755); err != nil {
		return fmt.Errorf("cannot create directory: %w", err)
	}

	w := vg.Length(widthIn) * vg.Inch
	h := vg.Length(heightIn) * vg.Inch

	// Create a 300 DPI raster canvas.
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

	png := vgimg.PngCanvas{Canvas: c}
	if _, err := png.WriteTo(bw); err != nil {
		return fmt.Errorf("cannot write png: %w", err)
	}
	return nil
}

// heatGrid wraps a flattened temperature field and exposes it as a GridXYZ for heatmap plotting.
type heatGrid struct {
	nx, ny int
	dx, dy float64
	data   []float64 // flattened in row-major (y-major) order
}

func (g heatGrid) Dims() (c, r int) { return g.nx, g.ny }
func (g heatGrid) Z(c, r int) float64 {
	return g.data[idx(c, r, g.nx)]
}
func (g heatGrid) X(c int) float64 { return float64(c) * g.dx }
func (g heatGrid) Y(r int) float64 { return float64(r) * g.dy }

func main() {
	// ------------------------------------------------------------
	// Output directory setup
	// ------------------------------------------------------------
	outDir := filepath.Join("output", "heat2d")
	if err := os.MkdirAll(outDir, 0o755); err != nil {
		log.Fatalf("cannot create output dir: %v", err)
	}

	// ------------------------------------------------------------
	// Physical parameters
	// ------------------------------------------------------------
	alpha := 1.0 // thermal diffusivity (chosen for demonstrative speed)

	// ------------------------------------------------------------
	// Domain and grid
	// ------------------------------------------------------------
	Lx, Ly := 1.0, 1.0

	// Grid resolution (increase for smoother but slower simulations)
	nx, ny := 81, 81

	dx := Lx / float64(nx-1)
	dy := Ly / float64(ny-1)

	// ------------------------------------------------------------
	// Time step selection (explicit stability)
	// ------------------------------------------------------------
	// For the 2D explicit heat equation (FTCS), a standard stability constraint is:
	//
	//   dt <= 1 / ( 2*α*( 1/dx^2 + 1/dy^2 ) )
	//
	// Use a conservative fraction to remain away from the limit.
	dtStable := 1.0 / (2.0 * alpha * (1.0/(dx*dx) + 1.0/(dy*dy)))
	dt := 0.80 * dtStable

	tEnd := 0.20
	nSteps := int(math.Ceil(tEnd / dt))

	// Heatmap snapshot scheduling
	desiredSnapshots := 30
	snapshotEvery := nSteps / desiredSnapshots
	if snapshotEvery < 1 {
		snapshotEvery = 1
	}

	// ------------------------------------------------------------
	// Allocate temperature fields
	// ------------------------------------------------------------
	T := make([]float64, nx*ny)
	TNew := make([]float64, nx*ny)

	// ------------------------------------------------------------
	// Boundary conditions (Dirichlet)
	// ------------------------------------------------------------
	TLeft := 100.0
	TRight := 0.0
	TTop := 0.0
	TBottom := 0.0

	// Apply boundary values to initial condition
	for j := 0; j < ny; j++ {
		T[idx(0, j, nx)] = TLeft
		T[idx(nx-1, j, nx)] = TRight
	}
	for i := 0; i < nx; i++ {
		T[idx(i, 0, nx)] = TBottom
		T[idx(i, ny-1, nx)] = TTop
	}

	// ------------------------------------------------------------
	// Logging (temperature at center over time)
	// ------------------------------------------------------------
	ic, jc := nx/2, ny/2

	timeLog := make([]float64, 0, nSteps+1)
	centerTLog := make([]float64, 0, nSteps+1)

	// ------------------------------------------------------------
	// Time integration loop
	// ------------------------------------------------------------
	t := 0.0
	for step := 0; step < nSteps; step++ {
		// Log center temperature
		timeLog = append(timeLog, t)
		centerTLog = append(centerTLog, T[idx(ic, jc, nx)])

		// Update interior nodes (boundaries remain fixed)
		for j := 1; j < ny-1; j++ {
			for i := 1; i < nx-1; i++ {
				// Second derivative in x (central difference)
				Txx := (T[idx(i+1, j, nx)] - 2.0*T[idx(i, j, nx)] + T[idx(i-1, j, nx)]) / (dx * dx)

				// Second derivative in y (central difference)
				Tyy := (T[idx(i, j+1, nx)] - 2.0*T[idx(i, j, nx)] + T[idx(i, j-1, nx)]) / (dy * dy)

				// Explicit FTCS update
				TNew[idx(i, j, nx)] = T[idx(i, j, nx)] + alpha*dt*(Txx+Tyy)
			}
		}

		// Re-apply boundaries to keep them fixed exactly
		for j := 0; j < ny; j++ {
			TNew[idx(0, j, nx)] = TLeft
			TNew[idx(nx-1, j, nx)] = TRight
		}
		for i := 0; i < nx; i++ {
			TNew[idx(i, 0, nx)] = TBottom
			TNew[idx(i, ny-1, nx)] = TTop
		}

		// Swap buffers
		T, TNew = TNew, T

		// Save a heatmap snapshot occasionally
		if step%snapshotEvery == 0 {
			g := heatGrid{nx: nx, ny: ny, dx: dx, dy: dy, data: T}

			p := plot.New()
			p.Title.Text = "2D Heat Equation - Temperature Field"
			p.X.Label.Text = "x (m)"
			p.Y.Label.Text = "y (m)"
			stylePlot(p)

			pal := moreland.Kindlmann().Palette(255)
			hm := plotter.NewHeatMap(g, pal)

			p.Add(hm)

			pngName := filepath.Join(outDir, fmt.Sprintf("heat_t%06d.png", step))
			if err := savePlotPNG(p, 8.0, 6.5, pngName); err != nil {
				log.Fatalf("cannot save heatmap: %v", err)
			}
			log.Printf("Saved snapshot: %s", pngName)
		}

		// Advance time
		t += dt
	}

	// ------------------------------------------------------------
	// Final plots: centerline temperature and center point vs time
	// ------------------------------------------------------------
	x := make([]float64, nx)
	centerline := make([]float64, nx)
	for i := 0; i < nx; i++ {
		x[i] = float64(i) * dx
		centerline[i] = T[idx(i, jc, nx)]
	}

	// Final centerline temperature
	{
		p := plot.New()
		p.Title.Text = "Final Centerline Temperature (y = 0.5)"
		p.X.Label.Text = "x (m)"
		p.Y.Label.Text = "T(x, y=0.5)"
		stylePlot(p)

		pts := make(plotter.XYs, nx)
		for i := 0; i < nx; i++ {
			pts[i].X = x[i]
			pts[i].Y = centerline[i]
		}

		line, err := plotter.NewLine(pts)
		if err != nil {
			log.Fatalf("cannot create line plot: %v", err)
		}
		line.LineStyle.Width = vg.Points(3.0)
		p.Add(line)

		if err := savePlotPNG(p, 8.0, 6.0, filepath.Join(outDir, "centerline_final.png")); err != nil {
			log.Fatalf("cannot save plot: %v", err)
		}
	}

	// Center point temperature vs time
	{
		p := plot.New()
		p.Title.Text = "Temperature at Plate Center vs Time"
		p.X.Label.Text = "time (s)"
		p.Y.Label.Text = "T(center)"
		stylePlot(p)

		pts := make(plotter.XYs, len(timeLog))
		for i := range timeLog {
			pts[i].X = timeLog[i]
			pts[i].Y = centerTLog[i]
		}

		line, err := plotter.NewLine(pts)
		if err != nil {
			log.Fatalf("cannot create line plot: %v", err)
		}
		line.LineStyle.Width = vg.Points(3.0)
		p.Add(line)

		if err := savePlotPNG(p, 8.0, 6.0, filepath.Join(outDir, "center_point_vs_time.png")); err != nil {
			log.Fatalf("cannot save plot: %v", err)
		}
	}

	// ------------------------------------------------------------
	// Save CSV log of center temperature
	// ------------------------------------------------------------
	if err := writeCSVTwoColumns(filepath.Join(outDir, "heat2d_log.csv"), "t", "T_center", timeLog, centerTLog); err != nil {
		log.Printf("warning: %v", err)
	}

	log.Printf("Heat2D finished. Results are in: %s", outDir)
}
