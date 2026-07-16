<div style="font-family: system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; line-height: 1.55;">

  <h1 align="center" style="margin-bottom: 0.2em;">ODE &amp; PDE Solvers in C# (Heat Equation + Inverted Pendulum SMC)</h1>

  <p style="font-size: 0.98rem; max-width: 980px; margin: 0.2rem auto 0;">
    A .NET repository focused on <strong>numerical simulation</strong> and <strong>visualization</strong> of:
    <strong>partial differential equations (PDEs)</strong> and <strong>ordinary differential equations (ODEs)</strong>.
    The project includes two complete, end-to-end examples implemented in modern <strong>C#</strong>:
  </p>

  <ul style="max-width: 980px; margin: 0.55rem auto 0.2rem; padding-left: 1.2rem;">
    <li>
      <strong>PDE:</strong> 2D heat-conduction equation solved using an explicit finite-difference method (FTCS),
      generating <strong>heatmap snapshots</strong>, summary plots, and a CSV log.
    </li>
    <li>
      <strong>ODE:</strong> a nonlinear cart–pole (inverted pendulum) stabilized using
      <strong>Sliding Mode Control (SMC)</strong>, generating <strong>high-FPS frames</strong>, plots, a CSV log,
      and optional <strong>MP4 encoding</strong> using FFmpeg.
    </li>
  </ul>

  <p align="center" style="font-size: 1rem; color: #666; margin: 0.35rem auto 0;">
    Tooling: <strong>.NET 8</strong> • IDE: <strong>Visual Studio 2022</strong> • Plots: <strong>ScottPlot 5</strong> •
    Rendering (frames): <strong>System.Drawing / WinForms</strong> • Optional video encoding: <strong>FFmpeg</strong>
  </p>

</div>

<hr />

<!-- ========================================================= -->
<!-- Table of Contents                                         -->
<!-- ========================================================= -->

<ul style="list-style: none; padding-left: 0; font-size: 0.97rem;">
  <li> <a href="#about-this-repository">About this repository</a></li>
  <li> <a href="#what-are-odes-and-pdes">What are ODEs and PDEs?</a></li>
  <li> <a href="#example-1-pde-2d-heat-equation">Example 1 (PDE): 2D Heat Equation</a></li>
  <li> <a href="#example-2-ode-inverted-pendulum-with-smc">Example 2 (ODE): Inverted Pendulum with Sliding Mode Control</a></li>
  <li> <a href="#sliding-mode-control-theory">Sliding Mode Control (SMC) theory</a></li>
  <li> <a href="#numerical-methods-used-in-this-repo">Numerical methods used in this repo</a></li>
  <li> <a href="#plots-and-animations-scottplot-systemdrawing-ffmpeg">Plots and animations (ScottPlot, System.Drawing, FFmpeg)</a></li>
  <li> <a href="#dependencies-and-installation">Dependencies and installation</a></li>
  <li> <a href="#building-the-solution">Building the solution</a></li>
  <li> <a href="#running-and-generating-results">Running and generating results</a></li>
  <li> <a href="#repository-file-guide">Repository file guide (full explanation)</a></li>
  <li> <a href="#operating-system-guides-windows-macos-linux">Operating system guides (Windows / macOS / Linux)</a></li>
  <li> <a href="#troubleshooting">Troubleshooting</a></li>
  <li> <a href="#implementation-tutorial-video">Implementation tutorial video</a></li>
</ul>

---

<a id="about-this-repository"></a>

## About this repository

This repository is designed as a practical reference for implementing numerical solvers in modern C# (.NET 8):

- A <strong>PDE workflow</strong> that starts from a physical diffusion model (heat conduction), discretizes it on a grid,
  runs an explicit time-marching solver, and produces plots and heatmap snapshots suitable for reports.
- An <strong>ODE + control workflow</strong> that implements a nonlinear plant (cart–pole), injects disturbances, applies a robust controller
  (Sliding Mode Control), and produces both engineering plots and a high-FPS animation.

Engineering practices emphasized throughout the code:

- explicit stability constraints for diffusion PDEs,
- deterministic integration and reproducible outputs,
- clean CSV logging for post-processing,
- high-quality PNG export (including 300 DPI metadata),
- optional MP4 encoding using an external encoder (FFmpeg).

### Quick map of the two executables

- <code>Heat2D</code> → solves the 2D heat equation and writes results into <code>output/heat2d/</code>
- <code>PendulumSlidingMode</code> → simulates a cart–pole and writes results into <code>output/pendulum_sliding_mode/</code>

---

<a id="what-are-odes-and-pdes"></a>

## What are ODEs and PDEs?

### Ordinary Differential Equations (ODEs)

An ODE describes the evolution of one or more state variables with respect to a single independent variable (usually time):

$$
\dot{\mathbf{x}}(t) = \mathbf{f}(\mathbf{x}(t), \mathbf{u}(t), t),
$$

where:

- $\mathbf{x}(t)$ is the system state (e.g., cart position and pole angle),
- $\mathbf{u}(t)$ is an input (e.g., a control force),
- $\mathbf{f}(\cdot)$ defines the system dynamics.

In this repository, the inverted pendulum example is a nonlinear ODE system integrated using an RK4 scheme with sub-stepping.

### Partial Differential Equations (PDEs)

A PDE involves partial derivatives with respect to two or more independent variables (e.g., time and space).
A canonical example is the heat (diffusion) equation:

$$
\frac{\partial T}{\partial t} = \alpha \left( \frac{\partial^2 T}{\partial x^2} + \frac{\partial^2 T}{\partial y^2} \right),
$$

where:

- $T(x,y,t)$ is temperature,
- $\alpha$ is thermal diffusivity,
- $(x,y)$ are spatial coordinates,
- $t$ is time.

In this repository, the heat equation is discretized in space using finite differences and marched forward in time using an explicit FTCS update.

---

<a id="example-1-pde-2d-heat-equation"></a>

## Example 1 (PDE): 2D Heat Equation

### Physical model

The PDE solved in <code>ODE_PDE_Csharp/Heat2D/Program.cs</code> is:

$$
\frac{\partial T}{\partial t} = \alpha \left( \frac{\partial^2 T}{\partial x^2} + \frac{\partial^2 T}{\partial y^2} \right).
$$

Interpretation:

- conduction in a 2D plate,
- no internal heat sources,
- constant isotropic diffusivity $\alpha$,
- fixed-temperature boundaries (Dirichlet boundary conditions).

The implemented boundary conditions are:

- left boundary held at $T=100$,
- right/top/bottom held at $T=0$,

which produces a diffusion front moving from the left edge into the interior.

### Discretization summary (what the code computes)

For interior grid nodes $(i,j)$, the FTCS update is:

$$
T^{n+1}_{i,j} = T^{n}_{i,j} + \alpha\,\Delta t\left(
\frac{T^n_{i+1,j}-2T^n_{i,j}+T^n_{i-1,j}}{\Delta x^2}
+
\frac{T^n_{i,j+1}-2T^n_{i,j}+T^n_{i,j-1}}{\Delta y^2}
\right).
$$

Implementation details:

- the 2D grid is stored as a flattened 1D array for cache efficiency,
- <code>Idx(i, j, nx)</code> converts 2D indices to a 1D index,
- boundary nodes are overwritten every step to enforce Dirichlet conditions exactly,
- snapshots are throttled (not saved every step) to avoid generating thousands of images.

### Stability constraint (explicit diffusion)

A common sufficient stability constraint for the 2D explicit diffusion scheme is:

$$
\Delta t \le \frac{1}{2\alpha \left(\frac{1}{\Delta x^2} + \frac{1}{\Delta y^2}\right)}.
$$

The code computes this limit and uses a conservative factor (<code>0.80</code>) to avoid running near the edge of stability.

### Outputs produced

Running <code>Heat2D</code> creates:

- heatmap snapshots: <code>output/heat2d/heat_t*.png</code>
- final centerline profile: <code>output/heat2d/centerline_final.png</code>
- center temperature vs time: <code>output/heat2d/center_point_vs_time.png</code>
- CSV log: <code>output/heat2d/heat2d_log.csv</code>

---

<a id="example-2-ode-inverted-pendulum-with-smc"></a>

## Example 2 (ODE): Inverted Pendulum with Sliding Mode Control

The second example (<code>ODE_PDE_Csharp/PendulumSlidingMode/Program.cs</code>) simulates a nonlinear cart–pole system with disturbances and stabilizes it using Sliding Mode Control (SMC).

### State variables and conventions

The state is:

$$
\mathbf{x} = \begin{bmatrix} x & \dot{x} & \theta & \dot{\theta} \end{bmatrix}^T
$$

- $x$ is the cart position (m),
- $\theta$ is the pole angle (rad) where $\theta=0$ is upright,
- the rendering convention is: $\theta &gt; 0$ visually leans the pole to the right.

Two practical helper operations are used:

- angle wrapping into $[-\pi,\pi]$ to avoid numeric drift,
- clamping/saturation for actuator limits and SMC boundary-layer logic.

### Disturbances (exactly two)

Two external torque pulses are injected about the pole pivot:

- pulse 1: starts at $t=0.5\,s$ (positive torque),
- pulse 2: starts at $t=5.0\,s$ (negative torque),

each using a smooth half-sine profile over a duration of $0.5\,s$.
The disturbance arrow is shown for <strong>0.5 seconds only</strong> after each pulse starts and is drawn with <strong>constant length</strong>.

Disturbance model:

$$
\tau_{ext}(t) =
\begin{cases}
+\tau_{amp}\sin\left(\pi\frac{t-t_1}{d}\right) &amp; t\in[t_1,t_1+d] \\
-\tau_{amp}\sin\left(\pi\frac{t-t_2}{d}\right) &amp; t\in[t_2,t_2+d] \\
0 &amp; \text{otherwise}
\end{cases}
$$

and the “equivalent bob force” used for arrow direction is:

$$
F_{eq}(t) = \frac{\tau_{ext}(t)}{L}.
$$

### Simulation and rendering pipeline

The program is intentionally end-to-end:

- integrate the nonlinear dynamics using RK4,
- render each frame using the current integrated state (System.Drawing),
- save frames to disk,
- optionally encode a video using FFmpeg,
- generate plots using ScottPlot,
- write a CSV log for offline analysis.

Outputs include:

- frames: <code>output/pendulum_sliding_mode/frames/frame_000000.png</code> … <code>frame_005999.png</code>
- plots: <code>output/pendulum_sliding_mode/*.png</code>
- CSV: <code>output/pendulum_sliding_mode/cartpole_log.csv</code>
- MP4: <code>output/pendulum_sliding_mode/pendulum_smc_10s_6000f.mp4</code> (if FFmpeg is installed and reachable via PATH)

---

<a id="sliding-mode-control-theory"></a>

## Sliding Mode Control (SMC) theory

Sliding Mode Control is a robust nonlinear control method designed to drive the system state to a chosen manifold (the “sliding surface”) and maintain it there even under disturbances and certain modeling uncertainties.

### 1) Sliding surface design

For a general second-order system, a classical sliding surface is:

$$
s = \dot{e} + \lambda e,
$$

where $e$ is a tracking error and $\lambda &gt; 0$. Enforcing $s \to 0$ yields stable error dynamics.

In this repository, the surface couples pendulum stabilization with cart motion:

$$
s(\mathbf{x}) = \dot{\theta} + \lambda_{\theta}\,\theta + \alpha\left(\dot{x} + \lambda_x\,x\right).
$$

Interpretation:

- $\lambda_{\theta}$ sets how aggressively the controller drives the pole upright,
- $\alpha$ couples cart motion (needed to “catch” the pole) into the surface,
- $\lambda_x$ introduces a cart-centering component.

### 2) Reaching condition (Lyapunov intuition)

A typical sufficient condition to guarantee reaching and maintaining the sliding manifold is:

$$
\frac{d}{dt}\left(\frac{1}{2}s^2\right) = s\dot{s} \le -\eta |s|,
$$

where $\eta &gt; 0$. This ensures $|s|$ decreases and reaches zero in finite time.

A common target dynamic is:

$$
\dot{s} = -k\,\mathrm{sign}(s), \quad k &gt; 0.
$$

### 3) Boundary layer to reduce chattering

The discontinuous sign function can cause chattering. A standard mitigation is to replace it with a continuous saturation function:

$$
\mathrm{sat}(z) =
\begin{cases}
-1 &amp; z &lt; -1 \\
z  &amp; |z| \le 1 \\
+1 &amp; z &gt; 1
\end{cases}
$$

and use:

$$
\dot{s} = -k\,\mathrm{sat}\left(\frac{s}{\phi}\right),
$$

where $\phi &gt; 0$ defines the boundary-layer thickness. This repository implements this directly using:

- <code>k</code> (gain)
- <code>phi</code> (boundary layer width)

### 4) Control computation used in the code

Because the plant is nonlinear, the program does not rely on a closed-form affine relationship between the cart force $u$ and $\dot{s}$.
Instead, it uses a practical numerical linearization around the current state:

1. Define the desired sliding surface derivative:  
   $\dot{s}_{des} = -k\,\mathrm{sat}\left(\frac{s}{\phi}\right)$.
2. Approximate $\dot{s}(u)$ locally as $\dot{s}(u) \approx a u + b$ by evaluating $\dot{s}$ at two nearby inputs.
3. Solve:
   $u_{smc} = \frac{\dot{s}_{des} - b}{a}$.
4. Add a gated cart-centering term and clamp to actuator limits.

### 5) Gated cart-centering term

To prevent long-term drift, a cart-centering PD term is used, but it is “gated” so it does not fight the stabilization maneuver:

$$
u_{hold} = g(\theta)\left(-k_p x - k_d \dot{x}\right),
$$

where:

$$
g(\theta) = \mathrm{clamp}\left(1 - \frac{|\theta|}{\theta_{gate}}, 0, 1\right).
$$

Result: the controller prioritizes catching the pole, then recenters the cart when the system is close to upright.

---

<a id="numerical-methods-used-in-this-repo"></a>

## Numerical methods used in this repo

### PDE solver: explicit finite differences (FTCS)

The heat solver uses:

- second-order central differences for spatial second derivatives,
- forward Euler in time.

Strengths:

- straightforward and fast per-step,
- easy to extend and optimize.

Limitation:

- explicit stability constraints can force small time steps as the grid is refined.

A common next step is an implicit method (Crank–Nicolson, ADI) to relax stability constraints.

### ODE solver: RK4 with sub-stepping

The cart–pole uses classical RK4.
Because the animation is rendered at high FPS, the code uses multiple physics substeps per frame:

- $\Delta t_{frame} = T_{video}/N_{frames}$
- $\Delta t_{physics} = \Delta t_{frame}/N_{substeps}$

This yields stable dynamics integration while maintaining deterministic frame output.

---

<a id="plots-and-animations-scottplot-systemdrawing-ffmpeg"></a>

## Plots and animations (ScottPlot, System.Drawing, FFmpeg)

### ScottPlot (plots and heatmaps)

This repository uses <a href="https://scottplot.net/" target="_blank">ScottPlot</a> (v5) for:

- line plots (time series),
- heatmaps (temperature field snapshots),
- saving PNG images in high resolution.

The programs export large pixel dimensions and embed 300 DPI metadata in the PNG files for report-quality figures.

### System.Drawing / WinForms (animation frames)

The pendulum example renders frames using:

- <code>System.Drawing.Bitmap</code> as the offscreen framebuffer,
- <code>Graphics</code> drawing primitives (lines, ellipses),
- optional <code>--preview</code> using WinForms (<code>Form</code> + <code>PictureBox</code>).

### FFmpeg (optional) for MP4 encoding

If FFmpeg is installed and available on PATH, the pendulum program attempts to encode an MP4 automatically.
If not, you can always encode manually:

```bash
ffmpeg -y -framerate 600 -i output/pendulum_sliding_mode/frames/frame_%06d.png \
  -c:v libx264 -pix_fmt yuv420p -crf 18 -preset veryfast \
  output/pendulum_sliding_mode/pendulum_smc_10s_6000f.mp4
```

---

<a id="dependencies-and-installation"></a>

## Dependencies and installation

### Required (Windows-first configuration)

- Visual Studio 2022 (recommended)
- .NET SDK 8
- NuGet packages (restored automatically by Visual Studio / dotnet):
  - <code>ScottPlot</code> (plots + heatmaps)
  - <code>System.Drawing.Common</code> (frame rendering and PNG metadata on Windows)
- Optional: FFmpeg for MP4 encoding

### Why the repository targets <code>net8.0-windows</code>

The pendulum renderer uses WinForms types and System.Drawing APIs intended for Windows.
Therefore the current projects target:

- <code>TargetFramework</code>: <code>net8.0-windows</code>
- <code>&lt;UseWindowsForms&gt;true&lt;/UseWindowsForms&gt;</code>

A cross-platform port is feasible (see OS guides), but the current configuration prioritizes Visual Studio 2022 on Windows.

---

<a id="building-the-solution"></a>

## Building the solution

### Visual Studio (recommended)

- Open <code>ODE_PDE_Csharp.sln</code>
- Set configuration to <strong>Release</strong>
- Build: <strong>Build → Build Solution</strong>

### .NET CLI

From the repository root:

```bash
dotnet restore
dotnet build -c Release
```

---

<a id="running-and-generating-results"></a>

## Running and generating results

### Run Heat2D

```bash
dotnet run --project ODE_PDE_Csharp/Heat2D -c Release
```

Outputs:

- <code>output/heat2d/</code>

### Run PendulumSlidingMode

```bash
dotnet run --project ODE_PDE_Csharp/PendulumSlidingMode -c Release
```

Optional preview window:

```bash
dotnet run --project ODE_PDE_Csharp/PendulumSlidingMode -c Release -- --preview
```

Outputs:

- <code>output/pendulum_sliding_mode/</code>

---

<a id="repository-file-guide"></a>

## Repository file guide (full explanation)

This section explains each important file in the repository and its role.

### <code>ODE_PDE_Csharp/Heat2D/Program.cs</code>

Responsibilities:

- defines the physical parameters and grid (<code>nx</code>, <code>ny</code>, <code>dx</code>, <code>dy</code>),
- computes a stable explicit time step and chooses a conservative <code>dt</code>,
- performs the FTCS update for interior nodes,
- enforces fixed-temperature boundaries at every step,
- logs center temperature over time,
- saves:
  - periodic heatmap snapshots (<code>heat_t*.png</code>),
  - a final centerline plot,
  - a center temperature vs time plot,
  - a CSV log.

Export quality notes:

- plots are saved at high pixel resolution,
- PNGs are rewritten to embed 300 DPI metadata without resampling (avoids cropping).

### <code>ODE_PDE_Csharp/Heat2D/Heat2D.csproj</code>

Defines project settings and dependencies:

- <code>TargetFramework</code>: <code>net8.0-windows</code>
- <code>UseWindowsForms</code>: enabled (safe for System.Drawing usage on Windows)
- NuGet references:
  - <code>ScottPlot</code>
  - <code>System.Drawing.Common</code>

### <code>ODE_PDE_Csharp/PendulumSlidingMode/Program.cs</code>

This file contains the full ODE + control + rendering pipeline:

1) <strong>Model and parameters</strong>
- cart mass, pole geometry, rod+bob inertia modeling, damping terms

2) <strong>Disturbance injection</strong>
- two half-sine torque pulses at fixed times (0.5s and 5.0s)
- constant-length arrow shown for 0.5s after each disturbance start

3) <strong>Controller</strong>
- sliding surface definition
- boundary-layer saturation
- gated cart-centering term
- actuator saturation

4) <strong>Integration</strong>
- RK4 dynamics integration
- multiple substeps per frame

5) <strong>Rendering</strong>
- offscreen <code>Bitmap</code> rendering with high-quality settings
- PNG export for each frame
- optional preview with WinForms

6) <strong>Outputs</strong>
- plots (ScottPlot)
- CSV log
- optional MP4 encoding via FFmpeg

### <code>ODE_PDE_Csharp/PendulumSlidingMode/PendulumSlidingMode.csproj</code>

Defines:

- <code>TargetFramework</code>: <code>net8.0-windows</code>
- <code>UseWindowsForms</code>: enabled for preview window
- NuGet references:
  - <code>ScottPlot</code>
  - <code>System.Drawing.Common</code>

---

<a id="operating-system-guides-windows-macos-linux"></a>

## Operating system guides (Windows / macOS / Linux)

### Windows (Visual Studio 2022) — full setup from a blank solution

This matches the “one solution, two projects” structure.

#### 1) Install prerequisites

- Visual Studio 2022
  - Workload: <strong>.NET desktop development</strong>
- .NET SDK 8 (if not already included)
- Optional: FFmpeg

#### 2) Create a blank solution

1. <strong>File → New → Project</strong>
2. Select: <strong>Blank Solution</strong>
3. Name: <strong>ODE_PDE_Csharp</strong>
4. Location: choose your repo folder
5. Create

#### 3) Add the first project: Heat2D

1. Right-click the solution → <strong>Add → New Project</strong>
2. Choose: <strong>Console App</strong> (C#)
3. Project name: <strong>Heat2D</strong>
4. Target framework: <strong>.NET 8</strong>
5. Create
6. Replace <code>Program.cs</code> with <code>ODE_PDE_Csharp/Heat2D/Program.cs</code> from this repo.
7. Add NuGet packages:
   - Right-click <strong>Heat2D</strong> → <strong>Manage NuGet Packages…</strong>
   - Install:
     - <code>ScottPlot</code>
     - <code>System.Drawing.Common</code>

#### 4) Add the second project: PendulumSlidingMode

Repeat the same steps:

- Add a new Console App named <strong>PendulumSlidingMode</strong>
- Replace <code>Program.cs</code> with <code>ODE_PDE_Csharp/PendulumSlidingMode/Program.cs</code>
- Install the same NuGet packages for this project:
  - <code>ScottPlot</code>
  - <code>System.Drawing.Common</code>

#### 5) Ensure the output folder appears where you expect

Both programs write to a relative folder:

- <code>output/...</code>

That path is resolved relative to the process <strong>working directory</strong>.
For Visual Studio, set the working directory for each project to the solution folder:

1. Right-click project → <strong>Properties</strong>
2. Go to: <strong>Debug</strong>
3. Set <strong>Working directory</strong> to:

- <code>$(SolutionDir)</code>

Now <code>output/</code> will be created next to the solution file.

#### 6) Run

- Right-click project → <strong>Set as Startup Project</strong>
- Press <strong>F5</strong> or <strong>Ctrl+F5</strong>
- Outputs appear under <code>output/...</code>

---

### macOS / Linux

#### Important note about the current repository configuration

The current projects target <code>net8.0-windows</code> and use WinForms/System.Drawing APIs.
This is a Windows-first configuration, and it is the most reliable way to reproduce the same rendering pipeline.

You have two practical options:

#### Option A: Run on Windows

- Use a Windows machine, a Windows VM, or a Windows CI runner.
- Run with Visual Studio or the <code>dotnet</code> CLI.

#### Option B: Port to cross-platform rendering (high-level plan)

To run natively on macOS/Linux, the typical engineering approach is:

- change <code>TargetFramework</code> to <code>net8.0</code>,
- remove <code>&lt;UseWindowsForms&gt;true&lt;/UseWindowsForms&gt;</code>,
- remove <code>System.Drawing.Common</code>,
- render frames using a cross-platform library (for example):
  - <a href="https://github.com/mono/SkiaSharp" target="_blank">SkiaSharp</a>, or
  - <a href="https://github.com/SixLabors/ImageSharp" target="_blank">SixLabors.ImageSharp</a>,
- keep ScottPlot for plots (ScottPlot v5 is cross-platform).

Once ported, you can build and run with:

```bash
dotnet build -c Release
dotnet run --project ODE_PDE_Csharp/Heat2D -c Release
dotnet run --project ODE_PDE_Csharp/PendulumSlidingMode -c Release
```

FFmpeg installation (macOS via Homebrew):

```bash
brew install ffmpeg
```

FFmpeg installation (Ubuntu/Debian):

```bash
sudo apt-get update
sudo apt-get install -y ffmpeg
```

---

<a id="troubleshooting"></a>

## Troubleshooting

### I cannot find the <code>output/</code> folder

This is almost always a working-directory issue.

- Visual Studio runs your program with a working directory that may be:
  - the project folder, or
  - <code>bin/Debug/net8.0-windows/</code>, depending on settings.

Fix (recommended):

- Project Properties → Debug → <strong>Working directory</strong> = <code>$(SolutionDir)</code>

Then run again.

### My MP4 was not created

- Confirm FFmpeg is installed and accessible:

```bash
ffmpeg -version
```

- If it is installed but the program still does not produce an MP4, encode manually:

```bash
ffmpeg -y -framerate 600 -i output/pendulum_sliding_mode/frames/frame_%06d.png \
  -c:v libx264 -pix_fmt yuv420p -crf 18 -preset veryfast \
  output/pendulum_sliding_mode/pendulum_smc_10s_6000f.mp4
```

### Frame generation is slow

Saving 6000 PNG images is disk-I/O heavy.

- Use an SSD (recommended).
- Run in <strong>Release</strong>.
- Close the preview window when not needed.
- If you are experimenting, reduce <code>totalFrames</code> temporarily.

---

<a id="implementation-tutorial-video"></a>

## Implementation tutorial video

At the end of the workflow, you can watch the full implementation and walkthrough on YouTube.

<!-- Replace the link below with your final uploaded video URL -->
<a href="https://www.youtube.com/watch?v=N5TD0oICQDw" target="_blank">
  <img
    src="https://i.ytimg.com/vi/N5TD0oICQDw/maxresdefault.jpg"
    alt="ODE/PDE in C# - Implementation Tutorial"
    style="max-width: 100%; border-radius: 10px; box-shadow: 0 6px 18px rgba(0,0,0,0.18); margin-top: 0.5rem;"
  />
</a>
