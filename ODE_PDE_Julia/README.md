<div style="font-family: system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; line-height: 1.6;">

  <h1 align="center" style="margin-bottom: 0.2em;">ODE &amp; PDE Solvers in Julia (2D Heat Equation + Inverted Pendulum SMC)</h1>

  <p style="font-size: 0.98rem; max-width: 920px; margin: 0.2rem auto 0;">
    A Julia repository focused on <strong>numerical simulation</strong> and <strong>visualization</strong> of:
    <strong>partial differential equations (PDEs)</strong> and <strong>ordinary differential equations (ODEs)</strong>.
    The project includes two complete, end-to-end examples:
  </p>

  <ul style="max-width: 920px; margin: 0.5rem auto 0.2rem; padding-left: 1.2rem;">
    <li>
      <strong>PDE:</strong> 2D heat-conduction equation solved with an explicit finite-difference method (FTCS),
      generating <strong>heatmap snapshots</strong>, summary plots, and CSV logs using <strong>Plots.jl</strong>.
    </li>
    <li>
      <strong>ODE:</strong> a nonlinear cart–pole (inverted pendulum) system stabilized using
      <strong>Sliding Mode Control (SMC)</strong>, with high-FPS frame rendering using <strong>Luxor.jl</strong>,
      plots via <strong>Plots.jl</strong>, optional MP4 encoding using <strong>FFmpeg</strong>, and CSV logging.
    </li>
  </ul>

  <p align="center" style="font-size: 1rem; color: #666; margin: 0.35rem auto 0;">
    Language: <strong>Julia</strong> • Plotting: <strong>Plots.jl</strong> (GR backend) • Frames: <strong>Luxor.jl</strong> • Optional video: <strong>FFmpeg</strong>
  </p>

</div>

<hr />

<!-- ========================================================= -->
<!-- Table of Contents                                        -->
<!-- ========================================================= -->

<ul style="list-style: none; padding-left: 0; font-size: 0.97rem;">
  <li> <a href="#about-this-repository">About this repository</a></li>
  <li> <a href="#what-are-odes-and-pdes">What are ODEs and PDEs?</a></li>
  <li> <a href="#example-1-pde-2d-heat-equation">Example 1 (PDE): 2D Heat Equation</a></li>
  <li> <a href="#example-2-ode-inverted-pendulum-with-smc">Example 2 (ODE): Inverted Pendulum with Sliding Mode Control</a></li>
  <li> <a href="#sliding-mode-control-theory">Sliding Mode Control (SMC) theory</a></li>
  <li> <a href="#numerical-methods-used-in-this-repo">Numerical methods used in this repo</a></li>
  <li> <a href="#solving-odes-and-pdes-in-julia">Solving ODEs and PDEs in Julia (workflow + plotting + animation)</a></li>
  <li> <a href="#dependencies-and-installation">Dependencies and installation</a></li>
  <li> <a href="#running-the-project-generating-results">Running the project and generating results</a></li>
  <li> <a href="#repository-file-guide">Repository file guide (full explanation)</a></li>
  <li> <a href="#operating-system-guides-windows-macos-linux">Operating system guides (Windows / macOS / Linux)</a></li>
  <li> <a href="#troubleshooting">Troubleshooting</a></li>
  <li> <a href="#implementation-tutorial-video">Implementation tutorial video</a></li>
</ul>

---

<a id="about-this-repository"></a>

## About this repository

This repository is designed as a practical reference for implementing numerical solvers and visualization pipelines in modern Julia:

- A <strong>PDE workflow</strong> that starts from a physical model (2D heat conduction), discretizes it on a grid, runs an explicit time-marching solver, and produces professional plots, heatmaps, and CSV logs.
- An <strong>ODE + control workflow</strong> that implements a nonlinear plant (cart–pole), injects disturbances, runs a robust controller (Sliding Mode Control), generates simulation plots, produces a high-FPS frame sequence, and (optionally) encodes a video.

The focus is not only “making it run,” but also showing engineering patterns that scale to larger projects:

- stability-aware time stepping (explicit diffusion constraints),
- clean logging to CSV for reproducibility,
- high-quality plots for analysis (large fonts, thick lines, limited tick count),
- frame-based animation for communication and debugging,
- project reproducibility via a pinned Julia environment (<code>Project.toml</code>).

### Quick map of the two examples

- <code>src/heat2d/main.jl</code> → solves the 2D heat equation and writes results into <code>output/heat2d/</code>
- <code>src/pendulum_sliding_mode/main.jl</code> → simulates a cart–pole with SMC and writes results into <code>output/pendulum_sliding_mode/</code>

---

<a id="what-are-odes-and-pdes"></a>

## What are ODEs and PDEs?

### Ordinary Differential Equations (ODEs)

An ODE describes the evolution of one or more state variables with respect to a single independent variable (usually time):

$$
\dot{\mathbf{x}}(t) = \mathbf{f}(\mathbf{x}(t), \mathbf{u}(t), t),
$$

where:

- $\mathbf{x}(t)$ is the state (e.g., cart position, pole angle),
- $\mathbf{u}(t)$ is an input (e.g., a control force),
- $\mathbf{f}(\cdot)$ describes the system dynamics.

In this repository, the inverted pendulum example is a nonlinear ODE system integrated in time using an RK4 scheme with sub-stepping for stable, high-FPS rendering.

### Partial Differential Equations (PDEs)

A PDE involves partial derivatives with respect to two or more independent variables (e.g., time and space). A canonical example is the heat equation (diffusion equation):

$$
\frac{\partial T}{\partial t} = \alpha \left( \frac{\partial^2 T}{\partial x^2} + \frac{\partial^2 T}{\partial y^2} \right),
$$

where:

- $T(x,y,t)$ is temperature,
- $\alpha$ is thermal diffusivity,
- $(x,y)$ are spatial coordinates,
- $t$ is time.

In this repository, the heat equation is discretized in space using finite differences and integrated forward in time using an explicit FTCS method.

---

<a id="example-1-pde-2d-heat-equation"></a>

## Example 1 (PDE): 2D Heat Equation

### Physical model

The PDE solved in <code>src/heat2d/main.jl</code> is:

$$
\frac{\partial T}{\partial t} = \alpha \left( \frac{\partial^2 T}{\partial x^2} + \frac{\partial^2 T}{\partial y^2} \right).
$$

Interpretation:

- conduction in a 2D plate,
- no internal heat sources,
- constant isotropic diffusivity $\alpha$,
- fixed temperature boundaries (Dirichlet BCs).

The implemented boundary conditions are:

- left boundary held at $T=100$,
- right/top/bottom held at $T=0$,

which produces a diffusion front moving from the left edge into the interior.

### Discretization summary (what the code does)

The solver uses:

- uniform grid in $x$ and $y$,
- second-order central differences for $\partial^2 T / \partial x^2$ and $\partial^2 T / \partial y^2$,
- explicit time stepping (Forward Euler in time).

At interior grid nodes $(i,j)$:

$$
T^{n+1}_{i,j} = T^{n}_{i,j} + \alpha\,\Delta t\left(
\frac{T^n_{i+1,j}-2T^n_{i,j}+T^n_{i-1,j}}{\Delta x^2}
+
\frac{T^n_{i,j+1}-2T^n_{i,j}+T^n_{i,j-1}}{\Delta y^2}
\right).
$$

Implementation note: the code stores $T$ as a 2D matrix <code>T[j,i]</code> aligned with coordinates <code>(x[i], y[j])</code>.

### Stability constraint (explicit diffusion)

The explicit 2D diffusion scheme requires a stability constraint on $\Delta t$. A commonly used sufficient condition is:

$$
\Delta t \le \frac{1}{2\alpha \left(\frac{1}{\Delta x^2} + \frac{1}{\Delta y^2}\right)}.
$$

The code computes this value (<code>dt_stable</code>) and chooses:

- <code>dt = 0.80 * dt_stable</code>

to remain safely within the stability margin.

### What you get as outputs

When you run <code>src/heat2d/main.jl</code>, you will find:

- multiple heatmap snapshots: <code>output/heat2d/heat_t*.png</code>
- final centerline profile: <code>output/heat2d/centerline_final.png</code>
- center temperature vs time: <code>output/heat2d/center_point_vs_time.png</code>
- CSV log: <code>output/heat2d/heat2d_log.csv</code>

This combination is typical for PDE workflows:

- images for qualitative verification,
- plots for engineering interpretation,
- CSV for reproducibility and post-processing in Python/Matlab/Excel.

---

<a id="example-2-ode-inverted-pendulum-with-smc"></a>

## Example 2 (ODE): Inverted Pendulum with Sliding Mode Control

The second example (<code>src/pendulum_sliding_mode/main.jl</code>) simulates a nonlinear cart–pole system with disturbances and stabilizes it using Sliding Mode Control (SMC).

### State variables and conventions

The code uses the following state definition:

$$
\mathbf{x} = \begin{bmatrix} x & \dot{x} & \theta & \dot{\theta} \end{bmatrix}^T
$$

- $x$ is the cart position (meters),
- $\theta$ is the pole angle (radians) with $\theta = 0$ being upright,
- sign convention is chosen such that $\theta &gt; 0$ visually leans the pole to the right.

### Disturbances (exactly two)

The example injects two disturbance pulses (implemented as an external torque about the pole pivot):

- pulse 1: starts at $t = 0.5\,s$, positive torque (“push right”)
- pulse 2: starts at $t = 5.0\,s$, negative torque (“push left”)

Each pulse uses a smooth half-sine profile over a 0.5 second duration, and the disturbance arrow is shown for 0.5 seconds only.

Disturbance torque model:

$$
\tau_{ext}(t) =
\begin{cases}
+\tau_{amp}\sin\left(\pi\frac{t-t_1}{d}\right) &amp; t\in[t_1,t_1+d] \\
-\tau_{amp}\sin\left(\pi\frac{t-t_2}{d}\right) &amp; t\in[t_2,t_2+d] \\
0 &amp; \text{otherwise}
\end{cases}
$$

where $d = 0.5\,s$ is the pulse duration.

For visualization and logging, the code also reports an “equivalent bob force”:

$$
F_{eq}(t) = \frac{\tau_{ext}(t)}{L}.
$$

### Simulation and rendering pipeline

This example is intentionally “complete”:

- integrate the nonlinear dynamics (RK4),
- render each frame using the current integrated state (Luxor),
- save frames to disk (<code>frame_000000.png</code> ...),
- optionally encode a video using FFmpeg,
- generate time-history plots (Plots.jl),
- save a CSV log for analysis.

Outputs include:

- <code>output/pendulum_sliding_mode/frames/frame_000000.png</code> ... etc.
- <code>output/pendulum_sliding_mode/cartpole_log.csv</code>
- plots: cart position, pole angle, control input, disturbance torque, equivalent force, sliding surface
- MP4 video (if FFmpeg is available): <code>output/pendulum_sliding_mode/pendulum_smc_10s_6000f.mp4</code>

---

<a id="sliding-mode-control-theory"></a>

## Sliding Mode Control (SMC) theory

Sliding Mode Control is a robust nonlinear control method designed to enforce a chosen “sliding manifold” (sliding surface) in the state space, even in the presence of disturbances and model uncertainties.

### 1) Sliding surface design

For a general second-order system, a common sliding surface is:

$$
s = \dot{e} + \lambda e,
$$

where $e$ is the tracking error. Enforcing $s \to 0$ yields stable error dynamics.

In this repository, the “error” is the inverted pendulum’s deviation from upright ($\theta$) plus a cart stabilizing term. The implemented sliding surface is:

$$
s(\mathbf{x}) = \dot{\theta} + \lambda_{\theta}\,\theta + \alpha\left(\dot{x} + \lambda_x\,x\right).
$$

### 2) Reaching condition and Lyapunov stability intuition

A classical sufficient condition for reaching and maintaining the sliding manifold is:

$$
\frac{d}{dt}\left(\frac{1}{2}s^2\right) = s\dot{s} \le -\eta |s|,
$$

with $\eta &gt; 0$. This implies finite-time convergence of $s$ to zero.

A standard reaching law is:

$$
\dot{s} = -k\,\mathrm{sign}(s),
$$

where $k &gt; 0$ is a gain.

### 3) Boundary layer to reduce chattering

To reduce chattering, the discontinuous sign function is replaced by a continuous saturation function:

$$
\mathrm{sat}(z) =
\begin{cases}
-1 &amp; z &lt; -1 \\
z  &amp; |z| \le 1 \\
+1 &amp; z &gt; 1
\end{cases}
$$

and the reaching law becomes:

$$
\dot{s} = -k\,\mathrm{sat}\left(\frac{s}{\phi}\right),
$$

where $\phi &gt; 0$ defines the boundary layer thickness.

### 4) How the code computes the control input

Because the plant dynamics are nonlinear, the code uses a local affine approximation:

1. Desired surface derivative:
$\dot{s}_{des} = -k\,\mathrm{sat}\left(\frac{s}{\phi}\right).$

2. Local approximation:
$\dot{s}(u) \approx a u + b.$

3. Numerical estimation (nominal, disturbance ignored):
$a \approx \dot{s}(1) - \dot{s}(0), \quad b \approx \dot{s}(0).$

4. Solve for $u$:
$u_{smc} = \frac{\dot{s}_{des} - b}{a}.$

5. Apply saturation and add a gated centering term:
$u = \mathrm{clamp}(u_{smc} + u_{hold}, -u_{max}, u_{max}).$

### 5) Cart-centering term with gating

The cart-centering term is activated only near the upright configuration:

$$
u_{hold} = g(\theta)\left(-k_p x - k_d \dot{x}\right),
$$

with:

$$
g(\theta) = \mathrm{clamp}\left(1 - \frac{|\theta|}{\theta_{gate}}, 0, 1\right).
$$

---

<a id="numerical-methods-used-in-this-repo"></a>

## Numerical methods used in this repo

### PDE solver: explicit finite differences (FTCS)

- space: central differences (2nd order),
- time: forward Euler (1st order),
- explicit stability constraint used to select $\Delta t$.

### ODE solver: RK4 with sub-stepping

- RK4 integration for nonlinear dynamics,
- multiple physics substeps per frame to maintain stability at 600 FPS output,
- deterministic frame pipeline (one state drives each frame).

---

<a id="solving-odes-and-pdes-in-julia"></a>

## Solving ODEs and PDEs in Julia (workflow + plotting + animation)

Julia supports multiple solution strategies. This repository intentionally demonstrates the “from first principles” approach:

- discretize PDEs manually (finite differences),
- integrate ODEs manually (RK4),
- render and plot results in a reproducible environment.

In production research/engineering, Julia also offers specialized solvers (e.g., adaptive ODE solvers, stiff integrators, PDE discretization frameworks). The approach here is intentionally explicit so that each numerical step is visible and modifiable.

### Plotting in Julia (Plots.jl)

This repository uses <strong>Plots.jl</strong> with the <strong>GR</strong> backend:

- heatmaps: <code>heatmap(x, y, T)</code>
- line plots: <code>plot(t, signal)</code>
- saving: <code>savefig(p, "file.png")</code>

Plot quality is configured to generate high-quality PNGs:

- 300 DPI,
- large fonts,
- thick lines,
- limited tick count (≤ 10),
- extra padding to prevent label clipping.

### Animation frames (Luxor.jl)

The pendulum example renders each frame to a PNG:

- cart body, wheels, pole, bob,
- disturbance arrow shown only during disturbance windows,
- all geometry derived from the current integrated state.

MP4 encoding is optionally performed if <code>ffmpeg</code> is available.

---

<a id="dependencies-and-installation"></a>

## Dependencies and installation

### Required

- <strong>Julia</strong>
- Julia packages (installed automatically via <code>Pkg.instantiate()</code>):
  <ul>
    <li><strong>Plots.jl</strong></li>
    <li><strong>Luxor.jl</strong></li>
  </ul>

### Optional (recommended)

- <strong>FFmpeg</strong> for MP4 encoding.

### Install dependencies

From the repository root:

<pre><code>julia --project=. -e "using Pkg; Pkg.instantiate()"</code></pre>

---

<a id="running-the-project-generating-results"></a>

## Running the project and generating results

### Windows (CMD or Git Bash)

From the repository root, run exactly:

<pre><code>julia --project=. -e "using Pkg; Pkg.instantiate()"
julia --project=. src/heat2d/main.jl
julia --project=. src/pendulum_sliding_mode/main.jl</code></pre>

### macOS / Linux

From the repository root:

<pre><code>julia --project=. -e "using Pkg; Pkg.instantiate()"
julia --project=. src/heat2d/main.jl
julia --project=. src/pendulum_sliding_mode/main.jl</code></pre>

---

<a id="repository-file-guide"></a>

## Repository file guide (full explanation)

### <code>Project.toml</code>

Defines the Julia project and its dependencies. Using <code>--project=.</code> ensures the scripts run with the correct environment.

### <code>src/heat2d/main.jl</code>

Implements a full PDE workflow:

- define the domain and grid,
- apply Dirichlet boundary conditions,
- compute a stable explicit time step,
- integrate the 2D heat equation forward in time,
- save heatmap snapshots and summary plots,
- log center temperature and save to CSV.

### <code>src/pendulum_sliding_mode/main.jl</code>

Implements a full ODE + control + rendering workflow:

- cart–pole parameterization including inertia terms,
- RK4 integration with sub-steps per frame,
- two scheduled external torque disturbances,
- sliding mode controller with boundary layer + gated cart-centering,
- Luxor frame renderer for 6000 frames,
- optional FFmpeg video encoding,
- plots and CSV logging of key signals.

---

<a id="operating-system-guides-windows-macos-linux"></a>

## Operating system guides (Windows / macOS / Linux)

### Windows

1) Confirm Julia is on PATH:
<pre><code>julia --version</code></pre>

2) (Optional) Install FFmpeg and verify:
<pre><code>ffmpeg -version</code></pre>

3) Run the project:
<pre><code>julia --project=. -e "using Pkg; Pkg.instantiate()"
julia --project=. src/heat2d/main.jl
julia --project=. src/pendulum_sliding_mode/main.jl</code></pre>

### macOS

- Install Julia and (optionally) FFmpeg:
<pre><code>brew install ffmpeg</code></pre>

- Run the project using the same three commands.

### Linux

- Install FFmpeg if needed:
<pre><code>sudo apt-get update
sudo apt-get install -y ffmpeg</code></pre>

- Run the project using the same three commands.

---

<a id="troubleshooting"></a>

## Troubleshooting

### Package errors

If you see “Package X not found,” run:

<pre><code>julia --project=. -e "using Pkg; Pkg.instantiate()"</code></pre>

### MP4 not created

If FFmpeg is not on PATH, MP4 encoding is skipped. You can encode manually:

<pre><code>ffmpeg -y -framerate 600 -i output/pendulum_sliding_mode/frames/frame_%06d.png \
  -c:v libx264 -pix_fmt yuv420p output/pendulum_sliding_mode/pendulum_smc.mp4</code></pre>

---

<a id="implementation-tutorial-video"></a>

## Implementation tutorial video

At the end of the workflow, you can watch the full implementation and walkthrough on YouTube.

<!-- Replace YOUR_VIDEO_ID with your actual video ID -->
<a href="https://www.youtube.com/watch?v=f4hElz3-Cmw" target="_blank">
  <img
    src="https://i.ytimg.com/vi/f4hElz3-Cmw/maxresdefault.jpg"
    alt="ODE/PDE in Julia - Implementation Tutorial"
    style="max-width: 100%; border-radius: 10px; box-shadow: 0 6px 18px rgba(0,0,0,0.18); margin-top: 0.5rem;"
  />
</a>
