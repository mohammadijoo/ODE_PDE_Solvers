<div style="font-family: system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; line-height: 1.6;">

  <h1 align="center" style="margin-bottom: 0.2em;">ODE &amp; PDE Solvers in Go (Heat Equation + Inverted Pendulum SMC)</h1>

  <p style="font-size: 0.98rem; max-width: 920px; margin: 0.2rem auto 0;">
    A Go repository focused on <strong>numerical simulation</strong> and <strong>visualization</strong> of:
    <strong>partial differential equations (PDEs)</strong> and <strong>ordinary differential equations (ODEs)</strong>.
    The project includes two complete, end-to-end examples:
  </p>

  <ul style="max-width: 920px; margin: 0.5rem auto 0.2rem; padding-left: 1.2rem;">
    <li>
      <strong>PDE:</strong> 2D heat-conduction equation solved with an explicit finite-difference method (FTCS),
      generating <strong>high-resolution heatmap snapshots</strong> and summary plots via <strong>Gonum Plot</strong>.
    </li>
    <li>
      <strong>ODE:</strong> a nonlinear cart–pole (inverted pendulum) system stabilized using
      <strong>Sliding Mode Control (SMC)</strong>, with high-FPS frame rendering via Go’s
      <strong>image</strong> libraries, plots via <strong>Gonum Plot</strong>, and optional MP4 encoding using <strong>FFmpeg</strong>.
    </li>
  </ul>

  <p align="center" style="font-size: 1rem; color: #666; margin: 0.35rem auto 0;">
    Language: <strong>Go</strong> • Plotting: <strong>Gonum Plot</strong> •
    Frame rendering: <strong>image/png</strong> • Optional video encoding: <strong>FFmpeg</strong>
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
  <li> <a href="#plots-and-animations-gonum-plot-png-ffmpeg">Plots and animations (Gonum Plot, PNG, FFmpeg)</a></li>
  <li> <a href="#dependencies-and-installation">Dependencies and installation</a></li>
  <li> <a href="#building-the-project">Building the project</a></li>
  <li> <a href="#running-the-executables-and-generating-results">Running the executables and generating results</a></li>
  <li> <a href="#repository-file-guide">Repository file guide (full explanation)</a></li>
  <li> <a href="#operating-system-guides-windows-macos-linux">Operating system guides (Windows / macOS / Linux)</a></li>
  <li> <a href="#troubleshooting">Troubleshooting</a></li>
  <li> <a href="#implementation-tutorial-video">Implementation tutorial video</a></li>
</ul>

---

<a id="about-this-repository"></a>

## About this repository

This repository is designed as a practical reference for implementing numerical solvers in modern Go:

- A <strong>PDE workflow</strong> that starts from a physical model (heat conduction), discretizes it, runs a time-marching solver,
  and produces professional plots and image snapshots.
- An <strong>ODE + control workflow</strong> that implements a nonlinear plant (cart–pole), injects disturbances, runs a robust controller
  (Sliding Mode Control), generates simulation plots, and produces a frame sequence and optional video.

The intent is not just “making it work,” but also illustrating common engineering requirements:

- stable time stepping (CFL / explicit stability constraints),
- clean logging to CSV for reproducibility,
- high-resolution plot generation for analysis and reporting,
- deterministic frame generation for communication and debugging,
- simple, portable builds using the Go toolchain.

### Quick map of the two executables

- <code>heat2d</code> → solves the 2D heat equation and writes results into <code>output/heat2d/</code>
- <code>pendulum_sliding_mode</code> → simulates a cart–pole and writes results into <code>output/pendulum_sliding_mode/</code>


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

In this repository, the inverted pendulum example is a nonlinear ODE system integrated in time using an RK4 scheme with sub-stepping.

### Partial Differential Equations (PDEs)

A PDE involves partial derivatives with respect to two or more independent variables (e.g., time and space). A canonical example is the
heat equation (diffusion equation):

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

The PDE solved in <code>src/heat2d/main.go</code> is:

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

### Stability constraint (explicit diffusion)

The explicit 2D diffusion scheme requires a stability constraint on $\Delta t$. A commonly used sufficient condition is:

$$
\Delta t \le \frac{1}{2\alpha \left(\frac{1}{\Delta x^2} + \frac{1}{\Delta y^2}\right)}.
$$

The code computes this value and applies a conservative factor (<code>0.80</code>) to avoid running near the limit.

### What you get as outputs

When you run <code>heat2d</code>, you will find:

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

The second example (<code>src/pendulum_sliding_mode/main.go</code>) simulates a nonlinear cart–pole system with disturbances and stabilizes it using Sliding Mode Control (SMC).

### State variables and conventions

The code uses the following state definition:

$$
\mathbf{x} = \begin{bmatrix} x & \dot{x} & \theta & \dot{\theta} \end{bmatrix}^T
$$

- $x$ is the cart position (meters),
- $\theta$ is the pole angle (radians) with $\theta=0$ being upright,
- sign convention is chosen such that $\theta &gt; 0$ visually leans the pole to the right.

Two practical helper operations are used:

- angle wrapping into $[-\pi,\pi]$ to avoid numeric drift,
- clamping/saturation for control and track-limit logic.

### Disturbances (exactly two)

The example injects two disturbance pulses (implemented as an external torque about the pole pivot):

- pulse 1: starts at $t=0.5\,s$, positive torque (“push right”)
- pulse 2: starts at $t=5.0\,s$, negative torque (“push left”)

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

where $d=0.5\,s$ is the pulse duration.

For visualization and logging, the code also reports an “equivalent bob force”:

$$
F_{eq}(t) = \frac{\tau_{ext}(t)}{L}.
$$

### Simulation and rendering pipeline

This example is intentionally “complete”:

- Integrate the nonlinear dynamics (RK4)
- Render each frame using the current integrated state (offscreen PNG rendering)
- Save frames to disk
- Optionally encode a video using FFmpeg
- Generate high-resolution plots
- Save a CSV log for offline analysis

Outputs include:

- <code>output/pendulum_sliding_mode/frames/frame_000000.png</code> ... <code>frame_005999.png</code>
- <code>output/pendulum_sliding_mode/cartpole_log.csv</code>
- multiple plots: cart position, pole angle, control force, disturbance torque, sliding surface, etc.
- MP4 video (if FFmpeg is available)


---

<a id="sliding-mode-control-theory"></a>

## Sliding Mode Control (SMC) theory

Sliding Mode Control is a robust nonlinear control method designed to enforce a chosen “sliding manifold” (sliding surface) in the state space, even in the presence of disturbances and model uncertainties.

### 1) Sliding surface design

For a general second-order system, a common sliding surface is:

$$
s = \dot{e} + \lambda e,
$$

where $e$ is the tracking error. Enforcing $s \to 0$ typically yields stable error dynamics.

In this repository, the “error” is the inverted pendulum’s deviation from upright ($\theta$) plus a cart stabilizing term. The implemented sliding surface is:

$$
s(\mathbf{x}) = \dot{\theta} + \lambda_{\theta}\,\theta + \alpha\left(\dot{x} + \lambda_x\,x\right).
$$

### 2) Reaching condition and stability intuition

A classical sufficient condition for reaching and maintaining the sliding manifold is the Lyapunov inequality:

$$
\frac{d}{dt}\left(\frac{1}{2}s^2\right) = s\dot{s} \le -\eta |s|,
$$

with $\eta &gt; 0$.

### 3) Boundary layer to reduce chattering

The ideal sign function can cause chattering. A common mitigation is to replace sign$(s)$ with a continuous saturation:

$$
\dot{s} = -k\,\mathrm{sat}\left(\frac{s}{\phi}\right).
$$

### 4) How the code computes the control input

The plant dynamics are nonlinear, so the code estimates a local affine approximation:

- $\dot{s}(u) \approx a u + b$,
- estimate $a,b$ numerically from two evaluations,
- solve $u = (\dot{s}_{des}-b)/a$,
- add a gated cart-hold term and saturate the final force.

### 5) Cart-centering term with gating

A gated cart-centering term avoids fighting the catch maneuver during large angle errors:

$$
u_{hold} = g(\theta)\left(-k_p x - k_d \dot{x}\right), \quad
g(\theta) = \mathrm{clamp}\left(1 - \frac{|\theta|}{\theta_{gate}}, 0, 1\right).
$$


---

<a id="numerical-methods-used-in-this-repo"></a>

## Numerical methods used in this repo

### PDE solver: explicit finite differences (FTCS)

- Space: central difference (2nd order)
- Time: forward Euler (1st order)

### ODE solver: RK4 with sub-stepping

- Classical Runge–Kutta 4th order (RK4)
- Several physics substeps per rendered frame for stability


---

<a id="plots-and-animations-gonum-plot-png-ffmpeg"></a>

## Plots and animations (Gonum Plot, PNG, FFmpeg)

- Plots are generated as high-resolution PNG images using a 300 DPI canvas.
- Axis labels and titles are set to large font sizes and thick strokes.
- Tick label count is limited to at most 10 labels per axis.
- The animation is saved as PNG frames and optionally encoded to MP4 via FFmpeg.


---

<a id="dependencies-and-installation"></a>

## Dependencies and installation

### Required

- Go 1.22+

### Optional

- FFmpeg (for MP4 encoding)

Install Go dependencies:

<pre><code>go mod tidy</code></pre>


---

<a id="building-the-project"></a>

## Building the project

<pre><code>go mod tidy
go build -o bin/heat2d ./src/heat2d
go build -o bin/pendulum_sliding_mode ./src/pendulum_sliding_mode</code></pre>


---

<a id="running-the-executables-and-generating-results"></a>

## Running the executables and generating results

<pre><code>go run ./src/heat2d
go run ./src/pendulum_sliding_mode</code></pre>

Manual MP4 encoding (if needed):

<pre><code>ffmpeg -y -framerate 600 -i output/pendulum_sliding_mode/frames/frame_%06d.png \
  -c:v libx264 -pix_fmt yuv420p output/pendulum_sliding_mode/pendulum_manual.mp4</code></pre>


---

<a id="repository-file-guide"></a>

## Repository file guide (full explanation)

- <code>src/heat2d/main.go</code>: 2D heat equation FTCS solver + heatmap snapshots + summary plots + CSV log.
- <code>src/pendulum_sliding_mode/main.go</code>: cart–pole dynamics + SMC + two disturbances + 6000 frame render + plots + CSV + optional FFmpeg encoding.
- <code>output/</code>: generated results (created after running programs).

---

<a id="operating-system-guides-windows-macos-linux"></a>

## Operating system guides (Windows / macOS / Linux)

### Windows 10 (CMD + Git Bash)

CMD:

<pre><code>git clone https://github.com/mohammadijoo/ODE_PDE_GO.git
cd ODE_PDE_GO

go mod tidy
go run ./src/heat2d
go run ./src/pendulum_sliding_mode</code></pre>

Build binaries (CMD):

<pre><code>go build -o bin\heat2d.exe .\src\heat2d
go build -o bin\pendulum_sliding_mode.exe .\src\pendulum_sliding_mode</code></pre>

Run binaries (CMD):

<pre><code>.\bin\heat2d.exe
.\bin\pendulum_sliding_mode.exe</code></pre>

### macOS / Linux

<pre><code>go mod tidy
go run ./src/heat2d
go run ./src/pendulum_sliding_mode</code></pre>

---

<a id="troubleshooting"></a>

## Troubleshooting

- If module download fails: <code>go clean -modcache</code> then <code>go mod tidy</code>.
- If MP4 is not created: confirm <code>ffmpeg -version</code> works, then encode manually.

---

<a id="implementation-tutorial-video"></a>

## Implementation tutorial video

<a href="https://www.youtube.com/watch?v=rhmgUoC8G4A" target="_blank">
  <img
    src="https://i.ytimg.com/vi/rhmgUoC8G4A/maxresdefault.jpg"
    alt="ODE/PDE in Go - Implementation Tutorial"
    style="max-width: 100%; border-radius: 10px; box-shadow: 0 6px 18px rgba(0,0,0,0.18); margin-top: 0.5rem;"
  />
</a>
