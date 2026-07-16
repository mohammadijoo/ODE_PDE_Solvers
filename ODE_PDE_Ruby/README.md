<div style="font-family: system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; line-height: 1.6;">

  <h1 align="center" style="margin-bottom: 0.2em;">ODE &amp; PDE Solvers in Ruby (Heat Equation + Inverted Pendulum SMC)</h1>

  <p style="font-size: 0.98rem; max-width: 920px; margin: 0.2rem auto 0;">
    A Ruby repository focused on <strong>numerical simulation</strong> and <strong>visualization</strong> of:
    <strong>partial differential equations (PDEs)</strong> and <strong>ordinary differential equations (ODEs)</strong>.
    The project includes two complete, end-to-end examples:
  </p>

  <ul style="max-width: 920px; margin: 0.5rem auto 0.2rem; padding-left: 1.2rem;">
    <li>
      <strong>PDE:</strong> 2D heat-conduction equation solved with an explicit finite-difference method (FTCS),
      generating <strong>heatmap snapshots</strong> and summary plots via <strong>Gnuplot</strong>.
    </li>
    <li>
      <strong>ODE:</strong> a nonlinear cart–pole (inverted pendulum) system stabilized using
      <strong>Sliding Mode Control (SMC)</strong>, with high-FPS frame rendering via <strong>ChunkyPNG</strong>,
      plots via <strong>Gnuplot</strong>, and optional MP4 encoding using <strong>FFmpeg</strong>.
    </li>
  </ul>

  <p align="center" style="font-size: 1rem; color: #666; margin: 0.35rem auto 0;">
    Runtime: <strong>Ruby</strong> • Plotting: <strong>Gnuplot</strong> •
    Frame rendering: <strong>ChunkyPNG</strong> • Optional video encoding: <strong>FFmpeg</strong>
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
  <li> <a href="#plots-and-frames-gnuplot-chunkypng-ffmpeg">Plots and frames (Gnuplot, ChunkyPNG, FFmpeg)</a></li>
  <li> <a href="#dependencies-and-installation">Dependencies and installation</a></li>
  <li> <a href="#running-the-programs">Running the programs</a></li>
  <li> <a href="#repository-file-guide">Repository file guide (full explanation)</a></li>
  <li> <a href="#operating-system-guides-windows">Windows guide (CMD + Git Bash)</a></li>
  <li> <a href="#troubleshooting">Troubleshooting</a></li>
  <li> <a href="#implementation-tutorial-video">Implementation tutorial video</a></li>
</ul>

---

<a id="about-this-repository"></a>

## About this repository

This repository is designed as a practical reference for implementing numerical solvers and simulation pipelines in Ruby:

- A <strong>PDE workflow</strong> that starts from a physical model (heat conduction), discretizes it, runs a time-marching solver,
  and produces professional plots and snapshot images.
- An <strong>ODE + control workflow</strong> that implements a nonlinear plant (cart–pole), injects disturbances, runs a robust controller
  (Sliding Mode Control), generates simulation plots, and produces a frame sequence suitable for video encoding.

The goal is not just “making it run,” but also illustrating common engineering requirements:

- stable time stepping (CFL / explicit stability constraints),
- clean logging to CSV for reproducibility,
- plot generation for analysis,
- frame generation for communication and debugging,
- portable execution with minimal dependencies.

### Quick map of the two programs

- <code>src/heat2d/main.rb</code> → solves the 2D heat equation and writes results into <code>output/heat2d/</code>
- <code>src/pendulum_sliding_mode/main.rb</code> → simulates a cart–pole and writes results into <code>output/pendulum_sliding_mode/</code>

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

In this repository, the inverted pendulum example is a nonlinear ODE system integrated in time using an RK4 scheme (with sub-stepping per frame for stability).

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

The PDE solved in <code>src/heat2d/main.rb</code> is:

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

The code computes this value (as <code>dt_stable</code>) and uses a conservative factor (<code>0.80</code>) to avoid running near the limit.

### What you get as outputs

When you run the heat solver, you will find:

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

The second example (<code>src/pendulum_sliding_mode/main.rb</code>) simulates a nonlinear cart–pole system with disturbances and stabilizes it using Sliding Mode Control (SMC).

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
- clamping/saturation for control and gating logic.

### Disturbances (exactly two)

The example injects two disturbance pulses (implemented as an external torque about the pole pivot):

- pulse 1: starts at $t=0.5\,s$, positive torque (“push right”)
- pulse 2: starts at $t=5.0\,s$, negative torque (“push left”)

Each pulse uses a smooth half-sine profile over a 0.5 second duration, and the disturbance arrow is shown for 0.5 seconds only.

Disturbance torque model:

$$
&bsol;tau_{ext}(t) =
\begin{cases}
+&bsol;tau_{amp}\sin\left(\pi\frac{t-t_1}{d}\right) &amp; t\in[t_1,t_1+d] \\
-&bsol;tau_{amp}\sin\left(\pi\frac{t-t_2}{d}\right) &amp; t\in[t_2,t_2+d] \\
0 &amp; \text{otherwise}
\end{cases}
$$

where $d=0.5\,s$ is the pulse duration.

For visualization and logging, the code also reports an “equivalent bob force”:

$$
F_{eq}(t) = \frac{&bsol;tau_{ext}(t)}{L}.
$$

### Simulation and rendering pipeline

This example is intentionally “complete”:

- Integrate the nonlinear dynamics (RK4)
- Render each frame using the current integrated state (ChunkyPNG offscreen rendering)
- Save frames to disk
- Optionally encode a video using FFmpeg
- Generate Gnuplot plots
- Save a CSV log for analysis

Outputs include:

- <code>output/pendulum_sliding_mode/frames/frame_000000.png</code> ... etc.
- <code>output/pendulum_sliding_mode/cartpole_log.csv</code>
- multiple plots: cart position, pole angle, control force, disturbance torque, sliding surface, etc.
- MP4 video (if FFmpeg is available)

<strong>Frame count and FPS:</strong> the default configuration renders <code>6000</code> frames over 10 seconds
(i.e., <code>600 FPS</code>), and the MP4 output filename reflects this (<code>..._6000f.mp4</code>).

If you want a different duration or frame count, update <code>video_seconds</code> and/or <code>total_frames</code>
in <code>src/pendulum_sliding_mode/main.rb</code>. If you change the frame count, you may also rename the MP4 output
file for clarity.

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

Interpretation:

- $\lambda_{\theta}$ defines how strongly the controller penalizes angle error,
- $\alpha$ couples cart motion to the sliding surface (cart movement is necessary to catch the pendulum),
- $\lambda_x$ adds a “soft” cart-centering effect as part of the surface.

### 2) Reaching condition and stability intuition

A classical sufficient condition for reaching and maintaining the sliding manifold is the Lyapunov inequality:

$$
\frac{d}{dt}\left(\frac{1}{2}s^2\right) = s\dot{s} \le -\eta |s|,
$$

with $\eta &gt; 0$. This implies $s$ decreases in magnitude and reaches zero in finite time.

A typical control law to satisfy this is:

$$
\dot{s} = -k\,\mathrm{sign}(s),
$$

where $k &gt; 0$ is a gain. This creates a discontinuous control action that strongly rejects matched disturbances.

### 3) Boundary layer to reduce chattering

The ideal sign function can cause chattering (high-frequency switching) in practice. A common mitigation is to replace sign$(s)$ with a continuous saturation function:

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

where $\phi &gt; 0$ defines a boundary layer thickness around $s=0$.

This repository implements exactly this idea, with parameters:

- <code>k</code> (gain)
- <code>phi</code> (boundary layer)

### 4) How the code computes the control input

The plant dynamics are nonlinear and do not provide a simple closed-form affine relationship between the cart force $u$ and the sliding surface derivative $\dot{s}$.

The code therefore uses a pragmatic engineering approach:

1. Define the desired sliding surface derivative:  
   $\dot{s}_{des} = -k\,\mathrm{sat}\left(\frac{s}{\phi}\right).$
2. Approximate $\dot{s}(u)$ locally as an affine function:
   $\dot{s}(u) \approx a\,u + b.$
3. Estimate $a$ and $b$ numerically using two evaluations of the nominal dynamics (disturbance ignored in the control law):
   $a \approx \dot{s}(1) - \dot{s}(0), \quad b \approx \dot{s}(0).$
4. Solve for the control:
   $u_{smc} = \frac{\dot{s}_{des} - b}{a}.$
5. Saturate the actuator:
   $u = \mathrm{clamp}(u_{smc} + u_{hold}, -u_{max}, u_{max}).$

### 5) Cart-centering term with gating

A practical issue in inverted pendulum stabilization is the cart drifting away from the origin. If we always apply a centering controller, it can interfere with the fast “catching” action needed during large angle errors.

This repository uses a gated cart-centering term:

$$
u_{hold} = g(\theta)\left(-k_p x - k_d \dot{x}\right),
$$

where:

- $g(\theta) \in [0,1]$,
- $g(\theta) \approx 1$ when $|\theta|$ is small (near upright),
- $g(\theta) \approx 0$ when $|\theta|$ is large (prioritize stabilization/catch maneuver).

This is implemented by:

$$
g(\theta) = \mathrm{clamp}\left(1 - \frac{|\theta|}{\theta_{gate}}, 0, 1\right).
$$

Result: the controller focuses on balancing first, then recenters the cart when it is safe.

---

<a id="numerical-methods-used-in-this-repo"></a>

## Numerical methods used in this repo

### PDE solver: explicit finite differences (FTCS)

The heat solver is a textbook example of an explicit diffusion integrator:

- Space: central difference (2nd order)
- Time: forward Euler (1st order)

Strengths:

- straightforward to implement,
- fast per-step computation,
- easy to parallelize later.

Limitations:

- stability constraint can force small $\Delta t$ for fine grids,
- accuracy is limited by explicit time stepping for stiff diffusion regimes.

A natural extension is an implicit method (e.g., Crank–Nicolson or ADI) to remove/relax stability constraints.

### ODE solver: RK4 with sub-stepping

The cart–pole uses classical Runge–Kutta 4th order (RK4). The simulation renders frames at a fixed FPS, so the code uses <em>substeps per frame</em>:

- choose a frame time-step $\Delta t_{frame}$ based on desired animation duration and frame count,
- integrate dynamics with smaller steps $\Delta t_{physics} = \Delta t_{frame}/N_{substeps}$.

This is a common workflow for physically plausible frame generation:

- stable dynamics integration,
- deterministic frame output,
- tight coupling between simulation and visualization.

---

<a id="plots-and-frames-gnuplot-chunkypng-ffmpeg"></a>

## Plots and frames (Gnuplot, ChunkyPNG, FFmpeg)

### Gnuplot (for plots and heatmap snapshots)

This repository uses <strong>Gnuplot</strong> for all plots and heatmap snapshots.

Key plotting requirements implemented in the generated images:

- high pixel resolution (publication style, “300 dpi class”),
- large axis labels and titles,
- thick plot lines,
- limited tick label count (about 10 or fewer per axis),
- English numeric formatting and one decimal place on tick labels,
- visible padding around plot region.

### ChunkyPNG (for frame rendering)

The inverted pendulum example renders frames using <strong>ChunkyPNG</strong>:

- each frame is drawn offscreen into a PNG image buffer,
- frames are saved to <code>output/pendulum_sliding_mode/frames/</code>,
- the rendered cart–pole state is the result of the integrated dynamics at that frame time.

### FFmpeg (optional) for MP4 encoding

If FFmpeg is installed and available on <code>PATH</code>, the pendulum program will attempt to encode an MP4 automatically.

If FFmpeg is not available, the program still generates PNG frames, and you can encode the video manually (commands below).

---

<a id="dependencies-and-installation"></a>

## Dependencies and installation

### Required

- Ruby 3.0+ (recommended: RubyInstaller on Windows)
- Bundler (usually included with Ruby)
- Gnuplot (required to generate plots and heatmaps)

### Optional (recommended for video)

- FFmpeg (required only if you want MP4 encoding)

### Ruby dependencies managed by Bundler

This repository uses Bundler to manage Ruby dependencies. The required gem is:

- <code>chunky_png</code> (pure Ruby PNG generation)

Install gems:

<pre><code>bundle install</code></pre>

---

<a id="running-the-programs"></a>

## Running the programs

### 1) Run the heat equation solver (PDE)

<pre><code>bundle exec ruby src/heat2d/main.rb</code></pre>

Results appear in:

- <code>output/heat2d/</code>

### 2) Run the pendulum SMC simulation (ODE + frames)

<pre><code>bundle exec ruby src/pendulum_sliding_mode/main.rb</code></pre>

Results appear in:

- <code>output/pendulum_sliding_mode/</code>
- frames in <code>output/pendulum_sliding_mode/frames/</code>

### Manual MP4 encoding (if needed)

If FFmpeg is installed but you prefer manual control:

<pre><code>ffmpeg -y -framerate 180 -i output/pendulum_sliding_mode/frames/frame_%06d.png ^
  -c:v libx264 -pix_fmt yuv420p output/pendulum_sliding_mode/pendulum_manual.mp4</code></pre>

(Replace <code>180</code> with your chosen FPS if you change <code>total_frames</code>.)

---

<a id="repository-file-guide"></a>

## Repository file guide (full explanation)

### <code>src/heat2d/main.rb</code> (PDE solver)

This file implements a complete PDE pipeline:

- defines a 2D domain with uniform grid (<code>nx</code>, <code>ny</code>),
- applies Dirichlet boundary conditions (fixed temperatures),
- computes a stable explicit time-step constraint and chooses a conservative <code>dt</code>,
- iterates over time steps:
  - updates interior nodes via FTCS
  - reapplies boundary conditions
  - periodically saves heatmap snapshots with Gnuplot
- logs the center temperature over time and saves:
  - a centerline plot at final time
  - a time-history plot
  - a CSV log (<code>heat2d_log.csv</code>)

Engineering notes that are easy to miss but important:

- the 2D grid is flattened into a 1D vector for speed and cache efficiency,
- <code>idx(i,j,nx)</code> encodes 2D indexing,
- snapshots are throttled (<code>snapshot_every</code>) to avoid generating thousands of PNG files,
- Gnuplot scripts are written to disk (in the output folder) for reproducibility.

### <code>src/pendulum_sliding_mode/main.rb</code> (ODE + SMC + frames)

This file contains modeling, control, integration, rendering, logging, plotting, and optional video encoding.

Main components:

1) <strong>Plant model (cart–pole):</strong>
<ul>
  <li><code>PoleModel</code> computes center-of-mass and inertia parameters for a rod + bob mass.</li>
  <li><code>CartPoleParams</code> stores masses, gravity, and damping.</li>
  <li><code>dynamics(...)</code> computes state derivatives given control force and external torque.</li>
</ul>

2) <strong>Numerical integration:</strong>
<ul>
  <li>RK4 is used via <code>rk4_step!(...)</code>.</li>
  <li>Multiple physics steps per rendered frame increase stability and realism.</li>
</ul>

3) <strong>Controller:</strong>
<ul>
  <li><code>sliding_surface(...)</code> defines the SMC surface <code>s</code>.</li>
  <li><code>compute_control(...)</code> computes the SMC action with a boundary-layer saturation and adds a gated cart-centering term.</li>
  <li>Actuator saturation prevents unrealistic forces.</li>
</ul>

4) <strong>Disturbance injection:</strong>
<ul>
  <li><code>Disturbance</code> generates two torque pulses at fixed times.</li>
  <li>The “equivalent bob force” drives the direction of the on-screen arrow.</li>
</ul>

5) <strong>Rendering and output:</strong>
<ul>
  <li>ChunkyPNG draws cart, pole, bob, and disturbance arrow into an offscreen buffer.</li>
  <li>Frames are saved as PNG images.</li>
</ul>

6) <strong>Plots and CSV logging:</strong>
<ul>
  <li>Gnuplot saves time histories for position, angle, control input, disturbance, and sliding surface.</li>
  <li>CSV is written for offline analysis.</li>
</ul>

7) <strong>FFmpeg encoding:</strong>
<ul>
  <li>The program attempts to call FFmpeg automatically if it is present on PATH.</li>
  <li>Manual encoding is also supported.</li>
</ul>

### <code>lib/common/*.rb</code>

Shared utilities:

- <code>math_utils.rb</code>: clamp, angle wrapping, saturation
- <code>csv_writer.rb</code>: minimal, explicit CSV logging
- <code>gnuplot_helpers.rb</code>: high-quality Gnuplot plot generation with consistent styling rules

---

<a id="operating-system-guides-windows"></a>

## Windows guide (CMD + Git Bash)

### 1) Install Ruby (Windows 10)

Recommended: RubyInstaller (Ruby 3.x). During installation:

- enable the option to add Ruby to <code>PATH</code>
- enable MSYS2 (RubyInstaller will offer this)

Verify:

<pre><code>ruby -v
gem -v
bundle -v</code></pre>

If Bundler is missing:

<pre><code>gem install bundler</code></pre>

### 2) Install Gnuplot

You need the <code>gnuplot</code> command available on PATH.

After installation, open a new terminal and verify:

<pre><code>gnuplot --version</code></pre>

### 3) (Optional) Install FFmpeg

Verify:

<pre><code>ffmpeg -version</code></pre>

### 4) Clone the repository (CMD or Git Bash)

CMD:

<pre><code>cd %USERPROFILE%
git clone &lt;YOUR_REPO_URL&gt;
cd ODE_PDE_Ruby</code></pre>

Git Bash:

<pre><code>cd ~
git clone &lt;YOUR_REPO_URL&gt;
cd ODE_PDE_Ruby</code></pre>

### 5) Install Ruby dependencies

CMD:

<pre><code>bundle install</code></pre>

Git Bash:

<pre><code>bundle install</code></pre>

### 6) Run the programs

CMD:

<pre><code>bundle exec ruby src\heat2d\main.rb
bundle exec ruby src\pendulum_sliding_mode\main.rb</code></pre>

Git Bash:

<pre><code>bundle exec ruby src/heat2d/main.rb
bundle exec ruby src/pendulum_sliding_mode/main.rb</code></pre>

Outputs are written to:

- <code>output/heat2d/</code>
- <code>output/pendulum_sliding_mode/</code>

### 7) Manual FFmpeg encoding (CMD)

<pre><code>ffmpeg -y -framerate 180 -i output\pendulum_sliding_mode\frames\frame_%06d.png ^
  -c:v libx264 -pix_fmt yuv420p output\pendulum_sliding_mode\pendulum_manual.mp4</code></pre>

---

<a id="troubleshooting"></a>

## Troubleshooting

### Gnuplot does not produce images (or cannot run)

- Confirm <code>gnuplot --version</code> works in the same terminal where you run the scripts.
- If you installed Gnuplot but it is not in PATH, add it and restart your terminal.

### MP4 not created

- The program only creates an MP4 if <code>ffmpeg</code> is available on PATH.
- If auto-encoding fails, encode manually using the provided commands.

### The simulation is slow / frames take too long to generate

- The pendulum example saves many PNGs; disk IO can dominate runtime.
- Reduce <code>total_frames</code> if you want faster runs.
- Consider running on an SSD and closing other heavy programs.

---

<a id="implementation-tutorial-video"></a>

## Implementation tutorial video

At the end of the workflow, you can watch a full implementation and walkthrough on YouTube.

<!-- Replace YOUR_VIDEO_ID and YOUR_VIDEO_URL with your real link. -->
<a href="https://www.youtube.com/watch?v=UgHHyfmOz9o" target="_blank">
  <img
    src="https://i.ytimg.com/vi/UgHHyfmOz9o/maxresdefault.jpg"
    alt="ODE/PDE in Ruby - Implementation Tutorial"
    style="max-width: 100%; border-radius: 10px; box-shadow: 0 6px 18px rgba(0,0,0,0.18); margin-top: 0.5rem;"
  />
</a>
