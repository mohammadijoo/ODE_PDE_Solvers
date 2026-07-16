<div style="font-family: system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; line-height: 1.6;">

  <h1 align="center" style="margin-bottom: 0.2em;">ODE &amp; PDE Solvers in Scala (2D Heat Equation + Inverted Pendulum SMC)</h1>

  <p style="font-size: 0.98rem; max-width: 920px; margin: 0.2rem auto 0;">
    A Scala repository focused on <strong>numerical simulation</strong> and <strong>visualization</strong> of:
    <strong>partial differential equations (PDEs)</strong> and <strong>ordinary differential equations (ODEs)</strong>.
    The project includes two complete, end-to-end examples:
  </p>

  <ul style="max-width: 920px; margin: 0.5rem auto 0.2rem; padding-left: 1.2rem;">
    <li>
      <strong>PDE:</strong> 2D heat-conduction equation solved with an explicit finite-difference method (FTCS),
      generating <strong>heatmap snapshots</strong> and summary plots via <strong>JFreeChart</strong>.
    </li>
    <li>
      <strong>ODE:</strong> a nonlinear cart–pole (inverted pendulum) system stabilized using
      <strong>Sliding Mode Control (SMC)</strong>, with high-FPS frame rendering via <strong>Java2D</strong>
      (optional preview using <strong>Swing</strong>), plots via <strong>JFreeChart</strong>, and optional MP4 encoding using <strong>FFmpeg</strong>.
    </li>
  </ul>

  <p align="center" style="font-size: 1rem; color: #666; margin: 0.35rem auto 0;">
    Build system: <strong>Maven</strong> • Language: <strong>Scala</strong> • Plotting: <strong>JFreeChart</strong> •
    Animation: <strong>Java2D</strong> (optional Swing preview) • Optional video encoding: <strong>FFmpeg</strong>
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
  <li> <a href="#plots-and-animations-jfreechart-java2d-ffmpeg">Plots and animations (JFreeChart, Java2D, FFmpeg)</a></li>
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

This repository is designed as a practical reference for implementing numerical solvers in modern **Scala**:

- A <strong>PDE workflow</strong> that starts from a physical model (heat conduction), discretizes it, runs a time-marching solver,
  and produces high-quality plots and image snapshots.
- An <strong>ODE + control workflow</strong> that implements a nonlinear plant (cart–pole), injects disturbances, runs a robust controller
  (Sliding Mode Control), generates simulation plots, and produces a frame-by-frame animation and optional MP4.

The goal is not just “making it work,” but also illustrating common engineering requirements:

- stable time stepping (explicit stability constraints),
- clean logging to CSV for reproducibility,
- plot generation for analysis,
- animations for communication and debugging,
- portable builds via Maven.

### Quick map of the two applications

- <code>com.mohammadijoo.odepde.heat2d.Heat2DApp</code> → solves the 2D heat equation and writes results into <code>output/heat2d/</code>
- <code>com.mohammadijoo.odepde.pendulum_sliding_mode.CartPoleSmcApp</code> → simulates a cart–pole and writes results into <code>output/pendulum_sliding_mode/</code>

### Plot output policy (quality + locale)

All plots/snapshots are configured for:

- high-resolution PNG output sized for ~300 DPI printing (default 2400×1800),
- large titles/labels and thick lines,
- at most ~10 tick labels per axis (bounded tick count),
- at least 20 px padding on all sides,
- <strong>English/Latin digits</strong> for numeric tick labels even if the OS display language is not English,
- tick labels formatted with <strong>exactly one decimal</strong> (e.g., <code>1.0</code>).

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

In this repository, the inverted pendulum example is a nonlinear ODE system integrated in time using an RK4 scheme.

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

The PDE solved in <code>src/main/scala/com/mohammadijoo/odepde/heat2d/Heat2DApp.scala</code> is:

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

The code computes this value and uses a conservative factor (<code>0.80</code>) to avoid running near the limit.

### What you get as outputs

When you run the Heat2D application, you will find:

- multiple heatmap snapshots: <code>output/heat2d/heat_tXXXX.png</code>
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

The second example (<code>src/main/scala/com/mohammadijoo/odepde/pendulum_sliding_mode/CartPoleSmcApp.scala</code>) simulates a nonlinear cart–pole system with disturbances and stabilizes it using Sliding Mode Control (SMC).

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
- Render each frame using the current integrated state (Java2D offscreen rendering)
- Save frames to disk
- Optionally encode a video using FFmpeg
- Generate plots (JFreeChart)
- Save a CSV log for analysis

Outputs include:

- <code>output/pendulum_sliding_mode/frames/frame_000000.png</code> ... etc.
- <code>output/pendulum_sliding_mode/cartpole_log.csv</code>
- plots: cart position, pole angle, control force, disturbance torque, sliding surface, etc.
- MP4 video (if FFmpeg is available on PATH)

Frame configuration:

- total animation duration: <strong>10 seconds</strong>
- default frames: <strong>6000</strong> (override using <code>--frames N</code>)
- physics integration uses sub-steps per frame for stability

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

where $k &gt; 0$ is a gain.

### 3) Boundary layer to reduce chattering

The ideal sign function can cause chattering (high-frequency switching) in practice. A common mitigation is to replace $sign(s)$ with a continuous saturation function:

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

The actual plant dynamics are nonlinear and do not provide a simple closed-form affine relationship between the cart force $u$ and the sliding surface derivative $\dot{s}$.

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
- easy to extend later (e.g., larger grids, different BCs).

Limitations:

- stability constraint can force small $\Delta t$ for fine grids,
- explicit time stepping can be inefficient for very stiff diffusion regimes.

A natural extension (not implemented here) is an implicit method (e.g., Crank–Nicolson or ADI) to remove/relax stability constraints.

### ODE solver: RK4 with sub-stepping

The cart–pole uses classical Runge–Kutta 4th order (RK4). The simulation renders frames at high FPS, so the code uses <em>substeps per frame</em>:

- choose a frame time-step $\Delta t_{frame}$ based on desired animation duration and frame count,
- integrate dynamics with smaller steps $\Delta t_{physics} = \Delta t_{frame}/N_{substeps}$.

This is a common workflow for physically plausible animations:

- stable dynamics integration,
- deterministic frame output,
- tight coupling between simulation and visualization.

---

<a id="plots-and-animations-jfreechart-java2d-ffmpeg"></a>

## Plots and animations (JFreeChart, Java2D, FFmpeg)

### JFreeChart (plots and heatmap snapshots)

This repository uses <strong>JFreeChart</strong> to save high-quality PNG images:

- line plots (time histories, cross-sections),
- heatmap snapshots (via an XY block renderer and a color legend),
- consistent styling across plots (font sizes, tick formatting, padding, line thickness).

No external plotting runtime (like gnuplot) is needed.

### Java2D + optional Swing preview (animation frames)

The inverted pendulum example uses Java2D rendering:

- frames are rendered offscreen into a buffered image,
- each frame is written as a PNG under <code>output/pendulum_sliding_mode/frames/</code>,
- an optional preview window can be enabled via <code>--preview</code>.

### FFmpeg (optional) for MP4 encoding

If FFmpeg is installed and available in <code>PATH</code>, the pendulum program will encode an MP4 automatically.

If FFmpeg is not available, the program still generates PNG frames, and you can encode the video manually.

---

<a id="dependencies-and-installation"></a>

## Dependencies and installation

### Required

- <strong>JDK 11+</strong>
- <strong>Maven</strong>

### Optional (recommended for video)

- <strong>FFmpeg</strong> (only needed if you want automatic MP4 encoding)

### What is managed by Maven?

- Scala compiler integration via <code>scala-maven-plugin</code>
- Charting library dependency: <code>org.jfree:jfreechart</code>
- Application runner: <code>exec-maven-plugin</code>

---

<a id="building-the-project"></a>

## Building the project

From the repository root:

<pre><code>mvn -q -DskipTests package</code></pre>

This compiles the Scala sources and builds the Maven artifact under <code>target/</code>.

---

<a id="running-the-executables-and-generating-results"></a>

## Running the executables and generating results

### Build + run (three commands)

From the project root:

<pre><code>mvn -q -DskipTests package
mvn -q -Dexec.mainClass=com.mohammadijoo.odepde.heat2d.Heat2DApp exec:java
mvn -q -Dexec.mainClass=com.mohammadijoo.odepde.pendulum_sliding_mode.CartPoleSmcApp exec:java</code></pre>

Optional preview window during cart–pole generation:

<pre><code>mvn -q -Dexec.mainClass=com.mohammadijoo.odepde.pendulum_sliding_mode.CartPoleSmcApp -Dexec.args="--preview" exec:java</code></pre>

Optional frame-count override (default is 6000 frames):

<pre><code>mvn -q -Dexec.mainClass=com.mohammadijoo.odepde.pendulum_sliding_mode.CartPoleSmcApp -Dexec.args="--frames 6000" exec:java</code></pre>

### Output folders

- Heat2D outputs: <code>output/heat2d/</code>
- Cart–pole outputs: <code>output/pendulum_sliding_mode/</code>

---

<a id="repository-file-guide"></a>

## Repository file guide (full explanation)

This section explains every file in the repository (excluding <code>.gitignore</code> and this <code>README.md</code>).

### Project root

- <code>pom.xml</code>  
  Maven build definition:
  - Scala compilation via <code>net.alchim31.maven:scala-maven-plugin</code>
  - Plotting dependency via <code>org.jfree:jfreechart</code>
  - Running via <code>org.codehaus.mojo:exec-maven-plugin</code>

### <code>scripts/</code> (optional convenience wrappers)

- <code>scripts/run_heat2d.cmd</code>  
  Windows CMD helper to build and run the Heat2D example.

- <code>scripts/run_pendulum_smc.cmd</code>  
  Windows CMD helper to build and run the cart–pole SMC example.

- <code>scripts/run_heat2d.sh</code>  
  Git Bash / POSIX shell helper to build and run the Heat2D example.

- <code>scripts/run_pendulum_smc.sh</code>  
  Git Bash / POSIX shell helper to build and run the cart–pole SMC example.

### Scala source code

- <code>src/main/scala/com/mohammadijoo/odepde/common/CsvUtils.scala</code>  
  Minimal CSV writer utilities:
  - strict column-length checks
  - 2-column CSV writer and generic N-column CSV writer

- <code>src/main/scala/com/mohammadijoo/odepde/common/PlotUtils.scala</code>  
  High-quality plotting utilities (JFreeChart):
  - line plots with thick strokes, large fonts, bounded tick count, and padding
  - heatmaps using an XY block renderer plus a color legend
  - English/Latin digits and one-decimal tick formatting independent of OS locale

- <code>src/main/scala/com/mohammadijoo/odepde/heat2d/Heat2DApp.scala</code>  
  PDE example entrypoint:
  - explicit FTCS time integration for the 2D heat equation
  - Dirichlet boundaries (left hot, other edges cold)
  - periodic snapshot heatmaps + final plots + CSV log

- <code>src/main/scala/com/mohammadijoo/odepde/pendulum_sliding_mode/CartPoleSmcApp.scala</code>  
  ODE + control example entrypoint:
  - nonlinear cart–pole dynamics integrated by RK4
  - sliding mode controller with boundary-layer saturation
  - two timed disturbance torques and arrow visualization
  - frame rendering (PNG), optional preview window, optional MP4 encoding via FFmpeg
  - plots + CSV log

---

<a id="operating-system-guides-windows-macos-linux"></a>

## Operating system guides (Windows / macOS / Linux)

### Windows 10 (CMD or Git Bash)

1) Install prerequisites:
- JDK 11+ (ensure <code>java</code> is on PATH)
- Maven (ensure <code>mvn</code> is on PATH)
- Optional: FFmpeg (ensure <code>ffmpeg</code> is on PATH)

2) Quick checks:
<pre><code>java -version
mvn -version
ffmpeg -version</code></pre>

3) Build + run (three commands):
<pre><code>mvn -q -DskipTests package
mvn -q -Dexec.mainClass=com.mohammadijoo.odepde.heat2d.Heat2DApp exec:java
mvn -q -Dexec.mainClass=com.mohammadijoo.odepde.pendulum_sliding_mode.CartPoleSmcApp exec:java</code></pre>

### macOS (Homebrew)

1) Install prerequisites:
<pre><code>brew update
brew install openjdk maven ffmpeg</code></pre>

2) Build + run:
<pre><code>mvn -q -DskipTests package
mvn -q -Dexec.mainClass=com.mohammadijoo.odepde.heat2d.Heat2DApp exec:java
mvn -q -Dexec.mainClass=com.mohammadijoo.odepde.pendulum_sliding_mode.CartPoleSmcApp exec:java</code></pre>

3) Manual MP4 encoding (if you prefer doing it yourself):
<pre><code>ffmpeg -y -framerate 600 -i output/pendulum_sliding_mode/frames/frame_%06d.png \
  -c:v libx264 -pix_fmt yuv420p output/pendulum_sliding_mode/pendulum.mp4</code></pre>

### Linux (Ubuntu/Debian)

1) Install prerequisites:
<pre><code>sudo apt-get update
sudo apt-get install -y default-jdk maven ffmpeg</code></pre>

2) Build + run:
<pre><code>mvn -q -DskipTests package
mvn -q -Dexec.mainClass=com.mohammadijoo.odepde.heat2d.Heat2DApp exec:java
mvn -q -Dexec.mainClass=com.mohammadijoo.odepde.pendulum_sliding_mode.CartPoleSmcApp exec:java</code></pre>

3) Manual MP4 encoding (optional):
<pre><code>ffmpeg -y -framerate 600 -i output/pendulum_sliding_mode/frames/frame_%06d.png \
  -c:v libx264 -pix_fmt yuv420p output/pendulum_sliding_mode/pendulum.mp4</code></pre>

---

<a id="troubleshooting"></a>

## Troubleshooting

### Build fails because Maven or Java is not found

- Confirm <code>java -version</code> and <code>mvn -version</code> work in the same terminal session.
- On Windows, after changing PATH, close and reopen the terminal.

### MP4 not created

- The program only creates an MP4 if <code>ffmpeg</code> is available on PATH.
- If FFmpeg is missing, you will still get PNG frames, and you can encode manually.

### Output folders are empty

- Ensure you are running commands from the repository root.
- Outputs are written under <code>output/</code> relative to the working directory.

### Frames take a long time to generate

- Generating 6000 PNG frames is IO-heavy.
- Use <code>--frames</code> to reduce the number (e.g., 1800) if you want faster runs while testing.

---

<a id="implementation-tutorial-video"></a>

## Implementation tutorial video

If you have a walkthrough video for this Scala repository, place the link here.

<!-- Replace with your real YouTube URL and thumbnail when available. -->

<a href="https://www.youtube.com/watch?v=sNCxkrdFH3s" target="_blank">
  <img
    src="https://i.ytimg.com/vi/sNCxkrdFH3s/maxresdefault.jpg"
    alt="ODE/PDE in Scala - Implementation Tutorial"
    style="max-width: 100%; border-radius: 10px; box-shadow: 0 6px 18px rgba(0,0,0,0.18); margin-top: 0.5rem;"
  />
</a>

