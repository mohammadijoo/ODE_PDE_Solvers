<div style="font-family: system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; line-height: 1.6;">

  <h1 align="center" style="margin-bottom: 0.2em;">ODE &amp; PDE Solvers in Java (2D Heat Equation + Inverted Pendulum SMC)</h1>

  <p style="font-size: 0.98rem; max-width: 980px; margin: 0.2rem auto 0;">
    A Java 17 repository focused on <strong>numerical simulation</strong> and <strong>visualization</strong> of:
    <strong>partial differential equations (PDEs)</strong> and <strong>ordinary differential equations (ODEs)</strong>.
    The project includes two complete, end-to-end examples implemented in pure Java:
  </p>

  <ul style="max-width: 980px; margin: 0.5rem auto 0.2rem; padding-left: 1.2rem;">
    <li>
      <strong>PDE:</strong> 2D heat-conduction (diffusion) equation solved with an explicit finite-difference method (FTCS),
      generating <strong>heatmap snapshots</strong> and summary plots via <strong>JFreeChart</strong> (PNG, 300 DPI metadata).
    </li>
    <li>
      <strong>ODE + Control:</strong> a nonlinear cart–pole (inverted pendulum) system stabilized using
      <strong>Sliding Mode Control (SMC)</strong>, with frame-by-frame rendering using <strong>Java2D/Swing</strong>,
      time-series plots via <strong>JFreeChart</strong> (PNG, 300 DPI metadata), and optional MP4 encoding using <strong>FFmpeg</strong>.
    </li>
  </ul>

  <p align="center" style="font-size: 1rem; color: #666; margin: 0.35rem auto 0;">
    Build system: <strong>Maven</strong> • Plotting: <strong>JFreeChart</strong> • Animation: <strong>Java2D + Swing</strong> • Optional video encoding: <strong>FFmpeg</strong>
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
  <li> <a href="#plots-and-animations-jfreechart-java2d-ffmpeg">Plots and animations (JFreeChart, Java2D/Swing, FFmpeg)</a></li>
  <li> <a href="#dependencies-and-installation">Dependencies and installation</a></li>
  <li> <a href="#building-the-project">Building the project</a></li>
  <li> <a href="#running-the-programs-and-generating-results">Running the programs and generating results</a></li>
  <li> <a href="#repository-file-guide">Repository file guide (full explanation)</a></li>
  <li> <a href="#operating-system-guides-windows-macos-linux">Operating system guides (Windows / macOS / Linux)</a></li>
  <li> <a href="#troubleshooting">Troubleshooting</a></li>
  <li> <a href="#implementation-tutorial-video">Implementation tutorial video</a></li>
</ul>

---

<a id="about-this-repository"></a>

## About this repository

This repository is designed as a practical reference for implementing **numerical solvers in modern Java (Java 17)**:

- A <strong>PDE workflow</strong> that starts from a physical diffusion model (heat conduction), discretizes it,
  runs a time-marching solver, and produces publication-quality plots and image snapshots.
- An <strong>ODE + control workflow</strong> that implements a nonlinear plant (cart–pole), injects disturbances,
  runs a robust controller (Sliding Mode Control), generates simulation plots, and produces an animation/video.

The focus is intentionally “end-to-end engineering,” not only the core math:

- stable time stepping (explicit stability constraints),
- clean logging to CSV (reproducibility),
- plot generation (analysis),
- high-resolution images with 300 DPI metadata (documentation/publication),
- animation frames and optional MP4 video (communication and debugging),
- portable builds and execution via Maven (cross-platform).

### Quick map of the two programs

- <code>com.mohammadijoo.odepde.heat2d.Heat2DMain</code> → solves the 2D heat equation and writes results into <code>output/heat2d/</code>
- <code>com.mohammadijoo.odepde.pendulum_smc.CartPoleSMCMain</code> → simulates a cart–pole + SMC controller and writes results into <code>output/pendulum_sliding_mode/</code>

---

<a id="what-are-odes-and-pdes"></a>

## What are ODEs and PDEs?

### Ordinary Differential Equations (ODEs)

An ODE models time evolution of one or more state variables with respect to a single independent variable (usually time):

$$
\dot{\mathbf{x}}(t) = \mathbf{f}\big(\mathbf{x}(t), \mathbf{u}(t), t\big),
$$

where:

- $\mathbf{x}(t)$ is the state (e.g., cart position, pole angle, velocities),
- $\mathbf{u}(t)$ is an input (e.g., a control force),
- $\mathbf{f}(\cdot)$ encodes the physics/dynamics.

In this repository, the inverted pendulum example is a **nonlinear ODE system** integrated in time using **RK4** with **sub-stepping** for stable animation rendering.

### Partial Differential Equations (PDEs)

A PDE involves partial derivatives with respect to two or more independent variables (e.g., time and space). A canonical example is the
heat (diffusion) equation:

$$
\frac{\partial T}{\partial t} = \alpha\left(\frac{\partial^2 T}{\partial x^2}+\frac{\partial^2 T}{\partial y^2}\right),
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

The PDE solved in <code>src/main/java/com/mohammadijoo/odepde/heat2d/Heat2DMain.java</code> is:

$$
\frac{\partial T}{\partial t} = \alpha\left(\frac{\partial^2 T}{\partial x^2}+\frac{\partial^2 T}{\partial y^2}\right).
$$

Interpretation:

- heat conduction in a 2D plate,
- no internal heat sources,
- constant isotropic diffusivity $\alpha$,
- fixed temperature boundaries (Dirichlet BCs).

The boundary conditions implemented in the Java code are:

- left boundary held at $T=100$,
- right/top/bottom held at $T=0$,

which creates a diffusion front moving from the left boundary into the interior.

### Discretization summary (what the code does)

The solver uses:

- uniform grid in $x$ and $y$,
- second-order central differences for $\partial^2 T/\partial x^2$ and $\partial^2 T/\partial y^2$,
- explicit time stepping (Forward Euler in time).

At interior grid nodes $(i,j)$:

$$
T^{n+1}_{i,j}=T^{n}_{i,j}+\alpha\,\Delta t\left(
\frac{T^n_{i+1,j}-2T^n_{i,j}+T^n_{i-1,j}}{\Delta x^2}
+
\frac{T^n_{i,j+1}-2T^n_{i,j}+T^n_{i,j-1}}{\Delta y^2}
\right).
$$

The implementation uses a 1D flattened array for cache efficiency. The mapping:

$$
k=\mathrm{idx}(i,j)=j\cdot n_x+i
$$

is used throughout.

### Stability constraint (explicit diffusion)

Explicit diffusion is stable only if the time step satisfies a constraint. A commonly used sufficient condition is:

$$
\Delta t \le \frac{1}{2\alpha\left(\frac{1}{\Delta x^2}+\frac{1}{\Delta y^2}\right)}.
$$

The code computes this as <code>dtStable</code> and uses a conservative factor (0.80) to avoid operating near the stability limit.

### What you get as outputs

When you run <code>Heat2DMain</code>, you will find:

- heatmap snapshots: <code>output/heat2d/heat_t*.png</code>
- final centerline profile: <code>output/heat2d/centerline_final.png</code>
- center temperature vs time: <code>output/heat2d/center_point_vs_time.png</code>
- CSV log: <code>output/heat2d/heat2d_log.csv</code>

All PNG images are rendered at high resolution (e.g., 2400×2000 for heatmaps) and written with **300 DPI metadata** (PNG <code>pHYs</code> chunk)
to preserve intended print/export quality.

---

<a id="example-2-ode-inverted-pendulum-with-smc"></a>

## Example 2 (ODE): Inverted Pendulum with Sliding Mode Control

The second example (<code>src/main/java/com/mohammadijoo/odepde/pendulum_smc/CartPoleSMCMain.java</code>) simulates a nonlinear cart–pole system with disturbances and stabilizes it using Sliding Mode Control (SMC).

### State variables and conventions

The code uses the following state:

$$
\mathbf{x}=\begin{bmatrix}x & \dot{x} & \theta & \dot{\theta}\end{bmatrix}^T
$$

- $x$ is cart position (meters),
- $\theta$ is pole angle (radians) with $\theta=0$ being upright,
- the angle is wrapped into $[-\pi,\pi]$ to avoid numeric drift.

### Disturbances (exactly two)

Two disturbance pulses are injected as an external torque about the pole pivot:

- pulse 1: starts at $t=0.5\,\text{s}$, positive torque (“push right”)
- pulse 2: starts at $t=5.0\,\text{s}$, negative torque (“push left”)

Each pulse uses a smooth half-sine profile over a duration $d=0.5\,\text{s}$:

$$
\tau_{\mathrm{ext}}(t)=
\begin{cases}
+\tau_{\mathrm{amp}}\sin\left(\pi\frac{t-t_1}{d}\right) & t\in[t_1,t_1+d]\\
-\tau_{\mathrm{amp}}\sin\left(\pi\frac{t-t_2}{d}\right) & t\in[t_2,t_2+d]\\
0 & \text{otherwise}
\end{cases}
$$

For visualization/logging, the code reports an “equivalent bob force”:

$$
F_{\mathrm{eq}}(t)=\frac{\tau_{\mathrm{ext}}(t)}{L}.
$$

The disturbance arrow is shown for **0.5 seconds** after each pulse begins.

### Simulation and rendering pipeline

This example is intentionally complete:

- integrate nonlinear dynamics (RK4),
- render each frame using the current integrated state (Java2D),
- save frames to disk as PNG,
- generate time-series plots (JFreeChart) and save them as PNG (300 DPI metadata),
- save a CSV log,
- optionally encode an MP4 using FFmpeg.

Outputs include:

- <code>output/pendulum_sliding_mode/frames/frame_000000.png</code> ... etc.
- <code>output/pendulum_sliding_mode/cartpole_log.csv</code>
- multiple plots: cart position, pole angle, control force, disturbance torque, equivalent bob force, sliding surface
- MP4 video (if FFmpeg is available): <code>output/pendulum_sliding_mode/pendulum_smc_10s_6000f.mp4</code>

---

<a id="sliding-mode-control-theory"></a>

## Sliding Mode Control (SMC) theory

Sliding Mode Control is a robust nonlinear control method designed to enforce a chosen “sliding manifold” (sliding surface) in the state space, even in the presence of disturbances and model uncertainties.

### 1) Sliding surface design

For a general second-order tracking problem, a common sliding surface is:

$$
s=\dot{e}+\lambda e,
$$

where $e$ is the tracking error and $\lambda>0$. Enforcing $s\to 0$ yields stable error dynamics:

$$
\dot{e}+\lambda e=0 \quad \Rightarrow \quad e(t)=e(0)e^{-\lambda t}.
$$

In an inverted pendulum, the balancing objective is $\theta\to 0$ and $\dot{\theta}\to 0$ while preventing the cart from drifting.
This repository defines a composite sliding surface:

$$
s(\mathbf{x})=\dot{\theta}+\lambda_{\theta}\theta+\alpha\left(\dot{x}+\lambda_x x\right).
$$

Interpretation:

- $\lambda_{\theta}$ defines how strongly the controller penalizes angular error,
- $\alpha$ couples cart motion into the balancing objective (moving the cart is the actuation mechanism),
- $\lambda_x$ introduces a cart-centering tendency.

### 2) Reaching condition and Lyapunov argument

A classical sufficient reaching condition uses the Lyapunov function:

$$
V=\frac{1}{2}s^2.
$$

Then:

$$
\dot{V}=s\dot{s}.
$$

If the controller enforces:

$$
s\dot{s}\le -\eta|s|,\quad \eta>0,
$$

then $s$ reaches 0 in finite time and stays near the manifold (robustness property).

A standard ideal reaching law is:

$$
\dot{s}=-k\,\mathrm{sign}(s),\quad k>0,
$$

leading to:

$$
\dot{V}=s(-k\,\mathrm{sign}(s))=-k|s|\le 0.
$$

### 3) Boundary layer (reducing chattering)

The discontinuity in $\mathrm{sign}(s)$ can cause chattering in real implementations. A common remedy is to introduce a boundary layer using saturation:

$$
\mathrm{sat}(z)=
\begin{cases}
-1 & z<-1\\
z & |z|\le 1\\
+1 & z>1
\end{cases}
$$

and apply:

$$
\dot{s}=-k\,\mathrm{sat}\left(\frac{s}{\phi}\right),
$$

where $\phi>0$ defines the boundary layer thickness.

This repository uses parameters:

- <code>k</code> (gain),
- <code>phi</code> (boundary layer).

### 4) How the code computes the control input

For the cart–pole dynamics, deriving an explicit closed-form expression for $u$ in terms of $s$ and $\dot{s}$ can be messy,
especially when you include damping terms and external torque.

Instead, the Java code uses a pragmatic engineering approach:

1. Define the desired sliding surface derivative:
   $\dot{s}_{\mathrm{des}}=-k\,\mathrm{sat}\left(\frac{s}{\phi}\right).$
2. Approximate $\dot{s}(u)$ locally as an affine function:
   $\dot{s}(u)\approx a\,u+b.$
3. Estimate $a$ and $b$ numerically using two evaluations of the nominal dynamics:
   $a\approx \dot{s}(1)-\dot{s}(0),\qquad b\approx \dot{s}(0).$
4. Solve for the control:
   $u_{\mathrm{smc}}=\frac{\dot{s}_{\mathrm{des}}-b}{a}.$
5. Apply actuator saturation:
   $u=\mathrm{clamp}(u_{\mathrm{smc}}+u_{\mathrm{hold}},-u_{\max},u_{\max}).$

This is a common pattern in applied nonlinear control: *use model evaluations to avoid algebraic inversion*, while still enforcing a valid reaching law.

### 5) Cart-centering term with gating

To prevent long-term drift, a cart-centering PD term is added, but only when the pendulum is near upright.
The term:

$$
u_{\mathrm{hold}}=g(\theta)\left(-k_p x-k_d \dot{x}\right)
$$

uses a gating function $g(\theta)\in[0,1]$:

$$
g(\theta)=\mathrm{clamp}\left(1-\frac{|\theta|}{\theta_{\mathrm{gate}}},0,1\right).
$$

Effect:

- if $|\theta|$ is large → $g(\theta)\approx 0$ (prioritize balancing),
- if $|\theta|$ is small → $g(\theta)\approx 1$ (recenter the cart).

---

<a id="numerical-methods-used-in-this-repo"></a>

## Numerical methods used in this repo

### PDE solver: explicit finite differences (FTCS)

The heat solver is a canonical explicit diffusion integrator:

- space: central difference (2nd order),
- time: forward Euler (1st order).

Strengths:

- straightforward implementation,
- cheap per time step,
- easy to extend/parallelize later.

Limitations:

- explicit diffusion stability constraints may enforce small $\Delta t$ for fine grids,
- implicit methods (Crank–Nicolson, ADI) are preferable for stiff regimes (future extension).

### ODE solver: RK4 with sub-stepping

The cart–pole uses classical Runge–Kutta 4th order (RK4). To generate smooth animation frames:

- choose a frame step $\Delta t_{\mathrm{frame}}=\frac{T}{N_{\mathrm{frames}}}$,
- integrate using smaller physics steps:
  $\Delta t_{\mathrm{phys}}=\frac{\Delta t_{\mathrm{frame}}}{N_{\mathrm{substeps}}}.$

This keeps integration stable while preserving a deterministic “one frame → one PNG” pipeline.

---

<a id="plots-and-animations-jfreechart-java2d-ffmpeg"></a>

## Plots and animations (JFreeChart, Java2D/Swing, FFmpeg)

### JFreeChart (plots + heatmaps)

This repository uses <strong>JFreeChart</strong> for:

- time-series plots (position, angle, control, etc.),
- heatmap visualization (via <code>XYBlockRenderer</code> and an <code>XYZDataset</code>),
- exporting PNG images.

Key implementation details you may care about:

- axis tick formatting is forced to English digits via <code>Locale.US</code> + <code>DecimalFormatSymbols</code>,
- tick density is explicitly controlled via <code>NumberTickUnit</code>,
- plot fonts and strokes are increased for visibility on high-resolution canvases (e.g., 2400×1800),
- a PNG <code>pHYs</code> chunk is written so the images carry **300 DPI metadata**.

### Java2D/Swing (animation frames)

The inverted pendulum animation uses Java2D drawing primitives on a <code>BufferedImage</code>:

- cart, wheels, pole, bob, track, and disturbance arrow are drawn each frame,
- the frame is saved to <code>output/pendulum_sliding_mode/frames/</code> as <code>frame_%06d.png</code>,
- optional preview window (Swing) can be enabled with <code>--preview</code>.

### FFmpeg (optional MP4 encoding)

If FFmpeg is available on your PATH, the pendulum program automatically calls it using <code>ProcessBuilder</code>:

- input: PNG frames,
- codec: H.264 (<code>libx264</code>),
- pixel format: <code>yuv420p</code> (compatible with most players).

If FFmpeg is not installed, you still get all frames and plots, and you can encode manually (see OS guides below).

---

<a id="dependencies-and-installation"></a>

## Dependencies and installation

### Required

- Java Development Kit: <strong>JDK 17</strong> (or compatible; project targets Java 17 via Maven)
- Apache Maven: <strong>3.8+</strong> recommended
- Git (recommended)

### Included via Maven (no manual installation)

- JFreeChart (<code>org.jfree:jfreechart:1.5.6</code>)

### Optional (recommended)

- FFmpeg (only required for MP4 encoding; frames and plots are generated without it)

### How dependencies are managed

All Java dependencies are defined in <code>pom.xml</code>. Maven will download and cache them automatically:

- on Windows: typically under <code>%USERPROFILE%\.m2\repository</code>
- on macOS/Linux: under <code>~/.m2/repository</code>

---

<a id="building-the-project"></a>

## Building the project

From the repository root:

<pre><code>mvn -DskipTests clean package</code></pre>

What this does:

- cleans the <code>target/</code> folder,
- compiles with Java 17,
- packages a JAR under <code>target/</code>.

---

<a id="running-the-programs-and-generating-results"></a>

## Running the programs and generating results

The project uses Maven’s Exec plugin to run the two main classes.

### 1) Run Heat2D (PDE)

<pre><code>mvn -DskipTests exec:java -Dexec.mainClass="com.mohammadijoo.odepde.heat2d.Heat2DMain"</code></pre>

Outputs:

- <code>output/heat2d/heat_t*.png</code>
- <code>output/heat2d/centerline_final.png</code>
- <code>output/heat2d/center_point_vs_time.png</code>
- <code>output/heat2d/heat2d_log.csv</code>

### 2) Run Cart–Pole SMC (ODE + animation)

<pre><code>mvn -DskipTests exec:java -Dexec.mainClass="com.mohammadijoo.odepde.pendulum_smc.CartPoleSMCMain"</code></pre>

Outputs:

- frames: <code>output/pendulum_sliding_mode/frames/frame_*.png</code>
- plots: <code>output/pendulum_sliding_mode/*.png</code>
- CSV: <code>output/pendulum_sliding_mode/cartpole_log.csv</code>
- MP4 (if FFmpeg is installed): <code>output/pendulum_sliding_mode/pendulum_smc_10s_6000f.mp4</code>

### Useful runtime flags (pendulum)

The pendulum program supports optional flags:

<ul>
  <li><code>--preview</code> : show a Swing preview window while still saving frames</li>
  <li><code>--frames N</code> : override default frame count (default: 6000)</li>
  <li><code>--no-mp4</code> : skip FFmpeg encoding step</li>
</ul>

Example:

<pre><code>mvn -DskipTests exec:java -Dexec.mainClass="com.mohammadijoo.odepde.pendulum_smc.CartPoleSMCMain" -Dexec.args="--frames 3000 --no-mp4"</code></pre>

---

<a id="repository-file-guide"></a>

## Repository file guide (full explanation)

This section explains the key files and their roles.

### <code>src/main/java/com/mohammadijoo/odepde/heat2d/Heat2DMain.java</code> (PDE solver)

This file implements a complete 2D heat-diffusion workflow:

1) <strong>Grid + discretization</strong>
<ul>
  <li>Uniform grid: <code>nx</code>, <code>ny</code>, with <code>dx</code> and <code>dy</code>.</li>
  <li>Flattened storage: <code>double[] T</code> and <code>double[] Tnew</code>.</li>
  <li>Index helper: <code>idx(i,j,nx) = j*nx + i</code>.</li>
</ul>

2) <strong>Boundary conditions</strong>
<ul>
  <li>Dirichlet boundaries (fixed temperatures).</li>
  <li>Left boundary is “hot” (100); others are 0.</li>
  <li>Re-applied at every time step for correctness.</li>
</ul>

3) <strong>Stability-aware time step</strong>
<ul>
  <li>Computes the explicit FTCS stable limit <code>dtStable</code>.</li>
  <li>Uses <code>dt = 0.80 * dtStable</code> for margin.</li>
</ul>

4) <strong>Output strategy</strong>
<ul>
  <li>Uses <code>snapshotEvery</code> to throttle snapshots (avoid thousands of PNGs).</li>
  <li>Saves heatmap snapshots (high resolution) as the simulation progresses.</li>
  <li>Logs a center temperature trace and exports CSV.</li>
</ul>

5) <strong>Visualization details</strong>
<ul>
  <li>Heatmaps are produced using <code>DefaultXYZDataset</code> + <code>XYBlockRenderer</code>.</li>
  <li>Color mapping uses a custom <code>ThermalPaintScale</code> (blue → cyan → green → yellow → red).</li>
  <li>Plot fonts and strokes are intentionally increased for high-resolution exports.</li>
  <li>Tick labels are enforced in English using <code>Locale.US</code> formatting.</li>
  <li>PNG images include 300 DPI metadata via the PNG <code>pHYs</code> chunk.</li>
</ul>

### <code>src/main/java/com/mohammadijoo/odepde/pendulum_smc/CartPoleSMCMain.java</code> (ODE + SMC + animation)

This is the most comprehensive file in the repository: it includes plant modeling, control design, numerical integration, rendering, logging, plotting, and optional MP4 encoding.

Main components:

1) <strong>Plant model</strong>
<ul>
  <li><code>PoleModel</code> computes center of mass and inertia for a rod + bob.</li>
  <li><code>CartPoleParams</code> stores masses, gravity, and damping.</li>
  <li><code>dynamics(...)</code> computes derivatives given a control force and external torque.</li>
</ul>

2) <strong>Numerical integration</strong>
<ul>
  <li>RK4 is implemented in <code>rk4Step</code>.</li>
  <li>Sub-stepping (<code>substeps</code>) integrates multiple physics steps per video frame.</li>
  <li>Angle normalization via <code>wrapToPi</code>.</li>
</ul>

3) <strong>Controller (SMC)</strong>
<ul>
  <li><code>slidingSurface</code> defines the manifold: <code>s = thetaDot + lambdaTheta*theta + alpha*(xDot + lambdaX*x)</code>.</li>
  <li><code>computeControl</code> enforces a reaching law using a saturation-based boundary layer.</li>
  <li>Includes a gated cart-centering PD term (<code>uHold</code>) to prevent drift.</li>
  <li>Applies actuator saturation (<code>uMax</code>) to keep forces realistic.</li>
</ul>

4) <strong>Disturbances</strong>
<ul>
  <li><code>Disturbance</code> injects exactly two torque pulses at fixed times.</li>
  <li>Uses a half-sine profile to avoid discontinuities.</li>
  <li>Arrow visualization is enabled for a fixed duration after each pulse start.</li>
</ul>

5) <strong>Rendering pipeline (frames)</strong>
<ul>
  <li>Each frame is rendered into a <code>BufferedImage</code> using Java2D.</li>
  <li>Cart/pole geometry and arrow are drawn with anti-aliasing enabled.</li>
  <li>Frames are saved to disk as PNG using a deterministic filename scheme.</li>
</ul>

6) <strong>Plots and logs</strong>
<ul>
  <li>Time-series plots are generated with JFreeChart and saved as 300-DPI PNGs.</li>
  <li>Signals logged: <code>x</code>, <code>theta</code>, <code>u</code>, <code>tau_ext</code>, <code>F_eq</code>, <code>s</code>.</li>
  <li>CSV log is written to <code>output/pendulum_sliding_mode/cartpole_log.csv</code>.</li>
</ul>

7) <strong>MP4 encoding (optional)</strong>
<ul>
  <li>The program checks if <code>ffmpeg</code> is available on PATH.</li>
  <li>If available, it encodes frames into H.264 MP4 automatically.</li>
  <li>If not, you still have frames and can encode manually.</li>
</ul>

### <code>pom.xml</code> (build + dependencies)

Key responsibilities:

- configures project coordinates (<code>groupId</code>, <code>artifactId</code>, <code>version</code>),
- sets Java version via <code>&lt;maven.compiler.release&gt;17&lt;/maven.compiler.release&gt;</code>,
- declares plotting dependency:
  <ul>
    <li><code>org.jfree:jfreechart:1.5.6</code></li>
  </ul>
- includes Maven plugins:
  <ul>
    <li><code>maven-compiler-plugin</code> to compile with the configured release</li>
    <li><code>exec-maven-plugin</code> to run main classes from Maven</li>
  </ul>

---

<a id="operating-system-guides-windows-macos-linux"></a>

## Operating system guides (Windows / macOS / Linux)

### Windows (Maven + Git Bash / CMD + Visual Studio Code)

This repository is Maven-based, so the most reliable Windows setup is:

- install JDK 17,
- install Maven,
- run from Git Bash or Command Prompt,
- optionally use Visual Studio Code for editing/debugging.

#### Step 1) Install prerequisites

<ul>
  <li><strong>JDK 17</strong>: Install Temurin (Adoptium) or Oracle JDK, and verify <code>java -version</code> returns 17.x.</li>
  <li><strong>Maven</strong>: Install Apache Maven and verify <code>mvn -version</code>.</li>
  <li><strong>Git</strong>: Recommended (and provides Git Bash).</li>
  <li><strong>FFmpeg</strong> (optional): Needed only if you want MP4 output; verify <code>ffmpeg -version</code>.</li>
</ul>

#### Step 2) Clone the repo

<pre><code>git clone &lt;YOUR_REPO_URL&gt;
cd ode-pde-java</code></pre>

#### Step 3) Build (Windows CMD or Git Bash)

Use exactly these commands (as requested):

<pre><code>mvn -DskipTests clean package</code></pre>

#### Step 4) Run Heat2D

<pre><code>mvn -DskipTests exec:java -Dexec.mainClass="com.mohammadijoo.odepde.heat2d.Heat2DMain"</code></pre>

#### Step 5) Run Pendulum SMC

<pre><code>mvn -DskipTests exec:java -Dexec.mainClass="com.mohammadijoo.odepde.pendulum_smc.CartPoleSMCMain"</code></pre>

#### Visual Studio setup on Windows (recommended: Visual Studio Code)

The phrase “start from a blank solution” is typically associated with Visual Studio; however, Java development is most commonly done in
<strong>Visual Studio Code</strong>, IntelliJ IDEA, or Eclipse. The workflow below uses Visual Studio Code while still keeping the project as a Maven repo.

1) Install Visual Studio Code and Java tooling:
<ul>
  <li>Install the <strong>Extension Pack for Java</strong> (Microsoft).</li>
  <li>Install <strong>Maven for Java</strong> extension (if not included).</li>
</ul>

2) Open the repository:
<ul>
  <li>File → Open Folder → select the repository root (the folder containing <code>pom.xml</code>).</li>
</ul>

3) Maven import and dependency sync:
<ul>
  <li>VS Code will detect <code>pom.xml</code> and download dependencies automatically.</li>
  <li>If it does not, open the Command Palette and run: “Maven: Reload project”.</li>
</ul>

4) Create “two run configurations” (Heat2D and Pendulum):
<ul>
  <li>In the Explorer panel, expand “Maven” and locate the <code>exec:java</code> goals.</li>
  <li>Or create tasks in <code>.vscode/tasks.json</code> that call the exact Maven commands above.</li>
</ul>

5) Debugging (optional):
<ul>
  <li>You can run with the Java debugger by creating a launch configuration for each main class.</li>
  <li>Because the project is Maven-based, most users simply run via Maven and inspect outputs in <code>output/</code>.</li>
</ul>

---

### macOS (Homebrew)

1) Install prerequisites:

<pre><code>brew update
brew install openjdk@17 maven ffmpeg</code></pre>

2) Confirm versions:

<pre><code>java -version
mvn -version
ffmpeg -version</code></pre>

3) Build and run:

<pre><code>mvn -DskipTests clean package
mvn -DskipTests exec:java -Dexec.mainClass="com.mohammadijoo.odepde.heat2d.Heat2DMain"
mvn -DskipTests exec:java -Dexec.mainClass="com.mohammadijoo.odepde.pendulum_smc.CartPoleSMCMain"</code></pre>

Manual MP4 encoding (if you choose to encode yourself):

<pre><code>ffmpeg -y -framerate 600 -i output/pendulum_sliding_mode/frames/frame_%06d.png \
  -c:v libx264 -pix_fmt yuv420p output/pendulum_sliding_mode/pendulum_manual.mp4</code></pre>

---

### Linux (Ubuntu/Debian)

1) Install prerequisites:

<pre><code>sudo apt-get update
sudo apt-get install -y openjdk-17-jdk maven ffmpeg</code></pre>

2) Build and run:

<pre><code>mvn -DskipTests clean package
mvn -DskipTests exec:java -Dexec.mainClass="com.mohammadijoo.odepde.heat2d.Heat2DMain"
mvn -DskipTests exec:java -Dexec.mainClass="com.mohammadijoo.odepde.pendulum_smc.CartPoleSMCMain"</code></pre>

Manual MP4 encoding (optional):

<pre><code>ffmpeg -y -framerate 600 -i output/pendulum_sliding_mode/frames/frame_%06d.png \
  -c:v libx264 -pix_fmt yuv420p output/pendulum_sliding_mode/pendulum_manual.mp4</code></pre>

---

<a id="troubleshooting"></a>

## Troubleshooting

### Maven cannot find Java 17

- Verify <code>java -version</code> prints 17.x.
- Verify <code>mvn -version</code> shows Maven using the intended Java home.
- If Maven uses another Java version, set <code>JAVA_HOME</code> to your JDK 17 installation.

### Plots not generated or fonts look wrong

- JFreeChart renders using Java2D fonts. Ensure a standard font is available (the code uses <code>SansSerif</code>).
- If you run on a headless environment, consider using the JVM option:
  <ul><li><code>-Djava.awt.headless=true</code></li></ul>
  (Most PNG rendering paths still work fine, but preview windows will not.)

### MP4 not created

- MP4 generation happens only if <code>ffmpeg</code> is available on PATH.
- If auto-encoding is skipped, encode manually using the commands in the OS sections.

### Pendulum simulation is slow

- Writing 6000 PNG frames can be IO-heavy.
- Reduce the frame count:
  <ul><li><code>--frames 3000</code> (or lower)</li></ul>
- Ensure you run on an SSD and avoid network folders.

---

<a id="implementation-tutorial-video"></a>

## Implementation tutorial video

At the end of the workflow, you can watch the full implementation and walkthrough on YouTube.

<!-- Replace YOUR_VIDEO_ID with your actual video id. -->
<a href="https://www.youtube.com/watch?v=KUzxJggAjuc" target="_blank">
  <img
    src="https://i.ytimg.com/vi/KUzxJggAjuc/maxresdefault.jpg"
    alt="ODE/PDE in Java - Implementation Tutorial"
    style="max-width: 100%; border-radius: 10px; box-shadow: 0 6px 18px rgba(0,0,0,0.18); margin-top: 0.5rem;"
  />
</a>
