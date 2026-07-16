<div style="font-family: system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; line-height: 1.6;">

  <h1 align="center" style="margin-bottom: 0.2em;">ODE &amp; PDE Solvers in C++ (Heat Equation + Inverted Pendulum SMC)</h1>

  <p style="font-size: 0.98rem; max-width: 920px; margin: 0.2rem auto 0;">
    A C++17 repository focused on <strong>numerical simulation</strong> and <strong>visualization</strong> of:
    <strong>partial differential equations (PDEs)</strong> and <strong>ordinary differential equations (ODEs)</strong>.
    The project includes two complete, end-to-end examples:
  </p>

  <ul style="max-width: 920px; margin: 0.5rem auto 0.2rem; padding-left: 1.2rem;">
    <li>
      <strong>PDE:</strong> 2D heat-conduction equation solved with an explicit finite-difference method (FTCS),
      generating <strong>heatmap snapshots</strong> and summary plots via <strong>Matplot++</strong>.
    </li>
    <li>
      <strong>ODE:</strong> a nonlinear cart–pole (inverted pendulum) system stabilized using
      <strong>Sliding Mode Control (SMC)</strong>, with high-FPS frame rendering via <strong>SFML</strong>,
      plots via <strong>Matplot++</strong>, and optional MP4 encoding using <strong>FFmpeg</strong>.
    </li>
  </ul>

  <p align="center" style="font-size: 1rem; color: #666; margin: 0.35rem auto 0;">
    Build system: <strong>CMake</strong> • Plotting: <strong>Matplot++</strong> + <strong>Gnuplot</strong> •
    Animation: <strong>SFML</strong> • Optional video encoding: <strong>FFmpeg</strong>
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
  <li> <a href="#plots-and-animations-matplot-sfml-ffmpeg">Plots and animations (Matplot++, SFML, FFmpeg)</a></li>
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

This repository is designed as a practical reference for implementing numerical solvers in modern C++ (C++17):

- A <strong>PDE workflow</strong> that starts from a physical model (heat conduction), discretizes it, runs a time-marching solver,
  and produces professional plots and image snapshots.
- An <strong>ODE + control workflow</strong> that implements a nonlinear plant (cart–pole), injects disturbances, runs a robust controller
  (Sliding Mode Control), generates simulation plots, and produces an animation/video.

The goal is not just “making it work,” but also illustrating common engineering requirements:

- stable time stepping (CFL / explicit stability constraints),
- clean logging to CSV for reproducibility,
- plot generation for analysis,
- animations for communication and debugging,
- portable builds via CMake and CI.

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

The PDE solved in <code>src/heat2d/main.cpp</code> is:

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

The second example (<code>src/pendulum_sliding_mode/main.cpp</code>) simulates a nonlinear cart–pole system with disturbances and stabilizes it using Sliding Mode Control (SMC).

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
- Render each frame using the current integrated state (SFML offscreen rendering)
- Save frames to disk
- Optionally encode a video using FFmpeg
- Generate Matplot++ plots
- Save a CSV log for analysis

Outputs include:

- <code>output/pendulum_sliding_mode/frames/frame_000000.png</code> ... etc.
- <code>output/pendulum_sliding_mode/cartpole_log.csv</code>
- multiple plots: cart position, pole angle, control force, disturbance torque, sliding surface, etc.
- MP4 video (if FFmpeg is available)

<strong>Important note about frame count:</strong> the header comment in the file mentions “6000 frames,” but the current code sets <code>total_frames = 1800</code> (which corresponds to 180 FPS for a 10s animation). The output MP4 filename still contains <code>6000f</code> in its name. If you want 6000 frames / 600 FPS exactly, update:
<ul>
  <li><code>const int total_frames = 6000;</code></li>
  <li>and (optionally) rename the MP4 output filename accordingly.</li>
</ul>


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

where $k &gt; 0$ is a gain. This creates a discontinuous control action that strongly rejects matched disturbances (disturbances entering through the same channel as the control).

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
- easy to parallelize later.

Limitations:

- stability constraint can force small $\Delta t$ for fine grids,
- accuracy is limited by explicit time stepping for stiff diffusion regimes.

A natural extension (not yet implemented here) is an implicit method (e.g., Crank–Nicolson or ADI) to remove/relax stability constraints.

### ODE solver: RK4 with sub-stepping

The cart–pole uses classical Runge–Kutta 4th order (RK4). The simulation renders frames at high FPS, so the code uses <em>substeps per frame</em>:

- choose a frame time-step $\Delta t_{frame}$ based on desired animation duration and frame count,
- integrate dynamics with smaller steps $\Delta t_{physics} = \Delta t_{frame}/N_{substeps}$.

This is a common workflow for physically plausible animations:

- stable dynamics integration,
- deterministic frame output,
- tight coupling between simulation and visualization.


---

<a id="plots-and-animations-matplot-sfml-ffmpeg"></a>

## Plots and animations (Matplot++, SFML, FFmpeg)

### Matplot++ and Gnuplot (for plots and image snapshots)

This repository uses <a href="https://github.com/alandefreitas/matplotplusplus" target="_blank">Matplot++</a> for plotting.
Matplot++ typically uses <strong>Gnuplot</strong> as a backend, so Gnuplot must be installed and available on your <code>PATH</code> at runtime.

Matplot++ is fetched automatically by CMake (FetchContent), so you do not need to clone it manually.

Typical outputs generated with Matplot++ in this repo:

- line plots (time histories, cross-sections),
- heatmaps saved as PNG images,
- diagnostic plots (sliding surface, disturbance signals, control signals).

### SFML (for animation frames)

The inverted pendulum example uses <a href="https://www.sfml-dev.org/" target="_blank">SFML</a> for rendering:

- an offscreen <code>sf::RenderTexture</code> is used to draw each frame,
- frames are saved as PNG images,
- optional “preview window” can be enabled via <code>--preview</code>.

### FFmpeg (optional) for MP4 encoding

If FFmpeg is installed and available in <code>PATH</code>, the pendulum program will attempt to encode an MP4 automatically.

If FFmpeg is not available, the program still generates PNG frames, and you can encode the video manually (see OS-specific sections).


---

<a id="dependencies-and-installation"></a>

## Dependencies and installation

### Required

- C++17 compiler (MSVC, Clang, or GCC)
- CMake 3.20+
- Gnuplot (required at runtime for Matplot++ rendering)
- SFML (required for the pendulum renderer)

### Optional (recommended for video)

- FFmpeg (required only if you want MP4 encoding)

### How dependencies are managed in this repo

- Matplot++ is pulled automatically using CMake FetchContent (see <code>CMakeLists.txt</code>).
- SFML is discovered via CMake’s <code>find_package(SFML ... CONFIG REQUIRED)</code>:
  - on Linux, CI installs <code>libsfml-dev</code> (system package)
  - on Windows, the most reliable approach is vcpkg
  - on macOS, Homebrew is convenient


---

<a id="building-the-project"></a>

## Building the project

The repository uses an out-of-source CMake build. A typical build looks like this:

<pre><code>cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j</code></pre>

Executable outputs are placed under <code>build/bin/</code> (depending on generator/configuration).


---

<a id="running-the-executables-and-generating-results"></a>

## Running the executables and generating results

### 1) Run the heat equation solver (PDE)

From the build output folder:

<pre><code># Linux/macOS
./build/bin/heat2d

# Windows (PowerShell)
.\build\bin\heat2d.exe</code></pre>

Results appear in:

- <code>output/heat2d/</code>

### 2) Run the pendulum SMC simulation (ODE + animation)

<pre><code># Linux/macOS
./build/bin/pendulum_sliding_mode

# Windows (PowerShell)
.\build\bin\pendulum_sliding_mode.exe</code></pre>

Optional preview window (still saves frames):

<pre><code>./build/bin/pendulum_sliding_mode --preview</code></pre>

Results appear in:

- <code>output/pendulum_sliding_mode/</code>
- frames in <code>output/pendulum_sliding_mode/frames/</code>


---

<a id="repository-file-guide"></a>

## Repository file guide (full explanation)

This section explains every important file in the repository and its role.

### <code>CMakeLists.txt</code>

Key responsibilities:

- sets C++17 standard and warning flags,
- fetches Matplot++ automatically via FetchContent:
  - repository: matplotplusplus
  - tag: <code>v1.2.2</code>
- finds SFML via CMake config mode (<code>find_package(SFML ... CONFIG REQUIRED)</code>),
- defines two executables:
  - <code>heat2d</code> from <code>src/heat2d/main.cpp</code>
  - <code>pendulum_sliding_mode</code> from <code>src/pendulum_sliding_mode/main.cpp</code>
- links required libraries:
  - Matplot++ for both executables
  - SFML + Matplot++ for the pendulum executable
- sets <code>CMAKE_RUNTIME_OUTPUT_DIRECTORY</code> to <code>build/bin</code> for predictable outputs
- adds a diagnostic compile definition:
  - <code>TRACE_GNUPLOT_COMMANDS</code> (helpful when debugging Matplot++/Gnuplot)

### <code>src/heat2d/main.cpp</code> (PDE solver)

This is the full PDE pipeline:

- defines a 2D domain with uniform grid (<code>nx</code>, <code>ny</code>),
- applies Dirichlet boundary conditions (fixed temperatures),
- computes a stable explicit time-step constraint and chooses a conservative <code>dt</code>,
- iterates over time steps:
  - updates interior nodes via FTCS
  - reapplies boundary conditions
  - periodically saves heatmap snapshots with Matplot++
- logs the center temperature over time and saves:
  - a centerline plot at final time
  - a time-history plot
  - a CSV log (<code>heat2d_log.csv</code>)

Engineering notes that are easy to miss but important:

- the 2D grid is flattened into a 1D vector for speed and cache efficiency,
- <code>idx(i,j,nx)</code> encodes 2D indexing,
- snapshots are intentionally throttled (<code>snapshot_every</code>) to avoid producing thousands of PNG files,
- plotting uses <code>figure(true)</code> which creates a quiet/offscreen figure suitable for saving images without opening a GUI window.

### <code>src/pendulum_sliding_mode/main.cpp</code> (ODE + SMC + animation)

This is the most comprehensive file in the repository: it contains modeling, control, integration, rendering, logging, plotting, and optional video encoding.

Main components:

1) <strong>Plant model (cart–pole):</strong>
<ul>
  <li><code>PoleModel</code> computes center-of-mass and inertia parameters for a rod + bob mass.</li>
  <li><code>CartPoleParams</code> stores masses, gravity, and damping.</li>
  <li><code>dynamics(...)</code> computes the state derivatives given control force and external torque.</li>
</ul>

2) <strong>Numerical integration:</strong>
<ul>
  <li>RK4 is used via <code>rk4_step(...)</code>.</li>
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
  <li>SFML <code>RenderTexture</code> draws cart, pole, bob, and arrows into an offscreen buffer.</li>
  <li>Frames are saved as PNG images.</li>
  <li>Optional <code>--preview</code> creates a window for interactive observation.</li>
</ul>

6) <strong>Plots and CSV logging:</strong>
<ul>
  <li>Matplot++ saves time histories for position, angle, control input, disturbance, and sliding surface.</li>
  <li>CSV is written for offline analysis.</li>
</ul>

7) <strong>FFmpeg encoding:</strong>
<ul>
  <li>The program attempts to call FFmpeg automatically if it is present on PATH.</li>
  <li>Manual encoding is also supported (see OS sections).</li>
</ul>

### <code>include/common/csv_writer.hpp</code>

A small, explicit CSV writer utility, suitable for logging simulation signals:

- validates column sizes,
- writes a header row,
- writes rows in a loop.

Even though the two <code>main.cpp</code> files currently implement local CSV writers, this header provides a reusable implementation if you want to refactor the examples to use a single common logger.

### <code>include/common/clamp.hpp</code>

A minimal helper function:

- clamps a value between <code>lo</code> and <code>hi</code>,
- increases readability in simulation loops and controller code.

The pendulum example currently includes its own clamp implementation, but this header exists so you can standardize utility functions across examples.

### <code>.github/workflows/ci.yml</code>

GitHub Actions workflow that builds the project on Ubuntu:

- checks out the repository,
- installs dependencies using <code>apt</code>:
  - <code>libsfml-dev</code>
  - <code>gnuplot</code>
- configures the project in Release mode
- builds using CMake

This provides baseline CI coverage and ensures the core build stays healthy on Linux.


---

<a id="operating-system-guides-windows-macos-linux"></a>

## Operating system guides (Windows / macOS / Linux)

### Windows (recommended: Visual Studio + vcpkg)

1) Install prerequisites:
<ul>
  <li>Visual Studio 2022 (Desktop development with C++)</li>
  <li>CMake (or use the version bundled with Visual Studio)</li>
  <li>Gnuplot (for Matplot++ runtime)</li>
  <li>(Optional) FFmpeg (for MP4 encoding)</li>
</ul>

2) Install vcpkg and SFML:
<pre><code># PowerShell (example path)
git clone https://github.com/microsoft/vcpkg.git C:\dev\vcpkg
cd C:\dev\vcpkg
.\bootstrap-vcpkg.bat

# Install SFML (choose triplet as needed)
.\vcpkg.exe install sfml</code></pre>

3) Configure and build with the vcpkg toolchain:
<pre><code>cmake -S . -B build -G "Visual Studio 17 2022" -A x64 `
  -DCMAKE_TOOLCHAIN_FILE="C:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake"
cmake --build build --config Release</code></pre>

4) Run:
<pre><code>.\build\bin\Release\heat2d.exe
.\build\bin\Release\pendulum_sliding_mode.exe</code></pre>

5) Install Gnuplot and FFmpeg:
<ul>
  <li>Ensure both <code>gnuplot</code> and <code>ffmpeg</code> commands work in a fresh terminal (PATH set correctly).</li>
</ul>

### macOS (Homebrew)

1) Install dependencies:
<pre><code>brew update
brew install cmake gnuplot sfml ffmpeg</code></pre>

2) Build:
<pre><code>cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j</code></pre>

3) Run:
<pre><code>./build/bin/heat2d
./build/bin/pendulum_sliding_mode</code></pre>

4) Manual MP4 encoding (if the program does not auto-encode):
<pre><code>ffmpeg -y -framerate 180 -i output/pendulum_sliding_mode/frames/frame_%06d.png \
  -c:v libx264 -pix_fmt yuv420p output/pendulum_sliding_mode/pendulum.mp4</code></pre>

### Linux (Ubuntu/Debian)

1) Install dependencies:
<pre><code>sudo apt-get update
sudo apt-get install -y cmake g++ gnuplot libsfml-dev ffmpeg</code></pre>

2) Build:
<pre><code>cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j</code></pre>

3) Run:
<pre><code>./build/bin/heat2d
./build/bin/pendulum_sliding_mode</code></pre>

4) Manual MP4 encoding (if needed):
<pre><code>ffmpeg -y -framerate 180 -i output/pendulum_sliding_mode/frames/frame_%06d.png \
  -c:v libx264 -pix_fmt yuv420p output/pendulum_sliding_mode/pendulum.mp4</code></pre>


---

<a id="troubleshooting"></a>

## Troubleshooting

### Matplot++ does not produce images (or says it cannot run gnuplot)

- Confirm <code>gnuplot --version</code> works in the same terminal where you run the executable.
- If you installed Gnuplot but it is not in PATH, add it and restart your terminal.

### SFML is not found by CMake

- On Windows, confirm you configured CMake with the vcpkg toolchain file.
- On macOS, confirm <code>brew install sfml</code> and that CMake can see Homebrew’s prefix.
- On Linux, confirm <code>libsfml-dev</code> is installed.

### MP4 not created

- The program only creates an MP4 if <code>ffmpeg</code> is available on PATH.
- If auto-encoding fails, encode manually using the provided commands in the OS sections.

### The simulation is slow / frames take too long to generate

- The pendulum example saves many PNGs; disk IO can dominate runtime.
- Reduce <code>total_frames</code> (or increase) depending on your desired quality vs time.
- Consider using a faster SSD and running in Release mode.


---

<a id="implementation-tutorial-video"></a>

## Implementation tutorial video

At the end of the workflow, you can watch the full implementation and walkthrough on YouTube.

<!-- Replace YOUR_VIDEO_ID and YOUR_VIDEO_URL with your real link. -->
<a href="https://www.youtube.com/watch?v=K-gh_15FszQ" target="_blank">
  <img
    src="https://i.ytimg.com/vi/K-gh_15FszQ/maxresdefault.jpg"
    alt="ODE/PDE in C++ - Implementation Tutorial"
    style="max-width: 100%; border-radius: 10px; box-shadow: 0 6px 18px rgba(0,0,0,0.18); margin-top: 0.5rem;"
  />
</a>


