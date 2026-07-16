<div style="font-family: system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; line-height: 1.6;">

  <h1 align="center" style="margin-bottom: 0.2em;">ODE &amp; PDE Solvers in Python (2D Heat Equation + Inverted Pendulum SMC)</h1>

  <p style="font-size: 0.98rem; max-width: 920px; margin: 0.2rem auto 0;">
    A Python repository focused on <strong>numerical simulation</strong> and <strong>visualization</strong> of:
    <strong>partial differential equations (PDEs)</strong> and <strong>ordinary differential equations (ODEs)</strong>.
    The project includes two complete, end-to-end examples (each provided in a pure-Python baseline and a NumPy/SciPy-accelerated variant):
  </p>

  <ul style="max-width: 920px; margin: 0.5rem auto 0.2rem; padding-left: 1.2rem;">
    <li>
      <strong>PDE:</strong> 2D heat-conduction equation solved with an explicit finite-difference method (FTCS),
      generating <strong>heatmap snapshots</strong> and summary plots via <strong>Matplotlib</strong> (saved at <strong>dpi=300</strong>).
    </li>
    <li>
      <strong>ODE:</strong> a nonlinear cart–pole (inverted pendulum) system stabilized using
      <strong>Sliding Mode Control (SMC)</strong>, with high-FPS frame rendering via <strong>Pygame</strong>,
      plots via <strong>Matplotlib</strong>, and optional MP4 encoding using <strong>FFmpeg</strong>.
    </li>
  </ul>

  <p align="center" style="font-size: 1rem; color: #666; margin: 0.35rem auto 0;">
    Language: <strong>Python</strong> • Plotting: <strong>Matplotlib (Agg backend)</strong> •
    Numerics: <strong>NumPy</strong> • Optional integrator: <strong>SciPy</strong> •
    Frames: <strong>Pygame</strong> • Optional video: <strong>FFmpeg</strong>
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
  <li> <a href="#plots-and-animations-matplotlib-pygame-ffmpeg">Plots and animations (Matplotlib, Pygame, FFmpeg)</a></li>
  <li> <a href="#dependencies-and-installation">Dependencies and installation</a></li>
  <li> <a href="#creating-a-virtual-environment">Creating a virtual environment</a></li>
  <li> <a href="#running-the-scripts-and-generating-results">Running the scripts and generating results</a></li>
  <li> <a href="#repository-file-guide">Repository file guide (full explanation)</a></li>
  <li> <a href="#operating-system-guides-windows-macos-linux">Operating system guides (Windows / macOS / Linux)</a></li>
  <li> <a href="#troubleshooting">Troubleshooting</a></li>
  <li> <a href="#implementation-tutorial-video">Implementation tutorial video</a></li>
</ul>

---

<a id="about-this-repository"></a>

## About this repository

This repository is designed as a practical reference for implementing numerical solvers in Python with reproducible outputs:

- A <strong>PDE workflow</strong> that starts from a physical model (heat conduction), discretizes it, runs a time-marching solver,
  and produces high-quality plots and image snapshots.
- An <strong>ODE + control workflow</strong> that implements a nonlinear plant (cart–pole), injects disturbances, runs a robust controller
  (Sliding Mode Control), generates simulation plots, and produces a high-FPS frame sequence (and optionally a video).

Engineering requirements emphasized throughout the code:

- stable time stepping (CFL / explicit stability constraints),
- clean logging to CSV for reproducibility,
- deterministic frame generation for animations,
- high-quality figures (saved at <strong>dpi=300</strong>),
- headless-friendly plotting (Matplotlib <code>Agg</code> backend; no Tcl/Tk required).

### Quick map of scripts

- <code>src/heat2d/main.py</code> → baseline (pure Python lists) FTCS heat solver → outputs in <code>output/heat2d/</code>
- <code>src/heat2d/heat2d_numpy.py</code> → NumPy-vectorized FTCS heat solver → outputs in <code>output/heat2d_numpy/</code>
- <code>src/pendulum_sliding_mode/main.py</code> → baseline (dataclasses + RK4) cart–pole SMC with Pygame frames → outputs in <code>output/pendulum_sliding_mode/</code>
- <code>src/pendulum_sliding_mode/pendulum_smc_numpy_scipy.py</code> → NumPy state + optional SciPy integrator → outputs in <code>output/pendulum_sliding_mode_numpy_scipy/</code>

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
- $\mathbf{f}(\cdot)$ is a (possibly nonlinear) dynamics function.

In this repository, the cart–pole example is a nonlinear ODE system integrated in time using RK4 (and optionally <code>scipy.integrate.solve_ivp</code>).

### Partial Differential Equations (PDEs)

A PDE involves partial derivatives with respect to two or more independent variables (e.g., time and space). A canonical example is the
heat equation (diffusion equation):

$$
\frac{\partial T}{\partial t} = \alpha \left( \frac{\partial^2 T}{\partial x^2} + \frac{\partial^2 T}{\partial y^2} \right),
$$

where:

- $T(x,y,t)$ is the temperature field,
- $\alpha$ is thermal diffusivity,
- $(x,y)$ are spatial coordinates,
- $t$ is time.

In this repository, the heat equation is discretized in space using finite differences and integrated forward in time using an explicit FTCS method.

---

<a id="example-1-pde-2d-heat-equation"></a>

## Example 1 (PDE): 2D Heat Equation

### Physical model

The PDE solved in <code>src/heat2d/main.py</code> and <code>src/heat2d/heat2d_numpy.py</code> is:

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

Both heat solvers implement the same numerical method:

- uniform grid in $x$ and $y$,
- second-order central differences for $\partial^2 T / \partial x^2$ and $\partial^2 T / \partial y^2$,
- explicit time stepping (Forward Euler in time).

At interior grid nodes $(i,j)$:

$$
T^{n+1}_{i,j} = T^{n}_{i,j} + \alpha\,\Delta t\left(
\frac{T^n_{i+1,j}-2T^n_{i,j}+T^n_{i-1,j}}{\Delta x^2}
+
\frac{T^n_{i,j+1}-2T^n_{i,j}+T^n_{i,j-1}}{\Delta y^2}
\right),
$$

with fixed boundary values applied at every step.

### Stability constraint (explicit diffusion)

The explicit 2D diffusion scheme requires a stability constraint on $\Delta t$. A commonly used sufficient condition is:

$$
\Delta t \le \frac{1}{2\alpha \left(\frac{1}{\Delta x^2} + \frac{1}{\Delta y^2}\right)}.
$$

Each script computes a stability-limited time step and uses a conservative factor (<code>0.80</code>) to remain safely within the stable region.

### What you get as outputs

When you run either heat script, you will find:

- multiple heatmap snapshots: <code>heat_t*.png</code>
- final centerline profile: <code>centerline_final.png</code>
- center temperature vs time: <code>center_point_vs_time.png</code>
- CSV log: <code>heat2d_log.csv</code>

Output folders:

- <code>src/heat2d/main.py</code> → <code>output/heat2d/</code>
- <code>src/heat2d/heat2d_numpy.py</code> → <code>output/heat2d_numpy/</code>

This combination is typical for PDE workflows:

- images for qualitative verification,
- plots for engineering interpretation,
- CSV for reproducibility and post-processing in Python/Matlab/Excel.

---

<a id="example-2-ode-inverted-pendulum-with-smc"></a>

## Example 2 (ODE): Inverted Pendulum with Sliding Mode Control

The second example (<code>src/pendulum_sliding_mode/main.py</code> and <code>src/pendulum_sliding_mode/pendulum_smc_numpy_scipy.py</code>)
simulates a nonlinear cart–pole system with disturbances and stabilizes it using Sliding Mode Control (SMC).

### State variables and conventions

The state is:

$$
\mathbf{x} = \begin{bmatrix} x & \dot{x} & \theta & \dot{\theta} \end{bmatrix}^T
$$

- $x$ is the cart position (meters),
- $\theta$ is the pole angle (radians) with $\theta=0$ being upright,
- sign convention is chosen such that $\theta>0$ visually leans the pole to the right.

Both scripts implement:

- angle wrapping into $[-\pi,\pi]$ to prevent drift and keep plots readable,
- clamping/saturation for actuator limits and sliding-mode boundary layer.

### Disturbances (exactly two)

Two disturbance pulses are applied as an <strong>external torque about the pole pivot</strong>:

- pulse 1: starts at $t=0.5\,s$, positive torque (“push right”)
- pulse 2: starts at $t=5.0\,s$, negative torque (“push left”)

Each pulse uses a smooth half-sine profile over a 0.5 second duration, and the on-screen arrow is shown for 0.5 seconds only.

Disturbance torque model:

$$
\tau_{ext}(t) =
\begin{cases}
+\tau_{amp}\sin\left(\pi\frac{t-t_1}{d}\right) & t\in[t_1,t_1+d] \\
-\tau_{amp}\sin\left(\pi\frac{t-t_2}{d}\right) & t\in[t_2,t_2+d] \\
0 & \text{otherwise}
\end{cases}
$$

where $d=0.5\,s$ is the pulse duration.

For visualization and logging, the scripts also compute an “equivalent bob force”:

$$
F_{eq}(t) = \frac{\tau_{ext}(t)}{L},
$$

used only to define arrow direction and to provide an intuitive disturbance magnitude signal.

### Simulation and rendering pipeline

This example is intentionally “complete” and end-to-end:

- integrate the nonlinear dynamics (RK4 fixed-step; optionally SciPy <code>solve_ivp</code>)
- render each frame using the current integrated state (Pygame offscreen surface)
- save frames to disk
- optionally encode a video using FFmpeg
- generate plots (Matplotlib, <strong>dpi=300</strong>)
- save a CSV log for analysis

Outputs include:

- <code>output/.../frames/frame_000000.png</code> ... <code>frame_005999.png</code>
- <code>output/.../cartpole_log.csv</code>
- multiple plots: cart position, pole angle, control force, disturbance torque, sliding surface, etc.
- MP4 video (if FFmpeg is available on <code>PATH</code>)

---

<a id="sliding-mode-control-theory"></a>

## Sliding Mode Control (SMC) theory

Sliding Mode Control is a robust nonlinear control method designed to enforce a chosen “sliding manifold” (sliding surface) in the state space, even in the presence of disturbances and certain model uncertainties.

### 1) Sliding surface design

For a general second-order system, a common sliding surface is:

$$
s = \dot{e} + \lambda e,
$$

where $e$ is the tracking error. Enforcing $s \to 0$ yields stable error dynamics under suitable conditions.

In this repository, stabilization is framed as regulating the pole to upright ($\theta \to 0$) while also preventing the cart from drifting. The implemented sliding surface is:

$$
s(\mathbf{x}) = \dot{\theta} + \lambda_{\theta}\,\theta + \alpha\left(\dot{x} + \lambda_x\,x\right).
$$

Interpretation:

- $\lambda_{\theta}$ defines how strongly the controller penalizes angle error,
- $\alpha$ couples cart motion to the sliding surface (cart motion is required to “catch” the pendulum),
- $\lambda_x$ introduces a cart-position component that discourages drift.

### 2) Reaching condition and stability intuition

A classical sufficient condition for reaching and maintaining the sliding manifold is the Lyapunov inequality:

$$
\frac{d}{dt}\left(\frac{1}{2}s^2\right) = s\dot{s} \le -\eta |s|,
$$

with $\eta > 0$. This implies $|s|$ decreases and reaches zero in finite time.

A control objective consistent with this is:

$$
\dot{s} = -k\,\mathrm{sign}(s),
$$

where $k>0$. The discontinuous control action yields strong robustness against matched disturbances (disturbances entering through the same channel as the control input).

### 3) Boundary layer to reduce chattering

The ideal sign function can cause chattering (high-frequency switching) in practical systems. A standard mitigation is a continuous saturation function:

$$
\mathrm{sat}(z) =
\begin{cases}
-1 & z < -1 \\
z  & |z| \le 1 \\
+1 & z > 1
\end{cases}
$$

and a boundary-layer reaching law:

$$
\dot{s} = -k\,\mathrm{sat}\left(\frac{s}{\phi}\right),
$$

where $\phi>0$ defines the boundary layer thickness around $s=0$.

This repository implements exactly this approach using:

- <code>k</code> (gain)
- <code>phi</code> (boundary layer)

### 4) How the scripts compute the control input

The cart–pole dynamics are nonlinear and do not provide a simple closed-form affine relation between cart force $u$ and $\dot{s}$. The controller therefore uses a pragmatic numerical affine approximation:

1. Define the desired sliding surface derivative:  
   $$\dot{s}_{des} = -k\,\mathrm{sat}\left(\frac{s}{\phi}\right).$$

2. Approximate $\dot{s}(u)$ locally as:  
   $$\dot{s}(u) \approx a\,u + b.$$

3. Estimate $a$ and $b$ numerically from two evaluations of the nominal dynamics (disturbance ignored in the control law):  
   $$a \approx \dot{s}(1) - \dot{s}(0), \quad b \approx \dot{s}(0).$$

4. Solve for the sliding-mode control action:  
   $$u_{smc} = \frac{\dot{s}_{des} - b}{a}.$$

5. Add a small cart-centering term and saturate the actuator:  
   $$u = \mathrm{clamp}(u_{smc} + u_{hold}, -u_{max}, u_{max}).$$

### 5) Cart-centering term with gating

To prevent cart drift without fighting the catch maneuver, the scripts include a <em>gated</em> centering term:

$$
u_{hold} = g(\theta)\left(-k_p x - k_d \dot{x}\right),
$$

where:

- $g(\theta)\in[0,1]$ is near 1 only when the pole is close to upright,
- $g(\theta)$ is near 0 when $|\theta|$ is large.

A simple gate used in the code is:

$$
g(\theta)=\mathrm{clamp}\left(1-\frac{|\theta|}{\theta_{gate}},0,1\right).
$$

Result: the controller prioritizes stabilization first, then recenters the cart when it is safe.

---

<a id="numerical-methods-used-in-this-repo"></a>

## Numerical methods used in this repo

### PDE solver: explicit finite differences (FTCS)

The heat solver is a standard explicit diffusion integrator:

- Space: central difference (2nd order)
- Time: forward Euler (1st order)

Strengths:

- straightforward implementation,
- low per-step computational cost,
- easy to vectorize (as shown in the NumPy version).

Limitations:

- stability constraint forces small $\Delta t$ as the grid is refined (diffusion stiffness).

A natural extension is an implicit method (e.g., Crank–Nicolson or ADI) to remove/relax the explicit stability restriction.

### ODE solver: RK4 with sub-stepping (and optional SciPy)

The cart–pole scripts generate exactly <strong>6000 frames</strong> for <strong>10 seconds</strong> of simulation (600 FPS). To keep the dynamics stable and physically plausible, both scripts integrate with multiple <em>substeps per frame</em>:

- define frame step: $\Delta t_{frame} = T / N_{frames}$
- integrate physics with smaller step: $\Delta t_{physics} = \Delta t_{frame} / N_{substeps}$

The NumPy/SciPy script also provides an optional integration path using:

- <code>scipy.integrate.solve_ivp</code> evaluated on the same physics time grid

This is useful if you want to experiment with adaptive stepping while still producing deterministic per-frame outputs.

---

<a id="plots-and-animations-matplotlib-pygame-ffmpeg"></a>

## Plots and animations (Matplotlib, Pygame, FFmpeg)

### Matplotlib (plots and heatmaps)

This repository uses Matplotlib for:

- heatmap snapshots of the temperature field,
- time histories (center temperature, cart position, pole angle, control input, etc.),
- diagnostic plots (sliding surface, disturbance signals).

All figures are saved at <strong>dpi=300</strong>.

To avoid GUI backends and Tcl/Tk issues, scripts force:

- <code>matplotlib.use("Agg")</code>

This enables plotting on headless servers and minimal Python installations.

### Pygame (frame rendering)

The cart–pole animation is rendered with Pygame:

- frames are drawn to an offscreen surface,
- each frame is saved as a PNG,
- an optional <code>--preview</code> flag opens a live preview window while still saving frames.

### FFmpeg (optional) for MP4 encoding

If FFmpeg is installed and available in <code>PATH</code>, the scripts attempt to encode an MP4 automatically.

If FFmpeg is not available, you still get PNG frames and can encode manually (commands provided in OS-specific sections).

---

<a id="dependencies-and-installation"></a>

## Dependencies and installation

### Required

- Python (recommended: 3.10+)
- pip
- A working C/C++ build toolchain may be needed on some systems for binary wheels (less common today, but still possible in constrained environments).

### Python dependencies (managed via <code>requirements.txt</code>)

- <code>matplotlib</code> → plots and heatmaps (saved at dpi=300)
- <code>pygame</code> → frame rendering for the cart–pole simulation
- <code>numpy</code> → fast arrays (vectorized PDE update; vectorized ODE state)
- <code>scipy</code> → optional ODE integration backend (<code>solve_ivp</code>) used only when selected

### Optional (recommended for video)

- FFmpeg (required only if you want automatic MP4 creation)

---

<a id="creating-a-virtual-environment"></a>

## Creating a virtual environment

Using a virtual environment keeps dependencies isolated and reproducible.

### Windows (PowerShell)

<pre><code>python -m venv .venv
.\.venv\Scripts\Activate.ps1

python -m pip install --upgrade pip
pip install -r requirements.txt</code></pre>

If PowerShell blocks activation, you can allow it for the current user:

<pre><code>Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser</code></pre>

### Windows (CMD)

<pre><code>python -m venv .venv
.\.venv\Scripts\activate.bat

python -m pip install --upgrade pip
pip install -r requirements.txt</code></pre>

### macOS / Linux

<pre><code>python3 -m venv .venv
source .venv/bin/activate

python -m pip install --upgrade pip
pip install -r requirements.txt</code></pre>

---

<a id="running-the-scripts-and-generating-results"></a>

## Running the scripts and generating results

All scripts should be run from the repository root so output folders are created consistently.

### 1) Run the heat equation solvers (PDE)

Baseline version:

<pre><code>python src/heat2d/main.py</code></pre>

NumPy-accelerated version:

<pre><code>python src/heat2d/heat2d_numpy.py</code></pre>

Outputs:

- <code>output/heat2d/</code> and <code>output/heat2d_numpy/</code> respectively

### 2) Run the cart–pole SMC simulations (ODE + animation)

Baseline version (RK4, Pygame frames):

<pre><code>python src/pendulum_sliding_mode/main.py</code></pre>

Optional live preview (still saves frames):

<pre><code>python src/pendulum_sliding_mode/main.py --preview</code></pre>

NumPy + optional SciPy version:

<pre><code># fixed-step RK4 on a deterministic time grid (default)
python src/pendulum_sliding_mode/pendulum_smc_numpy_scipy.py --integrator rk4

# SciPy solve_ivp evaluated on the same physics grid
python src/pendulum_sliding_mode/pendulum_smc_numpy_scipy.py --integrator scipy</code></pre>

Optional live preview:

<pre><code>python src/pendulum_sliding_mode/pendulum_smc_numpy_scipy.py --preview --integrator rk4</code></pre>

Outputs:

- <code>output/pendulum_sliding_mode/</code> and <code>output/pendulum_sliding_mode_numpy_scipy/</code> respectively

---

<a id="repository-file-guide"></a>

## Repository file guide (full explanation)

This section explains every important file in the repository and its role, with special attention to the four main scripts.

### <code>requirements.txt</code>

Contains pinned minimum versions for the Python runtime dependencies:

<ul>
  <li><strong>matplotlib</strong>: plot generation and heatmap snapshots (saved at dpi=300; <code>Agg</code> backend for headless execution)</li>
  <li><strong>pygame</strong>: frame rendering and saving PNG sequences for the cart–pole animation</li>
  <li><strong>numpy</strong>: fast numerical arrays (vectorized PDE update; vector-based ODE state)</li>
  <li><strong>scipy</strong>: optional integration backend (<code>solve_ivp</code>) for the NumPy/SciPy pendulum script</li>
</ul>

### <code>src/heat2d/main.py</code> (PDE heat solver, baseline)

This script is a readable baseline implementation of a 2D explicit FTCS heat solver:

<ul>
  <li>Defines a uniform 2D grid on a unit square domain.</li>
  <li>Imposes Dirichlet boundary conditions (left edge hot, other edges cold).</li>
  <li>Computes a conservative stable time step using the explicit diffusion constraint.</li>
  <li>Performs time stepping by updating interior points and reapplying boundary values each step.</li>
  <li>Writes periodic heatmap snapshots and summary plots at dpi=300.</li>
  <li>Logs the center temperature vs time and writes it to <code>heat2d_log.csv</code>.</li>
</ul>

Implementation notes:

<ul>
  <li>The temperature field is stored as a flattened 1D list for explicit indexing via <code>idx(i,j,nx)</code>.</li>
  <li>Matplotlib is configured to use <code>Agg</code>, so the script runs without GUI dependencies.</li>
</ul>

### <code>src/heat2d/heat2d_numpy.py</code> (PDE heat solver, NumPy)

This script implements the same FTCS method but accelerates the interior update using NumPy slicing:

<ul>
  <li>Stores temperature as a 2D <code>numpy.ndarray</code> shaped <code>(ny, nx)</code>.</li>
  <li>Computes the discrete Laplacian on the interior with array slices, avoiding Python loops.</li>
  <li>Produces the same class of outputs: snapshots, summary plots, and CSV log.</li>
</ul>

When to use it:

<ul>
  <li>Prefer this version when you increase grid resolution (larger <code>nx</code>, <code>ny</code>) and want significantly better runtime.</li>
</ul>

### <code>src/pendulum_sliding_mode/main.py</code> (ODE + SMC + frames, baseline)

This script is an end-to-end simulation pipeline:

<ul>
  <li><strong>Plant model</strong>: cart mass + rod/bob pole model, with derived COM and inertia.</li>
  <li><strong>Disturbances</strong>: exactly two half-sine torque pulses at 0.5s and 5.0s.</li>
  <li><strong>Controller</strong>: Sliding Mode Control with boundary layer and a gated cart-centering term.</li>
  <li><strong>Integration</strong>: RK4 with multiple substeps per frame for stability at 600 FPS.</li>
  <li><strong>Rendering</strong>: Pygame draws cart, wheels, pole, bob, and a constant-length disturbance arrow.</li>
  <li><strong>Outputs</strong>: 6000 PNG frames, optional MP4 via FFmpeg, dpi=300 plots, and a CSV log.</li>
</ul>

Practical notes:

<ul>
  <li>In non-preview mode the script sets <code>SDL_VIDEODRIVER=dummy</code> to allow headless rendering on servers.</li>
  <li>Actuator saturation avoids unrealistic control forces; track limits prevent the cart leaving the visual range.</li>
</ul>

### <code>src/pendulum_sliding_mode/pendulum_smc_numpy_scipy.py</code> (ODE + SMC, NumPy + optional SciPy)

This script uses a NumPy state vector and provides two integration backends:

<ul>
  <li><strong>Fixed-step RK4</strong> on an exact physics grid (deterministic, matches frame/substep timing).</li>
  <li><strong>SciPy solve_ivp</strong> (optional) evaluated at the same grid points; enabled via <code>--integrator scipy</code>.</li>
</ul>

It also generates the same outputs as the baseline version (frames, optional MP4, plots, CSV), but is structured to make it easier to:

<ul>
  <li>swap integrators,</li>
  <li>experiment with array-based computations,</li>
  <li>extend to parameter sweeps and batch simulations.</li>
</ul>

---

<a id="operating-system-guides-windows-macos-linux"></a>

## Operating system guides (Windows / macOS / Linux)

### Windows

1) Create venv and install:

<pre><code>python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt</code></pre>

2) Run:

<pre><code>python src\heat2d\main.py
python src\heat2d\heat2d_numpy.py

python src\pendulum_sliding_mode\main.py
python src\pendulum_sliding_mode\pendulum_smc_numpy_scipy.py --integrator rk4</code></pre>

3) Optional FFmpeg (MP4 output):
<ul>
  <li>Install FFmpeg and ensure <code>ffmpeg</code> works from a new terminal (PATH configured).</li>
</ul>

Manual encoding (if you want to encode from frames yourself):

<pre><code>ffmpeg -y -framerate 600 -i output/pendulum_sliding_mode/frames/frame_%06d.png ^
  -c:v libx264 -pix_fmt yuv420p output/pendulum_sliding_mode/pendulum.mp4</code></pre>

### macOS

1) Create venv and install:

<pre><code>python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt</code></pre>

2) Run:

<pre><code>python src/heat2d/main.py
python src/heat2d/heat2d_numpy.py

python src/pendulum_sliding_mode/main.py
python src/pendulum_sliding_mode/pendulum_smc_numpy_scipy.py --integrator rk4</code></pre>

3) Optional FFmpeg:
<ul>
  <li>Install FFmpeg using Homebrew if desired: <code>brew install ffmpeg</code></li>
</ul>

Manual encoding:

<pre><code>ffmpeg -y -framerate 600 -i output/pendulum_sliding_mode/frames/frame_%06d.png \
  -c:v libx264 -pix_fmt yuv420p output/pendulum_sliding_mode/pendulum.mp4</code></pre>

### Linux (Ubuntu/Debian)

1) Create venv and install:

<pre><code>python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt</code></pre>

2) Run:

<pre><code>python src/heat2d/main.py
python src/heat2d/heat2d_numpy.py

python src/pendulum_sliding_mode/main.py
python src/pendulum_sliding_mode/pendulum_smc_numpy_scipy.py --integrator rk4</code></pre>

3) Optional FFmpeg:
<ul>
  <li>Install FFmpeg: <code>sudo apt-get update &amp;&amp; sudo apt-get install -y ffmpeg</code></li>
</ul>

Manual encoding:

<pre><code>ffmpeg -y -framerate 600 -i output/pendulum_sliding_mode/frames/frame_%06d.png \
  -c:v libx264 -pix_fmt yuv420p output/pendulum_sliding_mode/pendulum.mp4</code></pre>

---

<a id="troubleshooting"></a>

## Troubleshooting

### Matplotlib errors related to Tk/Tcl (Windows)

These scripts force a non-GUI backend:

- <code>matplotlib.use("Agg")</code>

So plotting should not require Tcl/Tk. If you still encounter GUI-backend issues, ensure you are not forcing a different backend via environment variables or local Matplotlib config.

### FFmpeg not found / MP4 not created

- The scripts only create an MP4 if <code>ffmpeg</code> is available on <code>PATH</code>.
- If MP4 creation fails, you can always encode manually using the commands in the OS sections.

### The pendulum simulation is slow

- Writing 6000 PNG frames can be IO-bound.
- Use a fast SSD if possible.
- Running without <code>--preview</code> reduces overhead.

### Pygame headless issues (Linux servers)

- The scripts set <code>SDL_VIDEODRIVER=dummy</code> when <code>--preview</code> is not used.
- If your environment requires a different dummy driver configuration, set it before running:
  <code>export SDL_VIDEODRIVER=dummy</code>.

---

<a id="implementation-tutorial-video"></a>

## Implementation tutorial video

At the end of the workflow, you can watch the full implementation and walkthrough on YouTube.

<!-- Replace YOUR_VIDEO_ID with your real YouTube video ID -->
<a href="https://www.youtube.com/watch?v=XvfKpfUC5vI" target="_blank">
  <img
    src="https://i.ytimg.com/vi/XvfKpfUC5vI/maxresdefault.jpg"
    alt="ODE/PDE in Python - Implementation Tutorial"
    style="max-width: 100%; border-radius: 10px; box-shadow: 0 6px 18px rgba(0,0,0,0.18); margin-top: 0.5rem;"
  />
</a>
