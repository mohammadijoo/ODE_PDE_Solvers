# ODE & PDE Solvers — Two Numerical Experiments in Nine Languages

A cross-language scientific-computing collection that implements the same two numerical simulation workflows in **C#**, **C++**, **Java**, **Python**, **Julia**, **Go**, **Scala**, **Ruby**, and **Rust**:

1. a **2D heat-conduction PDE** solved with an explicit finite-difference method; and
2. a nonlinear **cart–pole ODE system** stabilized using **Sliding Mode Control (SMC)**.

Each implementation preserves the same broad mathematical model, numerical workflow, output strategy, and educational goals while using the build tools, plotting libraries, rendering APIs, and programming conventions native to its ecosystem.

> This repository is a comparative engineering and educational collection. It is not a general-purpose ODE/PDE solver package and is not intended for safety-critical control deployment.

---

## Why This Monorepo Exists

The nine implementations were originally developed as separate repositories. Because they solve the same two engineering problems, collecting them in one repository makes it easier to:

- clone every implementation with one command;
- compare numerical algorithms across languages;
- study differences in syntax, type systems, memory models, and project structure;
- compare plotting, image-generation, CSV, and video pipelines;
- run each implementation independently;
- maintain shared conceptual documentation in one place;
- extend the same experiments into additional languages.

The projects are **conceptually equivalent**, but they are not expected to be line-by-line translations. Floating-point behavior, plotting defaults, image dimensions, dependency APIs, and some implementation details differ.

---

## Implementations

| Language | Project folder | Build/package system | Plotting and frame rendering |
|---|---|---|---|
| C# | [`ODE_PDE_Csharp`](./ODE_PDE_Csharp/) | .NET 8 / Visual Studio solution | ScottPlot 5, System.Drawing, WinForms |
| C++ | [`ODE_PDE_CPP`](./ODE_PDE_CPP/) | CMake, C++17 | Matplot++/Gnuplot, SFML |
| Java | [`ODE_PDE_Java`](./ODE_PDE_Java/) | Maven, Java 17 | JFreeChart, Java2D, Swing |
| Python | [`ODE_PDE_Python`](./ODE_PDE_Python/) | `pip` / virtual environment | Matplotlib, Pygame, NumPy, optional SciPy |
| Julia | [`ODE_PDE_Julia`](./ODE_PDE_Julia/) | Julia Pkg | Plots.jl, Luxor.jl |
| Go | [`ODE_PDE_Go`](./ODE_PDE_Go/) | Go modules | Gonum Plot, standard image packages |
| Scala | [`ODE_PDE_Scala`](./ODE_PDE_Scala/) | Maven, Scala 2.13 | JFreeChart, Java2D, optional Swing preview |
| Ruby | [`ODE_PDE_Ruby`](./ODE_PDE_Ruby/) | Bundler, Ruby 3+ | Gnuplot, ChunkyPNG |
| Rust | [`ODE_PDE_Rust`](./ODE_PDE_Rust/) | Cargo, Rust 2021 | Plotters, image, imageproc, minifb |

**FFmpeg is optional** and is used to encode generated frame sequences into MP4 video where supported.

### Platform note

Most implementations are designed around portable language toolchains, but their external dependencies differ. In particular, the C# projects currently target `net8.0-windows` and use WinForms/System.Drawing. Consult the README inside each language folder for current operating-system requirements.

---

## Repository Structure

```text
ODE_PDE_Solvers/
├── README.md
├── ODE_PDE_Csharp/
│   ├── ODE_PDE_Csharp.sln
│   ├── Heat2D/
│   └── PendulumSlidingMode/
├── ODE_PDE_CPP/
│   ├── CMakeLists.txt
│   └── src/
│       ├── heat2d/
│       └── pendulum_sliding_mode/
├── ODE_PDE_Java/
│   ├── pom.xml
│   └── src/main/java/
├── ODE_PDE_Python/
│   ├── requirements.txt
│   └── src/
│       ├── heat2d/
│       └── pendulum_sliding_mode/
├── ODE_PDE_Julia/
│   ├── Project.toml
│   └── src/
│       ├── heat2d/
│       └── pendulum_sliding_mode/
├── ODE_PDE_Go/
│   ├── go.mod
│   └── src/
│       ├── heat2d/
│       └── pendulum_sliding_mode/
├── ODE_PDE_Scala/
│   ├── pom.xml
│   └── src/main/scala/
├── ODE_PDE_Ruby/
│   ├── Gemfile
│   ├── lib/
│   └── src/
│       ├── heat2d/
│       └── pendulum_sliding_mode/
└── ODE_PDE_Rust/
    ├── Cargo.toml
    └── src/
        ├── heat2d/
        └── pendulum_sliding_mode/
```

The root repository acts as an index and comparison guide. Each language directory remains a self-contained project with its own detailed setup, build, execution, troubleshooting, and file documentation.

---

## Clone and Select an Implementation

```bash
git clone https://github.com/abolfazl-mohammadijoo/ODE_PDE_Solvers.git
cd ODE_PDE_Solvers
```

Enter the language project you want to run:

```bash
cd ODE_PDE_Rust
```

Then follow that folder's `README.md`.

The implementations do not depend on one another. You can install, build, and execute one language project without building the remaining eight.

---

## Typical Entry Points

| Language | Heat-equation entry point | Cart–pole SMC entry point |
|---|---|---|
| C# | `Heat2D/Program.cs` | `PendulumSlidingMode/Program.cs` |
| C++ | `src/heat2d/main.cpp` | `src/pendulum_sliding_mode/main.cpp` |
| Java | `Heat2DMain.java` | `CartPoleSMCMain.java` |
| Python | `src/heat2d/main.py` | `src/pendulum_sliding_mode/main.py` |
| Julia | `src/heat2d/main.jl` | `src/pendulum_sliding_mode/main.jl` |
| Go | `src/heat2d/main.go` | `src/pendulum_sliding_mode/main.go` |
| Scala | `Heat2DApp.scala` | `CartPoleSmcApp.scala` |
| Ruby | `src/heat2d/main.rb` | `src/pendulum_sliding_mode/main.rb` |
| Rust | Cargo binary `heat2d` | Cargo binary `pendulum_sliding_mode` |

Python also provides accelerated alternatives using NumPy and optional SciPy:

```text
src/heat2d/heat2d_numpy.py
src/pendulum_sliding_mode/pendulum_smc_numpy_scipy.py
```

---

# Numerical Experiment 1: 2D Heat Equation

## Physical Model

The first experiment solves the two-dimensional heat or diffusion equation:

```math
\frac{\partial T}{\partial t}
=
\alpha
\left(
\frac{\partial^2 T}{\partial x^2}
+
\frac{\partial^2 T}{\partial y^2}
\right)
```

The modeled system is a square plate with constant isotropic diffusivity, no internal heat source, and fixed-temperature Dirichlet boundaries.

```text
left boundary   = 100
right boundary  = 0
top boundary    = 0
bottom boundary = 0
grid size       = 81 × 81
```

This creates a diffusion front that propagates from the left boundary into the plate.

## Spatial and Temporal Discretization

The implementations use a uniform Cartesian grid, second-order central differences in space, Forward Euler integration in time, and an explicit FTCS update:

```math
T_{i,j}^{n+1}
=
T_{i,j}^{n}
+
\alpha \Delta t
\left[
\frac{T_{i+1,j}^{n}-2T_{i,j}^{n}+T_{i-1,j}^{n}}{\Delta x^2}
+
\frac{T_{i,j+1}^{n}-2T_{i,j}^{n}+T_{i,j-1}^{n}}{\Delta y^2}
\right]
```

Boundary values are re-applied after every update.

Several implementations store the field in a flattened one-dimensional array:

```text
index(i,j) = j × nx + i
```

Julia uses a native two-dimensional matrix while preserving the same numerical update.

## Explicit Stability Constraint

The FTCS diffusion scheme is conditionally stable:

```math
\Delta t_{\mathrm{stable}}
=
\frac{1}
{2\alpha
\left(
\frac{1}{\Delta x^2}
+
\frac{1}{\Delta y^2}
\right)}
```

The implementations apply a conservative safety factor:

```math
\Delta t = 0.80\,\Delta t_{\mathrm{stable}}
```

Choosing a time step without respecting this limit can make the numerical solution oscillate or diverge.

## Heat-Solver Outputs

```text
output/heat2d/
├── heat_t*.png
├── centerline_final.png
├── center_point_vs_time.png
└── heat2d_log.csv
```

The implementations usually limit the number of heatmap snapshots to approximately 30 rather than saving every time step.

---

# Numerical Experiment 2: Cart–Pole with Sliding Mode Control

## Nonlinear ODE System

The state vector is:

```math
\mathbf{x}
=
\begin{bmatrix}
x &
\dot{x} &
\theta &
\dot{\theta}
\end{bmatrix}^{T}
```

`x` is the cart position and `θ = 0` represents the upright pole. The plant includes nonlinear trigonometric coupling, cart and pole damping, actuator saturation, and an external disturbance torque.

## Disturbance Schedule

```text
first disturbance  starts at t = 0.5 s
second disturbance starts at t = 5.0 s
pulse duration                = 0.5 s
```

A half-sine profile makes each pulse rise and return to zero smoothly. An equivalent force is used for visualization and logging:

```math
F_{\mathrm{eq}}(t)
=
\frac{\tau_{\mathrm{ext}}(t)}{L}
```

The displayed arrow is a direction indicator with fixed visual length rather than a literal force-vector scale.

## Runge–Kutta Integration

The nonlinear state equations are primarily integrated using classical RK4:

```math
\mathbf{x}_{n+1}
=
\mathbf{x}_{n}
+
\frac{\Delta t}{6}
\left(
\mathbf{k}_{1}
+
2\mathbf{k}_{2}
+
2\mathbf{k}_{3}
+
\mathbf{k}_{4}
\right)
```

Sub-stepping is used where required so the physics time step can remain finer than the rendering interval. Python additionally offers an optional SciPy integration path.

## Sliding Surface

```math
s
=
\dot{\theta}
+
\lambda_{\theta}\theta
+
\alpha_{s}
\left(
\dot{x}
+
\lambda_{x}x
\right)
```

The desired sliding dynamics use a boundary-layer saturation function:

```math
\dot{s}_{\mathrm{desired}}
=
-k\,\mathrm{sat}
\left(
\frac{s}{\phi}
\right)
```

This reduces numerical chattering compared with an ideal discontinuous sign function.

The implementations also include actuator saturation, angle wrapping into `[-π, π]`, a gated cart-centering term near upright, and track-position limits.

## End-to-End Simulation Pipeline

1. Evaluate the disturbance schedule.
2. Calculate the sliding surface.
3. Compute the saturated SMC input.
4. Integrate the nonlinear dynamics.
5. Enforce angle and track limits.
6. Log states, disturbances, and control signals.
7. Render a frame from the integrated state.
8. Save analysis plots and CSV data.
9. Optionally invoke FFmpeg.

A typical default animation is 10 seconds long and may contain 6000 generated frames.

## Cart–Pole Outputs

```text
output/pendulum_sliding_mode/
├── frames/
│   ├── frame_000000.png
│   ├── frame_000001.png
│   └── ...
├── cartpole_log.csv
├── cart_position.png
├── pole_angle.png
├── control_force.png
├── disturbance_torque.png
├── equivalent_force.png
├── sliding_surface.png
└── pendulum_smc_10s_6000f.mp4
```

Exact plot names and output locations may differ slightly among implementations.

---
# Language-Specific Characteristics

## C#

- Two .NET executable projects in one Visual Studio solution.
- Uses ScottPlot 5 for engineering plots.
- Uses System.Drawing and WinForms-compatible types for image work.
- Currently targets `net8.0-windows`.

## C++

- Builds `heat2d` and `pendulum_sliding_mode` with CMake.
- Fetches Matplot++ for plots.
- Uses SFML for cart–pole frame rendering.
- Requires an installed SFML package and Gnuplot runtime.

## Java

- Uses Java 17 and Maven.
- Uses JFreeChart for heatmaps and time-series plots.
- Uses Java2D/Swing for rendering and optional display.
- Writes high-resolution images with DPI metadata.

## Python

- Includes readable pure-Python baseline implementations.
- Includes NumPy-accelerated and optional SciPy variants.
- Uses Matplotlib's headless-friendly `Agg` backend.
- Uses Pygame for cart–pole frames.

## Julia

- Uses a project environment through `Project.toml`.
- Uses Plots.jl with the GR backend.
- Uses Luxor.jl for frame drawing.
- Uses native matrix operations for the heat field.

## Go

- Uses Go modules and Gonum Plot.
- Uses standard image and PNG facilities for offscreen rendering.
- Keeps the executable workflow simple and toolchain-native.

## Scala

- Uses Scala 2.13 with Maven.
- Uses JFreeChart and Java2D through JVM interoperability.
- Includes consistent English numeric labels in generated plots.

## Ruby

- Uses Ruby 3+ and Bundler.
- Uses Gnuplot as the plotting engine.
- Uses ChunkyPNG for pure-Ruby frame generation.
- Keeps shared CSV and Gnuplot helpers under `lib/`.

## Rust

- Defines two independent Cargo binary targets.
- Uses Plotters for static charts and heatmaps.
- Uses image/imageproc for frame generation.
- Uses release-profile optimization and explicit error handling.

---

# What This Repository Lets You Compare

Because the mathematical tasks are kept similar, the collection is useful for comparing:

- array and matrix representations;
- explicit nested loops versus vectorized calculations;
- numerical type systems and floating-point handling;
- mutable versus immutable data patterns;
- error and exception handling;
- package managers and build systems;
- CSV generation;
- static plotting libraries;
- offscreen raster rendering;
- FFmpeg process integration;
- ODE integration architecture;
- controller implementation patterns;
- native, JVM, .NET, scripting, and systems-language ecosystems.

Generated plots and trajectories should be qualitatively similar, but they may not be pixel-identical or numerically bit-for-bit identical.

---

# Running Projects Independently

This repository is a collection of nine separate applications, not one executable that mixes nine languages.

Every language folder should:

- contain its own dependency manifest;
- contain its own source tree;
- generate its own output directory;
- use relative paths inside that project;
- avoid dependencies on another language folder;
- provide its own detailed README.

There is intentionally no root command that installs every compiler, runtime, plotting backend, and native dependency.

---

# Scope and Limitations

The repository currently demonstrates only:

- one PDE: the two-dimensional heat equation;
- one nonlinear ODE/control system: the cart–pole;
- one PDE discretization family: explicit FTCS;
- one primary ODE integrator: fixed-step RK4;
- one robust-control approach: boundary-layer sliding-mode control.

It does not currently provide:

- an abstract reusable solver API;
- adaptive PDE time stepping;
- implicit heat solvers;
- finite-element or finite-volume methods;
- general boundary-condition objects;
- automatic stiffness detection;
- general event handling;
- formal convergence-order studies;
- automated cross-language numerical regression tests;
- hardware-in-the-loop or physical-robot control;
- validated safety or certification evidence.

The projects are intended for learning, comparison, visualization, and extension.

---

# Suggested Cross-Language Validation

A useful validation workflow is:

1. use the same physical parameters and grid size;
2. record the stable time step in each heat implementation;
3. compare center temperature at selected times;
4. compare final centerline profiles;
5. use the same cart–pole initial state and disturbances;
6. compare pole-angle recovery and maximum cart displacement;
7. compare actuator saturation and sliding-surface behavior;
8. export CSV logs and calculate error metrics with an independent analysis script.

Future automated tests could compare selected scalar outputs using language-independent tolerances.

---

# Adding Another Language

A new implementation should use a self-contained folder:

```text
ODE_PDE_<Language>/
```

It should preferably preserve the common experiment contract.

## Heat equation

- `81 × 81` grid or a clearly documented alternative;
- identical boundary values;
- explicit FTCS update;
- calculated stability-limited time step;
- `0.80` safety factor;
- heatmaps, summary plots, and CSV output.

## Cart–pole

- the same state convention;
- nonlinear dynamics;
- RK4 integration;
- the same two disturbance times;
- sliding-mode control with boundary-layer saturation;
- state and control CSV logging;
- plots and frame rendering;
- optional FFmpeg encoding.

Intentional deviations should be documented in the language-specific README.

---

## Project Goal

**Two numerical experiments, nine languages, nine ecosystems, and one place to compare scientific-computing implementations.**
