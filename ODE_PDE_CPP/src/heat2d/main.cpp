// ------------------------------------------------------------
// 2D Heat Equation (Heat Transfer by Conduction) in C++
// ------------------------------------------------------------
// PDE:
//   ∂T/∂t = α ( ∂²T/∂x² + ∂²T/∂y² )
//
// Method:
//   - 2D explicit finite-difference scheme (FTCS)
//   - Dirichlet boundary conditions (fixed temperature on boundaries)
//   - Snapshots saved as PNG heatmaps using Matplot++
//   - Additional plots saved (center point vs time, final centerline)
//
// Output folder (relative to where you run the program):
//   output/heat2d/
//
// Notes:
//   - Matplot++ needs gnuplot installed and available in PATH at runtime.
// ------------------------------------------------------------

#define _USE_MATH_DEFINES
#include <cmath>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <matplot/matplot.h>

// A small helper to flatten 2D indexing (i, j) into a 1D array index.
// nx is the number of columns in x-direction.
static std::size_t idx(std::size_t i, std::size_t j, std::size_t nx)
{
    return j * nx + i;
}

// Minimal CSV writer (kept local to keep this file self-contained).
static void write_csv_two_columns(
    const std::string& filename,
    const std::string& h1,
    const std::string& h2,
    const std::vector<double>& c1,
    const std::vector<double>& c2)
{
    // Basic safety checks: both columns must have the same number of rows.
    if (c1.size() != c2.size())
    {
        throw std::runtime_error("CSV write error: column sizes do not match.");
    }

    std::ofstream out(filename);
    if (!out)
    {
        throw std::runtime_error("CSV write error: cannot open file: " + filename);
    }

    // Write header
    out << h1 << "," << h2 << "\n";

    // Write data rows
    for (std::size_t k = 0; k < c1.size(); ++k)
    {
        out << c1[k] << "," << c2[k] << "\n";
    }
}

int main()
{
    // ------------------------------------------------------------
    // Output directory setup
    // ------------------------------------------------------------
    const std::string out_dir = "output/heat2d";
    std::filesystem::create_directories(out_dir);

    // ------------------------------------------------------------
    // Physical parameters
    // ------------------------------------------------------------
    const double alpha = 1.0; // thermal diffusivity (chosen for demonstrative speed)

    // ------------------------------------------------------------
    // Domain and grid
    // ------------------------------------------------------------
    const double Lx = 1.0;
    const double Ly = 1.0;

    // Grid resolution (increase for smoother but slower simulations)
    const std::size_t nx = 81;
    const std::size_t ny = 81;

    const double dx = Lx / (nx - 1);
    const double dy = Ly / (ny - 1);

    // ------------------------------------------------------------
    // Time step selection (explicit stability)
    // ------------------------------------------------------------
    // For the 2D explicit heat equation (FTCS), a standard stability constraint is:
    //
    //   dt <= 1 / ( 2*α*( 1/dx^2 + 1/dy^2 ) )
    //
    // We take a conservative fraction of this limit.
    const double dt_stable = 1.0 / (2.0 * alpha * (1.0 / (dx * dx) + 1.0 / (dy * dy)));
    const double dt = 0.80 * dt_stable; // conservative choice

    const double t_end = 0.20; // total simulation time (seconds)
    const std::size_t nsteps = static_cast<std::size_t>(std::ceil(t_end / dt));

    // Save a heatmap snapshot every N steps
    // We target a reasonable number of snapshots so we don't generate thousands of PNG files.
	const std::size_t desired_snapshots = 30;

	// snapshot_every = how many timesteps between saved frames.
	// Ensure it is at least 1 to avoid division-by-zero issues.
	const std::size_t snapshot_every = std::max<std::size_t>(1, nsteps / desired_snapshots);


    // ------------------------------------------------------------
    // Allocate temperature fields
    // ------------------------------------------------------------
    // We store T as a flattened 2D field of size nx*ny.
    std::vector<double> T(nx * ny, 0.0);
    std::vector<double> T_new(nx * ny, 0.0);

    // ------------------------------------------------------------
    // Boundary conditions (Dirichlet)
    // ------------------------------------------------------------
    // Left edge is hot; other edges cold.
    const double T_left = 100.0;
    const double T_right = 0.0;
    const double T_top = 0.0;
    const double T_bottom = 0.0;

    // Apply boundary values to initial condition
    for (std::size_t j = 0; j < ny; ++j)
    {
        T[idx(0, j, nx)] = T_left;
        T[idx(nx - 1, j, nx)] = T_right;
    }
    for (std::size_t i = 0; i < nx; ++i)
    {
        T[idx(i, 0, nx)] = T_bottom;
        T[idx(i, ny - 1, nx)] = T_top;
    }

    // ------------------------------------------------------------
    // Logging (temperature at center over time)
    // ------------------------------------------------------------
    const std::size_t ic = nx / 2;
    const std::size_t jc = ny / 2;

    std::vector<double> time_log;
    std::vector<double> center_T_log;

    // ------------------------------------------------------------
    // Time integration loop
    // ------------------------------------------------------------
    double t = 0.0;

    for (std::size_t step = 0; step < nsteps; ++step)
    {
        // Log center temperature
        time_log.push_back(t);
        center_T_log.push_back(T[idx(ic, jc, nx)]);

        // Update only the interior nodes (boundaries remain fixed)
        for (std::size_t j = 1; j < ny - 1; ++j)
        {
            for (std::size_t i = 1; i < nx - 1; ++i)
            {
                // Second derivative in x (central difference)
                const double Txx =
                    (T[idx(i + 1, j, nx)] - 2.0 * T[idx(i, j, nx)] + T[idx(i - 1, j, nx)]) / (dx * dx);

                // Second derivative in y (central difference)
                const double Tyy =
                    (T[idx(i, j + 1, nx)] - 2.0 * T[idx(i, j, nx)] + T[idx(i, j - 1, nx)]) / (dy * dy);

                // Explicit FTCS update
                T_new[idx(i, j, nx)] = T[idx(i, j, nx)] + alpha * dt * (Txx + Tyy);
            }
        }

        // Re-apply Dirichlet boundaries to T_new so they remain fixed exactly
        for (std::size_t j = 0; j < ny; ++j)
        {
            T_new[idx(0, j, nx)] = T_left;
            T_new[idx(nx - 1, j, nx)] = T_right;
        }
        for (std::size_t i = 0; i < nx; ++i)
        {
            T_new[idx(i, 0, nx)] = T_bottom;
            T_new[idx(i, ny - 1, nx)] = T_top;
        }

        // Swap buffers for the next iteration
        std::swap(T, T_new);

        // --------------------------------------------------------
        // Save a heatmap snapshot occasionally
        // --------------------------------------------------------
        if (step % snapshot_every == 0)
        {
            // Convert flattened field to a 2D matrix for Matplot++ heatmap
            std::vector<std::vector<double>> M(ny, std::vector<double>(nx, 0.0));
            for (std::size_t y = 0; y < ny; ++y)
            {
                for (std::size_t x = 0; x < nx; ++x)
                {
                    M[y][x] = T[idx(x, y, nx)];
                }
            }

            // Create a "quiet" figure (no GUI window) and save to PNG
            auto fig = matplot::figure(true);

            matplot::heatmap(M);
            matplot::title("2D Heat Equation - Temperature Field");
            matplot::xlabel("x index");
            matplot::ylabel("y index");

            const std::string png_name = out_dir + "/heat_t" + std::to_string(step) + ".png";
            fig->save(png_name);

            std::cout << "Saved snapshot: " << png_name << "\n";
        }

        // Advance time
        t += dt;
    }

    // ------------------------------------------------------------
    // Final plots: centerline temperature and center point vs time
    // ------------------------------------------------------------

    // Create x-axis values (physical coordinates) and centerline values
    std::vector<double> x(nx, 0.0);
    std::vector<double> centerline(nx, 0.0);
    for (std::size_t i = 0; i < nx; ++i)
    {
        x[i] = i * dx;
        centerline[i] = T[idx(i, jc, nx)];
    }

    // Plot final centerline temperature
    {
        auto fig = matplot::figure(true);
        matplot::plot(x, centerline);
        matplot::title("Final Centerline Temperature (y = 0.5)");
        matplot::xlabel("x (m)");
        matplot::ylabel("T(x, y=0.5)");
        fig->save(out_dir + "/centerline_final.png");
    }

    // Plot center point temperature vs time
    {
        auto fig = matplot::figure(true);
        matplot::plot(time_log, center_T_log);
        matplot::title("Temperature at Plate Center vs Time");
        matplot::xlabel("time (s)");
        matplot::ylabel("T(center)");
        fig->save(out_dir + "/center_point_vs_time.png");
    }

    // ------------------------------------------------------------
    // Save CSV log of center temperature
    // ------------------------------------------------------------
    try
    {
        write_csv_two_columns(out_dir + "/heat2d_log.csv", "t", "T_center", time_log, center_T_log);
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << "\n";
    }

    std::cout << "Heat2D finished. Results are in: " << out_dir << "\n";
    return 0;
}
