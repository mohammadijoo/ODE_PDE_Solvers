cmake_minimum_required(VERSION 3.20)

project(ODE_PDE_CPP LANGUAGES CXX)

# ------------------------------------------------------------
# Global C++ settings
# ------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Put executables in a predictable location.
# For multi-config generators (Visual Studio), $<CONFIG> is "Debug" or "Release".
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ------------------------------------------------------------
# Matplot++ (downloaded automatically using FetchContent)
# ------------------------------------------------------------
include(FetchContent)

# Disable Matplot++ examples/tests to keep the build minimal.
set(MATPLOT_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(MATPLOT_BUILD_TESTS    OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
  matplotplusplus
  GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus.git
  GIT_TAG v1.2.2
)
FetchContent_MakeAvailable(matplotplusplus)

# ------------------------------------------------------------
# SFML (installed via vcpkg, CONFIG mode)
# NOTE: We do NOT request version 2.5 because vcpkg currently provides SFML 3.x.
# ------------------------------------------------------------
find_package(SFML COMPONENTS Graphics Window System CONFIG REQUIRED)

# ------------------------------------------------------------
# Compiler warnings (optional but useful)
# ------------------------------------------------------------
add_library(project_warnings INTERFACE)
target_compile_options(project_warnings INTERFACE
  $<$<CXX_COMPILER_ID:MSVC>:/W4>
  $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>
)

# ------------------------------------------------------------
# Executables
# ------------------------------------------------------------
add_executable(heat2d
  src/heat2d/main.cpp
)
target_link_libraries(heat2d PRIVATE matplot project_warnings)

add_executable(pendulum_sliding_mode
  src/pendulum_sliding_mode/main.cpp
)
target_link_libraries(pendulum_sliding_mode PRIVATE SFML::Graphics SFML::Window SFML::System matplot project_warnings)
target_compile_definitions(heat2d PRIVATE TRACE_GNUPLOT_COMMANDS)
target_compile_definitions(pendulum_sliding_mode PRIVATE TRACE_GNUPLOT_COMMANDS)
