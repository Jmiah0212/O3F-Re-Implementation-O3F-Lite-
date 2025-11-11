# O3F-Lite (CSCE 4430)

A minimal C++/SFML re-implementation inspired by the IEEE IROS 2023 paper "Object-Oriented Option Framework for Robotics Manipulation in Clutter (O3F)".

This demo implements a lightweight, testable MVP of the Object-Oriented Option Framework: a 2D grid environment, a tabular Q-learning option planner, deterministic option executors (hand-crafted policies), an SFML visualizer and simple logging for experiments.

Key features
- 2D discrete grid environment with robot, target and obstacles
- Option hierarchy: high-level options (MoveToTarget, ClearObstacle, GraspTarget) chosen by a Q-learning planner
- Deterministic option execution via local path heuristics (executor) and environment primitives
- Reward shaping for progress, obstacle penalties and a large success bonus
- CSV training logs and a small Python plotting helper

## Requirements
- CMake 3.20+
- A C++17-capable toolchain (MSVC or MinGW-w64)
- SFML 2.5+ (Graphics)
- Optional: Python 3 with pandas & matplotlib to plot logs

## Build (Windows, recommended: Visual Studio + vcpkg)
1. Install Visual Studio 2019/2022 (Desktop development with C++) and CMake.
2. Install vcpkg and SFML via vcpkg:

```powershell
.\vcpkg\vcpkg.exe install sfml:x64-windows
```

3. Configure and build (PowerShell):

```powershell
#$triplet = "x64-windows"
#$toolchain = "C:/vcpkg/scripts/buildsystems/vcpkg.cmake"
#cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=$toolchain -DVCPKG_TARGET_TRIPLET=$triplet
#cmake --build build --config Release
# Or generate a Visual Studio solution:
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

4. Run:

```powershell
.\build\Release\o3f_lite.exe
```

If you see missing SFML DLLs at runtime, ensure the vcpkg `installed\x64-windows\bin` directory is on your PATH or copy required DLLs next to the executable.

## Build (MSYS2 / MinGW-w64)
If you prefer MinGW toolchain (useful on Windows without Visual Studio), use MSYS2's MinGW64 environment.

1. Open the "MSYS2 MinGW 64-bit" shell, then install packages:

```bash
# update msys packages (may require restarting the shell)
pacman -Syu
pacman -Su
# install toolchain, cmake and SFML
pacman -S --needed mingw-w64-x86_64-toolchain mingw-w64-x86_64-cmake mingw-w64-x86_64-SFML
```

2. From the MSYS2 MinGW64 shell, configure and build:

```bash
cd /o3f-re-implementation
rm -rf build
mkdir build
cd build
cmake .. -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DSFML_DIR=/mingw64/lib/cmake/SFML
mingw32-make -j4
```

3. Run from the same MinGW64 shell (so `/mingw64/bin` is on PATH):

```bash
./o3f_lite.exe
```

## Build (Linux)
On Debian/Ubuntu:

```bash
sudo apt update && sudo apt install -y libsfml-dev cmake build-essential
cmake -S . -B build
cmake --build build -j
./build/o3f_lite
```

## Running and controls
- R: reset a random scene
- Close window: exit

When running training mode (the default `main.cpp` loop), the program writes a timestamped CSV training log to the project root with columns:

- episode, total_reward, success (0/1), steps, options_used, epsilon

This CSV enables plotting learning curves and computing success rates over runs.

## Plotting training logs
A small helper script is included at `tools/plot_results.py`. Usage:

```bash
python tools/plot_results.py <training_log.csv>
```

The script plots rolling averages of episode reward and success rate (20-episode window) using pandas and matplotlib.

## Architecture
- `src/Env.cpp` / `include/Env.hpp`: environment, grid, reward function, obstacle management
- `src/Option.cpp` / `include/Option.hpp`: option classes and deterministic option policies
- `src/Planner.cpp` / `include/Planner.hpp`: tabular Q-learning over options (discrete state bucketing)
- `src/Executor.cpp` / `include/Executor.hpp`: runs option policies until option goals or timeouts
- `src/Agent.cpp`, `src/main.cpp`: orchestration, episode loop and training logic
- `include/Visualizer.hpp` and `src/Visualizer.*`: SFML visualization (grid, overlays)

## Troubleshooting & tips
- If CMake complains about a generator mismatch, remove the `build` directory and reconfigure with the desired generator.
- For MSYS2 builds, always use the "MSYS2 MinGW 64-bit" shell when building 64-bit MinGW targets.
- If training gets stuck frequently: try lowering obstacle density in `Environment2D::reset()` or enable more permissive clearing behavior (the demo already includes a `ClearObstacle` option).

## Next steps / experiments
- Export Q-table periodically and compare learned policies between seeds
- Sweep obstacle densities and seed values to verify robustness
- Replace local greedy pathing with a simple A* pathfinder in `Option::policy()` for more reliable execution
- Add reproducible seed options and a simple experiment harness to average multiple runs

## License
This repository is provided for educational purposes. Check project headers for any license statements you want to include or change.

