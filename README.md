# O3F-Lite (CSCE 4430)

A minimal C++/SFML re-implementation inspired by the IEEE IROS 2023 paper "Object-Oriented Option Framework for Robotics Manipulation in Clutter (O3F)". This demo shows:

- State representation of a 2D world (robot, objects)
- Option hierarchy: an Option Planner selects high-level options; an Executor handles low-level motion
- Reward shaping: success (objects in target region), proximity, time penalty
- Reinforcement learning: a simplified Q-learning planner over discrete options
- Visualization via SFML

## Build (Windows, CMake + vcpkg)

1. Install dependencies:
   - Visual Studio 2019/2022 with C++ workload
   - CMake 3.20+
   - vcpkg

2. Install SFML via vcpkg:

   ```powershell
   .\vcpkg\vcpkg.exe install sfml:x64-windows
   ```

3. Configure and build:

   ```powershell
   $triplet = "x64-windows"
   $toolchain = "C:/vcpkg/scripts/buildsystems/vcpkg.cmake"
   cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=$toolchain -DVCPKG_TARGET_TRIPLET=$triplet
   cmake --build build --config Release
   ```

4. Run:

   ```powershell
   .\build\Release\o3f_lite.exe
   ```

If DLLs are missing, ensure that vcpkg's installed `bin` directory is on your PATH or copy required SFML DLLs next to the executable.

## Controls

- R: Reset random scene
- Window close button: exit

