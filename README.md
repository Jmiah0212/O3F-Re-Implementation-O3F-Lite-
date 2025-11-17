# O3F-Lite: Smart Robot Task Learning

A C++/SFML implementation of a reinforcement learning agent that learns to manipulate objects in cluttered environments. Based on principles from the IEEE IROS 2023 paper "Object-Oriented Option Framework for Robotics Manipulation in Clutter (O3F)".

This project implements a **hierarchical reinforcement learning framework** where:
1. A high-level planner selects **options** (abstract actions) using Q-learning
2. Low-level **executors** run deterministic policies to accomplish each option
3. The agent learns to **clear obstacles, navigate to targets, and retrieve objects**

## Key Features

### Agent Capabilities
- **Intelligent Path Planning**: Uses BFS to find shortest paths while avoiding obstacles
- **Obstacle Clearing**: Strategically clears blocking obstacles to reach goals
- **Object Retrieval**: Navigates to objects and picks them up
- **Smart Return Navigation**: Remembers and follows the exact path taken to reach an object, then retraces it back to the target
- **Learning**: Tabular Q-learning over a discrete state space (position, distance, direction, carrying state)

### Options (High-Level Actions)
1. **ClearObstacle** - Clear blocking obstacles in the way
2. **MoveToTarget** - Navigate to the target location
3. **ReturnToObject** - Navigate to the object location (clearing obstacles as needed)
4. **MoveObjectToTarget** - Carry object back to target following the stored return path

### Technical Highlights
- 2D discrete grid environment with physics-lite interactions
- Deterministic policy execution with adaptive behaviors
- Reward shaping for progress, penalties, and task completion
- Real-time SFML visualization during training
- Comprehensive training logging and analysis tools
- Path memory system for efficient return navigation

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

## Running and Controls

### Interactive Controls
- **R**: Reset the scene with random positions
- **Close window**: Exit the program

### Training Mode (Default)
The program automatically runs training episodes and logs results to timestamped CSV files in the root directory:
- `training_log_YYYYMMDD_HHMM.csv` - Training metrics (episode, reward, success, steps, etc.)
- `qtable_final_YYYYMMDD_HHMM.csv` - Final Q-table state after training

### Command-Line Options
```bash
./o3f_lite.exe [--load-q <qtable.csv>] [--save-q-interval <episodes>]
```

**Examples:**
```bash
# Run training with automatic Q-table saves every 50 episodes
./o3f_lite.exe --save-q-interval 50

# Resume training from a previously learned Q-table
./o3f_lite.exe --load-q qtable_final_20251117_1500.csv --save-q-interval 25
```

## Training Visualization and Analysis

### Quick Plotting (No Pandas Required)
The simplest way to analyze results after training:

```bash
python tools/plot_results_simple.py training_log_YYYYMMDD_HHMM.csv qtable_final_YYYYMMDD_HHMM.csv
```

### Full Analysis (With Pandas)
For more detailed statistical analysis:

```bash
pip install pandas matplotlib numpy
python tools/plot_results.py training_log_YYYYMMDD_HHMM.csv qtable_final_YYYYMMDD_HHMM.csv
```

### Generated Graphs

Both tools generate **5 comprehensive graphs** saved to `training_graphs/` directory:

1. **Total Episode Reward vs Episodes** - Learning curve showing reward progression
   - Scatter plot with rolling average overlay
   - Shows when agent starts improving

2. **Steps per Episode vs Episodes** - Efficiency learning curve
   - Decreasing trend = agent learns more efficient solutions
   - Shows convergence to optimal behavior

3. **Total Reward vs Steps** - Performance-efficiency relationship
   - Episodes color-coded by training progress
   - Later episodes should show better reward-to-steps ratio

4. **Success Rate (10-Episode Moving Average)** - Task mastery progression
   - Shows smoothed success rate over training
   - Shaded area visualizes confidence
   - Target: converge to 1.0 (100% success)

5. **Q-Value Distribution** - Learning convergence indicator
   - Histogram of final Q-values in the learned policy
   - Tight distribution = stable convergence
   - High mean = effective learning
   - Wide distribution = still exploring

### Summary Statistics
The scripts print detailed statistics to console:
```
TRAINING SUMMARY STATISTICS
==================================================
Total episodes: 200
Average reward: 45.32
Max reward: 105.20
Min reward: -15.40

Average steps per episode: 42.5
Overall success rate: 65.3%
Last 10 episodes success rate: 92.0%
==================================================
```

For detailed documentation, see `tools/README.md`

## Architecture Overview

### Core Components
- **`src/Env.cpp` / `include/Env.hpp`**: 2D grid environment
  - Grid-based state representation
  - Obstacle management and clearing mechanics
  - Physics-lite object interactions
  - Reward computation

- **`src/Option.cpp` / `include/Option.hpp`**: Hierarchical options system
  - `MoveToTargetOption`: Navigate to goal location
  - `ClearObstacleOption`: Remove blocking obstacles strategically
  - `MoveToObjectOption`: Find and navigate to object (stores path via BFS)
  - `MoveObjectToTargetOption`: Return to target following stored path
  - `ReturnToObjectOption`: Smart navigation with obstacle avoidance
  - Path-based state for efficient return journeys

- **`src/Planner.cpp` / `include/Planner.hpp`**: High-level decision making
  - Tabular Q-learning over discrete states
  - State discretization: position, distance bucket, direction, carrying status
  - Epsilon-greedy exploration with decay
  - Q-table persistence (save/load)

- **`src/Executor.cpp` / `include/Executor.hpp`**: Option execution
  - Runs option policies until completion or timeout
  - Failure detection (stuck for 3+ steps)
  - Reward computation and feedback
  - Handles special option mechanics (e.g., obstacle clearing)

- **`src/main.cpp`**: Episode orchestration
  - Training loop with phase-based control
  - Phase transitions: ClearObstacles → MoveToTarget → ReturnToObject → MoveObjectToTarget
  - Path memory management
  - Episode logging and metrics

- **`include/Visualizer.hpp` / `src/Visualizer.cpp`**: Real-time visualization
  - SFML-based grid rendering
  - Robot, target, object, and obstacle display
  - Training overlay (episode, reward, success rate)
  - Interactive controls

### Pathfinding and Navigation
- **BFS Pathfinding**: Finds shortest collision-free paths
  - `bfsFullPath()`: Returns complete path waypoints for return navigation
  - `bfsNextAction()`: Returns next step along shortest path
  - `smartPathfinding()`: Heuristic-based greedy navigation (fallback)

- **Path Memory System**: Enables intelligent return navigation
  - Stores waypoint path during object search
  - Reverses path during return journey
  - Allows backward steps (away from target) to follow optimal route
  - Resets upon new path computation

### Training Flow

```
Episode Loop:
  ├─ Phase 0: Clear nearby obstacles
  ├─ Phase 1: Navigate to target (MoveToTargetOption)
  ├─ Phase 2: Find and retrieve object (ReturnToObjectOption + ClearObstacleOption)
  │  └─ BFS stores complete path to object
  └─ Phase 3: Return to target with object (MoveObjectToTargetOption)
     └─ Follows stored path in reverse, allowing backward progress
     
Q-Learning Update:
  ├─ Observe state before action
  ├─ Execute option (3 steps max per option)
  ├─ Compute reward
  ├─ Observe next state
  └─ Update Q-value: Q[s,a] ← Q[s,a] + α(r + γ*max(Q[s',·]) - Q[s,a])
```

## Troubleshooting & Tips

### Build Issues
- **CMake generator mismatch**: Remove `build/` directory and reconfigure
- **SFML not found**: On MSYS2, ensure `/mingw64/lib/cmake/SFML` exists, update SFML if needed
- **DLL loading errors**: Add `/mingw64/bin` to PATH or copy DLLs next to executable

### Training Issues
- **Low success rate** (<20%): 
  - Check obstacle density (increase/decrease in `Environment2D::reset()`)
  - Verify BFS pathfinding is working (check console output for path sizes)
  - Lower learning rate (reduce `alpha`) for stability

- **Agent gets stuck**:
  - This is normal early in training (high epsilon exploration)
  - Ensure `ClearObstacle` option is clearing obstacles properly
  - Check obstacle clearing reward is positive

- **Training not improving**:
  - Verify Q-table is being updated (check loaded Q-table)
  - Check epsilon decay is working (should trend toward 0.05)
  - Try increasing exploration with higher initial epsilon

### Interpreting Training Logs
- **Success rate stuck below 50%**: Agent hasn't learned basic navigation yet
- **Success rate 50-80%**: Agent learning, but inconsistent execution
- **Success rate >80%**: Agent has learned effective policy; continue training for convergence
- **Reward decreasing**: Unusual; check reward shaping parameters

### Performance Tuning
Parameter defaults in `main.cpp`:
```cpp
PlannerConfig plannerCfg;
plannerCfg.alpha = 0.1f;        // Learning rate: lower = more stable, slower learning
plannerCfg.gamma = 0.95f;        // Discount factor: higher = value future rewards more
plannerCfg.epsilon = 1.0f;       // Exploration rate: start high, decay over time
plannerCfg.epsilonDecay = 0.995f; // Decay per episode: higher = slower decay
plannerCfg.epsilonMin = 0.05f;   // Minimum exploration: never fully greedy
```

Suggested adjustments:
- **Faster learning**: Increase `alpha` to 0.2 (more variance)
- **Slower learning**: Decrease `alpha` to 0.05 (more stable)
- **More exploration**: Increase `epsilon` to 1.0 or `epsilonMin` to 0.1
- **Focus learning**: Decrease `gamma` to 0.9 (short-term rewards)

## Debugging Features

### Console Output
Training episodes 0-2 print detailed phase transitions:
```
Episode 0, Phase: MoveToTarget (Option: MoveToTarget), Reward: 2.5, Total: 2.5
Episode 0 - Reached target! Transitioning to ReturnToObject phase.
  Path size: 12 waypoints
  Path set for return journey
Episode 0 - Picked up object! Transitioning to MoveObjectToTarget phase.
Episode 0 SUCCESS! Reward: 52.50
```

### Q-Table Analysis
Load a saved Q-table to inspect learned policies:
```python
import pandas as pd
df = pd.read_csv('qtable_final_20251117_1500.csv')
print(f"States learned: {len(df)}")
print(f"Mean Q-value: {df.iloc[:,1:].values.mean():.4f}")
```

## Experimentation Ideas

### Short-term Experiments
- [ ] Vary initial obstacle density (5, 10, 20 obstacles)
- [ ] Test different learning rates (α = 0.05, 0.1, 0.2)
- [ ] Compare success rates across random seeds (run 3-5 independent trials)
- [ ] Measure efficiency improvement (steps per episode over time)

### Medium-term Improvements
- [ ] Add A* search as option policy (more reliable than greedy)
- [ ] Implement function approximation for larger state spaces
- [ ] Add experience replay for better sample efficiency
- [ ] Implement option interruption for dynamic switching

### Advanced Directions
- [ ] Hierarchical option discovery (learn sub-options)
- [ ] Multi-agent coordination (multiple robots)
- [ ] Transfer learning (pre-train on simple environments)
- [ ] Integration with real robot (e.g., UR5 with ROS)

## Performance Benchmarks

Expected performance on standard settings (20 episodes, 5 obstacles):
- **Episode 1**: Success rate ~5% (initial exploration)
- **Episode 50**: Success rate ~30-50% (learning in progress)
- **Episode 100**: Success rate ~60-80% (approaching convergence)
- **Episode 200**: Success rate >80% (learned policy)

Note: High variance early in training is expected due to random scene generation

## License
This repository is provided for educational purposes. Check project headers for any license statements you want to include or change.

