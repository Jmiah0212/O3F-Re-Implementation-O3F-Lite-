# O3F Training Visualization Tools

This directory contains Python scripts for visualizing the training results from the O3F-Lite reinforcement learning agent.

## Prerequisites

### For `plot_results.py` (Full-featured version):
```bash
pip install pandas matplotlib numpy
```

### For `plot_results_simple.py` (Minimal dependencies):
```bash
pip install matplotlib numpy
```

## Usage

### Generate All Graphs

After training completes, you'll have training log files in the root directory with names like:
- `training_log_YYYYMMDD_HHMM.csv`
- `qtable_final_YYYYMMDD_HHMM.csv` (optional, for Q-value analysis)

#### Option 1: With pandas (recommended)
```bash
python tools/plot_results.py training_log_YYYYMMDD_HHMM.csv qtable_final_YYYYMMDD_HHMM.csv
```

#### Option 2: Without pandas (minimal dependencies)
```bash
python tools/plot_results_simple.py training_log_YYYYMMDD_HHMM.csv qtable_final_YYYYMMDD_HHMM.csv
```

Both scripts will generate the following graphs and save them to `training_graphs/` directory:

## Generated Graphs

### 1. **Reward vs Episodes** (`1_reward_vs_episodes.png`)
- Shows individual episode rewards as scatter plot
- Overlays rolling average to show trend
- Helps identify when learning improves

### 2. **Steps per Episode vs Episodes** (`2_steps_vs_episodes.png`)
- Displays steps taken in each episode
- Shows if the agent learns to solve tasks more efficiently
- Decreasing trend = more efficient solutions

### 3. **Reward vs Steps** (`3_reward_vs_steps.png`)
- Scatter plot with episodes color-coded
- Shows relationship between episode length and final reward
- Later episodes (warmer colors) should show better efficiency

### 4. **Success Rate (10-Episode Average)** (`4_success_rate_vs_episodes.png`)
- Shows 10-episode moving average of success rate
- Shaded area under the curve
- Should trend toward 1.0 (100% success) during training
- Indicates when the agent masters the task

### 5. **Q-Value Distribution** (`5_qvalue_distribution.png`)
- Histogram of all Q-values in the final Q-table
- Shows mean and median Q-values
- Indicates convergence: narrow distribution = converged, wide = still learning
- Higher Q-values indicate better actions were discovered

## Output Files

All graphs are saved to `training_graphs/` directory with high resolution (300 DPI).

## Summary Statistics

The scripts also print terminal output with useful statistics:

```
TRAINING SUMMARY STATISTICS
==================================================
Total episodes: 200
Average reward: X.XX
Max reward: X.XX
Min reward: X.XX
Reward std dev: X.XX

Average steps per episode: X.XX
Max steps per episode: XXX
Min steps per episode: XX

Overall success rate: XX.X%
Last 10 episodes success rate: XX.X%
==================================================
```

## Expected Training Trends

### Early Training (Low Success Rate):
- High variance in rewards
- Increasing steps per episode (exploration phase)
- Wide Q-value distribution

### Mid Training (Improving Performance):
- Rewards trending upward
- Steps stabilizing
- Q-values becoming more concentrated

### Late Training (Convergence):
- High success rate (>80%)
- Consistent low step counts
- Tight Q-value distribution with high mean values

## Tips for Interpretation

1. **Success Rate Not Improving?**
   - Check if agents are reaching the target first
   - Verify obstacle clearing is working
   - Consider increasing epsilon-greedy exploration

2. **Reward Decreasing?**
   - Episode rewards should stabilize or increase
   - Decreasing trend suggests learning is degrading
   - Check reward shaping parameters

3. **Steps Not Decreasing?**
   - Agent should learn more efficient paths over time
   - If flat, agent may be stuck in local optima
   - Try adjusting learning rate (alpha)

4. **Q-Values Diverging?**
   - Indicates unstable learning
   - Reduce learning rate alpha
   - Check for reward clipping issues

## Example Workflow

```bash
# Train the agent (generates training log)
./build/o3f_lite --save-q-interval 50

# Wait for training to complete...

# Generate visualizations
python tools/plot_results_simple.py training_log_20251117_1500.csv qtable_final_20251117_1500.csv

# View results
# Open training_graphs/ folder to see PNG files
```

## Customization

Both scripts are designed to be easily modifiable:

- **Change window size for moving average**: Modify `window = 10` line
- **Add different statistics**: Edit summary statistics section
- **Modify colors/style**: Change matplotlib style and colors
- **Add new plots**: Follow the same pattern as existing plots

## Troubleshooting

**"Import X could not be resolved"**: This is just IDE linting - scripts will work at runtime

**"No such file or directory"**: Make sure training log CSV exists and path is correct

**"No success column found"**: CSV format mismatch - ensure CSV has correct column headers

**Graphs not showing**: Check that `training_graphs/` directory was created successfully
