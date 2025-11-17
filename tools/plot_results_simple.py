#!/usr/bin/env python3
"""
Simple graph plotter for O3F training results - no pandas required
Usage: python tools/plot_results_simple.py <training_log.csv> [qtable.csv]
"""

import sys
import os
import csv
import matplotlib.pyplot as plt
import numpy as np

def read_csv(filename):
    """Read CSV file and return as list of dictionaries"""
    data = []
    try:
        with open(filename, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                data.append(row)
        return data
    except Exception as e:
        print(f"Error reading {filename}: {e}")
        return None

def moving_average(values, window):
    """Calculate moving average"""
    if len(values) < window:
        return values
    avg = []
    for i in range(len(values)):
        start = max(0, i - window // 2)
        end = min(len(values), i + window // 2 + 1)
        avg.append(sum(values[start:end]) / len(values[start:end]))
    return avg

if len(sys.argv) < 2:
    print("Usage: python tools/plot_results_simple.py <training_log.csv> [qtable.csv]")
    print("  training_log.csv: CSV with columns: episode,total_reward,success,steps,options_used,epsilon")
    print("  qtable.csv (optional): Q-table CSV for convergence analysis")
    sys.exit(1)

csv_file = sys.argv[1]
qtable_file = sys.argv[2] if len(sys.argv) > 2 else None

# Read training log
print(f"Loading training log from {csv_file}...")
data = read_csv(csv_file)
if not data:
    print(f"Failed to load {csv_file}")
    sys.exit(1)

# Extract columns
episodes = []
rewards = []
successes = []
steps = []

for row in data:
    try:
        episodes.append(int(row['episode']))
        rewards.append(float(row['total_reward']))
        successes.append(int(row['success']))
        steps.append(int(row['steps']))
    except (KeyError, ValueError) as e:
        print(f"Warning: Skipping malformed row: {e}")
        continue

print(f"Loaded {len(episodes)} episodes\n")

# Create output directory
output_dir = "training_graphs"
os.makedirs(output_dir, exist_ok=True)

# Set up plotting style
plt.style.use('seaborn-v0_8-darkgrid')

# ============================================================================
# 1. Total Episode Reward vs Episodes
# ============================================================================
fig, ax = plt.subplots(figsize=(12, 6))
ax.scatter(episodes, rewards, alpha=0.5, s=20, label='Episode Reward')
window = min(10, max(1, len(episodes) // 20))
if len(episodes) > window:
    rolling = moving_average(rewards, window)
    ax.plot(episodes, rolling, color='red', linewidth=2, 
            label=f'Rolling Average ({window} episodes)')
ax.set_xlabel('Episode', fontsize=12)
ax.set_ylabel('Total Reward', fontsize=12)
ax.set_title('Total Episode Reward vs Episodes', fontsize=14, fontweight='bold')
ax.legend(fontsize=10)
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(output_dir, '1_reward_vs_episodes.png'), dpi=300)
print(f"✓ Saved: {output_dir}/1_reward_vs_episodes.png")
plt.close()

# ============================================================================
# 2. Steps per Episode vs Episodes
# ============================================================================
fig, ax = plt.subplots(figsize=(12, 6))
ax.scatter(episodes, steps, alpha=0.5, s=20, label='Steps per Episode', color='#ff7f0e')
if len(episodes) > window:
    rolling = moving_average(steps, window)
    ax.plot(episodes, rolling, color='darkred', linewidth=2, 
            label=f'Rolling Average ({window} episodes)')
ax.set_xlabel('Episode', fontsize=12)
ax.set_ylabel('Steps', fontsize=12)
ax.set_title('Steps per Episode vs Episodes', fontsize=14, fontweight='bold')
ax.legend(fontsize=10)
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(output_dir, '2_steps_vs_episodes.png'), dpi=300)
print(f"✓ Saved: {output_dir}/2_steps_vs_episodes.png")
plt.close()

# ============================================================================
# 3. Total Reward vs Steps per Episode
# ============================================================================
fig, ax = plt.subplots(figsize=(12, 6))
scatter = ax.scatter(steps, rewards, c=episodes, cmap='viridis', 
                     alpha=0.6, s=30)
ax.set_xlabel('Steps per Episode', fontsize=12)
ax.set_ylabel('Total Reward', fontsize=12)
ax.set_title('Total Reward vs Steps per Episode', fontsize=14, fontweight='bold')
cbar = plt.colorbar(scatter, ax=ax)
cbar.set_label('Episode', fontsize=10)
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(output_dir, '3_reward_vs_steps.png'), dpi=300)
print(f"✓ Saved: {output_dir}/3_reward_vs_steps.png")
plt.close()

# ============================================================================
# 4. Success Rate (10-episode moving average) vs Episodes
# ============================================================================
if successes:
    fig, ax = plt.subplots(figsize=(12, 6))
    window_success = 10
    rolling = moving_average(successes, window_success)
    ax.plot(episodes, rolling, color='green', linewidth=2.5, 
            label=f'{window_success}-Episode Moving Average')
    ax.scatter(episodes, successes, alpha=0.3, s=15, color='lightgreen', 
               label='Individual Episode')
    ax.fill_between(episodes, 0, rolling, alpha=0.2, color='green')
    ax.set_xlabel('Episode', fontsize=12)
    ax.set_ylabel('Success Rate (0-1)', fontsize=12)
    ax.set_title('Success Rate (10-Episode Moving Average) vs Episodes', fontsize=14, fontweight='bold')
    ax.set_ylim([-0.05, 1.05])
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, '4_success_rate_vs_episodes.png'), dpi=300)
    print(f"✓ Saved: {output_dir}/4_success_rate_vs_episodes.png")
    plt.close()
else:
    print("Warning: 'success' column not found; skipping success rate plot")

# ============================================================================
# 5. Q-value Convergence Analysis (if Q-table provided)
# ============================================================================
if qtable_file and os.path.exists(qtable_file):
    print(f"\nLoading Q-table from {qtable_file}...")
    qtable_data = read_csv(qtable_file)
    if qtable_data:
        # Extract Q-values (all numeric columns except state)
        qvalues = []
        for row in qtable_data:
            for key, val in row.items():
                if key != 'state':  # Skip state column
                    try:
                        qval = float(val)
                        if not np.isnan(qval):
                            qvalues.append(qval)
                    except (ValueError, TypeError):
                        pass
        
        if qvalues:
            fig, ax = plt.subplots(figsize=(12, 6))
            
            ax.hist(qvalues, bins=50, alpha=0.7, color='steelblue', edgecolor='black')
            mean_q = np.mean(qvalues)
            median_q = np.median(qvalues)
            ax.axvline(mean_q, color='red', linestyle='--', linewidth=2, 
                       label=f'Mean: {mean_q:.2f}')
            ax.axvline(median_q, color='orange', linestyle='--', linewidth=2, 
                       label=f'Median: {median_q:.2f}')
            
            ax.set_xlabel('Q-Value', fontsize=12)
            ax.set_ylabel('Frequency', fontsize=12)
            ax.set_title('Q-Value Distribution (Final Q-Table)', fontsize=14, fontweight='bold')
            ax.legend(fontsize=10)
            ax.grid(True, alpha=0.3, axis='y')
            plt.tight_layout()
            plt.savefig(os.path.join(output_dir, '5_qvalue_distribution.png'), dpi=300)
            print(f"✓ Saved: {output_dir}/5_qvalue_distribution.png")
            plt.close()
            
            # Statistics
            learned_states = len(qtable_data)
            print(f"\nQ-Table Statistics:")
            print(f"  Total states: {learned_states}")
            print(f"  Q-values analyzed: {len(qvalues)}")
            print(f"  Q-value mean: {mean_q:.4f}")
            print(f"  Q-value std: {np.std(qvalues):.4f}")
            print(f"  Q-value min: {np.min(qvalues):.4f}")
            print(f"  Q-value max: {np.max(qvalues):.4f}")
else:
    if not qtable_file:
        print("\nTo analyze Q-value convergence, provide a Q-table CSV file:")
        print("  python tools/plot_results_simple.py <training_log.csv> <qtable.csv>")

# ============================================================================
# Summary Statistics
# ============================================================================
print("\n" + "="*60)
print("TRAINING SUMMARY STATISTICS")
print("="*60)
print(f"Total episodes: {len(episodes)}")
print(f"Average reward: {np.mean(rewards):.2f}")
print(f"Max reward: {np.max(rewards):.2f}")
print(f"Min reward: {np.min(rewards):.2f}")
print(f"Reward std dev: {np.std(rewards):.2f}")
print(f"\nAverage steps per episode: {np.mean(steps):.2f}")
print(f"Max steps per episode: {np.max(steps):.0f}")
print(f"Min steps per episode: {np.min(steps):.0f}")

if successes:
    success_rate = np.mean(successes)
    success_rate_last10 = np.mean(successes[-10:]) if len(successes) >= 10 else np.mean(successes)
    print(f"\nOverall success rate: {success_rate*100:.1f}%")
    print(f"Last 10 episodes success rate: {success_rate_last10*100:.1f}%")

print(f"\nAll graphs saved to: {os.path.abspath(output_dir)}")
print("="*60)
