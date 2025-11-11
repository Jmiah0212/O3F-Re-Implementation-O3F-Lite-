import sys
import pandas as pd
import matplotlib.pyplot as plt

if len(sys.argv) < 2:
    print("Usage: python tools/plot_results.py <training_log.csv>")
    sys.exit(1)

csv_file = sys.argv[1]

df = pd.read_csv(csv_file)
if 'total_reward' not in df.columns and 'episode' in df.columns:
    # attempt common alternate names
    if 'total_reward' not in df.columns and 'totalReward' in df.columns:
        df = df.rename(columns={'totalReward': 'total_reward'})

# Rolling averages
window = 20
if 'total_reward' in df.columns:
    df['rolling_reward'] = df['total_reward'].rolling(window).mean()
else:
    # fallback to second column
    df['rolling_reward'] = df.iloc[:,1].rolling(window).mean()

if 'success' in df.columns:
    df['rolling_success'] = df['success'].rolling(window).mean()
else:
    df['rolling_success'] = None

plt.figure(figsize=(10,5))
plt.plot(df['episode'], df['rolling_reward'], label=f'Rolling Reward ({window})')
plt.title('Cumulative Reward over Episodes')
plt.xlabel('Episode')
plt.ylabel('Reward')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()

if df['rolling_success'].notnull().any():
    plt.figure(figsize=(10,5))
    plt.plot(df['episode'], df['rolling_success'], label=f'Rolling Success ({window})')
    plt.title('Success Rate (Moving Avg, 20 Episodes)')
    plt.xlabel('Episode')
    plt.ylabel('Success Rate')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()
else:
    print('No success column found in CSV; skipping success plot.')
