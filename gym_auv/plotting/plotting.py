import pandas as pd
import matplotlib.pyplot as plt
import argparse
import os

# File for plotting performance data when testing different perception vectors


import os
import pandas as pd
import matplotlib.pyplot as plt
import argparse

def plot_rewards_per_episode(args, window_size=50, save_plot=False, save_path=None):
    """
    Plots the total reward per episode and its moving average from a CSV file.

    Parameters:
    - args: Namespace object containing command-line arguments.
    - window_size (int): The number of episodes to include in the moving average window. Default is 50.
    - save_plot (bool): Whether to save the plot as an image file. Default is False.
    - save_path (str): The file path to save the plot if save_plot is True. Default is None.
    """
    # Verify that the CSV file exists
    if not os.path.isfile(args.episode_reward_path):
        print(f"Error: The file '{args.episode_reward_path}' does not exist.")
        return

    # Load Episode Rewards
    try:
        episode_rewards = pd.read_csv(args.episode_reward_path)
    except Exception as e:
        print(f"Error reading the CSV file: {e}")
        return

    # Check if the required columns exist
    expected_columns = ['Episode', 'Total Reward']
    if not all(column in episode_rewards.columns for column in expected_columns):
        print(f"Error: CSV file must contain the columns: {expected_columns}")
        print(f"Found columns: {episode_rewards.columns.tolist()}")
        return

    # Ensure 'Episode' is sorted
    episode_rewards = episode_rewards.sort_values('Episode')

    # Calculate Moving Average
    episode_rewards['Moving Average'] = episode_rewards['Total Reward'].rolling(window=window_size, min_periods=1).mean()

    # Plot Episode Rewards
    plt.figure(figsize=(14, 7))
    
    # Plot Raw Episode Rewards
    plt.plot(episode_rewards['Episode'], episode_rewards['Total Reward'], 
             label='Episode Reward', color='blue', alpha=0.5)
    
    # Plot Moving Average
    plt.plot(episode_rewards['Episode'], episode_rewards['Moving Average'], 
             label=f'{window_size}-Episode Moving Average', color='red', linewidth=2)
    
    # Customize Plot
    plt.xlabel('Episode', fontsize=14)
    plt.ylabel('Total Reward', fontsize=14)
    plt.title('Total Reward per Episode with Moving Average', fontsize=16)
    plt.legend(fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.tight_layout()
    
    # Show Plot
    plt.show()

    # Save Plot if Required
    if save_plot:
        if save_path is None:
            # Default save path if not provided
            save_path = os.path.join(os.path.dirname(args.episode_reward_path), 'episode_rewards_plot.png')
        plt.savefig(save_path)
        print(f"Plot saved to {save_path}")

def main(args):
    plot_rewards_per_episode(args)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot Episode Rewards with Moving Average from a CSV file.')
    parser.add_argument('--episode_reward_path', type=str, required=True,
                        help='Path to the CSV file containing episode rewards.')
    parser.add_argument('--window_size', type=int, default=50,
                        help='Number of episodes to include in the moving average window. Default is 50.')
    parser.add_argument('--save_plot', action='store_true',
                        help='Whether to save the plot as an image file.')
    parser.add_argument('--save_path', type=str, default=None,
                        help='File path to save the plot image. Required if --save_plot is used.')
    args = parser.parse_args()
    
    plot_rewards_per_episode(args, window_size=args.window_size, save_plot=args.save_plot, save_path=args.save_path)

