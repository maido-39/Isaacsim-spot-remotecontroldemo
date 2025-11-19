#!/usr/bin/env python3
"""
Experiment Data Analysis Script

This script demonstrates how to load and analyze experiment data
saved by the Spot robot simulation.

Usage:
    python analyze_experiment_data.py <experiment_directory>

Example:
    python analyze_experiment_data.py 241118_153045-my_experiment
"""

import sys
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path


def load_experiment_data(experiment_dir):
    """
    Load experiment configuration and CSV data.
    
    Args:
        experiment_dir: Path to experiment directory
        
    Returns:
        tuple: (config_dict, dataframe)
    """
    exp_path = Path(experiment_dir)
    
    # Load configuration
    config_path = exp_path / "config.json"
    with open(config_path, 'r') as f:
        config = json.load(f)
    
    # Load CSV data
    csv_path = exp_path / "data.csv"
    df = pd.read_csv(csv_path)
    
    return config, df


def quaternion_to_euler(qw, qx, qy, qz):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw) in radians.
    
    Args:
        qw, qx, qy, qz: Quaternion components
        
    Returns:
        tuple: (roll, pitch, yaw) in radians
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    pitch = np.copysign(np.pi / 2, sinp) if abs(sinp) >= 1 else np.arcsin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


def add_derived_features(df, config):
    """
    Add derived features to the dataframe.
    
    Args:
        df: Pandas dataframe with experiment data
        config: Configuration dictionary
        
    Returns:
        Modified dataframe with additional columns
    """
    # Convert quaternions to Euler angles
    euler_data = df.apply(
        lambda row: quaternion_to_euler(
            row['robot_orient_w'], row['robot_orient_x'],
            row['robot_orient_y'], row['robot_orient_z']
        ), axis=1
    )
    df['robot_roll'] = [x[0] for x in euler_data]
    df['robot_pitch'] = [x[1] for x in euler_data]
    df['robot_yaw'] = [x[2] for x in euler_data]
    
    # Calculate robot speed (2D)
    dt = df['timestamp'].diff()
    dx = df['robot_pos_x'].diff()
    dy = df['robot_pos_y'].diff()
    df['robot_speed_2d'] = np.sqrt(dx**2 + dy**2) / dt
    
    # Calculate distance to goal
    if 'goal_position' in config:
        goal_x, goal_y = config['goal_position'][:2]
        df['distance_to_goal'] = np.sqrt(
            (df['robot_pos_x'] - goal_x)**2 + 
            (df['robot_pos_y'] - goal_y)**2
        )
    
    # Calculate distance to start
    if 'start_position' in config:
        start_x, start_y = config['start_position'][:2]
        df['distance_from_start'] = np.sqrt(
            (df['robot_pos_x'] - start_x)**2 + 
            (df['robot_pos_y'] - start_y)**2
        )
    
    # Calculate object distance to goal (if object exists)
    if config.get('use_object', True) and config.get('object_type', 'box') != 'gate':
        if 'goal_position' in config:
            goal_x, goal_y = config['goal_position'][:2]
            df['object_distance_to_goal'] = np.sqrt(
                (df['object_pos_x'] - goal_x)**2 + 
                (df['object_pos_y'] - goal_y)**2
            )
    
    return df


def print_summary_statistics(df, config):
    """Print summary statistics about the experiment."""
    print("\n" + "="*60)
    print("EXPERIMENT SUMMARY")
    print("="*60)
    
    # Time statistics
    duration = df['timestamp'].max()
    num_frames = len(df)
    print(f"\nTime Statistics:")
    print(f"  Duration: {duration:.2f} seconds")
    print(f"  Number of frames: {num_frames}")
    print(f"  Average frame rate: {num_frames/duration:.2f} Hz")
    
    # Robot statistics
    print(f"\nRobot Statistics:")
    print(f"  Average speed: {df['robot_speed_2d'].mean():.3f} m/s")
    print(f"  Max speed: {df['robot_speed_2d'].max():.3f} m/s")
    
    # L1 distance statistics (new metric)
    if 'l1_distance_to_goal' in df.columns:
        print(f"\nL1 Distance to Goal (from CSV):")
        print(f"  Initial: {df['l1_distance_to_goal'].iloc[0]:.2f} m")
        print(f"  Final: {df['l1_distance_to_goal'].iloc[-1]:.2f} m")
        print(f"  Minimum: {df['l1_distance_to_goal'].min():.2f} m")
        print(f"  Change: {df['l1_distance_to_goal'].iloc[0] - df['l1_distance_to_goal'].iloc[-1]:.2f} m")
    
    if 'distance_to_goal' in df.columns:
        print(f"  Initial distance to goal: {df['distance_to_goal'].iloc[0]:.2f} m")
        print(f"  Final distance to goal: {df['distance_to_goal'].iloc[-1]:.2f} m")
        print(f"  Closest approach to goal: {df['distance_to_goal'].min():.2f} m")
    
    # Object statistics (if applicable)
    if 'object_distance_to_goal' in df.columns:
        print(f"\nObject Statistics:")
        print(f"  Initial distance to goal: {df['object_distance_to_goal'].iloc[0]:.2f} m")
        print(f"  Final distance to goal: {df['object_distance_to_goal'].iloc[-1]:.2f} m")
        print(f"  Change: {df['object_distance_to_goal'].iloc[-1] - df['object_distance_to_goal'].iloc[0]:.2f} m")


def plot_trajectory(df, config, save_path=None):
    """Plot robot trajectory in 2D."""
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Plot robot trajectory
    ax.plot(df['robot_pos_x'], df['robot_pos_y'], 'b-', linewidth=2, label='Robot path')
    
    # Plot start position
    if 'start_position' in config:
        start_x, start_y = config['start_position'][:2]
        ax.scatter(start_x, start_y, c='green', s=200, marker='o', 
                  label='Start', zorder=5, edgecolors='black', linewidths=2)
    
    # Plot goal position
    if 'goal_position' in config:
        goal_x, goal_y = config['goal_position'][:2]
        ax.scatter(goal_x, goal_y, c='red', s=200, marker='*', 
                  label='Goal', zorder=5, edgecolors='black', linewidths=2)
    
    # Plot object if it's a dynamic object
    if config.get('use_object', True) and config.get('object_type', 'box') != 'gate':
        # Plot initial and final object positions
        ax.scatter(df['object_pos_x'].iloc[0], df['object_pos_y'].iloc[0], 
                  c='orange', s=100, marker='s', label='Object (start)', 
                  zorder=4, edgecolors='black', linewidths=1)
        ax.scatter(df['object_pos_x'].iloc[-1], df['object_pos_y'].iloc[-1], 
                  c='brown', s=100, marker='s', label='Object (end)', 
                  zorder=4, edgecolors='black', linewidths=1)
        
        # Plot object trajectory
        ax.plot(df['object_pos_x'], df['object_pos_y'], 'orange', 
               linewidth=1, alpha=0.5, linestyle='--')
    
    # Plot environment boundaries
    if 'map_size' in config:
        map_size = config['map_size']
        half_size = map_size / 2.0
        ax.plot([-half_size, half_size, half_size, -half_size, -half_size],
               [-half_size, -half_size, half_size, half_size, -half_size],
               'k--', linewidth=1, alpha=0.5, label='Environment bounds')
    
    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.set_title('Robot Trajectory', fontsize=14, fontweight='bold')
    ax.legend(loc='best')
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Trajectory plot saved to: {save_path}")
    
    plt.tight_layout()
    return fig


def plot_time_series(df, save_path=None):
    """Plot time-series data."""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    
    # Plot 1: Robot speed
    axes[0].plot(df['timestamp'], df['robot_speed_2d'], 'b-', linewidth=1.5)
    axes[0].set_ylabel('Speed (m/s)', fontsize=11)
    axes[0].set_title('Robot Speed Over Time', fontsize=12, fontweight='bold')
    axes[0].grid(True, alpha=0.3)
    
    # Plot 2: L1 Distance to goal (prefer L1 from CSV if available, otherwise calculate)
    if 'l1_distance_to_goal' in df.columns:
        axes[1].plot(df['timestamp'], df['l1_distance_to_goal'], 'r-', linewidth=1.5, label='L1 Distance (CSV)')
        axes[1].set_ylabel('L1 Distance (m)', fontsize=11)
        axes[1].set_title('L1 Distance to Goal Over Time', fontsize=12, fontweight='bold')
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
    elif 'distance_to_goal' in df.columns:
        axes[1].plot(df['timestamp'], df['distance_to_goal'], 'r-', linewidth=1.5)
        axes[1].set_ylabel('Distance (m)', fontsize=11)
        axes[1].set_title('Distance to Goal Over Time', fontsize=12, fontweight='bold')
        axes[1].grid(True, alpha=0.3)
    
    # Plot 3: Robot orientation (yaw)
    axes[2].plot(df['timestamp'], np.degrees(df['robot_yaw']), 'g-', linewidth=1.5)
    axes[2].set_xlabel('Time (s)', fontsize=11)
    axes[2].set_ylabel('Yaw (degrees)', fontsize=11)
    axes[2].set_title('Robot Yaw Angle Over Time', fontsize=12, fontweight='bold')
    axes[2].grid(True, alpha=0.3)
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Time series plot saved to: {save_path}")
    
    plt.tight_layout()
    return fig


def main():
    """Main function."""
    if len(sys.argv) < 2:
        print("Usage: python analyze_experiment_data.py <experiment_directory>")
        print("Example: python analyze_experiment_data.py 241118_153045-my_experiment")
        sys.exit(1)
    
    experiment_dir = sys.argv[1]
    
    # Check if directory exists
    if not Path(experiment_dir).exists():
        print(f"Error: Directory '{experiment_dir}' not found")
        sys.exit(1)
    
    print(f"Loading experiment data from: {experiment_dir}")
    
    # Load data
    config, df = load_experiment_data(experiment_dir)
    
    # Add derived features
    df = add_derived_features(df, config)
    
    # Print summary statistics
    print_summary_statistics(df, config)
    
    # Create plots
    print("\n" + "="*60)
    print("GENERATING PLOTS")
    print("="*60)
    
    output_dir = Path(experiment_dir)
    
    # Plot trajectory
    trajectory_path = output_dir / "analysis_trajectory.png"
    plot_trajectory(df, config, save_path=trajectory_path)
    
    # Plot time series
    timeseries_path = output_dir / "analysis_timeseries.png"
    plot_time_series(df, save_path=timeseries_path)
    
    print("\nAnalysis complete!")
    print("Close the plot windows to exit.")
    
    # Show all plots
    plt.show()


if __name__ == "__main__":
    main()

