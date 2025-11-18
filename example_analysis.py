#!/usr/bin/env python3
"""
Example Experiment Data Analysis

This script demonstrates various ways to analyze experiment data.
Can be used as-is or adapted for Jupyter notebooks.

Usage:
    python example_analysis.py <experiment_directory>
"""

import sys
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path


def load_data(experiment_dir):
    """Load experiment data."""
    exp_path = Path(experiment_dir)
    
    # Load configuration
    with open(exp_path / "config.json", 'r') as f:
        config = json.load(f)
    
    # Load CSV data
    df = pd.read_csv(exp_path / "data.csv")
    
    return config, df


def example_basic_statistics(df, config):
    """Example 1: Basic statistics about the experiment."""
    print("\n" + "="*60)
    print("EXAMPLE 1: Basic Statistics")
    print("="*60)
    
    # Experiment duration
    duration = df['timestamp'].max() - df['timestamp'].min()
    print(f"Duration: {duration:.2f} seconds")
    print(f"Number of data points: {len(df)}")
    print(f"Average frame rate: {len(df)/duration:.2f} Hz")
    
    # Robot position range
    print(f"\nRobot position range:")
    print(f"  X: [{df['robot_pos_x'].min():.2f}, {df['robot_pos_x'].max():.2f}]")
    print(f"  Y: [{df['robot_pos_y'].min():.2f}, {df['robot_pos_y'].max():.2f}]")
    print(f"  Z: [{df['robot_pos_z'].min():.2f}, {df['robot_pos_z'].max():.2f}]")


def example_calculate_speed(df):
    """Example 2: Calculate robot speed."""
    print("\n" + "="*60)
    print("EXAMPLE 2: Robot Speed Calculation")
    print("="*60)
    
    # Calculate instantaneous speed
    dt = df['timestamp'].diff()
    dx = df['robot_pos_x'].diff()
    dy = df['robot_pos_y'].diff()
    dz = df['robot_pos_z'].diff()
    
    speed_2d = np.sqrt(dx**2 + dy**2) / dt
    speed_3d = np.sqrt(dx**2 + dy**2 + dz**2) / dt
    
    df['speed_2d'] = speed_2d
    df['speed_3d'] = speed_3d
    
    print(f"Average speed (2D): {speed_2d.mean():.3f} m/s")
    print(f"Max speed (2D): {speed_2d.max():.3f} m/s")
    print(f"Average speed (3D): {speed_3d.mean():.3f} m/s")
    print(f"Max speed (3D): {speed_3d.max():.3f} m/s")
    
    return df


def example_distance_to_goal(df, config):
    """Example 3: Calculate distance to goal over time."""
    print("\n" + "="*60)
    print("EXAMPLE 3: Distance to Goal")
    print("="*60)
    
    if 'goal_position' not in config:
        print("No goal position in config")
        return df
    
    goal_x, goal_y = config['goal_position'][:2]
    
    # Calculate distance to goal
    dist_to_goal = np.sqrt(
        (df['robot_pos_x'] - goal_x)**2 + 
        (df['robot_pos_y'] - goal_y)**2
    )
    
    df['distance_to_goal'] = dist_to_goal
    
    print(f"Initial distance: {dist_to_goal.iloc[0]:.2f} m")
    print(f"Final distance: {dist_to_goal.iloc[-1]:.2f} m")
    print(f"Minimum distance: {dist_to_goal.min():.2f} m")
    print(f"Change: {dist_to_goal.iloc[-1] - dist_to_goal.iloc[0]:.2f} m")
    
    # Find when robot was closest
    closest_idx = dist_to_goal.idxmin()
    closest_time = df.loc[closest_idx, 'timestamp']
    print(f"Closest approach at t={closest_time:.2f}s (frame {df.loc[closest_idx, 'frame_num']})")
    
    return df


def example_quaternion_to_yaw(df):
    """Example 4: Convert quaternions to yaw angle."""
    print("\n" + "="*60)
    print("EXAMPLE 4: Quaternion to Yaw Conversion")
    print("="*60)
    
    def quat_to_yaw(w, x, y, z):
        """Extract yaw angle from quaternion."""
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return np.arctan2(siny_cosp, cosy_cosp)
    
    # Calculate yaw for each row
    yaw = df.apply(lambda row: quat_to_yaw(
        row['robot_orient_w'], row['robot_orient_x'],
        row['robot_orient_y'], row['robot_orient_z']
    ), axis=1)
    
    df['robot_yaw'] = yaw
    df['robot_yaw_deg'] = np.degrees(yaw)
    
    print(f"Initial yaw: {df['robot_yaw_deg'].iloc[0]:.1f}°")
    print(f"Final yaw: {df['robot_yaw_deg'].iloc[-1]:.1f}°")
    print(f"Yaw range: [{df['robot_yaw_deg'].min():.1f}°, {df['robot_yaw_deg'].max():.1f}°]")
    
    # Calculate total rotation
    yaw_change = np.abs(np.diff(yaw))
    total_rotation = np.sum(yaw_change)
    print(f"Total rotation: {np.degrees(total_rotation):.1f}°")
    
    return df


def example_trajectory_analysis(df, config):
    """Example 5: Analyze trajectory characteristics."""
    print("\n" + "="*60)
    print("EXAMPLE 5: Trajectory Analysis")
    print("="*60)
    
    # Calculate path length
    dx = df['robot_pos_x'].diff()
    dy = df['robot_pos_y'].diff()
    segment_lengths = np.sqrt(dx**2 + dy**2)
    total_path_length = segment_lengths.sum()
    
    print(f"Total path length: {total_path_length:.2f} m")
    
    # Calculate straight-line distance
    if 'start_position' in config and 'goal_position' in config:
        start = np.array(config['start_position'][:2])
        goal = np.array(config['goal_position'][:2])
        straight_dist = np.linalg.norm(goal - start)
        
        # Calculate actual traveled distance
        actual_start = df[['robot_pos_x', 'robot_pos_y']].iloc[0].values
        actual_end = df[['robot_pos_x', 'robot_pos_y']].iloc[-1].values
        actual_dist = np.linalg.norm(actual_end - actual_start)
        
        print(f"Straight-line distance (start to goal): {straight_dist:.2f} m")
        print(f"Actual displacement: {actual_dist:.2f} m")
        print(f"Path efficiency: {actual_dist/total_path_length*100:.1f}%")
        print(f"Tortuosity: {total_path_length/straight_dist:.2f}")


def example_visualization(df, config):
    """Example 6: Create comprehensive visualization."""
    print("\n" + "="*60)
    print("EXAMPLE 6: Visualization")
    print("="*60)
    
    fig = plt.figure(figsize=(15, 10))
    
    # Plot 1: Trajectory with color-coded time
    ax1 = plt.subplot(2, 3, 1)
    scatter = ax1.scatter(df['robot_pos_x'], df['robot_pos_y'], 
                         c=df['timestamp'], cmap='viridis', s=10)
    
    if 'start_position' in config:
        start = config['start_position'][:2]
        ax1.scatter(start[0], start[1], c='green', s=200, marker='o', 
                   label='Start', zorder=5, edgecolors='black', linewidths=2)
    
    if 'goal_position' in config:
        goal = config['goal_position'][:2]
        ax1.scatter(goal[0], goal[1], c='red', s=200, marker='*', 
                   label='Goal', zorder=5, edgecolors='black', linewidths=2)
    
    plt.colorbar(scatter, ax=ax1, label='Time (s)')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Robot Trajectory (color = time)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # Plot 2: Speed over time
    if 'speed_2d' in df.columns:
        ax2 = plt.subplot(2, 3, 2)
        ax2.plot(df['timestamp'], df['speed_2d'], 'b-', linewidth=1)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Speed (m/s)')
        ax2.set_title('Robot Speed')
        ax2.grid(True, alpha=0.3)
    
    # Plot 3: Distance to goal over time
    if 'distance_to_goal' in df.columns:
        ax3 = plt.subplot(2, 3, 3)
        ax3.plot(df['timestamp'], df['distance_to_goal'], 'r-', linewidth=1)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Distance (m)')
        ax3.set_title('Distance to Goal')
        ax3.grid(True, alpha=0.3)
    
    # Plot 4: X and Y position over time
    ax4 = plt.subplot(2, 3, 4)
    ax4.plot(df['timestamp'], df['robot_pos_x'], label='X', linewidth=1)
    ax4.plot(df['timestamp'], df['robot_pos_y'], label='Y', linewidth=1)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Position (m)')
    ax4.set_title('X and Y Position')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    # Plot 5: Z position over time
    ax5 = plt.subplot(2, 3, 5)
    ax5.plot(df['timestamp'], df['robot_pos_z'], 'g-', linewidth=1)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Z Position (m)')
    ax5.set_title('Height (Z Position)')
    ax5.grid(True, alpha=0.3)
    
    # Plot 6: Yaw angle over time
    if 'robot_yaw_deg' in df.columns:
        ax6 = plt.subplot(2, 3, 6)
        ax6.plot(df['timestamp'], df['robot_yaw_deg'], 'purple', linewidth=1)
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel('Yaw (degrees)')
        ax6.set_title('Robot Orientation (Yaw)')
        ax6.grid(True, alpha=0.3)
    
    plt.tight_layout()
    print("Displaying comprehensive visualization...")
    return fig


def main():
    """Main function."""
    if len(sys.argv) < 2:
        print("Usage: python example_analysis.py <experiment_directory>")
        print("Example: python example_analysis.py 241118_153045-my_experiment")
        sys.exit(1)
    
    experiment_dir = sys.argv[1]
    
    if not Path(experiment_dir).exists():
        print(f"Error: Directory '{experiment_dir}' not found")
        sys.exit(1)
    
    print(f"Loading data from: {experiment_dir}")
    print("="*60)
    
    # Load data
    config, df = load_data(experiment_dir)
    
    # Run examples
    example_basic_statistics(df, config)
    df = example_calculate_speed(df)
    df = example_distance_to_goal(df, config)
    df = example_quaternion_to_yaw(df)
    example_trajectory_analysis(df, config)
    fig = example_visualization(df, config)
    
    # Save the comprehensive plot
    output_path = Path(experiment_dir) / "example_analysis_comprehensive.png"
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\nComprehensive plot saved to: {output_path}")
    
    print("\n" + "="*60)
    print("Analysis complete!")
    print("Close the plot window to exit.")
    print("="*60)
    
    # Show plot
    plt.show()


if __name__ == "__main__":
    main()

