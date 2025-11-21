#!/usr/bin/env python3
"""
Refined Experiment Plot Tool with Integrated Image Viewer

Creates comprehensive visualization of experiment data with:
- Map plot with robot/object trajectories and orientations
- Command velocity subplot
- L1 distance metric subplot
- Interactive image viewer with video player-style timestamp slider
- Time-based gradient coloring

Usage:
    python plot_experiment.py [experiment_directory]
    
If no directory provided, uses the latest experiment folder.
"""

import sys
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow
from matplotlib.collections import LineCollection
from matplotlib.widgets import Slider
from matplotlib import patches
from pathlib import Path
from datetime import datetime
from PIL import Image


def get_latest_experiment_folder():
    """
    Find the latest experiment folder based on timestamp in directory name.
    
    Returns:
        Path object of latest experiment folder, or None if not found
    """
    current_dir = Path('.')
    experiment_dirs = []
    
    for item in current_dir.iterdir():
        if item.is_dir():
            name = item.name
            if len(name) >= 15 and name[6] == '_' and name[13] == '-':
                try:
                    timestamp_str = name[:13]
                    datetime.strptime(timestamp_str, "%y%m%d_%H%M%S")
                    experiment_dirs.append(item)
                except ValueError:
                    continue
    
    if not experiment_dirs:
        return None
    
    experiment_dirs.sort(key=lambda x: x.name, reverse=True)
    return experiment_dirs[0]


def load_experiment_data(experiment_dir):
    """Load experiment data including config and CSV."""
    exp_path = Path(experiment_dir)
    
    config_path = exp_path / "config.json"
    with open(config_path, 'r') as f:
        config = json.load(f)
    
    csv_path = exp_path / "data.csv"
    df = pd.read_csv(csv_path)
    
    return config, df


def load_camera_images(experiment_dir):
    """
    Load camera images from experiment directory.
    
    Returns:
        dict: {'ego': [(timestamp, path), ...], 'top': [(timestamp, path), ...]}
    """
    exp_path = Path(experiment_dir)
    images = {'ego': [], 'top': []}
    
    # Load ego camera images
    ego_dir = exp_path / "camera" / "ego"
    if ego_dir.exists():
        for img_path in sorted(ego_dir.glob("*.jpg")):
            # Parse timestamp from filename: frame{N}-{timestamp}-ego.jpg
            try:
                parts = img_path.stem.split('-')
                if len(parts) >= 2:
                    timestamp = float(parts[1])
                    images['ego'].append((timestamp, img_path))
            except (ValueError, IndexError):
                continue
    
    # Load top camera images
    top_dir = exp_path / "camera" / "top"
    if top_dir.exists():
        for img_path in sorted(top_dir.glob("*.jpg")):
            try:
                parts = img_path.stem.split('-')
                if len(parts) >= 2:
                    timestamp = float(parts[1])
                    images['top'].append((timestamp, img_path))
            except (ValueError, IndexError):
                continue
    
    return images


def find_closest_image(images_list, target_timestamp):
    """Find the image closest to the target timestamp."""
    if not images_list:
        return None
    
    min_diff = float('inf')
    closest_img = None
    
    for timestamp, img_path in images_list:
        diff = abs(timestamp - target_timestamp)
        if diff < min_diff:
            min_diff = diff
            closest_img = img_path
    
    return closest_img


def quaternion_to_yaw(qw, qx, qy, qz):
    """Extract yaw angle from quaternion."""
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    return np.arctan2(siny_cosp, cosy_cosp)


def create_gradient_colormap(timestamps):
    """Create normalized time values for gradient coloring."""
    t_min = timestamps.min()
    t_max = timestamps.max()
    if t_max - t_min > 0:
        return (timestamps - t_min) / (t_max - t_min)
    else:
        return np.zeros_like(timestamps)


def plot_experiment(experiment_dir):
    """
    Create comprehensive experiment visualization with integrated image viewer.
    
    Args:
        experiment_dir: Path to experiment directory
    """
    print(f"Loading experiment data from: {experiment_dir}")
    config, df = load_experiment_data(experiment_dir)
    
    # Handle new data format: multiple rows per timestamp (one per object)
    # Extract unique robot trajectory (group by timestamp, take first row)
    if 'obj_index' in df.columns:
        # New format: multiple objects per timestamp
        df_robot = df.groupby('timestamp').first().reset_index()
        unique_times = df_robot['timestamp'].values
        print(f"Found {len(df_robot)} unique timestamps with {df['obj_index'].nunique()} objects")
    else:
        # Old format: single row per timestamp
        df_robot = df.copy()
        unique_times = df_robot['timestamp'].values
        print(f"Found {len(df_robot)} timestamps (old format)")
    
    # Load camera images
    camera_images = load_camera_images(experiment_dir)
    has_ego = len(camera_images['ego']) > 0
    has_top = len(camera_images['top']) > 0
    has_images = has_ego or has_top
    
    # Create figure with new layout
    # Layout: [Map       | XY Vel ]
    #         [Top | Ego | L1 Norm]
    #         [   Slider         ]
    
    if has_images:
        fig = plt.figure(figsize=(16, 10))
        # Create grid: 3 rows, 3 columns
        gs = fig.add_gridspec(3, 3, height_ratios=[1, 1, 0.1], 
                             width_ratios=[1, 1, 2], 
                             hspace=0.25, wspace=0.25,
                             left=0.08, right=0.96, top=0.94, bottom=0.05)
    else:
        fig = plt.figure(figsize=(16, 8))
        gs = fig.add_gridspec(2, 2, height_ratios=[1, 1], 
                             width_ratios=[1, 1], 
                             hspace=0.25, wspace=0.25,
                             left=0.08, right=0.96, top=0.94, bottom=0.08)
    
    # Create subplots
    if has_images:
        ax_map = fig.add_subplot(gs[0, :2])    # Map: top, left 2 cols
        ax_vel_xy = fig.add_subplot(gs[0, 2])  # XY velocity: top right
        ax_top = fig.add_subplot(gs[1, 0])     # Top camera: bottom left
        ax_ego = fig.add_subplot(gs[1, 1])     # Ego camera: bottom middle
        ax_metric = fig.add_subplot(gs[1, 2])  # L1 metric: bottom right
        ax_slider = fig.add_subplot(gs[2, :])  # Slider: bottom full width
    else:
        ax_map = fig.add_subplot(gs[0, 0])     # Map: top left
        ax_vel_xy = fig.add_subplot(gs[0, 1])  # XY velocity: top right
        ax_metric = fig.add_subplot(gs[1, :])  # L1 metric: bottom full width
    
    # Get time gradient for coloring (using unique timestamps)
    time_norm = create_gradient_colormap(unique_times)
    cmap = plt.cm.viridis
    
    # ==================== MAP PLOT ====================
    
    map_size = config.get('map_size', 10.0)
    half_size = map_size / 2.0
    
    ax_map.set_aspect('equal')
    ax_map.set_xlim(-half_size - 0.5, half_size + 0.5)
    ax_map.set_ylim(-half_size - 0.5, half_size + 0.5)
    
    # Draw boundary walls
    wall_x = [-half_size, half_size, half_size, -half_size, -half_size]
    wall_y = [-half_size, -half_size, half_size, half_size, -half_size]
    ax_map.plot(wall_x, wall_y, 'k--', linewidth=2, alpha=0.5, label='Boundary')
    
    # Draw start and goal points
    if 'start_position' in config:
        start = config['start_position']
        ax_map.scatter(start[0], start[1], c='green', s=300, marker='o', 
                      edgecolors='black', linewidths=2, label='Start', zorder=10)
    
    if 'goal_position' in config:
        goal = config['goal_position']
        ax_map.scatter(goal[0], goal[1], c='red', s=400, marker='*', 
                      edgecolors='black', linewidths=2, label='Goal', zorder=10)
    
    # Plot robot trajectory with gradient (using unique robot data)
    robot_x = df_robot['robot_pos_x'].values
    robot_y = df_robot['robot_pos_y'].values
    
    if len(robot_x) > 1:
        points = np.array([robot_x, robot_y]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        
        lc_robot = LineCollection(segments, cmap=cmap, linewidth=2.5, alpha=0.8)
        lc_robot.set_array(time_norm[:-1])
        ax_map.add_collection(lc_robot)
    
    # Plot robot orientations at intervals
    n_points = len(df_robot)
    arrow_interval = max(1, n_points // 15)
    
    for i in range(0, n_points, arrow_interval):
        row = df_robot.iloc[i]
        yaw = quaternion_to_yaw(row['robot_orient_w'], row['robot_orient_x'],
                                row['robot_orient_y'], row['robot_orient_z'])
        
        arrow_length = 0.5
        dx = arrow_length * np.cos(yaw)
        dy = arrow_length * np.sin(yaw)
        
        color = cmap(time_norm[i])
        arrow = FancyArrow(row['robot_pos_x'], row['robot_pos_y'], dx, dy,
                          width=0.15, head_width=0.3, head_length=0.2,
                          fc=color, ec='black', linewidth=0.5, alpha=0.7, zorder=5)
        ax_map.add_patch(arrow)
    
    # Add current position marker (will be updated by slider)
    current_marker, = ax_map.plot([], [], 'ro', markersize=15, markeredgecolor='white',
                                   markeredgewidth=2, zorder=15, label='Current')
    
    # Plot object trajectories if dynamic objects exist
    object_type = config.get('object_type', 'box')
    num_boxes = config.get('num_boxes', 1)
    
    if object_type in ['box', 'sphere'] and 'obj_index' in df.columns:
        # Plot each object's trajectory separately
        obj_indices = sorted(df['obj_index'].unique())
        object_colors = plt.cm.Set3(np.linspace(0, 1, len(obj_indices)))
        
        for obj_idx, obj_color in zip(obj_indices, object_colors):
            df_obj = df[df['obj_index'] == obj_idx].sort_values('timestamp')
            
            if len(df_obj) == 0:
                continue
            
            obj_x = df_obj['box_pos_x'].values
            obj_y = df_obj['box_pos_y'].values
            obj_times = df_obj['timestamp'].values
            
            # Skip if object position is all zeros (not valid)
            if np.allclose(obj_x, 0.0) and np.allclose(obj_y, 0.0):
                continue
            
            # Plot trajectory
            if len(obj_x) > 1:
                obj_points = np.array([obj_x, obj_y]).T.reshape(-1, 1, 2)
                obj_segments = np.concatenate([obj_points[:-1], obj_points[1:]], axis=1)
                
                obj_time_norm = create_gradient_colormap(obj_times)
                lc_object = LineCollection(obj_segments, cmap=cmap, linewidth=2.0, 
                                          alpha=0.6, linestyle='--')
                lc_object.set_array(obj_time_norm[:-1])
                ax_map.add_collection(lc_object)
            
            # Mark initial and final positions
            ax_map.scatter(obj_x[0], obj_y[0], c=[obj_color], s=200, 
                          marker='s', edgecolors='black', linewidths=1.5,
                          label=f'{object_type.capitalize()} {obj_idx} Start', zorder=8)
            ax_map.scatter(obj_x[-1], obj_y[-1], c=[obj_color], s=200, 
                          marker='^', edgecolors='black', linewidths=1.5,
                          label=f'{object_type.capitalize()} {obj_idx} End', zorder=8)
    
    # Add colorbar (more compact)
    sm = plt.cm.ScalarMappable(cmap=cmap, 
                               norm=plt.Normalize(vmin=unique_times.min(), 
                                                 vmax=unique_times.max()))
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax_map, orientation='horizontal', 
                       pad=0.08, aspect=35, shrink=0.7)
    cbar.set_label('Time (s)', fontsize=10)
    cbar.ax.tick_params(labelsize=8)
    
    ax_map.set_xlabel('X Position (m)', fontsize=11)
    ax_map.set_ylabel('Y Position (m)', fontsize=11)
    ax_map.set_title('Trajectory Map', fontsize=12, fontweight='bold', pad=10)
    ax_map.legend(loc='upper right', fontsize=8, framealpha=0.9)
    ax_map.grid(True, alpha=0.3, linestyle=':')
    ax_map.tick_params(labelsize=9)
    
    # ==================== XY VELOCITY PLOT ====================
    
    # Use unique robot data for velocity calculation
    dt = df_robot['timestamp'].diff()
    dvx = df_robot['robot_pos_x'].diff() / dt
    dvy = df_robot['robot_pos_y'].diff() / dt
    
    yaw = df_robot.apply(lambda row: quaternion_to_yaw(row['robot_orient_w'], 
                                                  row['robot_orient_x'],
                                                  row['robot_orient_y'], 
                                                  row['robot_orient_z']), axis=1)
    dyaw = yaw.diff() / dt
    
    times = unique_times
    
    # Create gradient colored lines for XY velocities
    if len(times) > 2:
        # vx line
        points_vx = np.array([times[1:], dvx[1:]]).T.reshape(-1, 1, 2)
        if len(points_vx) > 1:
            segments_vx = np.concatenate([points_vx[:-1], points_vx[1:]], axis=1)
            lc_vx = LineCollection(segments_vx, cmap=cmap, linewidth=2, alpha=0.8, label='vx')
            lc_vx.set_array(time_norm[1:-1])
            ax_vel_xy.add_collection(lc_vx)
        
        # vy line
        points_vy = np.array([times[1:], dvy[1:]]).T.reshape(-1, 1, 2)
        if len(points_vy) > 1:
            segments_vy = np.concatenate([points_vy[:-1], points_vy[1:]], axis=1)
            lc_vy = LineCollection(segments_vy, cmap=cmap, linewidth=2, alpha=0.8, 
                                   linestyle='--', label='vy')
            lc_vy.set_array(time_norm[1:-1])
            ax_vel_xy.add_collection(lc_vy)
    
    # Set axis limits for XY velocities
    vel_xy_values = np.concatenate([dvx[1:].dropna(), dvy[1:].dropna()])
    if len(vel_xy_values) > 0:
        vel_xy_min, vel_xy_max = vel_xy_values.min(), vel_xy_values.max()
        vel_xy_range = vel_xy_max - vel_xy_min
        ax_vel_xy.set_xlim(times.min(), times.max())
        if vel_xy_range > 0:
            ax_vel_xy.set_ylim(vel_xy_min - 0.1 * vel_xy_range, vel_xy_max + 0.1 * vel_xy_range)
        else:
            ax_vel_xy.set_ylim(vel_xy_min - 0.1, vel_xy_max + 0.1)
    
    # Add vertical line for current time
    vel_xy_vline = ax_vel_xy.axvline(x=times[0], color='red', linestyle='--', linewidth=2, alpha=0.7)
    
    ax_vel_xy.set_xlabel('Time (s)', fontsize=10)
    ax_vel_xy.set_ylabel('Velocity (m/s)', fontsize=10)
    ax_vel_xy.set_title('XY Velocities', fontsize=11, fontweight='bold', pad=8)
    ax_vel_xy.legend(loc='upper right', fontsize=8, framealpha=0.9)
    ax_vel_xy.grid(True, alpha=0.3)
    ax_vel_xy.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
    ax_vel_xy.tick_params(labelsize=9)
    
    # ==================== L1 DISTANCE METRIC PLOT ====================
    
    # Handle L1 distance: if multiple objects, show min/max/avg
    if 'box_l1_distance' in df.columns and 'obj_index' in df.columns:
        # New format: calculate aggregate metrics per timestamp
        l1_by_time = df.groupby('timestamp')['box_l1_distance'].agg(['min', 'max', 'mean']).reset_index()
        l1_min = l1_by_time['min'].values
        l1_max = l1_by_time['max'].values
        l1_mean = l1_by_time['mean'].values
        l1_times = l1_by_time['timestamp'].values
        
        # Plot min, max, and mean
        if len(l1_times) > 1:
            # Min distance (primary metric)
            points_min = np.array([l1_times, l1_min]).T.reshape(-1, 1, 2)
            segments_min = np.concatenate([points_min[:-1], points_min[1:]], axis=1)
            lc_min = LineCollection(segments_min, cmap=cmap, linewidth=2.5, alpha=0.9, label='Min')
            lc_min.set_array(create_gradient_colormap(l1_times)[:-1])
            ax_metric.add_collection(lc_min)
            
            # Mean distance
            points_mean = np.array([l1_times, l1_mean]).T.reshape(-1, 1, 2)
            segments_mean = np.concatenate([points_mean[:-1], points_mean[1:]], axis=1)
            lc_mean = LineCollection(segments_mean, cmap=cmap, linewidth=2.0, 
                                    alpha=0.7, linestyle='--', label='Mean')
            lc_mean.set_array(create_gradient_colormap(l1_times)[:-1])
            ax_metric.add_collection(lc_mean)
            
            # Max distance
            points_max = np.array([l1_times, l1_max]).T.reshape(-1, 1, 2)
            segments_max = np.concatenate([points_max[:-1], points_max[1:]], axis=1)
            lc_max = LineCollection(segments_max, cmap=cmap, linewidth=1.5, 
                                    alpha=0.5, linestyle=':', label='Max')
            lc_max.set_array(create_gradient_colormap(l1_times)[:-1])
            ax_metric.add_collection(lc_max)
        
        l1_dist = l1_min  # Use min for primary display
        metric_times = l1_times
    elif 'l1_distance_to_goal' in df_robot.columns:
        # Old format: single L1 distance column
        l1_dist = df_robot['l1_distance_to_goal'].values
        metric_times = times
    else:
        # Calculate from robot position
        goal = config.get('goal_position', [0, 0])
        l1_dist = np.abs(robot_x - goal[0]) + np.abs(robot_y - goal[1])
        metric_times = times
    
    ax_metric.set_xlim(metric_times.min(), metric_times.max())
    if len(l1_dist) > 0:
        ax_metric.set_ylim(0, l1_dist.max() * 1.1)
    
    # Add vertical line for current time
    metric_vline = ax_metric.axvline(x=metric_times[0], color='red', linestyle='--', linewidth=2, alpha=0.7)
    
    ax_metric.axhline(y=1.0, color='green', linestyle='--', linewidth=1.5, 
                     alpha=0.5, label='Success (1m)')
    
    ax_metric.set_xlabel('Time (s)', fontsize=10)
    ax_metric.set_ylabel('L1 Distance (m)', fontsize=10)
    ax_metric.set_title('L1 Distance to Goal', fontsize=11, fontweight='bold', pad=8)
    ax_metric.legend(loc='upper right', fontsize=8, framealpha=0.9)
    ax_metric.grid(True, alpha=0.3)
    ax_metric.tick_params(labelsize=9)
    
    # Add text annotation for current distance (more compact)
    if len(l1_dist) > 0:
        metric_text = ax_metric.text(0.02, 0.98, f'Now: {l1_dist[0]:.2f}m', 
                      transform=ax_metric.transAxes, fontsize=9,
                      verticalalignment='top', bbox=dict(boxstyle='round,pad=0.4', 
                      facecolor='wheat', alpha=0.8))
    else:
        metric_text = ax_metric.text(0.02, 0.98, 'Now: N/A', 
                      transform=ax_metric.transAxes, fontsize=9,
                      verticalalignment='top', bbox=dict(boxstyle='round,pad=0.4', 
                      facecolor='wheat', alpha=0.8))
    
    # Store variables for slider update function
    # (metric_times and l1_dist are already defined above)
    
    # ==================== IMAGE VIEWER ====================
    
    if has_images:
        # Initialize top camera view
        ax_top.axis('off')
        ax_top.set_title('Top Camera', fontsize=11, fontweight='bold', pad=5)
        top_img_obj = None
        
        if has_top:
            # Load first image and maintain aspect ratio
            first_top = camera_images['top'][0][1]
            img = Image.open(first_top)
            top_img_obj = ax_top.imshow(img)
            ax_top.set_aspect('equal')
        else:
            ax_top.text(0.5, 0.5, 'No top images', 
                       ha='center', va='center', fontsize=12, color='gray')
        
        # Initialize ego camera view
        ax_ego.axis('off')
        ax_ego.set_title('Ego Camera', fontsize=11, fontweight='bold', pad=5)
        ego_img_obj = None
        
        if has_ego:
            # Load first image and maintain aspect ratio
            first_ego = camera_images['ego'][0][1]
            img = Image.open(first_ego)
            ego_img_obj = ax_ego.imshow(img)
            ax_ego.set_aspect('equal')
        else:
            ax_ego.text(0.5, 0.5, 'No ego images', 
                       ha='center', va='center', fontsize=12, color='gray')
        
        # ==================== TIMESTAMP SLIDER ====================
        
        ax_slider.clear()
        ax_slider.axis('off')
        
        # Create slider axes (optimized position)
        slider_ax = fig.add_axes([0.12, 0.02, 0.76, 0.025])
        
        time_slider = Slider(
            slider_ax, 'Time', 
            times.min(), times.max(),
            valinit=times.min(),
            valstep=None,  # Allow continuous values
            color='skyblue'
        )
        slider_ax.tick_params(labelsize=8)
        
        # Update function for slider
        def update_slider(val):
            current_time = time_slider.val
            
            # Find closest data point in unique timestamps
            idx = np.argmin(np.abs(times - current_time))
            
            # Update current position marker on map
            if idx < len(robot_x) and idx < len(robot_y):
                current_marker.set_data([robot_x[idx]], [robot_y[idx]])
            
            # Update vertical lines on plots
            vel_xy_vline.set_xdata([current_time])
            metric_vline.set_xdata([current_time])
            
            # Update metric text - find closest in metric_times
            metric_idx = np.argmin(np.abs(metric_times - current_time))
            if metric_idx < len(l1_dist):
                metric_text.set_text(f'Now: {l1_dist[metric_idx]:.2f}m')
            
            # Update camera images
            if has_top and top_img_obj is not None:
                top_img_path = find_closest_image(camera_images['top'], current_time)
                if top_img_path:
                    img = Image.open(top_img_path)
                    top_img_obj.set_data(img)
            
            if has_ego and ego_img_obj is not None:
                ego_img_path = find_closest_image(camera_images['ego'], current_time)
                if ego_img_path:
                    img = Image.open(ego_img_path)
                    ego_img_obj.set_data(img)
            
            fig.canvas.draw_idle()
        
        time_slider.on_changed(update_slider)
        
        # Initialize with first frame
        update_slider(times.min())
    
    # ==================== OVERALL TITLE ====================
    
    fig.suptitle(f'Experiment: {Path(experiment_dir).name}', 
                fontsize=13, fontweight='bold', y=0.98)
    
    # Save figure
    output_path = Path(experiment_dir) / "experiment_plot.png"
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\nPlot saved to: {output_path}")
    
    return fig


def main():
    """Main function."""
    print("="*60)
    print("Experiment Plot Tool with Integrated Image Viewer")
    print("="*60)
    
    # Get experiment directory
    if len(sys.argv) >= 2:
        experiment_dir = sys.argv[1]
        print(f"Using specified directory: {experiment_dir}")
    else:
        user_input = input("\nEnter experiment directory (or press Enter for latest): ").strip()
        
        if user_input:
            experiment_dir = user_input
        else:
            experiment_dir = get_latest_experiment_folder()
            if experiment_dir is None:
                print("Error: No experiment folders found in current directory")
                sys.exit(1)
            print(f"Using latest experiment: {experiment_dir}")
    
    if not Path(experiment_dir).exists():
        print(f"Error: Directory '{experiment_dir}' not found")
        sys.exit(1)
    
    # Create plot
    try:
        fig = plot_experiment(experiment_dir)
        
        print("\n" + "="*60)
        print("Plot complete!")
        print("Use the slider at the bottom to navigate through time.")
        print("The images and current position marker will update automatically.")
        print("Close the plot window to exit.")
        print("="*60)
        
        plt.show()
        
    except Exception as e:
        print(f"Error creating plot: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
