# Experiment Data Saving Guide

This guide explains how to use the experiment data saving functionality in the Spot Robot simulation.

## Overview

The simulation now automatically saves experiment data including:
- Configuration used in the experiment
- Robot and object state data (position, orientation)
- Camera images from both ego-view and top-view cameras

## Quick Start

1. **Run the simulation:**
   ```bash
   python quadruped_example.py
   ```

2. **Enter experiment name:**
   When prompted, enter a descriptive name for your experiment. Press Enter without typing anything to use "NULL" as the default name.
   ```
   Enter experiment name (press Enter for 'NULL'): my_first_experiment
   ```

3. **Run your experiment:**
   Control the robot as normal using the keyboard controls.

4. **Stop the simulation:**
   Press ESC to stop. All data will be automatically saved and the CSV file will be closed.

## Folder Structure

After running an experiment, a new directory will be created with the following structure:

```
YYMMDD_HHMMSS-Experiment_Name/
├── config.json           # Configuration used in the experiment
├── data.csv             # Time-series data (robot & object states)
├── terminal.log         # Complete terminal/console output log
└── camera/
    ├── ego/             # Ego-view camera images
    │   ├── frame0-0.000-ego.jpg
    │   ├── frame1-0.100-ego.jpg
    │   └── ...
    └── top/             # Top-view camera images
        ├── frame0-0.000-top.jpg
        ├── frame1-0.100-top.jpg
        └── ...
```

### Directory Naming

- **Timestamp format:** `YYMMDD_HHMMSS` (e.g., `241118_153045` for Nov 18, 2024 at 3:30:45 PM)
- **Full directory name:** `241118_153045-my_first_experiment`

## Data Files

### 1. config.json

Contains the actual configuration values used in the experiment. This includes:
- Environment parameters (map size, wall height, colors)
- Physics parameters (friction, restitution)
- Robot parameters (start position, goal position, orientation)
- Object parameters (position, scale, mass, type)
- Controller settings (max velocity, acceleration, decay)

**Note:** Randomization range parameters are excluded. Only the actual values used in the experiment are saved.

### 2. data.csv

Time-series data saved at 10Hz (every 0.1 seconds). Columns include:

| Column | Description |
|--------|-------------|
| `timestamp` | Elapsed time since first keyboard command (seconds) |
| `frame_num` | Frame number (increments each save) |
| `robot_pos_x`, `robot_pos_y`, `robot_pos_z` | Robot base position in world coordinates (meters) |
| `robot_orient_w`, `robot_orient_x`, `robot_orient_y`, `robot_orient_z` | Robot base orientation quaternion (w, x, y, z) |
| `object_pos_x`, `object_pos_y`, `object_pos_z` | Object position in world coordinates (meters) |
| `object_orient_w`, `object_orient_x`, `object_orient_y`, `object_orient_z` | Object orientation quaternion (w, x, y, z) |
| `l1_distance_to_goal` | L1 (Manhattan) distance to goal (meters) |

**Quaternion Format:** [w, x, y, z] where w is the real component

**L1 Distance Metric:**
- For **gate** or **no object** scenarios: Robot <-> Goal L1 distance
- For **box/sphere** objects: Box/Sphere <-> Goal L1 distance

**Object Handling:**
- For dynamic objects (box, sphere): Position and orientation are updated each frame
- For static objects (gate): Position and orientation are constant
- For "no object" scenarios: Object position is [0, 0, 0] and orientation is identity quaternion [1, 0, 0, 0]

### 3. terminal.log

Complete log of all console/terminal output during the experiment. This includes:
- Timestamp for each log entry
- Log level (INFO, WARNING, ERROR, etc.)
- All system messages: initialization, setup, errors, warnings
- Camera setup details
- Physics simulation status
- File save confirmations
- Cleanup messages

**Format:** Same as terminal output with timestamp prefix
```
2024-11-18 15:30:45 - SpotSimulation - INFO - Experiment directory created: 241118_153045-my_experiment
2024-11-18 15:30:46 - SpotSimulation - INFO - ✓ Camera 'EgoCamera' created:
2024-11-18 15:30:46 - SpotSimulation - INFO -   Resolution: 1280×800 (aspect ratio: 0.625)
```

**Use Cases:**
- Debugging: Track when errors or warnings occurred
- Performance analysis: Check timing of operations
- Reproducibility: Verify exact sequence of events
- Documentation: Complete record of experiment execution

---

**Data Saving Trigger:**
- Data saving begins automatically when you press the first keyboard command
- This ensures only active control periods are recorded
- Timestamp is relative to when first command was received

### 4. Camera Images

Images are saved at 10Hz (synchronized with CSV data) in JPEG format.

**Filename format:** `frame{N}-{timestamp}-{camera_type}.jpg`
- `N`: Frame number (0, 1, 2, ...)
- `timestamp`: Elapsed time in seconds (e.g., 0.000, 0.100, 0.200)
- `camera_type`: Either "ego" or "top"

**Examples:**
- `frame0-0.000-ego.jpg` - First ego-view frame at t=0.000s
- `frame1-0.100-top.jpg` - Second top-view frame at t=0.100s
- `frame42-4.200-ego.jpg` - 43rd ego-view frame at t=4.200s

## Usage Examples

### Example 1: Basic Usage with Default Configuration

```python
# In main()
experiment_name = input("Enter experiment name: ")
sim = SpotSimulation(experiment_name=experiment_name)
sim.setup()
sim.run()
sim.cleanup()
```

### Example 2: Custom Configuration

```python
# With custom configuration
experiment_name = "high_friction_test"
sim = SpotSimulation(
    experiment_name=experiment_name,
    randomize=False,
    box_friction_static=1.5,
    box_friction_dynamic=1.2
)
sim.setup()
sim.run()
sim.cleanup()
```

### Example 3: Loading Configuration from File

```python
# Load from JSON config file
experiment_name = "config_based_test"
sim = SpotSimulation(
    config_file="example_config.json",
    experiment_name=experiment_name
)
sim.setup()
sim.run()
sim.cleanup()
```

## Data Collection Rate

- **CSV data:** 10Hz (0.1 second intervals)
- **Camera images:** 10Hz (synchronized with CSV data)
- **Physics simulation:** 500Hz (internal)

## Camera Image Capture

The system attempts to capture images using Isaac Sim's Replicator API. If Replicator is not available, it falls back to viewport-based capture.

**Resolution:** 800x600 pixels (configurable in `_save_camera_image()`)

**Format:** JPEG with 95% quality

## CSV Data Analysis Example

```python
import pandas as pd
import numpy as np

# Load CSV data
df = pd.read_csv('241118_153045-my_experiment/data.csv')

# Calculate robot speed
df['robot_speed'] = np.sqrt(
    df['robot_pos_x'].diff()**2 + 
    df['robot_pos_y'].diff()**2
) / df['timestamp'].diff()

# Calculate distance to goal
goal_x, goal_y = -4.0, -4.0  # From config
df['distance_to_goal'] = np.sqrt(
    (df['robot_pos_x'] - goal_x)**2 + 
    (df['robot_pos_y'] - goal_y)**2
)

# Plot robot trajectory
import matplotlib.pyplot as plt
plt.figure(figsize=(10, 10))
plt.plot(df['robot_pos_x'], df['robot_pos_y'])
plt.scatter(goal_x, goal_y, c='red', s=100, label='Goal')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Robot Trajectory')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()
```

## Quaternion to Euler Conversion

If you need to convert quaternions to Euler angles (roll, pitch, yaw):

```python
import numpy as np

def quaternion_to_euler(qw, qx, qy, qz):
    """Convert quaternion to Euler angles (roll, pitch, yaw) in radians"""
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

# Apply to dataframe
df['robot_roll'], df['robot_pitch'], df['robot_yaw'] = zip(*df.apply(
    lambda row: quaternion_to_euler(
        row['robot_orient_w'], row['robot_orient_x'],
        row['robot_orient_y'], row['robot_orient_z']
    ), axis=1
))
```

## Troubleshooting

### Images Not Saving

If camera images are not being saved:
1. Check console logs for warnings about image capture
2. Verify that cameras are properly initialized (check logs for "Camera created" messages)
3. The system will fall back to debug logging if image capture fails
4. Image capture depends on Isaac Sim's Replicator API - ensure you have the latest version

### CSV File Empty or Incomplete

- The CSV file is flushed every 10 frames
- Make sure to properly call `sim.cleanup()` to ensure all data is written
- If the simulation crashes, data may be lost since the last flush

### Experiment Directory Not Created

- Check that you have write permissions in the current directory
- Check console logs for error messages during directory creation
- The directory is created during `sim.setup()`, after world reset

## Notes

1. **Data Storage:** Experiment data can be large, especially camera images. Plan storage accordingly.
   - ~2MB per second for images (800x600 JPEG at 10Hz for 2 cameras)
   - ~1KB per second for CSV data

2. **Performance:** Image capture may slightly reduce simulation performance. Adjust capture rate if needed by modifying the logging counter frequency.

3. **Synchronization:** CSV data and camera images are synchronized at 10Hz. Frame numbers in filenames match the `frame_num` column in CSV.

4. **Data Integrity:** Always use `sim.cleanup()` in a try-finally block to ensure data is properly saved even if the simulation encounters an error.

## Future Enhancements

Possible improvements to consider:
- Configurable data capture rate
- Option to disable camera capture for faster simulation
- Compressed video output instead of individual images
- Real-time data visualization during simulation
- Automatic backup to cloud storage
- Additional sensors (lidar, depth, etc.)

