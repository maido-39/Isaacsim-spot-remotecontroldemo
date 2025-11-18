# Experiment Data - Quick Reference Card

## Starting an Experiment

```bash
python quadruped_example.py
# When prompted: Enter experiment name (or press Enter for "NULL")
```

**Important:** Data saving begins automatically when you press the **first keyboard command**. This ensures only active control periods are recorded.

## Output Directory

**Format:** `YYMMDD_HHMMSS-Experiment_Name/`

**Example:** `241118_153045-my_test/`

## What Gets Saved

| Item | Location | Rate | Format |
|------|----------|------|--------|
| Configuration | `config.json` | Once | JSON |
| Robot/Object Data | `data.csv` | 10Hz | CSV |
| Ego Camera Images | `camera/ego/*.jpg` | 10Hz | JPEG |
| Top Camera Images | `camera/top/*.jpg` | 10Hz | JPEG |

## CSV Data Columns

```
timestamp, frame_num,
robot_pos_x, robot_pos_y, robot_pos_z,
robot_orient_w, robot_orient_x, robot_orient_y, robot_orient_z,
object_pos_x, object_pos_y, object_pos_z,
object_orient_w, object_orient_x, object_orient_y, object_orient_z,
l1_distance_to_goal
```

**Quaternion Format:** [w, x, y, z] where w is the real component

**L1 Distance Metric:**
- **Gate or no object:** Robot <-> Goal L1 norm
- **Box/Sphere:** Box/Sphere <-> Goal L1 norm

## Image Naming

**Format:** `frame{N}-{timestamp}-{type}.jpg`

**Examples:**
- `frame0-0.000-ego.jpg` - First frame, ego view
- `frame10-1.000-top.jpg` - 11th frame at 1 second, top view

## Analysis

### Quick Plotting (Recommended)

```bash
python plot_experiment.py
# Press Enter to use latest experiment, or type folder name
```

**Features:**
- 2:1 canvas layout (map + subplots)
- Time-gradient coloring throughout
- Robot/object trajectories with orientations
- Command velocity and L1 distance plots
- Auto-detects latest experiment
- Saves to: `experiment_plot.png`

### Comprehensive Analysis

```bash
python analyze_experiment_data.py 241118_153045-my_test
```

**Outputs:**
- Console: Summary statistics
- File: `analysis_trajectory.png` (robot path)
- File: `analysis_timeseries.png` (speed/distance/yaw)
- Display: Interactive plots

### Custom Analysis with Python

```python
import pandas as pd
import json

# Load data
df = pd.read_csv('241118_153045-my_test/data.csv')
with open('241118_153045-my_test/config.json') as f:
    config = json.load(f)

# Access data
robot_positions = df[['robot_pos_x', 'robot_pos_y', 'robot_pos_z']]
timestamps = df['timestamp']

# Calculate speed
speed = ((df['robot_pos_x'].diff()**2 + 
          df['robot_pos_y'].diff()**2)**0.5 / 
         df['timestamp'].diff())
```

## Quaternion to Euler (Yaw)

For simple yaw extraction from quaternion [w, x, y, z]:

```python
import numpy as np

def quat_to_yaw(w, x, y, z):
    """Extract yaw angle from quaternion"""
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return np.arctan2(siny_cosp, cosy_cosp)

# Apply to dataframe
df['yaw'] = df.apply(lambda row: quat_to_yaw(
    row['robot_orient_w'], row['robot_orient_x'],
    row['robot_orient_y'], row['robot_orient_z']
), axis=1)
```

## Common Analysis Tasks

### 1. Plot Robot Trajectory

```python
import matplotlib.pyplot as plt

plt.figure(figsize=(10, 10))
plt.plot(df['robot_pos_x'], df['robot_pos_y'])
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Robot Trajectory')
plt.grid(True)
plt.axis('equal')
plt.show()
```

### 2. Calculate Distance to Goal

```python
goal_x, goal_y = config['goal_position'][:2]
df['dist_to_goal'] = ((df['robot_pos_x'] - goal_x)**2 + 
                       (df['robot_pos_y'] - goal_y)**2)**0.5
```

### 3. Calculate Robot Speed

```python
dt = df['timestamp'].diff()
dx = df['robot_pos_x'].diff()
dy = df['robot_pos_y'].diff()
df['speed'] = (dx**2 + dy**2)**0.5 / dt
```

### 4. Find Closest Approach to Goal

```python
min_dist = df['dist_to_goal'].min()
closest_frame = df.loc[df['dist_to_goal'].idxmin(), 'frame_num']
closest_time = df.loc[df['dist_to_goal'].idxmin(), 'timestamp']
print(f"Closest: {min_dist:.2f}m at frame {closest_frame} (t={closest_time:.2f}s)")
```

## Data Rates & Storage

| Duration | CSV Size | Images Size | Total |
|----------|----------|-------------|-------|
| 10 sec   | ~10 KB   | ~1 MB       | ~1 MB |
| 1 min    | ~60 KB   | ~60 MB      | ~60 MB |
| 5 min    | ~300 KB  | ~300 MB     | ~300 MB |
| 1 hour   | ~3.6 MB  | ~3.6 GB     | ~3.6 GB |

**Note:** Image size assumes 800x600 JPEG at ~50KB each, 2 cameras at 10Hz

## Tips

1. **Short Names:** Use concise experiment names (e.g., "test1", "high_friction")
2. **Backup Data:** Copy experiment directories to backup storage regularly
3. **CSV Only:** To save disk space, modify code to skip image capture
4. **Batch Analysis:** Process multiple experiments by looping over directories
5. **Real-time Plot:** Use `matplotlib` in interactive mode for live plotting

## Troubleshooting

| Issue | Solution |
|-------|----------|
| No images saved | Check console for camera warnings; may need Replicator API |
| CSV incomplete | Ensure `sim.cleanup()` was called (use try/finally) |
| Out of disk space | Reduce experiment duration or disable camera capture |
| Can't find experiment | Check current directory; experiment dirs created in pwd |

## File Locations

**Main simulation:** `quadruped_example.py`

**Analysis script:** `analyze_experiment_data.py`

**Full documentation:** `EXPERIMENT_DATA_GUIDE.md`

**Technical details:** `EXPERIMENT_DATA_IMPLEMENTATION.md`

## Quick Command Reference

```bash
# Run simulation
python quadruped_example.py

# Quick plot (latest experiment)
python plot_experiment.py

# Plot specific experiment
python plot_experiment.py 241118_153045-my_test

# Comprehensive analysis
python analyze_experiment_data.py <experiment_dir>

# List experiments
ls -d */  # Shows directories (experiments start with YYMMDD_)

# Check experiment size
du -sh 241118_153045-my_test

# View first few CSV rows
head -20 241118_153045-my_test/data.csv

# Count images saved
ls 241118_153045-my_test/camera/ego/*.jpg | wc -l
```

---

**For detailed information, see:** [EXPERIMENT_DATA_GUIDE.md](EXPERIMENT_DATA_GUIDE.md)

