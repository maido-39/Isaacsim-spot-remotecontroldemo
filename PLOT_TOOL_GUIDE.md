# Refined Experiment Plot Tool - User Guide

## Overview

The **Refined Experiment Plot Tool** (`plot_experiment.py`) creates comprehensive visualizations of experiment data with an optimized layout and time-gradient coloring throughout all plots.

## Features

### 🎨 Visual Design
- **Optimized Canvas Layout** (16:10 inches with images, 16:8 without)
- **Time-Gradient Coloring** using viridis colormap for all trajectories and data
- **Equal Aspect Ratio** for accurate spatial representation
- **Compact Professional Styling** with optimized margins and text sizing
- **No Overlapping Elements** - carefully positioned labels and legends

### 📊 Plot Components

#### Left Side: Map Plot
- Robot trajectory with orientation arrows
- Object trajectory (for dynamic objects like box/sphere)
- Start position (green circle)
- Goal position (red star)
- Environment boundaries (wall size)
- Time colorbar showing temporal progression

#### Right Side: Two Subplots

**Top Subplot: Command Velocities**
- Estimated vx, vy velocities
- Angular velocity (yaw rate)
- Time-gradient colored scatter points

**Bottom Subplot: L1 Distance Metric**
- L1 distance to goal over time
- Success threshold line (1 meter)
- Final distance annotation
- Time-gradient colored line

## Usage

### Basic Usage

```bash
# Auto-detect latest experiment
python plot_experiment.py

# Specify experiment folder
python plot_experiment.py 241118_153045-my_experiment
```

### Interactive Prompt

If run without arguments, the tool will prompt:

```
Experiment Plot Tool
============================================================

Enter experiment directory (or press Enter for latest):
```

- **Press Enter**: Uses the latest experiment folder automatically
- **Type folder name**: Uses the specified experiment folder

### Command Line Arguments

```bash
# Use latest experiment
python plot_experiment.py

# Use specific experiment
python plot_experiment.py 241118_153045-test1

# Use relative path
python plot_experiment.py ./experiments/241118_153045-test1

# Use absolute path
python plot_experiment.py /full/path/to/experiment
```

## Output

### Saved File

The plot is automatically saved as:
```
<experiment_folder>/experiment_plot.png
```

**Resolution:** 150 DPI (publication quality)

**Size:** ~500-800 KB (depending on complexity)

### Display Window

An interactive matplotlib window opens showing the complete visualization:
- Pan and zoom using toolbar
- Save with custom settings using save button
- Close window to exit

## Plot Details

### Map Plot Features

**Robot Trajectory:**
- Colored line showing robot path (blue → yellow gradient = time)
- Orientation arrows at regular intervals
- Arrow color matches time gradient
- Arrow points in heading direction

**Object Trajectory** (for box/sphere):
- Dashed line showing object path
- Orientation arrows (smaller than robot)
- Start position marker (orange square)
- End position marker (brown square)

**Environment Elements:**
- Boundary walls (black dashed lines)
- Start point (green circle, size 300)
- Goal point (red star, size 400)
- Equal aspect ratio (1:1 scale)
- Wall size boundaries from config

**Colorbar:**
- Horizontal orientation below map
- Shows time in seconds
- Matches trajectory gradient

### Command Velocity Plot

**Data:**
- vx: Forward/backward velocity (circles)
- vy: Left/right velocity (squares)
- ω: Yaw rate / angular velocity (triangles)

**Note:** Velocities are **estimated** from position differences:
```python
vx = Δx / Δt
vy = Δy / Δt
ω = Δyaw / Δt
```

**Features:**
- Time-gradient colored scatter points
- Zero reference line
- Legend in upper right
- Grid for easy reading

### L1 Distance Metric Plot

**Data:**
- L1 distance from CSV (if available)
- Or calculated as: |robot_x - goal_x| + |robot_y - goal_y|

**Features:**
- Time-gradient colored line
- Success threshold at 1.0 meter (green dashed line)
- Final distance annotation in text box
- Grid for easy reading
- Y-axis starts at 0

## Latest Experiment Detection

The tool automatically finds the latest experiment using:

1. **Directory Pattern**: `YYMMDD_HHMMSS-ExperimentName`
2. **Timestamp Parsing**: Extracts and sorts by timestamp
3. **Selection**: Returns the most recent folder

**Example:**
```
.
├── 241118_150000-test1/     # Older
├── 241118_153000-test2/     # Middle
└── 241118_160000-test3/     # Latest ← This one selected
```

## Time Gradient Coloring

All plots use consistent time-based gradient coloring:

**Colormap:** viridis (blue → green → yellow)
- **Blue**: Early in experiment (t = 0)
- **Green**: Middle of experiment
- **Yellow**: End of experiment (t = max)

**Applied to:**
- Robot trajectory line
- Object trajectory line (if present)
- Robot orientation arrows
- Object orientation arrows
- Velocity scatter points
- L1 distance line

**Normalization:**
```python
time_normalized = (t - t_min) / (t_max - t_min)
color = viridis(time_normalized)
```

## Requirements

### Python Packages

```bash
pip install numpy pandas matplotlib
```

**Package Versions:**
- numpy >= 1.20
- pandas >= 1.3
- matplotlib >= 3.4

### Data Requirements

**Required Files in Experiment Folder:**
- `config.json` - Configuration with map_size, positions
- `data.csv` - Time-series data with robot/object poses

**CSV Columns Required:**
- timestamp
- robot_pos_x, robot_pos_y
- robot_orient_w, robot_orient_x, robot_orient_y, robot_orient_z
- object_pos_x, object_pos_y (for object trajectory)
- object_orient_w, object_orient_x, object_orient_y, object_orient_z (for object)
- l1_distance_to_goal (optional, will calculate if missing)

## Examples

### Example 1: Quick Visualization

```bash
# Run experiment
python quadruped_example.py
# Enter: "quick_test"

# Immediately plot (uses latest = quick_test)
python plot_experiment.py
# Press Enter at prompt
```

### Example 2: Compare Multiple Experiments

```bash
# Plot experiment 1
python plot_experiment.py 241118_150000-test1
# Save or screenshot the plot

# Plot experiment 2
python plot_experiment.py 241118_153000-test2
# Compare visually
```

### Example 3: Custom Analysis Pipeline

```python
import subprocess
from pathlib import Path

# Run multiple experiments
for i in range(3):
    # Run experiment (would need automation)
    subprocess.run(['python', 'quadruped_example.py'])
    
    # Plot latest
    subprocess.run(['python', 'plot_experiment.py'])
    
    # Rename plot
    latest_exp = get_latest_experiment()
    plot_path = Path(latest_exp) / 'experiment_plot.png'
    plot_path.rename(f'results/experiment_{i}_plot.png')
```

## Customization

### Modify Plot Appearance

Edit `plot_experiment.py`:

```python
# Change figure size (default: 16x8)
fig = plt.figure(figsize=(20, 10))

# Change colormap (default: viridis)
cmap = plt.cm.plasma  # or coolwarm, jet, etc.

# Change arrow interval (default: ~15 arrows)
arrow_interval = max(1, n_points // 20)  # Show 20 arrows

# Change DPI (default: 150)
plt.savefig(output_path, dpi=300)  # Higher quality
```

### Add Custom Annotations

```python
# Add text annotation
ax_map.text(x, y, 'Custom label', fontsize=12, color='red')

# Add custom marker
ax_map.scatter(x, y, c='purple', s=100, marker='D')

# Add reference line
ax_map.axhline(y=0, color='k', linestyle='--', alpha=0.3)
```

## Troubleshooting

### Common Issues

**Issue: "No experiment folders found"**
- Solution: Run from directory containing experiment folders
- Or specify full path to experiment

**Issue: "Directory not found"**
- Check spelling of folder name
- Ensure folder exists in current directory
- Try absolute path

**Issue: Plot looks cluttered**
- Reduce arrow_interval in code (show fewer arrows)
- Increase figure size
- Zoom in on map plot using toolbar

**Issue: Time gradient not visible**
- Experiment may be very short (< 1 second)
- Try running longer experiments
- Check that timestamps are varying

**Issue: Object trajectory not shown**
- Check if use_object=True in config
- Verify object_type is 'box' or 'sphere' (not 'gate')
- Check that object actually moved (not at origin)

### Error Messages

**"KeyError: 'l1_distance_to_goal'"**
- Old CSV format (before L1 distance update)
- Tool will automatically calculate from robot position
- Not an error, just a fallback

**"ValueError: could not convert string to float"**
- CSV file may be corrupted
- Check CSV file opens correctly in text editor
- Verify no missing values in CSV

**"FileNotFoundError: config.json"**
- Experiment folder incomplete
- Ensure both config.json and data.csv exist
- Check folder structure is correct

## Performance

**Typical Performance:**
- 10-second experiment: ~1 second to plot
- 60-second experiment: ~2 seconds to plot
- 300-second experiment: ~5 seconds to plot

**Memory Usage:**
- Small experiment (<1000 points): ~50 MB
- Medium experiment (~6000 points): ~100 MB
- Large experiment (>18000 points): ~200 MB

## Tips and Best Practices

1. **Use Latest Feature**: Save time by pressing Enter for latest experiment

2. **Regular Plotting**: Plot after each experiment to catch issues early

3. **Save Screenshots**: Use matplotlib save button for custom resolution

4. **Compare Experiments**: Open multiple terminal windows to plot different experiments side-by-side

5. **Check Boundaries**: Verify robot stayed within walls (should be inside boundary lines)

6. **Monitor Progress**: L1 distance should generally decrease over time

7. **Orientation Arrows**: Check that arrows point in sensible directions

8. **Time Gradient**: Verify smooth color progression (no jumps = consistent data rate)

## Integration with Workflow

### Typical Experiment Workflow

```bash
# 1. Run experiment
python quadruped_example.py
# Enter experiment name: "baseline_test"

# 2. Quick plot check
python plot_experiment.py
# Press Enter (uses latest)

# 3. Review plot
# - Check trajectory makes sense
# - Verify L1 distance decreased
# - Look for anomalies

# 4. If good, proceed to comprehensive analysis
python analyze_experiment_data.py 241118_153045-baseline_test

# 5. Save results
mkdir results
cp 241118_153045-baseline_test/experiment_plot.png results/
cp 241118_153045-baseline_test/data.csv results/
```

## Comparison with Other Analysis Tools

| Feature | plot_experiment.py | analyze_experiment_data.py | example_analysis.py |
|---------|-------------------|---------------------------|---------------------|
| Speed | Fast (~2s) | Medium (~5s) | Slow (~10s) |
| Layout | Optimized 2:1 | Multiple figures | 6-subplot grid |
| Gradient | Yes, all plots | No | Yes, some plots |
| Auto-latest | Yes | No | No |
| Orientations | Yes, arrows | No | No |
| Statistics | No | Yes, detailed | Yes, basic |
| Best for | Quick check | Full analysis | Learning |

**Recommendation:** 
- Use `plot_experiment.py` for quick checks after each experiment
- Use `analyze_experiment_data.py` for detailed analysis and statistics
- Use `example_analysis.py` for learning and custom analysis

## Advanced Features

### Batch Processing

```bash
# Plot all experiments
for dir in */; do
    if [[ $dir =~ ^[0-9]{6}_[0-9]{6}- ]]; then
        python plot_experiment.py "$dir"
    fi
done
```

### Automated Comparison

```python
# Create comparison of multiple experiments
import matplotlib.pyplot as plt
from pathlib import Path

experiments = ['241118_150000-test1', '241118_153000-test2']

fig, axes = plt.subplots(1, len(experiments), figsize=(16, 8))

for i, exp in enumerate(experiments):
    img = plt.imread(Path(exp) / 'experiment_plot.png')
    axes[i].imshow(img)
    axes[i].set_title(exp)
    axes[i].axis('off')

plt.savefig('comparison.png', dpi=150, bbox_inches='tight')
```

## Summary

**plot_experiment.py** is the recommended tool for quick visualization of experiment results with:
- ✅ Fast execution
- ✅ Automatic latest experiment detection
- ✅ Comprehensive single-view layout
- ✅ Time-gradient coloring throughout
- ✅ Robot and object orientations
- ✅ Publication-quality output

**Perfect for:** Quick checks, progress monitoring, presentations

**Use when:** You need a fast, clear overview of an experiment

---

**For more information:**
- Detailed data analysis: `analyze_experiment_data.py`
- Data format reference: `EXPERIMENT_DATA_GUIDE.md`
- Quick reference: `EXPERIMENT_DATA_QUICKREF.md`

