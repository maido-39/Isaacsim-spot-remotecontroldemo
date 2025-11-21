# Isaac Sim Spot Remote Control Demo

A comprehensive simulation project for controlling a Boston Dynamics Spot quadruped robot in NVIDIA Isaac Sim with keyboard input, automatic experiment data collection, performance monitoring, and advanced visualization tools.

> **한국어 버전**: [README_KR.md](README_KR.md)를 참조하세요.

## Overview

This project provides a complete simulation environment for the Spot robot featuring:
- **Real-time keyboard control** with smooth acceleration/decay model
- **Automatic experiment data collection** (CSV, camera images, configuration)
- **Performance monitoring** with real-time statistics (physics, rendering, frame rates)
- **Multiple object types** (box, sphere, gate) for various task scenarios
- **Dual camera system** (ego-view and top-down) with synchronized image capture
- **Interactive visualization tool** for analyzing experiment results
- **Environment randomization** for varied training scenarios
- **Performance optimization** options to disable logging/saving for maximum speed

## Features

### Core Simulation (`quadruped_example.py`)
- 🤖 **Spot Robot Control**: Full control using Isaac Sim's `SpotFlatTerrainPolicy`
- ⌨️ **Keyboard Controller**: Thread-safe keyboard input with smooth velocity ramping
- 🎲 **Environment Randomization**: Configurable randomization of start/goal positions and objects
- 📦 **Multiple Object Types**: Support for dynamic boxes, spheres, and static gates
- 📊 **Real-time State Tracking**: Robot and object positions/orientations at 10Hz
- 💾 **Automatic Data Saving**: CSV logs, camera images, and configuration files (can be disabled)
- 🎥 **Dual Camera System**: Ego-view (robot-mounted) and top-down (overhead) cameras with Pygame display
- 📏 **L1 Distance Metrics**: Task completion tracking based on object type
- 🎯 **Task Completion Detection**: Automatic goal color change when task is complete
- ⚡ **Performance Monitoring**: Real-time performance statistics (physics, rendering, frame rates) logged at 1Hz
- 🔧 **Gate Transform**: Automatic gate positioning with manual transform application support
- 🚀 **Performance Optimization**: CLI flags to disable CSV logging and image saving for maximum speed

### Visualization Tool (`plot_experiment.py`)
- 📈 **Comprehensive Plots**: Trajectory maps, velocity plots, and distance metrics
- 🎨 **Time Gradient Visualization**: Color-coded trajectories showing temporal progression
- 🖼️ **Integrated Image Viewer**: Synchronized ego and top camera image display (if images were saved)
- 🎚️ **Interactive Timestamp Slider**: Navigate through experiment timeline
- 🔄 **Automatic Latest Detection**: Finds most recent experiment if no directory specified
- 💾 **Auto-save**: Saves visualization as `experiment_plot.png` in experiment directory

## Prerequisites

- **NVIDIA Isaac Sim** (version 4.5.0 or compatible)
- **Python 3.8+**
- **Conda environment** with Isaac Sim packages installed
- **Required Python packages**: `pygame`, `numpy`, `pandas`, `matplotlib`, `pillow`

## Installation
0. H/W & Nvidia driver & OS 
- Tested @
   - RTX 2060 12G
   - NVIDIA-SMI 535.216.01
   - Driver Version: 535.216.01
   - CUDA Version: 12.2  
   - Ubuntu 22.04.5 LTS

1. **Create conda environment**:
   ```bash
   # Create a new conda environment (if not already created)
   conda create -n isc-demo python=3.10 -y && conda activate isc-demo
   ```

2. **Install Python dependencies & Isaacsim 4.5.0**:
   ```bash
   # Install Isaacsim
   pip install "isaacsim[all,extscache]==4.5.0" --extra-index-url https://pypi.nvidia.com
   # Install Deps
   pip install pygame numpy pandas matplotlib pillow
   ```

4. **Verify installation**:
   ```bash
   python -c "from isaacsim import SimulationApp; print('Isaac Sim ready')"
   ```

## Quick Start

### Running the Main Simulation

```bash
conda activate isc-demo
python quadruped_example.py
```

When prompted, enter an experiment name (or press Enter for 'NULL'):
```
Enter experiment name (press Enter for 'NULL'): my_experiment
```

### Command-Line Interface (CLI)

The `quadruped_example.py` script supports various command-line arguments for customization:

#### Basic Usage

```bash
# Default configuration
python quadruped_example.py

# Specify object type
python quadruped_example.py --object-type gate

# Set logging level
python quadruped_example.py --loglevel DEBUG

# Combine multiple options
python quadruped_example.py --object-type box --loglevel INFO
```

#### CLI Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `--object-type` | `str` | `None` | Type of object to spawn: `"none"`, `"box"`, `"sphere"`, or `"gate"` (default: from config) |
| `--loglevel` | `str` | `"INFO"` | Logging level: `"DEBUG"`, `"INFO"`, `"WARNING"`, `"ERROR"`, or `"CRITICAL"` |
| `--no-csv-logging` | `flag` | `False` | Disable CSV data logging (improves performance) |
| `--no-image-saving` | `flag` | `False` | Disable camera image saving (improves performance) |

#### Performance Optimization

For maximum simulation performance, disable data logging:

```bash
# Disable CSV logging only
python quadruped_example.py --no-csv-logging

# Disable image saving only
python quadruped_example.py --no-image-saving

# Disable both for maximum performance
python quadruped_example.py --no-csv-logging --no-image-saving

# Combine with other options
python quadruped_example.py --object-type gate --no-csv-logging --no-image-saving
```

#### Examples

```bash
# Gate navigation task with debug logging
python quadruped_example.py --object-type gate --loglevel DEBUG

# Box pushing task with performance mode (no logging/saving)
python quadruped_example.py --object-type box --no-csv-logging --no-image-saving

# High-performance testing (no object, no logging)
python quadruped_example.py --object-type none --no-csv-logging --no-image-saving --loglevel WARNING
```

### Visualizing Results

```bash
# Plot a specific experiment
python plot_experiment.py expr_data/251118_185550-my_experiment

# Or use latest experiment (press Enter when prompted)
python plot_experiment.py
```

## Controls

### Keyboard Controls

| Key | Action |
|-----|--------|
| `i` | Move forward (+x direction) |
| `k` | Move backward (-x direction) |
| `j` | Strafe left (+y direction) |
| `l` | Strafe right (-y direction) |
| `u` | Turn left (+yaw) |
| `o` | Turn right (-yaw) |
| `G` | Apply gate transform (only when gate exists) |
| `ESC` | Quit simulation |

### Control Features

- **Smooth acceleration**: Velocity commands ramp up gradually when keys are pressed
- **Decay model**: Velocity decays smoothly when keys are released
- **Configurable limits**: Maximum velocities and acceleration rates adjustable via config
- **Pygame window**: All keyboard input handled in Pygame display window (must have focus)

## Main Components

### `quadruped_example.py` - Main Simulation

The core simulation class `SpotSimulation` provides:

#### Key Methods

- `initialize()`: Creates Isaac Sim world and stage
- `setup()`: Complete simulation setup (environment, robot, cameras, controller)
- `run()`: Main simulation loop
- `cleanup()`: Resource cleanup and file closing

#### Features

1. **Environment Setup**
   - Configurable square environment with boundary walls
   - Start and goal markers (red sphere and blue hemisphere)
   - Support for multiple object types:
     - **Box**: Dynamic pushable box with configurable mass and friction
     - **Sphere**: Dynamic sphere object
     - **Gate**: Static gate with two walls and a gap (for navigation tasks)
     - **None**: No object (robot navigation only)

2. **Robot Setup**
   - Spawns Spot robot at start position
   - Automatically orients robot to face goal direction
   - Uses Isaac Sim's `SpotFlatTerrainPolicy` for locomotion

3. **Camera System**
   - **Ego Camera**: Mounted on robot body (1280×800, 16:10 aspect ratio)
   - **Top Camera**: Overhead view (1600×1600, square aspect ratio)
   - **Pygame Display**: Real-time dual camera view with aspect ratio preservation
   - Both cameras capture images at 10Hz during experiments (if enabled)
   - Images saved as JPEG files with timestamped filenames (if enabled)

4. **Experiment Data Collection** (can be disabled via CLI)
   - **Automatic start**: Data saving begins after first keyboard command
   - **Data saving frequency**: 10 Hz (10 times per second, every 0.1 seconds)
   - **CSV logging**: Robot and object positions/orientations at 10Hz (optional)
   - **Image capture**: Synchronized ego and top camera images at 10Hz (optional)
   - **Configuration save**: Actual values used (not ranges)
   - **Terminal logging**: All console output saved to `terminal.log`

5. **Task Metrics**
   - **L1 Distance**: Manhattan distance metric for task completion
   - **Object-specific**: 
     - Gate tasks: Robot ↔ Goal distance
     - Box/Sphere tasks: Object ↔ Goal distance
     - No object: Robot ↔ Goal distance
   - **Completion detection**: Goal turns green when distance < 1.0m

6. **Performance Monitoring**
   - **Real-time statistics**: Logged at 1Hz (every 500 physics steps)
   - **Physics step times**: Average, min, max, and effective Hz
   - **Render times**: Average, min, max, and effective FPS
   - **Frame times**: Overall loop iteration times
   - **Performance logging**: Displayed in console/log file

7. **Gate Transform System**
   - **Automatic application**: Gate transform applied automatically after setup
   - **Manual trigger**: Press 'G' key in Pygame window to reapply transform
   - **Visual indicator**: Yellow button label shown when gate exists

#### Configuration

The simulation can be configured via:
1. **Command-line arguments** (recommended for quick changes)
2. **Default values** (hardcoded in `DEFAULT_CONFIG`)
3. **JSON file** (`config_file` parameter)
4. **Keyword arguments** (passed to `SpotSimulation()`)

**CLI Configuration Example:**
```bash
python quadruped_example.py --object-type gate --loglevel DEBUG
```

**Python API Example:**
```python
sim = SpotSimulation(
    experiment_name="my_test",
    randomize=True,
    map_size=12.0,
    object_type="gate",  # or "box", "sphere", "none"
    enable_csv_logging=True,  # Can disable for performance
    enable_image_saving=True,  # Can disable for performance
    max_vx=2.5
)
```

### `plot_experiment.py` - Visualization Tool

Interactive visualization tool for analyzing experiment results.

#### Features

1. **Trajectory Map**
   - Robot and object trajectories with time-based color gradient
   - Orientation arrows showing robot heading at intervals
   - Start/goal markers and boundary walls
   - Current position marker (updates with slider)

2. **Velocity Plots**
   - XY velocity components (vx, vy) over time
   - Time gradient coloring
   - Vertical line showing current time position

3. **L1 Distance Metric**
   - Distance to goal over time
   - Success threshold line (1.0m)
   - Current distance display

4. **Image Viewer** (if camera images available)
   - **Top Camera View**: Overhead perspective
   - **Ego Camera View**: Robot's perspective
   - Images update automatically with timestamp slider

5. **Interactive Slider**
   - Navigate through experiment timeline
   - Updates all plots and images synchronously
   - Shows current timestamp

#### Usage

```bash
# Specify experiment directory
python plot_experiment.py expr_data/251118_185550-my_experiment

# Use latest experiment (press Enter)
python plot_experiment.py
```

The tool automatically:
- Finds the latest experiment if no directory specified
- Loads configuration and CSV data
- Detects available camera images
- Creates comprehensive visualization
- Saves plot as `experiment_plot.png` in experiment directory

## Experiment Data Structure

Each experiment creates a directory with timestamp and name:
```
expr_data/
└── YYMMDD_HHMMSS-Experiment_Name/
    ├── config.json          # Configuration used in experiment
    ├── data.csv             # Time-series data (10Hz)
    ├── terminal.log         # Console output log
    ├── experiment_plot.png  # Generated visualization (from plot tool)
    └── camera/
        ├── ego/            # Ego-view camera images
        │   ├── frame0-0.000-ego.jpg
        │   ├── frame1-0.100-ego.jpg
        │   └── ...
        └── top/             # Top-down camera images
            ├── frame0-0.000-top.jpg
            ├── frame1-0.100-top.jpg
            └── ...
```

### CSV Data Format

The `data.csv` file contains:

| Column | Description |
|--------|-------------|
| `timestamp` | Elapsed time since first command (seconds) |
| `frame_num` | Frame number (0-indexed) |
| `robot_pos_x/y/z` | Robot position (meters) |
| `robot_orient_w/x/y/z` | Robot orientation (quaternion) |
| `object_pos_x/y/z` | Object position (meters) |
| `object_orient_w/x/y/z` | Object orientation (quaternion) |
| `l1_distance_to_goal` | L1 distance to goal (meters) |

### Image Naming

Images are named: `frame{N}-{timestamp}-{camera_type}.jpg`

- `N`: Frame number (0-indexed)
- `timestamp`: Elapsed time in seconds (3 decimal places)
- `camera_type`: `ego` or `top`

Example: `frame42-4.200-ego.jpg` (43rd frame at 4.2 seconds, ego-view)

## Configuration Parameters

### Robot Control
- `max_vx`, `max_vy`, `max_yaw`: Maximum velocities (m/s, rad/s)
- `acc_vx`, `acc_vy`, `acc_yaw`: Acceleration rates
- `decay_vx`, `decay_vy`, `decay_yaw`: Decay coefficients
- `update_dt`: Controller update period (default: 0.02s = 50Hz)

### Environment
- `map_size`: Square environment size (meters)
- `wall_height`: Boundary wall height (meters)
- `start_position`, `goal_position`: Robot start/goal [x, y] positions
- `randomize`: Enable environment randomization
- `wall_inset`: Inset from walls for valid spawn area

### Objects
- `use_object`: Enable/disable object spawning
- `object_type`: `"box"`, `"sphere"`, or `"gate"`
- `box_position`, `box_scale`, `box_mass`: Box properties
- `box_friction_static`, `box_friction_dynamic`: Box friction
- `box_scale_range`, `box_mass_range`: Randomization ranges

### Gate-Specific (when `object_type="gate"`)
- `wall_depth_min/max`: Gate wall depth range (meters)
- `gate_y_offset_min/max`: Y-axis offset range (meters)
- `gap_min/max`: Gap between gate walls (meters)
- `gate_location_min/max`: Position along start-goal line (0.0-1.0)

### Cameras
- `ego_camera_resolution`: [width, height] for ego camera (default: [1280, 800])
- `top_camera_resolution`: [width, height] for top camera (default: [1600, 1600])

## Simulation Architecture

### Physics Loop

The simulation runs at:
- **Physics timestep**: 500Hz (2ms)
- **Rendering timestep**: 50Hz (20ms)
- **Command update**: 50Hz (every 10 physics steps)
- **Data logging**: 10Hz (every 50 physics steps)

### Data Saving Logic

1. **Experiment directory** created during `setup()`
2. **Data saving starts** after first keyboard command received
3. **Experiment start time** recorded when first data point saved
4. **Data collection continues** while task is in progress (L1 distance > 1.0m)
5. **Task completion** triggers goal color change to green
6. **Cleanup** closes files and detaches camera annotators

### Data Saving Frequency

**Frequency: 10 Hz** (10 times per second, every 0.1 seconds)

- **Physics rate**: 500 Hz (physics steps per second)
- **Logging rate**: 10 Hz (every 50 physics steps)
  - `logging_counter` increments each physics step
  - When `logging_counter >= 50`, it resets and triggers data saving
  - Calculation: 500 Hz ÷ 50 = 10 Hz

**Data saved at 10 Hz includes:**
- CSV data (robot position, orientation, box positions, etc.)
- Camera images (ego and top view) - synchronized with CSV logging

**Time between saves**: 0.1 seconds (100 ms)
**Physics steps per save**: 50 steps

## Project Structure

```
.
├── README.md                    # This file (English)
├── README_KR.md                 # Korean version
├── quadruped_example.py         # Main simulation (SpotSimulation class) ⭐
├── plot_experiment.py           # Visualization tool ⭐
├── keyboard_controller.py       # Pygame keyboard controller
├── example_config.json          # Example configuration file
├── custom_robots/
│   └── spot.py                  # Spot robot implementation
├── expr_data/                   # Experiment data directory
│   └── YYMMDD_HHMMSS-Name/     # Individual experiment folders
└── docs/                        # Documentation files
    ├── EXPERIMENT_DATA_GUIDE.md
    ├── D455_CAMERA_GUIDE.md
    └── ...
```

**Main Components:**
- ⭐ **`quadruped_example.py`**: Core simulation with CLI interface
- ⭐ **`plot_experiment.py`**: Experiment visualization and analysis tool

## Examples

### Basic Usage (CLI)

```bash
# Default configuration
python quadruped_example.py

# Gate navigation task
python quadruped_example.py --object-type gate

# Box pushing with debug logging
python quadruped_example.py --object-type box --loglevel DEBUG

# High-performance mode (no logging/saving)
python quadruped_example.py --object-type none --no-csv-logging --no-image-saving
```

### Python API Usage

```python
from quadruped_example import SpotSimulation

# Basic simulation
sim = SpotSimulation(experiment_name="test_run")
sim.setup()
sim.run()
sim.cleanup()
```

### Custom Configuration (Python API)

```python
# Load from JSON
sim = SpotSimulation(
    config_file="example_config.json",
    experiment_name="custom_test"
)

# Override specific values
sim = SpotSimulation(
    experiment_name="gate_navigation",
    randomize=True,
    map_size=15.0,
    object_type="gate",
    enable_csv_logging=True,
    enable_image_saving=True,
    max_vx=2.5
)
```

### Performance-Optimized Usage

```python
# Maximum performance (no data logging)
sim = SpotSimulation(
    experiment_name="performance_test",
    enable_csv_logging=False,
    enable_image_saving=False
)
```

### Box Pushing Task

```bash
# Via CLI
python quadruped_example.py --object-type box

# Via Python API
sim = SpotSimulation(
    experiment_name="box_push",
    object_type="box",
    box_mass=10.0,
    box_friction_static=0.8,
    randomize=True
)
```

## Troubleshooting

### Common Issues

1. **Pygame window not appearing**
   - Ensure display is available (X11/Wayland)
   - Simulation runs in headless mode but uses Pygame for display
   - Check that Pygame can initialize (may need `DISPLAY` environment variable)

2. **Robot not responding**
   - Ensure Pygame window has focus (click on it)
   - Check that keyboard input is working in Pygame window
   - Verify controller updates in physics callback

3. **No experiment data saved**
   - Data saving starts only after first keyboard command
   - Check if CSV logging is disabled (`--no-csv-logging` flag)
   - Verify `expr_data/` directory exists and is writable
   - Check experiment directory was created

4. **Camera images not captured**
   - Check if image saving is disabled (`--no-image-saving` flag)
   - Ensure `omni.replicator` is available
   - Verify camera render products initialized
   - Check camera paths are valid

5. **Ego camera view squished**
   - Fixed: Ego camera now maintains 16:10 aspect ratio in Pygame window
   - Camera displays with correct proportions, centered with black bars if needed

6. **Gate transform not applied**
   - Gate transform is applied automatically after setup
   - If gate appears in wrong position, press 'G' key in Pygame window
   - Check logs for gate transform application messages

7. **Plot tool errors**
   - Ensure experiment directory contains `config.json` and `data.csv`
   - If CSV logging was disabled, `data.csv` won't exist
   - Check camera image directories exist (even if empty)
   - Verify matplotlib backend supports interactive display

8. **Performance issues**
   - Use `--no-csv-logging` and `--no-image-saving` flags for better performance
   - Check performance logs in console (logged at 1Hz)
   - Reduce camera resolution if needed
   - Disable rendering if not needed (set `enable_rendering=False` in config)

## Performance

### Simulation Rates

- **Physics**: 500Hz (2ms timestep)
- **Rendering**: 50Hz (20ms timestep)
- **Command update**: 50Hz (every 10 physics steps)
- **Data logging**: 10Hz (every 50 physics steps, if enabled)
  - CSV data and camera images saved at 10Hz
  - Time between saves: 0.1 seconds (100 ms)
- **Image capture**: 10Hz (synchronized with logging, if enabled)
- **Performance logging**: 1Hz (every 500 physics steps)

### Performance Monitoring

The simulation automatically logs performance statistics every second:

```
[Performance] Physics: 2.000ms avg (1.500-3.200ms), 500.0 Hz | Render: 20.000ms avg (18.500-25.300ms), 50.0 FPS | Frame: 22.500ms avg (20.000-28.000ms), 44.4 FPS
```

### Performance Optimization

To maximize simulation speed, use the CLI flags:

```bash
# Disable CSV logging (reduces I/O overhead)
python quadruped_example.py --no-csv-logging

# Disable image saving (reduces encoding/disk I/O)
python quadruped_example.py --no-image-saving

# Maximum performance (disable both)
python quadruped_example.py --no-csv-logging --no-image-saving
```

**Expected Performance Impact:**
- CSV logging disabled: ~5-10% performance improvement
- Image saving disabled: ~10-20% performance improvement
- Both disabled: ~15-30% performance improvement (depending on hardware)

## Documentation

Additional documentation available in `docs/`:
- `EXPERIMENT_DATA_GUIDE.md`: Complete data saving guide
- `D455_CAMERA_GUIDE.md`: Camera system documentation
- `ENVIRONMENT_PARAMS.md`: Complete parameter reference

## License

Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property and proprietary rights in and to this software, related documentation and any modifications thereto. Any use, reproduction, disclosure or distribution of this software and related documentation without an express license agreement from NVIDIA CORPORATION is strictly prohibited.

## References

- [Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/)
- [Isaac Sim Core API Tutorials](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/core_api_tutorials/tutorial_core_hello_world.html)
- [Boston Dynamics Spot](https://www.bostondynamics.com/products/spot)
