# Isaac Sim Spot Remote Control Demo

A comprehensive simulation project for controlling a Boston Dynamics Spot quadruped robot in NVIDIA Isaac Sim with keyboard input, automatic experiment data collection, and advanced visualization tools.

> **한국어 버전**: [README_KR.md](README_KR.md)를 참조하세요.

## Overview

This project provides a complete simulation environment for the Spot robot featuring:
- **Real-time keyboard control** with smooth acceleration/decay model
- **Automatic experiment data collection** (CSV, camera images, configuration)
- **Multiple object types** (box, sphere, gate) for various task scenarios
- **Dual camera system** (ego-view and top-down) with synchronized image capture
- **Interactive visualization tool** for analyzing experiment results
- **Environment randomization** for varied training scenarios

## Features

### Core Simulation (`quadruped_example.py`)
- 🤖 **Spot Robot Control**: Full control using Isaac Sim's `SpotFlatTerrainPolicy`
- ⌨️ **Keyboard Controller**: Thread-safe keyboard input with smooth velocity ramping
- 🎲 **Environment Randomization**: Configurable randomization of start/goal positions and objects
- 📦 **Multiple Object Types**: Support for dynamic boxes, spheres, and static gates
- 📊 **Real-time State Tracking**: Robot and object positions/orientations at 10Hz
- 💾 **Automatic Data Saving**: CSV logs, camera images, and configuration files
- 🎥 **Dual Camera System**: Ego-view (robot-mounted) and top-down (overhead) cameras
- 📏 **L1 Distance Metrics**: Task completion tracking based on object type
- 🎯 **Task Completion Detection**: Automatic goal color change when task is complete

### Visualization Tool (`plot_experiment.py`)
- 📈 **Comprehensive Plots**: Trajectory maps, velocity plots, and distance metrics
- 🎨 **Time Gradient Visualization**: Color-coded trajectories showing temporal progression
- 🖼️ **Integrated Image Viewer**: Synchronized ego and top camera image display
- 🎚️ **Interactive Timestamp Slider**: Navigate through experiment timeline
- 🔄 **Automatic Latest Detection**: Finds most recent experiment if no directory specified

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

### Running the Simplified Demo

For a quick start with minimal setup (no experiment logging, just robot control and a sample box):

```bash
conda activate isc-demo
python spot_demo.py
```

This simplified demo includes:
- Spot robot at origin
- Sample box that can be pushed
- Keyboard control (same controls as main simulation)
- No experiment data logging or cameras

### Running the Full Simulation

```bash
conda activate isc-demo
python quadruped_example.py
```

When prompted, enter an experiment name (or press Enter for 'NULL'):
```
Enter experiment name (press Enter for 'NULL'): my_experiment
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
| `ESC` | Quit simulation |

### Control Features

- **Smooth acceleration**: Velocity commands ramp up gradually when keys are pressed
- **Decay model**: Velocity decays smoothly when keys are released
- **Configurable limits**: Maximum velocities and acceleration rates adjustable via config

## Main Components

### `spot_demo.py` - Simplified Demo

A minimal demonstration script for quick testing and learning. This simplified version includes only the essential components:

#### Features

- **Spot Robot**: Spawns at origin with default orientation
- **Sample Box**: A dynamic box at position [2.0, 0.0, 0.25] that can be pushed
- **Keyboard Control**: Full keyboard control using the same `KeyboardController`
- **Basic Environment**: Ground plane with physics properties
- **No Data Logging**: No experiment data collection, cameras, or file saving

#### Usage

```bash
python spot_demo.py
```

#### Key Methods

- `initialize()`: Creates Isaac Sim world and stage
- `setup_environment()`: Creates ground plane and sample box
- `setup_robot()`: Spawns Spot robot at origin
- `setup()`: Complete setup (environment, robot, controller)
- `run()`: Main simulation loop
- `cleanup()`: Resource cleanup

#### When to Use

- Quick testing of robot control
- Learning the basic simulation setup
- Prototyping without experiment overhead
- Simple box pushing tasks

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

2. **Robot Setup**
   - Spawns Spot robot at start position
   - Automatically orients robot to face goal direction
   - Uses Isaac Sim's `SpotFlatTerrainPolicy` for locomotion

3. **Camera System**
   - **Ego Camera**: Mounted on robot body (1280×800, 16:10 aspect ratio)
   - **Top Camera**: Overhead view (1600×1600, square aspect ratio)
   - Both cameras capture images at 10Hz during experiments
   - Images saved as JPEG files with timestamped filenames

4. **Experiment Data Collection**
   - **Automatic start**: Data saving begins after first keyboard command
   - **CSV logging**: Robot and object positions/orientations at 10Hz
   - **Image capture**: Synchronized ego and top camera images
   - **Configuration save**: Actual values used (not ranges)
   - **Terminal logging**: All console output saved to `terminal.log`

5. **Task Metrics**
   - **L1 Distance**: Manhattan distance metric for task completion
   - **Object-specific**: 
     - Gate tasks: Robot ↔ Goal distance
     - Box/Sphere tasks: Object ↔ Goal distance
   - **Completion detection**: Goal turns green when distance < 1.0m

#### Configuration

The simulation can be configured via:
1. **Default values** (hardcoded in `DEFAULT_CONFIG`)
2. **JSON file** (`config_file` parameter)
3. **Keyword arguments** (passed to `SpotSimulation()`)

Example:
```python
sim = SpotSimulation(
    experiment_name="my_test",
    randomize=True,
    map_size=12.0,
    object_type="gate",  # or "box", "sphere"
    use_object=True,
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

## Project Structure

```
.
├── README.md                    # This file (English)
├── README_KR.md                 # Korean version
├── spot_demo.py                 # Simplified demo (minimal setup)
├── quadruped_example.py         # Main simulation (SpotSimulation class)
├── plot_experiment.py           # Visualization tool
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

## Examples

### Simplified Demo

```python
from spot_demo import SpotDemo

# Create simplified demo
demo = SpotDemo()
demo.setup()
demo.run()
demo.cleanup()
```

### Basic Usage (Full Simulation)

```python
from quadruped_example import SpotSimulation

# Create simulation
sim = SpotSimulation(experiment_name="test_run")
sim.setup()
sim.run()
sim.cleanup()
```

### Custom Configuration

```python
# Load from JSON
sim = SpotSimulation(
    config_file="example_config.json",
    experiment_name="custom_test"
)

# Or override specific values
sim = SpotSimulation(
    experiment_name="gate_navigation",
    randomize=True,
    map_size=15.0,
    object_type="gate",
    max_vx=2.5
)
```

### Box Pushing Task

```python
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
   - Ensure display is available (not headless mode)
   - Check `headless=False` in `SimulationApp` initialization

2. **Robot not responding**
   - Verify keyboard controller thread started
   - Check pygame window has focus
   - Ensure controller updates in physics callback

3. **No experiment data saved**
   - Data saving starts only after first keyboard command
   - Check `expr_data/` directory exists
   - Verify experiment directory was created

4. **Camera images not captured**
   - Ensure `omni.replicator` is available
   - Check camera render products initialized
   - Verify camera paths are valid

5. **Plot tool errors**
   - Ensure experiment directory contains `config.json` and `data.csv`
   - Check camera image directories exist (even if empty)
   - Verify matplotlib backend supports interactive display

## Performance

- **Physics**: 500Hz (2ms timestep)
- **Rendering**: 50Hz (20ms timestep)
- **Data logging**: 10Hz (100ms interval)
- **Image capture**: 10Hz (synchronized with logging)

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
