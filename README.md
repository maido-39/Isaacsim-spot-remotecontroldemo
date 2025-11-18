# Isaac Sim Spot Remote Control Demo

A simulation project for controlling a Boston Dynamics Spot quadruped robot in NVIDIA Isaac Sim using keyboard input. This demo showcases real-time robot control, environment setup, and physics simulation capabilities.

## Overview

This project provides a complete simulation environment for the Spot robot with:
- **Keyboard-based control** using Pygame for smooth velocity commands
- **Configurable environment** with walls, obstacles, and markers
- **Physics simulation** with realistic robot dynamics
- **Contact sensor examples** for collision detection
- **Environment randomization** for varied training scenarios

## Features

- 🤖 **Spot Robot Control**: Full control of Boston Dynamics Spot robot using Isaac Sim's policy controller
- ⌨️ **Keyboard Controller**: Smooth acceleration/decay model for intuitive robot movement
- 🎲 **Environment Randomization**: Configurable randomization of start/goal positions and obstacles
- 📊 **Real-time Logging**: Detailed robot state logging (position, orientation, velocity)
- 💾 **Experiment Data Saving**: Automatic saving of experiment data (CSV + camera images + config)
- 🎥 **RealSense D455 Camera**: Complete D455 camera system with RGB, depth, and stereo IR cameras
- 📸 **Multiple Camera Views**: Top-down overview camera and robot ego-view camera
- 📦 **Dynamic Obstacles**: Pushable boxes with configurable physics properties
- 🔍 **Contact Sensors**: Example implementation of contact detection between objects

## Prerequisites

- **NVIDIA Isaac Sim** (version 4.5.0 or compatible)
- **Python 3.8+**
- **Conda environment** with Isaac Sim packages installed
- **Pygame** (for keyboard input)

## Installation

1. **Set up Isaac Sim environment**:
   ```bash
   # Activate your Isaac Sim conda environment
   conda activate isc-pak  # or your Isaac Sim environment name
   ```

2. **Install additional dependencies** (if not already installed):
   ```bash
   pip install pygame numpy pandas matplotlib pillow
   ```
   
   **Note:** `pandas`, `matplotlib`, and `pillow` are only required for the analysis script (`analyze_experiment_data.py`). The main simulation only requires `pygame` and `numpy`.

3. **Verify Isaac Sim installation**:
   ```bash
   python -c "from isaacsim import SimulationApp; print('Isaac Sim ready')"
   ```

## Quick Start

### Basic Usage

Run the main Spot simulation:

```bash
cd /home/syaro/MikuchanRemote/Remotecontrol-Demo/Isaacsim-spot-remotecontroldemo
conda activate isc-pak
python quadruped_example.py
```

### Using Configuration Files

You can customize the simulation using a JSON configuration file:

```bash
python quadruped_example.py
# Or modify the code to load: sim = SpotSimulation(config_file="example_config.json")
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
- **Configurable limits**: Maximum velocities and acceleration rates can be adjusted in config

## Project Structure

```
.
├── README.md                          # This file
├── quadruped_example.py               # Main Spot robot simulation
├── keyboard_controller.py             # Pygame-based keyboard controller
├── analyze_experiment_data.py         # Experiment data analysis script
├── plot_experiment.py                 # Refined experiment plotting tool
├── example_analysis.py                # Additional analysis examples
├── contact_sensor-example.py          # Contact sensor demo
├── example_config.json                # Example configuration file
├── custom_robots/
│   └── spot.py                        # Custom Spot robot implementation
├── CONTACT_SENSOR_GUIDE.md            # Contact sensor documentation
├── D455_CAMERA_GUIDE.md               # RealSense D455 camera guide
├── CAMERA_UPGRADE_SUMMARY.md          # Camera upgrade technical summary
├── ENVIRONMENT_PARAMS.md              # Environment parameter reference
├── EXPERIMENT_DATA_GUIDE.md           # Experiment data saving guide
├── EXPERIMENT_DATA_IMPLEMENTATION.md  # Data saving implementation details
└── FIXES_APPLIED.md                   # Fix documentation
```

## RealSense D455 Camera System

The robot is equipped with an Intel RealSense D455 camera system mounted on its body. This provides:

### Camera Components
- **RGB Camera**: 1920x1080 color imaging (primary ego-view)
- **Depth Camera**: Distance measurement for obstacle detection
- **Left/Right IR Cameras**: Stereo infrared imaging with 95mm baseline

### Camera Specifications
- **Field of View**: ~69° horizontal
- **Position**: 0.3m forward, 0.1m up from robot body center
- **Orientation**: Forward-facing, aligned with robot
- **Resolution**: Configurable (default: 1280x720 for performance)
- **Frame Rate**: 30 FPS

### Usage Modes

#### Mode 1: Visualization Only (Default)
The camera is automatically created and visible in the viewport. No additional setup required.

#### Mode 2: With Data Capture
To enable programmatic access to camera data:

1. Edit `quadruped_example.py`, find the `setup()` method
2. Uncomment this line:
   ```python
   self._initialize_d455_sensors()
   ```
3. Access camera data in your code:
   ```python
   data = sim.get_d455_data()
   if data:
       rgb_image = data['rgb']['rgba']
       depth_map = data['depth']['distance_to_camera']
       left_ir = data['left_ir']
       right_ir = data['right_ir']
   ```

### Camera Hierarchy
```
/World/Spot/body/camera_mount/
├── d455_rgb          # RGB camera (primary view)
├── d455_depth        # Depth camera
├── d455_left_ir      # Left infrared camera
└── d455_right_ir     # Right infrared camera
```

### Documentation
For detailed camera information, see:
- **[D455_CAMERA_GUIDE.md](D455_CAMERA_GUIDE.md)**: Complete usage guide with examples
- **[CAMERA_UPGRADE_SUMMARY.md](CAMERA_UPGRADE_SUMMARY.md)**: Technical implementation details

## Experiment Data Saving

The simulation automatically saves comprehensive experiment data for analysis and replay:

### What Gets Saved

1. **Configuration** (`config.json`): Actual values used in the experiment
2. **Time-Series Data** (`data.csv`): Robot and object positions/orientations at 10Hz
3. **Camera Images** (`camera/ego/` and `camera/top/`): JPEG images at 10Hz

### Usage

When you run the simulation, you'll be prompted for an experiment name:

```bash
python quadruped_example.py
# Enter experiment name (press Enter for 'NULL'): my_test_run
```

Data is saved to: `YYMMDD_HHMMSS-Experiment_Name/`

Example: `241118_153045-my_test_run/`

### Analyzing Results

**Option 1: Comprehensive Analysis Script**

```bash
python analyze_experiment_data.py 241118_153045-my_test_run
```

This will:
- Print summary statistics (duration, speeds, distances)
- Generate trajectory plots
- Generate time-series plots (speed, distance, orientation)
- Save plots to the experiment directory

**Option 2: Refined Experiment Plot Tool (Recommended)**

```bash
python plot_experiment.py 241118_153045-my_test_run
# Or just press Enter when prompted to use the latest experiment
```

This creates a refined visualization with:
- Map plot with robot/object trajectories and orientations (with time gradient)
- Command velocity subplot
- L1 distance metric subplot
- Automatic detection of latest experiment if no input provided

### Data Format

**CSV Columns:**
- `timestamp`: Elapsed time since first keyboard command (seconds)
- `frame_num`: Frame number
- `robot_pos_x/y/z`: Robot position (meters)
- `robot_orient_w/x/y/z`: Robot orientation (quaternion)
- `object_pos_x/y/z`: Object position (meters)
- `object_orient_w/x/y/z`: Object orientation (quaternion)
- `l1_distance_to_goal`: L1 distance to goal (robot<->goal for gate, box<->goal for box)

**Image Naming:**
- Format: `frame{N}-{timestamp}-{camera_type}.jpg`
- Example: `frame42-4.200-ego.jpg` (43rd frame at 4.2 seconds, ego-view)

### Documentation
- **[EXPERIMENT_DATA_GUIDE.md](EXPERIMENT_DATA_GUIDE.md)**: Complete usage guide
- **[EXPERIMENT_DATA_IMPLEMENTATION.md](EXPERIMENT_DATA_IMPLEMENTATION.md)**: Technical details

## Configuration

The simulation can be configured through:

1. **Default configuration** (hardcoded in `quadruped_example.py`)
2. **JSON configuration file** (see `example_config.json`)
3. **Python keyword arguments** (passed to `SpotSimulation()`)

### Key Configuration Parameters

#### Robot Control
- `max_vx`, `max_vy`, `max_yaw`: Maximum velocities (m/s, rad/s)
- `acc_vx`, `acc_vy`, `acc_yaw`: Acceleration rates
- `decay_vx`, `decay_vy`, `decay_yaw`: Decay coefficients

#### Environment
- `map_size`: Size of the square environment (meters)
- `wall_height`: Height of boundary walls
- `start_position`, `goal_position`: Robot start and goal positions
- `use_box`: Enable/disable obstacle box
- `randomize`: Enable environment randomization

#### Physics
- `ground_friction_static`, `ground_friction_dynamic`: Ground friction
- `box_mass`: Mass of obstacle box
- `box_friction_static`, `box_friction_dynamic`: Box friction

See `ENVIRONMENT_PARAMS.md` for a complete parameter reference.

### Example Configuration

```python
# Load from JSON file
sim = SpotSimulation(config_file="example_config.json")

# Override specific values
sim = SpotSimulation(
    randomize=True,
    map_size=12.0,
    max_vx=3.0,
    use_box=False
)
```

## Examples

### 1. Basic Spot Control

```python
from quadruped_example import SpotSimulation

# Create and run simulation
sim = SpotSimulation()
sim.setup()
sim.run()
sim.cleanup()
```

### 2. Custom Environment

```python
sim = SpotSimulation(
    map_size=15.0,
    start_position=[5.0, 5.0],
    goal_position=[-5.0, -5.0],
    use_box=True,
    box_mass=10.0
)
sim.setup()
sim.run()
sim.cleanup()
```

### 3. Contact Sensor Demo

Run the contact sensor example:

```bash
python contact_sensor-example.py
```

See `CONTACT_SENSOR_GUIDE.md` for detailed instructions.

## Architecture

### Main Components

1. **SpotSimulation**: Main simulation class managing world, environment, and robot
2. **KeyboardController**: Thread-safe keyboard input handler with acceleration/decay model
3. **SpotFlatTerrainPolicy**: Isaac Sim Spot robot controller with RL policy
4. **Environment Setup**: Configurable environment with walls, obstacles, and markers

### Simulation Loop

1. **Initialization**: Create Isaac Sim world and stage
2. **Environment Setup**: Create ground, walls, obstacles, markers
3. **Robot Setup**: Spawn Spot robot at start position facing goal
4. **Controller Setup**: Initialize keyboard controller in separate thread
5. **Physics Loop**: 
   - Update controller commands (50Hz)
   - Apply commands to robot
   - Log robot state (10Hz)
   - Render scene (50Hz)

## Documentation

- **[EXPERIMENT_DATA_GUIDE.md](EXPERIMENT_DATA_GUIDE.md)**: Experiment data saving and analysis guide
- **[EXPERIMENT_DATA_IMPLEMENTATION.md](EXPERIMENT_DATA_IMPLEMENTATION.md)**: Data saving technical details
- **[D455_CAMERA_GUIDE.md](D455_CAMERA_GUIDE.md)**: RealSense D455 camera usage guide
- **[CAMERA_UPGRADE_SUMMARY.md](CAMERA_UPGRADE_SUMMARY.md)**: Camera upgrade technical details
- **[CONTACT_SENSOR_GUIDE.md](CONTACT_SENSOR_GUIDE.md)**: Guide for contact sensor implementation
- **[ENVIRONMENT_PARAMS.md](ENVIRONMENT_PARAMS.md)**: Complete environment parameter reference
- **[FIXES_APPLIED.md](FIXES_APPLIED.md)**: Documentation of fixes and improvements

## Troubleshooting

### Common Issues

1. **Pygame window not appearing**
   - Ensure display is available (not in headless mode)
   - Check that `headless=False` in `SimulationApp` initialization

2. **Robot not responding to controls**
   - Verify keyboard controller thread started successfully
   - Check that pygame window has focus
   - Ensure controller is being updated in physics callback

3. **Import errors**
   - Verify Isaac Sim conda environment is activated
   - Check that all required packages are installed
   - Ensure Isaac Sim is properly installed

4. **Physics simulation issues**
   - Check that world is reset after setup
   - Verify robot is initialized before applying commands
   - Ensure physics timestep is appropriate (500Hz default)

### Debug Mode

Enable detailed logging by modifying the logger level in `SpotSimulation._setup_logging()`:

```python
self.logger.setLevel(logging.DEBUG)  # Already set by default
```

## Performance

- **Physics timestep**: 500Hz (2ms)
- **Rendering timestep**: 50Hz (20ms)
- **Command update**: 50Hz
- **Logging frequency**: 10Hz

## License

Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property and proprietary rights in and to this software, related documentation and any modifications thereto. Any use, reproduction, disclosure or distribution of this software and related documentation without an express license agreement from NVIDIA CORPORATION is strictly prohibited.

## References

- [Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/)
- [Isaac Sim Core API Tutorials](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/core_api_tutorials/tutorial_core_hello_world.html)
- [Boston Dynamics Spot](https://www.bostondynamics.com/products/spot)

## Contributing

This is a demo project. For improvements or bug fixes, please ensure:
- Code follows existing style and structure
- Documentation is updated accordingly
- Changes are tested with Isaac Sim 4.5.0+

## Acknowledgments

- NVIDIA Isaac Sim team for the simulation platform
- Boston Dynamics for the Spot robot model
- Isaac Sim community for examples and documentation



