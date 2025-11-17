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
- 🎥 **Multiple Camera Views**: Top-down overview camera and robot ego-view camera
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
   pip install pygame numpy
   ```

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
├── README.md                      # This file
├── quadruped_example.py           # Main Spot robot simulation
├── keyboard_controller.py         # Pygame-based keyboard controller
├── contact_sensor-example.py      # Contact sensor demo
├── example_config.json            # Example configuration file
├── custom_robots/
│   └── spot.py                    # Custom Spot robot implementation
├── CONTACT_SENSOR_GUIDE.md        # Contact sensor documentation
├── ENVIRONMENT_PARAMS.md          # Environment parameter reference
└── FIXES_APPLIED.md               # Fix documentation
```

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



