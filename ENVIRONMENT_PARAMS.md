# Environment Parameters Guide

## Overview
The `setup_environment()` function allows you to customize the simulation environment with both **fixed** and **randomizable** parameters.

## Parameter Categories

### Fixed Parameters
These parameters define the static properties of the environment:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `ground_color` | np.ndarray | [0.5, 0.5, 0.5] | Ground plane RGB color |
| `wall_color` | np.ndarray | [0.7, 0.7, 0.7] | Wall RGB color |
| `wall_height` | float | 2.0 | Wall height in meters |
| `map_size` | float | 10.0 | Square map size in meters |
| `ground_friction_static` | float | 0.2 | Ground static friction coefficient |
| `ground_friction_dynamic` | float | 0.2 | Ground dynamic friction coefficient |
| `ground_restitution` | float | 0.01 | Ground restitution (bounciness) |
| `box_friction_static` | float | 0.8 | Obstacle box static friction |
| `box_friction_dynamic` | float | 0.7 | Obstacle box dynamic friction |
| `box_restitution` | float | 0.1 | Obstacle box restitution |
| `dome_light_intensity` | float | 600.0 | Dome light intensity |
| `marker_radius` | float | 0.3 | Start/goal marker radius |

### Randomizable Parameters
These parameters can be set to fixed values or randomized:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `start_position` | np.ndarray | [4.0, 4.0] | Start marker position [x, y] |
| `goal_position` | np.ndarray | [-4.0, -4.0] | Goal marker position [x, y] |
| `box_position` | np.ndarray | [2.0, 0.0, 0.25] | Obstacle box position [x, y, z] |
| `box_scale` | np.ndarray | [1.0, 1.0, 0.5] | Obstacle box scale [x, y, z] |
| `box_mass` | float | 5.0 | Obstacle box mass in kg |
| `box_color` | np.ndarray | [0.6, 0.4, 0.2] | Obstacle box RGB color |
| `robot_height` | float | 0.65 | Robot spawn height above ground |

### Randomization Settings

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `randomize` | bool | False | Enable/disable randomization |
| `start_pos_range` | tuple | ((-4.5, 4.5), (-4.5, 4.5)) | Start position range ((x_min, x_max), (y_min, y_max)) |
| `goal_pos_range` | tuple | ((-4.5, 4.5), (-4.5, 4.5)) | Goal position range |
| `box_pos_range` | tuple | ((-3.0, 3.0), (-3.0, 3.0)) | Box position range (z is kept fixed) |
| `box_scale_range` | tuple | ((0.5, 2.0), (0.5, 2.0), (0.3, 1.0)) | Box scale range (x, y, z) |
| `box_mass_range` | tuple | (3.0, 10.0) | Box mass range (min, max) |

## Robot Placement

The robot is automatically placed at the **start position** and oriented to face the **goal position**. The robot's:
- **Position**: Set to `[start_position[0], start_position[1], robot_height]`
- **Orientation**: Calculated to face from start to goal using yaw angle: `yaw = atan2(goal_y - start_y, goal_x - start_x)`

This ensures the robot always starts facing its goal, regardless of randomization.

## Usage Examples

### 1. Default Configuration
```python
# Use all default parameters
start_pos, goal_pos, robot_height = setup_environment(world, stage)
```

### 2. Custom Fixed Parameters
```python
# Customize colors and map size
start_pos, goal_pos, robot_height = setup_environment(
    world, 
    stage,
    ground_color=np.array([0.3, 0.3, 0.3]),  # Darker ground
    wall_color=np.array([0.9, 0.9, 0.9]),    # Lighter walls
    wall_height=3.0,                          # Taller walls
    map_size=15.0,                            # Larger map
    dome_light_intensity=800.0                # Brighter lighting
)
```

### 3. Custom Start/Goal Positions
```python
# Set specific marker positions
start_pos, goal_pos, robot_height = setup_environment(
    world, 
    stage,
    start_position=np.array([3.0, 3.0]),
    goal_position=np.array([-3.0, -3.0])
)
```

### 4. Custom Obstacle Box and Robot Height
```python
# Customize box properties and robot height
start_pos, goal_pos, robot_height = setup_environment(
    world, 
    stage,
    box_position=np.array([0.0, 0.0, 0.5]),
    box_scale=np.array([2.0, 2.0, 1.0]),   # Larger box
    box_mass=10.0,                          # Heavier box
    box_friction_static=0.9,                # More friction
    box_color=np.array([0.8, 0.2, 0.2]),   # Red box
    robot_height=0.7                        # Taller spawn height
)
```

### 5. Enable Randomization
```python
# Randomize positions and box properties
start_pos, goal_pos, robot_height = setup_environment(
    world, 
    stage,
    randomize=True,
    start_pos_range=((-4.0, 4.0), (-4.0, 4.0)),
    goal_pos_range=((-4.0, 4.0), (-4.0, 4.0)),
    box_pos_range=((-2.0, 2.0), (-2.0, 2.0)),
    box_scale_range=((0.5, 1.5), (0.5, 1.5), (0.3, 0.8)),
    box_mass_range=(3.0, 8.0)
)
```

### 6. Full Customization with Randomization
```python
# Combine fixed and randomizable parameters
start_pos, goal_pos, robot_height = setup_environment(
    world, 
    stage,
    # Fixed parameters
    ground_color=np.array([0.4, 0.4, 0.4]),
    wall_color=np.array([0.8, 0.8, 0.8]),
    map_size=12.0,
    wall_height=2.5,
    dome_light_intensity=700.0,
    robot_height=0.7,
    # Enable randomization
    randomize=True,
    start_pos_range=((-5.0, 5.0), (-5.0, 5.0)),
    goal_pos_range=((-5.0, 5.0), (-5.0, 5.0)),
    box_pos_range=((-4.0, 4.0), (-4.0, 4.0)),
    box_scale_range=((0.5, 2.0), (0.5, 2.0), (0.3, 1.0)),
    box_mass_range=(2.0, 12.0)
)
```

## Tips

1. **Map Size**: Ensure `start_pos_range` and `goal_pos_range` are within `map_size / 2` to keep markers inside the walls.

2. **Box Position**: The z-coordinate of `box_position` is automatically preserved during randomization to ensure the box stays on the ground.

3. **Friction Values**: Higher friction values (0.8-1.0) prevent slipping, while lower values (0.1-0.3) make surfaces more slippery.

4. **Randomization Ranges**: Make sure the ranges are sensible:
   - Keep markers away from walls (add margin)
   - Don't make boxes too small (< 0.3m) or too large (> 3.0m)
   - Keep mass reasonable for realistic physics (2-15 kg)

5. **Lighting**: Dome light intensity typically ranges from 400 to 1000. Higher values brighten the scene.

## Robot Placement Example

After setting up the environment, the robot is automatically positioned and oriented:

```python
# Setup environment and get start/goal positions
start_pos, goal_pos, robot_height = setup_environment(world, stage, randomize=True)

# Calculate robot orientation to face goal
direction_vector = goal_pos - start_pos
robot_yaw = np.arctan2(direction_vector[1], direction_vector[0])

# Convert yaw to quaternion
half_yaw = robot_yaw / 2.0
robot_orientation = np.array([
    np.cos(half_yaw),  # w
    0.0,                # x
    0.0,                # y
    np.sin(half_yaw)   # z
])

# Spawn robot at start position facing goal
robot = YourRobotClass(
    position=np.array([start_pos[0], start_pos[1], robot_height]),
    orientation=robot_orientation
)
```

This pattern is already implemented in `quadruped_example.py`.

## Environment Reset for Multiple Episodes

If you want to randomize the environment for multiple episodes:

```python
# Episode loop
for episode in range(num_episodes):
    # Reset world
    world.reset()
    
    # Setup environment with randomization
    start_pos, goal_pos, robot_height = setup_environment(
        world, 
        stage,
        randomize=True,
        # ... other parameters
    )
    
    # Calculate and set robot orientation
    # ... (see Robot Placement Example above)
    
    # Run episode
    # ...
```

Note: Currently, the environment setup creates new prims. For efficient multi-episode randomization, consider implementing a separate `randomize_environment()` function that only updates positions without recreating objects.

