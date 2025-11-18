# RealSense D455 Camera Integration Guide

This guide explains how to use the Intel RealSense D455 camera as the robot's ego-view camera in the Isaac Sim Spot simulation.

## Overview

The robot's ego-view camera has been upgraded from a simple camera to a RealSense D455 camera system. The D455 provides:
- **RGB Camera**: Color imaging (1920x1080 max resolution)
- **Depth Camera**: Distance measurement
- **Left/Right IR Cameras**: Stereo infrared imaging (95mm baseline)

## Changes Made

### 1. Camera Setup (`_setup_robot_camera()`)

The method now creates a complete D455 camera system attached to the robot body:

- **Camera Mount**: Created at `/World/Spot/body/camera_mount`
  - Position: 0.3m forward, 0.1m up from robot body center
  - Orientation: Forward-facing (aligned with robot)

- **Four Cameras Created**:
  - `d455_rgb`: RGB color camera
  - `d455_depth`: Depth camera
  - `d455_left_ir`: Left infrared camera (offset -47.5mm)
  - `d455_right_ir`: Right infrared camera (offset +47.5mm)

### 2. Camera Specifications

Based on RealSense D455 specs:
- **Field of View**: ~69° horizontal (simulated with 24mm focal length)
- **Aspect Ratio**: 16:9
- **Stereo Baseline**: 95mm (distance between IR cameras)
- **Sensor Size**: 36mm x 20.25mm (simulated)

### 3. Sensor Initialization (`_initialize_d455_sensors()`)

Optional method to enable data capture from cameras:
- Creates Isaac Sim Camera sensor objects
- Configures resolution (default: 1280x720)
- Sets frequency (default: 30 FPS)
- Enables motion vectors

### 4. Data Capture (`get_d455_data()`)

Helper method to retrieve sensor data:
```python
data = sim.get_d455_data()
if data:
    rgb_frame = data['rgb']
    depth_frame = data['depth']
    left_ir_frame = data['left_ir']
    right_ir_frame = data['right_ir']
```

## Usage Options

### Option 1: Visualization Only (Default)

The default configuration creates D455 cameras for visualization in the viewport. No code changes needed.

**Pros:**
- Simple setup
- Low computational overhead
- Suitable for teleoperation/control tasks

**Cons:**
- Cannot capture sensor data programmatically
- No access to raw image data

### Option 2: With Sensor Data Capture

Enable sensor initialization to capture RGB, depth, and IR data.

**Steps:**
1. In `quadruped_example.py`, locate the `setup()` method
2. Uncomment this line:
   ```python
   self._initialize_d455_sensors()
   ```
3. Use `get_d455_data()` to capture frames in your code

**Pros:**
- Full access to RGB, depth, and IR data
- Can save images, process with CV algorithms, etc.
- Useful for vision-based navigation/ML

**Cons:**
- Higher computational cost
- May reduce simulation performance

### Option 3: Use External D455 USD File

If you have a complete D455 USD asset (with 3D model):

**Steps:**
1. In `_setup_robot_camera()`, uncomment and modify these lines:
   ```python
   realsense_usd_path = "/path/to/your/realsense_d455.usd"
   realsense_prim_path = f"{camera_mount_path}/realsense_d455"
   from isaacsim.core.utils.stage import add_reference_to_stage
   add_reference_to_stage(realsense_usd_path, realsense_prim_path)
   ```
2. Comment out the "Option 2" camera creation code (lines creating individual cameras)

**Where to get D455 USD:**
- Isaac Sim Asset Library (check Nucleus server)
- Create your own from CAD models
- Download from Intel/community resources

**Pros:**
- Realistic 3D model appearance
- Accurate physical dimensions
- May include pre-configured camera parameters

**Cons:**
- Requires finding/creating USD file
- Larger file size
- May need to adjust camera paths for your specific USD

## Camera Positioning

Current mount position:
- **Forward**: 0.3m from body center (facing forward)
- **Up**: 0.1m from body center (slightly above)
- **Rotation**: Aligned with robot body (forward-facing)

To adjust camera position, modify in `_setup_robot_camera()`:
```python
translate_op.Set(Gf.Vec3f(0.3, 0.0, 0.1))  # (forward, right, up)
```

To adjust camera angle, modify rotation quaternion:
```python
# Example: Tilt down 15 degrees
import numpy as np
pitch = np.radians(-15)  # Negative = down
quat = Gf.Quatf(
    np.cos(pitch/2), 
    0.0, 
    np.sin(pitch/2), 
    0.0
)
```

## Accessing Camera Views

### Viewport Display
The RGB camera (`d455_rgb`) is set as the robot ego-view by default. A separate viewport window will open showing this view.

### Accessing Other Cameras
To view depth or IR cameras, you can:
1. Use the viewport camera selector in Isaac Sim UI
2. Manually switch to other camera paths:
   - `/World/Spot/body/camera_mount/d455_depth`
   - `/World/Spot/body/camera_mount/d455_left_ir`
   - `/World/Spot/body/camera_mount/d455_right_ir`

## Example: Capturing and Saving Images

If you've enabled sensor initialization, you can capture images:

```python
# In your simulation loop or callback
def capture_images(self):
    data = self.get_d455_data()
    if data:
        # RGB image
        rgb = data['rgb']['rgba']  # RGBA array
        
        # Depth image
        depth = data['depth']['distance_to_camera']  # Distance values
        
        # Save with your preferred method (cv2, PIL, etc.)
        import cv2
        cv2.imwrite('rgb_frame.png', rgb[:, :, :3])  # BGR format
        cv2.imwrite('depth_frame.png', depth.astype('uint16'))
```

## Performance Considerations

- **Viewport only**: Minimal performance impact
- **With sensor capture**: ~10-30% performance decrease depending on resolution
- **High resolution (1920x1080)**: May significantly slow simulation
- **Recommended for real-time**: 1280x720 or lower

To change resolution, edit in `_initialize_d455_sensors()`:
```python
resolution = (1280, 720)  # (width, height)
```

## Troubleshooting

### Camera Not Visible
- Check that robot body path is correct: `/World/Spot/body`
- Verify world.reset() was called before camera setup
- Check console for warning messages

### Sensor Data Returns None
- Ensure `_initialize_d455_sensors()` was called
- Check that world.reset() completed successfully
- Verify camera paths exist in the stage

### Poor Performance
- Reduce camera resolution
- Disable sensor initialization if not needed
- Close unnecessary viewport windows

## References

- **Forum Discussion**: [Attaching RealSense D455 Camera to Go1 via Python](https://forums.developer.nvidia.com/t/attaching-realsense-d455-camera-to-go1-via-python/338056)
- **Isaac Sim Docs**: [USD Assets and Sensors](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/assets/usd_assets_sensors.html)
- **RealSense D455 Specs**: Intel RealSense D455 product page

## Camera Hierarchy

```
/World/Spot/body/
└── camera_mount/
    ├── d455_rgb          [Primary RGB camera]
    ├── d455_depth        [Depth camera]
    ├── d455_left_ir      [Left infrared camera, -47.5mm offset]
    └── d455_right_ir     [Right infrared camera, +47.5mm offset]
```

## Next Steps

1. **Test the visualization**: Run the simulation and check the robot ego-view
2. **Enable sensors if needed**: Uncomment `_initialize_d455_sensors()` for data capture
3. **Adjust camera position**: Modify translation/rotation if needed
4. **Integrate with your code**: Use `get_d455_data()` in your control loop
5. **Optimize performance**: Adjust resolution based on your needs


