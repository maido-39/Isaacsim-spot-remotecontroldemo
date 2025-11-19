# D455 Camera Rollback Note

## Date
November 17, 2025

## Status
**ROLLED BACK** - Reverted to original simple camera implementation

## Reason
Camera was not working properly with the D455 implementation.

## Changes Reverted

1. **`_setup_robot_camera()`**: Restored original simple camera setup
   - Single camera with quaternion rotation (0.5, 0.5, -0.5, -0.5)
   - Focal length: 24.7mm
   - Camera path: `/World/Spot/body/CameraPrim/EgoCamera`

2. **Removed Methods**:
   - `_initialize_d455_sensors()` - removed
   - `get_d455_data()` - removed

3. **Removed from `setup()`**:
   - Commented sensor initialization call - removed

## D455 USD Path (For Future Reference)

If you want to try again with the actual D455 USD file:

```python
# Path to D455 USD file in Isaac Sim assets:
realsense_usd_path = "Sensors/Intel/RealSense/rsd455.usd"

# To use it, add reference to stage:
from isaacsim.core.utils.stage import add_reference_to_stage
body_path = "/World/Spot/body"
camera_mount_path = f"{body_path}/camera_mount"
realsense_prim_path = f"{camera_mount_path}/realsense_d455"
add_reference_to_stage(realsense_usd_path, realsense_prim_path)
```

## Current Camera Configuration

```
/World/Spot/body/CameraPrim/
└── EgoCamera          # Simple camera
    - Focal length: 24.7mm
    - Rotation: quaternion (0.5, 0.5, -0.5, -0.5)
    - Translation: (0, 0, 0)
```

## Documentation Files

The D455 documentation files are still available for reference:
- `D455_CAMERA_GUIDE.md`
- `CAMERA_UPGRADE_SUMMARY.md`
- `D455_IMPLEMENTATION_COMPLETE.md`
- `D455_QUICK_REFERENCE.md`
- `d455_camera_example.py`

These can be used as reference if you want to implement D455 again in the future.

## Current Status

✅ Original simple camera restored
✅ Code working (back to previous state)
✅ D455 documentation preserved for future reference
✅ USD path noted for future attempts

## To Try D455 Again (Future)

If you want to try implementing D455 with the actual USD file:

1. Use the USD path: `Sensors/Intel/RealSense/rsd455.usd`
2. Add reference to stage under robot body
3. Position and orient the camera mount appropriately
4. Find the camera prims within the USD structure
5. Set viewport to the D455 RGB camera

Example:
```python
from isaacsim.core.utils.stage import add_reference_to_stage

# In _setup_robot_camera():
body_path = "/World/Spot/body"
camera_mount_path = f"{body_path}/camera_mount"

# Create mount with position/rotation
camera_mount = UsdGeom.Xform.Define(self.stage, camera_mount_path)
camera_xform = UsdGeom.Xformable(camera_mount)
camera_xform.ClearXformOpOrder()
translate_op = camera_xform.AddTranslateOp()
translate_op.Set(Gf.Vec3f(0.3, 0.0, 0.1))  # Forward, Right, Up

# Add D455 USD reference
realsense_usd_path = "Sensors/Intel/RealSense/rsd455.usd"
realsense_prim_path = f"{camera_mount_path}/realsense_d455"
add_reference_to_stage(realsense_usd_path, realsense_prim_path)

# Find camera paths in the USD structure (may vary)
# Typically something like:
# rgb_camera_path = f"{realsense_prim_path}/rsd455/RSD455/Camera_RGB"
# depth_camera_path = f"{realsense_prim_path}/rsd455/RSD455/Camera_Depth"
```

## Notes

- The D455 USD file structure may be different than expected
- Need to inspect the USD file to find the actual camera prim paths
- The forum reference showed paths like: `/rsd455/RSD455/Camera_OmniVision_OV9782_Color`
- May need to adjust based on actual USD structure


