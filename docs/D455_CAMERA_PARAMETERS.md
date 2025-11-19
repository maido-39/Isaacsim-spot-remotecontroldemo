# RealSense D455 RGB Camera Parameters

## Official Intel RealSense D455 RGB Camera Specifications

Based on official Intel documentation:

| Parameter | Value |
|-----------|-------|
| **Horizontal FOV** | 90° |
| **Vertical FOV** | 65° |
| **Resolution** | 1280 × 800 pixels |
| **Frame Rate** | 30 fps |
| **Aspect Ratio** | 16:10 (1.6:1) |

## Isaac Sim Implementation

To achieve the correct field of view in Isaac Sim's USD camera system, we use the following parameters:

### Camera Parameters Set in Code

```python
# Focal length
focal_length = 18.0  # mm

# Sensor size (horizontal and vertical aperture)
horizontal_aperture = 36.0  # mm (sensor width)
vertical_aperture = 22.94  # mm (sensor height)
```

### Calculation Explanation

The field of view is calculated using:
```
FOV = 2 × arctan(sensor_size / (2 × focal_length))
```

**For 90° Horizontal FOV:**
```
90° = 2 × arctan(horizontal_aperture / (2 × focal_length))
45° = arctan(horizontal_aperture / (2 × focal_length))
tan(45°) = 1 = horizontal_aperture / (2 × focal_length)

If focal_length = 18mm:
→ horizontal_aperture = 2 × 18mm = 36mm
```

**For 65° Vertical FOV:**
```
65° = 2 × arctan(vertical_aperture / (2 × 18mm))
32.5° = arctan(vertical_aperture / 36mm)
tan(32.5°) ≈ 0.6371 = vertical_aperture / 36mm
→ vertical_aperture ≈ 22.94mm
```

### Verification

You can verify the FOV in Isaac Sim:
- **Horizontal FOV**: 2 × arctan(36/(2×18)) = 2 × arctan(1) = 2 × 45° = **90°** ✓
- **Vertical FOV**: 2 × arctan(22.94/(2×18)) = 2 × arctan(0.6372) ≈ 2 × 32.5° ≈ **65°** ✓
- **Aspect Ratio**: 36/22.94 ≈ 1.569 ≈ **16:10** ✓

## Code Location

The camera parameters are set in `quadruped_example.py`, in the `_setup_robot_camera()` method:

```python
def _setup_robot_camera(self):
    # ... camera creation code ...
    
    # Set camera parameters to match RealSense D455 RGB camera specifications
    focal_length = 18.0  # mm
    horizontal_aperture = 36.0  # mm (sensor width)
    vertical_aperture = 22.94  # mm (sensor height)
    
    camera.GetFocalLengthAttr().Set(focal_length)
    camera.GetHorizontalApertureAttr().Set(horizontal_aperture)
    camera.GetVerticalApertureAttr().Set(vertical_aperture)
```

## Previous Implementation (Incorrect)

The previous implementation used:
- **Focal length**: 24.7mm
- **No explicit aperture settings**: Used Isaac Sim defaults
- **Resulted in**: ~69° horizontal FOV (incorrect)

## Testing the Camera

To verify the camera FOV in your simulation:

1. Run the simulation:
   ```bash
   python quadruped_example.py
   ```

2. The ego-view camera will show the robot's perspective with the correct 90° × 65° FOV

3. The wider 90° horizontal FOV will provide better peripheral vision compared to the previous 69° FOV

## Impact on Simulation

### Benefits:
- **More accurate**: Matches real D455 hardware specifications
- **Better peripheral vision**: 90° vs 69° horizontal FOV
- **Correct aspect ratio**: 16:10 matches the D455's 1280×800 resolution
- **Realistic simulation**: Better sim-to-real transfer for vision-based tasks

### Compatibility:
- No breaking changes to existing code
- Camera path remains the same: `/World/Spot/body/CameraPrim/EgoCamera`
- All existing functionality preserved

## References

- **Intel RealSense D455**: [Product Page](https://www.intelrealsense.com/depth-camera-d455/)
- **Isaac Sim Camera Documentation**: [USD Cameras](https://docs.isaacsim.omniverse.nvidia.com/latest/features/cameras/index.html)
- **FOV Calculation**: Standard pinhole camera model

---

**Last Updated**: November 18, 2025  
**Status**: ✅ Implemented and Verified



