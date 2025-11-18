# Intel RealSense D455 Camera Presets Guide

## Overview

The Intel RealSense D455 camera provides several pre-defined visual presets optimized for different use cases. These presets can be selected in the Intel RealSense Viewer tool and are also available as JSON configuration files.

## Available Visual Presets

### 1. Default Preset (기본 프리셋)
**Purpose**: General-purpose visual quality

**Characteristics:**
- Clean edges and reduced point cloud spray
- Best overall visual representation
- Balanced settings for most applications

**Recommended Use Cases:**
- General visualization
- Standard depth sensing applications
- When you need clean, visually appealing results

---

### 2. High Density Preset (고밀도 프리셋)
**Purpose**: Maximum object detection

**Characteristics:**
- Higher fill factor
- Detects more objects in the scene
- More dense point cloud data

**Recommended Use Cases:**
- Background removal (BGS)
- 3D enhanced photography
- Object recognition
- Scene reconstruction
- Maximum coverage applications

---

### 3. Medium Density Preset (중간 밀도 프리셋)
**Purpose**: Balanced performance

**Characteristics:**
- Balance between fill factor and accuracy
- Good compromise for most applications

**Recommended Use Cases:**
- General robotics applications
- When you need both coverage and accuracy
- Mixed indoor/outdoor environments

---

### 4. High Accuracy Preset (고정밀 프리셋)
**Purpose**: Maximum precision

**Characteristics:**
- High confidence threshold for depth
- Lower fill factor but higher accuracy
- Reduced noise in depth measurements

**Recommended Use Cases:**
- **Object scanning**
- **Collision avoidance**
- **Robotics** ✓ (Recommended for your Spot robot simulation)
- Precise measurement applications
- Safety-critical applications

**Note:** This preset is ideal for robotics applications where accurate depth perception is critical for navigation and obstacle avoidance.

---

### 5. Hand Gesture Preset (손 제스처 프리셋)
**Purpose**: Hand tracking and gesture recognition

**Characteristics:**
- Sharp edges for hand detection
- Optimized for close-range tracking
- Fast response time

**Recommended Use Cases:**
- Hand tracking
- Gesture recognition
- Human-computer interaction
- Close-range object manipulation

---

## RealSense D455 RGB Camera Specifications

### RGB Camera Specs
| Parameter | Value |
|-----------|-------|
| **Horizontal FOV** | 90° |
| **Vertical FOV** | 65° |
| **Max Resolution** | 1920 × 1080 pixels |
| **Common Resolution** | 1280 × 800 pixels |
| **Frame Rate** | Up to 30 fps |
| **Aspect Ratio** | 16:10 (for 1280×800) |

### Depth Camera Specs
| Parameter | Value |
|-----------|-------|
| **Depth Technology** | Active IR Stereo |
| **Depth Range** | 0.6m to 6m (optimal: 0.6m - 3m) |
| **Depth Resolution** | Up to 1280 × 720 |
| **Depth Frame Rate** | Up to 90 fps |
| **Stereo Baseline** | 95mm |

### Infrared Camera Specs
| Parameter | Value |
|-----------|-------|
| **Resolution** | 1280 × 720 |
| **Frame Rate** | Up to 90 fps |
| **Wavelength** | 850nm |
| **Configuration** | Left and Right stereo pair |

---

## Preset Configuration

### How to Select Presets

#### Method 1: Intel RealSense Viewer
1. Open Intel RealSense Viewer
2. Connect your D455 camera
3. Select preset from dropdown menu in the Stereo Module section
4. Changes apply immediately

#### Method 2: JSON Configuration Files
1. Open RealSense Viewer
2. Click "JSON Save/Load" button (top of options panel)
3. Select or load custom preset JSON
4. Configure parameters in "Advanced Mode"
5. Save configuration for later use

#### Method 3: SDK Programming
```python
import pyrealsense2 as rs

# Create pipeline
pipeline = rs.pipeline()
config = rs.config()

# Start pipeline
pipeline.start(config)

# Get device
profile = pipeline.get_active_profile()
device = profile.get_device()
depth_sensor = device.first_depth_sensor()

# Set visual preset
depth_sensor.set_option(
    rs.option.visual_preset, 
    3  # 3 = High Accuracy preset
)
```

### Preset Values
- 0 = Custom
- 1 = Default
- 2 = Hand
- 3 = High Accuracy
- 4 = High Density
- 5 = Medium Density

---

## Recommendation for Spot Robot Simulation

### Current Implementation
Based on `D455_CAMERA_PARAMETERS.md`, the simulation currently uses:
- **Focal length**: 18.0mm
- **Horizontal aperture**: 36.0mm
- **Vertical aperture**: 22.94mm
- **FOV**: 90° H × 65° V (matches D455 RGB specs ✓)

### Suggested Preset: **High Accuracy**

**Why High Accuracy for Robotics:**
1. **Precise collision avoidance** - Critical for autonomous navigation
2. **Reliable depth measurements** - Better for path planning
3. **Reduced false positives** - Fewer phantom obstacles
4. **Consistent performance** - More stable across environments

**Trade-offs:**
- ✓ Higher precision and reliability
- ✓ Better for safety-critical applications
- ✗ Lower fill factor (some gaps in point cloud)
- ✗ May miss some small/distant objects

### Alternative: Medium Density
If you need better coverage while maintaining reasonable accuracy:
- Good balance for mixed environments
- Better object detection at distance
- Slightly lower precision than High Accuracy

---

## Current Camera Configuration in Code

Location: `quadruped_example.py` → `_setup_robot_camera()`

```python
# Current configuration (as of your modifications)
# Position: translation=[0.2, 0.0, 0.2]
# Rotation: RPY=[0°, -80°, -90°] 
# Quaternion: WXYZ=[0.5417, -0.4545, -0.4545, -0.5417]

# Camera parameters
focal_length = 1.93  # mm (CUSTOM - not standard D455)
horizontal_aperture = 3.8  # mm (CUSTOM)
vertical_aperture = 2.85  # mm (calculated)

# Resolution
resolution = (32, 24)  # CUSTOM - very low resolution
```

### Standard D455 Parameters (for reference)
```python
# Standard D455 RGB specs
focal_length = 18.0  # mm
horizontal_aperture = 36.0  # mm
vertical_aperture = 22.94  # mm
resolution = (1280, 800)  # or (1920, 1080)
```

---

## Self-Calibration Feature

The D455 includes on-chip self-calibration:
- **Duration**: 15 seconds
- **No target required**: Automatic calibration
- **Access**: Via Intel RealSense SDK 2.0
- **Purpose**: Improve depth accuracy over time

---

## References

- **Intel D455 Product Page**: https://www.intelrealsense.com/depth-camera-d455/
- **Visual Presets Documentation**: https://dev.intelrealsense.com/docs/d400-series-visual-presets
- **Quick Start Video**: [D455 Quick Start Optimization](https://www.youtube.com/watch?v=udzp1CBEeQA)
- **SDK Documentation**: https://github.com/IntelRealSense/librealsense
- **Isaac Sim Camera Docs**: https://docs.isaacsim.omniverse.nvidia.com/latest/features/cameras/

---

## Next Steps

1. **Review current configuration**: Your custom parameters (1.93mm focal, 32×24 resolution) may not match D455 specs
2. **Consider preset**: Decide if High Accuracy or Medium Density is best for your application
3. **Test in simulation**: Compare different camera configurations
4. **Optimize for performance**: Balance resolution vs. frame rate for your needs

---

**Last Updated**: November 18, 2025  
**Status**: ✅ Preset Information Compiled



