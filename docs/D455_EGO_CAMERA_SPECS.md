# RealSense D455 Ego Camera Configuration

## Reference Specifications
- **RGB Sensor FOV**: 90° H × 65° V
- **RGB Frame Resolution**: Up to 1280 × 800 pixels
- **Aspect Ratio**: 16:10 (1280/800 = 1.6)

---

## Implemented Configuration

### Resolution
```python
"ego_camera_resolution": [1280, 800]  # 16:10 aspect ratio
```

### Camera Parameters
```python
focal_length = 18.0  # mm
horizontal_aperture = 36.0  # mm
resolution = (1280, 800)
```

### Calculated Values
```python
# Vertical aperture (auto-calculated in _create_camera)
aspect_ratio = 800 / 1280 = 0.625
vertical_aperture = 36.0 × 0.625 = 22.5 mm
```

---

## FOV Verification

### Horizontal FOV Calculation:
```
FOV_H = 2 × arctan(h_aperture / (2 × focal_length))
FOV_H = 2 × arctan(36.0 / (2 × 18.0))
FOV_H = 2 × arctan(36.0 / 36.0)
FOV_H = 2 × arctan(1.0)
FOV_H = 2 × 45°
FOV_H = 90° ✓
```

### Vertical FOV Calculation:
```
FOV_V = 2 × arctan(v_aperture / (2 × focal_length))
FOV_V = 2 × arctan(22.5 / (2 × 18.0))
FOV_V = 2 × arctan(22.5 / 36.0)
FOV_V = 2 × arctan(0.625)
FOV_V = 2 × 32.0°
FOV_V = 64° ≈ 65° ✓
```

---

## Comparison Table

| Parameter | Reference (D455) | Implemented | Match |
|-----------|------------------|-------------|-------|
| Resolution | 1280 × 800 | 1280 × 800 | ✅ |
| Aspect Ratio | 16:10 | 16:10 | ✅ |
| H-FOV | 90° | 90° | ✅ |
| V-FOV | 65° | 64° | ✅ (~1° difference) |
| Focal Length | - | 18.0mm | - |
| H-Aperture | - | 36.0mm | - |
| V-Aperture | - | 22.5mm (calc) | - |

---

## Position & Orientation

### Position (relative to robot body)
```python
translation = (0.3, 0.0, 0.2)  # meters
```
- X: 0.3m forward
- Y: 0.0m centered
- Z: 0.2m up

### Rotation
```python
rotation_quat = (0.5, 0.5, -0.5, -0.5)  # (w, x, y, z)
# Equivalent to RPY: (90°, -90°, 0°)
```
- Roll: 90°
- Pitch: -90° (pointing down)
- Yaw: 0°

---

## Usage

The camera will automatically:
1. ✅ Match D455 RGB sensor FOV (90° × 65°)
2. ✅ Use D455 resolution (1280 × 800)
3. ✅ Calculate correct aspect ratio (16:10)
4. ✅ Provide accurate viewport and saved images

---

## Benefits

✅ **Sim-to-Real Transfer** - Matches real D455 hardware  
✅ **Accurate FOV** - 90° × 65° field of view  
✅ **Standard Resolution** - Industry-standard 1280 × 800  
✅ **Proper Aspect Ratio** - 16:10 matches sensor  
✅ **Professional Standards** - Based on manufacturer specs  

---

**Configuration Status**: ✅ Complete  
**Last Updated**: November 18, 2025  
**Reference**: Intel RealSense D455 RGB Camera Specifications

