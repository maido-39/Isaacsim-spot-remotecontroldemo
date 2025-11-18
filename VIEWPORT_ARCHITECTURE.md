# Viewport Architecture - Dual Camera Windows

## Overview
Both ego-view and top-down cameras now open in **separate viewport windows** instead of using the default scene viewport.

## Architecture

### Two Independent Viewport Windows

| Viewport | Camera | Resolution | Position | Purpose |
|----------|--------|------------|----------|---------|
| **RobotEgoView** | Ego Camera | 1280×800 | (100, 100) | Robot perspective |
| **TopDownView** | Top Camera | 1600×1600 | (1400, 100) | Overhead view |

### Implementation

#### 1. Ego Camera Viewport
**Function:** `_open_robot_camera_viewport()` (lines 1496-1516)

```python
def _open_robot_camera_viewport(self, camera_path):
    """Open a new viewport window showing the robot's ego-view camera."""
    create_viewport_for_camera(
        viewport_name="RobotEgoView",
        camera_prim_path=camera_path,
        width=1280,
        height=800,
        position_x=100,
        position_y=100
    )
```

#### 2. Top Camera Viewport
**Function:** `_open_top_camera_viewport()` (lines 1518-1538)

```python
def _open_top_camera_viewport(self, camera_path):
    """Open a new viewport window showing the top-down camera view."""
    create_viewport_for_camera(
        viewport_name="TopDownView",
        camera_prim_path=camera_path,
        width=1600,
        height=1600,
        position_x=1400,  # Positioned to the right of ego view
        position_y=100
    )
```

#### 3. Setup Flow
**In `setup()` method (lines 1715-1728):**

```python
# 1. Open ego camera viewport
if hasattr(self, 'robot_camera_path') and self.robot_camera_path:
    self._open_robot_camera_viewport(self.robot_camera_path)

# 2. Open top camera viewport
if hasattr(self, 'camera_path') and self.camera_path:
    self._open_top_camera_viewport(self.camera_path)
```

## Key Features

### ✅ Advantages

1. **Independent Views**
   - Each camera has its own dedicated window
   - Can position and resize independently
   - No interference between views

2. **Consistent Architecture**
   - Both cameras use the same viewport creation pattern
   - Same error handling and logging
   - Clean, symmetric code

3. **Automatic Positioning**
   - Ego view: Top-left corner (100, 100)
   - Top view: Positioned to the right (1400, 100)
   - No window overlap

4. **Configuration-Driven**
   - Resolution from `DEFAULT_CONFIG`
   - Easy to adjust via config file
   - No hardcoded values in viewport logic

### 🔧 Benefits

- **Professional Layout**: Side-by-side windows for monitoring
- **Flexibility**: Close/move/resize any window independently
- **Scalability**: Easy to add more camera viewports
- **Clarity**: Clear separation of concerns

## Visual Layout

```
┌─────────────────────┐     ┌─────────────────────┐
│  RobotEgoView       │     │  TopDownView        │
│  1280×800           │     │  1600×1600          │
│                     │     │                     │
│  [Ego Camera]       │     │  [Top Camera]       │
│  Robot perspective  │     │  Overhead view      │
│                     │     │                     │
│  Position: (100,100)│     │  Position: (1400,100)
└─────────────────────┘     └─────────────────────┘
```

## Previous Behavior (Removed)

❌ **Old Method:** `_set_viewport_camera()`
- Set top camera as default viewport view
- Used `viewport.set_active_camera()`
- No separate window for top camera

✅ **New Method:** `_open_top_camera_viewport()`
- Creates dedicated window for top camera
- Consistent with ego camera approach
- Better user experience

## Summary

**Before:** Ego camera in separate window, top camera in default viewport  
**After:** Both cameras in separate, independent windows  

This provides a **professional, symmetric, and flexible** dual-camera monitoring system! 🎉

