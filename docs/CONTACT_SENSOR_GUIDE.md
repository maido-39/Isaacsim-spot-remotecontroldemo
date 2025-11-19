# Contact Sensor Demo - Testing Guide

## Overview
This demo shows real-time contact detection between two cubes with force visualization.

## What I Fixed

### 1. **API Setup** (Critical!)
Added required collision APIs before creating contact sensor:
```python
UsdPhysics.CollisionAPI.Apply(cube2_prim)
UsdPhysics.MeshCollisionAPI.Apply(cube2_prim)
PhysxSchema.PhysxCollisionAPI.Apply(cube2_prim)
PhysxSchema.PhysxContactReportAPI.Apply(cube2_prim)
```
Reference: https://forums.developer.nvidia.com/t/error-with-contact-sensor-initialization/304026

### 2. **Improved Debug Output**
- Shows sensor status every second
- Prints contact events immediately
- Shows force, position, normal vectors
- Position checking with 'P' key

### 3. **Better Initial Setup**
- Cubes start closer (1m apart instead of 2m)
- Clear movement instructions
- Initial position display

## How to Run

```bash
cd /home/syaro/MikuchanRemote/Remotecontrol-Demo/Isaacsim-spot-remotecontroldemo
conda activate isc-pak
python contact_sensor-example.py
```

## Controls

| Key | Action |
|-----|--------|
| `D` | Move Red cube RIGHT (toward Blue cube) ⭐ |
| `A` | Move Red cube LEFT |
| `W` | Move Red cube FORWARD |
| `S` | Move Red cube BACKWARD |
| `Q` | Move Red cube DOWN |
| `E` | Move Red cube UP |
| `R` | Reset Red cube position |
| `P` | Print current positions and distance |

## What to Expect

### 1. **Startup Messages**
```
✓ Contact sensor created at /World/Cube2/ContactSensor
✓ Contact sensor interface acquired
Initial positions:
  Red Cube:  [0.000, 0.000, 0.500]
  Blue Cube: [1.000, 0.000, 0.500]
  Distance: 1.000m
```

### 2. **Debug Output (Every Second)**
```
[Debug] Frame 60: Sensor valid=True, in_contact=False, force=0.00
```

### 3. **Contact Detection**
When cubes touch:
```
🔴 CONTACT DETECTED! Force: 45.23 N
   Time: 3.450s
  └─ Position: [0.65, 0.00, 0.50]
  └─ Normal: [-1.00, 0.00, 0.00]
  └─ Arrow length: 0.90
```

### 4. **Visual Feedback**
- **Green Arrow** appears at contact point
- Arrow points in direction of contact normal
- Arrow length scales with force magnitude

## Testing Steps

1. **Start the simulation** - you should see initial positions
2. **Press 'P'** - verify cube positions
3. **Hold 'D' key** - Red cube moves right toward Blue cube
4. **Watch terminal** - Debug messages show sensor is working
5. **When they touch** - You'll see contact detection message
6. **Look in viewport** - Green arrow shows at contact point
7. **Release 'D'** - Contact ends, arrow disappears

## Troubleshooting

### No Debug Messages?
- Check if simulation is playing (not paused)
- Make sure terminal has focus for keyboard input

### No Contact Detection?
- Press 'P' to check distance between cubes
- Make sure Red cube is moving (should see velocity)
- Check debug output shows "Sensor valid=True"

### Cubes Pass Through Each Other?
- Physics collision might not be set up
- Try resetting with 'R' and moving slower

### Arrow Not Visible?
- Arrow only appears during active contact
- Check that contact is actually detected in terminal
- Arrow is green and points from contact surface

## Expected Behavior

**Distance ~1.0m**: No contact, sensor reports force=0.00  
**Distance ~0.6m**: Contact begins, force increases  
**Distance <0.6m**: Strong contact, large force values, arrow visible  

## Demo Features

✅ Real-time contact detection  
✅ Force magnitude measurement  
✅ Contact position tracking  
✅ Contact normal direction  
✅ Visual force arrow  
✅ Debug output for monitoring  
✅ Interactive keyboard control  

