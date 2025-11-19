# Reset Implementation Plan

## Research Findings

Based on Isaac Sim documentation and IsaacLab examples:

### DON'T:
- ❌ Remove and recreate prims during simulation
- ❌ Call `world.stop()` then `world.reset()` 
- ❌ Use `stage.RemovePrim()` on scene-managed objects
- ❌ Destroy and recreate World instance

### DO:
- ✅ Keep objects in scene, reposition them using `set_world_pose()`
- ✅ Call `world.reset()` to reset physics state
- ✅ Update object properties using their API methods
- ✅ Randomize seed before calling reset logic

## Implementation Strategy

```python
def reset_environment(self, randomize_seed=True):
    """Reset environment by repositioning objects, not recreating them"""
    
    # 1. Randomize seed
    if randomize_seed:
        new_seed = np.random.randint(0, 2**31 - 1)
        np.random.seed(new_seed)
    
    # 2. Reset controller velocities
    if self.controller:
        self.controller.reset()
    
    # 3. Calculate new randomized positions
    self._apply_randomization()  # Updates self.config with new positions
    
    # 4. Reposition robot using set_world_pose()
    #    - Calculate new orientation from start to goal
    #    - Use spot.robot.set_world_pose(position, orientation)
    
    # 5. Reposition box using set_world_pose()
    #    - Get reference to box from world.scene
    #    - Use box.set_world_pose(new_position)
    
    # 6. Update markers by modifying their USD transforms
    #    - Use UsdGeom.Xformable to modify start/goal markers
    
    # 7. Reset world physics
    world.reset()
    
    # 8. Reset physics state flags
    self.physics_ready = False
```

## Key References:
- IsaacLab Discussion #3866: Using set_world_pose() for reset
- IsaacLab Discussion #1326: Avoid recreating stage during reset
- Isaac Sim Core API: DynamicCuboid.set_world_pose()
- Isaac Sim Core API: Articulation.set_world_pose()

## Implementation Order:
1. Add randomization calculation method (reuse _apply_randomization but don't recreate)
2. Add object repositioning method using set_world_pose()
3. Add marker updating method  
4. Add robot repositioning method
5. Combine into reset_environment() method
6. Add UI button to trigger reset

