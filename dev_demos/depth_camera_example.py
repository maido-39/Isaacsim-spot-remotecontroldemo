"""
Simple depth camera example for standalone Isaac Sim execution.

This example demonstrates:
1. Launching Isaac Sim in standalone mode
2. Creating a simple scene with objects
3. Setting up a depth camera
4. Capturing and displaying depth images using Replicator
"""

# Launch Isaac Sim before any other imports
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# Import after SimulationApp initialization
import numpy as np
from pxr import Gf, UsdGeom
import omni
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid

# Create World instance
world = World(physics_dt=1.0/60.0, rendering_dt=1.0/60.0, stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

# Add ground plane
world.scene.add_default_ground_plane(prim_path="/World/GroundPlane")

# Create some objects for depth visualization
# Fixed cube
world.scene.add(
    FixedCuboid(
        prim_path="/World/Cube1",
        name="fixed_cube",
        position=np.array([1.0, 0.0, 0.5]),
        scale=np.array([0.5, 0.5, 0.5]),
        color=np.array([1.0, 0.0, 0.0])  # Red
    )
)

# Dynamic cube
world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube2",
        name="dynamic_cube",
        position=np.array([-1.0, 0.0, 0.5]),
        scale=np.array([0.5, 0.5, 0.5]),
        color=np.array([0.0, 0.0, 1.0]),  # Blue
        mass=1.0
    )
)

# Reset world to initialize physics
world.reset()

# Create depth camera
def create_depth_camera():
    """Create a depth camera looking at the scene."""
    camera_path = "/World/DepthCamera"
    
    # Create camera prim
    camera = UsdGeom.Camera.Define(stage, camera_path)
    
    # Set camera position and orientation
    camera_xform = UsdGeom.Xformable(camera)
    camera_xform.ClearXformOpOrder()
    
    # Position camera 3m above origin, looking down at -45 degrees
    translate_op = camera_xform.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(0.0, -3.0, 3.0))
    
    # Rotate to look at origin (45 degrees down, 90 degrees around Y)
    rotate_op = camera_xform.AddRotateXYZOp()
    rotate_op.Set(Gf.Vec3d(0.0, 45.0, 90.0))
    
    # Set camera parameters
    camera.GetFocalLengthAttr().Set(18.0)  # mm
    camera.GetHorizontalApertureAttr().Set(20.955)  # mm
    camera.GetVerticalApertureAttr().Set(15.955)  # mm
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.1, 100.0))
    
    return camera_path

# Create the depth camera
camera_path = create_depth_camera()
print(f"✓ Depth camera created at: {camera_path}")

# Setup Replicator for depth capture
try:
    import omni.replicator.core as rep
    
    # Create render product for depth camera
    resolution = (640, 480)
    render_product = rep.create.render_product(camera_path, resolution)
    
    # Create depth annotator
    depth_annotator = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
    depth_annotator.attach([render_product])
    
    # Create RGB annotator (optional, for visualization)
    rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annotator.attach([render_product])
    
    print(f"✓ Replicator initialized: {resolution[0]}×{resolution[1]}")
    
except ImportError:
    print("Warning: Replicator not available. Depth capture disabled.")
    depth_annotator = None
    rgb_annotator = None
    render_product = None

# Main simulation loop
frame_count = 0
print("\nStarting simulation...")
print("Press Ctrl+C to stop\n")

try:
    while simulation_app.is_running():
        # Step simulation
        world.step(render=True)
        
        # Capture depth data every 30 frames (~0.5 seconds at 60 FPS)
        if depth_annotator and frame_count % 30 == 0:
            try:
                # Get depth data
                depth_data = depth_annotator.get_data()
                
                if depth_data is not None:
                    # Convert to numpy array
                    depth_array = np.asarray(depth_data)
                    
                    # Get valid depth values (remove inf/nan)
                    valid_depth = depth_array[np.isfinite(depth_array)]
                    
                    if len(valid_depth) > 0:
                        min_depth = np.min(valid_depth)
                        max_depth = np.max(valid_depth)
                        mean_depth = np.mean(valid_depth)
                        
                        print(f"Frame {frame_count}: Depth range [{min_depth:.2f}, {max_depth:.2f}]m, Mean: {mean_depth:.2f}m")
                    
                    # Optional: Get RGB for visualization
                    if rgb_annotator:
                        rgb_data = rgb_annotator.get_data()
                        if rgb_data is not None:
                            # RGB data available but not printed to keep output clean
                            pass
                            
            except Exception as e:
                print(f"Error capturing depth: {e}")
        
        frame_count += 1
        
except KeyboardInterrupt:
    print("\nSimulation stopped by user")

# Cleanup
if depth_annotator and render_product:
    try:
        depth_annotator.detach([render_product])
        if rgb_annotator:
            rgb_annotator.detach([render_product])
        print("✓ Replicator cleanup complete")
    except:
        pass

print("Closing simulation...")
simulation_app.close()

