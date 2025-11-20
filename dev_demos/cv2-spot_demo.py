# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

"""
Spot robot demo with keyboard control, sample box, and top-view camera displayed with cv2.imshow
"""

# Launch Isaac Sim before any other imports
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})  # Need GUI for cv2.imshow

# SimulationApp 초기화 후에만 다른 모듈을 import할 수 있습니다.
import sys
from pathlib import Path

# Add parent directory to path to import keyboard_controller
parent_dir = Path(__file__).parent.parent
sys.path.insert(0, str(parent_dir))

import numpy as np
from pxr import Gf, UsdPhysics, Sdf, UsdGeom
import omni
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy
from keyboard_controller import KeyboardController
import cv2


class CV2SpotDemo:
    """Spot robot demo with keyboard control, sample box, and top-view camera displayed with cv2.imshow"""

    def __init__(self):
        """Initialize the demo"""
        self.world = None
        self.stage = None
        self.spot = None
        self.controller = None
        self.physics_ready = False
        self.command_counter = 0
        self.camera_path = None  # Top camera path
        self.render_products = {}  # Camera render products for display
        self.rgb_annotators = {}  # RGB annotators for camera capture
        self.camera_render_initialized = False
        self.window_name = "Spot Robot - Top View Camera"

    def initialize(self):
        """Initialize Isaac Sim world and stage"""
        # Create World: physics at 500Hz, rendering at 50Hz
        self.world = World(physics_dt=1.0/500.0, rendering_dt=10.0/500.0, stage_units_in_meters=1.0)
        # Get USD stage for scene manipulation
        self.stage = omni.usd.get_context().get_stage()
        print("World initialized")

    def _create_camera(self, parent_path, camera_name, translation, rotation_quat, 
                       focal_length, horizontal_aperture, resolution, 
                       clipping_range=(0.01, 10000.0)):
        """
        General camera creation function with configurable parameters.
        
        Args:
            parent_path: USD path of parent prim
            camera_name: Name for the camera
            translation: Translation tuple (x, y, z) in meters
            rotation_quat: Rotation quaternion (w, x, y, z)
            focal_length: Focal length in mm
            horizontal_aperture: Horizontal aperture in mm
            resolution: Resolution tuple (width, height)
            clipping_range: Clipping range tuple (near, far) in meters
            
        Returns:
            str: Camera USD path if successful, None otherwise
        """
        try:
            # Create transform prim for camera
            camera_prim_path = f"{parent_path}/{camera_name}_Prim"
            camera_prim = UsdGeom.Xform.Define(self.stage, camera_prim_path)
            
            # Apply transformations
            camera_xform = UsdGeom.Xformable(camera_prim)
            camera_xform.ClearXformOpOrder()
            
            # Add translation
            translate_op = camera_xform.AddTranslateOp()
            translate_op.Set(Gf.Vec3f(*translation))
            
            # Add rotation
            rotate_op = camera_xform.AddOrientOp()
            rotate_op.Set(Gf.Quatf(*rotation_quat))
            
            # Create camera
            camera_path = f"{camera_prim_path}/{camera_name}"
            camera = UsdGeom.Camera.Define(self.stage, camera_path)
            
            # Calculate vertical aperture from resolution aspect ratio
            aspect_ratio = resolution[1] / resolution[0]  # height / width
            vertical_aperture = horizontal_aperture * aspect_ratio
            
            # Set camera intrinsic parameters
            camera.GetFocalLengthAttr().Set(focal_length)
            camera.GetHorizontalApertureAttr().Set(horizontal_aperture)
            camera.GetVerticalApertureAttr().Set(vertical_aperture)
            camera.GetClippingRangeAttr().Set(Gf.Vec2f(*clipping_range))
            
            return camera_path
            
        except Exception as e:
            print(f"Failed to create camera {camera_name}: {e}")
            return None

    def _initialize_camera_render_products(self):
        """
        Initialize render products and annotators for top camera.
        This enables camera image capture for cv2 display.
        """
        try:
            import omni.replicator.core as rep
            
            # Setup top camera render product
            if self.camera_path:
                top_res = (1600, 1600)  # Top camera resolution
                self.render_products["top"] = rep.create.render_product(
                    self.camera_path, 
                    top_res
                )
                self.rgb_annotators["top"] = rep.AnnotatorRegistry.get_annotator("rgb")
                self.rgb_annotators["top"].attach([self.render_products["top"]])
                print(f"✓ Top camera render product initialized: {top_res[0]}×{top_res[1]}")
                
                self.camera_render_initialized = True
            else:
                print("Warning: Camera path not available for render product initialization")
                self.camera_render_initialized = False
                
        except ImportError as e:
            print(f"Warning: Replicator not available, camera capture disabled: {e}")
            self.camera_render_initialized = False
        except Exception as e:
            print(f"Warning: Failed to initialize camera render products: {e}")
            self.camera_render_initialized = False

    def setup_environment(self):
        """Setup basic environment: ground, sample box, and top-view camera"""
        if self.world is None or self.stage is None:
            raise RuntimeError("World must be initialized first")

        # Add dome light for scene illumination
        omni.kit.commands.execute("CreatePrim", prim_path="/World/DomeLight", prim_type="DomeLight")
        light_prim = self.stage.GetPrimAtPath("/World/DomeLight")
        if light_prim.IsValid():
            light_prim.GetAttribute("inputs:intensity").Set(600.0)

        # Create ground plane with physics properties
        self.world.scene.add_default_ground_plane(
            z_position=0,
            name="default_ground_plane",
            prim_path="/World/GroundPlane",
            static_friction=0.2,
            dynamic_friction=0.2,
            restitution=0.01,
        )

        # Create sample box (dynamic, can be pushed by robot)
        box_position = np.array([2.0, 0.0, 0.25])
        box_scale = np.array([1.0, 1.0, 0.5])
        box_color = np.array([0.6, 0.4, 0.2])  # Brown color

        self.world.scene.add(DynamicCuboid(
            prim_path="/World/SampleBox",
            name="sample_box",
            position=box_position,
            scale=box_scale,
            color=box_color,
            mass=5.0,
            linear_velocity=np.array([0.0, 0.0, 0.0])
        ))

        # Apply physics material to box (friction and restitution)
        box_prim = self.stage.GetPrimAtPath("/World/SampleBox")
        if box_prim.IsValid():
            physics_material_path = "/World/Materials/BoxPhysicsMaterial"
            # Create physics material with friction properties
            physics_material = UsdPhysics.MaterialAPI.Apply(
                self.stage.DefinePrim(physics_material_path, "Material")
            )
            physics_material.CreateStaticFrictionAttr().Set(0.8)
            physics_material.CreateDynamicFrictionAttr().Set(0.7)
            physics_material.CreateRestitutionAttr().Set(0.1)

            # Bind physics material to box collider
            collider = UsdPhysics.CollisionAPI.Get(self.stage, "/World/SampleBox")
            if collider:
                collider.GetPrim().CreateRelationship("physics:material").SetTargets(
                    [Sdf.Path(physics_material_path)]
                )

        # Create top-down camera using generic camera creation function
        # Position: 20m above origin, looking down
        top_res = (1600, 1600)  # Square aspect ratio
        
        # Calculate quaternion for -90° rotation around Z axis
        rotation_z = -np.pi / 2.0
        half_angle = rotation_z / 2.0
        rotation_quat = (
            float(np.cos(half_angle)),  # w
            0.0,                        # x
            0.0,                        # y
            float(np.sin(half_angle))   # z
        )
        
        camera_path = self._create_camera(
            parent_path="/World",
            camera_name="TopCamera",
            translation=(0.0, 0.0, 20.0),  # 20m above origin
            rotation_quat=rotation_quat,  # -90° around Z axis
            focal_length=18.0,  # mm (for overhead view)
            horizontal_aperture=18.0,  # mm (square aspect ratio)
            resolution=top_res,
            clipping_range=(0.1, 50.0)  # Appropriate for overhead view
        )
        
        if camera_path:
            self.camera_path = camera_path
            print(f"✓ Top camera created: {camera_path}")
            print(f"  Resolution: {top_res[0]}×{top_res[1]} pixels")
        else:
            print("Warning: Failed to create top camera")

        print("Environment setup complete: ground, sample box, and top camera created")

    def setup_robot(self):
        """Setup Spot robot at origin"""
        # Add Spot robot to scene at origin
        self.spot = SpotFlatTerrainPolicy(
            prim_path="/World/Spot",
            name="Spot",
            position=np.array([0.0, 0.0, 0.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),  # Identity quaternion (no rotation)
        )
        print("Spot robot placed at origin")

    def _on_physics_step(self, step_size):
        """
        Physics step callback - called every physics timestep (500Hz).
        Handles command updates (50Hz) and robot control.
        """
        # Command update: update controller at 50Hz (every 10 physics steps)
        self.command_counter += 1
        if self.command_counter >= 10:
            self.command_counter = 0
            self.controller.update()  # Update controller state based on keyboard input

        # Robot control: apply commands to robot
        if self.physics_ready:
            # Robot is initialized, apply forward control with current command
            self.spot.forward(step_size, self.controller.get_command())
        else:
            # First physics step: initialize robot
            self.physics_ready = True
            self.spot.initialize()  # Initialize robot policy
            self.spot.post_reset()  # Post-reset setup
            self.spot.robot.set_joints_default_state(self.spot.default_pos)  # Set default joint positions
            print("Spot initialized")

    def get_top_camera_image(self):
        """
        Get current frame from top camera for cv2 display.
        
        Returns:
            numpy.ndarray: BGR image array (H, W, 3) or None if not available
        """
        if not self.camera_render_initialized:
            return None
        
        if "top" not in self.rgb_annotators:
            return None
        
        try:
            rgb_data = self.rgb_annotators["top"].get_data()
            if rgb_data is None:
                return None
            
            # Convert to numpy array
            image_array = np.asarray(rgb_data)
            
            # Check if we have valid image data
            if image_array is None or image_array.size == 0:
                return None
            
            # Handle RGBA to RGB conversion if needed
            if len(image_array.shape) == 3 and image_array.shape[2] == 4:
                # Convert RGBA to RGB
                image_array = image_array[:, :, :3]
            
            # Convert RGB to BGR for OpenCV (cv2 uses BGR format)
            if len(image_array.shape) == 3 and image_array.shape[2] == 3:
                try:
                    image_array = cv2.cvtColor(image_array, cv2.COLOR_RGB2BGR)
                except Exception:
                    # If conversion fails, try direct assignment (might already be BGR)
                    pass
            
            return image_array
            
        except Exception as e:
            # Silently return None on error (avoid spam during shutdown)
            return None

    def setup(self):
        """Complete simulation setup: environment, robot, controller, and cameras"""
        # Initialize world if not already done
        if self.world is None:
            self.initialize()

        # Setup environment (ground, box, and top camera)
        self.setup_environment()

        # Setup robot at origin
        self.setup_robot()

        # Create keyboard controller
        self.controller = KeyboardController(
            max_vx=2.0, max_vy=2.0, max_yaw=2.0,
            acc_vx=5.0, acc_vy=5.0, acc_yaw=10.0,
            decay_vx=0.7, decay_vy=0.7, decay_yaw=0.6,
            update_dt=0.02  # 50Hz update rate
        )

        # Reset world (required before querying articulation properties)
        self.world.reset()

        # Initialize camera render products for cv2 display
        self._initialize_camera_render_products()

        # Register physics callback for robot control
        self.world.add_physics_callback("physics_step", callback_fn=self._on_physics_step)

        # Start keyboard controller (runs in separate thread)
        self.controller.start()

        # Create OpenCV window
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 800, 800)  # Initial window size

        print("Setup complete")

    def run(self):
        """Run main simulation loop"""
        if self.world is None or self.spot is None or self.controller is None:
            raise RuntimeError("Simulation must be setup first")

        print("Starting simulation...")
        print("Controls: i/k (x), j/l (y), u/o (yaw), ESC (quit)")
        print("Camera view displayed in OpenCV window")

        frame_counter = 0
        
        # Main simulation loop
        while simulation_app.is_running():
            # Check if quit requested from keyboard controller
            if self.controller.is_quit_requested():
                break
            
            # Step physics and rendering
            self.world.step(render=True)
            
            # Capture and display camera frame (every 3 frames = ~16 FPS)
            frame_counter += 1
            if frame_counter >= 3:
                frame_counter = 0
                top_image = self.get_top_camera_image()
                if top_image is not None:
                    try:
                        # Display image in OpenCV window
                        cv2.imshow(self.window_name, top_image)
                        
                        # Check for OpenCV window close or ESC key
                        key = cv2.waitKey(1) & 0xFF
                        if key == 27:  # ESC key
                            print("ESC pressed - quitting...")
                            break
                    except cv2.error as e:
                        # Handle OpenCV errors (window might be closed)
                        print(f"OpenCV display error: {e}")
                        break
            
            # Check if OpenCV window was closed
            try:
                if cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) < 1:
                    print("OpenCV window closed - quitting...")
                    break
            except cv2.error:
                # Window might have been destroyed
                break

    def cleanup(self):
        """Cleanup resources: stop controller, detach camera annotators, and remove physics callback"""
        # Stop keyboard controller thread
        if self.controller:
            self.controller.stop()

        # Close OpenCV window
        try:
            cv2.destroyAllWindows()
        except Exception as e:
            print(f"Warning: Error closing OpenCV window: {e}")

        # Cleanup camera render products and annotators
        if self.camera_render_initialized:
            try:
                # Try to detach annotators more carefully
                for camera_type, annotator in self.rgb_annotators.items():
                    if camera_type in self.render_products:
                        try:
                            # Check if render product still exists before detaching
                            render_product = self.render_products[camera_type]
                            if render_product is not None:
                                annotator.detach([render_product])
                        except AttributeError:
                            # Handle case where detach fails due to internal issues
                            pass
                        except Exception as e:
                            # Silently ignore detach errors (common during shutdown)
                            pass
                print("Camera annotators cleaned up")
            except Exception as e:
                # Silently ignore cleanup errors during shutdown
                pass

        # Remove physics callback
        try:
            if self.world and self.world.physics_callback_exists("physics_step"):
                self.world.remove_physics_callback("physics_step")
        except Exception as e:
            print(f"Warning: Error removing physics callback: {e}")

        print("Cleanup complete")


# ===================== Main Entry Point =====================
def main():
    """Main entry point for the demo"""
    # Create demo instance
    demo = CV2SpotDemo()

    # Setup simulation (environment, robot, controller, cameras)
    demo.setup()

    # Run simulation loop
    try:
        demo.run()
    finally:
        # Always cleanup resources
        demo.cleanup()
        simulation_app.close()
        print("[INFO]: Simulation closed")


if __name__ == "__main__":
    main()

