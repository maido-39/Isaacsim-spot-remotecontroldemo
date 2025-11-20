# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

"""
Spot robot demo with keyboard control, sample box, and parameterized multi-camera system.
Cameras are positioned on a circle at Z=10m with radius 6m, pointing toward origin.
"""

# Launch Isaac Sim before any other imports
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

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


class SpotDemoMultiCamera:
    """Spot robot demo with keyboard control, sample box, and parameterized multi-camera system"""

    def __init__(self, num_cameras=6):
        """Initialize the demo
        
        Args:
            num_cameras: Number of cameras to create (default: 6)
                         Cameras are positioned on a circle with radius 6m at Z=10m
                         and point toward origin (0,0,0)
        """
        self.world = None
        self.stage = None
        self.spot = None
        self.controller = None
        self.physics_ready = False
        self.command_counter = 0
        self.num_cameras = num_cameras
        self.camera_paths = []  # List of camera paths
        self.render_products = {}  # Camera render products for display
        self.rgb_annotators = {}  # RGB annotators for camera capture
        self.camera_render_initialized = False
        self.window_name = "Spot Robot - Multi Camera View"

    def initialize(self):
        """Initialize Isaac Sim world and stage"""
        # Create World: physics at 500Hz, rendering at 50Hz
        self.world = World(physics_dt=1.0/500.0, rendering_dt=10.0/500.0, stage_units_in_meters=1.0)
        # Get USD stage for scene manipulation
        self.stage = omni.usd.get_context().get_stage()
        print("World initialized")

    def setup_environment(self):
        """Setup basic environment: ground and sample box"""
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

        print("Environment setup complete: ground and sample box created")

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

    def _look_at_quaternion(self, camera_pos, target_pos):
        """
        Calculate quaternion to make camera look at target.
        Camera's default -Z axis will point toward target.
        
        Args:
            camera_pos: Camera position (x, y, z)
            target_pos: Target position (x, y, z)
            
        Returns:
            tuple: Quaternion (w, x, y, z)
        """
        # Direction vector from camera to target
        direction = np.array(target_pos) - np.array(camera_pos)
        direction = direction / np.linalg.norm(direction)  # Normalize
        
        # Default camera forward is -Z axis
        default_forward = np.array([0.0, 0.0, -1.0])
        
        # Calculate rotation axis and angle
        axis = np.cross(default_forward, direction)
        axis_norm = np.linalg.norm(axis)
        
        if axis_norm < 1e-6:
            # Vectors are parallel, no rotation needed
            if np.dot(default_forward, direction) > 0:
                return (1.0, 0.0, 0.0, 0.0)  # Identity
            else:
                # Opposite direction, rotate 180 degrees around Y axis
                return (0.0, 0.0, 1.0, 0.0)
        
        axis = axis / axis_norm
        angle = np.arccos(np.clip(np.dot(default_forward, direction), -1.0, 1.0))
        
        # Convert axis-angle to quaternion
        half_angle = angle / 2.0
        w = np.cos(half_angle)
        sin_half = np.sin(half_angle)
        x, y, z = axis * sin_half
        
        return (float(w), float(x), float(y), float(z))

    def setup_cameras(self):
        """Setup parameterized cameras in a circle pointing at origin"""
        if self.world is None or self.stage is None:
            raise RuntimeError("World must be initialized first")
        
        camera_radius = 6.0  # meters
        camera_height = 10.0  # meters (Z position)
        camera_resolution = (640, 480)  # Resolution for each camera
        
        # Calculate angle between cameras
        angle_step = 2.0 * np.pi / self.num_cameras
        
        print(f"Setting up {self.num_cameras} cameras on circle (radius={camera_radius}m, height={camera_height}m)")
        
        for i in range(self.num_cameras):
            # Calculate camera position on circle
            angle = i * angle_step
            x = camera_radius * np.cos(angle)
            y = camera_radius * np.sin(angle)
            z = camera_height
            
            camera_pos = (x, y, z)
            target_pos = (0.0, 0.0, 0.0)  # Origin
            
            # Calculate rotation quaternion to look at origin
            rotation_quat = self._look_at_quaternion(camera_pos, target_pos)
            
            # Create camera
            camera_name = f"Camera_{i}"
            camera_path = self._create_camera(
                parent_path="/World",
                camera_name=camera_name,
                translation=camera_pos,
                rotation_quat=rotation_quat,
                focal_length=18.0,  # mm
                horizontal_aperture=36.0,  # mm
                resolution=camera_resolution,
                clipping_range=(0.1, 100.0)
            )
            
            if camera_path:
                self.camera_paths.append(camera_path)
                print(f"  ✓ Camera {i} created at ({x:.2f}, {y:.2f}, {z:.2f})")
            else:
                print(f"  ✗ Failed to create camera {i}")
        
        print(f"Created {len(self.camera_paths)} cameras")

    def _initialize_camera_render_products(self):
        """
        Initialize render products and annotators for all cameras.
        This enables camera image capture for cv2 display.
        """
        try:
            import omni.replicator.core as rep
            
            if len(self.camera_paths) == 0:
                print("Warning: No camera paths available for render product initialization")
                self.camera_render_initialized = False
                return
            
            camera_resolution = (640, 480)
            
            # Setup render products for each camera
            for i, camera_path in enumerate(self.camera_paths):
                camera_key = f"camera_{i}"
                self.render_products[camera_key] = rep.create.render_product(
                    camera_path, 
                    camera_resolution
                )
                self.rgb_annotators[camera_key] = rep.AnnotatorRegistry.get_annotator("rgb")
                self.rgb_annotators[camera_key].attach([self.render_products[camera_key]])
            
            self.camera_render_initialized = True
            print(f"✓ Camera render products initialized for {len(self.camera_paths)} cameras")
                
        except ImportError as e:
            print(f"Warning: Replicator not available, camera capture disabled: {e}")
            self.camera_render_initialized = False
        except Exception as e:
            print(f"Warning: Failed to initialize camera render products: {e}")
            self.camera_render_initialized = False

    def get_camera_images(self):
        """
        Get current frames from all cameras.
        
        Returns:
            list: List of BGR image arrays (H, W, 3) or None for unavailable cameras
        """
        if not self.camera_render_initialized:
            return [None] * len(self.camera_paths)
        
        images = []
        for i in range(len(self.camera_paths)):
            camera_key = f"camera_{i}"
            
            if camera_key not in self.rgb_annotators:
                images.append(None)
                continue
            
            try:
                rgb_data = self.rgb_annotators[camera_key].get_data()
                if rgb_data is None:
                    images.append(None)
                    continue
                
                # Convert to numpy array
                image_array = np.asarray(rgb_data)
                
                # Check if we have valid image data
                if image_array is None or image_array.size == 0:
                    images.append(None)
                    continue
                
                # Handle RGBA to RGB conversion if needed
                if len(image_array.shape) == 3 and image_array.shape[2] == 4:
                    image_array = image_array[:, :, :3]
                
                # Convert RGB to BGR for OpenCV (cv2 uses BGR format)
                if len(image_array.shape) == 3 and image_array.shape[2] == 3:
                    try:
                        image_array = cv2.cvtColor(image_array, cv2.COLOR_RGB2BGR)
                    except Exception:
                        pass
                
                images.append(image_array)
                
            except Exception as e:
                images.append(None)
        
        return images

    def merge_camera_images(self, images):
        """
        Merge multiple camera images into a grid layout.
        
        Args:
            images: List of image arrays (can contain None for missing images)
            
        Returns:
            numpy.ndarray: Merged image or None if no valid images
        """
        # Filter out None images
        valid_images = [img for img in images if img is not None]
        
        if len(valid_images) == 0:
            return None
        
        # Calculate grid layout
        num_images = len(valid_images)
        if num_images == 1:
            return valid_images[0]
        
        # Calculate grid dimensions
        cols = int(np.ceil(np.sqrt(num_images)))
        rows = int(np.ceil(num_images / cols))
        
        # Get image dimensions (assume all images have same size)
        h, w = valid_images[0].shape[:2]
        
        # Create merged image
        merged = np.zeros((rows * h, cols * w, 3), dtype=np.uint8)
        
        # Place images in grid
        for idx, img in enumerate(valid_images):
            row = idx // cols
            col = idx % cols
            merged[row*h:(row+1)*h, col*w:(col+1)*w] = img
        
        return merged

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

    def setup(self):
        """Complete simulation setup: environment, robot, controller, and cameras"""
        # Initialize world if not already done
        if self.world is None:
            self.initialize()

        # Setup environment (ground and box)
        self.setup_environment()

        # Setup robot at origin
        self.setup_robot()
        
        # Setup cameras
        self.setup_cameras()

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
        cv2.resizeWindow(self.window_name, 1280, 960)  # Initial window size

        print("Setup complete")

    def run(self):
        """Run main simulation loop"""
        if self.world is None or self.spot is None or self.controller is None:
            raise RuntimeError("Simulation must be setup first")

        print("Starting simulation...")
        print("Controls: i/k (x), j/l (y), u/o (yaw), ESC (quit)")
        print(f"Displaying {self.num_cameras} camera views at 5Hz")

        frame_counter = 0
        
        # Main simulation loop
        while simulation_app.is_running():
            # Check if quit requested from keyboard controller
            if self.controller.is_quit_requested():
                break
            
            # Step physics and rendering
            self.world.step(render=True)
            
            # Capture and display camera frames at 5Hz (every 10 frames at 50Hz rendering)
            frame_counter += 1
            if frame_counter >= 10:
                frame_counter = 0
                
                # Get images from all cameras
                images = self.get_camera_images()
                
                # Merge images into grid
                merged_image = self.merge_camera_images(images)
                
                if merged_image is not None:
                    try:
                        # Display merged image in OpenCV window
                        cv2.imshow(self.window_name, merged_image)
                        
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
                for camera_key, annotator in self.rgb_annotators.items():
                    if camera_key in self.render_products:
                        try:
                            # Check if render product still exists before detaching
                            render_product = self.render_products[camera_key]
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
    # Number of cameras (can be changed)
    # Examples: 2 cameras = 180deg apart, 6 cameras = 60deg apart
    num_cameras = 6
    
    # Create demo instance
    demo = SpotDemoMultiCamera(num_cameras=num_cameras)

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

