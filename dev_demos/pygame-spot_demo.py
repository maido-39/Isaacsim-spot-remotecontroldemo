# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

"""
Spot robot demo with keyboard control, sample box, and top-view camera displayed with Pygame
Pygame provides much smoother high-FPS display compared to cv2.imshow
"""

# Launch Isaac Sim before any other imports
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})  # Headless for Pygame display

# SimulationApp 초기화 후에만 다른 모듈을 import할 수 있습니다.
import sys
from pathlib import Path

# Add parent directory to path to import keyboard_controller
parent_dir = Path(__file__).parent.parent
sys.path.insert(0, str(parent_dir))

import numpy as np
import threading
import queue
import time
from pxr import Gf, UsdPhysics, Sdf, UsdGeom
import omni
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy
from keyboard_controller import KeyboardController
import pygame


class PygameDisplay:
    """
    High-performance Pygame display for smooth video streaming.
    Uses separate thread to avoid blocking main simulation loop.
    Also handles keyboard input for robot control.
    """
    
    def __init__(self, window_size=(800, 800), window_title="Spot Robot - Top View Camera", key_state_callback=None):
        self.window_size = window_size
        self.window_title = window_title
        self.frame_queue = queue.Queue(maxsize=2)  # Keep only latest 2 frames (drop old frames)
        self.running = False
        self.display_thread = None
        self.screen = None
        self.clock = None
        self.key_state_callback = key_state_callback  # Callback to update key state
        
    def start(self):
        """Start display thread"""
        self.running = True
        self.display_thread = threading.Thread(target=self._display_loop, daemon=True)
        self.display_thread.start()
        time.sleep(0.1)  # Give thread time to initialize
        print(f"✓ Pygame display started: {self.window_size[0]}×{self.window_size[1]}")
    
    def stop(self):
        """Stop display thread"""
        self.running = False
        if self.display_thread:
            self.display_thread.join(timeout=1.0)
        if self.screen:
            pygame.quit()
        print("Pygame display stopped")
    
    def update_frame(self, image_array):
        """
        Update display with new frame (non-blocking, drops old frames if queue full)
        
        Args:
            image_array: numpy array (H, W, 3) in RGB format
        """
        if not self.running:
            return
        
        # Convert numpy array to pygame surface
        try:
            # Ensure image is in correct format: (H, W, 3) RGB uint8
            if image_array.dtype != np.uint8:
                image_array = (image_array * 255).astype(np.uint8) if image_array.max() <= 1.0 else image_array.astype(np.uint8)
            
            # Convert to pygame surface first
            # Pygame surfarray.make_surface expects array in (W, H, 3) format
            # Swap axes: (H, W, 3) -> (W, H, 3)
            image_swapped = np.swapaxes(image_array, 0, 1)
            surface = pygame.surfarray.make_surface(image_swapped)
            
            # Resize if needed using pygame (more efficient than numpy/cv2)
            if surface.get_size() != self.window_size:
                surface = pygame.transform.scale(surface, self.window_size)
            
            # Put in queue (non-blocking, drop if full)
            try:
                self.frame_queue.put_nowait(surface)
            except queue.Full:
                # Drop oldest frame and add new one
                try:
                    self.frame_queue.get_nowait()
                    self.frame_queue.put_nowait(surface)
                except queue.Empty:
                    pass
        except Exception as e:
            # Silently ignore conversion errors
            pass
    
    def _display_loop(self):
        """Display loop running in separate thread"""
        try:
            pygame.init()
            self.screen = pygame.display.set_mode(self.window_size, pygame.RESIZABLE)
            pygame.display.set_caption(self.window_title)
            self.clock = pygame.time.Clock()
            
            # Target FPS for display (60 FPS for smooth display)
            target_fps = 60
            
            while self.running:
                # Handle pygame events (window close, resize, keyboard, etc.)
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.running = False
                        if self.key_state_callback:
                            self.key_state_callback('quit', True)
                        break
                    elif event.type == pygame.VIDEORESIZE:
                        self.window_size = event.size
                        self.screen = pygame.display.set_mode(self.window_size, pygame.RESIZABLE)
                    elif event.type == pygame.KEYDOWN:
                        # Handle key press
                        if self.key_state_callback:
                            if event.key == pygame.K_i:
                                self.key_state_callback('x_pos', True)
                            elif event.key == pygame.K_k:
                                self.key_state_callback('x_neg', True)
                            elif event.key == pygame.K_j:
                                self.key_state_callback('y_pos', True)
                            elif event.key == pygame.K_l:
                                self.key_state_callback('y_neg', True)
                            elif event.key == pygame.K_u:
                                self.key_state_callback('yaw_pos', True)
                            elif event.key == pygame.K_o:
                                self.key_state_callback('yaw_neg', True)
                            elif event.key == pygame.K_ESCAPE:
                                self.key_state_callback('quit', True)
                                self.running = False
                    elif event.type == pygame.KEYUP:
                        # Handle key release
                        if self.key_state_callback:
                            if event.key == pygame.K_i:
                                self.key_state_callback('x_pos', False)
                            elif event.key == pygame.K_k:
                                self.key_state_callback('x_neg', False)
                            elif event.key == pygame.K_j:
                                self.key_state_callback('y_pos', False)
                            elif event.key == pygame.K_l:
                                self.key_state_callback('y_neg', False)
                            elif event.key == pygame.K_u:
                                self.key_state_callback('yaw_pos', False)
                            elif event.key == pygame.K_o:
                                self.key_state_callback('yaw_neg', False)
                
                # Get latest frame from queue (non-blocking)
                try:
                    frame_surface = self.frame_queue.get_nowait()
                    # Blit frame to screen
                    self.screen.blit(frame_surface, (0, 0))
                    pygame.display.flip()
                except queue.Empty:
                    # No new frame, just update display
                    pass
                
                # Limit to target FPS
                self.clock.tick(target_fps)
                
        except Exception as e:
            print(f"Pygame display error: {e}")
        finally:
            if self.screen:
                pygame.quit()


class PygameSpotDemo:
    """Spot robot demo with keyboard control, sample box, and top-view camera displayed with Pygame"""

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
        self.display = None  # Pygame display

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
        This enables camera image capture for Pygame display.
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
        # Position: Higher up with wider FOV to see full scene
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
            translation=(0.0, 0.0, 10.0),  # 45m above origin (higher for wider view)
            rotation_quat=rotation_quat,  # -90° around Z axis
            focal_length=24.0,  # mm (wider FOV)
            horizontal_aperture=36.0,  # mm (wider aperture for wider FOV)
            resolution=top_res,
            clipping_range=(0.1, 100.0)  # Wider clipping range
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
        Get current frame from top camera for Pygame display.
        
        Returns:
            numpy.ndarray: RGB image array (H, W, 3) or None if not available
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
            
            # Ensure RGB format (not BGR)
            # Pygame expects RGB format
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

        # Initialize camera render products for Pygame display
        self._initialize_camera_render_products()

        # Create keyboard controller (we'll handle input in display window, not separate window)
        self.controller = KeyboardController(
            max_vx=2.0, max_vy=2.0, max_yaw=2.0,
            acc_vx=5.0, acc_vy=5.0, acc_yaw=10.0,
            decay_vx=0.7, decay_vy=0.7, decay_yaw=0.6,
            update_dt=0.02  # 50Hz update rate
        )
        
        # Create callback function to update controller key state
        def update_key_state(key, pressed):
            with self.controller._state_lock:
                if key in self.controller._key_state:
                    self.controller._key_state[key] = pressed
        
        # Initialize Pygame display with keyboard callback
        self.display = PygameDisplay(
            window_size=(800, 800), 
            window_title="Spot Robot - Top View Camera (i/j/k/l: x,y | u/o: yaw | ESC: quit)",
            key_state_callback=update_key_state
        )
        self.display.start()

        # Register physics callback for robot control
        self.world.add_physics_callback("physics_step", callback_fn=self._on_physics_step)

        # Note: We don't call controller.start() because we handle keyboard input in display window
        # The controller.update() is called in _on_physics_step callback

        print("Setup complete")

    def run(self):
        """Run main simulation loop"""
        if self.world is None or self.spot is None or self.controller is None:
            raise RuntimeError("Simulation must be setup first")

        print("Starting simulation...")
        print("Controls: i/k (x), j/l (y), u/o (yaw), ESC (quit)")
        print("Camera view displayed in Pygame window (smooth 50+ FPS)")

        # Main simulation loop
        while simulation_app.is_running():
            # Check if quit requested from keyboard controller
            if self.controller.is_quit_requested():
                break
            
            # Check if Pygame window was closed
            if self.display and not self.display.running:
                print("Pygame window closed - quitting...")
                break
            
            # Step physics and rendering
            self.world.step(render=True)
            
            # Capture and update display with camera frame (every frame for smooth 50 FPS)
            top_image = self.get_top_camera_image()
            if top_image is not None and self.display:
                self.display.update_frame(top_image)

    def cleanup(self):
        """Cleanup resources: stop controller, detach camera annotators, and remove physics callback"""
        # Mark controller as stopped (we didn't start a separate thread, but set quit flag)
        if self.controller:
            with self.controller._state_lock:
                self.controller._key_state['quit'] = True

        # Stop Pygame display
        if self.display:
            self.display.stop()

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
    demo = PygameSpotDemo()

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

