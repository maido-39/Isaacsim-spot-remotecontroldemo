# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

"""
Spot robot demo with keyboard control, sample box, and top-view camera for web streaming
"""

# Launch Isaac Sim before any other imports
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})  # Run in headless mode for web streaming

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
import threading
import time
import io
from flask import Flask, Response, render_template_string, request, jsonify
from PIL import Image
from keyboard_controller import KeyboardController


class WebController:
    """
    Web-based controller that accepts commands via HTTP API.
    Provides same interface as KeyboardController but for web control.
    """
    
    def __init__(
        self,
        max_vx: float = 2.0,
        max_vy: float = 2.0,
        max_yaw: float = 2.0,
        acc_vx: float = 5.0,
        acc_vy: float = 5.0,
        acc_yaw: float = 10.0,
        decay_vx: float = 0.7,
        decay_vy: float = 0.7,
        decay_yaw: float = 0.6,
        update_dt: float = 0.02,
        eps_linear: float = 0.001,
        eps_angular: float = 0.001,
    ):
        """Initialize web controller with same parameters as KeyboardController"""
        self.max_vx = max_vx
        self.max_vy = max_vy
        self.max_yaw = max_yaw
        self.acc_vx = acc_vx
        self.acc_vy = acc_vy
        self.acc_yaw = acc_yaw
        self.decay_vx = decay_vx
        self.decay_vy = decay_vy
        self.decay_yaw = decay_yaw
        self.update_dt = update_dt
        self.eps_linear = eps_linear
        self.eps_angular = eps_angular
        
        # Web command state (set via HTTP API)
        self._key_state = {
            'x_pos': False,
            'x_neg': False,
            'y_pos': False,
            'y_neg': False,
            'yaw_pos': False,
            'yaw_neg': False,
            'quit': False
        }
        self._state_lock = threading.Lock()
        
        # Accumulated command state
        self._vx_cmd = 0.0
        self._vy_cmd = 0.0
        self._yaw_cmd = 0.0
    
    def set_key_state(self, key: str, pressed: bool):
        """Set key state from web API"""
        with self._state_lock:
            if key in self._key_state:
                self._key_state[key] = pressed
    
    def get_command(self) -> np.ndarray:
        """Get current command [vx, vy, yaw]"""
        return np.array([self._vx_cmd, self._vy_cmd, self._yaw_cmd])
    
    def update(self):
        """Update commands based on key state (same logic as KeyboardController)"""
        with self._state_lock:
            in_x = float(self._key_state['x_pos']) - float(self._key_state['x_neg'])
            in_y = float(self._key_state['y_pos']) - float(self._key_state['y_neg'])
            in_yaw = float(self._key_state['yaw_pos']) - float(self._key_state['yaw_neg'])
        
        # Integrate acceleration & apply decay
        if in_x != 0.0:
            self._vx_cmd += in_x * self.acc_vx * self.update_dt
        else:
            self._vx_cmd *= self.decay_vx
            
        if in_y != 0.0:
            self._vy_cmd += in_y * self.acc_vy * self.update_dt
        else:
            self._vy_cmd *= self.decay_vy
            
        if in_yaw != 0.0:
            self._yaw_cmd += in_yaw * self.acc_yaw * self.update_dt
        else:
            self._yaw_cmd *= self.decay_yaw
        
        # Clamp speeds
        self._vx_cmd = max(-self.max_vx, min(self.max_vx, self._vx_cmd))
        self._vy_cmd = max(-self.max_vy, min(self.max_vy, self._vy_cmd))
        self._yaw_cmd = max(-self.max_yaw, min(self.max_yaw, self._yaw_cmd))
        
        # Zero out small values
        if abs(self._vx_cmd) < self.eps_linear:
            self._vx_cmd = 0.0
        if abs(self._vy_cmd) < self.eps_linear:
            self._vy_cmd = 0.0
        if abs(self._yaw_cmd) < self.eps_angular:
            self._yaw_cmd = 0.0
    
    def is_quit_requested(self) -> bool:
        """Check if quit is requested"""
        with self._state_lock:
            return self._key_state['quit']
    
    def start(self):
        """Start controller (no-op for web controller)"""
        pass
    
    def stop(self):
        """Stop controller"""
        with self._state_lock:
            self._key_state['quit'] = True


class WebStreamSpotDemo:
    """Spot robot demo with keyboard control, sample box, and top-view camera for web streaming"""

    def __init__(self, web_port=5000):
        """Initialize the demo"""
        self.world = None
        self.stage = None
        self.spot = None
        self.keyboard_controller = None  # Local keyboard controller
        self.web_controller = None  # Web-based controller
        self.physics_ready = False
        self.command_counter = 0
        self.camera_path = None  # Top camera path
        self.render_products = {}  # Camera render products for streaming
        self.rgb_annotators = {}  # RGB annotators for camera capture
        self.camera_render_initialized = False
        
        # Web server setup
        self.web_port = web_port
        self.app = Flask(__name__)
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self._setup_flask_routes()

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
        This enables camera image capture for web streaming.
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

    def _set_default_viewport_camera(self, camera_path):
        """
        Set the top camera as the default viewport camera (replaces main scene view).
        Note: In headless mode, viewport is not available.
        """
        # Skip in headless mode
        pass

    def _setup_flask_routes(self):
        """Setup Flask routes for web streaming and control"""
        
        HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>Spot Robot Web Stream</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        body {
            font-family: Arial, sans-serif;
            background: #000;
            color: #fff;
            overflow: hidden;
            width: 100vw;
            height: 100vh;
        }
        .video-container {
            width: 100vw;
            height: 100vh;
            display: flex;
            align-items: center;
            justify-content: center;
            background: #000;
        }
        img {
            max-width: 100%;
            max-height: 100%;
            width: auto;
            height: auto;
            object-fit: contain;
        }
        .status {
            position: absolute;
            top: 10px;
            right: 10px;
            padding: 8px 12px;
            background: rgba(42, 42, 42, 0.8);
            border-radius: 4px;
            font-size: 12px;
            color: #4CAF50;
        }
    </style>
</head>
<body>
    <div class="video-container">
        <img id="videoStream" src="/video_feed" alt="Camera Feed">
    </div>
    
    <div class="status">
        <div>Status: <span id="status">Connected</span></div>
        <div>Commands: <span id="commands">vx: 0.00, vy: 0.00, yaw: 0.00</span></div>
    </div>
    
    <script>
        // Keyboard controls
        const keyMap = {
            'i': 'x_pos', 'I': 'x_pos',
            'k': 'x_neg', 'K': 'x_neg',
            'j': 'y_pos', 'J': 'y_pos',
            'l': 'y_neg', 'L': 'y_neg',
            'u': 'yaw_pos', 'U': 'yaw_pos',
            'o': 'yaw_neg', 'O': 'yaw_neg'
        };
        
        document.addEventListener('keydown', (e) => {
            if (keyMap[e.key]) {
                sendCommand(keyMap[e.key], true);
            }
        });
        
        document.addEventListener('keyup', (e) => {
            if (keyMap[e.key]) {
                sendCommand(keyMap[e.key], false);
            }
        });
        
        function sendCommand(key, pressed) {
            fetch('/control', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({key: key, pressed: pressed})
            }).catch(err => console.error('Command error:', err));
        }
        
        // Update command status
        setInterval(() => {
            fetch('/status')
                .then(r => r.json())
                .then(data => {
                    document.getElementById('commands').textContent = 
                        `vx: ${data.vx.toFixed(2)}, vy: ${data.vy.toFixed(2)}, yaw: ${data.yaw.toFixed(2)}`;
                })
                .catch(err => console.error('Status error:', err));
        }, 100);
    </script>
</body>
</html>
"""
        
        @self.app.route('/')
        def index():
            return render_template_string(HTML_TEMPLATE)
        
        @self.app.route('/video_feed')
        def video_feed():
            """Video streaming route"""
            def generate():
                while True:
                    with self.frame_lock:
                        if self.latest_frame is not None:
                            # Convert numpy array to JPEG
                            img = Image.fromarray(self.latest_frame)
                            buf = io.BytesIO()
                            img.save(buf, format='JPEG', quality=85)
                            frame = buf.getvalue()
                            yield (b'--frame\r\n'
                                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                    time.sleep(0.033)  # ~30 FPS
            
            return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')
        
        @self.app.route('/control', methods=['POST'])
        def control():
            """Handle control commands from web"""
            data = request.json
            key = data.get('key')
            pressed = data.get('pressed', False)
            
            if self.web_controller and hasattr(self.web_controller, 'set_key_state'):
                self.web_controller.set_key_state(key, pressed)
                return jsonify({'status': 'ok'})
            return jsonify({'status': 'error', 'message': 'Web controller not available'}), 400
        
        @self.app.route('/status')
        def status():
            """Get current robot status"""
            # Get merged command from both controllers
            cmd = self._get_merged_command()
            return jsonify({
                'vx': float(cmd[0]),
                'vy': float(cmd[1]),
                'yaw': float(cmd[2])
            })

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
            rotation_quat=rotation_quat,  # -90° around X axis (looking down)
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

    def _get_merged_command(self):
        """
        Merge commands from both keyboard and web controllers.
        Takes the maximum absolute value from each controller for each axis.
        """
        kbd_cmd = np.array([0.0, 0.0, 0.0])
        web_cmd = np.array([0.0, 0.0, 0.0])
        
        if self.keyboard_controller:
            kbd_cmd = self.keyboard_controller.get_command()
        if self.web_controller:
            web_cmd = self.web_controller.get_command()
        
        # Merge: take the maximum absolute value, preserving sign
        merged = np.array([
            kbd_cmd[0] if abs(kbd_cmd[0]) > abs(web_cmd[0]) else web_cmd[0],
            kbd_cmd[1] if abs(kbd_cmd[1]) > abs(web_cmd[1]) else web_cmd[1],
            kbd_cmd[2] if abs(kbd_cmd[2]) > abs(web_cmd[2]) else web_cmd[2]
        ])
        
        return merged

    def _on_physics_step(self, step_size):
        """
        Physics step callback - called every physics timestep (500Hz).
        Handles command updates (50Hz) and robot control.
        """
        # Command update: update both controllers at 50Hz (every 10 physics steps)
        self.command_counter += 1
        if self.command_counter >= 10:
            self.command_counter = 0
            if self.keyboard_controller:
                self.keyboard_controller.update()  # Update keyboard controller
            if self.web_controller:
                self.web_controller.update()  # Update web controller

        # Robot control: apply merged commands to robot
        if self.physics_ready:
            # Robot is initialized, apply forward control with merged command
            merged_cmd = self._get_merged_command()
            self.spot.forward(step_size, merged_cmd)
        else:
            # First physics step: initialize robot
            self.physics_ready = True
            self.spot.initialize()  # Initialize robot policy
            self.spot.post_reset()  # Post-reset setup
            self.spot.robot.set_joints_default_state(self.spot.default_pos)  # Set default joint positions
            print("Spot initialized")

    def get_top_camera_image(self):
        """
        Get current frame from top camera for web streaming.
        
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
            import numpy as np
            image_array = np.asarray(rgb_data)
            
            # Handle RGBA to RGB conversion if needed
            if len(image_array.shape) == 3 and image_array.shape[2] == 4:
                # Convert RGBA to RGB
                image_array = image_array[:, :, :3]
            
            return image_array
            
        except Exception as e:
            print(f"Warning: Failed to get top camera image: {e}")
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

        # Create local keyboard controller
        self.keyboard_controller = KeyboardController(
            max_vx=2.0, max_vy=2.0, max_yaw=2.0,
            acc_vx=5.0, acc_vy=5.0, acc_yaw=10.0,
            decay_vx=0.7, decay_vy=0.7, decay_yaw=0.6,
            update_dt=0.02  # 50Hz update rate
        )

        # Create web controller for web-based control
        self.web_controller = WebController(
            max_vx=2.0, max_vy=2.0, max_yaw=2.0,
            acc_vx=5.0, acc_vy=5.0, acc_yaw=10.0,
            decay_vx=0.7, decay_vy=0.7, decay_yaw=0.6,
            update_dt=0.02  # 50Hz update rate
        )

        # Reset world (required before querying articulation properties)
        self.world.reset()

        # Initialize camera render products for web streaming
        self._initialize_camera_render_products()

        # Register physics callback for robot control
        self.world.add_physics_callback("physics_step", callback_fn=self._on_physics_step)

        # Start keyboard controller (runs in separate thread for local keyboard input)
        self.keyboard_controller.start()

        # Start web controller
        self.web_controller.start()

        # Start Flask web server in separate thread
        self._start_web_server()

        print("Setup complete")
        print(f"Web server running at http://localhost:{self.web_port}")
        print("Local keyboard controls: i/k (x), j/l (y), u/o (yaw), ESC (quit)")

    def _start_web_server(self):
        """Start Flask web server in a separate thread"""
        def run_flask():
            self.app.run(host='0.0.0.0', port=self.web_port, debug=False, use_reloader=False, threaded=True)
        
        flask_thread = threading.Thread(target=run_flask, daemon=True)
        flask_thread.start()
        time.sleep(1)  # Give server time to start
        print(f"✓ Web server started on port {self.web_port}")

    def run(self):
        """Run main simulation loop"""
        if self.world is None or self.spot is None:
            raise RuntimeError("Simulation must be setup first")

        print("Starting simulation...")
        print(f"Web interface available at http://localhost:{self.web_port}")
        print("Local keyboard controls: i/k (x), j/l (y), u/o (yaw), ESC (quit)")
        print("Web controls: Use web interface buttons or keyboard in browser")

        frame_counter = 0
        
        # Main simulation loop
        while simulation_app.is_running():
            # Check if quit requested from keyboard controller (local)
            if self.keyboard_controller and self.keyboard_controller.is_quit_requested():
                break
            
            # Step physics and rendering
            self.world.step(render=True)
            
            # Capture camera frame for web streaming (every 3 frames = ~16 FPS)
            frame_counter += 1
            if frame_counter >= 3:
                frame_counter = 0
                top_image = self.get_top_camera_image()
                if top_image is not None:
                    with self.frame_lock:
                        self.latest_frame = top_image.copy()

    def cleanup(self):
        """Cleanup resources: stop controllers, detach camera annotators, and remove physics callback"""
        # Stop keyboard controller thread (local)
        if self.keyboard_controller:
            self.keyboard_controller.stop()
        
        # Stop web controller
        if self.web_controller:
            self.web_controller.stop()

        # Cleanup camera render products and annotators
        if self.camera_render_initialized:
            try:
                for camera_type, annotator in self.rgb_annotators.items():
                    if camera_type in self.render_products:
                        annotator.detach([self.render_products[camera_type]])
                print("Camera annotators detached")
            except Exception as e:
                print(f"Warning: Error cleaning up camera annotators: {e}")

        # Remove physics callback
        if self.world and self.world.physics_callback_exists("physics_step"):
            self.world.remove_physics_callback("physics_step")

        print("Cleanup complete")


# ===================== Main Entry Point =====================
def main():
    """Main entry point for the demo"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Spot Robot Web Streaming Demo")
    parser.add_argument('--port', type=int, default=5000, help='Web server port (default: 5000)')
    args = parser.parse_args()
    
    # Create demo instance
    demo = WebStreamSpotDemo(web_port=args.port)

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

