# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# Isaac Sim standalone 스크립트 with pygame keyboard control
# 참고: https://docs.isaacsim.omniverse.nvidia.com/4.5.0/core_api_tutorials/tutorial_core_hello_world.html#converting-the-example-to-a-standalone-application
# BaseSample 대신 World를 직접 사용하여 standalone application으로 변환
# pygame을 별도 스레드로 실행하여 키보드 입력 처리

# Launch Isaac Sim before any other imports
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# SimulationApp 초기화 후에만 다른 모듈을 import할 수 있습니다.
import numpy as np
import logging
import sys
import json
from pxr import Gf, UsdGeom, UsdPhysics, UsdShade, Sdf
import omni
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
from isaacsim.core.api.materials import PreviewSurface
import omni.kit.commands
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy
from keyboard_controller import KeyboardController


# ===================== Default Configuration =====================
DEFAULT_CONFIG = {
    # Fixed parameters
    "ground_color": [0.5, 0.5, 0.5],
    "wall_color": [0.7, 0.7, 0.7],
    "wall_height": 2.0,
    "map_size": 10.0,
    "ground_friction_static": 0.2,
    "ground_friction_dynamic": 0.2,
    "ground_restitution": 0.01,
    "box_friction_static": 0.8,
    "box_friction_dynamic": 0.7,
    "box_restitution": 0.1,
    "dome_light_intensity": 600.0,
    "marker_radius": 0.3,
    "goal_hemisphere_diameter": 1.0,  # Diameter of hemisphere at goal point (meters)
    "use_box": True,  # Whether to spawn the obstacle box in the environment
    
    # Randomizable parameters (will be set by randomization function)
    "start_position": [4.0, 4.0],
    "goal_position": [-4.0, -4.0],
    "box_position": [2.0, 0.0, 0.25],
    "box_scale": [1.0, 1.0, 0.5],
    "box_mass": 5.0,
    "box_color": [0.6, 0.4, 0.2],
    "robot_height": 0.65,
    
    # Randomization settings
    "randomize": True,  # Enable randomization by default
    "wall_inset": 1.0,  # Inset from walls for spawning (meters)
    "box_line_distance_min": 2.0,  # Minimum distance from start-goal line (meters)
    "box_line_distance_max": 3.0,  # Maximum distance from start-goal line (meters)
    "box_scale_range": [[0.5, 2.0], [0.5, 2.0], [0.3, 1.0]],
    "box_mass_range": [3.0, 10.0],
    
    # Controller settings
    "max_vx": 2.0,
    "max_vy": 2.0,
    "max_yaw": 2.0,
    "acc_vx": 5.0,
    "acc_vy": 5.0,
    "acc_yaw": 10.0,
    "decay_vx": 0.7,
    "decay_vy": 0.7,
    "decay_yaw": 0.6,
    "update_dt": 0.02,
}


# ===================== Main Simulation Class =====================
class SpotSimulation:
    """Main simulation class for Isaac Sim Spot robot control"""

    def __init__(self, config_file=None, **config_overrides):
        """
        Initialize simulation.
    
    Args:
            config_file: Path to JSON config file (optional)
            **config_overrides: Override any config values
        """
        # Load configuration: start with defaults, then load from file if provided, then apply overrides
        self.config = DEFAULT_CONFIG.copy()
        if config_file:
            with open(config_file, 'r') as f:
                file_config = json.load(f)
                self.config.update(file_config)
        self.config.update(config_overrides)
        
        # Setup logging system
        self._setup_logging()
        
        # Initialize Isaac Sim components (will be set up later)
        self.world = None          # Isaac Sim World instance
        self.stage = None          # USD Stage for scene manipulation
        self.spot = None           # Spot robot instance
        self.controller = None      # Keyboard controller instance
        
        # Simulation state variables
        self.physics_ready = False      # Flag to track if physics is initialized
        self.command_counter = 0       # Counter for command updates (50Hz)
        self.logging_counter = 0        # Counter for logging updates (10Hz)
        self.start_pos = None           # Start position [x, y]
        self.goal_pos = None            # Goal position [x, y]
        self.robot_camera_path = None   # Path to robot ego-view camera

    def _setup_logging(self):
        """
        Setup logging system with console output.
        Configures logger to output DEBUG level messages with timestamps.
        """
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.setLevel(logging.DEBUG)
        
        # Clear existing handlers to avoid duplicates
        if self.logger.handlers:
            self.logger.handlers.clear()
        
        # Create console handler that outputs to stdout
        handler = logging.StreamHandler(sys.stdout)
        handler.setFormatter(logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        ))
        self.logger.addHandler(handler)
        self.logger.propagate = False  # Prevent messages from propagating to root logger

    # ===================== Utility Functions =====================
    @staticmethod
    def quaternion_to_rpy(q):
        """
        Convert quaternion [w, x, y, z] to Euler angles RPY [roll, pitch, yaw] in radians.
        
        Uses standard quaternion to Euler angle conversion formulas.
        Handles gimbal lock case for pitch angle.
        
        Args:
            q: Quaternion as numpy array [w, x, y, z]
            
        Returns:
            np.ndarray: [roll, pitch, yaw] in radians
        """
        w, x, y, z = q[0], q[1], q[2], q[3]
        
        # Roll (rotation around X-axis)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (rotation around Y-axis)
        # Handle gimbal lock: if sinp is at ±1, use ±90 degrees
        sinp = 2 * (w * y - z * x)
        pitch = np.copysign(np.pi / 2, sinp) if abs(sinp) >= 1 else np.arcsin(sinp)
        
        # Yaw (rotation around Z-axis)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return np.array([roll, pitch, yaw])

    def _apply_randomization(self):
        """
        Apply randomization to environment parameters if enabled.
        Randomizes start/goal positions and box position with constraints:
        - All points inside walls with inset
        - Start and goal have at least 2/3 of map diagonal distance
        - Box is positioned 2-3m from line connecting start and goal
        """
        if not self.config["randomize"]:
            return
        
        rng = np.random
        cfg = self.config
        map_size = cfg["map_size"]
        wall_inset = cfg["wall_inset"]
        
        # Calculate valid spawn area (inside walls with inset)
        half_size = map_size / 2.0
        min_coord = -half_size + wall_inset
        max_coord = half_size - wall_inset
        
        # Calculate minimum distance between start and goal (2/3 of map diagonal)
        map_diagonal = np.sqrt(2) * (max_coord - min_coord)
        min_start_goal_distance = (2.0 / 3.0) * map_diagonal
        
        # Randomize start position within valid area
        max_attempts = 1000
        start_pos = None
        goal_pos = None
        distance = 0.0
        
        for attempt in range(max_attempts):
            # Generate random start position
            start_pos = np.array([
                rng.uniform(min_coord, max_coord),
                rng.uniform(min_coord, max_coord)
            ])
            
            # Generate goal position that satisfies distance constraint
            for goal_attempt in range(max_attempts):
                goal_pos = np.array([
                    rng.uniform(min_coord, max_coord),
                    rng.uniform(min_coord, max_coord)
                ])
                
                # Check if distance is sufficient
                distance = np.linalg.norm(goal_pos - start_pos)
                if distance >= min_start_goal_distance:
                    break
            
            # If we found valid start and goal, break
            if distance >= min_start_goal_distance:
                break
        
        if start_pos is None or goal_pos is None:
            # Fallback: use default positions if randomization fails
            self.logger.warning("Randomization failed, using default positions")
            start_pos = np.array(cfg["start_position"][:2])
            goal_pos = np.array(cfg["goal_position"][:2])
        
        self.config["start_position"] = start_pos.tolist()
        self.config["goal_position"] = goal_pos.tolist()
        
        # Randomize box properties only if use_box is True
        if cfg.get("use_box", True):
            # Calculate box position: 2-3m from line connecting start and goal
            # Vector from start to goal
            start_to_goal = goal_pos - start_pos
            line_length = np.linalg.norm(start_to_goal)
            line_direction = start_to_goal / line_length if line_length > 0 else np.array([1.0, 0.0])
            
            # Perpendicular direction (rotate 90 degrees)
            perp_direction = np.array([-line_direction[1], line_direction[0]])
            
            # Random distance along the line (between 0.2 and 0.8 of line length)
            t = rng.uniform(0.2, 0.8)
            point_on_line = start_pos + t * start_to_goal
            
            # Random distance perpendicular to line (between min and max)
            perp_distance = rng.uniform(cfg["box_line_distance_min"], cfg["box_line_distance_max"])
            # Random sign (left or right of line)
            perp_distance *= rng.choice([-1, 1])
            
            # Box position
            box_pos_2d = point_on_line + perp_distance * perp_direction
            
            # Clamp box position to valid area
            box_pos_2d = np.clip(box_pos_2d, min_coord, max_coord)
            
            # Keep original z coordinate
            self.config["box_position"] = [
                float(box_pos_2d[0]),
                float(box_pos_2d[1]),
                cfg["box_position"][2]
            ]
            
            # Randomize box scale in all three dimensions
            self.config["box_scale"] = [
                rng.uniform(*cfg["box_scale_range"][0]),  # x scale
                rng.uniform(*cfg["box_scale_range"][1]),  # y scale
                rng.uniform(*cfg["box_scale_range"][2])   # z scale
            ]
            
            # Randomize box mass within specified range
            self.config["box_mass"] = rng.uniform(*cfg["box_mass_range"])
            
            self.logger.info(f"Randomization applied:")
            self.logger.info(f"  Start: [{start_pos[0]:.2f}, {start_pos[1]:.2f}]")
            self.logger.info(f"  Goal: [{goal_pos[0]:.2f}, {goal_pos[1]:.2f}]")
            self.logger.info(f"  Start-Goal distance: {np.linalg.norm(goal_pos - start_pos):.2f} m (min: {min_start_goal_distance:.2f} m)")
            self.logger.info(f"  Box: [{box_pos_2d[0]:.2f}, {box_pos_2d[1]:.2f}]")
        else:
            # Box is disabled, only log start and goal
            self.logger.info(f"Randomization applied (box disabled):")
            self.logger.info(f"  Start: [{start_pos[0]:.2f}, {start_pos[1]:.2f}]")
            self.logger.info(f"  Goal: [{goal_pos[0]:.2f}, {goal_pos[1]:.2f}]")
            self.logger.info(f"  Start-Goal distance: {np.linalg.norm(goal_pos - start_pos):.2f} m (min: {min_start_goal_distance:.2f} m)")

    # ===================== Environment Setup =====================
    def setup_environment(self):
        """
        Setup simulation environment: ground, walls, boxes, markers, lighting.
        Creates all static and dynamic objects in the scene.
        """
        if self.world is None or self.stage is None:
            raise RuntimeError("World must be initialized first")
        
        # Apply randomization if enabled (modifies config values)
        self._apply_randomization()
        cfg = self.config
        
        # Convert configuration values to numpy arrays for Isaac Sim API
        ground_color = np.array(cfg["ground_color"])
        wall_color = np.array(cfg["wall_color"])
        box_color = np.array(cfg["box_color"])
        box_pos = np.array(cfg["box_position"])
        box_scale = np.array(cfg["box_scale"])
        start_pos = np.array(cfg["start_position"])
        goal_pos = np.array(cfg["goal_position"])
        
        # Store positions for robot setup
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        
        # 1. Add dome light for scene illumination
        omni.kit.commands.execute("CreatePrim", prim_path="/World/DomeLight", prim_type="DomeLight")
        light_prim = self.stage.GetPrimAtPath("/World/DomeLight")
        if light_prim.IsValid():
            light_prim.GetAttribute("inputs:intensity").Set(cfg["dome_light_intensity"])
    
        # 2. Create ground plane with physics properties
        self.world.scene.add_default_ground_plane(
            z_position=0,
            name="default_ground_plane",
        prim_path="/World/GroundPlane",
            static_friction=cfg["ground_friction_static"],
            dynamic_friction=cfg["ground_friction_dynamic"],
            restitution=cfg["ground_restitution"],
        )
        
        # 3. Apply ground material (gray color)
        ground_material = PreviewSurface(
            prim_path="/World/Materials/GroundMaterial",
            color=ground_color
        )
        ground_prim = self.stage.GetPrimAtPath("/World/GroundPlane/Environment/Geometry")
        material_prim = self.stage.GetPrimAtPath(ground_material.prim_path)
        # Bind material to ground geometry
        UsdShade.MaterialBindingAPI(ground_prim).Bind(UsdShade.Material(material_prim))
        
        # 4. Create boundary walls (4 walls forming a square enclosure)
        wall_thickness = 0.2
        half_size = cfg["map_size"] / 2.0
        
        # North wall (+Y direction)
        self.world.scene.add(FixedCuboid(
            prim_path="/World/WallNorth", name="wall_north",
            position=np.array([0.0, half_size, cfg["wall_height"] / 2.0]),
            scale=np.array([cfg["map_size"], wall_thickness, cfg["wall_height"]]),
            color=wall_color
        ))
        
        # South wall (-Y direction)
        self.world.scene.add(FixedCuboid(
            prim_path="/World/WallSouth", name="wall_south",
            position=np.array([0.0, -half_size, cfg["wall_height"] / 2.0]),
            scale=np.array([cfg["map_size"], wall_thickness, cfg["wall_height"]]),
            color=wall_color
        ))
        
        # East wall (+X direction)
        self.world.scene.add(FixedCuboid(
            prim_path="/World/WallEast", name="wall_east",
            position=np.array([half_size, 0.0, cfg["wall_height"] / 2.0]),
            scale=np.array([wall_thickness, cfg["map_size"], cfg["wall_height"]]),
            color=wall_color
        ))
        
        # West wall (-X direction)
        self.world.scene.add(FixedCuboid(
            prim_path="/World/WallWest", name="wall_west",
            position=np.array([-half_size, 0.0, cfg["wall_height"] / 2.0]),
            scale=np.array([wall_thickness, cfg["map_size"], cfg["wall_height"]]),
            color=wall_color
        ))
        
        # 5. Create dynamic obstacle box (can be pushed by robot) - only if use_box is True
        if cfg["use_box"]:
            self.world.scene.add(DynamicCuboid(
                prim_path="/World/ObstacleBox", name="obstacle_box",
                position=box_pos, scale=box_scale, color=box_color,
                mass=cfg["box_mass"], linear_velocity=np.array([0.0, 0.0, 0.0])
            ))
            
            # 6. Apply physics material to box (friction and restitution)
            box_prim = self.stage.GetPrimAtPath("/World/ObstacleBox")
            if box_prim.IsValid():
                physics_material_path = "/World/Materials/BoxPhysicsMaterial"
                # Create physics material with friction properties
                physics_material = UsdPhysics.MaterialAPI.Apply(
                    self.stage.DefinePrim(physics_material_path, "Material")
                )
                physics_material.CreateStaticFrictionAttr().Set(cfg["box_friction_static"])
                physics_material.CreateDynamicFrictionAttr().Set(cfg["box_friction_dynamic"])
                physics_material.CreateRestitutionAttr().Set(cfg["box_restitution"])
                
                # Bind physics material to box collider
                collider = UsdPhysics.CollisionAPI.Get(self.stage, "/World/ObstacleBox")
                if collider:
                    collider.GetPrim().CreateRelationship("physics:material").SetTargets(
                        [Sdf.Path(physics_material_path)]
                    )
        else:
            self.logger.info("Box spawning disabled (use_box=False)")
        
        # 7. Create start marker (red sphere) - visual only, no physics
        start_sphere = UsdGeom.Sphere.Define(self.stage, "/World/StartMarker")
        start_sphere.GetRadiusAttr().Set(cfg["marker_radius"])
        start_sphere.CreateDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.0, 0.0)])  # Red color
        start_xform = UsdGeom.Xformable(start_sphere)
        start_xform.ClearXformOpOrder()
        start_xform.AddTranslateOp().Set(Gf.Vec3d(float(start_pos[0]), float(start_pos[1]), 0.0))
        
        # 8. Create goal marker (blue sphere) - visual only, no physics
        goal_sphere = UsdGeom.Sphere.Define(self.stage, "/World/GoalMarker")
        goal_sphere.GetRadiusAttr().Set(cfg["marker_radius"])
        goal_sphere.CreateDisplayColorAttr().Set([Gf.Vec3f(0.0, 0.0, 1.0)])  # Blue color
        goal_xform = UsdGeom.Xformable(goal_sphere)
        goal_xform.ClearXformOpOrder()
        goal_xform.AddTranslateOp().Set(Gf.Vec3d(float(goal_pos[0]), float(goal_pos[1]), 0.0))
        
        # 9. Create camera on root prim and set as default view
        # Position: Z = +17m, Rotation: Z axis -90 degrees
        camera_path = "/World/Camera"
        camera = UsdGeom.Camera.Define(self.stage, camera_path)
        
        # Set camera position: [0, 0, 17]
        camera_xform = UsdGeom.Xformable(camera)
        camera_xform.ClearXformOpOrder()
        translate_op = camera_xform.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(0.0, 0.0, 45.0))
        
        # Set camera rotation: -90 degrees around Z axis
        # Rotation around Z axis: -90 degrees = -π/2 radians
        rotation_z = -np.pi / 2.0
        # Convert to quaternion for Z-axis rotation (use GfQuatf for camera)
        half_angle = rotation_z / 2.0
        quat = Gf.Quatf(
            float(np.cos(half_angle)),  # w
            0.0,                        # x
            0.0,                        # y
            float(np.sin(half_angle))   # z
        )
        rotate_op = camera_xform.AddOrientOp()
        rotate_op.Set(quat)
        
        # Store camera path for later activation (viewport may not be ready yet)
        self.camera_path = camera_path
        self.logger.info(f"Camera created at {camera_path}")
        
        self.logger.info("Environment setup complete")

    def setup_robot(self):
        """
        Setup Spot robot at start position, oriented to face the goal.
        Calculates yaw angle from start to goal and converts to quaternion.
        """
        if self.start_pos is None or self.goal_pos is None:
            raise RuntimeError("Environment must be setup first")
        
        # Calculate direction vector from start to goal
        direction = self.goal_pos - self.start_pos
        # Calculate yaw angle (rotation around Z-axis) using atan2
        yaw = np.arctan2(direction[1], direction[0])
        
        # Convert yaw to quaternion [w, x, y, z] for rotation around Z-axis
        half_yaw = yaw / 2.0
        orientation = np.array([
            np.cos(half_yaw),  # w component
            0.0,                # x component (no roll)
            0.0,                # y component (no pitch)
            np.sin(half_yaw)   # z component (yaw rotation)
        ])
        
        # Add Spot robot to scene at start position with calculated orientation
        # Robot z position is 0.0 (on the ground) - the robot model has its own base height
        self.spot = SpotFlatTerrainPolicy(
            prim_path="/World/Spot",
            name="Spot",
            position=np.array([self.start_pos[0], self.start_pos[1], 0.0]),
            orientation=orientation,
        )
        
        self.logger.info(f"Robot placed at [{self.start_pos[0]:.2f}, {self.start_pos[1]:.2f}]")
        
        # Create hemisphere at goal point
        self._create_goal_hemisphere()

    def _create_goal_hemisphere(self):
        """
        Create a hemisphere at the goal point with specified diameter.
        Hemisphere is positioned at goal point, with half sphere above ground.
        """
        try:
            cfg = self.config
            diameter = cfg["goal_hemisphere_diameter"]
            radius = diameter / 2.0
            
            # Create sphere for hemisphere (we'll position it so half is above ground)
            hemisphere_path = "/World/GoalHemisphere"
            hemisphere = UsdGeom.Sphere.Define(self.stage, hemisphere_path)
            hemisphere.GetRadiusAttr().Set(radius)
            
            # Set color (blue to match goal marker)
            hemisphere.CreateDisplayColorAttr().Set([Gf.Vec3f(0.0, 0.0, 1.0)])  # Blue color
            
            # Position sphere so that half is above ground (z = radius)
            # This creates the hemisphere effect
            hemisphere_xform = UsdGeom.Xformable(hemisphere)
            hemisphere_xform.ClearXformOpOrder()
            translate_op = hemisphere_xform.AddTranslateOp()
            translate_op.Set(Gf.Vec3d(
                float(self.goal_pos[0]), 
                float(self.goal_pos[1]), 
                float(radius)  # Position at radius height so half sphere is above ground
            ))
            
            self.logger.info(f"Goal hemisphere created at [{self.goal_pos[0]:.2f}, {self.goal_pos[1]:.2f}] with diameter {diameter:.2f}m")
            
        except Exception as e:
            self.logger.warning(f"Failed to create goal hemisphere: {e}")

    def _setup_robot_camera(self):
        """
        Create ego-view camera attached to Spot robot body.
        Creates a camera prim under /World/Spot/body with transformations, then adds camera under that prim.
        Transformations: z +0.2m, x -0.4m, rotation x 90°, y 90°
        Must be called after world reset when robot is fully initialized.
        """
        try:
            # Attach camera to /World/Spot/body
            body_path = "/World/Spot/body"
            body_prim = self.stage.GetPrimAtPath(body_path)
            if not body_prim.IsValid():
                self.logger.warning(f"Robot body prim not found at {body_path}, cannot attach camera")
                return
            
            # Create a prim for the camera with transformations
            camera_prim_path = f"{body_path}/CameraPrim"
            camera_prim = UsdGeom.Xform.Define(self.stage, camera_prim_path)
            
            # Apply transformations to the camera prim
            # Translation: (0, 0, 0) - no translation
            # Rotation: Quaternion WXYZ (0.5, 0.5, -0.5, -0.5) to avoid gimbal lock
            camera_xform = UsdGeom.Xformable(camera_prim)
            camera_xform.ClearXformOpOrder()
            
            # Add translation: (0, 0, 0)
            translate_op = camera_xform.AddTranslateOp()
            translate_op.Set(Gf.Vec3f(0.0, 0.0, 0.0))
            
            # Add rotation: Quaternion WXYZ (0.5, 0.5, -0.5, -0.5)
            quat = Gf.Quatf(0.5, 0.5, -0.5, -0.5)  # (w, x, y, z)
            
            # Add rotation operation
            rotate_op = camera_xform.AddOrientOp()
            rotate_op.Set(quat)
            
            # Create camera as child of the camera prim
            camera_path = f"{camera_prim_path}/EgoCamera"
            camera = UsdGeom.Camera.Define(self.stage, camera_path)
            
            # Set focal length to 24.7mm
            camera.GetFocalLengthAttr().Set(24.7)
            
            self.logger.info(f"Ego-view camera created at {camera_path} (quaternion WXYZ=[0.5, 0.5, -0.5, -0.5], translation=[0,0,0], focal_length=24.7mm)")
            
            # Store camera path for viewport creation
            self.robot_camera_path = camera_path
            
        except Exception as e:
            self.logger.warning(f"Failed to create robot camera: {e}")
    
    def _open_robot_camera_viewport(self, camera_path):
        """
        Open a new viewport window showing the robot's ego-view camera.
        """
        try:
            # Use Isaac Sim viewport utility to create new viewport window
            from isaacsim.core.utils.viewports import create_viewport_for_camera
            
            # Create new viewport window for robot camera
            create_viewport_for_camera(
                viewport_name="RobotEgoView",
                camera_prim_path=camera_path,
                width=800,
                height=600,
                position_x=100,
                position_y=100
            )
            self.logger.info(f"Robot ego-view camera window opened with camera {camera_path}")
            
        except ImportError:
            # Fallback: try alternative method if import fails
            try:
                import omni.kit.viewport.utility as vp_utils
                viewport = vp_utils.get_active_viewport()
                if viewport:
                    self.logger.info(f"Setting robot camera {camera_path} (fallback method - using existing viewport)")
            except Exception as e2:
                self.logger.warning(f"Could not open robot camera viewport window: {e2}")
        except Exception as e:
            self.logger.warning(f"Could not open robot camera viewport window: {e}")

    def _set_viewport_camera(self):
        """
        Set the camera as the default viewport camera.
        Called after world reset when viewport should be ready.
        """
        if not hasattr(self, 'camera_path') or self.camera_path is None:
            return
        
        try:
            # Method 1: Use viewport utility to get active viewport
            import omni.kit.viewport.utility as vp_utils
            viewport = vp_utils.get_active_viewport()
            if viewport:
                viewport.set_active_camera(self.camera_path)
                self.logger.info(f"Camera set as default view at {self.camera_path}")
                return
        except Exception as e:
            self.logger.debug(f"Method 1 failed: {e}")
        
        try:
            # Method 2: Try to get viewport by window name
            import omni.kit.viewport.utility as vp_utils
            viewport_window = vp_utils.get_viewport_from_window_name("Viewport")
            if viewport_window:
                viewport_window.set_active_camera(self.camera_path)
                self.logger.info(f"Camera set as default view at {self.camera_path} (method 2)")
                return
        except Exception as e:
            self.logger.debug(f"Method 2 failed: {e}")
        
        try:
            # Method 3: Use omni.kit.commands
            omni.kit.commands.execute(
                "SetActiveCamera",
                path=self.camera_path
            )
            self.logger.info(f"Camera set as default view at {self.camera_path} (method 3)")
            return
        except Exception as e:
            self.logger.debug(f"Method 3 failed: {e}")
        
        # If all methods fail, log warning but continue
        self.logger.warning(f"Could not set camera as default view. Camera exists at {self.camera_path}")

    def _on_physics_step(self, step_size):
        """
        Physics step callback - called every physics timestep (500Hz).
        Handles command updates (50Hz), logging (10Hz), and robot control.
        
        Args:
            step_size: Physics timestep size
        """
        # Command update: update controller at 50Hz (every 10 physics steps)
        # Physics runs at 500Hz, so 500Hz / 10 = 50Hz
        self.command_counter += 1
        if self.command_counter >= 10:
            self.command_counter = 0
            self.controller.update()  # Update controller state based on keyboard input
        
        # Logging: log robot state at 10Hz (every 50 physics steps)
        # Physics runs at 500Hz, so 500Hz / 50 = 10Hz
        self.logging_counter += 1
        if self.logging_counter >= 50:
            self.logging_counter = 0
            
            if self.physics_ready:
                # Get robot state
                robot_pos, robot_quat = self.spot.robot.get_world_pose()  # Position and quaternion
                robot_rpy = self.quaternion_to_rpy(robot_quat)  # Convert to roll, pitch, yaw
                cmd_vel = self.controller.get_command()  # Command velocity [vx, vy, yaw]
                
                # Get box position from stage (box is dynamic, so position can change)
                # Only if box is enabled in configuration
                l1_distance = 0.0
                if self.config.get("use_box", True):
                    box_prim = self.stage.GetPrimAtPath("/World/ObstacleBox")
                    if box_prim.IsValid():
                        # Get world transform of the box
                        box_xform = UsdGeom.Xformable(box_prim)
                        box_transform = box_xform.ComputeLocalToWorldTransform(0)  # Get transform at time 0
                        box_pos_world = box_transform.ExtractTranslation()
                        box_pos = np.array([box_pos_world[0], box_pos_world[1], box_pos_world[2]])
                        
                        # Calculate L1 distance (Manhattan distance) between box and goal
                        goal_pos_2d = np.array([self.goal_pos[0], self.goal_pos[1]])
                        l1_distance = np.sum(np.abs(box_pos[:2] - goal_pos_2d))
                
                # Log at DEBUG level: detailed robot state
                self.logger.debug(
                    f"Robot pos: [{robot_pos[0]:.2f}, {robot_pos[1]:.2f}, {robot_pos[2]:.2f}], "
                    f"RPY: [{np.degrees(robot_rpy[0]):.2f}, {np.degrees(robot_rpy[1]):.2f}, {np.degrees(robot_rpy[2]):.2f}]°, "
                    f"Cmd vel: [vx={cmd_vel[0]:.2f}, vy={cmd_vel[1]:.2f}, yaw={cmd_vel[2]:.2f}]"
                )
                # Log at INFO level: distance between box and goal (only if box exists)
                if self.config.get("use_box", True):
                    self.logger.info(
                        f"Box <-> Goal L1 distance: {l1_distance:.2f} m"
                    )
        
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
            self.logger.info("Spot initialized")

    # ===================== Main Simulation =====================
    def initialize(self):
        """
        Initialize Isaac Sim world and stage.
        Creates World instance with physics and rendering timesteps.
        """
        # Create World: physics at 500Hz, rendering at 50Hz
        self.world = World(physics_dt=1.0/500.0, rendering_dt=10.0/500.0, stage_units_in_meters=1.0)
        # Get USD stage for scene manipulation
        self.stage = omni.usd.get_context().get_stage()
        self.logger.info("World initialized")

    def setup(self):
        """
        Complete simulation setup: environment, robot, and controller.
        Must be called before run().
        """
        # Initialize world if not already done
        if self.world is None:
            self.initialize()
        
        # Setup environment (ground, walls, boxes, markers)
        self.setup_environment()
        
        # Setup robot at start position
        self.setup_robot()
        
        # Create keyboard controller with configuration parameters
        cfg = self.config
        self.controller = KeyboardController(
            max_vx=cfg["max_vx"], max_vy=cfg["max_vy"], max_yaw=cfg["max_yaw"],
            acc_vx=cfg["acc_vx"], acc_vy=cfg["acc_vy"], acc_yaw=cfg["acc_yaw"],
            decay_vx=cfg["decay_vx"], decay_vy=cfg["decay_vy"], decay_yaw=cfg["decay_yaw"],
            update_dt=cfg["update_dt"]  # 50Hz update rate
        )
        
        # Reset world (required before querying articulation properties)
        self.world.reset()
        
        # Create ego-view camera attached to robot base (after world reset, robot is fully initialized)
        self._setup_robot_camera()
        
        # Set camera as default viewport camera (after world reset, viewport should be ready)
        self._set_viewport_camera()
        
        # Open robot ego-view camera window (after world reset, viewport system should be ready)
        if hasattr(self, 'robot_camera_path') and self.robot_camera_path:
            self._open_robot_camera_viewport(self.robot_camera_path)
        
        # Register physics callback for robot control and logging
        self.world.add_physics_callback("physics_step", callback_fn=self._on_physics_step)
        
        # Start keyboard controller (runs in separate thread)
        self.controller.start()
        
        self.logger.info("Setup complete")

    def run(self):
        """
        Run main simulation loop.
        Steps physics and rendering until simulation is stopped or quit is requested.
        """
        if self.world is None or self.spot is None or self.controller is None:
            raise RuntimeError("Simulation must be setup first")
        
        self.logger.info("Starting simulation...")
        self.logger.info("Controls: i/k (x), j/l (y), u/o (yaw), ESC (quit)")
        
        # Main simulation loop
        while simulation_app.is_running():
            # Check if quit requested from keyboard controller
            if self.controller.is_quit_requested():
                break
            # Step physics and rendering
            self.world.step(render=True)

    def cleanup(self):
        """
        Cleanup resources: stop controller and remove physics callback.
        Should be called after simulation ends.
        """
        # Stop keyboard controller thread
        if self.controller:
            self.controller.stop()
        
        # Remove physics callback
        if self.world and self.world.physics_callback_exists("physics_step"):
            self.world.remove_physics_callback("physics_step")
        
        self.logger.info("Cleanup complete")


# ===================== Main Entry Point =====================
def main():
    """
    Main entry point for the simulation.
    
    Usage examples:
    1. Default configuration:
       sim = SpotSimulation()
    
    2. Load from JSON file:
       sim = SpotSimulation(config_file="example_config.json")
    
    3. Override specific values:
       sim = SpotSimulation(randomize=True, map_size=12.0)
    """
    # Create simulation instance
    # Can pass config_file="config.json" to load from file
    # Can pass keyword arguments to override config values
    sim = SpotSimulation()
    # sim = SpotSimulation(config_file="example_config.json")
    # sim = SpotSimulation(randomize=True, map_size=12.0)
    
    # Setup simulation (environment, robot, controller)
    sim.setup()
    
    # Run simulation loop
    try:
        sim.run()
    finally:
        # Always cleanup resources
        sim.cleanup()
        simulation_app.close()
        print("[INFO]: Simulation closed")


if __name__ == "__main__":
    main()
