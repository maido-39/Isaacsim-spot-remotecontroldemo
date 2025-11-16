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
# default first two lines in any standalone application
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})  # we can also run as headless.

# SimulationApp 초기화 후에만 다른 모듈을 import할 수 있습니다.
import numpy as np
import logging
import sys
from pxr import Gf, UsdGeom, UsdPhysics
import omni
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
from isaacsim.core.api.materials import PreviewSurface
from isaacsim.core.utils.stage import add_reference_to_stage
import omni.kit.commands
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy

# Import keyboard controller
from keyboard_controller import KeyboardController


# ===================== Logging Setup =====================
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

# Remove existing handlers to avoid duplicates
if logger.handlers:
    logger.handlers.clear()

# Create console handler with format
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s', 
                              datefmt='%Y-%m-%d %H:%M:%S')
console_handler.setFormatter(formatter)
logger.addHandler(console_handler)

# Prevent propagation to root logger to avoid duplicate messages
logger.propagate = False


# ===================== Utility Functions =====================
def quaternion_to_rpy(q: np.ndarray) -> np.ndarray:
    """
    Convert quaternion [w, x, y, z] to roll, pitch, yaw (RPY) in radians.
    
    Args:
        q: Quaternion as [w, x, y, z]
        
    Returns:
        np.ndarray: [roll, pitch, yaw] in radians
    """
    w, x, y, z = q[0], q[1], q[2], q[3]
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return np.array([roll, pitch, yaw])


# ===================== Environment Setup =====================
def setup_environment(
    world: World, 
    stage,
    # ===== Fixed Parameters =====
    ground_color: np.ndarray = np.array([0.5, 0.5, 0.5]),
    wall_color: np.ndarray = np.array([0.7, 0.7, 0.7]),
    wall_height: float = 2.0,
    map_size: float = 10.0,
    ground_friction_static: float = 0.2,
    ground_friction_dynamic: float = 0.2,
    ground_restitution: float = 0.01,
    box_friction_static: float = 0.8,
    box_friction_dynamic: float = 0.7,
    box_restitution: float = 0.1,
    dome_light_intensity: float = 600.0,
    marker_radius: float = 0.3,
    
    # ===== Randomizable Parameters =====
    start_position: np.ndarray = np.array([4.0, 4.0]),
    goal_position: np.ndarray = np.array([-4.0, -4.0]),
    box_position: np.ndarray = np.array([2.0, 0.0, 0.25]),
    box_scale: np.ndarray = np.array([1.0, 1.0, 0.5]),
    box_mass: float = 5.0,
    box_color: np.ndarray = np.array([0.6, 0.4, 0.2]),
    robot_height: float = 0.65,
    
    # ===== Randomization Settings =====
    randomize: bool = False,
    start_pos_range: tuple = ((-4.5, 4.5), (-4.5, 4.5)),  # ((x_min, x_max), (y_min, y_max))
    goal_pos_range: tuple = ((-4.5, 4.5), (-4.5, 4.5)),
    box_pos_range: tuple = ((-3.0, 3.0), (-3.0, 3.0)),
    box_scale_range: tuple = ((0.5, 2.0), (0.5, 2.0), (0.3, 1.0)),  # (x_range, y_range, z_range)
    box_mass_range: tuple = (3.0, 10.0),
):
    """
    환경 설정: 회색 바닥, 벽, 박스, 마커, 조명
    
    Args:
        world: Isaac Sim World instance
        stage: USD Stage
        
        Fixed Parameters:
            ground_color: Ground plane color (RGB)
            wall_color: Wall color (RGB)
            wall_height: Wall height (m)
            map_size: Map size (square, m)
            ground_friction_static: Ground static friction coefficient
            ground_friction_dynamic: Ground dynamic friction coefficient
            ground_restitution: Ground restitution coefficient
            box_friction_static: Box static friction coefficient
            box_friction_dynamic: Box dynamic friction coefficient
            box_restitution: Box restitution coefficient
            dome_light_intensity: Dome light intensity
            marker_radius: Start/goal marker sphere radius
            
        Randomizable Parameters:
            start_position: Start marker position [x, y]
            goal_position: Goal marker position [x, y]
            box_position: Obstacle box position [x, y, z]
            box_scale: Obstacle box scale [x, y, z]
            box_mass: Obstacle box mass (kg)
            box_color: Obstacle box color (RGB)
            robot_height: Robot height above ground (m)
            
        Randomization Settings:
            randomize: Enable randomization
            start_pos_range: Start position randomization range ((x_min, x_max), (y_min, y_max))
            goal_pos_range: Goal position randomization range
            box_pos_range: Box position randomization range
            box_scale_range: Box scale randomization range
            box_mass_range: Box mass randomization range (min, max)
    """
    
    # ===== Apply Randomization if Enabled =====
    if randomize:
        # Randomize start position
        start_position = np.array([
            np.random.uniform(*start_pos_range[0]),
            np.random.uniform(*start_pos_range[1])
        ])
        
        # Randomize goal position
        goal_position = np.array([
            np.random.uniform(*goal_pos_range[0]),
            np.random.uniform(*goal_pos_range[1])
        ])
        
        # Randomize box position (keep z)
        box_position = np.array([
            np.random.uniform(*box_pos_range[0]),
            np.random.uniform(*box_pos_range[1]),
            box_position[2]  # Keep original z
        ])
        
        # Randomize box scale
        box_scale = np.array([
            np.random.uniform(*box_scale_range[0]),
            np.random.uniform(*box_scale_range[1]),
            np.random.uniform(*box_scale_range[2])
        ])
        
        # Randomize box mass
        box_mass = np.random.uniform(*box_mass_range)
        
        print("[INFO] Randomization applied:")
        print(f"  - Start: {start_position}")
        print(f"  - Goal: {goal_position}")
        print(f"  - Box pos: {box_position}")
        print(f"  - Box scale: {box_scale}")
        print(f"  - Box mass: {box_mass:.2f} kg")
    
    # ===== Environment Setup =====
    # 0. 돔 라이트 추가
    omni.kit.commands.execute("CreatePrim", prim_path="/World/DomeLight", prim_type="DomeLight")
    light_prim = stage.GetPrimAtPath("/World/DomeLight")
    if light_prim.IsValid():
        light_prim.GetAttribute("inputs:intensity").Set(dome_light_intensity)
    print(f"[INFO] Dome light added with intensity {dome_light_intensity}")
    
    # 1. 기본 ground plane + 회색 머티리얼
    world.scene.add_default_ground_plane(
            z_position=0,
            name="default_ground_plane",
        prim_path="/World/GroundPlane",
        static_friction=ground_friction_static,
        dynamic_friction=ground_friction_dynamic,
        restitution=ground_restitution,
    )
    
    # Ground plane에 회색 머티리얼 적용 (theGrid 머티리얼 오버라이드)
    from pxr import UsdShade
    
    ground_material = PreviewSurface(
        prim_path="/World/Materials/GroundMaterial",
        color=ground_color
    )
    
    # Ground plane의 geometry prim에 머티리얼 바인딩
    ground_geom_path = "/World/GroundPlane/Environment/Geometry"
    ground_prim = stage.GetPrimAtPath(ground_geom_path)
    material_prim = stage.GetPrimAtPath(ground_material.prim_path)
    
    binding_api = UsdShade.MaterialBindingAPI(ground_prim)
    binding_api.Bind(UsdShade.Material(material_prim))
    print(f"[INFO] Ground plane material bound to: {ground_geom_path}")
    
    # 2. 맵 경계 벽 만들기 (4개의 벽)
    wall_thickness = 0.2
    half_size = map_size / 2.0
    
    # 북쪽 벽 (+Y)
    world.scene.add(
        FixedCuboid(
            prim_path="/World/WallNorth",
            name="wall_north",
            position=np.array([0.0, half_size, wall_height / 2.0]),
            scale=np.array([map_size, wall_thickness, wall_height]),
            color=wall_color
        )
    )
    
    # 남쪽 벽 (-Y)
    world.scene.add(
        FixedCuboid(
            prim_path="/World/WallSouth",
            name="wall_south",
            position=np.array([0.0, -half_size, wall_height / 2.0]),
            scale=np.array([map_size, wall_thickness, wall_height]),
            color=wall_color
        )
    )
    
    # 동쪽 벽 (+X)
    world.scene.add(
        FixedCuboid(
            prim_path="/World/WallEast",
            name="wall_east",
            position=np.array([half_size, 0.0, wall_height / 2.0]),
            scale=np.array([wall_thickness, map_size, wall_height]),
            color=wall_color
        )
    )
    
    # 서쪽 벽 (-X)
    world.scene.add(
        FixedCuboid(
            prim_path="/World/WallWest",
            name="wall_west",
            position=np.array([-half_size, 0.0, wall_height / 2.0]),
            scale=np.array([wall_thickness, map_size, wall_height]),
            color=wall_color
        )
    )
    
    # 3. 마찰이 있는 박스 (장애물로 사용 가능)
    obstacle_box = world.scene.add(
        DynamicCuboid(
            prim_path="/World/ObstacleBox",
            name="obstacle_box",
            position=box_position,
            scale=box_scale,
            color=box_color,
            mass=box_mass,
            linear_velocity=np.array([0.0, 0.0, 0.0])
        )
    )
    
    # 박스에 마찰 속성 설정
    box_prim = stage.GetPrimAtPath("/World/ObstacleBox")
    if box_prim.IsValid():
        physics_material_path = "/World/Materials/BoxPhysicsMaterial"
        from pxr import UsdShade, PhysxSchema
        
        # Physics material 생성
        physics_material = UsdPhysics.MaterialAPI.Apply(
            stage.DefinePrim(physics_material_path, "Material")
        )
        physics_material.CreateStaticFrictionAttr().Set(box_friction_static)
        physics_material.CreateDynamicFrictionAttr().Set(box_friction_dynamic)
        physics_material.CreateRestitutionAttr().Set(box_restitution)
        
        # Apply physics material to box
        collider = UsdPhysics.CollisionAPI.Get(stage, "/World/ObstacleBox")
        if collider:
            from pxr import Sdf
            collider.GetPrim().CreateRelationship(
                "physics:material"
            ).SetTargets([Sdf.Path(physics_material_path)])
    
    # 4. 반구형 마커 2개 (충돌체 없음, 반구만 보이도록)
    # 빨간색 시작점
    start_marker_path = "/World/StartMarker"
    start_sphere = UsdGeom.Sphere.Define(stage, start_marker_path)
    start_sphere.GetRadiusAttr().Set(marker_radius)
    start_sphere.CreateDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.0, 0.0)])  # 빨간색
    
    # 위치 설정 및 반구만 보이도록 (아래쪽 절반은 바닥 아래)
    start_xform = UsdGeom.Xformable(start_sphere)
    start_xform.ClearXformOpOrder()
    start_xform.AddTranslateOp().Set(Gf.Vec3d(float(start_position[0]), float(start_position[1]), 0.0))
    
    # 충돌체 없음 (physics 설정 안 함)
    
    # 파란색 목표점
    goal_marker_path = "/World/GoalMarker"
    goal_sphere = UsdGeom.Sphere.Define(stage, goal_marker_path)
    goal_sphere.GetRadiusAttr().Set(marker_radius)
    goal_sphere.CreateDisplayColorAttr().Set([Gf.Vec3f(0.0, 0.0, 1.0)])  # 파란색
    
    # 위치 설정
    goal_xform = UsdGeom.Xformable(goal_sphere)
    goal_xform.ClearXformOpOrder()
    goal_xform.AddTranslateOp().Set(Gf.Vec3d(float(goal_position[0]), float(goal_position[1]), 0.0))
    
    print("[INFO] Environment setup complete:")
    print(f"  - Dome light with intensity {dome_light_intensity}")
    print(f"  - Ground: color={ground_color}, friction=({ground_friction_static}, {ground_friction_dynamic})")
    print(f"  - Walls: {map_size}M x {map_size}M, height={wall_height}M, color={wall_color}")
    print(f"  - Obstacle box: pos={box_position}, scale={box_scale}, mass={box_mass:.2f}kg")
    print(f"  - Start marker (RED): pos={start_position}, radius={marker_radius}")
    print(f"  - Goal marker (BLUE): pos={goal_position}, radius={marker_radius}")
    print(f"  - Robot height: {robot_height}m")
    
    # Return start position, goal position, and robot height for robot placement
    return start_position, goal_position, robot_height


# ===================== Main Script =====================
# World 초기화
world = World(physics_dt=1.0 / 500.0, rendering_dt=10.0 / 500.0, stage_units_in_meters=1.0)

# USD Stage 가져오기
stage = omni.usd.get_context().get_stage()

# 환경 설정 (기본값 사용)
start_pos, goal_pos, robot_height = setup_environment(world, stage)

# ===== Randomization Example (Uncomment to use) =====
# start_pos, goal_pos, robot_height = setup_environment(
#     world, 
#     stage,
#     # Fixed parameters (optional)
#     ground_color=np.array([0.4, 0.4, 0.4]),  # Darker gray
#     wall_color=np.array([0.8, 0.8, 0.8]),    # Lighter gray
#     map_size=12.0,                             # Larger map
#     robot_height=0.7,                          # Taller robot
#     # Enable randomization
#     randomize=True,
#     start_pos_range=((-4.5, 4.5), (-4.5, 4.5)),
#     goal_pos_range=((-4.5, 4.5), (-4.5, 4.5)),
#     box_pos_range=((-3.0, 3.0), (-3.0, 3.0)),
#     box_scale_range=((0.5, 2.0), (0.5, 2.0), (0.3, 1.0)),
#     box_mass_range=(3.0, 10.0),
# )

# Calculate robot orientation (yaw) to face from start to goal
direction_vector = goal_pos - start_pos
robot_yaw = np.arctan2(direction_vector[1], direction_vector[0])

# Convert yaw to quaternion (rotation around Z-axis)
# Quaternion = [w, x, y, z] for rotation around Z-axis
half_yaw = robot_yaw / 2.0
robot_orientation = np.array([
    np.cos(half_yaw),  # w
    0.0,                # x
    0.0,                # y
    np.sin(half_yaw)   # z
])

# Spot 로봇 추가 (시작점에 배치, 목표점을 향하도록 방향 설정)
spot = SpotFlatTerrainPolicy(
    prim_path="/World/Spot",
    name="Spot",
    position=np.array([start_pos[0], start_pos[1], robot_height]),
    orientation=robot_orientation,
)

print(f"[INFO] Robot placed at start position: [{start_pos[0]:.2f}, {start_pos[1]:.2f}, {robot_height:.2f}]")
print(f"[INFO] Robot yaw: {np.degrees(robot_yaw):.1f}° (facing goal at [{goal_pos[0]:.2f}, {goal_pos[1]:.2f}])")

# 키보드 컨트롤러 생성
controller = KeyboardController(
    max_vx=2.0,
    max_vy=2.0,
    max_yaw=2.0,
    acc_vx=5.0,
    acc_vy=5.0,
    acc_yaw=10.0,
    decay_vx=0.7,
    decay_vy=0.7,
    decay_yaw=0.6,
    update_dt=0.02,  # 50Hz
)

# Physics ready 플래그
physics_ready = False

# 명령 업데이트 카운터 (50Hz로 업데이트, physics는 500Hz)
command_update_counter = 0
COMMAND_UPDATE_RATE = 10  # 500Hz / 10 = 50Hz

# 로깅 카운터 (10Hz로 로깅, physics는 500Hz)
logging_counter = 0
LOGGING_UPDATE_RATE = 50  # 500Hz / 50 = 10Hz

# ===================== Physics Step Callback =====================
def on_physics_step(step_size):
    """Physics step마다 호출되는 콜백"""
    global physics_ready, command_update_counter, logging_counter
    
    # 명령 업데이트 (50Hz로 업데이트, physics는 500Hz)
    command_update_counter += 1
    if command_update_counter >= COMMAND_UPDATE_RATE:
        command_update_counter = 0
        controller.update()
    
    # 로깅 업데이트 (10Hz로 로깅, physics는 500Hz)
    logging_counter += 1
    if logging_counter >= LOGGING_UPDATE_RATE:
        logging_counter = 0
        
        if physics_ready:
            # Get robot position and orientation
            robot_pos, robot_quat = spot.robot.get_world_pose()
            robot_rpy = quaternion_to_rpy(robot_quat)
            
            # Get command velocity
            cmd_vel = controller.get_command()  # [vx, vy, yaw]
            
            # Calculate L1 distance (Manhattan distance) to goal
            # goal_pos is 2D [x, y], robot_pos is 3D [x, y, z]
            robot_pos_2d = robot_pos[:2]
            l1_distance = np.sum(np.abs(robot_pos_2d - goal_pos))
            
            # Log at DEBUG level: position, orientation (RPY), command velocity
            logger.debug(
                f"Robot pos: [{robot_pos[0]:.2f}, {robot_pos[1]:.2f}, {robot_pos[2]:.2f}], "
                f"RPY: [{np.degrees(robot_rpy[0]):.2f}, {np.degrees(robot_rpy[1]):.2f}, {np.degrees(robot_rpy[2]):.2f}]°, "
                f"Cmd vel: [vx={cmd_vel[0]:.2f}, vy={cmd_vel[1]:.2f}, yaw={cmd_vel[2]:.2f}]"
            )
            
            # Log at INFO level: L1 distance to goal
            logger.info(f"Robot <-> Goal L1 distance: {l1_distance:.2f} m")
    
    if physics_ready:
        spot.forward(step_size, controller.get_command())
    else:
        physics_ready = True
        spot.initialize()
        spot.post_reset()
        spot.robot.set_joints_default_state(spot.default_pos)
        print("[INFO] Spot initialized and ready")

# Resetting the world needs to be called before querying anything related to an articulation specifically.
# Its recommended to always do a reset after adding your assets, for physics handles to be propagated properly
world.reset()

# Physics callback 추가
world.add_physics_callback("physics_step", callback_fn=on_physics_step)

# ===================== Start Controller =====================
controller.start()

# ===================== Main Simulation Loop =====================
print("[INFO]: Starting simulation...")
print("[INFO]: Keyboard controls (pygame window):")
print("  - i: Move Forward (+x)")
print("  - k: Move Backward (-x)")
print("  - j: Strafe Left (+y)")
print("  - l: Strafe Right (-y)")
print("  - u: Turn Left (+yaw)")
print("  - o: Turn Right (-yaw)")
print("  - ESC: Quit")

# 시뮬레이션 실행 루프
while simulation_app.is_running():
    # 종료 요청 확인
    if controller.is_quit_requested():
        break
    
    # we have control over stepping physics and rendering in this workflow
    # things run in sync
    world.step(render=True)  # execute one physics step and one rendering step

# Cleanup
print("[INFO]: Cleaning up...")
controller.stop()

if world.physics_callback_exists("physics_step"):
    world.remove_physics_callback("physics_step")

# close Isaac Sim
simulation_app.close()
print("[INFO]: Simulation closed")
