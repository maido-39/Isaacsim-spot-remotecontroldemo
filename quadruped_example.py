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


# ===================== Environment Setup =====================
def setup_environment(world: World, stage):
    """환경 설정: 회색 바닥, 벽, 박스, 마커, 조명"""
    
    # 0. 돔 라이트 추가 (intensity 600)
    omni.kit.commands.execute("CreatePrim", prim_path="/World/DomeLight", prim_type="DomeLight")
    light_prim = stage.GetPrimAtPath("/World/DomeLight")
    if light_prim.IsValid():
        light_prim.GetAttribute("inputs:intensity").Set(600.0)
    print("[INFO] Dome light added with intensity 600")
    
    # 1. 기본 ground plane + 회색 머티리얼
    world.scene.add_default_ground_plane(
        z_position=0,
        name="default_ground_plane",
        prim_path="/World/GroundPlane",
        static_friction=0.2,
        dynamic_friction=0.2,
        restitution=0.01,
    )
    
    # Ground plane에 회색 머티리얼 적용 (theGrid 머티리얼 오버라이드)
    from pxr import UsdShade
    
    ground_material = PreviewSurface(
        prim_path="/World/Materials/GroundMaterial",
        color=np.array([0.5, 0.5, 0.5])  # 회색
    )
    
    # Ground plane의 geometry prim에 머티리얼 바인딩
    ground_geom_path = "/World/GroundPlane/Environment/Geometry"
    ground_prim = stage.GetPrimAtPath(ground_geom_path)
    material_prim = stage.GetPrimAtPath(ground_material.prim_path)
    
    binding_api = UsdShade.MaterialBindingAPI(ground_prim)
    binding_api.Bind(UsdShade.Material(material_prim))
    print(f"[INFO] Ground plane material bound to: {ground_geom_path}")
    
    # 2. 10M x 10M 영역에 2m 높이 벽 만들기 (4개의 벽)
    wall_height = 2.0
    wall_thickness = 0.2
    map_size = 10.0  # 15.0에서 10.0으로 변경
    half_size = map_size / 2.0
    
    # 북쪽 벽 (+Y)
    world.scene.add(
        FixedCuboid(
            prim_path="/World/WallNorth",
            name="wall_north",
            position=np.array([0.0, half_size, wall_height / 2.0]),
            scale=np.array([map_size, wall_thickness, wall_height]),
            color=np.array([0.7, 0.7, 0.7])  # 밝은 회색
        )
    )
    
    # 남쪽 벽 (-Y)
    world.scene.add(
        FixedCuboid(
            prim_path="/World/WallSouth",
            name="wall_south",
            position=np.array([0.0, -half_size, wall_height / 2.0]),
            scale=np.array([map_size, wall_thickness, wall_height]),
            color=np.array([0.7, 0.7, 0.7])
        )
    )
    
    # 동쪽 벽 (+X)
    world.scene.add(
        FixedCuboid(
            prim_path="/World/WallEast",
            name="wall_east",
            position=np.array([half_size, 0.0, wall_height / 2.0]),
            scale=np.array([wall_thickness, map_size, wall_height]),
            color=np.array([0.7, 0.7, 0.7])
        )
    )
    
    # 서쪽 벽 (-X)
    world.scene.add(
        FixedCuboid(
            prim_path="/World/WallWest",
            name="wall_west",
            position=np.array([-half_size, 0.0, wall_height / 2.0]),
            scale=np.array([wall_thickness, map_size, wall_height]),
            color=np.array([0.7, 0.7, 0.7])
        )
    )
    
    # 3. 마찰이 있는 박스 (장애물로 사용 가능) - 평평하고 넓게
    obstacle_box = world.scene.add(
        DynamicCuboid(
            prim_path="/World/ObstacleBox",
            name="obstacle_box",
            position=np.array([2.0, 0.0, 0.25]),  # 위치 조정 (10m 맵 기준), z는 높이의 절반
            scale=np.array([1.0, 1.0, 0.5]),  # 100cm x 100cm x 50cm (평평하고 넓게)
            color=np.array([0.6, 0.4, 0.2]),  # 갈색 (박스 느낌)
            mass=5.0,
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
        physics_material.CreateStaticFrictionAttr().Set(0.8)
        physics_material.CreateDynamicFrictionAttr().Set(0.7)
        physics_material.CreateRestitutionAttr().Set(0.1)
        
        # Apply physics material to box
        collider = UsdPhysics.CollisionAPI.Get(stage, "/World/ObstacleBox")
        if collider:
            from pxr import Sdf
            collider.GetPrim().CreateRelationship(
                "physics:material"
            ).SetTargets([Sdf.Path(physics_material_path)])
    
    # 4. 반구형 마커 2개 (충돌체 없음, 반구만 보이도록)
    # 빨간색 시작점 [4, 4] (10m 맵 기준으로 조정)
    start_marker_path = "/World/StartMarker"
    start_sphere = UsdGeom.Sphere.Define(stage, start_marker_path)
    start_sphere.GetRadiusAttr().Set(0.3)
    start_sphere.CreateDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.0, 0.0)])  # 빨간색
    
    # 위치 설정 및 반구만 보이도록 (아래쪽 절반은 바닥 아래)
    start_xform = UsdGeom.Xformable(start_sphere)
    start_xform.ClearXformOpOrder()
    start_xform.AddTranslateOp().Set(Gf.Vec3d(4.0, 4.0, 0.0))
    
    # 충돌체 없음 (physics 설정 안 함)
    
    # 파란색 목표점 [-4, -4] (10m 맵 기준으로 조정)
    goal_marker_path = "/World/GoalMarker"
    goal_sphere = UsdGeom.Sphere.Define(stage, goal_marker_path)
    goal_sphere.GetRadiusAttr().Set(0.3)
    goal_sphere.CreateDisplayColorAttr().Set([Gf.Vec3f(0.0, 0.0, 1.0)])  # 파란색
    
    # 위치 설정
    goal_xform = UsdGeom.Xformable(goal_sphere)
    goal_xform.ClearXformOpOrder()
    goal_xform.AddTranslateOp().Set(Gf.Vec3d(-4.0, -4.0, 0.0))
    
    print("[INFO] Environment setup complete:")
    print("  - Dome light with intensity 600")
    print("  - Gray floor with friction")
    print("  - 10M x 10M walls (2m height)")
    print("  - Obstacle box (flat and wide) at [2, 0, 0.25]")
    print("  - Start marker (RED) at [4, 4]")
    print("  - Goal marker (BLUE) at [-4, -4]")


# ===================== Main Script =====================
# World 초기화
world = World(physics_dt=1.0 / 500.0, rendering_dt=10.0 / 500.0, stage_units_in_meters=1.0)

# USD Stage 가져오기
stage = omni.usd.get_context().get_stage()

# 환경 설정
setup_environment(world, stage)

# Spot 로봇 추가 (시작점 근처에 배치, 10m 맵 기준으로 조정)
spot = SpotFlatTerrainPolicy(
    prim_path="/World/Spot",
    name="Spot",
    position=np.array([3.5, 3.5, 0.8]),  # 시작점 [4, 4] 근처
)

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

# ===================== Physics Step Callback =====================
def on_physics_step(step_size):
    """Physics step마다 호출되는 콜백"""
    global physics_ready, command_update_counter
    
    # 명령 업데이트 (50Hz로 업데이트, physics는 500Hz)
    command_update_counter += 1
    if command_update_counter >= COMMAND_UPDATE_RATE:
        command_update_counter = 0
        controller.update()
    
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
