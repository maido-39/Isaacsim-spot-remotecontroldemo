# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# Isaac Sim standalone 스크립트
# 참고: https://docs.isaacsim.omniverse.nvidia.com/4.5.0/core_api_tutorials/tutorial_core_hello_world.html#converting-the-example-to-a-standalone-application
# BaseSample 대신 World를 직접 사용하여 standalone application으로 변환

# Launch Isaac Sim before any other imports
# default first two lines in any standalone application
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})  # we can also run as headless.

# SimulationApp 초기화 후에만 다른 모듈을 import할 수 있습니다.
import numpy as np
import carb
import omni
import omni.appwindow  # Contains handle to keyboard
from isaacsim.core.api import World
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy

# World 초기화
world = World(physics_dt=1.0 / 500.0, rendering_dt=10.0 / 500.0, stage_units_in_meters=1.0)

# Scene 설정
world.scene.add_default_ground_plane(
    z_position=0,
    name="default_ground_plane",
    prim_path="/World/defaultGroundPlane",
    static_friction=0.2,
    dynamic_friction=0.2,
    restitution=0.01,
)

# Spot 로봇 추가
spot = SpotFlatTerrainPolicy(
    prim_path="/World/Spot",
    name="Spot",
    position=np.array([0, 0, 0.8]),
)

# 키보드 입력 매핑
input_keyboard_mapping = {
    # forward command
    "NUMPAD_8": [2.0, 0.0, 0.0],
    "UP": [2.0, 0.0, 0.0],
    # back command
    "NUMPAD_2": [-2.0, 0.0, 0.0],
    "DOWN": [-2.0, 0.0, 0.0],
    # left command
    "NUMPAD_6": [0.0, -2.0, 0.0],
    "RIGHT": [0.0, -2.0, 0.0],
    # right command
    "NUMPAD_4": [0.0, 2.0, 0.0],
    "LEFT": [0.0, 2.0, 0.0],
    # yaw command (positive)
    "NUMPAD_7": [0.0, 0.0, 2.0],
    "N": [0.0, 0.0, 2.0],
    # yaw command (negative)
    "NUMPAD_9": [0.0, 0.0, -2.0],
    "M": [0.0, 0.0, -2.0],
}

# 기본 명령 초기화
base_command = np.array([0.0, 0.0, 0.0])
physics_ready = False

# 키보드 입력 처리 함수
def sub_keyboard_event(event, *args, **kwargs) -> bool:
    """키보드 이벤트 콜백"""
    global base_command
    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        # 키를 누르면 명령 증가
        if event.input.name in input_keyboard_mapping:
            base_command += np.array(input_keyboard_mapping[event.input.name])
    elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        # 키를 놓으면 명령 감소
        if event.input.name in input_keyboard_mapping:
            base_command -= np.array(input_keyboard_mapping[event.input.name])
    return True

# Physics step 콜백
def on_physics_step(step_size):
    """Physics step마다 호출되는 콜백"""
    global physics_ready
    if physics_ready:
        spot.forward(step_size, base_command)
    else:
        physics_ready = True
        spot.initialize()
        spot.post_reset()
        spot.robot.set_joints_default_state(spot.default_pos)

# Resetting the world needs to be called before querying anything related to an articulation specifically.
# Its recommended to always do a reset after adding your assets, for physics handles to be propagated properly
world.reset()

# 키보드 입력 구독 설정
appwindow = omni.appwindow.get_default_app_window()
input_interface = carb.input.acquire_input_interface()
keyboard = appwindow.get_keyboard()
sub_keyboard = input_interface.subscribe_to_keyboard_events(keyboard, sub_keyboard_event)

# Physics callback 추가
world.add_physics_callback("physics_step", callback_fn=on_physics_step)

# 시뮬레이션 루프
# standalone application에서는 world.step(render=True)를 직접 호출하여 제어합니다
print("[INFO]: Starting simulation...")
print("[INFO]: Keyboard controls:")
print("  - UP/NUMPAD_8: Move Forward")
print("  - DOWN/NUMPAD_2: Move Backward")
print("  - LEFT/NUMPAD_4: Move Left")
print("  - RIGHT/NUMPAD_6: Move Right")
print("  - N/NUMPAD_7: Spin Counterclockwise")
print("  - M/NUMPAD_9: Spin Clockwise")
print("[INFO]: Press ESC or close window to exit")

# 시뮬레이션 실행 루프
while simulation_app.is_running():
    # we have control over stepping physics and rendering in this workflow
    # things run in sync
    world.step(render=True)  # execute one physics step and one rendering step

# Cleanup
if world.physics_callback_exists("physics_step"):
    world.remove_physics_callback("physics_step")

# close Isaac Sim
simulation_app.close()
