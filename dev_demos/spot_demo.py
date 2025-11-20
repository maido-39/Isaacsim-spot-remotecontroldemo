# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

"""
Simplified Spot robot demo with keyboard control and sample box
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
from pxr import Gf, UsdPhysics, Sdf
import omni
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy
from keyboard_controller import KeyboardController


class SpotDemo:
    """Simple Spot robot demo with keyboard control and sample box"""

    def __init__(self):
        """Initialize the demo"""
        self.world = None
        self.stage = None
        self.spot = None
        self.controller = None
        self.physics_ready = False
        self.command_counter = 0

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
        """Complete simulation setup: environment, robot, and controller"""
        # Initialize world if not already done
        if self.world is None:
            self.initialize()

        # Setup environment (ground and box)
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

        # Register physics callback for robot control
        self.world.add_physics_callback("physics_step", callback_fn=self._on_physics_step)

        # Start keyboard controller (runs in separate thread)
        self.controller.start()

        print("Setup complete")

    def run(self):
        """Run main simulation loop"""
        if self.world is None or self.spot is None or self.controller is None:
            raise RuntimeError("Simulation must be setup first")

        print("Starting simulation...")
        print("Controls: i/k (x), j/l (y), u/o (yaw), ESC (quit)")

        # Main simulation loop
        while simulation_app.is_running():
            # Check if quit requested from keyboard controller
            if self.controller.is_quit_requested():
                break
            # Step physics and rendering
            self.world.step(render=True)

    def cleanup(self):
        """Cleanup resources: stop controller and remove physics callback"""
        # Stop keyboard controller thread
        if self.controller:
            self.controller.stop()

        # Remove physics callback
        if self.world and self.world.physics_callback_exists("physics_step"):
            self.world.remove_physics_callback("physics_step")

        print("Cleanup complete")


# ===================== Main Entry Point =====================
def main():
    """Main entry point for the demo"""
    # Create demo instance
    demo = SpotDemo()

    # Setup simulation (environment, robot, controller)
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

