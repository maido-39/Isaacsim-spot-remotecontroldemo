# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

"""
Performance-optimized Spot robot demo with keyboard control, sample box, and FPS logging
"""

# Launch Isaac Sim before any other imports
from isaacsim import SimulationApp

# Performance-optimized SimulationApp configuration
# Note: Some options may not be available in all Isaac Sim versions
simulation_app = SimulationApp({
    "headless": False,
    # Basic optimizations - adjust based on your Isaac Sim version
    # "sync_loads": False,  # Async loading (if available)
})

# SimulationApp 초기화 후에만 다른 모듈을 import할 수 있습니다.
import sys
import time
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
from contextlib import contextmanager
from collections import defaultdict


class SpotDemo:
    """
    Performance-optimized Spot robot demo with keyboard control, sample box, and FPS logging.
    
    Performance optimizations applied:
    - Reduced RTX rendering quality (reflections, translucency, indirect lighting disabled)
    - Lower shadow resolution (512x512)
    - Limited viewport resolution (1280x720)
    - Reduced ray-tracing bounces and samples
    - Optional frame skipping for rendering (configurable via render_every_n_frames)
    
    To further optimize, you can:
    - Set render_every_n_frames=2 to render every other frame (reduces visual quality)
    - Reduce viewport resolution in _optimize_rendering_settings()
    - Disable more RTX features if visual quality is not critical
    """

    def __init__(self):
        """Initialize the demo"""
        self.world = None
        self.stage = None
        self.spot = None
        self.controller = None
        self.physics_ready = False
        self.command_counter = 0
        
        # FPS tracking variables
        self.frame_times = []
        self.last_fps_print_time = time.time()
        self.fps_print_interval = 5.0  # Print FPS every 5 seconds
        self.frame_count = 0
        self.start_time = None
        
        # Method profiling variables
        self.method_times = defaultdict(list)  # Store execution times per method
        self.method_call_counts = defaultdict(int)  # Count calls per method
        self.profiling_enabled = True
        
        # Rendering optimization options
        self.render_every_n_frames = 1  # Render every N frames (1 = every frame, 2 = every other frame)
        self.render_frame_counter = 0
        self.enable_async_rendering = True  # Use async rendering if available
    
    def configure_performance(self, render_every_n_frames=1):
        """
        Configure performance settings
        
        Args:
            render_every_n_frames: Render every N frames (1=every frame, 2=every other frame, etc.)
                                  Higher values = better performance but lower visual quality
        """
        self.render_every_n_frames = max(1, int(render_every_n_frames))
        print(f"Performance configured: rendering every {self.render_every_n_frames} frame(s)")

    @contextmanager
    def _profile_method(self, method_name):
        """Context manager to profile method execution time"""
        if not self.profiling_enabled:
            yield
            return
        
        start_time = time.perf_counter()
        try:
            yield
        finally:
            end_time = time.perf_counter()
            execution_time = (end_time - start_time) * 1000  # Convert to milliseconds
            self.method_times[method_name].append(execution_time)
            self.method_call_counts[method_name] += 1

    def _optimize_rendering_settings(self):
        """Optimize rendering settings for better performance"""
        try:
            # Get rendering settings
            import omni.kit.app
            carb_settings = omni.kit.app.get_app().get_settings()
            
            optimizations_applied = []
            
            # Disable expensive rendering features
            try:
                carb_settings.set("/rtx/reflections/enabled", False)
                optimizations_applied.append("reflections disabled")
            except:
                pass
            
            try:
                carb_settings.set("/rtx/translucency/enabled", False)
                optimizations_applied.append("translucency disabled")
            except:
                pass
            
            try:
                carb_settings.set("/rtx/indirectDiffuse/enabled", False)
                optimizations_applied.append("indirect diffuse disabled")
            except:
                pass
            
            try:
                carb_settings.set("/rtx/indirectSpecular/enabled", False)
                optimizations_applied.append("indirect specular disabled")
            except:
                pass
            
            try:
                carb_settings.set("/rtx/raytracing/maxBounces", 1)
                optimizations_applied.append("max bounces reduced to 1")
            except:
                pass
            
            try:
                carb_settings.set("/rtx/raytracing/samplesPerPixel", 1)
                optimizations_applied.append("samples per pixel reduced to 1")
            except:
                pass
            
            # Reduce shadow quality
            try:
                carb_settings.set("/rtx/shadows/resolution", 512)
                optimizations_applied.append("shadow resolution reduced to 512")
            except:
                pass
            
            # Optimize viewport settings
            try:
                carb_settings.set("/app/viewport/resolution/width", 1280)
                carb_settings.set("/app/viewport/resolution/height", 720)
                optimizations_applied.append("viewport limited to 1280x720")
            except:
                pass
            
            if optimizations_applied:
                print(f"Rendering optimizations applied: {', '.join(optimizations_applied)}")
            else:
                print("Warning: Could not apply rendering optimizations (settings may not be available)")
        except Exception as e:
            print(f"Warning: Error applying rendering optimizations: {e}")

    def initialize(self):
        """Initialize Isaac Sim world and stage"""
        # Optimize rendering settings first
        self._optimize_rendering_settings()
        
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
        with self._profile_method("_on_physics_step"):
            # Command update: update controller at 50Hz (every 10 physics steps)
            self.command_counter += 1
            if self.command_counter >= 10:
                self.command_counter = 0
                with self._profile_method("controller.update"):
                    self.controller.update()  # Update controller state based on keyboard input

            # Robot control: apply commands to robot
            if self.physics_ready:
                # Robot is initialized, apply forward control with current command
                with self._profile_method("controller.get_command"):
                    command = self.controller.get_command()
                with self._profile_method("spot.forward"):
                    self.spot.forward(step_size, command)
            else:
                # First physics step: initialize robot
                self.physics_ready = True
                with self._profile_method("spot.initialize"):
                    self.spot.initialize()  # Initialize robot policy
                with self._profile_method("spot.post_reset"):
                    self.spot.post_reset()  # Post-reset setup
                with self._profile_method("spot.set_joints_default_state"):
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

    def _calculate_and_print_fps(self):
        """Calculate and print FPS statistics with method profiling"""
        with self._profile_method("_calculate_and_print_fps"):
            current_time = time.time()
            elapsed_time = current_time - self.last_fps_print_time
            
            if elapsed_time >= self.fps_print_interval:
                if len(self.frame_times) > 0:
                    # Calculate FPS statistics
                    avg_frame_time = np.mean(self.frame_times)
                    min_frame_time = np.min(self.frame_times)
                    max_frame_time = np.max(self.frame_times)
                    
                    avg_fps = 1.0 / avg_frame_time if avg_frame_time > 0 else 0.0
                    min_fps = 1.0 / max_frame_time if max_frame_time > 0 else 0.0
                    max_fps = 1.0 / min_frame_time if min_frame_time > 0 else 0.0
                    
                    # Calculate overall FPS from frame count
                    total_elapsed = current_time - self.start_time if self.start_time else elapsed_time
                    overall_fps = self.frame_count / total_elapsed if total_elapsed > 0 else 0.0
                    
                    print(f"\n{'='*80}")
                    print(f"[FPS] Avg: {avg_fps:.2f} | Min: {min_fps:.2f} | Max: {max_fps:.2f} | Overall: {overall_fps:.2f} | Frames: {self.frame_count}")
                    print(f"[Target] 50Hz = 20ms per frame | Current Avg: {avg_frame_time*1000:.2f}ms")
                    
                    # Performance warning
                    if avg_frame_time * 1000 > 20.0:
                        print(f"[WARNING] Frame time ({avg_frame_time*1000:.2f}ms) exceeds target (20ms)")
                        print(f"  Performance is {((avg_frame_time * 1000) / 20.0 - 1.0) * 100:.1f}% slower than target")
                        print(f"  Suggestions:")
                        print(f"    - Set render_every_n_frames=2 to render every other frame")
                        print(f"    - Reduce viewport resolution in _optimize_rendering_settings()")
                        print(f"    - Check if world.step() is the bottleneck (see method profiling below)")
                    
                    print(f"{'-'*80}")
                    
                    # Print method profiling statistics
                    if self.method_times:
                        print("[METHOD PROFILING] Execution overhead breakdown:")
                        print(f"{'Method':<40} {'Calls':<10} {'Avg(ms)':<12} {'Min(ms)':<12} {'Max(ms)':<12} {'Total(ms)':<12} {'% of Frame':<12}")
                        print(f"{'-'*110}")
                        
                        # Sort by total time (descending)
                        method_stats = []
                        for method_name, times in self.method_times.items():
                            if times:
                                avg_time = np.mean(times)
                                min_time = np.min(times)
                                max_time = np.max(times)
                                total_time = np.sum(times)
                                call_count = self.method_call_counts[method_name]
                                # Calculate percentage of average frame time
                                pct_of_frame = (avg_time / (avg_frame_time * 1000)) * 100 if avg_frame_time > 0 else 0.0
                                method_stats.append((
                                    method_name, call_count, avg_time, min_time, max_time, total_time, pct_of_frame
                                ))
                        
                        # Sort by total time descending
                        method_stats.sort(key=lambda x: x[5], reverse=True)
                        
                        for method_name, call_count, avg_time, min_time, max_time, total_time, pct_of_frame in method_stats:
                            print(f"{method_name:<40} {call_count:<10} {avg_time:<12.4f} {min_time:<12.4f} {max_time:<12.4f} {total_time:<12.4f} {pct_of_frame:<12.2f}%")
                        
                        print(f"{'='*80}\n")
                    
                    # Reset frame times and method profiling for next interval
                    self.frame_times.clear()
                    self.method_times.clear()
                    self.method_call_counts.clear()
                
                self.last_fps_print_time = current_time

    def run(self):
        """Run main simulation loop with FPS tracking"""
        if self.world is None or self.spot is None or self.controller is None:
            raise RuntimeError("Simulation must be setup first")

        print("Starting simulation...")
        print("Controls: i/k (x), j/l (y), u/o (yaw), ESC (quit)")
        print("FPS and method profiling will be logged every 5 seconds")
        print("Target: 50Hz (20ms per frame)")
        print(f"Rendering: Every {self.render_every_n_frames} frame(s)")
        if self.render_every_n_frames > 1:
            print(f"  NOTE: Rendering every {self.render_every_n_frames} frames to improve performance")

        # Initialize timing
        self.start_time = time.time()
        self.last_fps_print_time = self.start_time
        last_frame_time = self.start_time

        # Main simulation loop
        while simulation_app.is_running():
            # Check if quit requested from keyboard controller
            with self._profile_method("controller.is_quit_requested"):
                if self.controller.is_quit_requested():
                    break
            
            # Track frame time
            current_frame_time = time.time()
            frame_delta = current_frame_time - last_frame_time
            self.frame_times.append(frame_delta)
            self.frame_count += 1
            last_frame_time = current_frame_time
            
            # Determine if we should render this frame
            self.render_frame_counter += 1
            should_render = (self.render_frame_counter >= self.render_every_n_frames)
            if should_render:
                self.render_frame_counter = 0
            
            # Step physics and rendering (this is the main bottleneck)
            # Profile physics and rendering separately
            with self._profile_method("world.step"):
                if should_render:
                    with self._profile_method("world.step.render"):
                        self.world.step(render=True)
                else:
                    with self._profile_method("world.step.no_render"):
                        self.world.step(render=False)
            
            # Calculate and print FPS every 5 seconds
            self._calculate_and_print_fps()

    def cleanup(self):
        """Cleanup resources: stop controller and remove physics callback"""
        # Stop keyboard controller thread
        if self.controller:
            with self._profile_method("controller.stop"):
                self.controller.stop()

        # Remove physics callback
        if self.world and self.world.physics_callback_exists("physics_step"):
            self.world.remove_physics_callback("physics_step")

        # Print final FPS statistics
        if self.start_time:
            total_time = time.time() - self.start_time
            final_fps = self.frame_count / total_time if total_time > 0 else 0.0
            print(f"\n{'='*80}")
            print(f"[FPS] Final Statistics - Total Frames: {self.frame_count}, Total Time: {total_time:.2f}s, Average FPS: {final_fps:.2f}")
            
            # Print final method profiling summary
            if self.method_times:
                print(f"\n[FINAL METHOD PROFILING SUMMARY]")
                print(f"{'Method':<40} {'Total Calls':<15} {'Total Time(ms)':<20} {'Avg Time(ms)':<15}")
                print(f"{'-'*90}")
                
                method_summary = []
                for method_name, times in self.method_times.items():
                    if times:
                        total_calls = self.method_call_counts[method_name]
                        total_time = np.sum(times)
                        avg_time = np.mean(times)
                        method_summary.append((method_name, total_calls, total_time, avg_time))
                
                method_summary.sort(key=lambda x: x[2], reverse=True)
                
                for method_name, total_calls, total_time, avg_time in method_summary:
                    print(f"{method_name:<40} {total_calls:<15} {total_time:<20.4f} {avg_time:<15.4f}")
                
                print(f"{'='*80}")

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

