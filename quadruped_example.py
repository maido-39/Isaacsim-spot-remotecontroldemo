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

simulation_app = SimulationApp({"headless": True})  # Headless mode for Pygame display

# SimulationApp 초기화 후에만 다른 모듈을 import할 수 있습니다.
import numpy as np
import logging
import threading
import queue
import time
import pygame
import sys
import json
import os
import csv
import argparse
from datetime import datetime
from pathlib import Path
from pxr import Gf, UsdGeom, UsdPhysics, UsdShade, Sdf
import omni
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
from isaacsim.core.api.materials import PreviewSurface
import omni.kit.commands
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy
from keyboard_controller import KeyboardController
import colorsys


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
    "object_type": "box",  # Type of object to spawn: "none", "box", "sphere", "gate"
    
    # Gate-specific parameters
    "wall_depth_min": 0.3,  # Minimum wall depth (meters)
    "wall_depth_max": 1.0,  # Maximum wall depth (meters)
    "gate_y_offset_min": -2.0,  # Minimum Y offset (meters)
    "gate_y_offset_max": 2.0,  # Maximum Y offset (meters)
    "gap_min": 1.0,  # Minimum gap between gates (meters)
    "gap_max": 1.5,  # Maximum gap between gates (meters)
    "gate_location_min": 0.2,  # Minimum location along start-goal line (0.0-1.0)
    "gate_location_max": 0.8,  # Maximum location along start-goal line (0.0-1.0)
    "gate_color": [0.5, 0.3, 0.1],  # Gate wall color [r, g, b]
    
    # Randomizable parameters (will be set by randomization function)
    "start_position": [4.0, 4.0],
    "goal_position": [-4.0, -4.0],
    "box_position": [2.0, 0.0, 0.25],
    "box_scale": [1.0, 1.0, 0.5],
    "box_mass": 5.0,
    "box_color": [0.6, 0.4, 0.2],
    "robot_height": 0.8,
    
    # Randomization settings
    "randomize": True,  # Enable randomization by default
    "random_seed": None,  # Random seed for reproducibility (None = use random seed, int = fixed seed)
    "wall_inset": 1.0,  # Inset from walls for spawning (meters)
    "box_line_distance_min": 2.0,  # Minimum distance from start-goal line (meters)
    "box_line_distance_max": 3.0,  # Maximum distance from start-goal line (meters) 
    "box_scale_range": [[0.8, 2.0], [0.8, 2.0], [0.5, 1.0]],
    "box_mass_range": [3.0, 10.0],
    "num_boxes": 2,  # Number of boxes to spawn
    "box_color_range": [[0.0, 1.0], [0.0, 1.0], [0.0, 1.0]],  # RGB color ranges [r_min, r_max], [g_min, g_max], [b_min, b_max]
    "box_min_separation": 1.5,  # Minimum separation distance between boxes (meters)
    
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
    
    # Camera settings (RealSense D455 RGB specs)
    "ego_camera_resolution": [640, 480],  # Ego-view camera resolution [width, height] - 16:10
    "top_camera_resolution": [800, 800],  # Top-down camera resolution [width, height]
    "top_camera_height": 10.0,  # Top camera z position (height above origin in meters)
}


# ===================== Pygame Display for Dual Camera =====================
class PygameDualCameraDisplay:
    """
    High-performance Pygame display for dual camera streaming (ego + top view).
    Uses separate thread to avoid blocking main simulation loop.
    Also handles keyboard input for robot control.
    """
    
    def __init__(self, window_size=(1600, 800), window_title="Spot Robot - Ego & Top View", key_state_callback=None, 
                 ego_camera_resolution=(1280, 800), gate_transform_callback=None):
        """
        Initialize dual camera display.
        
        Args:
            window_size: Total window size (width, height) - will be split in half
            window_title: Window title
            key_state_callback: Callback function(key, pressed) to update key state
            ego_camera_resolution: Original ego camera resolution (width, height) for aspect ratio calculation
            gate_transform_callback: Callback function() to apply gate transform (called on 'g' key press)
        """
        self.window_size = window_size
        self.window_title = window_title
        self.ego_frame_queue = queue.Queue(maxsize=2)  # Keep only latest 2 frames
        self.top_frame_queue = queue.Queue(maxsize=2)  # Keep only latest 2 frames
        self.gate_transform_queue = queue.Queue(maxsize=1)  # Queue for gate transform requests
        self.running = False
        self.display_thread = None
        self.screen = None
        self.clock = None
        self.key_state_callback = key_state_callback  # Callback to update key state
        self.gate_transform_callback = gate_transform_callback  # Callback for gate transform
        self.ego_camera_resolution = ego_camera_resolution  # Store original ego camera resolution
        self.has_gate = gate_transform_callback is not None  # Track if gate exists
        
        # Calculate individual camera display sizes (side by side)
        self.camera_width = window_size[0] // 2
        self.camera_height = window_size[1]
        
        # Note: We no longer need _update_ego_display_size() since we scale to fill
        # Both cameras will fill their allocated space (camera_width x camera_height)
        
    def start(self):
        """Start display thread"""
        self.running = True
        self.display_thread = threading.Thread(target=self._display_loop, daemon=True)
        self.display_thread.start()
        time.sleep(0.1)  # Give thread time to initialize
        print(f"✓ Pygame dual camera display started: {self.window_size[0]}×{self.window_size[1]}")
    
    def stop(self):
        """Stop display thread"""
        self.running = False
        if self.display_thread:
            self.display_thread.join(timeout=1.0)
        if self.screen:
            pygame.quit()
        print("Pygame display stopped")
    
    def update_ego_frame(self, image_array):
        """
        Update ego camera frame (non-blocking, drops old frames if queue full).
        Scales to fill available space while maintaining aspect ratio.
        
        Args:
            image_array: numpy array (H, W, 3) in RGB format
        """
        if not self.running:
            return
        # Scale to fill the entire camera area (half window width, full height)
        self._update_frame_queue(
            self.ego_frame_queue, 
            image_array, 
            (self.camera_width, self.camera_height),
            maintain_aspect=True,
            scale_to_fill=True
        )
    
    def update_top_frame(self, image_array):
        """
        Update top camera frame (non-blocking, drops old frames if queue full).
        Scales to fill available space while maintaining aspect ratio.
        
        Args:
            image_array: numpy array (H, W, 3) in RGB format
        """
        if not self.running:
            return
        # Scale to fill the entire camera area (half window width, full height)
        self._update_frame_queue(
            self.top_frame_queue, 
            image_array, 
            (self.camera_width, self.camera_height),
            maintain_aspect=True,
            scale_to_fill=True
        )
    
    def _update_frame_queue(self, frame_queue, image_array, target_size, maintain_aspect=False, scale_to_fill=True):
        """
        Helper method to update a frame queue
        
        Args:
            frame_queue: Queue to put the surface in
            image_array: numpy array (H, W, 3) in RGB format
            target_size: Target size (width, height) for the surface
            maintain_aspect: If True, resize maintaining aspect ratio (fits within target_size)
            scale_to_fill: If True, scale image to fill target_size (may crop if maintain_aspect=True)
        """
        try:
            # Ensure image is in correct format: (H, W, 3) RGB uint8
            if image_array.dtype != np.uint8:
                image_array = (image_array * 255).astype(np.uint8) if image_array.max() <= 1.0 else image_array.astype(np.uint8)
            
            # Convert to pygame surface
            # Pygame surfarray.make_surface expects array in (W, H, 3) format
            # Swap axes: (H, W, 3) -> (W, H, 3)
            image_swapped = np.swapaxes(image_array, 0, 1)
            surface = pygame.surfarray.make_surface(image_swapped)
            
            # Resize if needed
            if surface.get_size() != target_size:
                if maintain_aspect and scale_to_fill:
                    # Scale to fill target_size while maintaining aspect ratio (may crop)
                    # Calculate scale factors
                    img_w, img_h = surface.get_size()
                    target_w, target_h = target_size
                    
                    scale_w = target_w / img_w
                    scale_h = target_h / img_h
                    scale = max(scale_w, scale_h)  # Use larger scale to fill
                    
                    # Scale image
                    scaled_w = int(img_w * scale)
                    scaled_h = int(img_h * scale)
                    surface = pygame.transform.smoothscale(surface, (scaled_w, scaled_h))
                    
                    # Crop to target size (center crop)
                    if scaled_w != target_w or scaled_h != target_h:
                        crop_x = (scaled_w - target_w) // 2
                        crop_y = (scaled_h - target_h) // 2
                        surface = surface.subsurface((crop_x, crop_y, target_w, target_h))
                elif maintain_aspect:
                    # Resize maintaining aspect ratio (fits within target_size, may have black bars)
                    surface = pygame.transform.smoothscale(surface, target_size)
                else:
                    # Resize to exact target size (may distort)
                    surface = pygame.transform.scale(surface, target_size)
            
            # Put in queue (non-blocking, drop if full)
            try:
                frame_queue.put_nowait(surface)
            except queue.Full:
                # Drop oldest frame and add new one
                try:
                    frame_queue.get_nowait()
                    frame_queue.put_nowait(surface)
                except queue.Empty:
                    pass
        except Exception as e:
            # Silently ignore conversion errors
            pass
    
    def _display_loop(self):
        """Display loop running in separate thread"""
        try:
            pygame.init()
            # Enable double buffering and hardware acceleration to prevent flickering
            flags = pygame.RESIZABLE | pygame.DOUBLEBUF | pygame.HWSURFACE
            self.screen = pygame.display.set_mode(self.window_size, flags)
            pygame.display.set_caption(self.window_title)
            self.clock = pygame.time.Clock()
            
            # Target FPS for display (60 FPS for smooth display)
            target_fps = 60
            
            # Font for labels (render once, reuse)
            font = pygame.font.Font(None, 36)
            small_font = pygame.font.Font(None, 24)
            ego_label = font.render("Ego View", True, (255, 255, 255))
            top_label = font.render("Top View", True, (255, 255, 255))
            
            # Create back buffer surface for double buffering
            back_buffer = pygame.Surface(self.window_size)
            
            # Keep track of last surfaces to avoid unnecessary redraws
            last_ego_surface = None
            last_top_surface = None
            
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
                        # Recalculate camera display sizes
                        self.camera_width = self.window_size[0] // 2
                        self.camera_height = self.window_size[1]
                        flags = pygame.RESIZABLE | pygame.DOUBLEBUF | pygame.HWSURFACE
                        self.screen = pygame.display.set_mode(self.window_size, flags)
                        back_buffer = pygame.Surface(self.window_size)
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
                            elif event.key == pygame.K_g and self.gate_transform_callback:
                                # Queue gate transform request (will be processed in main thread)
                                try:
                                    self.gate_transform_queue.put_nowait(True)
                                except queue.Full:
                                    pass  # Request already queued
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
                
                # Get latest frames from queues (non-blocking, drain queue to get latest)
                ego_surface = None
                top_surface = None
                
                # Drain queue to get the latest frame (drop old frames)
                try:
                    while True:
                        ego_surface = self.ego_frame_queue.get_nowait()
                except queue.Empty:
                    pass
                
                try:
                    while True:
                        top_surface = self.top_frame_queue.get_nowait()
                except queue.Empty:
                    pass
                
                # Use last surface if no new frame available
                if ego_surface is None:
                    ego_surface = last_ego_surface
                else:
                    last_ego_surface = ego_surface
                
                if top_surface is None:
                    top_surface = last_top_surface
                else:
                    last_top_surface = top_surface
                
                # Draw to back buffer first (double buffering)
                back_buffer.fill((0, 0, 0))  # Black background
                
                # Display ego camera (left side) - scaled to fill the area
                if ego_surface:
                    # Surface is already scaled to fill (self.camera_width, self.camera_height)
                    back_buffer.blit(ego_surface, (0, 0))
                else:
                    # Placeholder if no frame
                    pygame.draw.rect(back_buffer, (50, 50, 50), (0, 0, self.camera_width, self.camera_height))
                
                # Display top camera (right side) - scaled to fill the area
                if top_surface:
                    # Surface is already scaled to fill (self.camera_width, self.camera_height)
                    back_buffer.blit(top_surface, (self.camera_width, 0))
                else:
                    # Placeholder if no frame
                    pygame.draw.rect(back_buffer, (50, 50, 50), (self.camera_width, 0, self.camera_width, self.camera_height))
                
                # Draw labels
                back_buffer.blit(ego_label, (10, 10))
                back_buffer.blit(top_label, (self.camera_width + 10, 10))
                
                # Draw gate transform button label (if gate exists)
                if self.has_gate:
                    # Render label on the fly (lightweight operation)
                    gate_button_label = small_font.render("Press 'G' to Apply Gate Transform", True, (255, 200, 0))
                    # Position at bottom center of window
                    label_x = (self.window_size[0] - gate_button_label.get_width()) // 2
                    label_y = self.window_size[1] - 35
                    # Draw semi-transparent background for better visibility
                    bg_rect = pygame.Rect(label_x - 10, label_y - 5, 
                                        gate_button_label.get_width() + 20, 
                                        gate_button_label.get_height() + 10)
                    # Create semi-transparent surface for background
                    bg_surface = pygame.Surface((bg_rect.width, bg_rect.height), pygame.SRCALPHA)
                    bg_surface.fill((0, 0, 0, 180))  # Black with alpha
                    back_buffer.blit(bg_surface, (bg_rect.x, bg_rect.y))
                    back_buffer.blit(gate_button_label, (label_x, label_y))
                
                # Draw divider line
                pygame.draw.line(back_buffer, (100, 100, 100), 
                               (self.camera_width, 0), 
                               (self.camera_width, self.camera_height), 2)
                
                # Blit back buffer to screen (single operation, no flicker)
                self.screen.blit(back_buffer, (0, 0))
                pygame.display.flip()
                
                # Limit to target FPS
                self.clock.tick(target_fps)
                
        except Exception as e:
            print(f"Pygame display error: {e}")
        finally:
            if self.screen:
                pygame.quit()


# ===================== Main Simulation Class =====================
class SpotSimulation:
    """Main simulation class for Isaac Sim Spot robot control"""

    def __init__(self, config_file=None, experiment_name=None, log_level=logging.INFO, 
                 enable_csv_logging=True, enable_image_saving=True, **config_overrides):
        """
        Initialize simulation.
    
    Args:
            config_file: Path to JSON config file (optional)
            experiment_name: Name of the experiment (optional, defaults to "NULL")
            log_level: Logging level (default: logging.INFO)
            enable_csv_logging: Enable CSV data logging (default: True)
            enable_image_saving: Enable camera image saving (default: True)
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
        self._setup_logging(log_level)
        
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
        self._gate_button_window = None # GUI window for gate transform control
        self.task_in_progress = True    # Flag to track if task is in progress
        self.display = None             # Pygame dual camera display
        
        # Performance tracking variables
        self.performance_counter = 0    # Counter for performance logging (1Hz)
        self.physics_step_times = []   # List to track physics step durations (ms)
        self.render_times = []         # List to track render step durations (ms)
        self.frame_times = []          # List to track overall frame times (ms)
        self.last_physics_time = None   # Timestamp of last physics step
        self.last_render_time = None    # Timestamp of last render step
        self.last_frame_time = None     # Timestamp of last frame
        self.performance_log_interval = 500  # Log performance every 500 physics steps (~1 second at 500Hz)
        
        # Experiment data tracking
        self.experiment_name = experiment_name if experiment_name else "NULL"
        self.experiment_dir = None      # Path to experiment directory
        self.csv_file = None            # CSV file handle
        self.csv_writer = None          # CSV writer object
        self.frame_counter = 0          # Frame counter for data logging
        self.experiment_start_time = None  # Experiment start timestamp
        self.data_saving_started = False   # Flag to track if data saving has started
        self.first_command_received = False  # Flag to track first keyboard command
        
        # Performance flags (can be set via command-line args)
        self.enable_csv_logging = enable_csv_logging   # Enable/disable CSV logging
        self.enable_image_saving = enable_image_saving  # Enable/disable image saving

    def _setup_logging(self, log_level=logging.INFO):
        """
        Setup logging system with console output.
        Configures logger to output messages with timestamps.
        
        Args:
            log_level: Logging level (default: logging.INFO)
        """
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.setLevel(log_level)
        
        # Clear existing handlers to avoid duplicates
        if self.logger.handlers:
            self.logger.handlers.clear()
        
        # Create console handler that outputs to stdout
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setFormatter(logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        ))
        self.logger.addHandler(console_handler)
        self.logger.propagate = False  # Prevent messages from propagating to root logger
    
    def _add_file_logging(self, log_file_path):
        """
        Add file handler to save logs to terminal.log in experiment folder.
        Called after experiment directory is created.
        
        Args:
            log_file_path: Path to the log file
        """
        try:
            # Create file handler
            file_handler = logging.FileHandler(log_file_path, mode='w', encoding='utf-8')
            file_handler.setFormatter(logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                datefmt='%Y-%m-%d %H:%M:%S'
            ))
            self.logger.addHandler(file_handler)
            self.logger.info(f"✓ Terminal logging to file: {log_file_path}")
        except Exception as e:
            self.logger.warning(f"Failed to add file logging: {e}")

    # ===================== Experiment Data Management =====================
    def _initialize_experiment_directory(self):
        """
        Create experiment directory structure and initialize CSV file.
        Directory structure:
            expr_data/
                YYMMDD_HHMMSS-Experiment_Name/
                    config.json
                    data.csv
                    terminal.log
                    camera/
                        ego/
                        top/
        """
        # Create timestamp string in YYMMDD_HHMMSS format
        now = datetime.now()
        timestamp = now.strftime("%y%m%d_%H%M%S")
        
        # Create experiment directory name
        dir_name = f"{timestamp}-{self.experiment_name}"
        
        # Create expr_data directory if it doesn't exist
        expr_data_dir = Path("expr_data")
        expr_data_dir.mkdir(exist_ok=True)
        
        # Create experiment directory inside expr_data
        self.experiment_dir = expr_data_dir / dir_name
        # Note: experiment_start_time will be set when first keyboard command is received
        
        # Create directory structure
        try:
            self.experiment_dir.mkdir(exist_ok=True)
            # Only create camera directories if image saving is enabled
            if self.enable_image_saving:
                (self.experiment_dir / "camera" / "ego").mkdir(parents=True, exist_ok=True)
                (self.experiment_dir / "camera" / "top").mkdir(parents=True, exist_ok=True)
            
            self.logger.info(f"Experiment directory created: {self.experiment_dir}")
            
            # Add file logging to save terminal output to terminal.log
            terminal_log_path = self.experiment_dir / "terminal.log"
            self._add_file_logging(terminal_log_path)
            
            # Save configuration file (actual values used, not ranges)
            self._save_config()
            
            # Initialize CSV file (only if CSV logging is enabled)
            if self.enable_csv_logging:
                csv_path = self.experiment_dir / "data.csv"
                self.csv_file = open(csv_path, 'w', newline='')
                self.csv_writer = csv.writer(self.csv_file)
                
                # Write CSV header - one row per box with obj_index
                # Structure: timestamp, frame_num, obj_index, robot data, box data
                header = [
                    'timestamp', 'frame_num', 'obj_index',
                    'robot_pos_x', 'robot_pos_y', 'robot_pos_z',
                    'robot_orient_w', 'robot_orient_x', 'robot_orient_y', 'robot_orient_z',
                    'box_pos_x', 'box_pos_y', 'box_pos_z',
                    'box_orient_w', 'box_orient_x', 'box_orient_y', 'box_orient_z',
                    'box_l1_distance'
                ]
                
                self.csv_writer.writerow(header)
                self.csv_file.flush()
                
                self.logger.info(f"CSV file initialized: {csv_path}")
            else:
                self.logger.info("CSV logging disabled")
            
        except Exception as e:
            self.logger.error(f"Failed to create experiment directory: {e}")
            raise
    
    def _save_config(self):
        """
        Save current configuration to config.json.
        Saves the actual values used in the experiment (no random ranges).
        Ensures boxes_config is saved as a list format for compatibility.
        """
        if self.experiment_dir is None:
            return
        
        try:
            # Create a clean config dictionary with only the actual values used
            # Remove range parameters that are only for randomization
            clean_config = self.config.copy()
            
            # Remove randomization-specific keys
            keys_to_remove = [
                'box_scale_range', 'box_mass_range',
                'box_line_distance_min', 'box_line_distance_max',
                'wall_inset', 'box_color_range'  # Remove color range, keep actual colors
            ]
            for key in keys_to_remove:
                clean_config.pop(key, None)
            
            # Ensure boxes_config is in list format (even for single box)
            if 'boxes_config' not in clean_config or not isinstance(clean_config.get('boxes_config'), list):
                # Create boxes_config from single box parameters if it doesn't exist
                if clean_config.get('object_type') in ['box', 'sphere']:
                    boxes_config = [{
                        'position': clean_config.get('box_position', [0.0, 0.0, 0.25]),
                        'scale': clean_config.get('box_scale', [1.0, 1.0, 0.5]),
                        'color': clean_config.get('box_color', [0.6, 0.4, 0.2]),
                        'mass': clean_config.get('box_mass', 5.0)
                    }]
                    clean_config['boxes_config'] = boxes_config
                    clean_config['num_boxes'] = 1
            
            # Ensure num_boxes is set
            if 'num_boxes' not in clean_config:
                clean_config['num_boxes'] = len(clean_config.get('boxes_config', []))
            
            config_path = self.experiment_dir / "config.json"
            with open(config_path, 'w') as f:
                json.dump(clean_config, f, indent=2)
            
            self.logger.info(f"Configuration saved: {config_path}")
            
        except Exception as e:
            self.logger.error(f"Failed to save configuration: {e}")
    
    def _initialize_camera_render_products(self):
        """
        Initialize render products and annotators for cameras once during setup.
        This ensures consistency between viewport and captured images.
        Called after camera setup and world reset.
        """
        try:
            import omni.replicator.core as rep
            
            # Initialize render products dictionary and annotators
            self.render_products = {}
            self.rgb_annotators = {}
            
            # Setup ego camera render product
            if hasattr(self, 'robot_camera_path') and self.robot_camera_path:
                ego_res = tuple(self.config["ego_camera_resolution"])
                self.render_products["ego"] = rep.create.render_product(
                    self.robot_camera_path, 
                    ego_res
                )
                self.rgb_annotators["ego"] = rep.AnnotatorRegistry.get_annotator("rgb")
                self.rgb_annotators["ego"].attach([self.render_products["ego"]])
                self.logger.info(f"✓ Ego camera render product initialized: {ego_res[0]}×{ego_res[1]}")
            
            # Setup top camera render product
            if hasattr(self, 'camera_path') and self.camera_path:
                top_res = tuple(self.config["top_camera_resolution"])
                self.render_products["top"] = rep.create.render_product(
                    self.camera_path, 
                    top_res
                )
                self.rgb_annotators["top"] = rep.AnnotatorRegistry.get_annotator("rgb")
                self.rgb_annotators["top"].attach([self.render_products["top"]])
                self.logger.info(f"✓ Top camera render product initialized: {top_res[0]}×{top_res[1]}")
            
            self.camera_render_initialized = True
            
        except ImportError as e:
            self.logger.warning(f"Replicator not available, camera capture disabled: {e}")
            self.camera_render_initialized = False
        except Exception as e:
            self.logger.warning(f"Failed to initialize camera render products: {e}")
            self.camera_render_initialized = False
    
    def _save_camera_image(self, camera_path, camera_type, frame_num, timestamp_str):
        """
        Capture and save image directly from pre-initialized render products.
        No image modification - saves raw data from annotator.
        
        Args:
            camera_path: USD path to the camera (not used, kept for compatibility)
            camera_type: "ego" or "top" (determines subfolder)
            frame_num: Frame number
            timestamp_str: Timestamp string for filename
        """
        # Check if render products are initialized
        if not hasattr(self, 'camera_render_initialized') or not self.camera_render_initialized:
            return
        
        # Check if this camera type has a render product
        if camera_type not in self.render_products or camera_type not in self.rgb_annotators:
            return
        
        try:
            from PIL import Image
            import numpy as np
            
            # Create filename and path
            filename = f"frame{frame_num}-{timestamp_str}-{camera_type}.jpg"
            image_path = self.experiment_dir / "camera" / camera_type / filename
            
            # Get data directly from annotator
            rgb_data = self.rgb_annotators[camera_type].get_data()
            
            if rgb_data is None:
                self.logger.debug(f"No data from {camera_type} annotator")
                return
            
            # Convert to numpy array
            image_array = np.asarray(rgb_data)
            
            # Check if we have valid image data
            if image_array is None or image_array.size == 0:
                self.logger.debug(f"Empty image data for {camera_type}")
                return
            
            # Handle RGBA to RGB conversion if needed
            if len(image_array.shape) == 3 and image_array.shape[2] == 4:
                # Convert RGBA to RGB
                image_array = image_array[:, :, :3]
            
            # Save image
            Image.fromarray(image_array, mode='RGB').save(str(image_path), quality=95)
            self.logger.debug(f"✓ Image saved: {filename}")
            
        except Exception as e:
            self.logger.warning(f"Failed to save {camera_type} camera image: {e}")
    
    def _save_camera_image_fallback(self, camera_path, camera_type, frame_num, timestamp_str):
        """
        Fallback method to capture camera image using viewport screenshot.
        
        Args:
            camera_path: USD path to the camera
            camera_type: "ego" or "top" (determines subfolder)
            frame_num: Frame number
            timestamp_str: Timestamp string for filename
        """
        try:
            import omni.kit.viewport.utility as vp_utils
            
            filename = f"frame{frame_num}-{timestamp_str}-{camera_type}.jpg"
            image_path = self.experiment_dir / "camera" / camera_type / filename
            
            # Get viewport
            viewport = vp_utils.get_active_viewport()
            if viewport:
                # Try to temporarily set camera and capture
                original_camera = viewport.get_active_camera()
                viewport.set_active_camera(camera_path)
                
                # Wait a frame for the camera to update
                # Then capture (this is a simplified approach)
                # The actual implementation may need more sophisticated frame synchronization
                
                # Restore original camera
                viewport.set_active_camera(original_camera)
                
                self.logger.debug(f"Fallback image capture attempted: {image_path}")
            
        except Exception as e:
            self.logger.debug(f"Fallback image capture also failed: {e}")
    
    def _save_experiment_data(self, robot_pos, robot_quat, boxes_data, object_pos=None, object_quat=None, l1_distance=None):
        """
        Save experiment data to CSV and capture camera images.
        
        Args:
            robot_pos: Robot position [x, y, z]
            robot_quat: Robot orientation quaternion [w, x, y, z]
            boxes_data: List of box data dicts, each with 'pos', 'quat', 'l1_distance'
                       Always use list format (even for single box) for compatibility
            object_pos: Object position [x, y, z] (for backward compatibility, same as boxes_data[0] if available)
            object_quat: Object orientation quaternion [w, x, y, z] (for backward compatibility)
            l1_distance: L1 distance to goal (for backward compatibility, same as boxes_data[0]['l1_distance'] if available)
        """
        # Skip if CSV logging is disabled
        if not self.enable_csv_logging or self.csv_writer is None:
            # Still increment frame counter for consistency
            if not self.enable_csv_logging:
                self.frame_counter += 1
            return
        
        try:
            # Get current timestamp
            elapsed = (datetime.now() - self.experiment_start_time).total_seconds()
            timestamp_str = f"{elapsed:.3f}"
            
            # Ensure boxes_data is a list (for compatibility)
            if not isinstance(boxes_data, list):
                boxes_data = [boxes_data]
            
            # Write CSV rows (only if CSV logging is enabled)
            # Write one row per box with obj_index
            if self.enable_csv_logging:
                # Ensure boxes_data is not empty
                if len(boxes_data) == 0:
                    # If no boxes, write a single row with obj_index=-1 and default values
                    row = [
                        timestamp_str, self.frame_counter, -1,
                        float(robot_pos[0]), float(robot_pos[1]), float(robot_pos[2]),
                        float(robot_quat[0]), float(robot_quat[1]), float(robot_quat[2]), float(robot_quat[3]),
                        0.0, 0.0, 0.0,  # box_pos (default)
                        1.0, 0.0, 0.0, 0.0,  # box_orient (default quaternion)
                        0.0  # box_l1_distance (default)
                    ]
                    self.csv_writer.writerow(row)
                else:
                    # Write one row per box
                    for obj_index, box_data in enumerate(boxes_data):
                        row = [
                            timestamp_str, self.frame_counter, obj_index,
                            float(robot_pos[0]), float(robot_pos[1]), float(robot_pos[2]),
                            float(robot_quat[0]), float(robot_quat[1]), float(robot_quat[2]), float(robot_quat[3]),
                            float(box_data['pos'][0]), float(box_data['pos'][1]), float(box_data['pos'][2]),
                            float(box_data['quat'][0]), float(box_data['quat'][1]), float(box_data['quat'][2]), float(box_data['quat'][3]),
                            float(box_data['l1_distance'])
                        ]
                        self.csv_writer.writerow(row)
                
                # Flush every 10 frames to ensure data is written
                if self.frame_counter % 10 == 0:
                    self.csv_file.flush()
            
            # Save camera images (only if image saving is enabled)
            if self.enable_image_saving:
                if self.robot_camera_path:
                    self._save_camera_image(self.robot_camera_path, "ego", self.frame_counter, timestamp_str)
                if hasattr(self, 'camera_path') and self.camera_path:
                    self._save_camera_image(self.camera_path, "top", self.frame_counter, timestamp_str)
            
            self.frame_counter += 1
            
        except Exception as e:
            self.logger.error(f"Failed to save experiment data: {e}")

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

    def _colors_are_similar(self, color1, color2, hue_threshold=15.0):
        """
        Check if two colors are too similar based on their HSL hue values.
        
        Args:
            color1: RGB color [r, g, b] (0-1 range)
            color2: RGB color [r, g, b] (0-1 range)
            hue_threshold: Minimum hue difference in degrees to consider colors different (default: 15°)
        
        Returns:
            bool: True if colors are too similar, False otherwise
        """
        # Convert RGB to HSL to compare hues
        h1, s1, l1 = colorsys.rgb_to_hls(color1[0], color1[1], color1[2])
        h2, s2, l2 = colorsys.rgb_to_hls(color2[0], color2[1], color2[2])
        
        # Convert hue to degrees (0-360)
        h1_deg = (h1 * 360) % 360
        h2_deg = (h2 * 360) % 360
        
        # Calculate hue difference (accounting for wrap-around)
        hue_diff = abs(h1_deg - h2_deg)
        if hue_diff > 180:
            hue_diff = 360 - hue_diff
        
        # Colors are similar if hue difference is less than threshold
        return hue_diff < hue_threshold
    
    def _generate_random_hsl_color(self, rng, exclude_primary=True, exclusion_range=30.0, existing_colors=None, min_hue_separation=15.0):
        """
        Generate a random color in HSL space with random hue, saturation=100%, lightness=50%.
        Optionally excludes colors near Red (0°), Green (120°), and Blue (240°).
        Ensures the generated color is different from existing colors.
        
        Args:
            rng: Random number generator (numpy RandomState or Generator)
            exclude_primary: If True, exclude colors near Red, Green, Blue
            exclusion_range: Degrees to exclude around each primary color (default: 30°)
            existing_colors: List of existing RGB colors to avoid similarity with
            min_hue_separation: Minimum hue difference in degrees from existing colors (default: 15°)
        
        Returns:
            RGB tuple (0-1 range) as list [r, g, b]
        """
        
        if exclude_primary:
            # Define excluded hue ranges (in degrees)
            # Red: 0° (and 360°), Green: 120°, Blue: 240°
            # Red wraps around: exclude (360-exclusion_range) to exclusion_range
            red_start = 360 - exclusion_range
            red_end = exclusion_range
            green_start = 120 - exclusion_range
            green_end = 120 + exclusion_range
            blue_start = 240 - exclusion_range
            blue_end = 240 + exclusion_range
            
            max_attempts = 500  # Increased attempts to find unique color
            hue = None
            
            for attempt in range(max_attempts):
                candidate_hue = rng.uniform(0, 360)
                
                # Check if hue is in any excluded range
                is_excluded = False
                
                # Check Red (wraps around 0/360)
                if candidate_hue >= red_start or candidate_hue <= red_end:
                    is_excluded = True
                # Check Green
                elif green_start <= candidate_hue <= green_end:
                    is_excluded = True
                # Check Blue
                elif blue_start <= candidate_hue <= blue_end:
                    is_excluded = True
                
                if is_excluded:
                    continue
                
                # Check if hue is too similar to existing colors
                if existing_colors is not None and len(existing_colors) > 0:
                    # Convert candidate hue to RGB to check similarity
                    h_norm = (candidate_hue % 360) / 360.0
                    s_norm = 1.0  # 100% saturation
                    l_norm = 0.5  # 50% lightness
                    candidate_rgb = colorsys.hls_to_rgb(h_norm, l_norm, s_norm)
                    candidate_color = [float(candidate_rgb[0]), float(candidate_rgb[1]), float(candidate_rgb[2])]
                    
                    # Check similarity with all existing colors
                    too_similar = False
                    for existing_color in existing_colors:
                        if self._colors_are_similar(candidate_color, existing_color, min_hue_separation):
                            too_similar = True
                            break
                    
                    if too_similar:
                        continue
                
                # Valid hue found
                hue = candidate_hue
                break
            
            # Fallback: if we couldn't find a non-excluded hue, use a random one
            if hue is None:
                hue = rng.uniform(0, 360)
                self.logger.warning("Could not find unique non-excluded hue, using random hue")
        else:
            # No primary exclusion, but still check for uniqueness
            max_attempts = 500
            hue = None
            
            for attempt in range(max_attempts):
                candidate_hue = rng.uniform(0, 360)
                
                # Check if hue is too similar to existing colors
                if existing_colors is not None and len(existing_colors) > 0:
                    # Convert candidate hue to RGB to check similarity
                    h_norm = (candidate_hue % 360) / 360.0
                    s_norm = 1.0  # 100% saturation
                    l_norm = 0.5  # 50% lightness
                    candidate_rgb = colorsys.hls_to_rgb(h_norm, l_norm, s_norm)
                    candidate_color = [float(candidate_rgb[0]), float(candidate_rgb[1]), float(candidate_rgb[2])]
                    
                    # Check similarity with all existing colors
                    too_similar = False
                    for existing_color in existing_colors:
                        if self._colors_are_similar(candidate_color, existing_color, min_hue_separation):
                            too_similar = True
                            break
                    
                    if too_similar:
                        continue
                
                # Valid hue found
                hue = candidate_hue
                break
            
            if hue is None:
                hue = rng.uniform(0, 360)
                self.logger.warning("Could not find unique hue, using random hue")
        
        # Fixed: 100% saturation, 50% lightness
        saturation = 100.0
        lightness = 50.0
        
        # Convert HSL to RGB
        # colorsys uses HLS (hue, lightness, saturation) - same as HSL but different order
        h_norm = (hue % 360) / 360.0
        s_norm = saturation / 100.0
        l_norm = lightness / 100.0
        
        rgb = colorsys.hls_to_rgb(h_norm, l_norm, s_norm)
        
        # Ensure RGB values are in valid range [0, 1]
        rgb_clamped = [
            max(0.0, min(1.0, float(rgb[0]))),
            max(0.0, min(1.0, float(rgb[1]))),
            max(0.0, min(1.0, float(rgb[2])))
        ]
        
        # Verify the color is valid (sanity check)
        if not all(0.0 <= c <= 1.0 for c in rgb_clamped):
            self.logger.warning(f"Generated invalid RGB color: {rgb_clamped}, clamping to valid range")
        
        return rgb_clamped

    def _check_box_collision(self, new_pos_2d, new_scale, existing_boxes, min_separation):
        """
        Check if a new box position would collide with existing boxes.
        
        Args:
            new_pos_2d: New box position [x, y]
            new_scale: New box scale [x, y, z]
            existing_boxes: List of existing boxes, each as dict with 'position' [x, y, z] and 'scale' [x, y, z]
            min_separation: Minimum separation distance between box centers (meters)
        
        Returns:
            bool: True if collision detected, False otherwise
        """
        # Calculate effective radius for new box (using max of x and y scale)
        new_radius = max(new_scale[0], new_scale[1]) / 2.0
        
        for existing_box in existing_boxes:
            existing_pos_2d = existing_box['position'][:2]
            existing_scale = existing_box['scale']
            existing_radius = max(existing_scale[0], existing_scale[1]) / 2.0
            
            # Calculate distance between box centers
            distance = np.linalg.norm(new_pos_2d - existing_pos_2d)
            
            # Check if boxes are too close (considering their sizes and minimum separation)
            required_distance = new_radius + existing_radius + min_separation
            if distance < required_distance:
                return True
        
        return False
    
    def _apply_randomization(self):
        """
        Apply randomization to environment parameters if enabled.
        Randomizes start/goal positions and box positions with constraints:
        - All points inside walls with inset
        - Start and goal have at least 2/3 of map diagonal distance
        - Boxes are positioned 2-3m from line connecting start and goal
        - Boxes do not interfere with each other
        """
        if not self.config["randomize"]:
            return
        
        cfg = self.config
        
        # Initialize random number generator with seed for reproducibility
        random_seed = cfg.get("random_seed", None)
        if random_seed is None:
            # Generate a random seed if not specified
            random_seed = np.random.randint(0, 2**31 - 1)
            self.logger.info(f"Using random seed: {random_seed}")
        else:
            self.logger.info(f"Using fixed seed: {random_seed}")
        
        # Create seeded random number generator
        rng = np.random.RandomState(random_seed)
        
        # Store the seed that was used for reference
        self.config["_used_seed"] = random_seed
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
        
        # Randomize object properties only if object_type is not "none"
        object_type = cfg.get("object_type", "none")
        if object_type != "none":
            if object_type == "gate":
                # Gate randomization is handled in _create_gate_object()
                # Just log start and goal here
                self.logger.info(f"Randomization applied (gate will be created during setup):")
                self.logger.info(f"  Start: [{start_pos[0]:.2f}, {start_pos[1]:.2f}]")
                self.logger.info(f"  Goal: [{goal_pos[0]:.2f}, {goal_pos[1]:.2f}]")
                self.logger.info(f"  Start-Goal distance: {np.linalg.norm(goal_pos - start_pos):.2f} m (min: {min_start_goal_distance:.2f} m)")
            else:
                # Box/sphere randomization for multiple boxes
                num_boxes = cfg.get("num_boxes", 1)
                min_separation = cfg.get("box_min_separation", 1.5)
                
                # Vector from start to goal
                start_to_goal = goal_pos - start_pos
                line_length = np.linalg.norm(start_to_goal)
                line_direction = start_to_goal / line_length if line_length > 0 else np.array([1.0, 0.0])
                
                # Perpendicular direction (rotate 90 degrees)
                perp_direction = np.array([-line_direction[1], line_direction[0]])
                
                # Store multiple boxes configuration
                boxes_config = []
                existing_boxes = []  # For collision checking
                used_colors = []  # Track used colors to ensure uniqueness
                
                for box_idx in range(num_boxes):
                    max_box_attempts = 500
                    box_pos_2d = None
                    box_scale = None
                    
                    for box_attempt in range(max_box_attempts):
                        # Random distance along the line (between 0.2 and 0.8 of line length)
                        t = rng.uniform(0.2, 0.8)
                        point_on_line = start_pos + t * start_to_goal
                        
                        # Random distance perpendicular to line (between min and max)
                        perp_distance = rng.uniform(cfg["box_line_distance_min"], cfg["box_line_distance_max"])
                        # Random sign (left or right of line)
                        perp_distance *= rng.choice([-1, 1])
                        
                        # Box position
                        candidate_pos_2d = point_on_line + perp_distance * perp_direction
                        
                        # Clamp box position to valid area
                        candidate_pos_2d = np.clip(candidate_pos_2d, min_coord, max_coord)
                        
                        # Randomize box scale in all three dimensions
                        candidate_scale = [
                            rng.uniform(*cfg["box_scale_range"][0]),  # x scale
                            rng.uniform(*cfg["box_scale_range"][1]),  # y scale
                            rng.uniform(*cfg["box_scale_range"][2])   # z scale
                        ]
                        
                        # Check collision with existing boxes
                        if not self._check_box_collision(candidate_pos_2d, candidate_scale, existing_boxes, min_separation):
                            box_pos_2d = candidate_pos_2d
                            box_scale = candidate_scale
                            break
                    
                    if box_pos_2d is None:
                        self.logger.warning(f"Failed to find valid position for box {box_idx + 1} after {max_box_attempts} attempts, skipping")
                        continue
                    
                    # Generate random HSL color (hue random, saturation=100%, lightness=50%)
                    # Excludes colors near Red, Green, Blue and ensures uniqueness
                    # Use larger exclusion range (50°) to ensure colors are clearly different from primaries
                    box_color = self._generate_random_hsl_color(
                        rng, 
                        exclude_primary=True, 
                        exclusion_range=50.0,  # Increased from 30° to 50° for better separation
                        existing_colors=used_colors,
                        min_hue_separation=20.0  # Increased from 15° to 20° for better distinction
                    )
                    used_colors.append(box_color)  # Track this color
                    
                    # Randomize box mass within specified range
                    box_mass = rng.uniform(*cfg["box_mass_range"])
                    
                    # Store box configuration
                    box_config = {
                        "position": [
                            float(box_pos_2d[0]),
                            float(box_pos_2d[1]),
                            cfg["box_position"][2]  # Keep original z coordinate
                        ],
                        "scale": box_scale,
                        "color": box_color,
                        "mass": box_mass
                    }
                    boxes_config.append(box_config)
                    
                    # Add to existing boxes for collision checking
                    existing_boxes.append(box_config)
                
                # Store boxes configuration
                self.config["boxes_config"] = boxes_config
                
                # For backward compatibility, also set single box config (use first box if available)
                if len(boxes_config) > 0:
                    first_box = boxes_config[0]
                    self.config["box_position"] = first_box["position"]
                    self.config["box_scale"] = first_box["scale"]
                    self.config["box_color"] = first_box["color"]
                    self.config["box_mass"] = first_box["mass"]
                
                self.logger.info(f"Randomization applied:")
                self.logger.info(f"  Start: [{start_pos[0]:.2f}, {start_pos[1]:.2f}]")
                self.logger.info(f"  Goal: [{goal_pos[0]:.2f}, {goal_pos[1]:.2f}]")
                self.logger.info(f"  Start-Goal distance: {np.linalg.norm(goal_pos - start_pos):.2f} m (min: {min_start_goal_distance:.2f} m)")
                self.logger.info(f"  {object_type.capitalize()}s: {len(boxes_config)} spawned")
                # Log boxes as list format
                boxes_info = []
                for idx, box_cfg in enumerate(boxes_config):
                    pos = box_cfg["position"]
                    scale = box_cfg["scale"]
                    color = box_cfg["color"]
                    mass = box_cfg["mass"]
                    boxes_info.append({
                        'index': idx,
                        'position': [float(pos[0]), float(pos[1]), float(pos[2])],
                        'scale': [float(scale[0]), float(scale[1]), float(scale[2])],
                        'color': [float(color[0]), float(color[1]), float(color[2])],
                        'mass': float(mass)
                    })
                self.logger.info(f"  Boxes (list format): {boxes_info}")
        else:
            # Box is disabled, only log start and goal
            self.logger.info(f"Randomization applied (object disabled):")
            self.logger.info(f"  Start: [{start_pos[0]:.2f}, {start_pos[1]:.2f}]")
            self.logger.info(f"  Goal: [{goal_pos[0]:.2f}, {goal_pos[1]:.2f}]")
            self.logger.info(f"  Start-Goal distance: {np.linalg.norm(goal_pos - start_pos):.2f} m (min: {min_start_goal_distance:.2f} m)")

    # ===================== Object Management =====================
    def _get_object_prim_path(self, box_idx=0):
        """
        Get the prim path for the current object based on object_type.
        
        Args:
            box_idx: Index of the box (for multiple boxes)
        
        Returns:
            str: Prim path of the object
        """
        object_type = self.config.get("object_type", "none")
        if object_type == "box":
            if box_idx == 0:
                return "/World/ObstacleBox"  # Backward compatibility
            else:
                return f"/World/ObstacleBox{box_idx}"
        elif object_type == "sphere":
            if box_idx == 0:
                return "/World/ObstacleSphere"  # Backward compatibility
            else:
                return f"/World/ObstacleSphere{box_idx}"
        elif object_type == "gate":
            return "/World/Gate"  # Base path for gate (contains GateL and GateR)
        else:
            # Return None for "none" or unknown types
            return None
    
    def _create_object(self, cfg, position, scale, color, box_idx=0, mass=None):
        """
        Create a dynamic object in the environment based on object_type.
        Modular function to support different object types (box, sphere, gate).
        
        Args:
            cfg: Configuration dictionary
            position: Object position [x, y, z]
            scale: Object scale [x, y, z]
            color: Object color [r, g, b]
            box_idx: Index of the box (for multiple boxes)
            mass: Object mass (if None, uses cfg["box_mass"])
        """
        object_type = cfg.get("object_type", "none")
        
        if object_type == "none":
            self.logger.warning("_create_object called with object_type='none', skipping object creation")
            return
        
        object_path = self._get_object_prim_path(box_idx)
        
        if object_type == "box":
            box_mass = mass if mass is not None else cfg["box_mass"]
            self._create_box_object(cfg, object_path, position, scale, color, box_mass)
        elif object_type == "sphere":
            box_mass = mass if mass is not None else cfg["box_mass"]
            self._create_sphere_object(cfg, object_path, position, scale, color, box_mass)
        elif object_type == "gate":
            # Gate doesn't use position/scale/color from randomization, it calculates its own
            self._create_gate_object(cfg)
        else:
            self.logger.warning(f"Unknown object type '{object_type}', skipping object creation")
    
    def _create_box_object(self, cfg, prim_path, position, scale, color, mass):
        """
        Create a dynamic box object.
        
        Args:
            cfg: Configuration dictionary
            prim_path: USD prim path for the box
            position: Box position [x, y, z]
            scale: Box scale [x, y, z]
            color: Box color [r, g, b] (list or numpy array)
            mass: Box mass (kg)
        """
        # Ensure color is a numpy array with values in [0, 1] range
        if not isinstance(color, np.ndarray):
            color = np.array(color, dtype=np.float32)
        else:
            color = color.astype(np.float32)
        
        # Validate color values are in valid range
        color = np.clip(color, 0.0, 1.0)
        
        # Ensure position and scale are numpy arrays
        if not isinstance(position, np.ndarray):
            position = np.array(position, dtype=np.float32)
        if not isinstance(scale, np.ndarray):
            scale = np.array(scale, dtype=np.float32)
        
        # Create dynamic box
        self.world.scene.add(DynamicCuboid(
            prim_path=prim_path, name=f"obstacle_box_{prim_path.split('/')[-1]}",
            position=position, scale=scale, color=color,
            mass=mass, linear_velocity=np.array([0.0, 0.0, 0.0])
        ))
        
        # Apply physics material to box (friction and restitution)
        box_prim = self.stage.GetPrimAtPath(prim_path)
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
            collider = UsdPhysics.CollisionAPI.Get(self.stage, prim_path)
            if collider:
                collider.GetPrim().CreateRelationship("physics:material").SetTargets(
                    [Sdf.Path(physics_material_path)]
                )
        
        # Log box creation with list format for compatibility
        box_info = {
            'prim_path': prim_path,
            'position': [float(position[0]), float(position[1]), float(position[2])],
            'scale': [float(scale[0]), float(scale[1]), float(scale[2])],
            'color': [float(color[0]), float(color[1]), float(color[2])],
            'mass': float(mass)
        }
        self.logger.info(f"Box object created: {box_info}")
    
    def _create_sphere_object(self, cfg, prim_path, position, scale, color, mass):
        """
        Create a dynamic sphere object.
        
        Args:
            cfg: Configuration dictionary
            prim_path: USD prim path for the sphere
            position: Sphere position [x, y, z]
            scale: Sphere scale [x, y, z] (radius will be average of x, y, z)
            color: Sphere color [r, g, b] (list or numpy array)
            mass: Sphere mass (kg)
        """
        # Ensure color is a numpy array with values in [0, 1] range
        if not isinstance(color, np.ndarray):
            color = np.array(color, dtype=np.float32)
        else:
            color = color.astype(np.float32)
        
        # Validate color values are in valid range
        color = np.clip(color, 0.0, 1.0)
        
        # Ensure position and scale are numpy arrays
        if not isinstance(position, np.ndarray):
            position = np.array(position, dtype=np.float32)
        if not isinstance(scale, np.ndarray):
            scale = np.array(scale, dtype=np.float32)
        # Ensure color is a numpy array with values in [0, 1] range
        if not isinstance(color, np.ndarray):
            color = np.array(color, dtype=np.float32)
        else:
            color = color.astype(np.float32)
        
        # Validate color values are in valid range
        color = np.clip(color, 0.0, 1.0)
        
        # Ensure position and scale are numpy arrays
        if not isinstance(position, np.ndarray):
            position = np.array(position, dtype=np.float32)
        if not isinstance(scale, np.ndarray):
            scale = np.array(scale, dtype=np.float32)
        
        # For sphere, use average of scale dimensions as radius
        radius = np.mean(scale) if len(scale) >= 3 else scale[0] if len(scale) > 0 else 0.5
        
        # Create dynamic sphere using USD prim (Isaac Sim doesn't have DynamicSphere directly)
        sphere = UsdGeom.Sphere.Define(self.stage, prim_path)
        sphere.GetRadiusAttr().Set(radius)
        sphere.CreateDisplayColorAttr().Set([Gf.Vec3f(float(color[0]), float(color[1]), float(color[2]))])
        
        # Set position
        sphere_xform = UsdGeom.Xformable(sphere)
        sphere_xform.ClearXformOpOrder()
        sphere_xform.AddTranslateOp().Set(Gf.Vec3d(float(position[0]), float(position[1]), float(position[2])))
        
        # Apply physics properties
        sphere_prim = self.stage.GetPrimAtPath(prim_path)
        if sphere_prim.IsValid():
            # Add collision API
            UsdPhysics.CollisionAPI.Apply(sphere_prim)
            
            # Add rigid body API
            rigid_body = UsdPhysics.RigidBodyAPI.Apply(sphere_prim)
            rigid_body.CreateMassAttr().Set(mass)
            
            # Apply physics material
            physics_material_path = "/World/Materials/SpherePhysicsMaterial"
            physics_material = UsdPhysics.MaterialAPI.Apply(
                self.stage.DefinePrim(physics_material_path, "Material")
            )
            physics_material.CreateStaticFrictionAttr().Set(cfg["box_friction_static"])
            physics_material.CreateDynamicFrictionAttr().Set(cfg["box_friction_dynamic"])
            physics_material.CreateRestitutionAttr().Set(cfg["box_restitution"])
            
            # Bind physics material
            collider = UsdPhysics.CollisionAPI.Get(self.stage, prim_path)
            if collider:
                collider.GetPrim().CreateRelationship("physics:material").SetTargets(
                    [Sdf.Path(physics_material_path)]
                )
        
        self.logger.info(f"Sphere object created at {prim_path} with radius {radius:.2f}m")
    
    # ===================== Debug Transform Methods =====================
    def get_prim_transform(self, prim_path):
        """
        Get transform (translation, rotation, scale) of a prim.
        
        Args:
            prim_path: Path to the prim (e.g., "/World/Gate")
            
        Returns:
            dict: Dictionary containing 'translation', 'rotation_euler', 'rotation_quat', 'scale'
                  Returns None if prim is invalid or not xformable
        """
        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            self.logger.warning(f"Prim at {prim_path} is not valid")
            return None
        
        if not prim.IsA(UsdGeom.Xformable):
            self.logger.warning(f"Prim at {prim_path} is not Xformable")
            return None
        
        xformable = UsdGeom.Xformable(prim)
        
        # Get all xform ops
        xform_ops = xformable.GetOrderedXformOps()
        
        result = {
            'translation': None,
            'rotation_euler': None,
            'rotation_quat': None,
            'scale': None,
            'xform_ops': []
        }
        
        for op in xform_ops:
            op_type = op.GetOpType()
            op_name = op.GetOpName()
            value = op.Get()
            
            result['xform_ops'].append({
                'name': op_name,
                'type': str(op_type),
                'value': value
            })
            
            if op_type == UsdGeom.XformOp.TypeTranslate:
                result['translation'] = [value[0], value[1], value[2]]
            elif op_type == UsdGeom.XformOp.TypeRotateXYZ:
                result['rotation_euler'] = [value[0], value[1], value[2]]
            elif op_type == UsdGeom.XformOp.TypeOrient:
                quat = value
                result['rotation_quat'] = {
                    'w': quat.GetReal(),
                    'x': quat.GetImaginary()[0],
                    'y': quat.GetImaginary()[1],
                    'z': quat.GetImaginary()[2]
                }
                # Convert quaternion to euler angles (in degrees)
                # Assuming Z-axis rotation only for simplicity
                yaw = 2.0 * np.arctan2(quat.GetImaginary()[2], quat.GetReal())
                result['rotation_euler_from_quat'] = [0.0, 0.0, np.degrees(yaw)]
            elif op_type == UsdGeom.XformOp.TypeScale:
                result['scale'] = [value[0], value[1], value[2]]
        
        # Get world transform
        world_transform = xformable.ComputeLocalToWorldTransform(0)
        result['world_matrix'] = world_transform
        
        # Log the results
        self.logger.info(f"Transform for prim '{prim_path}':")
        self.logger.info(f"  Translation: {result['translation']}")
        if result['rotation_quat']:
            self.logger.info(f"  Rotation (quat): w={result['rotation_quat']['w']:.4f}, x={result['rotation_quat']['x']:.4f}, y={result['rotation_quat']['y']:.4f}, z={result['rotation_quat']['z']:.4f}")
            self.logger.info(f"  Rotation (euler from quat): {result['rotation_euler_from_quat']}")
        if result['rotation_euler']:
            self.logger.info(f"  Rotation (euler): {result['rotation_euler']}")
        self.logger.info(f"  Scale: {result['scale']}")
        self.logger.info(f"  XformOps count: {len(result['xform_ops'])}")
        for i, op in enumerate(result['xform_ops']):
            self.logger.info(f"    Op {i}: {op['name']} ({op['type']}) = {op['value']}")
        
        return result
    
    def set_prim_transform(self, prim_path, translation=None, rotation_deg=None, rotation_quat=None, scale=None, clear_existing=False):
        """
        Set transform (translation, rotation, scale) of a prim.
        
        Args:
            prim_path: Path to the prim (e.g., "/World/Gate")
            translation: [x, y, z] translation (meters)
            rotation_deg: [x, y, z] rotation in degrees (euler angles)
            rotation_quat: [w, x, y, z] rotation as quaternion (if provided, overrides rotation_deg)
            scale: [x, y, z] scale
            clear_existing: If True, clear all existing xform ops before setting new ones
            
        Returns:
            bool: True if successful, False otherwise
        """
        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            self.logger.warning(f"Prim at {prim_path} is not valid")
            return False
        
        if not prim.IsA(UsdGeom.Xformable):
            self.logger.warning(f"Prim at {prim_path} is not Xformable")
            return False
        
        xformable = UsdGeom.Xformable(prim)
        
        if clear_existing:
            xformable.ClearXformOpOrder()
            self.logger.info(f"Cleared existing xform ops for {prim_path}")
        
        # Apply transformations in order: scale -> rotate -> translate
        if scale is not None:
            scale_op = xformable.AddScaleOp(UsdGeom.XformOp.PrecisionDouble)
            scale_op.Set(Gf.Vec3d(scale[0], scale[1], scale[2]))
            self.logger.info(f"Set scale: {scale}")
        
        if rotation_quat is not None:
            # Use quaternion rotation
            rotate_op = xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
            quat = Gf.Quatd(rotation_quat[0], rotation_quat[1], rotation_quat[2], rotation_quat[3])  # w, x, y, z
            rotate_op.Set(quat)
            self.logger.info(f"Set rotation (quat): w={rotation_quat[0]:.4f}, x={rotation_quat[1]:.4f}, y={rotation_quat[2]:.4f}, z={rotation_quat[3]:.4f}")
        elif rotation_deg is not None:
            # Use euler angle rotation
            rotate_op = xformable.AddRotateXYZOp(UsdGeom.XformOp.PrecisionDouble)
            rotate_op.Set(Gf.Vec3d(rotation_deg[0], rotation_deg[1], rotation_deg[2]))
            self.logger.info(f"Set rotation (euler): {rotation_deg}°")
        
        if translation is not None:
            translate_op = xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
            translate_op.Set(Gf.Vec3d(translation[0], translation[1], translation[2]))
            self.logger.info(f"Set translation: {translation}")
        
        self.logger.info(f"Transform set successfully for {prim_path}")
        return True
    
    def set_prim_transform_from_yaw(self, prim_path, translation=None, yaw_deg=None, yaw_rad=None, clear_existing=False):
        """
        Set transform with Z-axis rotation (yaw) only.
        Convenience method for 2D transformations.
        
        Args:
            prim_path: Path to the prim (e.g., "/World/Gate")
            translation: [x, y, z] translation (meters)
            yaw_deg: Yaw angle in degrees (Z-axis rotation)
            yaw_rad: Yaw angle in radians (Z-axis rotation, overrides yaw_deg if provided)
            clear_existing: If True, clear all existing xform ops before setting new ones
            
        Returns:
            bool: True if successful, False otherwise
        """
        # Convert yaw to quaternion
        if yaw_rad is not None:
            yaw = yaw_rad
        elif yaw_deg is not None:
            yaw = np.radians(yaw_deg)
        else:
            yaw = 0.0
        
        # Create quaternion for Z-axis rotation
        half_yaw = yaw / 2.0
        rotation_quat = [
            np.cos(half_yaw),   # w
            0.0,                # x
            0.0,                # y
            np.sin(half_yaw)    # z
        ]
        
        self.logger.info(f"Setting transform with yaw: {np.degrees(yaw):.2f}° ({yaw:.4f} rad)")
        
        return self.set_prim_transform(
            prim_path=prim_path,
            translation=translation,
            rotation_quat=rotation_quat,
            clear_existing=clear_existing
        )
    
    # ===================== Object Creation Methods =====================
    def _get_rng(self):
        """
        Get the random number generator for this simulation.
        Uses the seed from config if available, otherwise creates a new one.
        
        Returns:
            numpy RandomState instance
        """
        cfg = self.config
        random_seed = cfg.get("_used_seed", None)
        if random_seed is None:
            # If no seed was used yet, get or generate one
            random_seed = cfg.get("random_seed", None)
            if random_seed is None:
                random_seed = np.random.randint(0, 2**31 - 1)
            cfg["_used_seed"] = random_seed
        return np.random.RandomState(random_seed)
    
    def _create_gate_object(self, cfg):
        """
        Create a gate object with two collinear walls (GateL and GateR) on Y-axis.
        The gap between them allows the robot to pass through.
        
        Simple version: Only apply scale and Y position to GateL and GateR.
        No Gate prim transformations.
        
        Args:
            cfg: Configuration dictionary
        """
        if self.start_pos is None or self.goal_pos is None:
            raise RuntimeError("Start and goal positions must be set before creating gate")
        
        rng = self._get_rng()
        start_pos = self.start_pos
        goal_pos = self.goal_pos
        
        # Randomize gate parameters
        wall_depth = rng.uniform(cfg["wall_depth_min"], cfg["wall_depth_max"])
        gate_y_offset = rng.uniform(cfg["gate_y_offset_min"], cfg["gate_y_offset_max"])
        gap = rng.uniform(cfg["gap_min"], cfg["gap_max"])
        gate_location_t = rng.uniform(cfg["gate_location_min"], cfg["gate_location_max"])
        
        # Gate wall scale: [wall_depth (x), map_size (y), wall_height (z)]
        gate_y_scale = cfg["map_size"]  # Y-axis scale uses environment wall scale
        gate_z_scale = cfg["wall_height"]  # Z-axis scale uses environment wall height
        gate_scale = np.array([wall_depth, gate_y_scale, gate_z_scale])
        gate_color = np.array(cfg["gate_color"])
        
        # Calculate GateL and GateR Y positions
        # GateL: -Y direction by -(gate_y_scale + gap)/2 + gate_y_offset
        # GateR: +Y direction by +(gate_y_scale + gap)/2 + gate_y_offset
        gate_separation_L = -(gate_y_scale + gap) / 2.0 + gate_y_offset
        gate_separation_R = (gate_y_scale + gap) / 2.0 + gate_y_offset
        
        # ===== Calculate Gate prim transformation (for manual xform application) =====
        # Calculate start-to-goal vector and line direction
        start_to_goal = goal_pos - start_pos
        line_length = np.linalg.norm(start_to_goal)
        line_direction = start_to_goal / line_length if line_length > 0 else np.array([1.0, 0.0])
        
        # Calculate gate position along start-goal line (0.2 ~ 0.8)
        gate_pos_2d = start_pos + gate_location_t * start_to_goal
        
        # Calculate rotation: Y-axis should be perpendicular to start-goal line
        # Line yaw angle
        line_yaw = np.arctan2(line_direction[1], line_direction[0])
        # Gate Y-axis should be perpendicular: subtract 90 degrees
        gate_yaw = line_yaw - np.pi / 2.0
        
        # Convert to degrees for easier reading
        line_yaw_deg = np.degrees(line_yaw)
        gate_yaw_deg = np.degrees(gate_yaw)
        
        # Step 1: Create empty Gate prim (Xform) and setup xformOps with initial values
        # This is equivalent to creating a prim in GUI - it will have translate and rotate ops
        gate_prim_path = "/World/Gate"
        gate_xform_prim = UsdGeom.Xform.Define(self.stage, gate_prim_path)
        
        # Setup xformOps (like GUI does) with initial values at origin
        gate_xform = UsdGeom.Xformable(gate_xform_prim)
        gate_xform.ClearXformOpOrder()
        
        # Add translate op (will be modified later)
        gate_translate_op = gate_xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble, "")
        gate_translate_op.Set(Gf.Vec3d(0.0, 0.0, 0.0))  # Initial: origin
        
        # Add rotateXYZ op (will be modified later) - this is what GUI uses
        gate_rotate_op = gate_xform.AddRotateXYZOp(UsdGeom.XformOp.PrecisionDouble, "")
        gate_rotate_op.Set(Gf.Vec3d(0.0, 0.0, 0.0))  # Initial: no rotation
        
        # Step 2: Create GateL and GateR with scale and Y position only
        gateL_path = f"{gate_prim_path}/GateL"
        gateR_path = f"{gate_prim_path}/GateR"
        
        # Create GateL with scale and Y position
        gateL = self.world.scene.add(FixedCuboid(
            prim_path=gateL_path,
            name="gate_left",
            position=np.array([0.0, gate_separation_L, 0.0]),  # X=0, Y=negative, Z=0
            scale=gate_scale,
            color=gate_color
        ))
        
        # Create GateR with scale and Y position
        gateR = self.world.scene.add(FixedCuboid(
            prim_path=gateR_path,
            name="gate_right",
            position=np.array([0.0, gate_separation_R, 0.0]),  # X=0, Y=positive, Z=0
            scale=gate_scale,
            color=gate_color
        ))
        
        # Apply Collider preset to GateL and GateR
        gateL_prim = self.stage.GetPrimAtPath(gateL_path)
        gateR_prim = self.stage.GetPrimAtPath(gateR_path)
        
        if gateL_prim.IsValid():
            UsdPhysics.CollisionAPI.Apply(gateL_prim)
        if gateR_prim.IsValid():
            UsdPhysics.CollisionAPI.Apply(gateR_prim)
        
        # Store gate properties in config for logging
        self.config["gate_wall_depth"] = wall_depth
        self.config["gate_y_offset"] = gate_y_offset
        self.config["gate_gap"] = gap
        self.config["gate_location_t"] = gate_location_t
        self.config["gate_pos"] = gate_pos_2d.tolist()
        self.config["gate_yaw"] = gate_yaw
        
        self.logger.info(f"Gate created (simple version - no Gate prim transformations):")
        self.logger.info(f"  Wall depth (X-scale): {wall_depth:.2f} m")
        self.logger.info(f"  Y-axis scale: {gate_y_scale:.2f} m (matches environment)")
        self.logger.info(f"  Z-axis scale: {gate_z_scale:.2f} m (matches environment)")
        self.logger.info(f"  Gap: {gap:.2f} m")
        self.logger.info(f"  Y offset: {gate_y_offset:.2f} m")
        self.logger.info(f"  GateL position (local): [0.0, {gate_separation_L:.2f}, 0.0]")
        self.logger.info(f"  GateR position (local): [0.0, {gate_separation_R:.2f}, 0.0]")
        self.logger.info(f"")
        self.logger.info(f"===== Gate Prim Xform Values (for manual application) =====")
        self.logger.info(f"  Start position: [{start_pos[0]:.2f}, {start_pos[1]:.2f}]")
        self.logger.info(f"  Goal position: [{goal_pos[0]:.2f}, {goal_pos[1]:.2f}]")
        self.logger.info(f"  Start->Goal line angle: {line_yaw_deg:.2f}° (radians: {line_yaw:.4f})")
        self.logger.info(f"")
        self.logger.info(f"  Gate XY Position: [{gate_pos_2d[0]:.4f}, {gate_pos_2d[1]:.4f}] (Z=0)")
        self.logger.info(f"  Gate location_t: {gate_location_t:.2f} ({gate_location_t*100:.1f}% along start-goal line)")
        self.logger.info(f"")
        self.logger.info(f"  Gate Yaw Angle (degrees): {gate_yaw_deg:.2f}°")
        self.logger.info(f"  Gate Yaw Angle (radians): {gate_yaw:.4f}")
        self.logger.info(f"  (This is line_yaw - 90° = {line_yaw_deg:.2f}° - 90° = {gate_yaw_deg:.2f}°)")
        self.logger.info(f"============================================================")
        
        # Create GUI button for applying transform (register with display if available)
        self.create_gate_transform_button()
        
        # Update display if it already exists (gate is created after display initialization)
        if self.display and hasattr(self.display, 'gate_transform_callback'):
            self.display.gate_transform_callback = self.apply_gate_transform
            self.display.has_gate = True
    
    def apply_gate_transform(self):
        """
        Apply Gate prim transform by modifying existing xformOp values.
        This is equivalent to manually editing values in GUI's Property panel.
        
        The xformOps are already created during gate creation, we just modify their values.
        This method should be called after gate creation.
        """
        # Get stored gate transform values
        if "gate_pos" not in self.config or "gate_yaw" not in self.config:
            self.logger.error("Gate transform values not found. Create gate first.")
            return False
        
        gate_pos = self.config["gate_pos"]  # [x, y]
        gate_yaw_rad = self.config["gate_yaw"]  # radians
        
        # Convert yaw to degrees and add 90 degrees as requested
        gate_yaw_deg = np.degrees(gate_yaw_rad) + 90.0
        
        # Normalize angle to [-180, 180] range
        while gate_yaw_deg > 180:
            gate_yaw_deg -= 360
        while gate_yaw_deg < -180:
            gate_yaw_deg += 360
        
        gate_prim_path = "/World/Gate"
        
        self.logger.info(f"Applying Gate transform (like GUI editing):")
        self.logger.info(f"  Translation: [{gate_pos[0]:.4f}, {gate_pos[1]:.4f}, 0.0]")
        self.logger.info(f"  Rotation Z: {gate_yaw_deg:.2f}° (original yaw + 90°)")
        
        try:
            prim = self.stage.GetPrimAtPath(gate_prim_path)
            if not prim.IsValid() or not prim.IsA(UsdGeom.Xformable):
                self.logger.error(f"Gate prim at {gate_prim_path} is not valid or not Xformable")
                return False
            
            xformable = UsdGeom.Xformable(prim)
            xform_ops = xformable.GetOrderedXformOps()
            
            # Find and modify existing xformOps (like GUI does)
            translate_op = None
            rotate_op = None
            
            for op in xform_ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    translate_op = op
                elif op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                    rotate_op = op
            
            # Modify translate op value (like typing in GUI Translate tab)
            if translate_op:
                translate_op.Set(Gf.Vec3d(gate_pos[0], gate_pos[1], 0.0))
                self.logger.info(f"  ✓ Updated xformOp:translate")
            else:
                self.logger.warning(f"  ✗ translate op not found!")
            
            # Modify rotate op value (like typing in GUI Rotate tab)
            if rotate_op:
                rotate_op.Set(Gf.Vec3d(0.0, 0.0, gate_yaw_deg))
                self.logger.info(f"  ✓ Updated xformOp:rotateXYZ")
            else:
                self.logger.warning(f"  ✗ rotateXYZ op not found!")
            
            if translate_op and rotate_op:
                self.logger.info(f"Gate transform applied successfully!")
                return True
            else:
                return False
                
        except Exception as e:
            self.logger.error(f"Failed to apply gate transform: {e}")
            return False
    
    def create_gate_transform_button(self):
        """
        Create a GUI button to apply Gate transform.
        In headless mode with Pygame display, registers a keyboard shortcut ('g' key).
        """
        # Register gate transform callback with Pygame display if available
        if self.display and hasattr(self.display, 'gate_transform_callback'):
            self.display.gate_transform_callback = self.apply_gate_transform
            self.display.has_gate = True
            self.logger.info("Gate transform button registered: Press 'G' key in Pygame window to apply transform")
        else:
            self.logger.info("Gate transform available: Press 'G' key in Pygame window to apply transform")

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
        start_pos = np.array(cfg["start_position"])
        goal_pos = np.array(cfg["goal_position"])
        
        # For backward compatibility, get single box config if boxes_config doesn't exist
        box_color = np.array(cfg.get("box_color", [0.6, 0.4, 0.2]))
        box_pos = np.array(cfg.get("box_position", [2.0, 0.0, 0.25]))
        box_scale = np.array(cfg.get("box_scale", [1.0, 1.0, 0.5]))
        
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
        
        # 5. Create dynamic objects (can be pushed by robot) - only if object_type is not "none"
        object_type = cfg.get("object_type", "none")
        if object_type != "none":
            # Check if we have multiple boxes configuration from randomization
            boxes_config = cfg.get("boxes_config", None)
            num_boxes = cfg.get("num_boxes", 1)
            
            if boxes_config is not None and len(boxes_config) > 0:
                # Spawn multiple boxes from randomization
                for box_idx, box_cfg in enumerate(boxes_config):
                    box_pos = np.array(box_cfg["position"])
                    box_scale = np.array(box_cfg["scale"])
                    box_color = np.array(box_cfg["color"])
                    box_mass = box_cfg["mass"]
                    self._create_object(cfg, box_pos, box_scale, box_color, box_idx=box_idx, mass=box_mass)
            elif num_boxes > 1 and not cfg.get("randomize", False):
                # Multiple boxes but randomization disabled - create boxes at fixed offsets
                base_pos = box_pos
                base_scale = box_scale
                base_color = box_color
                base_mass = cfg.get("box_mass", 5.0)
                min_separation = cfg.get("box_min_separation", 1.5)
                
                # Create boxes in a grid pattern around the base position
                boxes_per_row = int(np.ceil(np.sqrt(num_boxes)))
                spacing = max(base_scale[0], base_scale[1]) + min_separation
                
                boxes_config = []
                used_colors = []  # Track used colors to ensure uniqueness
                rng = self._get_rng()
                
                for box_idx in range(num_boxes):
                    row = box_idx // boxes_per_row
                    col = box_idx % boxes_per_row
                    offset_x = (col - (boxes_per_row - 1) / 2.0) * spacing
                    offset_y = (row - (boxes_per_row - 1) / 2.0) * spacing
                    
                    box_pos_offset = np.array([
                        base_pos[0] + offset_x,
                        base_pos[1] + offset_y,
                        base_pos[2]
                    ])
                    
                    # Generate random HSL color for each box (hue random, saturation=100%, lightness=50%)
                    # Excludes colors near Red, Green, Blue and ensures uniqueness
                    # Use larger exclusion range (50°) to ensure colors are clearly different from primaries
                    box_color_offset = self._generate_random_hsl_color(
                        rng, 
                        exclude_primary=True, 
                        exclusion_range=50.0,  # Increased from 30° to 50° for better separation
                        existing_colors=used_colors,
                        min_hue_separation=20.0  # Increased from 15° to 20° for better distinction
                    )
                    used_colors.append(box_color_offset)  # Track this color
                    
                    self._create_object(cfg, box_pos_offset, base_scale, box_color_offset, box_idx=box_idx, mass=base_mass)
            else:
                # Backward compatibility: spawn single box with random HSL color
                # Generate random HSL color (hue random, saturation=100%, lightness=50%)
                # Excludes colors near Red, Green, Blue
                # Use larger exclusion range (50°) to ensure colors are clearly different from primaries
                rng = self._get_rng()
                random_box_color = self._generate_random_hsl_color(
                    rng, 
                    exclude_primary=True, 
                    exclusion_range=50.0  # Increased from 30° to 50° for better separation
                )
                self._create_object(cfg, box_pos, box_scale, random_box_color, box_idx=0)
        else:
            self.logger.info("Object spawning disabled (object_type='none')")
        
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
        
        # 9. Create top-down camera using generic camera creation function
        # Position: configurable height above origin, looking down
        top_res = self.config["top_camera_resolution"]
        top_camera_height = self.config.get("top_camera_height", 20.0)  # Default 20m if not specified
        
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
            translation=(0.0, 0.0, top_camera_height),  # Height from config
            rotation_quat=rotation_quat,  # -90° around Z axis
            focal_length=18.0,  # mm (for overhead view)
            horizontal_aperture=18.0,  # mm (square aspect ratio)
            resolution=top_res,
            clipping_range=(0.1, 50.0)  # Appropriate for overhead view
        )
        
        if camera_path:
            self.camera_path = camera_path
            self.logger.info(f"✓ Top camera created: {camera_path}")
            self.logger.info(f"  Resolution: {top_res[0]}×{top_res[1]} pixels")
        else:
            self.logger.warning("Failed to create top camera")
        
        self.logger.info("Environment setup complete")

    def setup_robot(self):
        """
        Setup Spot robot at start position, oriented to face the goal.
        Calculates yaw angle from start to goal and converts to quaternion.
        Spawn position is offset by 1/2 hemisphere diameter toward goal to prevent collision.
        """
        if self.start_pos is None or self.goal_pos is None:
            raise RuntimeError("Environment must be setup first")
        
        # Calculate direction vector from start to goal
        direction = self.goal_pos - self.start_pos

        ## Offset spawn position by 1/2 hemisphere diameter toward goal to prevent collision
        ## ===========================================================
        direction_norm = np.linalg.norm(direction)
        
        # Get hemisphere diameter from config
        hemisphere_diameter = self.config.get("goal_hemisphere_diameter", 1.0)
        offset_distance = hemisphere_diameter / 2.0
        
        # Calculate offset: move spawn point by half hemisphere diameter toward goal
        if direction_norm > 0:
            direction_normalized = direction / direction_norm
            offset = direction_normalized * offset_distance
        else:
            # If start and goal are at same position, no offset needed
            offset = np.array([0.0, 0.0])
        
        # Calculate spawn position: start position + offset toward goal
        spawn_pos = self.start_pos + offset
        ## ===============================================================

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
        
        # Add Spot robot to scene at offset spawn position with calculated orientation
        # Robot z position uses robot_height from config (default 0.65m above ground)
        robot_height = self.config.get("robot_height", 0.65)
        self.spot = SpotFlatTerrainPolicy(
            prim_path="/World/Spot",
            name="Spot",
            position=np.array([spawn_pos[0], spawn_pos[1], robot_height]),
            orientation=orientation,
        )
        
        self.logger.info(f"Robot placed at [{spawn_pos[0]:.2f}, {spawn_pos[1]:.2f}] (offset {offset_distance:.2f}m from start toward goal)")
        
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
            
            # Store hemisphere path for later color change
            self.goal_hemisphere_path = hemisphere_path
            
            self.logger.info(f"Goal hemisphere created at [{self.goal_pos[0]:.2f}, {self.goal_pos[1]:.2f}] with diameter {diameter:.2f}m")
            
        except Exception as e:
            self.logger.warning(f"Failed to create goal hemisphere: {e}")

    def _change_goal_color_to_green(self):
        """
        Change goal marker and hemisphere color from blue to green.
        Called when task is complete (distance < 1m).
        """
        try:
            # Change goal marker color
            goal_marker_path = "/World/GoalMarker"
            goal_marker_prim = self.stage.GetPrimAtPath(goal_marker_path)
            if goal_marker_prim.IsValid():
                goal_sphere = UsdGeom.Sphere(goal_marker_prim)
                goal_sphere.GetDisplayColorAttr().Set([Gf.Vec3f(0.0, 1.0, 0.0)])  # Green color
                self.logger.info("Goal marker color changed to green")
            
            # Change goal hemisphere color
            if hasattr(self, 'goal_hemisphere_path'):
                hemisphere_prim = self.stage.GetPrimAtPath(self.goal_hemisphere_path)
                if hemisphere_prim.IsValid():
                    hemisphere_sphere = UsdGeom.Sphere(hemisphere_prim)
                    hemisphere_sphere.GetDisplayColorAttr().Set([Gf.Vec3f(0.0, 1.0, 0.0)])  # Green color
                    self.logger.info("Goal hemisphere color changed to green")
        
        except Exception as e:
            self.logger.warning(f"Failed to change goal color: {e}")

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
            self.logger.error(f"Failed to create camera {camera_name}: {e}")
            return None
    
    def _setup_robot_camera(self):
        """
        Create ego-view camera attached to Spot robot body.
        Uses generic camera creation function with specific parameters.
        Must be called after world reset when robot is fully initialized.
        """
        try:
            # Check if robot body exists
            body_path = "/World/Spot/body"
            body_prim = self.stage.GetPrimAtPath(body_path)
            if not body_prim.IsValid():
                self.logger.warning(f"Robot body prim not found at {body_path}")
                return
            
            # Get camera configuration
            ego_res = self.config["ego_camera_resolution"]
            
            # Create ego camera with RealSense D455 RGB specifications
            # FOV: 90° H × 65° V, Resolution: 1280 × 800
            camera_path = self._create_camera(
                parent_path=body_path,
                camera_name="EgoCamera",
                translation=(0.3, 0.0, 0.2),  # Position relative to robot body
                rotation_quat=(0.5, 0.5, -0.5, -0.5),  # RPY (90°, -90°, 0°)
                focal_length=18.0,  # mm (for 90° H-FOV)
                horizontal_aperture=36.0,  # mm (90° H-FOV)
                resolution=ego_res,
                clipping_range=(0.01, 10000.0)
            )
            
            if camera_path:
                self.robot_camera_path = camera_path
            else:
                self.logger.warning("Failed to create ego camera")
            
        except Exception as e:
            self.logger.error(f"Failed to setup robot camera: {e}")
    
    def _open_robot_camera_viewport(self, camera_path):
        """
        Open a new viewport window showing the robot's ego-view camera.
        Note: This is for Isaac Sim viewport (not used when Pygame display is active).
        """
        try:
            from isaacsim.core.utils.viewports import create_viewport_for_camera
            
            ego_res = self.config["ego_camera_resolution"]
            
            create_viewport_for_camera(
                viewport_name="RobotEgoView",
                camera_prim_path=camera_path,
                width=ego_res[0],
                height=ego_res[1],
                position_x=100,
                position_y=100
            )
            
            self.logger.info(f"✓ Robot ego-view viewport opened: {ego_res[0]}×{ego_res[1]}")
            
        except Exception as e:
            self.logger.warning(f"Failed to open robot ego-view viewport: {e}")
    
    def _set_default_viewport_camera(self, camera_path):
        """
        Set the top camera as the default viewport camera (replaces main scene view).
        Note: Disabled in headless mode with Pygame display.
        """
        # Skip viewport setup in headless mode (we use Pygame display instead)
        pass
    
    def get_ego_camera_image(self):
        """
        Get current frame from ego camera for Pygame display.
        
        Returns:
            numpy.ndarray: RGB image array (H, W, 3) or None if not available
        """
        if not hasattr(self, 'camera_render_initialized') or not self.camera_render_initialized:
            return None
        
        if "ego" not in self.rgb_annotators:
            return None
        
        try:
            rgb_data = self.rgb_annotators["ego"].get_data()
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
            return image_array
                
        except Exception as e:
            # Silently return None on error (avoid spam during shutdown)
            return None
    
    def get_top_camera_image(self):
        """
        Get current frame from top camera for Pygame display.
        
        Returns:
            numpy.ndarray: RGB image array (H, W, 3) or None if not available
        """
        if not hasattr(self, 'camera_render_initialized') or not self.camera_render_initialized:
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
            return image_array
            
        except Exception as e:
            # Silently return None on error (avoid spam during shutdown)
            return None

    def _on_physics_step(self, step_size):
        """
        Physics step callback - called every physics timestep (500Hz).
        Handles command updates (50Hz), logging (10Hz), and robot control.
        
        Args:
            step_size: Physics timestep size
        """
        # Performance tracking: measure physics step time
        current_time = time.time()
        if self.last_physics_time is not None:
            physics_duration = (current_time - self.last_physics_time) * 1000  # Convert to ms
            self.physics_step_times.append(physics_duration)
            # Keep only last 500 samples (1 second at 500Hz)
            if len(self.physics_step_times) > 500:
                self.physics_step_times.pop(0)
        self.last_physics_time = current_time
        
        # Check for gate transform requests from Pygame display thread
        if self.display and hasattr(self.display, 'gate_transform_queue'):
            try:
                self.display.gate_transform_queue.get_nowait()
                # Process gate transform in main thread (safe to access USD stage here)
                if hasattr(self, 'apply_gate_transform'):
                    self.apply_gate_transform()
            except queue.Empty:
                pass
        
        # Command update: update controller at 50Hz (every 10 physics steps)
        # Physics runs at 500Hz, so 500Hz / 10 = 50Hz
        self.command_counter += 1
        if self.command_counter >= 10:
            self.command_counter = 0
            self.controller.update()  # Update controller state based on keyboard input
        
        # Performance logging: log performance stats at 1Hz (every 500 physics steps)
        self.performance_counter += 1
        if self.performance_counter >= self.performance_log_interval:
            self.performance_counter = 0
            self._log_performance_stats()
        
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
                
                # Check if first command has been received (any non-zero velocity)
                if not self.first_command_received:
                    if abs(cmd_vel[0]) > 0.01 or abs(cmd_vel[1]) > 0.01 or abs(cmd_vel[2]) > 0.01:
                        self.first_command_received = True
                        self.logger.info("First keyboard command received - starting data saving")
                
                # Get object positions and orientations from stage
                # Calculate L1 distance metric based on object type
                # Always use list format for compatibility (even for single box)
                boxes_data = []
                goal_pos_2d = np.array([self.goal_pos[0], self.goal_pos[1]])
                l1_distance = 0.0  # Initialize l1_distance to avoid UnboundLocalError
                
                object_type = self.config.get("object_type", "none")
                if object_type != "none":
                    if object_type == "gate":
                        # Gate: calculate robot <-> goal L1 distance
                        robot_pos_2d = robot_pos[:2]
                        l1_distance = np.sum(np.abs(robot_pos_2d - goal_pos_2d))
                        
                        # Gate is static, get position from config for CSV
                        gate_pos = np.array([0.0, 0.0, 0.0])
                        gate_quat = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion
                        
                        if "gate_pos" in self.config:
                            gate_pos_2d = self.config["gate_pos"]
                            gate_pos = np.array([gate_pos_2d[0], gate_pos_2d[1], 0.0])
                        # Get orientation from gate yaw
                        if "gate_yaw" in self.config:
                            gate_yaw = self.config["gate_yaw"]
                            half_yaw = gate_yaw / 2.0
                            gate_quat = np.array([
                                np.cos(half_yaw), 0.0, 0.0, np.sin(half_yaw)
                            ])
                        
                        # Gate as single "box" in list format
                        boxes_data.append({
                            'pos': gate_pos,
                            'quat': gate_quat,
                            'l1_distance': l1_distance
                        })
                    else:
                        # Dynamic objects (box, sphere): get all boxes
                        boxes_config = self.config.get("boxes_config", None)
                        num_boxes = self.config.get("num_boxes", 1)
                        
                        if boxes_config is not None and len(boxes_config) > 0:
                            # Multiple boxes: get each box's position and orientation
                            for box_idx, box_cfg in enumerate(boxes_config):
                                box_path = self._get_object_prim_path(box_idx)
                                box_prim = self.stage.GetPrimAtPath(box_path)
                                
                                if box_prim.IsValid():
                                    # Get world transform of the box
                                    box_xform = UsdGeom.Xformable(box_prim)
                                    box_transform = box_xform.ComputeLocalToWorldTransform(0)
                                    box_pos_world = box_transform.ExtractTranslation()
                                    box_pos = np.array([box_pos_world[0], box_pos_world[1], box_pos_world[2]])
                                    
                                    # Get box orientation (quaternion)
                                    box_rotation = box_transform.ExtractRotationQuat()
                                    box_quat = np.array([
                                        box_rotation.GetReal(),
                                        box_rotation.GetImaginary()[0],
                                        box_rotation.GetImaginary()[1],
                                        box_rotation.GetImaginary()[2]
                                    ])
                                    
                                    # Calculate L1 distance (Manhattan distance) between box and goal
                                    box_l1_distance = np.sum(np.abs(box_pos[:2] - goal_pos_2d))
                                    
                                    boxes_data.append({
                                        'pos': box_pos,
                                        'quat': box_quat,
                                        'l1_distance': box_l1_distance
                                    })
                                else:
                                    # Box not found, use default values
                                    boxes_data.append({
                                        'pos': np.array([0.0, 0.0, 0.0]),
                                        'quat': np.array([1.0, 0.0, 0.0, 0.0]),
                                        'l1_distance': 0.0
                                    })
                        else:
                            # Single box (backward compatibility)
                            object_path = self._get_object_prim_path(0)
                            object_prim = self.stage.GetPrimAtPath(object_path)
                            if object_prim.IsValid():
                                # Get world transform of the object
                                object_xform = UsdGeom.Xformable(object_prim)
                                object_transform = object_xform.ComputeLocalToWorldTransform(0)
                                object_pos_world = object_transform.ExtractTranslation()
                                object_pos = np.array([object_pos_world[0], object_pos_world[1], object_pos_world[2]])
                                
                                # Get object orientation (quaternion)
                                object_rotation = object_transform.ExtractRotationQuat()
                                object_quat = np.array([
                                    object_rotation.GetReal(),
                                    object_rotation.GetImaginary()[0],
                                    object_rotation.GetImaginary()[1],
                                    object_rotation.GetImaginary()[2]
                                ])
                                
                                # Calculate L1 distance (Manhattan distance) between object and goal
                                l1_distance = np.sum(np.abs(object_pos[:2] - goal_pos_2d))
                                
                                boxes_data.append({
                                    'pos': object_pos,
                                    'quat': object_quat,
                                    'l1_distance': l1_distance
                                })
                            else:
                                # No object found
                                boxes_data.append({
                                    'pos': np.array([0.0, 0.0, 0.0]),
                                    'quat': np.array([1.0, 0.0, 0.0, 0.0]),
                                    'l1_distance': 0.0
                                })
                else:
                    # No object: calculate robot <-> goal L1 distance
                    robot_pos_2d = robot_pos[:2]
                    l1_distance = np.sum(np.abs(robot_pos_2d - goal_pos_2d))
                    
                    # Add empty box entry for compatibility
                    boxes_data.append({
                        'pos': np.array([0.0, 0.0, 0.0]),
                        'quat': np.array([1.0, 0.0, 0.0, 0.0]),
                        'l1_distance': l1_distance
                    })
                
                # Initialize l1_distance if not already set (shouldn't happen, but safety check)
                if 'l1_distance' not in locals():
                    l1_distance = 0.0
                
                # Calculate aggregate metrics from all boxes
                # Ensure l1_distance is always set for logging
                if len(boxes_data) > 0:
                    all_box_distances = [box_data['l1_distance'] for box_data in boxes_data if box_data['l1_distance'] > 0]
                    if len(all_box_distances) > 0:
                        min_box_distance = min(all_box_distances)  # Closest box to goal
                        max_box_distance = max(all_box_distances)  # Farthest box from goal
                        avg_box_distance = np.mean(all_box_distances)  # Average distance
                        primary_l1_distance = min_box_distance  # Use minimum for task completion
                        # Set l1_distance for backward compatibility (use minimum distance)
                        l1_distance = min_box_distance
                    else:
                        # No valid box distances
                        primary_l1_distance = float('inf')
                        min_box_distance = max_box_distance = avg_box_distance = 0.0
                        l1_distance = 0.0
                else:
                    # No boxes, use robot distance (for gate/no object scenarios)
                    # l1_distance should already be set from gate/no object code above
                    # If it's still 0.0 and we have no object, calculate robot distance
                    if l1_distance == 0.0 and object_type == "none":
                        robot_pos_2d = robot_pos[:2]
                        l1_distance = np.sum(np.abs(robot_pos_2d - goal_pos_2d))
                    primary_l1_distance = l1_distance
                    min_box_distance = max_box_distance = avg_box_distance = l1_distance
                
                # Check if task is complete (closest box distance < 1m)
                if self.task_in_progress and primary_l1_distance < 1.0:
                    self.task_in_progress = False
                    self._change_goal_color_to_green()
                    if len(boxes_data) > 1:
                        self.logger.info(f"Task completed! Closest box distance to goal: {primary_l1_distance:.3f}m < 1.0m")
                        self.logger.info(f"  All box distances: min={min_box_distance:.3f}m, max={max_box_distance:.3f}m, avg={avg_box_distance:.3f}m")
                    else:
                        self.logger.info(f"Task completed! Distance to goal: {primary_l1_distance:.3f}m < 1.0m")
                    self.logger.info("Logging stopped")
                
                # Save experiment data (CSV and camera images) only after first command and while task is in progress
                if self.first_command_received and self.task_in_progress:
                    # Set experiment start time on first save
                    if not self.data_saving_started:
                        self.data_saving_started = True
                        self.experiment_start_time = datetime.now()
                        self.logger.info("Experiment data saving started")
                    
                    # For backward compatibility, also provide single object format
                    # Use closest box (minimum distance) for backward compatibility column
                    object_pos = boxes_data[0]['pos'] if len(boxes_data) > 0 else np.array([0.0, 0.0, 0.0])
                    object_quat = boxes_data[0]['quat'] if len(boxes_data) > 0 else np.array([1.0, 0.0, 0.0, 0.0])
                    # Use minimum distance (closest box) for backward compatibility
                    l1_distance = min_box_distance if len(boxes_data) > 0 else 0.0
                    
                    self._save_experiment_data(robot_pos, robot_quat, boxes_data, object_pos, object_quat, l1_distance)
                
                # Log at DEBUG level: detailed robot state
                self.logger.debug(
                    f"Robot pos: [{robot_pos[0]:.2f}, {robot_pos[1]:.2f}, {robot_pos[2]:.2f}], "
                    f"RPY: [{np.degrees(robot_rpy[0]):.2f}, {np.degrees(robot_rpy[1]):.2f}, {np.degrees(robot_rpy[2]):.2f}]°, "
                    f"Cmd vel: [vx={cmd_vel[0]:.2f}, vy={cmd_vel[1]:.2f}, yaw={cmd_vel[2]:.2f}]"
                )
                
                # Log at INFO level: L1 distance to goal (show metrics from all boxes)
                if object_type != "none":
                    if object_type == "gate":
                        self.logger.info(f"Robot <-> Goal L1 distance: {l1_distance:.2f} m")
                    else:
                        # Show metrics from all boxes
                        if len(boxes_data) > 1:
                            all_distances = [box_data['l1_distance'] for box_data in boxes_data]
                            self.logger.info(f"Boxes <-> Goal L1 distances: {[f'{d:.2f}' for d in all_distances]} m")
                            self.logger.info(f"  Aggregate: min={min_box_distance:.2f}m, max={max_box_distance:.2f}m, avg={avg_box_distance:.2f}m")
                        else:
                            # Single box - use its distance
                            single_box_distance = boxes_data[0]['l1_distance'] if len(boxes_data) > 0 else 0.0
                            self.logger.info(f"{object_type.capitalize()} <-> Goal L1 distance: {single_box_distance:.2f} m")
                else:
                    self.logger.info(f"Robot <-> Goal L1 distance: {l1_distance:.2f} m")
        
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
        
        # Initialize experiment directory structure and CSV file
        self._initialize_experiment_directory()
        
        # Create ego-view camera attached to robot base (after world reset, robot is fully initialized)
        self._setup_robot_camera()
        
        # Initialize camera render products (once, after cameras are set up)
        # This enables camera image capture for Pygame display
        self._initialize_camera_render_products()
        
        # Create callback function to update controller key state from pygame window
        def update_key_state(key, pressed):
            with self.controller._state_lock:
                if key in self.controller._key_state:
                    self.controller._key_state[key] = pressed
        
        # Initialize Pygame dual camera display
        window_width = 1600  # Total width for side-by-side display
        window_height = 800  # Height (matches ego camera aspect ratio)
        ego_res = self.config["ego_camera_resolution"]
        
        # Check if gate exists (will be set during gate creation)
        has_gate = self.config.get("object_type") == "gate"
        gate_transform_cb = self.apply_gate_transform if has_gate else None
        
        self.display = PygameDualCameraDisplay(
            window_size=(window_width, window_height),
            window_title="Spot Robot - Ego & Top View (i/j/k/l: x,y | u/o: yaw | ESC: quit)",
            key_state_callback=update_key_state,
            ego_camera_resolution=ego_res,
            gate_transform_callback=gate_transform_cb
        )
        self.display.start()
        
        # Register physics callback for robot control and logging
        self.world.add_physics_callback("physics_step", callback_fn=self._on_physics_step)
        
        # Note: We don't call controller.start() because we handle keyboard input in display window
        # The controller.update() is called in _on_physics_step callback
        
        # Automatically apply gate transform if gate exists (after all components are loaded)
        if self.config.get("object_type") == "gate" and "gate_pos" in self.config:
            self.logger.info("Automatically applying gate transform after all components loaded...")
            # Apply transform automatically
            self.apply_gate_transform()
        
        self.logger.info("Setup complete")
    
    def _log_performance_stats(self):
        """
        Log performance statistics (called at 1Hz).
        Calculates and logs average, min, max times for physics, rendering, and frames.
        """
        if not self.physics_step_times and not self.render_times and not self.frame_times:
            return  # No data yet
        
        stats = []
        
        # Physics step statistics
        if self.physics_step_times:
            physics_avg = np.mean(self.physics_step_times)
            physics_min = np.min(self.physics_step_times)
            physics_max = np.max(self.physics_step_times)
            physics_fps = 1000.0 / physics_avg if physics_avg > 0 else 0
            stats.append(f"Physics: {physics_avg:.3f}ms avg ({physics_min:.3f}-{physics_max:.3f}ms), {physics_fps:.1f} Hz")
        
        # Render step statistics
        if self.render_times:
            render_avg = np.mean(self.render_times)
            render_min = np.min(self.render_times)
            render_max = np.max(self.render_times)
            render_fps = 1000.0 / render_avg if render_avg > 0 else 0
            stats.append(f"Render: {render_avg:.3f}ms avg ({render_min:.3f}-{render_max:.3f}ms), {render_fps:.1f} FPS")
        
        # Overall frame statistics
        if self.frame_times:
            frame_avg = np.mean(self.frame_times)
            frame_min = np.min(self.frame_times)
            frame_max = np.max(self.frame_times)
            frame_fps = 1000.0 / frame_avg if frame_avg > 0 else 0
            stats.append(f"Frame: {frame_avg:.3f}ms avg ({frame_min:.3f}-{frame_max:.3f}ms), {frame_fps:.1f} FPS")
        
        if stats:
            self.logger.info(f"[Performance] {' | '.join(stats)}")

    def run(self):
        """
        Run main simulation loop.
        Steps physics and rendering until simulation is stopped or quit is requested.
        Updates Pygame display with camera frames.
        """
        if self.world is None or self.spot is None or self.controller is None:
            raise RuntimeError("Simulation must be setup first")
        
        self.logger.info("Starting simulation...")
        self.logger.info("Controls: i/k (x), j/l (y), u/o (yaw), ESC (quit)")
        self.logger.info("Camera views displayed in Pygame window (smooth 50+ FPS)")
        self.logger.info("Performance logging enabled (1Hz)")
        
        # Initialize frame timing
        self.last_frame_time = time.time()
        
        # Main simulation loop
        while simulation_app.is_running():
            # Track frame time
            frame_start_time = time.time()
            
            # Check if quit requested from keyboard controller or display window
            if self.controller.is_quit_requested():
                break
            
            # Check if Pygame window was closed
            if self.display and not self.display.running:
                self.logger.info("Pygame window closed - quitting...")
                break
            
            # Step physics and rendering (track render time)
            render_start_time = time.time()
            self.world.step(render=True)
            render_duration = (time.time() - render_start_time) * 1000  # Convert to ms
            self.render_times.append(render_duration)
            # Keep only last 500 samples
            if len(self.render_times) > 500:
                self.render_times.pop(0)
            
            # Track overall frame time
            frame_duration = (time.time() - frame_start_time) * 1000  # Convert to ms
            self.frame_times.append(frame_duration)
            if len(self.frame_times) > 500:
                self.frame_times.pop(0)
            
            # Update Pygame display with camera frames (every frame for smooth display)
            if self.display:
                ego_image = self.get_ego_camera_image()
                if ego_image is not None:
                    self.display.update_ego_frame(ego_image)
                
                top_image = self.get_top_camera_image()
                if top_image is not None:
                    self.display.update_top_frame(top_image)

    def cleanup(self):
        """
        Cleanup resources: stop controller, close CSV file, detach annotators, and remove physics callback.
        Should be called after simulation ends.
        """
        # Stop Pygame display
        if self.display:
            self.display.stop()
        
        # Mark controller as stopped (we didn't start a separate thread, but set quit flag)
        if self.controller:
            with self.controller._state_lock:
                self.controller._key_state['quit'] = True
        
        # Close CSV file (only if CSV logging was enabled)
        if self.enable_csv_logging and self.csv_file:
            self.csv_file.flush()
            self.csv_file.close()
            self.logger.info(f"CSV file closed with {self.frame_counter} frames saved")
        elif not self.enable_csv_logging:
            self.logger.info("CSV logging was disabled - no file to close")
        
        # Cleanup camera render products and annotators
        if hasattr(self, 'rgb_annotators') and hasattr(self, 'render_products'):
            try:
                for camera_type, annotator in self.rgb_annotators.items():
                    if camera_type in self.render_products:
                        annotator.detach([self.render_products[camera_type]])
                self.logger.info("Camera annotators detached")
            except Exception as e:
                self.logger.debug(f"Error cleaning up camera annotators: {e}")
        
        # Remove physics callback
        if self.world and self.world.physics_callback_exists("physics_step"):
            self.world.remove_physics_callback("physics_step")
        
        self.logger.info("Cleanup complete")
        
        # Close all logging handlers (flush and close terminal.log)
        for handler in self.logger.handlers[:]:
            handler.flush()
            handler.close()
            self.logger.removeHandler(handler)


# ===================== Main Entry Point =====================
def main():
    """
    Main entry point for the simulation.
    
    Command-Line Interface (CLI) Usage:
    
    Basic Usage:
        python quadruped_example.py
        python quadruped_example.py --object-type gate
        python quadruped_example.py --loglevel DEBUG
    
    Command-Line Arguments:
        --object-type {none,box,sphere,gate}
            Type of object to spawn (default: from config, usually "gate")
        
        --loglevel {DEBUG,INFO,WARNING,ERROR,CRITICAL}
            Logging level (default: "INFO")
        
        --no-csv-logging
            Disable CSV data logging (improves performance)
        
        --no-image-saving
            Disable camera image saving (improves performance)
    
    Usage Examples:
    
    1. Default configuration:
       python quadruped_example.py
    
    2. Gate navigation task with debug logging:
       python quadruped_example.py --object-type gate --loglevel DEBUG
    
    3. Box pushing task:
       python quadruped_example.py --object-type box
    
    4. High-performance mode (no logging/saving):
       python quadruped_example.py --no-csv-logging --no-image-saving
    
    5. Maximum performance with no object:
       python quadruped_example.py --object-type none --no-csv-logging --no-image-saving
    
    6. Combine multiple options:
       python quadruped_example.py --object-type gate --loglevel INFO --no-image-saving
    
    Python API Usage:
    
    1. Basic usage:
       sim = SpotSimulation(experiment_name="test")
       sim.setup()
       sim.run()
       sim.cleanup()
    
    2. Load from JSON file:
       sim = SpotSimulation(config_file="example_config.json", experiment_name="test")
    
    3. Override specific values:
       sim = SpotSimulation(
           experiment_name="gate_nav",
           randomize=True,
           map_size=12.0,
           object_type="gate",
           enable_csv_logging=True,
           enable_image_saving=True
       )
    
    4. Performance-optimized:
       sim = SpotSimulation(
           experiment_name="perf_test",
           enable_csv_logging=False,
           enable_image_saving=False
       )
    """
    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description="Isaac Sim - Spot Robot Remote Control Demo",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--object-type",
        type=str,
        choices=["none", "box", "sphere", "gate"],
        default=None,
        help="Type of object to spawn: 'none', 'box', 'sphere', or 'gate' (default: from config)"
    )
    parser.add_argument(
        "--loglevel",
        type=str,
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
        default="INFO",
        help="Logging level: DEBUG, INFO, WARNING, ERROR, or CRITICAL (default: INFO)"
    )
    parser.add_argument(
        "--no-csv-logging",
        action="store_true",
        help="Disable CSV logging (improves performance)"
    )
    parser.add_argument(
        "--no-image-saving",
        action="store_true",
        help="Disable camera image saving (improves performance)"
    )
    
    args = parser.parse_args()
    
    # Convert log level string to logging constant
    log_level_map = {
        "DEBUG": logging.DEBUG,
        "INFO": logging.INFO,
        "WARNING": logging.WARNING,
        "ERROR": logging.ERROR,
        "CRITICAL": logging.CRITICAL
    }
    log_level = log_level_map[args.loglevel.upper()]
    
    # Prepare config overrides
    config_overrides = {}
    if args.object_type is not None:
        config_overrides["object_type"] = args.object_type
    
    # Prompt for experiment name
    print("=" * 60)
    print("Isaac Sim - Spot Robot Remote Control Demo")
    print("=" * 60)
    if args.object_type:
        print(f"Object type: {args.object_type}")
    print(f"Log level: {args.loglevel}")
    if args.no_csv_logging:
        print("CSV logging: DISABLED (performance mode)")
    if args.no_image_saving:
        print("Image saving: DISABLED (performance mode)")
    print("=" * 60)
    experiment_name = input("Enter experiment name (press Enter for 'NULL'): ").strip()
    if not experiment_name:
        experiment_name = "NULL"
    print(f"Experiment name: {experiment_name}")
    print("=" * 60)
    
    # Create simulation instance with experiment name and config overrides
    # Can pass config_file="config.json" to load from file
    # Can pass keyword arguments to override config values
    sim = SpotSimulation(
        experiment_name=experiment_name, 
        log_level=log_level,
        enable_csv_logging=not args.no_csv_logging,
        enable_image_saving=not args.no_image_saving,
        **config_overrides
    )
    # sim = SpotSimulation(config_file="example_config.json", experiment_name=experiment_name, log_level=log_level)
    # sim = SpotSimulation(randomize=True, map_size=12.0, experiment_name=experiment_name, log_level=log_level)
    
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
