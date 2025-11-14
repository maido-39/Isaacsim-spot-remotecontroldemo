# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

"""
Pygame-based keyboard controller for robot control
Provides acceleration/decay model for smooth velocity commands
"""

import numpy as np
import threading
import time
import pygame
from typing import Tuple, Dict, Optional


class KeyboardController:
    """
    General-purpose keyboard controller using Pygame
    
    Features:
    - Runs in separate thread to not block main simulation
    - Acceleration/decay model for smooth control
    - Configurable velocity limits and dynamics
    - Thread-safe state sharing
    
    Key mapping:
    - i/k: forward/backward (x-axis)
    - j/l: strafe left/right (y-axis)
    - u/o: turn left/right (yaw)
    - ESC: quit
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
        update_dt: float = 0.02,  # 50Hz update
        eps_linear: float = 0.001,
        eps_angular: float = 0.001,
        window_size: Tuple[int, int] = (420, 320),
        window_title: str = "Robot Control (i/j/k/l: x,y | u/o: yaw | ESC: quit)"
    ):
        """
        Initialize keyboard controller
        
        Args:
            max_vx: Maximum forward/backward velocity (m/s)
            max_vy: Maximum lateral velocity (m/s)
            max_yaw: Maximum rotation velocity (rad/s)
            acc_vx: x-axis acceleration (m/s^2)
            acc_vy: y-axis acceleration (m/s^2)
            acc_yaw: yaw acceleration (rad/s^2)
            decay_vx: x-axis decay coefficient (0~1, smaller = faster decay)
            decay_vy: y-axis decay coefficient (0~1)
            decay_yaw: yaw decay coefficient (0~1)
            update_dt: Command update period (seconds)
            eps_linear: Linear velocity threshold (small values → 0)
            eps_angular: Angular velocity threshold
            window_size: Pygame window size (width, height)
            window_title: Window title
        """
        # Store parameters
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
        self.window_size = window_size
        self.window_title = window_title
        
        # Shared key state (between pygame thread and main loop)
        self._key_state = {
            'x_pos': False,   # forward (+x)
            'x_neg': False,   # backward (-x)
            'y_pos': False,   # strafe left (+y)
            'y_neg': False,   # strafe right (-y)
            'yaw_pos': False, # turn left (+yaw)
            'yaw_neg': False, # turn right (-yaw)
            'quit': False
        }
        self._state_lock = threading.Lock()
        
        # Accumulated command state
        self._vx_cmd = 0.0
        self._vy_cmd = 0.0
        self._yaw_cmd = 0.0
        
        # Thread management
        self._keyboard_thread: Optional[threading.Thread] = None
        self._running = False
        
    def start(self) -> bool:
        """
        Start keyboard controller (runs pygame event loop in separate thread)
        
        Returns:
            bool: Success status
        """
        if self._running:
            print("[WARNING] Controller already running")
            return False
            
        self._running = True
        self._keyboard_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._keyboard_thread.start()
        time.sleep(0.1)  # Wait for pygame initialization
        return True
        
    def stop(self):
        """Stop the controller"""
        with self._state_lock:
            self._key_state['quit'] = True
        self._running = False
        
        if self._keyboard_thread and self._keyboard_thread.is_alive():
            self._keyboard_thread.join(timeout=1.0)
            
    def reset(self):
        """Reset command state (velocities to zero)"""
        self._vx_cmd = 0.0
        self._vy_cmd = 0.0
        self._yaw_cmd = 0.0
        
    def get_command(self) -> np.ndarray:
        """
        Get current command
        
        Returns:
            np.ndarray: [vx, vy, yaw] command array
        """
        return np.array([self._vx_cmd, self._vy_cmd, self._yaw_cmd])
        
    def update(self):
        """
        Update commands based on keyboard input (apply acceleration/decay)
        Should be called periodically from main loop
        """
        with self._state_lock:
            # Digital input → -1/0/+1 intensity
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
        
        # Clamp speeds to max limits
        self._vx_cmd = max(-self.max_vx, min(self.max_vx, self._vx_cmd))
        self._vy_cmd = max(-self.max_vy, min(self.max_vy, self._vy_cmd))
        self._yaw_cmd = max(-self.max_yaw, min(self.max_yaw, self._yaw_cmd))
        
        # Zero out small values (prevent jitter)
        if abs(self._vx_cmd) < self.eps_linear:
            self._vx_cmd = 0.0
        if abs(self._vy_cmd) < self.eps_linear:
            self._vy_cmd = 0.0
        if abs(self._yaw_cmd) < self.eps_angular:
            self._yaw_cmd = 0.0
            
    def is_quit_requested(self) -> bool:
        """
        Check if quit is requested
        
        Returns:
            bool: True if quit requested
        """
        with self._state_lock:
            return self._key_state['quit']
            
    def get_input_state(self) -> Dict[str, bool]:
        """
        Get current input state
        
        Returns:
            Dict[str, bool]: Copy of key state dictionary
        """
        with self._state_lock:
            return self._key_state.copy()
            
    def get_velocities(self) -> Tuple[float, float, float]:
        """
        Get current velocity commands
        
        Returns:
            Tuple[float, float, float]: (vx, vy, yaw_rate)
        """
        return (self._vx_cmd, self._vy_cmd, self._yaw_cmd)
        
    def _keyboard_loop(self):
        """
        Keyboard input loop using pygame (private method)
        Runs in separate thread
        """
        try:
            pygame.init()
            screen = pygame.display.set_mode(self.window_size)
            pygame.display.set_caption(self.window_title)
            clock = pygame.time.Clock()
            print("[INFO] Pygame control window opened")
        except Exception as e:
            print(f"[ERROR] Failed to initialize pygame: {e}")
            with self._state_lock:
                self._key_state['quit'] = True
            return
        
        while self._running and not self._key_state['quit']:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    with self._state_lock:
                        self._key_state['quit'] = True
                
                elif event.type == pygame.KEYDOWN:
                    with self._state_lock:
                        if event.key == pygame.K_i:
                            self._key_state['x_pos'] = True
                        elif event.key == pygame.K_k:
                            self._key_state['x_neg'] = True
                        elif event.key == pygame.K_j:
                            self._key_state['y_pos'] = True
                        elif event.key == pygame.K_l:
                            self._key_state['y_neg'] = True
                        elif event.key == pygame.K_u:
                            self._key_state['yaw_pos'] = True
                        elif event.key == pygame.K_o:
                            self._key_state['yaw_neg'] = True
                        elif event.key == pygame.K_ESCAPE:
                            self._key_state['quit'] = True
                            print("[INFO] Quitting...")
                
                elif event.type == pygame.KEYUP:
                    with self._state_lock:
                        if event.key == pygame.K_i:
                            self._key_state['x_pos'] = False
                        elif event.key == pygame.K_k:
                            self._key_state['x_neg'] = False
                        elif event.key == pygame.K_j:
                            self._key_state['y_pos'] = False
                        elif event.key == pygame.K_l:
                            self._key_state['y_neg'] = False
                        elif event.key == pygame.K_u:
                            self._key_state['yaw_pos'] = False
                        elif event.key == pygame.K_o:
                            self._key_state['yaw_neg'] = False
            
            clock.tick(120)  # High polling rate for responsiveness
        
        pygame.quit()

