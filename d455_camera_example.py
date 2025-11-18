#!/usr/bin/env python3
"""
RealSense D455 Camera Data Capture Example

This example demonstrates how to capture and save data from the D455 camera
system on the Spot robot.

Usage:
    python d455_camera_example.py

Features:
- Captures RGB, depth, and IR images from D455 camera
- Saves images to disk periodically
- Displays camera information in console

Press ESC to quit.
"""

from isaacsim import SimulationApp

# Initialize Isaac Sim
simulation_app = SimulationApp({"headless": False})

# Import after SimulationApp
import numpy as np
import os
from datetime import datetime
from quadruped_example import SpotSimulation

# Create output directory for captured images
OUTPUT_DIR = "d455_captures"
os.makedirs(OUTPUT_DIR, exist_ok=True)


class D455CaptureDemo(SpotSimulation):
    """
    Extended SpotSimulation class with D455 camera data capture.
    """
    
    def __init__(self, **config_overrides):
        super().__init__(**config_overrides)
        self.capture_counter = 0
        self.capture_interval = 100  # Capture every 100 physics steps (~0.2 seconds)
        
    def setup(self):
        """Setup simulation with D455 sensor initialization enabled."""
        # Call parent setup
        super().setup()
        
        # Enable D455 sensor capture
        self._initialize_d455_sensors()
        self.logger.info(f"D455 sensors initialized. Images will be saved to: {OUTPUT_DIR}")
        
    def _on_physics_step(self, step_size):
        """
        Extended physics step callback with image capture.
        """
        # Call parent physics step
        super()._on_physics_step(step_size)
        
        # Capture images periodically
        self.capture_counter += 1
        if self.capture_counter >= self.capture_interval:
            self.capture_counter = 0
            self._capture_and_save_images()
    
    def _capture_and_save_images(self):
        """
        Capture current frame from all D455 cameras and save to disk.
        """
        try:
            # Get camera data
            data = self.get_d455_data()
            if data is None:
                return
            
            # Generate timestamp for filenames
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            
            # Save RGB image
            if 'rgb' in data and data['rgb'] is not None:
                rgb_data = data['rgb'].get('rgba', None)
                if rgb_data is not None:
                    # Save as numpy array (can be loaded with np.load)
                    rgb_path = os.path.join(OUTPUT_DIR, f"rgb_{timestamp}.npy")
                    np.save(rgb_path, rgb_data)
                    self.logger.info(f"Saved RGB image: {rgb_path}")
                    
                    # Optionally save as PNG using OpenCV (if available)
                    try:
                        import cv2
                        rgb_bgr = cv2.cvtColor(rgb_data[:, :, :3], cv2.COLOR_RGB2BGR)
                        png_path = os.path.join(OUTPUT_DIR, f"rgb_{timestamp}.png")
                        cv2.imwrite(png_path, rgb_bgr)
                    except ImportError:
                        pass  # OpenCV not available, skip PNG export
            
            # Save depth image
            if 'depth' in data and data['depth'] is not None:
                depth_data = data['depth'].get('distance_to_camera', None)
                if depth_data is not None:
                    depth_path = os.path.join(OUTPUT_DIR, f"depth_{timestamp}.npy")
                    np.save(depth_path, depth_data)
                    self.logger.info(f"Saved depth image: {depth_path}")
                    
                    # Get depth statistics
                    valid_depth = depth_data[np.isfinite(depth_data)]
                    if len(valid_depth) > 0:
                        min_depth = np.min(valid_depth)
                        max_depth = np.max(valid_depth)
                        mean_depth = np.mean(valid_depth)
                        self.logger.info(f"  Depth range: {min_depth:.2f}m to {max_depth:.2f}m (mean: {mean_depth:.2f}m)")
            
            # Save left IR image
            if 'left_ir' in data and data['left_ir'] is not None:
                left_ir_data = data['left_ir'].get('rgba', None)
                if left_ir_data is not None:
                    left_ir_path = os.path.join(OUTPUT_DIR, f"left_ir_{timestamp}.npy")
                    np.save(left_ir_path, left_ir_data)
            
            # Save right IR image
            if 'right_ir' in data and data['right_ir'] is not None:
                right_ir_data = data['right_ir'].get('rgba', None)
                if right_ir_data is not None:
                    right_ir_path = os.path.join(OUTPUT_DIR, f"right_ir_{timestamp}.npy")
                    np.save(right_ir_path, right_ir_data)
            
        except Exception as e:
            self.logger.warning(f"Failed to capture images: {e}")


def main():
    """Main entry point."""
    print("=" * 70)
    print("RealSense D455 Camera Data Capture Example")
    print("=" * 70)
    print(f"Output directory: {OUTPUT_DIR}")
    print("Images will be captured every ~0.2 seconds")
    print("Press ESC to quit")
    print("=" * 70)
    
    # Create simulation with D455 capture enabled
    sim = D455CaptureDemo(
        randomize=False,  # Disable randomization for reproducible captures
        use_object=True   # Include obstacle for depth testing
    )
    
    # Setup and run
    try:
        sim.setup()
        sim.run()
    finally:
        sim.cleanup()
        simulation_app.close()
        print(f"\n[INFO]: Simulation closed. Images saved to: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()

