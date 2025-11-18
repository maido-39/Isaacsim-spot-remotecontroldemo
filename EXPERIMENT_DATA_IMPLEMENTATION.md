# Experiment Data Saving Implementation Summary

## Overview

This document summarizes the implementation of the experiment data saving system for the Spot Robot Isaac Sim simulation.

## Implementation Date
November 18, 2024

## Features Implemented

### 1. User Input for Experiment Name
- Prompts user at startup for experiment name
- Defaults to "NULL" if no input provided
- Stored in simulation configuration

### 2. Automatic Directory Structure Creation
```
YYMMDD_HHMMSS-Experiment_Name/
├── config.json
├── data.csv
└── camera/
    ├── ego/
    └── top/
```

### 3. Configuration Saving (config.json)
- Saves actual experiment configuration values
- Excludes randomization range parameters
- JSON format for easy parsing
- Includes all physics, environment, and controller parameters

### 4. Time-Series Data Logging (data.csv)
- **Rate:** 10Hz (every 0.1 seconds)
- **Columns:**
  - timestamp (elapsed seconds)
  - frame_num (incremental counter)
  - robot_pos_x, robot_pos_y, robot_pos_z (meters)
  - robot_orient_w, robot_orient_x, robot_orient_y, robot_orient_z (quaternion)
  - object_pos_x, object_pos_y, object_pos_z (meters)
  - object_orient_w, object_orient_x, object_orient_y, object_orient_z (quaternion)

### 5. Camera Image Capture
- **Cameras:** Ego-view and top-view
- **Rate:** 10Hz (synchronized with CSV data)
- **Format:** JPEG (800x600, 95% quality)
- **Naming:** `frame{N}-{timestamp}-{camera_type}.jpg`
- **Method:** Isaac Sim Replicator API with fallback

## Code Changes

### Modified Files

#### 1. `quadruped_example.py`

**Imports Added:**
```python
import os
import csv
from datetime import datetime
from pathlib import Path
```

**New Class Variables:**
- `experiment_name`: Name of experiment
- `experiment_dir`: Path to experiment directory
- `csv_file`: CSV file handle
- `csv_writer`: CSV writer object
- `frame_counter`: Frame counter for logging
- `experiment_start_time`: Experiment start timestamp

**New Methods:**
- `_initialize_experiment_directory()`: Creates directory structure and CSV file
- `_save_config()`: Saves configuration to JSON
- `_save_camera_image()`: Captures and saves camera images using Replicator
- `_save_camera_image_fallback()`: Fallback image capture method
- `_save_experiment_data()`: Main data saving coordinator

**Modified Methods:**
- `__init__()`: Added experiment_name parameter and data tracking variables
- `_on_physics_step()`: Added data saving at 10Hz rate
- `setup()`: Added experiment directory initialization
- `cleanup()`: Added CSV file closing
- `main()`: Added experiment name input prompt

### New Files

#### 1. `EXPERIMENT_DATA_GUIDE.md`
Comprehensive user guide covering:
- Quick start instructions
- Folder structure explanation
- Data file formats
- Usage examples
- CSV analysis examples
- Troubleshooting guide

#### 2. `analyze_experiment_data.py`
Python script for analyzing saved experiment data:
- Loads configuration and CSV data
- Calculates derived features (speed, distances, Euler angles)
- Prints summary statistics
- Generates trajectory plots
- Generates time-series plots
- Saves analysis plots as PNG files

#### 3. `EXPERIMENT_DATA_IMPLEMENTATION.md`
This file - implementation summary and technical details

## Technical Details

### Data Collection Flow

1. **Initialization** (during `setup()`):
   - Create timestamp string (YYMMDD_HHMMSS)
   - Create experiment directory
   - Create camera subdirectories
   - Save config.json
   - Initialize CSV file with header
   - Store experiment start time

2. **During Simulation** (in `_on_physics_step()` at 10Hz):
   - Get robot pose (position + quaternion)
   - Get object pose (position + quaternion)
   - Write CSV row
   - Capture ego-view camera image
   - Capture top-view camera image
   - Increment frame counter
   - Flush CSV buffer every 10 frames

3. **Cleanup** (in `cleanup()`):
   - Flush CSV buffer
   - Close CSV file
   - Log total frames saved

### Quaternion Convention

Uses [w, x, y, z] convention:
- w: Real component
- x, y, z: Imaginary components

### Camera Image Capture

Primary method uses Isaac Sim Replicator:
```python
render_product = rep.create.render_product(camera_path, (800, 600))
rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
rgb_annotator.attach([render_product])
rgb_data = rgb_annotator.get_data()
```

Fallback uses viewport switching (less reliable).

### CSV Buffering

- Buffer flushed every 10 frames (1 second at 10Hz)
- Final flush on cleanup
- Ensures data integrity even if simulation crashes

## Usage Instructions

### Running an Experiment

```bash
cd /home/syaro/MikuchanRemote/Remotecontrol-Demo/Isaacsim-spot-remotecontroldemo
python quadruped_example.py
```

When prompted:
```
Enter experiment name (press Enter for 'NULL'): my_test_run
```

### Analyzing Results

```bash
python analyze_experiment_data.py 241118_153045-my_test_run
```

This will:
- Print summary statistics
- Generate trajectory plot
- Generate time-series plots
- Save plots to experiment directory

## Dependencies

### Required Python Packages
- numpy (already present)
- pandas (for analysis script)
- matplotlib (for analysis script)
- PIL/Pillow (for image saving)

### Isaac Sim Modules
- omni.replicator.core (for camera capture)
- omni.kit.viewport.utility (fallback camera capture)
- Standard Isaac Sim modules (already in use)

## Data Storage Estimates

For a 60-second experiment at 10Hz:

### CSV Data
- ~600 rows × 16 columns
- ~50KB per minute
- ~3MB per hour

### Camera Images
- ~600 frames × 2 cameras
- ~50KB per image (JPEG 800x600)
- ~60MB per minute (both cameras)
- ~3.6GB per hour

### Total
- ~60MB per minute
- ~3.6GB per hour

## Known Limitations

1. **Camera Capture:** 
   - Depends on Isaac Sim Replicator availability
   - May fail silently if Replicator not available
   - Fallback method is less reliable

2. **Performance:**
   - Image capture may reduce simulation speed
   - High-frequency capture (>10Hz) may cause lag

3. **Data Loss:**
   - CSV buffer flushed every 10 frames
   - If simulation crashes, up to 1 second of data may be lost

4. **Storage:**
   - Long experiments generate large amounts of data
   - No automatic cleanup or compression

## Future Enhancements

### Potential Improvements

1. **Performance:**
   - Async image saving
   - Video encoding instead of individual frames
   - Configurable capture rates
   - Option to disable camera capture

2. **Data Management:**
   - Automatic compression
   - Cloud backup integration
   - Data streaming support

3. **Analysis:**
   - Real-time visualization
   - Automatic report generation
   - Multi-experiment comparison

4. **Robustness:**
   - Better error handling for camera capture
   - Automatic recovery from failures
   - Data validation and integrity checks

## Testing Recommendations

1. **Basic Functionality:**
   - Run short experiment (10 seconds)
   - Verify directory creation
   - Check config.json content
   - Verify CSV has correct columns and data
   - Check camera folders for images

2. **Long Running:**
   - Run 5-minute experiment
   - Monitor disk space
   - Verify no memory leaks
   - Check data continuity

3. **Error Handling:**
   - Test with read-only directory (should fail gracefully)
   - Test with insufficient disk space
   - Test camera capture failure handling

## Code Review Checklist

- [x] User input validation
- [x] Directory creation error handling
- [x] CSV file proper opening/closing
- [x] Data synchronization (CSV + images)
- [x] Proper cleanup on exit
- [x] Configuration filtering (remove ranges)
- [x] Quaternion data format consistency
- [x] Timestamp calculation
- [x] Frame numbering
- [x] Documentation
- [x] Example analysis script

## Backward Compatibility

The changes are fully backward compatible:
- Experiment name is optional (defaults to "NULL")
- All existing functionality preserved
- No changes to existing API
- Data saving can be disabled by modifying code if needed

## Contact/Maintenance

For questions or issues:
1. Check EXPERIMENT_DATA_GUIDE.md for usage
2. Review this implementation summary
3. Check console logs for error messages
4. Verify Isaac Sim version compatibility

## Version History

### v1.0 (2024-11-18)
- Initial implementation
- Basic CSV and image saving
- Configuration export
- Analysis script
- Documentation

