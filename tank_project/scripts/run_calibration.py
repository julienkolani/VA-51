#!/usr/bin/env python3
"""
Calibration Wizard Runner

Interactive calibration process to set up the arena:
1. Safe zone definition
2. Geometric calibration (projected corners)
3. Metric calibration (physical marker)
4. Obstacle mapping

Saves results to config/arena.yaml

Usage:
    python3 run_calibration.py
    
Prerequisites:
    - RealSense camera connected
    - Projector showing Pygame window with projected ArUco corners
"""

import sys
import yaml
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from perception.camera.realsense_stream import RealSenseStream
from perception.calibration.calibration_wizard import CalibrationWizard


def main():
    print("[CALIB_RUNNER] ========== Calibration Wizard ==========")
    
    # Load configuration
    print("[CALIB_RUNNER] Loading configuration...")
    config_dir = Path(__file__).parent.parent / 'config'
    
    with open(config_dir / 'camera.yaml') as f:
        camera_config = yaml.safe_load(f)
    
    with open(config_dir / 'arena.yaml') as f:
        arena_config = yaml.safe_load(f)
    
    # Initialize camera with config
    print("[CALIB_RUNNER] Initializing camera...")
    camera = RealSenseStream(
        width=camera_config['realsense']['width'],
        height=camera_config['realsense']['height'],
        fps=camera_config['realsense']['fps']
    )
    camera.start()
    
    # Run wizard with config
    wizard = CalibrationWizard(
        camera, 
        projector_width=arena_config['projector']['width'],
        projector_height=arena_config['projector']['height']
    )
    
    try:
        print("[CALIB_RUNNER] Running calibration wizard...")
        results = wizard.run()
        print("[CALIB_RUNNER] Calibration wizard completed!")
        
        # Save to config
        config_path = Path(__file__).parent.parent / 'config' / 'arena.yaml'
        
        print("[CALIB_RUNNER] Saving calibration to {}".format(config_path))
        
        with open(config_path, 'w') as f:
            yaml.dump(results, f, default_flow_style=False)
        
        print("[CALIB_RUNNER] Calibration saved successfully!")
        print("[CALIB_RUNNER] You can now run the game with: python3 run_game.py")
        
    except Exception as e:
        print("[CALIB_RUNNER] ERROR: {}".format(e))
        
    finally:
        camera.stop()
        print("[CALIB_RUNNER] Done")


if __name__ == '__main__':
    main()
