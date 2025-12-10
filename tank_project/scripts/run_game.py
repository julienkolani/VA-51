#!/usr/bin/env python3
"""
Main Game Loop

Runs the tank arena game at 30 FPS:
1. Vision & tracking (ArUco detection, Kalman filtering)
2. World update (occupancy grid, robot poses)
3. Game engine tick (shots, hits, timers)
4. AI decision (behavior tree, path planning)
5. Control (trajectory following, ROS commands)
6. Visualization (Pygame rendering)

Usage:
    python3 run_game.py
    
Prerequisites:
    - Calibration completed (config/arena.yaml exists)
    - ROS bridge running
    - RealSense camera connected
    - Projector configured
"""

import sys
import time
import yaml
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from perception.camera.realsense_stream import RealSenseStream
from perception.camera.aruco_detector import ArucoDetector
from perception.camera.kalman_filter import KalmanFilter
from core.world.world_model import WorldModel
from core.game.game_engine import GameEngine
from core.ia.strategy import AIStrategy
from core.control.trajectory_follower import TrajectoryFollower
from core.control.ros_bridge_client import ROSBridgeClient
from visualization.pygame_renderer import PygameRenderer


def load_config():
    """Load all configuration files."""
    config_dir = Path(__file__).parent.parent / 'config'
    
    configs = {}
    for config_file in ['arena', 'camera', 'game', 'ia', 'robot']:
        with open(config_dir / f'{config_file}.yaml') as f:
            configs[config_file] = yaml.safe_load(f)
    
    return configs


def main():
    print("[MAIN] ========== Tank Arena Game ==========")
    
    # Load configuration
    print("[MAIN] Loading configuration...")
    configs = load_config()
    
    # Initialize subsystems
    print("[MAIN] Initializing subsystems...")
    
    # 1. Vision
    camera = RealSenseStream(
        width=configs['camera']['realsense']['width'],
        height=configs['camera']['realsense']['height'],
        fps=configs['camera']['realsense']['fps']
    )
    camera.start()
    
    aruco = ArucoDetector()
    kalman_ai = KalmanFilter()
    kalman_human = KalmanFilter()
    
    # 2. World
    world = WorldModel(
        arena_width_m=configs['arena']['arena']['width_m'],
        arena_height_m=configs['arena']['arena']['height_m'],
        grid_resolution_m=configs['arena']['grid']['resolution_m']
    )
    
    # 3. Game
    game_engine = GameEngine(configs['game'])
    
    # 4. IA
    ai_strategy = AIStrategy(configs['ia'])
    ai_strategy.set_planner(world.grid)
    
    # 5. Control
    controller = TrajectoryFollower(configs['robot'])
    ros_bridge = ROSBridgeClient(
        host=configs['robot']['ros_bridge']['host'],
        port=configs['robot']['ros_bridge']['port']
    )
    ros_bridge.connect()
    
    # 6. Visualization
    renderer = PygameRenderer(
        width=configs['arena']['projector']['width'],
        height=configs['arena']['projector']['height'],
        margin=configs['arena']['projector']['margin_px'],
        fullscreen=configs['arena']['display']['fullscreen'],
        display_index=configs['arena']['display']['display_index']
    )
    renderer.set_arena_dimensions(
        configs['arena']['arena']['width_m'],
        configs['arena']['arena']['height_m']
    )
    
    print("[MAIN] All subsystems initialized")
    print("[MAIN] Starting game loop at 30 FPS...")
    
    # Game loop
    dt = 1.0 / configs['game']['match']['tick_rate_fps']
    running = True
    
    try:
        while running:
            tick_start = time.time()
            
            # 1. Vision
            color_frame, _ = camera.get_frames()
            if color_frame is None:
                continue
                
            detections = aruco.detect(color_frame)
            
            # 2. Update robot poses
            if 4 in detections:  # AI robot
                # Extract pose from ArUco
                # (Simplified - full version would use transform from calibration)
                pass
                
            if 5 in detections:  # Human robot
                pass
            
            # 3. Update world
            world.update_occupancy()
            
            # 4. Game tick
            # game_state = game_engine.tick(...)
            
            # 5. AI decision
            # ai_decision = ai_strategy.decide(world.get_state_dict())
            
            # 6. Control
            # commands = controller.compute_control(...)
            # ros_bridge.send_velocity_command(4, v, omega)
            
            # 7. Render
            renderer.render_frame(world.get_state_dict(), {})
            
            # Maintain frame rate
            elapsed = time.time() - tick_start
            if elapsed < dt:
                time.sleep(dt - elapsed)
                
    except KeyboardInterrupt:
        print("\n[MAIN] Shutting down...")
        
    finally:
        # Cleanup
        camera.stop()
        ros_bridge.disconnect()
        renderer.cleanup()
        print("[MAIN] Goodbye!")


if __name__ == '__main__':
    main()
