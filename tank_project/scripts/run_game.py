#!/usr/bin/env python3
"""
Main Game Loop

Runs the tank arena game at 30 FPS:
1. Vision and tracking (ArUco detection, Kalman filtering)
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
import pygame
import numpy as np
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from perception.camera.realsense_stream import RealSenseStream
from perception.camera.aruco_detector import ArucoDetector
from perception.camera.kalman_filter import KalmanFilter
from perception.camera.homography import apply_homography_single
from core.world.world_model import WorldModel
from core.world.coordinate_frames import TransformManager
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
        with open(config_dir / '{}.yaml'.format(config_file)) as f:
            configs[config_file] = yaml.safe_load(f)
    
    return configs


def setup_transform_manager(configs):
    """
    Setup coordinate transform from calibration data.
    
    Returns:
        TransformManager with H_C2W set, or None if not calibrated
    """
    transform_mgr = TransformManager()
    
    # Load H_C2W from calibration if available
    transform_data = configs['arena'].get('transform', {})
    scale = transform_data.get('scale_m_per_av', 1.0)
    h_c2w = transform_data.get('H_C2W')
    
    if h_c2w is not None:
        transform_mgr.H_C2W = np.array(h_c2w)
        print("[MAIN] Loaded H_C2W from calibration")
    else:
        # Fallback: use simple scaling (assumes camera aligned with arena)
        transform_mgr.set_av_to_world_scale(scale)
        print("[MAIN] WARNING: No H_C2W found, using fallback scaling")
    
    return transform_mgr


def camera_to_world(detection_center, transform_mgr, fallback_scale=1.0):
    """
    Transform ArUco detection from camera pixels to world meters.
    
    Args:
        detection_center: (u, v) pixel coordinates
        transform_mgr: TransformManager instance
        fallback_scale: Scale to use if no homography
        
    Returns:
        (x, y) in meters
    """
    u, v = detection_center
    
    if transform_mgr.H_C2W is not None:
        return transform_mgr.camera_to_world(u, v)
    else:
        # Fallback: simple scaling from center
        # This is approximate and should be replaced with proper calibration
        x = u * fallback_scale / 1000.0
        y = v * fallback_scale / 1000.0
        return (x, y)


def main():
    print("[MAIN] ========== Tank Arena Game ==========")
    
    # Load configuration
    print("[MAIN] Loading configuration...")
    configs = load_config()
    
    # Setup coordinate transforms
    transform_mgr = setup_transform_manager(configs)
    
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
    
    # 5. Control - get velocity limits from correct config path
    velocity_limits = configs['robot'].get('velocity_limits', {})
    max_linear_vel = velocity_limits.get('max_linear_mps', 0.22)
    max_angular_vel = velocity_limits.get('max_angular_radps', 2.84)
    
    controller = TrajectoryFollower(configs['robot'].get('control', {}))
    ros_bridge = ROSBridgeClient(
        host=configs['robot']['ros_bridge']['host'],
        port=configs['robot']['ros_bridge']['port']
    )
    ros_bridge.connect()
    
    # 6. Visualization
    display_config = configs['arena'].get('display', {})
    renderer = PygameRenderer(
        width=configs['arena']['projector']['width'],
        height=configs['arena']['projector']['height'],
        margin=configs['arena']['projector']['margin_px'],
        fullscreen=display_config.get('fullscreen', False),
        display_index=display_config.get('display_index', 0)
    )
    renderer.set_arena_dimensions(
        configs['arena']['arena']['width_m'],
        configs['arena']['arena']['height_m']
    )
    
    print("[MAIN] All subsystems initialized")
    print("[MAIN] Starting game loop at 30 FPS...")
    
    # Initialize Input (Joystick/Keyboard)
    joystick = None
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print("[MAIN] Joystick detected: {}".format(joystick.get_name()))
    
    # Game loop
    dt = 1.0 / configs['game']['match']['tick_rate_fps']
    running = True
    
    # Fallback scale from config
    fallback_scale = configs['arena'].get('transform', {}).get('scale_m_per_av', 1.0)
    
    try:
        while running:
            tick_start = time.time()
            
            # --- INPUT HANDLING ---
            human_input = {
                'v': 0.0,
                'omega': 0.0,
                'fire_request': False,
                'start_game': False
            }
            
            # Process Pygame Events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        human_input['fire_request'] = True
                    elif event.key == pygame.K_RETURN:
                        human_input['start_game'] = True
                    elif event.key == pygame.K_d:
                        renderer.handle_keypress(event)  # Toggle debug
                        
            # Joystick Input (PS4/Xbox)
            if joystick:
                # Axis 1: Left Stick Y (Linear V)
                # Axis 3: Right Stick X (Angular Omega)
                v = -joystick.get_axis(1) * max_linear_vel
                omega = -joystick.get_axis(3) * max_angular_vel
                
                if abs(v) > 0.05:
                    human_input['v'] = v
                if abs(omega) > 0.05:
                    human_input['omega'] = omega
                
                if joystick.get_button(5) or joystick.get_button(0):  # R1 or X/A
                    human_input['fire_request'] = True
                if joystick.get_button(9):  # Options/Start
                    human_input['start_game'] = True

            # Keyboard fallback for movement (Arrow keys)
            keys = pygame.key.get_pressed()
            if keys[pygame.K_UP]:
                human_input['v'] = max_linear_vel
            if keys[pygame.K_DOWN]:
                human_input['v'] = -max_linear_vel
            if keys[pygame.K_LEFT]:
                human_input['omega'] = max_angular_vel
            if keys[pygame.K_RIGHT]:
                human_input['omega'] = -max_angular_vel

            
            # 1. Vision
            color_frame, _ = camera.get_frames()
            if color_frame is None:
                continue
                
            detections = aruco.detect(color_frame)
            
            # 2. Update robot poses with proper coordinate transform
            if 4 in detections:  # AI robot
                d = detections[4]
                # Transform from camera pixels to world meters
                x, y = camera_to_world(d['center'], transform_mgr, fallback_scale)
                theta = d['orientation']
                
                # Update Kalman filter
                kalman_ai.predict()
                kalman_ai.update((x, y, theta))
                
                # Get filtered pose and update world
                filtered_pose = kalman_ai.get_pose()
                world.update_robot_pose(4, filtered_pose)
                 
            if 5 in detections:  # Human robot
                d = detections[5]
                x, y = camera_to_world(d['center'], transform_mgr, fallback_scale)
                theta = d['orientation']
                
                kalman_human.predict()
                kalman_human.update((x, y, theta))
                
                filtered_pose = kalman_human.get_pose()
                world.update_robot_pose(5, filtered_pose)
            
            # 3. Update world occupancy
            world.update_occupancy()
            
            # 4. AI Decision
            # Construct AI Context
            ai_context = world.get_state_dict()
            ai_context['ai_pose'] = world.get_robot_pose(4)
            ai_context['human_pose'] = world.get_robot_pose(5)
            ai_context['raycast_sys'] = game_engine.raycast  # Pass physics engine to AI
            ai_context['game_time'] = time.time()
            
            ai_decision = ai_strategy.decide(ai_context)
            
            # 5. Game tick
            game_state = game_engine.tick(world, ai_decision, human_input)
            
            # 6. Control
            # AI Control
            if ai_decision.get('target_position'):
                # Use Planner Path
                path = ai_strategy.current_path
                v_ai, omega_ai = controller.compute_control(world.get_robot_pose(4), path)
                
                # Send to ROS
                ros_bridge.send_velocity_command(4, v_ai, omega_ai)
                
                # Visualization of Path
                renderer.set_ai_path(path)
            else:
                ros_bridge.send_velocity_command(4, 0, 0)  # Stop
                 
            # Human Control (for RC mode)
            if human_input['v'] != 0 or human_input['omega'] != 0:
                ros_bridge.send_velocity_command(5, human_input['v'], human_input['omega'])

            
            # 7. Render
            renderer.render_frame(world.get_state_dict(), game_state)
            
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
