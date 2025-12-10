# launch/safety_bridge.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import sys
import os

# Importer la config
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'config'))
from safety_config import SafetyBridgeConfig, DevConfig, ProdConfig

def generate_launch_description():
    """
    Lance le Safety Bridge avec configuration centralisée
    
    Usage:
        ros2 launch turtlebot_game safety_bridge.launch.py
        ros2 launch turtlebot_game safety_bridge.launch.py robot_namespace:=robot2 ws_port:=8766
    """
    
    # Choisir la config (change selon besoin)
    config = SafetyBridgeConfig  # Ou DevConfig, ProdConfig
    
    # ===== ARGUMENTS DYNAMIQUES =====
    # Ces arguments peuvent override la config
    robot_namespace = LaunchConfiguration('robot_namespace')
    ws_host = LaunchConfiguration('ws_host')
    ws_port = LaunchConfiguration('ws_port')
    max_linear_vel = LaunchConfiguration('max_linear_vel')
    min_linear_vel = LaunchConfiguration('min_linear_vel')
    max_angular_vel = LaunchConfiguration('max_angular_vel')
    log_level = LaunchConfiguration('log_level')
    debug = LaunchConfiguration('debug')
    
    return LaunchDescription([
        # ===== DECLARE ARGUMENTS avec valeurs depuis config =====
        DeclareLaunchArgument(
            'robot_namespace', 
            default_value=config.ROBOT_NAMESPACE,
            description='Namespace du robot'
        ),
        DeclareLaunchArgument(
            'ws_host', 
            default_value=config.WS_HOST,
            description='Host WebSocket (0.0.0.0 pour toutes interfaces)'
        ),
        DeclareLaunchArgument(
            'ws_port', 
            default_value=str(config.WS_PORT),
            description='Port WebSocket'
        ),
        DeclareLaunchArgument(
            'max_linear_vel', 
            default_value=str(config.MAX_LINEAR_VEL),
            description='Vitesse linéaire max (m/s)'
        ),
        DeclareLaunchArgument(
            'min_linear_vel', 
            default_value=str(config.MIN_LINEAR_VEL),
            description='Vitesse linéaire min (m/s)'
        ),
        DeclareLaunchArgument(
            'max_angular_vel', 
            default_value=str(config.MAX_ANGULAR_VEL),
            description='Vitesse angulaire max (rad/s)'
        ),
        DeclareLaunchArgument(
            'log_level', 
            default_value=config.LOG_LEVEL,
            description='Niveau de log (DEBUG, INFO, WARN, ERROR)'
        ),
        DeclareLaunchArgument(
            'debug', 
            default_value=str(config.DEBUG).lower(),
            description='Mode debug'
        ),
        
        # ===== SAFETY BRIDGE NODE =====
        Node(
            package='turtlebot_game',
            executable='safety_bridge',
            namespace=robot_namespace,
            name='safety_bridge',
            output='screen',
            emulate_tty=True,
            
            # Remappings
            remappings=[
                ('cmd_vel', '/cmd_vel'),
            ],
            
            # Paramètres: mix config + launch args
            parameters=[{
                # Paramètres de base
                'robot_namespace': robot_namespace,
                'ws_host': ws_host,
                'ws_port': ws_port,
                'max_linear_vel': max_linear_vel,
                'min_linear_vel': min_linear_vel,
                'max_angular_vel': max_angular_vel,
                'log_level': log_level,
                
                # Paramètres depuis config (non-overridables par launch)
                'safety_distance': config.SAFETY_DISTANCE,
                'robot_safety_distance': config.ROBOT_SAFETY_DISTANCE,
                'robot_radius': config.ROBOT_RADIUS,
                'prediction_steps': config.PREDICTION_STEPS,
                'prediction_dt': config.PREDICTION_DT,
                'qos_depth': config.QOS_DEPTH,
                'ws_ping_interval': config.WS_PING_INTERVAL,
                'ws_ping_timeout': config.WS_PING_TIMEOUT,
                'status_broadcast_interval': config.STATUS_BROADCAST_INTERVAL,
                'cmd_timeout': config.CMD_TIMEOUT,
            }],
            
            # Arguments ROS2
            arguments=['--ros-args', '--log-level', log_level]
        ),
    ])