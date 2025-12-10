"""
ROS Bridge Client - Communication with ROS System

WebSocket client for sending commands to ROS bridge:
- Connects to ROS bridge server
- Sends velocity commands to /cmd_vel
- Receives odometry feedback (optional)
- Maintains connection health

The ROS bridge runs on the robot and translates between
WebSocket JSON and ROS messages.

Logs: [ROS] Command sent: v=X, ω=Y
"""

import socket
import json
import time
from typing import Tuple, Optional


class ROSBridgeClient:
    """
    Client for communicating with ROS bridge via WebSocket/TCP.
    
    Sends velocity commands to physical robot.
    """
    
    def __init__(self, host: str = 'localhost', port: int = 9090):
        """
        Initialize ROS bridge client.
        
        Args:
            host: ROS bridge server host
            port: ROS bridge server port
        """
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        
    def connect(self):
        """
        Establish connection to ROS bridge.
        
        Logs:
            [ROS] Connected to bridge at host:port
            [ROS] Connection failed: error
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            self.connected = True
            print(f"[ROS] Connected to bridge at {self.host}:{self.port}")
        except Exception as e:
            self.connected = False
            print(f"[ROS] Connection failed: {e}")
    
    def disconnect(self):
        """Close connection to ROS bridge."""
        if self.socket:
            self.socket.close()
            self.connected = False
            print("[ROS] Disconnected from bridge")
    
    def send_velocity_command(self, 
                             robot_id: int, 
                             v: float, 
                             omega: float) -> bool:
        """
        Send velocity command to robot.
        
        Args:
            robot_id: 4 (AI) or 5 (Human) - only 4 is actually controlled
            v: Linear velocity in m/s
            omega: Angular velocity in rad/s
            
        Returns:
            True if sent successfully
            
        Message format (JSON):
            {
                "robot_id": 4,
                "linear": v,
                "angular": omega,
                "timestamp": unix_time
            }
            
        Logs:
            [ROS] Robot4 cmd: v=0.15 m/s, ω=-0.30 rad/s
        """
        if not self.connected:
            print("[ROS] Not connected, attempting reconnect...")
            self.connect()
            if not self.connected:
                return False
        
        message = {
            "robot_id": robot_id,
            "linear": round(v, 3),
            "angular": round(omega, 3),
            "timestamp": time.time()
        }
        
        try:
            msg_json = json.dumps(message) + "\n"
            self.socket.sendall(msg_json.encode('utf-8'))
            
            # Log
            print(f"[ROS] Robot{robot_id} cmd: v={v:.2f} m/s, ω={omega:.2f} rad/s")
            
            return True
            
        except Exception as e:
            print(f"[ROS] Send failed: {e}")
            self.connected = False
            return False
    
    def receive_feedback(self, timeout: float = 0.01) -> Optional[dict]:
        """
        Receive feedback from ROS bridge (non-blocking).
        
        Args:
            timeout: Socket timeout in seconds
            
        Returns:
            dict with odometry data, or None
        """
        if not self.connected:
            return None
        
        try:
            self.socket.settimeout(timeout)
            data = self.socket.recv(1024)
            
            if data:
                return json.loads(data.decode('utf-8'))
                
        except socket.timeout:
            pass  # No data available
        except Exception as e:
            print(f"[ROS] Receive error: {e}")
            
        return None
    
    def send_stop_command(self, robot_id: int):
        """
        Send emergency stop to robot.
        
        Args:
            robot_id: Robot to stop
        """
        self.send_velocity_command(robot_id, 0.0, 0.0)
