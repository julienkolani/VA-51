"""
Trajectory Follower - Path Following Controller

Implements trajectory following for waypoint navigation:
- Pure pursuit controller
- PID-based heading control
- Dynamic waypoint advancement
- Obstacle avoidance reactions

Takes a path (list of waypoints) and current pose, outputs (v, ω).

Logs: [CTRL] Following waypoint (x,y), distance: Dm
"""

import numpy as np
from typing import Tuple, List, Optional


class TrajectoryFollower:
    """
    Trajectory following controller for waypoint navigation.
    
    Uses pure pursuit algorithm for smooth path following.
    """
    
    def __init__(self, config):
        """
        Initialize trajectory follower.
        
        Args:
            config: Controller parameters from config/robot.yaml:
                - lookahead_distance: Pure pursuit lookahead
                - k_v: Linear velocity gain
                - k_theta: Angular velocity gain
                - max_linear_vel: Max v in m/s
                - max_angular_vel: Max ω in rad/s
        """
        self.lookahead_distance = config.get('lookahead_distance', 0.3)
        self.k_v = config.get('k_v', 1.0)
        self.k_theta = config.get('k_theta', 2.0)
        self.max_linear_vel = config.get('max_linear_vel', 0.22)
        self.max_angular_vel = config.get('max_angular_vel', 2.84)
        
        self.waypoint_reached_threshold = config.get('waypoint_threshold', 0.1)
        
    def compute_control(self, 
                       current_pose: Tuple[float, float, float],
                       waypoints: List[Tuple[float, float]]) -> Tuple[float, float]:
        """
        Compute control commands to follow waypoints.
        
        Args:
            current_pose: (x, y, theta) current robot pose
            waypoints: List of (x, y) waypoints in meters
            
        Returns:
            (v, omega): linear and angular velocities
            
        Algorithm (Pure Pursuit):
            1. Find lookahead point on path
            2. Calculate curvature to reach lookahead point
            3. Compute v and ω from curvature
            4. Clamp to velocity limits
            
        Logs:
            [CTRL] Target waypoint (x,y), distance: Dm, heading error: θ rad
        """
        if not waypoints:
            return (0.0, 0.0)
        
        x, y, theta = current_pose
        
        # Find target waypoint (lookahead)
        target_wp = self._get_lookahead_point(current_pose, waypoints)
        
        if target_wp is None:
            return (0.0, 0.0)
        
        # Calculate control
        v, omega = self._pure_pursuit(current_pose, target_wp)
        
        # Clamp velocities
        v = np.clip(v, -self.max_linear_vel, self.max_linear_vel)
        omega = np.clip(omega, -self.max_angular_vel, self.max_angular_vel)
        
        return (v, omega)
    
    def _get_lookahead_point(self, pose, waypoints):
        """
        Find the waypoint at lookahead distance.
        
        Args:
            pose: Current robot pose
            waypoints: List of waypoints
            
        Returns:
            (x, y) target waypoint
        """
        x, y, _ = pose
        
        # Find first waypoint beyond lookahead distance
        for wp in waypoints:
            dist = np.sqrt((wp[0] - x)**2 + (wp[1] - y)**2)
            if dist >= self.lookahead_distance:
                return wp
        
        # If all waypoints are closer, return last one
        return waypoints[-1] if waypoints else None
    
    def _pure_pursuit(self, pose, target):
        """
        Pure pursuit control law.
        
        Args:
            pose: (x, y, theta)
            target: (x_t, y_t)
            
        Returns:
            (v, omega)
        """
        x, y, theta = pose
        x_t, y_t = target
        
        # Calculate distance and angle to target
        dx = x_t - x
        dy = y_t - y
        distance = np.sqrt(dx**2 + dy**2)
        
        # Target angle
        target_theta = np.arctan2(dy, dx)
        
        # Heading error
        theta_error = self._normalize_angle(target_theta - theta)
        
        # Linear velocity proportional to distance
        v = self.k_v * distance
        
        # Angular velocity proportional to heading error
        omega = self.k_theta * theta_error
        
        # Reduce linear velocity when turning
        v *= np.cos(theta_error)
        
        return (v, omega)
    
    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def is_waypoint_reached(self, pose, waypoint):
        """
        Check if current waypoint is reached.
        
        Args:
            pose: (x, y, theta)
            waypoint: (x, y)
            
        Returns:
            True if within threshold
        """
        x, y, _ = pose
        dist = np.sqrt((waypoint[0] - x)**2 + (waypoint[1] - y)**2)
        return dist < self.waypoint_reached_threshold
