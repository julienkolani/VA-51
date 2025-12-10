"""
Kinematics - Robot Motion Model

Differential drive robot kinematics for Turtlebot Burger:
- Forward kinematics: (v, ω) → (dx, dy, dθ)
- Inverse kinematics: velocity constraints
- Dynamics model (simplified)

Used for simulation and control validation.
"""

import numpy as np
from typing import Tuple


class DifferentialDriveKinematics:
    """
    Kinematics for differential drive robot (Turtlebot Burger).
    
    Robot parameters:
    - Wheel base: distance between wheels
    - Wheel radius
    """
    
    def __init__(self, wheel_base: float = 0.16, wheel_radius: float = 0.033):
        """
        Initialize kinematics model.
        
        Args:
            wheel_base: Distance between wheels in meters (Burger: 0.16m)
            wheel_radius: Wheel radius in meters (Burger: 0.033m)
        """
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        
    def forward_kinematics(self, 
                          v: float, 
                          omega: float, 
                          current_pose: Tuple[float, float, float],
                          dt: float) -> Tuple[float, float, float]:
        """
        Compute new pose from velocity commands.
        
        Args:
            v: Linear velocity in m/s
            omega: Angular velocity in rad/s
            current_pose: (x, y, theta) current pose
            dt: Time step in seconds
            
        Returns:
            (x_new, y_new, theta_new) updated pose
            
        Equations:
            dx = v * cos(θ) * dt
            dy = v * sin(θ) * dt
            dθ = ω * dt
        """
        x, y, theta = current_pose
        
        # Update orientation first
        theta_new = theta + omega * dt
        
        # Average theta for more accurate integration
        theta_avg = theta + 0.5 * omega * dt
        
        # Update position
        x_new = x + v * np.cos(theta_avg) * dt
        y_new = y + v * np.sin(theta_avg) * dt
        
        # Normalize theta to [-pi, pi]
        theta_new = np.arctan2(np.sin(theta_new), np.cos(theta_new))
        
        return (x_new, y_new, theta_new)
    
    def wheel_velocities_to_body(self, 
                                 v_left: float, 
                                 v_right: float) -> Tuple[float, float]:
        """
        Convert wheel velocities to body velocities.
        
        Args:
            v_left: Left wheel velocity in m/s
            v_right: Right wheel velocity in m/s
            
        Returns:
            (v, omega): body linear and angular velocities
        """
        v = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / self.wheel_base
        
        return (v, omega)
    
    def body_to_wheel_velocities(self, 
                                 v: float, 
                                 omega: float) -> Tuple[float, float]:
        """
        Convert body velocities to wheel velocities.
        
        Args:
            v: Linear velocity in m/s
            omega: Angular velocity in rad/s
            
        Returns:
            (v_left, v_right): wheel velocities in m/s
        """
        v_left = v - (omega * self.wheel_base) / 2.0
        v_right = v + (omega * self.wheel_base) / 2.0
        
        return (v_left, v_right)
    
    def validate_velocities(self, 
                           v: float, 
                           omega: float,
                           max_v: float = 0.22,
                           max_omega: float = 2.84) -> Tuple[float, float]:
        """
        Validate and clamp velocities to robot limits.
        
        Args:
            v, omega: Desired velocities
            max_v: Maximum linear velocity (Burger: 0.22 m/s)
            max_omega: Maximum angular velocity (Burger: 2.84 rad/s)
            
        Returns:
            (v_clamped, omega_clamped)
        """
        v_clamped = np.clip(v, -max_v, max_v)
        omega_clamped = np.clip(omega, -max_omega, max_omega)
        
        return (v_clamped, omega_clamped)
