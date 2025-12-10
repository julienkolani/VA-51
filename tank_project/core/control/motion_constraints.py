"""
Motion Constraints - Velocity & Acceleration Limits

Enforces robot physical constraints:
- Maximum velocities (linear, angular)
- Maximum accelerations
- Smooth velocity ramping
- Emergency stop logic

Prevents damaging commands and ensures smooth motion.
"""

import numpy as np
from typing import Tuple


class MotionConstraints:
    """
    Enforces physical motion constraints for safe robot control.
    
    Prevents:
    - Exceeding velocity limits
    - Excessive acceleration (sudden changes)
    - Unsafe commands
    """
    
    def __init__(self, config):
        """
        Initialize motion constraints.
        
        Args:
            config: Robot configuration:
                - max_linear_vel: m/s
                - max_angular_vel: rad/s
                - max_linear_accel: m/s²
                - max_angular_accel: rad/s²
        """
        self.max_v = config.get('max_linear_vel', 0.22)
        self.max_omega = config.get('max_angular_vel', 2.84)
        self.max_accel_v = config.get('max_linear_accel', 0.5)
        self.max_accel_omega = config.get('max_angular_accel', 5.0)
        
        # Previous commands for acceleration limiting
        self.prev_v = 0.0
        self.prev_omega = 0.0
        
    def apply_constraints(self, 
                         v_desired: float, 
                         omega_desired: float,
                         dt: float) -> Tuple[float, float]:
        """
        Apply all motion constraints.
        
        Args:
            v_desired: Desired linear velocity
            omega_desired: Desired angular velocity
            dt: Time since last command (seconds)
            
        Returns:
            (v_safe, omega_safe): constrained velocities
            
        Steps:
            1. Clamp to velocity limits
            2. Limit acceleration
            3. Update previous commands
        """
        # Velocity limits
        v = np.clip(v_desired, -self.max_v, self.max_v)
        omega = np.clip(omega_desired, -self.max_omega, self.max_omega)
        
        # Acceleration limits
        if dt > 0:
            v = self._limit_acceleration(v, self.prev_v, self.max_accel_v, dt)
            omega = self._limit_acceleration(omega, self.prev_omega, 
                                            self.max_accel_omega, dt)
        
        # Store for next iteration
        self.prev_v = v
        self.prev_omega = omega
        
        return (v, omega)
    
    def _limit_acceleration(self, 
                           desired: float, 
                           previous: float,
                           max_accel: float, 
                           dt: float) -> float:
        """
        Limit acceleration of a single velocity component.
        
        Args:
            desired: Desired velocity
            previous: Previous velocity
            max_accel: Maximum allowed acceleration
            dt: Time step
            
        Returns:
            Acceleration-limited velocity
        """
        delta = desired - previous
        max_delta = max_accel * dt
        
        if abs(delta) > max_delta:
            delta = np.sign(delta) * max_delta
        
        return previous + delta
    
    def emergency_stop(self):
        """
        Reset to zero velocity immediately.
        
        Used for safety stop.
        """
        self.prev_v = 0.0
        self.prev_omega = 0.0
        
        return (0.0, 0.0)
    
    def soft_stop(self, dt: float) -> Tuple[float, float]:
        """
        Gradually decelerate to zero.
        
        Args:
            dt: Time step
            
        Returns:
            Decelerating velocities
        """
        return self.apply_constraints(0.0, 0.0, dt)
