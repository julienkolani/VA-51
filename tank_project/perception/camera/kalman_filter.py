"""
Kalman Filter - Robot Pose Tracking

Extended Kalman filter for robot state estimation:
- State: [x, y, vx, vy, theta, omega]
- Measurements: [x, y, theta] from ArUco
- Prediction: constant velocity model

Smooths noisy ArUco detections and estimates velocities.

Logs: [KALMAN] RobotX state: x=X, y=Y, theta=T, vx=VX, vy=VY
"""

import numpy as np
from typing import Tuple


class KalmanFilter:
    """
    Extended Kalman Filter for 2D robot pose and velocity estimation.
    
    State vector: [x, y, vx, vy, theta, omega]
    """
    
    def __init__(self, dt: float = 1/30.0):
        """
        Initialize Kalman filter.
        
        Args:
            dt: Time step (default 30 FPS = 0.033s)
        """
        self.dt = dt
        
        # State: [x, y, vx, vy, theta, omega]
        self.state = np.zeros(6)
        
        # State covariance
        self.P = np.eye(6) * 1.0
        
        # Process noise
        self.Q = np.diag([0.01, 0.01, 0.1, 0.1, 0.01, 0.1])
        
        # Measurement noise
        self.R = np.diag([0.05, 0.05, 0.1])  # [x, y, theta]
        
    def predict(self):
        """
        Prediction step: propagate state forward.
        
        State transition:
            x += vx * dt
            y += vy * dt
            vx (constant)
            vy (constant)
            theta += omega * dt
            omega (constant)
        """
        # State transition matrix
        F = np.array([
            [1, 0, self.dt, 0, 0, 0],
            [0, 1, 0, self.dt, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, self.dt],
            [0, 0, 0, 0, 0, 1]
        ])
        
        # Predict state
        self.state = F @ self.state
        
        # Normalize theta
        self.state[4] = np.arctan2(np.sin(self.state[4]), np.cos(self.state[4]))
        
        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q
        
    def update(self, measurement: Tuple[float, float, float]):
        """
        Update step: incorporate measurement.
        
        Args:
            measurement: (x, y, theta) from ArUco detection
        """
        # Measurement matrix (observe x, y, theta)
        H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0]
        ])
        
        z = np.array(measurement)
        
        # Innovation
        y = z - H @ self.state
        
        # Normalize angle innovation
        y[2] = np.arctan2(np.sin(y[2]), np.cos(y[2]))
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.state = self.state + K @ y
        
        # Normalize theta
        self.state[4] = np.arctan2(np.sin(self.state[4]), np.cos(self.state[4]))
        
        # Update covariance
        self.P = (np.eye(6) - K @ H) @ self.P
        
    def get_pose(self) -> Tuple[float, float, float]:
        """
        Get current pose estimate.
        
        Returns:
            (x, y, theta)
        """
        return (self.state[0], self.state[1], self.state[4])
    
    def get_velocity(self) -> Tuple[float, float, float]:
        """
        Get current velocity estimate.
        
        Returns:
            (vx, vy, omega)
        """
        return (self.state[2], self.state[3], self.state[5])
    
    def get_full_state(self) -> np.ndarray:
        """
        Get complete state vector.
        
        Returns:
            [x, y, vx, vy, theta, omega]
        """
        return self.state.copy()
    
    def reset(self, initial_pose: Tuple[float, float, float]):
        """
        Reset filter with new initial pose.
        
        Args:
            initial_pose: (x, y, theta)
        """
        self.state = np.array([
            initial_pose[0],  # x
            initial_pose[1],  # y
            0.0,              # vx
            0.0,              # vy
            initial_pose[2],  # theta
            0.0               # omega
        ])
        
        self.P = np.eye(6) * 1.0
