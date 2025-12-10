"""
Coordinate Frames - Transformation Management

Manages all coordinate transformations:
- Camera → Arena Virtual (H_C2AV) - homography from projected corners
- Arena Virtual → World (scaling) - metric calibration
- Camera → World (H_C2W) - combined transform
- World → Pygame (projection display)
- World → Projector (overlay display)

All transformations are 2D homographies or affine transforms.

Logs: [TRANSFORM] prefix for all transform operations
"""

import numpy as np
import cv2
from typing import Tuple, List


class TransformManager:
    """
    Manages coordinate frame transformations.
    
    Stores and applies homographies between different coordinate systems.
    """
    
    def __init__(self):
        """Initialize transform manager."""
        self.H_C2AV = None  # Camera → Arena Virtual
        self.H_AV2W = None  # Arena Virtual → World (scaling)
        self.H_C2W = None   # Camera → World (combined)
        self.H_W2Proj = None  # World → Projector display
        
        self.scale_m_per_av = None  # Metric scale factor
        
    def set_camera_to_av(self, src_points: np.ndarray, dst_points: np.ndarray):
        """
        Compute H_C2AV from corner correspondences.
        
        Args:
            src_points: 4x2 array of camera pixel coordinates
            dst_points: 4x2 array of arena virtual coordinates (e.g. unit square)
            
        Computes homography using cv2.findHomography.
        
        Logs:
            [TRANSFORM] H_C2AV computed from 4 corners
        """
        self.H_C2AV, _ = cv2.findHomography(src_points, dst_points)
        self._update_combined()
        
    def set_av_to_world_scale(self, scale: float):
        """
        Set scaling from Arena Virtual to World (meters).
        
        Args:
            scale: meters per AV unit
            
        Creates scaling transform H_AV2W.
        
        Logs:
            [TRANSFORM] Scale set: 1.15 m/AV_unit
        """
        self.scale_m_per_av = scale
        self.H_AV2W = np.array([
            [scale, 0, 0],
            [0, scale, 0],
            [0, 0, 1]
        ], dtype=np.float32)
        self._update_combined()
        
    def _update_combined(self):
        """Update H_C2W = H_AV2W @ H_C2AV."""
        if self.H_C2AV is not None and self.H_AV2W is not None:
            self.H_C2W = self.H_AV2W @ self.H_C2AV
            
    def camera_to_world(self, u: float, v: float) -> Tuple[float, float]:
        """
        Transform camera pixel to world meters.
        
        Args:
            u, v: Camera pixel coordinates
            
        Returns:
            (x, y) in meters
        """
        if self.H_C2W is None:
            raise ValueError("H_C2W not set, run calibration first")
        
        # Homogeneous coordinates
        p_cam = np.array([u, v, 1.0])
        p_world = self.H_C2W @ p_cam
        
        # Normalize
        x = p_world[0] / p_world[2]
        y = p_world[1] / p_world[2]
        
        return (x, y)
    
    def world_to_projector(self, x: float, y: float, 
                          arena_width_m: float, arena_height_m: float,
                          proj_width_px: int, proj_height_px: int,
                          margin_px: int = 50) -> Tuple[int, int]:
        """
        Transform world position to projector pixel.
        
        Args:
            x, y: World position in meters
            arena_width_m, arena_height_m: Arena size
            proj_width_px, proj_height_px: Projector resolution
            margin_px: Safe zone margin
            
        Returns:
            (px, py) projector pixel coordinates
        """
        # Scale to projector (with margin)
        draw_width = proj_width_px - 2 * margin_px
        draw_height = proj_height_px - 2 * margin_px
        
        scale_x = draw_width / arena_width_m
        scale_y = draw_height / arena_height_m
        scale = min(scale_x, scale_y)  # Maintain aspect ratio
        
        px = margin_px + int(x * scale)
        py = margin_px + int((arena_height_m - y) * scale)  # Flip Y (pygame origin top-left)
        
        return (px, py)
    
    def batch_camera_to_world(self, points_cam: np.ndarray) -> np.ndarray:
        """
        Transform multiple camera points to world.
        
        Args:
            points_cam: Nx2 array of camera coordinates
            
        Returns:
            Nx2 array of world coordinates
        """
        if self.H_C2W is None:
            raise ValueError("H_C2W not set")
        
        # Add homogeneous coordinate
        ones = np.ones((points_cam.shape[0], 1))
        points_h = np.hstack([points_cam, ones])
        
        # Transform
        points_world_h = (self.H_C2W @ points_h.T).T
        
        # Normalize
        points_world = points_world_h[:, :2] / points_world_h[:, 2:3]
        
        return points_world
