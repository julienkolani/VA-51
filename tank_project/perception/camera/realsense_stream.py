"""
RealSense Stream - Intel RealSense Camera Interface

Manages RealSense D435/D455 camera:
- Color stream acquisition
- Depth stream (optional)
- Camera configuration
- Frame rate management

Provides synchronized color and depth frames at 30 FPS.

Logs: [REALSENSE] prefix for camera operations
"""

import pyrealsense2 as rs
import numpy as np
from typing import Tuple, Optional


class RealSenseStream:
    """
    Interface for Intel RealSense camera.
    
    Handles camera initialization and frame acquisition.
    """
    
    def __init__(self, 
                 width: int = 640, 
                 height: int = 480, 
                 fps: int = 30,
                 enable_depth: bool = False):
        """
        Initialize RealSense camera.
        
        Args:
            width: Frame width
            height: Frame height
            fps: Frame rate
            enable_depth: Enable depth stream
            
        Logs:
            [REALSENSE] Camera initialized: WxH @ FPS fps
        """
        self.width = width
        self.height = height
        self.fps = fps
        self.enable_depth = enable_depth
        
        self.pipeline = None
        self.config = None
        
    def start(self):
        """
        Start camera pipeline.
        
        Logs:
            [REALSENSE] Pipeline started
            [REALSENSE] Failed to start: error
        """
        try:
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # Configure streams
            self.config.enable_stream(rs.stream.color, 
                                     self.width, self.height, 
                                     rs.format.bgr8, self.fps)
            
            if self.enable_depth:
                self.config.enable_stream(rs.stream.depth, 
                                         self.width, self.height, 
                                         rs.format.z16, self.fps)
            
            # Start pipeline
            self.pipeline.start(self.config)
            
            print("[REALSENSE] Pipeline started: {}x{} @ {} fps".format(self.width, self.height, self.fps))
            
        except Exception as e:
            print("[REALSENSE] Failed to start: {}".format(e))
            raise
    
    def stop(self):
        """Stop camera pipeline."""
        if self.pipeline:
            self.pipeline.stop()
            print("[REALSENSE] Pipeline stopped")
    
    def get_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Get latest color and depth frames.
        
        Returns:
            (color_frame, depth_frame): numpy arrays
            color_frame: HxWx3 BGR image
            depth_frame: HxW depth map (mm) or None
        """
        if not self.pipeline:
            return (None, None)
        
        try:
            # Wait for frames
            frames = self.pipeline.wait_for_frames()
            
            # Get color frame
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data()) if color_frame else None
            
            # Get depth frame (if enabled)
            depth_image = None
            if self.enable_depth:
                depth_frame = frames.get_depth_frame()
                depth_image = np.asanyarray(depth_frame.get_data()) if depth_frame else None
            
            return (color_image, depth_image)
            
        except Exception as e:
            print("[REALSENSE] Frame acquisition error: {}".format(e))
            return (None, None)
    
    def get_intrinsics(self):
        """
        Get camera intrinsic parameters.
        
        Returns:
            rs.intrinsics object with fx, fy, cx, cy
        """
        if self.pipeline:
            profile = self.pipeline.get_active_profile()
            color_stream = profile.get_stream(rs.stream.color)
            intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
            return intrinsics
        return None
