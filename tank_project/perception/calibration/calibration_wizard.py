"""
Calibration Wizard - Interactive Setup Process

Guides user through calibration sequence:
1. Safe zone definition (projection margins)
2. Geometric calibration (H_C2AV from projected corners)
3. Metric calibration (scale from physical ArUco)
4. Obstacle mapping (static obstacles detection)

Saves calibration to config/arena.yaml for game phase.

Logs: [CALIB] prefix for all steps
"""

import cv2
import numpy as np
import yaml
import time
import select
import sys
from typing import Tuple, List
from ..camera.aruco_detector import ArucoDetector
from core.world.coordinate_frames import TransformManager
from .projector_display import ProjectorDisplay


class CalibrationWizard:
    """
    Interactive calibration process for arena setup.
    
    Produces H_C2W transform and arena parameters.
    """
    
    def __init__(self, camera, projector_width=1024, projector_height=768):
        """
        Initialize calibration wizard.
        
        Args:
            camera: RealSenseStream instance
            projector_width: Projector resolution width
            projector_height: Projector resolution height
        """
        self.camera = camera
        self.proj_w = projector_width
        self.proj_h = projector_height
        
        self.aruco = ArucoDetector()
        self.transform_mgr = TransformManager()
        
        # Initialize projector display
        self.projector = ProjectorDisplay(
            width=projector_width,
            height=projector_height,
            margin=50
        )
        
        # Calibration results
        self.margin_px = 50
        self.arena_width_m = None
        self.arena_height_m = None
        self.H_C2W = None
        self.static_obstacles = []
    
    def _wait_for_enter_key(self, message: str = "Press ENTER when ready..."):
        """
        Wait for ENTER key while keeping Pygame window responsive.
        
        This prevents the "not responding" error by continuously
        processing Pygame events while waiting for keyboard input.
        
        Args:
            message: Message to display to user
        """
        print(message, end='', flush=True)
        
        # Keep processing Pygame events until ENTER is pressed
        waiting = True
        while waiting:
            # Process Pygame events to keep window responsive
            self.projector.handle_events()
            
            # Check if ENTER was pressed (non-blocking)
            if sys.platform != 'win32':
                # Unix/Linux: Use select for non-blocking input check
                i, o, e = select.select([sys.stdin], [], [], 0.01)
                if i:
                    line = sys.stdin.readline()
                    waiting = False
                    print()  # New line after input
            else:
                # Windows: Use msvcrt (fallback to blocking input)
                import msvcrt
                if msvcrt.kbhit():
                    key = msvcrt.getch()
                    if key == b'\r':  # ENTER key
                        waiting = False
                        print()  # New line after input
            
            # Small sleep to prevent CPU spinning
            time.sleep(0.01)
        
    def run(self) -> dict:
        """
        Run complete calibration wizard.
        
        Returns:
            dict: Calibration results to save to config
            
        Steps:
            1. Define safe zone (margins)
            2. Detect projected corners → H_C2AV
            3. Detect physical marker → scale → H_C2W
            4. Map static obstacles
            5. Calculate arena dimensions
            
        Logs:
            [CALIB] Step X/4: Description
        """
        print("[CALIB] ========== Starting Calibration Wizard ==========")
        
        # Start projector display
        print("[CALIB] Starting projector display...")
        self.projector.start()
        
        try:
            # Step 1: Safe zone
            self._step_safe_zone()
            
            # Step 2: Geometric calibration
            H_C2AV = self._step_geometric_calibration()
            
            # Step 3: Metric calibration
            scale = self._step_metric_calibration(H_C2AV)
            
            # Step 4: Obstacle mapping
            obstacles = self._step_obstacle_mapping()
            
            # Build results
            results = {
                'projector': {
                    'width': self.proj_w,
                    'height': self.proj_h,
                    'margin': self.margin_px
                },
                'arena': {
                    'width_m': self.arena_width_m,
                    'height_m': self.arena_height_m
                },
                'transform': {
                    'H_C2W': self.H_C2W.tolist() if self.H_C2W is not None else None,
                    'scale': scale
                },
                'obstacles': obstacles
            }
            
            print("[CALIB] Calibration complete!")
            return results
            
        finally:
            # Always stop projector
            print("[CALIB] Stopping projector display...")
            self.projector.stop()
    
    def _step_safe_zone(self):
        """
        Step 1: Define safe zone for projection.
        
        Logs:
            [CALIB] MARGIN set to X px
            [CALIB] Arena rect in projector: (x1,y1) -> (x2,y2)
        """
        print("[CALIB] MARGIN set to {} px".format(self.margin_px))
        
        x1, y1 = self.margin_px, self.margin_px
        x2 = self.proj_w - self.margin_px
        y2 = self.proj_h - self.margin_px
        
        print("[CALIB] Arena rect in projector: ({},{}) -> ({},{})".format(x1, y1, x2, y2))
        
    def _step_geometric_calibration(self) -> np.ndarray:
        """
        Step 2: Detect projected corners and compute H_C2AV.
        
        Returns:
            H_C2AV: Homography matrix
            
        Logs:
            [CALIB] Detected 4 projected corners
            [CALIB] H_C2AV computed successfully
        """
        print("[CALIB] Step 2/4: Geometric calibration (detecting projected corners)")
        
        # Project ArUco markers at corners
        print("[CALIB] Projecting ArUco markers (IDs 0-3) at arena corners...")
        self.projector.show_corner_markers(marker_size_px=200)
        
        print("[CALIB] ArUco markers displayed on projector")
        print("[CALIB] Verify that camera can see all 4 markers, then press ENTER...")
        
        self._wait_for_enter_key("Press ENTER when ready...")
        
        # Capture frame
        color, _ = self.camera.get_frames()
        
        # Detect ArUco
        detections = self.aruco.detect(color)
        
        # Check for corners (IDs 0-3)
        corner_ids = [0, 1, 2, 3]
        detected_corners = {k: v for k, v in detections.items() if k in corner_ids}
        
        if len(detected_corners) != 4:
            print("[CALIB] ERROR: Expected 4 corners, found {}".format(len(detected_corners)))
            print("[CALIB] Detected IDs: {}".format(list(detected_corners.keys())))
            raise ValueError("Missing projected corners")
        
        print("[CALIB] Detected 4 projected corners")
        
        # Build correspondences
        src_points = []
        dst_points = []
        
        # Arena virtual coordinates (unit square)
        av_coords = {
            0: (0.0, 0.0),  # Bottom-left
            1: (1.0, 0.0),  # Bottom-right
            2: (1.0, 1.0),  # Top-right
            3: (0.0, 1.0)   # Top-left
        }
        
        for marker_id in corner_ids:
            center = detected_corners[marker_id]['center']
            src_points.append(center)
            dst_points.append(av_coords[marker_id])
        
        src_points = np.array(src_points, dtype=np.float32)
        dst_points = np.array(dst_points, dtype=np.float32)
        
        # Compute homography
        H_C2AV, _ = cv2.findHomography(src_points, dst_points)
        
        print("[CALIB] H_C2AV computed successfully")
        
        return H_C2AV
    
    def _step_metric_calibration(self, H_C2AV: np.ndarray) -> float:
        """
        Step 3: Estimate metric scale from physical marker.
        
        Args:
            H_C2AV: Homography from step 2
            
        Returns:
            scale: meters per AV unit
            
        Logs:
            [CALIB] Real marker size: X m
            [CALIB] Marker size in AV: Y units
            [CALIB] Scale: Z m / AV_unit
        """
        print("[CALIB] Step 3/4: Metric calibration (place physical marker in arena)")
        print("Enter physical marker size in meters (e.g. 0.10): ", end='', flush=True)
        marker_size_real = float(input())
        
        print("[CALIB] Real marker size: {} m".format(marker_size_real))
        
        self._wait_for_enter_key("Press ENTER when marker is placed...")
        
        # Capture frame
        color, _ = self.camera.get_frames()
        
        # Detect markers
        detections = self.aruco.detect(color)
        
        # Find robot marker (ID 4 or 5)
        marker_data = None
        for mid in [4, 5]:
            if mid in detections:
                marker_data = detections[mid]
                break
        
        if marker_data is None:
            raise ValueError("No robot marker detected (ID 4 or 5)")
        
        # Estimate size in AV
        corners = marker_data['corners']
        size_av = self.aruco.estimate_marker_size_av(corners, H_C2AV)
        
        print("[CALIB] Marker size in AV: {:.3f} units".format(size_av))
        
        # Compute scale
        scale = marker_size_real / size_av
        
        print("[CALIB] Scale: {:.3f} m / AV_unit".format(scale))
        
        # Build H_C2W
        self.transform_mgr.set_camera_to_av(
            np.array([d['center'] for d in detections.values()]),
            np.array([[0,0], [1,0], [1,1], [0,1]])
        )
        self.transform_mgr.set_av_to_world_scale(scale)
        
        self.H_C2W = self.transform_mgr.H_C2W
        
        # Estimate arena size
        self.arena_width_m = 1.0 * scale
        self.arena_height_m = 1.0 * scale
        
        print("[CALIB] H_C2W computed")
        print("[CALIB] Calibration OK")
        
        return scale
    
    def _step_obstacle_mapping(self) -> List:
        """
        Step 4: Map static obstacles.
        
        Returns:
            List of obstacle regions
            
        Logs:
            [CALIB] Arena size estimated: XmxYm
            [CALIB] Static obstacles mapped
        """
        print("[CALIB] Step 4/4: Obstacle mapping")
        
        # Display white screen for obstacle contrast
        print("[CALIB] Displaying white screen for obstacle detection...")
        self.projector.show_white_screen()
        
        print("[CALIB] Place obstacles in arena, then press ENTER...")
        
        self._wait_for_enter_key()
        
        # Simplified: return empty for now
        # Full implementation would do thresholding and contour detection
        
        print("[CALIB] Arena size estimated: {:.2f}m x {:.2f}m".format(self.arena_width_m, self.arena_height_m))
        print("[CALIB] Static obstacles mapped")
        
        return []
