"""
ArUco Detector - ArUco Marker Detection & Pose Estimation

Detects ArUco markers in camera images:
- Projected markers (ID 0-3): arena corners for calibration
- Robot markers (ID 4, 5): robot tracking

Provides:
- Marker center positions (pixels)
- Marker orientations (radians)
- Corner positions for scale estimation

Logs: [ARUCO] Detected N markers: [IDs]
"""

import cv2
import numpy as np
from typing import List, Dict, Tuple, Optional


class ArucoDetector:
    """
    ArUco marker detection and pose estimation.
    
    Uses cv2.aruco for marker detection.
    """
    
    def __init__(self, 
                 dictionary_type=cv2.aruco.DICT_4X4_50,
                 marker_size_m: float = 0.10):
        """
        Initialize ArUco detector.
        
        Args:
            dictionary_type: ArUco dictionary (default: 4x4, 50 markers)
            marker_size_m: Physical marker size in meters (for scale estimation)
        """
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_type)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        
        self.marker_size_m = marker_size_m
        
    def detect(self, image: np.ndarray) -> Dict[int, Dict]:
        """
        Detect ArUco markers in image.
        
        Args:
            image: Input image (BGR or grayscale)
            
        Returns:
            dict: {
                marker_id: {
                    'center': (u, v),  # pixel coordinates
                    'corners': [(u1,v1), (u2,v2), (u3,v3), (u4,v4)],
                    'orientation': theta  # radians
                }
            }
            
        Logs:
            [ARUCO] Detected N markers: [IDs]
        """
        # Convert to grayscale if needed
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        # Detect markers
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        results = {}
        
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                marker_corners = corners[i][0]  # Shape: (4, 2)
                
                # Calculate center
                center = marker_corners.mean(axis=0)
                
                # Calculate orientation (from corner 0 to corner 1)
                # Corner order: top-left, top-right, bottom-right, bottom-left
                dx = marker_corners[1][0] - marker_corners[0][0]
                dy = marker_corners[1][1] - marker_corners[0][1]
                orientation = np.arctan2(dy, dx)
                
                results[marker_id] = {
                    'center': tuple(center),
                    'corners': [tuple(c) for c in marker_corners],
                    'orientation': orientation
                }
            
            print("[ARUCO] Detected {} markers: {}".format(len(ids), ids.flatten().tolist()))
        
        return results
    
    def estimate_marker_size_av(self, marker_corners, H_C2AV):
        """
        Estimate marker size in Arena Virtual units.
        
        Used during calibration to compute metric scale.
        
        Args:
            marker_corners: List of 4 corner positions in pixels
            H_C2AV: Homography camera → arena virtual
            
        Returns:
            float: Marker side length in AV units
        """
        # Transform corners to AV space
        corners_av = []
        for u, v in marker_corners:
            p_cam = np.array([u, v, 1.0])
            p_av = H_C2AV @ p_cam
            p_av = p_av[:2] / p_av[2]  # Normalize
            corners_av.append(p_av)
        
        # Calculate average side length
        corners_av = np.array(corners_av)
        side_lengths = []
        for i in range(4):
            j = (i + 1) % 4
            length = np.linalg.norm(corners_av[j] - corners_av[i])
            side_lengths.append(length)
        
        avg_length = np.mean(side_lengths)
        
        return avg_length
    
    def draw_detections(self, image: np.ndarray, detections: Dict) -> np.ndarray:
        """
        Draw detected markers on image (for debugging).
        
        Args:
            image: Input image
            detections: Detection results from detect()
            
        Returns:
            Image with drawn markers
        """
        img_draw = image.copy()
        
        for marker_id, data in detections.items():
            center = data['center']
            corners = data['corners']
            
            # Draw corners
            corners_array = np.array(corners, dtype=np.int32)
            cv2.polylines(img_draw, [corners_array], True, (0, 255, 0), 2)
            
            # Draw center
            cv2.circle(img_draw, (int(center[0]), int(center[1])), 5, (0, 0, 255), -1)
            
            # Draw ID
            cv2.putText(img_draw, f"ID:{marker_id}", 
                       (int(center[0]), int(center[1]) - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        return img_draw
