"""
Path Utilities - Path Processing and Optimization

Utilities for working with planned paths:
- Path smoothing
- Waypoint simplification (Douglas-Peucker)
- Path validation
- Distance calculation
- Interpolation

Takes raw A* output and makes it execution-ready.
"""

import numpy as np
from typing import List, Tuple, Optional


def smooth_path(waypoints: List[Tuple[float, float]], 
                weight_data: float = 0.5,
                weight_smooth: float = 0.3,
                tolerance: float = 0.01,
                max_iterations: int = 100) -> List[Tuple[float, float]]:
    """
    Smooth a path using gradient descent.
    
    Balances staying close to original path vs smoothness.
    
    Args:
        waypoints: Original path [(x1,y1), ...]
        weight_data: How much to stay close to original
        weight_smooth: How much to smooth
        tolerance: Convergence threshold
        max_iterations: Maximum iterations
        
    Returns:
        Smoothed path
    """
    if len(waypoints) <= 2:
        return waypoints
    
    # Convert to numpy array for easier manipulation
    path = np.array(waypoints, dtype=np.float64)
    smoothed = path.copy()
    
    n_points = len(path)
    
    for iteration in range(max_iterations):
        change = 0.0
        
        # Don't modify first and last points
        for i in range(1, n_points - 1):
            for j in range(2):  # x and y
                old_val = smoothed[i, j]
                
                # Data term: stay close to original
                data_term = weight_data * (path[i, j] - smoothed[i, j])
                
                # Smooth term: average of neighbors
                smooth_term = weight_smooth * (
                    smoothed[i-1, j] + smoothed[i+1, j] - 2 * smoothed[i, j]
                )
                
                smoothed[i, j] += data_term + smooth_term
                change += abs(smoothed[i, j] - old_val)
        
        # Check convergence
        if change < tolerance:
            break
    
    return [(float(p[0]), float(p[1])) for p in smoothed]


def simplify_path_douglas_peucker(waypoints: List[Tuple[float, float]], 
                                  epsilon: float = 0.05) -> List[Tuple[float, float]]:
    """
    Simplify path using Douglas-Peucker algorithm.
    
    Removes waypoints that are nearly collinear.
    
    Args:
        waypoints: Original path
        epsilon: Maximum deviation tolerance in meters
        
    Returns:
        Simplified path with fewer waypoints
        
    Algorithm:
        1. Find point farthest from line between start and end
        2. If distance < epsilon, remove all intermediate points
        3. Otherwise, recursively apply to [start, farthest] and [farthest, end]
    """
    if len(waypoints) <= 2:
        return waypoints
    
    # Convert to numpy for calculations
    points = np.array(waypoints)
    
    # Find point with maximum distance from line (start -> end)
    start = points[0]
    end = points[-1]
    
    # Line vector
    line_vec = end - start
    line_len = np.linalg.norm(line_vec)
    
    if line_len < 1e-10:
        # Start and end are same point
        return [waypoints[0], waypoints[-1]]
    
    line_unit = line_vec / line_len
    
    # Find perpendicular distances
    max_dist = 0.0
    max_idx = 0
    
    for i in range(1, len(points) - 1):
        # Vector from start to point
        vec_to_point = points[i] - start
        
        # Project onto line
        proj_length = np.dot(vec_to_point, line_unit)
        proj_point = start + proj_length * line_unit
        
        # Perpendicular distance
        dist = np.linalg.norm(points[i] - proj_point)
        
        if dist > max_dist:
            max_dist = dist
            max_idx = i
    
    # If max distance is less than epsilon, simplify to just endpoints
    if max_dist < epsilon:
        return [waypoints[0], waypoints[-1]]
    
    # Otherwise, recursively simplify
    left_simplified = simplify_path_douglas_peucker(waypoints[:max_idx + 1], epsilon)
    right_simplified = simplify_path_douglas_peucker(waypoints[max_idx:], epsilon)
    
    # Combine (avoid duplicating the split point)
    return left_simplified[:-1] + right_simplified


def validate_path(waypoints: List[Tuple[float, float]], 
                 occupancy_grid) -> bool:
    """
    Check if path is collision-free.
    
    Args:
        waypoints: Path to validate
        occupancy_grid: Current occupancy grid
        
    Returns:
        True if path is valid (no collisions)
    """
    if len(waypoints) < 2:
        return True
    
    # Check each waypoint
    for x, y in waypoints:
        if not (0 <= x <= occupancy_grid.width_m and 0 <= y <= occupancy_grid.height_m):
            return False
        if occupancy_grid.is_occupied(x, y):
            return False
    
    # Check line segments between waypoints
    resolution = occupancy_grid.resolution
    
    for i in range(len(waypoints) - 1):
        x1, y1 = waypoints[i]
        x2, y2 = waypoints[i + 1]
        
        # Sample points along segment
        dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        n_samples = max(2, int(dist / resolution) + 1)
        
        for t in np.linspace(0, 1, n_samples):
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            
            if occupancy_grid.is_occupied(x, y):
                return False
    
    return True


def calculate_path_length(waypoints: List[Tuple[float, float]]) -> float:
    """
    Calculate total path length in meters.
    
    Args:
        waypoints: Path [(x1,y1), ...]
        
    Returns:
        Total length in meters
    """
    if len(waypoints) < 2:
        return 0.0
    
    length = 0.0
    for i in range(len(waypoints) - 1):
        dx = waypoints[i+1][0] - waypoints[i][0]
        dy = waypoints[i+1][1] - waypoints[i][1]
        length += np.sqrt(dx**2 + dy**2)
    
    return length


def interpolate_path(waypoints: List[Tuple[float, float]], 
                    resolution: float = 0.05) -> List[Tuple[float, float]]:
    """
    Densify path by interpolating between waypoints.
    
    Useful for smooth visualization or fine-grained control.
    
    Args:
        waypoints: Sparse path
        resolution: Desired spacing in meters
        
    Returns:
        Dense path with points every ~resolution meters
    """
    if len(waypoints) < 2:
        return waypoints
    
    dense_path = [waypoints[0]]
    
    for i in range(len(waypoints) - 1):
        x1, y1 = waypoints[i]
        x2, y2 = waypoints[i + 1]
        
        # Distance between consecutive waypoints
        dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        if dist < resolution:
            # No interpolation needed
            dense_path.append((x2, y2))
            continue
        
        # Number of intermediate points
        n_points = int(dist / resolution)
        
        for j in range(1, n_points + 1):
            t = j / (n_points + 1)
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            dense_path.append((x, y))
        
        dense_path.append((x2, y2))
    
    return dense_path


def get_path_curvature(waypoints: List[Tuple[float, float]]) -> List[float]:
    """
    Calculate curvature at each waypoint.
    
    Useful for speed adaptation (slow down at sharp turns).
    
    Args:
        waypoints: Path [(x1,y1), ...]
        
    Returns:
        List of curvature values (1/radius, 0 for straight)
    """
    if len(waypoints) < 3:
        return [0.0] * len(waypoints)
    
    curvatures = [0.0]  # First point has no curvature
    
    for i in range(1, len(waypoints) - 1):
        p0 = np.array(waypoints[i - 1])
        p1 = np.array(waypoints[i])
        p2 = np.array(waypoints[i + 1])
        
        # Vectors
        v1 = p1 - p0
        v2 = p2 - p1
        
        # Cross product magnitude (2D)
        cross = abs(v1[0] * v2[1] - v1[1] * v2[0])
        
        # Segment lengths
        len1 = np.linalg.norm(v1)
        len2 = np.linalg.norm(v2)
        
        if len1 < 1e-10 or len2 < 1e-10:
            curvatures.append(0.0)
            continue
        
        # Curvature approximation: 2 * sin(angle) / chord_length
        # Simplified: cross / (len1 * len2)
        curvature = cross / (len1 * len2)
        curvatures.append(curvature)
    
    curvatures.append(0.0)  # Last point has no curvature
    
    return curvatures
