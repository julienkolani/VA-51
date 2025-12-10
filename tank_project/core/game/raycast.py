"""
Raycast - Shot Collision Detection

Implements ray-based collision detection for laser shots:
- Cast ray from shooter position in direction theta
- Check intersections with:
  * Static obstacles (walls, blocks)
  * Dynamic obstacles (robots)
- Return first hit or None

Uses the occupancy grid from core/world for obstacle detection.
Implements DDA (Digital Differential Analyzer) for efficient grid traversal.

Logs: [RAYCAST] Hit detected / Miss
"""

import numpy as np
from typing import Optional, Tuple


class Raycast:
    """
    Efficient ray-based collision detection for shots.
    
    Uses DDA algorithm to traverse occupancy grid and detect hits.
    """
    
    def __init__(self, occupancy_grid):
        """
        Initialize raycast with world occupancy grid.
        
        Args:
            occupancy_grid: OccupancyGrid instance from core/world
        """
        self.grid = occupancy_grid
    
    def cast_shot(self, start_pos, theta, max_range_m):
        """
        Cast a shot ray and detect collisions.
        
        Args:
            start_pos: (x, y) shooter position in meters
            theta: Shot direction in radians
            max_range_m: Maximum shot range
            
        Returns:
            dict: {
                'hit': bool,
                'target': 'robot4' | 'robot5' | 'obstacle' | None,
                'impact_point': (x, y) in meters or None,
                'distance': float in meters
            }
            
        Algorithm:
            1. Convert start position to grid coordinates
            2. Use DDA to traverse grid cells along ray direction
            3. Check each cell for:
               - Static obstacle (grid value = 1)
               - Robot presence (check robot positions)
            4. Return first hit or max_range
            
        Logs:
            [RAYCAST] Shot from (x,y) theta=T -> HIT robot5 at (x2,y2)
            [RAYCAST] Shot from (x,y) theta=T -> MISS (obstacle at d=D)
        """
        pass
    
    def _check_line_of_sight(self, pos1, pos2):
        """
        Check if there's a clear line of sight between two points.
        
        Used by AI to determine if enemy is visible.
        
        Args:
            pos1: (x, y) start position in meters
            pos2: (x, y) end position in meters
            
        Returns:
            bool: True if line of sight is clear
        """
        pass
    
    def _dda_traverse(self, start_cell, direction, max_cells):
        """
        DDA algorithm for grid traversal.
        
        Args:
            start_cell: (row, col) starting grid cell
            direction: (dx, dy) normalized direction vector
            max_cells: Maximum cells to traverse
            
        Yields:
            (row, col) for each cell along the ray
        """
        pass
