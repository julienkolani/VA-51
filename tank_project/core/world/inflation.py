"""
Inflation - Obstacle Cost Inflation

Inflates obstacles in the costmap for safe path planning:
- Adds safety margin around obstacles
- Creates gradient for smoother planning
- Accounts for robot size

Uses distance transform for efficient computation.

Logs: [INFLATION] Inflated with radius: Xm -> Y cells
"""

import numpy as np
from scipy.ndimage import distance_transform_edt


class CostmapInflation:
    """
    Inflates obstacles in costmap for safe planning.
    
    Creates a cost gradient around obstacles.
    """
    
    def __init__(self, inflation_radius_m: float, resolution_m: float):
        """
        Initialize inflation.
        
        Args:
            inflation_radius_m: How far to inflate in meters
            resolution_m: Grid resolution
        """
        self.inflation_radius_m = inflation_radius_m
        self.resolution = resolution_m
        self.inflation_cells = int(inflation_radius_m / resolution_m)
        
    def inflate(self, binary_grid: np.ndarray) -> np.ndarray:
        """
        Inflate obstacles in grid.
        
        Args:
            binary_grid: Grid with 0 = free, 1 = occupied
            
        Returns:
            Inflated costmap with gradient (0-1 float)
            
        Algorithm:
            1. Compute distance transform (distance to nearest obstacle)
            2. Convert distances to costs:
               - d = 0: cost = 1.0 (occupied)
               - d < inflation_radius: cost = 1.0 - (d / radius)
               - d >= inflation_radius: cost = 0.0 (free)
               
        Logs:
            [INFLATION] Inflated grid with radius: 0.24m (12 cells)
        """
        # Distance transform: each cell = distance to nearest obstacle
        distances = distance_transform_edt(1 - binary_grid) * self.resolution
        
        # Convert to costs
        costmap = np.zeros_like(distances, dtype=np.float32)
        
        # Occupied cells
        costmap[binary_grid == 1] = 1.0
        
        # Inflated region
        inflation_mask = (distances > 0) & (distances < self.inflation_radius_m)
        costmap[inflation_mask] = 1.0 - (distances[inflation_mask] / self.inflation_radius_m)
        
        return costmap
    
    def inflate_discrete(self, binary_grid: np.ndarray, 
                        lethal: int = 100, inscribed: int = 99) -> np.ndarray:
        """
        Inflate with discrete cost values (ROS-style costmap).
        
        Args:
            binary_grid: Binary occupancy grid
            lethal: Cost for occupied cells (default 100)
            inscribed: Cost for cells within inflation radius
            
        Returns:
            Costmap with values [0, inscribed, lethal]
        """
        distances = distance_transform_edt(1 - binary_grid)
        
        costmap = np.zeros_like(distances, dtype=np.uint8)
        
        # Lethal obstacles
        costmap[binary_grid == 1] = lethal
        
        # Inscribed region
        inflation_mask = (distances > 0) & (distances <= self.inflation_cells)
        costmap[inflation_mask] = inscribed
        
        return costmap
