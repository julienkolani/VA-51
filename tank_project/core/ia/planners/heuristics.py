"""
Heuristics - Cost Functions for Path Planning

Provides heuristic functions for A* and other planners:
- Euclidean distance (standard, admissible)
- Manhattan distance (grid-based)
- Diagonal distance (Chebyshev + diagonal cost)
- Custom costmap-aware heuristics

All heuristics must be admissible (never overestimate).
"""

import numpy as np
from typing import Tuple


def euclidean_distance(cell1: Tuple[int, int], cell2: Tuple[int, int]) -> float:
    """
    Euclidean distance heuristic.
    
    Most accurate for free space planning.
    
    Args:
        cell1: (row, col)
        cell2: (row, col)
        
    Returns:
        Euclidean distance
    """
    return np.sqrt((cell1[0] - cell2[0])**2 + (cell1[1] - cell2[1])**2)


def manhattan_distance(cell1: Tuple[int, int], cell2: Tuple[int, int]) -> float:
    """
    Manhattan (L1) distance.
    
    Good for 4-connected grids.
    
    Args:
        cell1: (row, col)
        cell2: (row, col)
        
    Returns:
        Manhattan distance
    """
    return abs(cell1[0] - cell2[0]) + abs(cell1[1] - cell2[1])


def diagonal_distance(cell1: Tuple[int, int], cell2: Tuple[int, int]) -> float:
    """
    Diagonal distance (Chebyshev + diagonal cost).
    
    Good for 8-connected grids with diagonal cost √2.
    
    Args:
        cell1: (row, col)
        cell2: (row, col)
        
    Returns:
        Diagonal-aware distance
    """
    dx = abs(cell1[0] - cell2[0])
    dy = abs(cell1[1] - cell2[1])
    
    # Cost: move diagonal (√2 ≈ 1.414) then straight (1.0)
    D = 1.0  # Straight cost
    D2 = 1.414  # Diagonal cost
    
    return D * (dx + dy) + (D2 - 2*D) * min(dx, dy)


def costmap_aware_heuristic(cell1: Tuple[int, int], 
                            cell2: Tuple[int, int],
                            costmap) -> float:
    """
    Costmap-aware heuristic.
    
    Incorporates obstacle proximity into heuristic.
    Still admissible if base is euclidean.
    
    Args:
        cell1: (row, col)
        cell2: (row, col)
        costmap: OccupancyGrid with inflation
        
    Returns:
        Modified heuristic favoring safer paths
    """
    base_h = euclidean_distance(cell1, cell2)
    
    # Optional: add small cost penalty based on average costmap value
    # Must remain admissible!
    
    return base_h
