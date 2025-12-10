"""
Decisions - Tactical Decision Functions

Provides tactical assessment functions used by behavior tree conditions:
- Is enemy too close? (threat assessment)
- Do we have line of sight? (visibility check)
- Are we in optimal firing range?
- Is cover available nearby?
- Should we retreat?

All functions take context dict and return bool or tactical value.

Logs: [DECISION] Assessment X: value Y
"""

import numpy as np
from typing import Dict, Tuple


def is_enemy_too_close(context: Dict, threshold_m: float = 0.8) -> bool:
    """
    Check if enemy is dangerously close.
    
    Args:
        context: World state with robot poses
        threshold_m: Danger threshold in meters
        
    Returns:
        True if enemy within threshold
    """
    ai_pos = context['ai_pose'][:2]
    human_pos = context['human_pose'][:2]
    distance = np.linalg.norm(np.array(ai_pos) - np.array(human_pos))
    
    return distance < threshold_m


def has_line_of_sight(context: Dict) -> bool:
    """
    Check if AI has clear line of sight to enemy.
    
    Uses raycast from core.world to check for obstacles.
    
    Args:
        context: World state with robot poses and raycast
        
    Returns:
        True if clear line of sight exists
        
    Logs:
        [DECISION] LOS check: CLEAR/BLOCKED
    """
    pass


def is_optimal_firing_range(context: Dict, 
                            min_range: float = 1.2, 
                            max_range: float = 3.5) -> bool:
    """
    Check if enemy is in optimal firing range.
    
    Too close: risk of being hit back
    Too far: accuracy decreases
    
    Args:
        context: World state
        min_range: Minimum safe distance
        max_range: Maximum effective distance
        
    Returns:
        True if in optimal range
    """
    pass


def find_nearest_cover(context: Dict) -> Tuple[float, float]:
    """
    Find nearest cover position relative to enemy.
    
    Cover = obstacle that blocks line of sight to enemy.
    
    Args:
        context: World state with occupancy grid
        
    Returns:
        (x, y) position of best cover, or None
        
    Algorithm:
        1. Get all obstacle cells from grid
        2. For each obstacle, check if it blocks LOS to enemy
        3. Rank by:
           - Distance to AI (closer is better)
           - Cover effectiveness (blocks LOS well)
        4. Return best position
    """
    pass


def should_retreat(context: Dict) -> bool:
    """
    Comprehensive retreat decision.
    
    Retreat if:
    - Enemy too close AND has LOS
    - Low health (future feature)
    - Surrounded
    
    Args:
        context: World state
        
    Returns:
        True if should retreat
    """
    return (is_enemy_too_close(context) and 
            has_line_of_sight(context))


def calculate_flank_position(context: Dict) -> Tuple[float, float]:
    """
    Calculate optimal flanking position.
    
    Goal: position that:
    - Gives AI line of sight to enemy
    - Is NOT in enemy's current line of sight
    - Uses cover for approach
    
    Args:
        context: World state
        
    Returns:
        (x, y) target flanking position
        
    Algorithm:
        1. Get enemy position and orientation
        2. Find positions 90° left/right of enemy facing
        3. Filter by cover availability during approach
        4. Choose closest valid position
    """
    pass
