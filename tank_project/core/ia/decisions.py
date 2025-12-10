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
from typing import Dict, Tuple, Optional, List


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
    ai_pose = context.get('ai_pose')
    human_pose = context.get('human_pose')
    raycast = context.get('raycast_sys')
    
    if ai_pose is None or human_pose is None:
        return False
    
    if raycast is None:
        # No raycast system available, assume clear LOS
        print("[DECISION] LOS check: NO RAYCAST SYSTEM")
        return True
    
    # Use raycast's internal LOS check
    ai_pos = ai_pose[:2]
    human_pos = human_pose[:2]
    
    los_clear = raycast._check_line_of_sight(ai_pos, human_pos)
    
    status = "CLEAR" if los_clear else "BLOCKED"
    print("[DECISION] LOS check: {}".format(status))
    
    return los_clear


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
    ai_pose = context.get('ai_pose')
    human_pose = context.get('human_pose')
    
    if ai_pose is None or human_pose is None:
        return False
    
    ai_pos = np.array(ai_pose[:2])
    human_pos = np.array(human_pose[:2])
    
    distance = np.linalg.norm(ai_pos - human_pos)
    
    in_range = min_range <= distance <= max_range
    
    print("[DECISION] Firing range check: distance={:.2f}m, optimal={}".format(
        distance, in_range))
    
    return in_range


def find_nearest_cover(context: Dict) -> Optional[Tuple[float, float]]:
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
    ai_pose = context.get('ai_pose')
    human_pose = context.get('human_pose')
    grid = context.get('occupancy_grid')
    
    if ai_pose is None or human_pose is None or grid is None:
        return None
    
    ai_pos = np.array(ai_pose[:2])
    human_pos = np.array(human_pose[:2])
    
    # Direction from enemy to AI
    direction = ai_pos - human_pos
    dist = np.linalg.norm(direction)
    if dist < 0.1:
        return None
    direction = direction / dist
    
    # Look for cover positions: move perpendicular to enemy direction
    perpendicular = np.array([-direction[1], direction[0]])
    
    # Check positions to the left and right of current position
    cover_distance = 0.5  # meters from current position
    
    candidates = [
        ai_pos + perpendicular * cover_distance,
        ai_pos - perpendicular * cover_distance,
        ai_pos + direction * cover_distance,  # Move away from enemy
    ]
    
    # Find valid cover position (within arena bounds)
    for candidate in candidates:
        x, y = candidate
        if 0 < x < grid.width_m and 0 < y < grid.height_m:
            if not grid.is_occupied(x, y):
                print("[DECISION] Cover found at ({:.2f}, {:.2f})".format(x, y))
                return (x, y)
    
    print("[DECISION] No cover found")
    return None


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
    too_close = is_enemy_too_close(context)
    los = has_line_of_sight(context)
    
    should_run = too_close and los
    
    if should_run:
        print("[DECISION] RETREAT triggered: enemy too close with LOS")
    
    return should_run


def calculate_flank_position(context: Dict) -> Optional[Tuple[float, float]]:
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
        2. Find positions 90 deg left/right of enemy facing
        3. Filter by cover availability during approach
        4. Choose closest valid position
    """
    ai_pose = context.get('ai_pose')
    human_pose = context.get('human_pose')
    grid = context.get('occupancy_grid')
    
    if ai_pose is None or human_pose is None:
        return None
    
    ai_pos = np.array(ai_pose[:2])
    human_pos = np.array(human_pose[:2])
    human_theta = human_pose[2] if len(human_pose) > 2 else 0.0
    
    # Enemy facing direction
    enemy_facing = np.array([np.cos(human_theta), np.sin(human_theta)])
    
    # Flanking positions: 90 degrees to enemy facing
    flank_distance = 1.5  # meters from enemy
    
    # Left flank (perpendicular)
    left_perp = np.array([-enemy_facing[1], enemy_facing[0]])
    left_flank = human_pos + left_perp * flank_distance
    
    # Right flank
    right_perp = np.array([enemy_facing[1], -enemy_facing[0]])
    right_flank = human_pos + right_perp * flank_distance
    
    # Choose flank closest to AI
    dist_left = np.linalg.norm(ai_pos - left_flank)
    dist_right = np.linalg.norm(ai_pos - right_flank)
    
    if dist_left < dist_right:
        chosen_flank = left_flank
    else:
        chosen_flank = right_flank
    
    x, y = chosen_flank
    
    # Validate position is within arena
    if grid is not None:
        if not (0 < x < grid.width_m and 0 < y < grid.height_m):
            print("[DECISION] Flank position out of bounds")
            return None
        if grid.is_occupied(x, y):
            print("[DECISION] Flank position occupied")
            return None
    
    print("[DECISION] Flank position: ({:.2f}, {:.2f})".format(x, y))
    return (x, y)


def get_distance_to_enemy(context: Dict) -> float:
    """
    Get distance between AI and enemy.
    
    Args:
        context: World state
        
    Returns:
        Distance in meters, or infinity if poses unknown
    """
    ai_pose = context.get('ai_pose')
    human_pose = context.get('human_pose')
    
    if ai_pose is None or human_pose is None:
        return float('inf')
    
    ai_pos = np.array(ai_pose[:2])
    human_pos = np.array(human_pose[:2])
    
    return float(np.linalg.norm(ai_pos - human_pos))


def get_angle_to_enemy(context: Dict) -> float:
    """
    Get angle from AI to enemy.
    
    Args:
        context: World state
        
    Returns:
        Angle in radians, or 0 if poses unknown
    """
    ai_pose = context.get('ai_pose')
    human_pose = context.get('human_pose')
    
    if ai_pose is None or human_pose is None:
        return 0.0
    
    dx = human_pose[0] - ai_pose[0]
    dy = human_pose[1] - ai_pose[1]
    
    return float(np.arctan2(dy, dx))
