"""
Hits - Scoring & Impact Management

Manages hit detection and scoring:
- Validate hits (is target in range, not obstructed)
- Record hits for both robots
- Calculate score deltas
- Emit hit events for visualization

Works with raycast.py for collision detection.
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class HitEvent:
    """
    Represents a single hit event.
    
    Used for visualization (flash effect, sound, score update).
    """
    shooter_id: int  # 4 (AI) or 5 (Human)
    target_id: int   # 4 or 5
    impact_point: tuple  # (x, y) in meters
    timestamp: float
    damage: int = 1  # Future: variable damage
    

class HitManager:
    """
    Manages hit validation and scoring.
    
    Collaborates with Raycast to determine valid hits.
    """
    
    def __init__(self, raycast):
        """
        Initialize hit manager.
        
        Args:
            raycast: Raycast instance for collision detection
        """
        self.raycast = raycast
        self.hit_history = []  # List of HitEvent
        
    
    def process_shot(self, shooter_id, shooter_pose, target_pose, current_time):
        """
        Process a shot attempt and determine if it hits.
        
        Args:
            shooter_id: 4 (AI) or 5 (Human)
            shooter_pose: (x, y, theta) of shooter
            target_pose: (x, y, theta) of target
            current_time: Current game time
            
        Returns:
            HitEvent if hit, None if miss
        """
        x, y, theta = shooter_pose
        target_id = 5 if shooter_id == 4 else 4
        
        target_x, target_y, _ = target_pose
        
        # Max range hardcoded for now (should come from rules)
        MAX_RANGE = 5.0 
        
        # 1. Check for obstacles first
        # We only care about obstacles closer than the target
        dist_to_target = ((target_x - x)**2 + (target_y - y)**2)**0.5
        
        obstacle_hit = self.raycast.cast_shot((x, y), theta, dist_to_target)
        if obstacle_hit['hit'] and obstacle_hit['target'] == 'obstacle':
            # Hit obstacle before target
            print("[HIT] Robot {} shot blocked by obstacle at {:.2f}m".format(shooter_id, obstacle_hit['distance']))
            return None
            
        # 2. Check for robot hit
        is_hit = self.raycast.check_robot_collision(
            (x, y), theta, MAX_RANGE, (target_x, target_y)
        )
        
        if is_hit:
            print("[HIT] Robot {} HIT Robot {}!".format(shooter_id, target_id))
            
            # Calculate impact point (approximate)
            impact_point = (target_x, target_y) 
            
            event = HitEvent(
                shooter_id=shooter_id,
                target_id=target_id,
                impact_point=impact_point,
                timestamp=current_time
            )
            self.hit_history.append(event)
            return event
            
        return None
    
    def get_score_summary(self):
        """
        Calculate current score from hit history.
        
        Returns:
            dict: {
                'robot_4_hits': int,  # Hits scored by AI
                'robot_5_hits': int,  # Hits scored by Human
            }
        """
        r4_hits = sum(1 for h in self.hit_history if h.shooter_id == 4)
        r5_hits = sum(1 for h in self.hit_history if h.shooter_id == 5)
        
        return {
            'robot_4_hits_inflicted': r4_hits,
            'robot_5_hits_inflicted': r5_hits,
            'robot_4_hits_received': r5_hits,
            'robot_5_hits_received': r4_hits
        }
    
    def clear_history(self):
        """Clear hit history (used when starting new match)."""
        self.hit_history = []
