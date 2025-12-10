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
            
        Algorithm:
            1. Use raycast from shooter position in direction theta
            2. Check if ray intersects target robot's bounding circle
            3. If hit, create HitEvent and add to history
            
        Logs:
            [HIT] Robot{shooter_id} -> Robot{target_id} at (x,y)
            [HIT] Robot{shooter_id} -> MISS
        """
        pass
    
    def get_score_summary(self):
        """
        Calculate current score from hit history.
        
        Returns:
            dict: {
                'robot_4_hits': int,  # Hits scored by AI
                'robot_5_hits': int,  # Hits scored by Human
            }
        """
        pass
    
    def clear_history(self):
        """Clear hit history (used when starting new match)."""
        self.hit_history = []
