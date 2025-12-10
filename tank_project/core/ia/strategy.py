"""
Strategy - High-Level AI Controller

Orchestrates the AI system:
1. Read world state
2. Execute behavior tree
3. Trigger path planning if needed
4. Return AI decisions (target, fire_request)

This is the main entry point for the AI subsystem.

Logs: [AI] State: ATTACK/FLANK/RETREAT, target=(x,y), fire=True/False
"""

from typing import Dict, Tuple, Optional
from .behavior_tree import build_ai_behavior_tree, NodeStatus
from .decisions import *
from .planners.a_star import AStarPlanner


class AIStrategy:
    """
    Main AI controller.
    
    Combines behavior tree + path planning to produce AI actions.
    """
    
    def __init__(self, config):
        """
        Initialize AI strategy.
        
        Args:
            config: AI configuration from config/ia.yaml
        """
        self.config = config
        self.behavior_tree = build_ai_behavior_tree()
        self.planner = None  # Set when world is available
        
        # AI state
        self.current_path = []
        self.current_waypoint_idx = 0
        self.state = "IDLE"  # ATTACK, FLANK, RETREAT
        
    def set_planner(self, occupancy_grid):
        """
        Initialize path planner with occupancy grid.
        
        Args:
            occupancy_grid: OccupancyGrid from core/world
        """
        self.planner = AStarPlanner(occupancy_grid)
        
    def decide(self, world_state: Dict) -> Dict:
        """
        Main decision function called each game tick.
        
        Args:
            world_state: {
                'ai_pose': (x, y, theta),
                'human_pose': (x, y, theta),
                'occupancy_grid': grid object,
                'raycast': raycast object,
                'game_time': float,
            }
            
        Returns:
            {
                'target_position': (x, y) or None,
                'target_orientation': theta or None,
                'fire_request': bool,
                'state': str (ATTACK/FLANK/RETREAT),
            }
            
        Algorithm:
            1. Prepare context for behavior tree
            2. Execute behavior tree
            3. Extract decisions from context['ai_output']
            4. If new target, plan path
            5. Return decisions
            
        Logs:
            [AI] State: ATTACK, target=(1.5, 2.3), fire=True
            [AI] Path length: 12 waypoints
        """
        pass
    
    def get_next_waypoint(self) -> Optional[Tuple[float, float]]:
        """
        Get next waypoint from current path.
        
        Used by control module for trajectory following.
        
        Returns:
            (x, y) of next waypoint, or None if no path
        """
        if self.current_waypoint_idx >= len(self.current_path):
            return None
        return self.current_path[self.current_waypoint_idx]
    
    def advance_waypoint(self):
        """
        Mark current waypoint as reached, move to next.
        
        Called by control module when waypoint is reached.
        """
        self.current_waypoint_idx += 1
