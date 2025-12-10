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

import numpy as np
from typing import Dict, Tuple, Optional
from .behavior_tree import BehaviorTreeExecutor, NodeStatus
from .decisions import has_line_of_sight, is_enemy_too_close, is_optimal_firing_range
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
        self.behavior_tree = BehaviorTreeExecutor()
        self.planner = None  # Set when world is available
        
        # AI state
        self.current_path = []
        self.current_waypoint_idx = 0
        self.state = "IDLE"  # ATTACK, FLANK, RETREAT, HUNT
        
        # Decision rate control
        self.decision_interval = config.get('decision_rate', {}).get('replan_interval', 10)
        self.tick_count = 0
        
        print("[AI] Strategy initialized")
        
    def set_planner(self, occupancy_grid):
        """
        Initialize path planner with occupancy grid.
        
        Args:
            occupancy_grid: OccupancyGrid from core/world
        """
        heuristic = self.config.get('strategy', {}).get('heuristic', 'euclidean')
        self.planner = AStarPlanner(occupancy_grid, heuristic)
        print("[AI] Path planner initialized with {} heuristic".format(heuristic))
        
    def decide(self, world_state: Dict) -> Dict:
        """
        Main decision function called each game tick.
        
        Args:
            world_state: {
                'ai_pose': (x, y, theta),
                'human_pose': (x, y, theta),
                'occupancy_grid': grid object,
                'raycast_sys': raycast object,
                'game_time': float,
            }
            
        Returns:
            {
                'target_position': (x, y) or None,
                'target_orientation': theta or None,
                'fire_request': bool,
                'state': str (ATTACK/FLANK/RETREAT),
                'has_los': bool,
            }
        """
        self.tick_count += 1
        
        ai_pose = world_state.get('ai_pose')
        enemy_pose = world_state.get('human_pose')
        
        # Default Output
        decision = {
            'target_position': None,
            'target_orientation': None,
            'fire_request': False,
            'state': self.state,
            'has_los': False
        }
        
        if ai_pose is None or enemy_pose is None:
            return decision

        # Prepare context for behavior tree
        context = {
            'ai_pose': ai_pose,
            'human_pose': enemy_pose,
            'occupancy_grid': world_state.get('occupancy_grid'),
            'raycast_sys': world_state.get('raycast_sys'),
        }
        
        # Execute behavior tree
        bt_output = self.behavior_tree.execute(context)
        
        # Copy behavior tree decisions
        decision['fire_request'] = bt_output.get('fire_request', False)
        decision['has_los'] = bt_output.get('has_los', False)
        decision['state'] = bt_output.get('state', 'IDLE')
        self.state = decision['state']
        
        target_pos = bt_output.get('target_position')
        
        # Path Planning - only replan periodically or when target changes significantly
        if target_pos is not None and self.planner:
            should_replan = False
            
            # Replan if no path or on interval
            if not self.current_path:
                should_replan = True
            elif self.tick_count % self.decision_interval == 0:
                should_replan = True
            elif len(self.current_path) > 0:
                # Replan if target moved significantly
                last_goal = self.current_path[-1]
                dist_to_new = np.linalg.norm(
                    np.array(last_goal) - np.array(target_pos)
                )
                if dist_to_new > 0.5:
                    should_replan = True
            
            if should_replan:
                path = self.planner.plan(ai_pose[:2], target_pos)
                if path:
                    self.current_path = path
                    self.current_waypoint_idx = 0
                    print("[AI] New path planned: {} waypoints".format(len(path)))
        
        decision['target_position'] = target_pos
        
        # Log decision
        if self.tick_count % 30 == 0:  # Log every second at 30 FPS
            self._log_decision(decision)
        
        return decision
    
    def _log_decision(self, decision):
        """Log current AI decision."""
        target = decision.get('target_position')
        target_str = "({:.2f}, {:.2f})".format(target[0], target[1]) if target else "None"
        
        print("[AI] State: {}, target={}, fire={}, LOS={}".format(
            decision['state'],
            target_str,
            decision['fire_request'],
            decision['has_los']
        ))
    
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
        
    def get_full_path(self) -> list:
        """
        Get complete current path for visualization.
        
        Returns:
            List of (x, y) waypoints
        """
        return self.current_path
