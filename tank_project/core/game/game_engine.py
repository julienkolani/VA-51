"""
Game Engine - Central Arbitrator

This module orchestrates the game loop and coordinates all subsystems.
It acts as the referee, managing:
- Game state transitions
- Timer management  
- Shot validation and resolution
- Win/loss conditions
- Tick-based game cycle (30 FPS)

The game engine receives:
- World state (robot poses, obstacles from perception)
- IA decisions (target, fire_request)
- Human input (triggers)

The game engine produces:
- Updated game state (scores, cooldowns, status)
- Events (shots fired, hits registered, game over)

Logs: [GAME] prefix for all game-related events
"""

class GameEngine:
    """
    Central game arbitrator managing the game loop and rules enforcement.
    
    Responsibilities:
    - Orchestrate 30 FPS game tick
    - Validate and execute shots (human + AI)
    - Update timers and cooldowns
    - Check win conditions
    - Emit game events for visualization
    
    Does NOT:
    - Make AI decisions (that's core/ia/)
    - Control motors (that's core/control/)
    - Draw anything (that's visualization/)
    """
    
    
    def __init__(self, config):
        """
        Initialize game engine with configuration.
        
        Args:
            config: Game configuration from config/game.yaml
                   (match duration, cooldowns, win conditions)
        """
        self.config = config
        
        # Load rules
        self.rules = GameRules.from_config(config['match']) if 'match' in config else GameRules()
        
        # Helper subsystems
        # Note: Raycast and Hits need WorldModel, which is passed in tick() or initialized later
        # For now we create placeholders or require world in tick
        self.raycast = None 
        self.hit_manager = None
        
        # Timers
        self.cooldowns = CooldownManager(
            self.rules.ai_shot_cooldown, 
            self.rules.human_shot_cooldown
        )
        
        # State
        self.start_time = 0.0
        self.state = GameStatus.READY
        
        print("[GAME] Engine initialized")
    
    def _ensure_subsystems(self, world_model):
        """Lazy initialization of subsystems that need world model."""
        if self.raycast is None:
            from .raycast import Raycast
            from .hits import HitManager
            
            self.raycast = Raycast(world_model.grid)
            self.hit_manager = HitManager(self.raycast)
            print("[GAME] Subsystems linked to WorldModel")

    def tick(self, world_state, ia_request, human_input):
        """
        Execute one game tick (called at 30 FPS).
        
        Args:
            world_state: WorldModel instance (NOT just a dict)
            ia_request: AI decision (target, fire_request)
            human_input: Human controls/triggers
            
        Returns:
            dict: Updated game state for visualization
        """
        import time
        current_time = time.time()
        
        # Ensure subsystems are ready
        self._ensure_subsystems(world_state)
        
        # 0. Handle Game State Transitions
        if self.state == GameStatus.READY:
            # Check for start condition (e.g. human input)
            if human_input.get('start_game', False):
                self.start_time = current_time
                self.state = GameStatus.PLAYING
                self.hit_manager.clear_history()
                print("[GAME] Match STARTED")
                
        elif self.state == GameStatus.PLAYING:
            # Check time expiration
            elapsed = current_time - self.start_time
            if elapsed >= self.rules.match_duration_seconds:
                self.state = GameStatus.FINISHED
                self._check_win_condition()
                print("[GAME] Match TIME OVER")
                
            # 1. Process Shooting
            self._handle_shooting(world_state, ia_request, human_input, current_time)
            
            # 2. Check Win Condition (Score limit)
            game_over, winner = self._check_win_condition()
            if game_over and self.state != GameStatus.FINISHED:
                self.state = GameStatus.FINISHED
                print("[GAME] Match FINISHED. Winner: {}".format(winner))

        # 3. Build State Dictionary for View
        scores = self.hit_manager.get_score_summary()
        
        state_dict = {
            'status': self.state.value,
            'time_remaining_s': max(0, self.rules.match_duration_seconds - (current_time - self.start_time)) if self.state == GameStatus.PLAYING else 0,
            
            # Scores
            'robot_4_hits_inflicted': scores['robot_4_hits_inflicted'],
            'robot_5_hits_inflicted': scores['robot_5_hits_inflicted'],
            'robot_4_hits_received': scores['robot_4_hits_received'],
            'robot_5_hits_received': scores['robot_5_hits_received'],
            
            # Cooldowns
            'can_shoot_ai': self.cooldowns.can_shoot_ai(),
            'can_shoot_human': self.cooldowns.can_shoot_human(),
            
            # Debug info passed through
            'ai_has_los': ia_request.get('has_los', False),
            'ai_fire_request': ia_request.get('fire_request', False),
            'ai_state': ia_request.get('state', 'UNKNOWN')
        }
        
        return state_dict
    
    def _handle_shooting(self, world_state, ia_request, human_input, current_time):
        """Handle fire requests from AI and Human."""
        
        # Robot poses from WorldModel
        # Assuming world_state has methods or attributes for poses
        # We need to access the latest poses. 
        # Since world_state passed here is WorldModel object, we can ask it.
        # But wait, main.py passes `world` which is WorldModel. 
        # Let's assume we can get poses from it.
        
        r4_pose = world_state.get_robot_pose(4)
        r5_pose = world_state.get_robot_pose(5)
        
        if r4_pose is None or r5_pose is None:
            return # Can't shoot if robots not tracked
            
        # --- AI SHOOTING ---
        if ia_request.get('fire_request', False):
            if self.cooldowns.can_shoot_ai():
                self.cooldowns.register_shot_ai()
                print("[GAME] AI Firing!")
                self.hit_manager.process_shot(4, r4_pose, r5_pose, current_time)
                
        # --- HUMAN SHOOTING ---
        if human_input.get('fire_request', False):
            if self.cooldowns.can_shoot_human():
                self.cooldowns.register_shot_human()
                print("[GAME] Human Firing!")
                self.hit_manager.process_shot(5, r5_pose, r4_pose, current_time)

    def _check_win_condition(self):
        """
        Check if match should end based on hits.
        
        Returns:
            (game_over: bool, winner: str or None)
        """
        scores = self.hit_manager.get_score_summary()
        ai_hits = scores['robot_4_hits_inflicted']
        human_hits = scores['robot_5_hits_inflicted']
        
        max_hits = self.rules.max_hits_to_win
        
        if ai_hits >= max_hits:
            return True, "AI"
        
        if human_hits >= max_hits:
            return True, "HUMAN"
            
        return False, None

# Imports at bottom to avoid circular deps if needed, 
# or top if safe. 
from .rules import GameRules
from .timers import CooldownManager
from .state import GameStatus
