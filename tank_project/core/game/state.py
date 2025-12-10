"""
Game State - Complete Game Status

Maintains the complete state of the ongoing match:
- Robot scores (hits given/received)
- Match timer (elapsed, remaining)
- Cooldown timers (next allowed shot for each robot)
- Game status (calibration, playing, paused, finished)
- Winner information

This is a pure data structure with no logic.
All state modifications are done by game_engine.py.
"""

from dataclasses import dataclass
from enum import Enum

class GameStatus(Enum):
    """Game lifecycle states"""
    CALIBRATION = "calibration"
    READY = "ready"
    PLAYING = "playing"
    PAUSED = "paused"
    FINISHED = "finished"


@dataclass
class RobotScore:
    """Per-robot scoring information"""
    robot_id: int
    hits_inflicted: int = 0  # Hits this robot scored on enemy
    hits_received: int = 0   # Hits this robot took
    shots_fired: int = 0     # Total shots attempted
    

@dataclass
class GameState:
    """
    Complete game state snapshot.
    
    This is the single source of truth for game status.
    Immutable between ticks - game_engine creates new state each tick.
    """
    status: GameStatus
    
    # Time tracking
    match_start_time: float  # Unix timestamp
    current_time: float      # Unix timestamp
    match_duration: float    # Total match length in seconds
    
    # Scores
    robot_4_score: RobotScore  # AI robot
    robot_5_score: RobotScore  # Human robot
    
    # Cooldowns (Unix timestamps)
    next_shot_robot_4: float
    next_shot_robot_5: float
    
    # Winner info (None if ongoing)
    winner: str = None  # "AI", "HUMAN", or "DRAW"
    
    @property
    def time_remaining(self):
        """Calculate remaining match time in seconds."""
        elapsed = self.current_time - self.match_start_time
        return max(0, self.match_duration - elapsed)
    
    @property
    def can_shoot_ai(self):
        """Check if AI cooldown allows shooting."""
        return self.current_time >= self.next_shot_robot_4
    
    @property
    def can_shoot_human(self):
        """Check if human cooldown allows shooting."""
        return self.current_time >= self.next_shot_robot_5
