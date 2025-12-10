"""
Timers - Cooldown & Match Time Management

Manages all time-based game mechanics:
- Match timer (total elapsed, remaining)
- Shot cooldowns per robot
- Temporary buffs/debuffs (future)

All times are in seconds (float).
Uses system time (time.time()) for absolute timing.
"""

import time


class Timer:
    """Simple countdown timer."""
    
    def __init__(self, duration_seconds):
        self.duration = duration_seconds
        self.start_time = None
        
    def start(self):
        """Start the timer."""
        self.start_time = time.time()
        
    def elapsed(self):
        """Get elapsed time in seconds."""
        if self.start_time is None:
            return 0.0
        return time.time() - self.start_time
    
    def remaining(self):
        """Get remaining time in seconds."""
        return max(0.0, self.duration - self.elapsed())
    
    def is_expired(self):
        """Check if timer has expired."""
        return self.elapsed() >= self.duration
    
    def reset(self):
        """Reset timer to start."""
        self.start_time = None


class CooldownManager:
    """
    Manages shot cooldowns for both robots.
    
    Tracks when each robot last fired and when they can fire again.
    """
    
    def __init__(self, ai_cooldown_sec, human_cooldown_sec):
        """
        Initialize cooldown manager.
        
        Args:
            ai_cooldown_sec: AI robot cooldown duration
            human_cooldown_sec: Human robot cooldown duration
        """
        self.ai_cooldown = ai_cooldown_sec
        self.human_cooldown = human_cooldown_sec
        
        self.last_shot_ai = 0.0
        self.last_shot_human = 0.0
        
    def can_shoot_ai(self):
        """Check if AI can shoot now."""
        return time.time() >= self.last_shot_ai + self.ai_cooldown
    
    def can_shoot_human(self):
        """Check if human can shoot now."""
        return time.time() >= self.last_shot_human + self.human_cooldown
    
    def register_shot_ai(self):
        """
        Register that AI fired.
        
        Logs:
            [TIMER] AI cooldown started (Xs remaining)
        """
        self.last_shot_ai = time.time()
        
    def register_shot_human(self):
        """
        Register that human fired.
        
        Logs:
            [TIMER] Human cooldown started (Xs remaining)
        """
        self.last_shot_human = time.time()
    
    def time_until_next_shot_ai(self):
        """Get seconds until AI can shoot again."""
        return max(0.0, self.last_shot_ai + self.ai_cooldown - time.time())
    
    def time_until_next_shot_human(self):
        """Get seconds until human can shoot again."""
        return max(0.0, self.last_shot_human + self.human_cooldown - time.time())
