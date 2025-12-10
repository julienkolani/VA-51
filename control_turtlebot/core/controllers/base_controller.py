"""Base controller abstract class."""

from abc import ABC, abstractmethod
from typing import Tuple, Dict, Any
import logging

logger = logging.getLogger(__name__)


class BaseController(ABC):
    """
    Abstract base class for all robot controllers.
    
    Controllers translate input (keyboard, joystick, etc.) 
    into robot velocity commands (linear, angular).
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize controller with configuration.
        
        Args:
            config: Controller-specific configuration dictionary
        """
        self.config = config
        self.linear_cmd = 0.0
        self.angular_cmd = 0.0
        self._enabled = True
        
        logger.info(f"{self.__class__.__name__} initialized")
    
    @abstractmethod
    def update(self, events: list) -> None:
        """
        Update controller state based on input events.
        
        Args:
            events: List of pygame events to process
        """
        pass
    
    @abstractmethod
    def get_command(self) -> Tuple[float, float]:
        """
        Get current velocity command.
        
        Returns:
            Tuple of (linear_velocity, angular_velocity)
        """
        pass
    
    @abstractmethod
    def emergency_stop(self) -> None:
        """Execute emergency stop - immediately set all velocities to zero."""
        pass
    
    def enable(self) -> None:
        """Enable the controller."""
        self._enabled = True
        logger.info(f"{self.__class__.__name__} enabled")
    
    def disable(self) -> None:
        """Disable the controller and stop robot."""
        self._enabled = False
        self.emergency_stop()
        logger.info(f"{self.__class__.__name__} disabled")
    
    def is_enabled(self) -> bool:
        """Check if controller is enabled."""
        return self._enabled
    
    def reset(self) -> None:
        """Reset controller to initial state."""
        self.linear_cmd = 0.0
        self.angular_cmd = 0.0
        logger.debug(f"{self.__class__.__name__} reset")
