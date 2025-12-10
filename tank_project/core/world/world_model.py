"""
World Model - Unified World Representation

Central repository for all world state:
- Robot poses (filtered by Kalman)
- Occupancy grid (obstacles)
- Arena boundaries
- Coordinate frames

This is the single source of truth for spatial information.
All other modules query the world model.

Does NOT contain game logic (scores, etc.) - only spatial state.
"""

from typing import Dict, List, Tuple
from .occupancy_grid import OccupancyGrid
from .coordinate_frames import TransformManager


class WorldModel:
    """
    Complete spatial world representation.
    
    Manages:
    - Robot state (positions, velocities, orientations)
    - Obstacles (static + dynamic)
    - Coordinate transformations
    - Arena boundaries
    """
    
    def __init__(self, arena_width_m: float, arena_height_m: float, 
                 grid_resolution_m: float = 0.02):
        """
        Initialize world model.
        
        Args:
            arena_width_m: Arena width from calibration
            arena_height_m: Arena height from calibration
            grid_resolution_m: Grid cell size
        """
        self.arena_width = arena_width_m
        self.arena_height = arena_height_m
        
        # Occupancy grid
        self.grid = OccupancyGrid(arena_width_m, arena_height_m, grid_resolution_m)
        
        # Robot state
        self.robots = {
            4: {  # AI robot
                'pose': (0.0, 0.0, 0.0),  # (x, y, theta)
                'velocity': (0.0, 0.0, 0.0),  # (vx, vy, omega)
                'radius_m': 0.09,  # Turtlebot Burger radius
            },
            5: {  # Human robot
                'pose': (0.0, 0.0, 0.0),
                'velocity': (0.0, 0.0, 0.0),
                'radius_m': 0.09,
            }
        }
        
        # Coordinate transforms
        self.transforms = TransformManager()
        
    def update_robot_pose(self, robot_id: int, pose: Tuple[float, float, float]):
        """
        Update robot pose from Kalman filter.
        
        Args:
            robot_id: 4 or 5
            pose: (x, y, theta) in meters/radians
        """
        if robot_id in self.robots:
            self.robots[robot_id]['pose'] = pose
    
    def update_robot_velocity(self, robot_id: int, 
                             velocity: Tuple[float, float, float]):
        """
        Update robot velocity from Kalman filter.
        
        Args:
            robot_id: 4 or 5
            velocity: (vx, vy, omega) in m/s and rad/s
        """
        if robot_id in self.robots:
            self.robots[robot_id]['velocity'] = velocity
    
    def update_occupancy(self):
        """
        Update occupancy grid with current robot positions.
        
        Called each frame after robot poses are updated.
        """
        robot_poses = [self.robots[rid]['pose'] for rid in [4, 5]]
        robot_radius = self.robots[4]['radius_m']
        
        self.grid.update_dynamic_obstacles(robot_poses, robot_radius)
    
    def get_robot_pose(self, robot_id: int) -> Tuple[float, float, float]:
        """Get current robot pose."""
        return self.robots[robot_id]['pose']
    
    def get_robot_velocity(self, robot_id: int) -> Tuple[float, float, float]:
        """Get current robot velocity."""
        return self.robots[robot_id]['velocity']
    
    def is_position_valid(self, x: float, y: float) -> bool:
        """
        Check if a position is within arena and not occupied.
        
        Args:
            x, y: Position in meters
            
        Returns:
            True if position is valid (in bounds and free)
        """
        # Check bounds
        if not (0 <= x <= self.arena_width and 0 <= y <= self.arena_height):
            return False
        
        # Check occupancy
        return not self.grid.is_occupied(x, y)
    
    def get_state_dict(self) -> Dict:
        """
        Export complete world state as dictionary.
        
        Used by AI, game engine, visualization.
        
        Returns:
            dict with all world information
        """
        return {
            'arena_size': (self.arena_width, self.arena_height),
            'robot_4_pose': self.robots[4]['pose'],
            'robot_5_pose': self.robots[5]['pose'],
            'robot_4_velocity': self.robots[4]['velocity'],
            'robot_5_velocity': self.robots[5]['velocity'],
            'occupancy_grid': self.grid,
        }
