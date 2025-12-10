"""
Projector Display for Calibration

Displays ArUco markers on projector for calibration wizard.
Shows 4 corner markers (IDs 0-3) in windowed mode for geometric calibration.

Logs: [PROJ_DISPLAY] prefix
"""

import pygame
import cv2
import numpy as np
from typing import Tuple, Optional


class ProjectorDisplay:
    """
    Pygame window for projecting calibration patterns.
    
    Displays ArUco markers at corners for geometric calibration.
    """
    
    def __init__(self, width=1024, height=768, margin=50):
        """
        Initialize projector display.
        
        Args:
            width: Display width in pixels
            height: Display height in pixels  
            margin: Safety margin from edges (px)
        
        Usage:
            1. Window will open in normal windowed mode
            2. Drag window to projector screen if needed
        """
        self.width = width
        self.height = height
        self.margin = margin
        
        self.screen = None
        self.running = False
        
        # Arena rectangle (with margins)
        self.arena_x1 = margin
        self.arena_y1 = margin
        self.arena_x2 = width - margin
        self.arena_y2 = height - margin
        self.arena_w = self.arena_x2 - self.arena_x1
        self.arena_h = self.arena_y2 - self.arena_y1
        
        print("[PROJ_DISPLAY] Initialized: {}x{}, margin={}px".format(width, height, margin))
        print("[PROJ_DISPLAY] Arena zone: ({},{}) -> ({},{})".format(self.arena_x1, self.arena_y1, self.arena_x2, self.arena_y2))
    
    def start(self):
        """Start Pygame and create window."""
        pygame.init()
        
        # Always start in windowed mode
        self.screen = pygame.display.set_mode(
            (self.width, self.height), 
            pygame.RESIZABLE
        )
        
        pygame.display.set_caption("Tank Arena - Calibration Projection")
        self.running = True
        
        print("[PROJ_DISPLAY] Display window created (windowed mode: {}x{})".format(self.width, self.height))
        print("[PROJ_DISPLAY] Drag window to projector screen if needed")
    
    def stop(self):
        """Close display."""
        if self.running:
            pygame.quit()
            self.running = False
            print("[PROJ_DISPLAY] Display closed")
    
    def clear(self, color=(0, 0, 0)):
        """Clear screen to solid color."""
        if self.screen:
            self.screen.fill(color)
    
    def show_corner_markers(self, marker_size_px=200, aruco_dict=cv2.aruco.DICT_4X4_100):
        """
        Display ArUco markers at 4 corners of arena.
        
        Args:
            marker_size_px: Size of each marker in pixels
            aruco_dict: ArUco dictionary to use
            
        Corner positions:
            ID 0: Bottom-left
            ID 1: Bottom-right
            ID 2: Top-right
            ID 3: Top-left
        """
        if not self.running:
            return
        
        # Clear to white background
        self.clear((255, 255, 255))
        
        # Generate markers
        aruco_dict_obj = cv2.aruco.getPredefinedDictionary(aruco_dict)
        
        # Corner positions (center of marker)
        corners = {
            0: (self.arena_x1 + marker_size_px // 2, self.arena_y2 - marker_size_px // 2),  # Bottom-left
            1: (self.arena_x2 - marker_size_px // 2, self.arena_y2 - marker_size_px // 2),  # Bottom-right
            2: (self.arena_x2 - marker_size_px // 2, self.arena_y1 + marker_size_px // 2),  # Top-right
            3: (self.arena_x1 + marker_size_px // 2, self.arena_y1 + marker_size_px // 2),  # Top-left
        }
        
        print("[PROJ_DISPLAY] Projecting 4 corner markers (IDs 0-3)")
        
        for marker_id, (cx, cy) in corners.items():
            # Generate marker image
            marker_img = cv2.aruco.generateImageMarker(aruco_dict_obj, marker_id, marker_size_px)
            
            # Convert to pygame surface
            marker_img_rgb = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2RGB)
            marker_surface = pygame.surfarray.make_surface(
                np.transpose(marker_img_rgb, (1, 0, 2))
            )
            
            # Calculate top-left corner
            x = cx - marker_size_px // 2
            y = cy - marker_size_px // 2
            
            # Draw marker
            self.screen.blit(marker_surface, (x, y))
            
            # Add ID label below marker
            font = pygame.font.Font(None, 36)
            text = font.render(f"ID {marker_id}", True, (0, 0, 0))
            text_rect = text.get_rect(center=(cx, cy + marker_size_px // 2 + 30))
            self.screen.blit(text, text_rect)
        
        self.handle_events() # Keep window responsive
        # Update display
        pygame.display.flip()
        
        print("[PROJ_DISPLAY] Corner markers displayed")
    
    def show_white_screen(self):
        """Display solid white screen (for obstacle detection)."""
        if not self.running:
            return
        
        self.clear((255, 255, 255))
        
        # Add text instruction
        font = pygame.font.Font(None, 48)
        text = font.render("Place obstacles in arena", True, (0, 0, 0))
        text_rect = text.get_rect(center=(self.width // 2, 100))
        self.screen.blit(text, text_rect)
        
        self.handle_events() # Keep window responsive
        pygame.display.flip()
        print("[PROJ_DISPLAY] White screen displayed for obstacle mapping")
    
    def show_message(self, message: str, color=(255, 255, 255), bg_color=(0, 0, 0)):
        """
        Display a text message.
        
        Args:
            message: Text to display
            color: Text color (RGB)
            bg_color: Background color (RGB)
        """
        if not self.running:
            return
        
        self.clear(bg_color)
        
        font = pygame.font.Font(None, 72)
        text = font.render(message, True, color)
        text_rect = text.get_rect(center=(self.width // 2, self.height // 2))
        self.screen.blit(text, text_rect)
        
        pygame.display.flip()
    

    
    def handle_events(self):
        """Process pygame events (call periodically to prevent freezing)."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.stop()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.stop()
