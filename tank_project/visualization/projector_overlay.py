"""
Projector Overlay - Gestion Affichage Projecteur

Gère affichage superposé sur projecteur (éléments calibration, debug).

Logs: [OVERLAY] prefix
"""

import pygame
import numpy as np
from typing import Tuple, Optional
from .colors import *


class ProjectorOverlay:
    """
    Gère éléments overlay projetés.
    """
    
    def __init__(self, surface: pygame.Surface):
        """
        Initialize overlay.
        
        Args:
            surface: Surface Pygame où dessiner
        """
        self.surface = surface
        self.font = pygame.font.SysFont('Arial', 32)
        self.font_small = pygame.font.SysFont('Arial', 20)
        
    def draw_calibration_markers(self,
                                positions: list,
                                marker_ids: list,
                                size: int = 100):
        """
        Dessine marqueurs ArUco virtuels pour calibration.
        
        Args:
            positions: Liste (x, y) positions pixels
            marker_ids: Liste IDs marqueurs (0-3)
            size: Taille marqueurs pixels
        """
        for pos, marker_id in zip(positions, marker_ids):
            # Dessiner carré blanc avec bordure noire
            rect = pygame.Rect(pos[0] - size//2, pos[1] - size//2, size, size)
            pygame.draw.rect(self.surface, WHITE, rect)
            pygame.draw.rect(self.surface, BLACK, rect, 3)
            
            # Dessiner ID au centre
            text = self.font.render(str(marker_id), True, BLACK)
            text_rect = text.get_rect(center=pos)
            self.surface.blit(text, text_rect)
    
    def draw_crosshair(self, pos: Tuple[int, int], 
                      size: int = 20,
                      color: Tuple = RED):
        """
        Dessine réticule.
        
        Args:
            pos: Position (x, y)
            size: Taille
            color: Couleur
        """
        x, y = pos
        pygame.draw.line(self.surface, color, 
                        (x - size, y), (x + size, y), 2)
        pygame.draw.line(self.surface, color,
                        (x, y - size), (x, y + size), 2)
        pygame.draw.circle(self.surface, color, pos, size, 2)
    
    def draw_message(self, message: str,
                    position: Tuple[int, int],
                    color: Tuple = BLACK,
                    font_size: str = 'normal'):
        """
        Affiche message texte.
        
        Args:
            message: Texte à afficher
            position: (x, y) position
            color: Couleur texte
            font_size: 'normal' ou 'small'
        """
        font = self.font if font_size == 'normal' else self.font_small
        text = font.render(message, True, color)
        self.surface.blit(text, position)
    
    def draw_instruction(self, instruction: str):
        """
        Affiche instruction centrée en haut.
        
        Args:
            instruction: Texte instruction
        """
        text = self.font.render(instruction, True, BLACK)
        text_rect = text.get_rect(center=(self.surface.get_width() // 2, 100))
        
        # Fond semi-transparent
        bg_rect = text_rect.inflate(40, 20)
        s = pygame.Surface((bg_rect.width, bg_rect.height))
        s.set_alpha(200)
        s.fill(WHITE)
        self.surface.blit(s, bg_rect)
        
        # Texte
        self.surface.blit(text, text_rect)
