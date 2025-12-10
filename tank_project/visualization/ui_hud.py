"""
UI HUD - Heads-Up Display

Gère affichage HUD (scores, timer, status).

Logs: [HUD] prefix
"""

import pygame
import time
from typing import Dict, Optional
from .colors import *


class UI_HUD:
    """
    Gère HUD du jeu.
    """
    
    def __init__(self, surface: pygame.Surface):
        """
        Initialize HUD.
        
        Args:
            surface: Surface Pygame
        """
        self.surface = surface
        self.width = surface.get_width()
        self.height = surface.get_height()
        
        # Fonts
        self.font_large = pygame.font.SysFont('Arial', 64, bold=True)
        self.font_medium = pygame.font.SysFont('Arial', 40)
        self.font_small = pygame.font.SysFont('Arial', 28)
        
    def draw_timer(self, time_remaining: float):
        """
        Dessine chronomètre.
        
        Args:
            time_remaining: Temps restant (secondes)
        """
        minutes = int(time_remaining // 60)
        seconds = int(time_remaining % 60)
        
        time_text = f"{minutes:02d}:{seconds:02d}"
        text = self.font_large.render(time_text, True, TIMER_COLOR)
        text_rect = text.get_rect(center=(self.width // 2, 60))
        
        # Fond
        bg_rect = text_rect.inflate(40, 20)
        pygame.draw.rect(self.surface, WHITE, bg_rect)
        pygame.draw.rect(self.surface, BLACK, bg_rect, 3)
        
        self.surface.blit(text, text_rect)
        
    def draw_scores(self, ai_hits: int, human_hits: int):
        """
        Dessine scores.
        
        Args:
            ai_hits: Hits IA
            human_hits: Hits humain
        """
        # Score IA (gauche)
        ai_text = f"IA: {ai_hits}"
        ai_surface = self.font_medium.render(ai_text, True, SCORE_AI_COLOR)
        
        bg_ai = pygame.Rect(40, 40, 150, 60)
        pygame.draw.rect(self.surface, WHITE, bg_ai)
        pygame.draw.rect(self.surface, SCORE_AI_COLOR, bg_ai, 3)
        self.surface.blit(ai_surface, (50, 50))
        
        # Score Humain (droite)
        human_text = f"HUMAIN: {human_hits}"
        human_surface = self.font_medium.render(human_text, True, SCORE_HUMAN_COLOR)
        
        bg_human = pygame.Rect(self.width - 250, 40, 210, 60)
        pygame.draw.rect(self.surface, WHITE, bg_human)
        pygame.draw.rect(self.surface, SCORE_HUMAN_COLOR, bg_human, 3)
        self.surface.blit(human_surface, (self.width - 240, 50))
        
    def draw_cooldown(self, robot_id: int, cooldown_remaining: float):
        """
        Dessine barre cooldown tir.
        
        Args:
            robot_id: 4 (IA) ou 5 (Humain)
            cooldown_remaining: Temps restant (secondes)
        """
        if cooldown_remaining <= 0:
            return
        
        # Position selon robot
        if robot_id == 4:  # IA
            x, y = 40, 120
            color = SCORE_AI_COLOR
        else:  # Humain
            x, y = self.width - 250, 120
            color = SCORE_HUMAN_COLOR
        
        # Barre fond
        bar_width = 200
        bar_height = 20
        pygame.draw.rect(self.surface, GRAY_LIGHT, (x, y, bar_width, bar_height))
        
        # Barre remplissage
        fill_width = int(bar_width * (cooldown_remaining / 5.0))  # Max 5s
        pygame.draw.rect(self.surface, color, (x, y, fill_width, bar_height))
        
        # Bordure
        pygame.draw.rect(self.surface, BLACK, (x, y, bar_width, bar_height), 2)
        
    def draw_game_over(self, winner: str):
        """
        Affiche écran fin de partie.
        
        Args:
            winner: "AI", "HUMAN", ou "DRAW"
        """
        # Fond semi-transparent
        overlay = pygame.Surface((self.width, self.height))
        overlay.set_alpha(180)
        overlay.fill(BLACK)
        self.surface.blit(overlay, (0, 0))
        
        # Texte vainqueur
        if winner == "DRAW":
            text = "ÉGALITÉ!"
            color = YELLOW
        elif winner == "AI":
            text = "VICTOIRE IA!"
            color = SCORE_AI_COLOR
        else:
            text = "VICTOIRE HUMAIN!"
            color = SCORE_HUMAN_COLOR
        
        winner_surface = self.font_large.render(text, True, color)
        winner_rect = winner_surface.get_rect(center=(self.width // 2, self.height // 2))
        self.surface.blit(winner_surface, winner_rect)
        
    def draw_status_message(self, message: str, 
                          position: Optional[tuple] = None):
        """
        Affiche message status.
        
        Args:
            message: Message à afficher
            position: Position (x,y) ou None pour centré bas
        """
        text = self.font_small.render(message, True, BLACK)
        
        if position is None:
            position = (self.width // 2 - text.get_width() // 2, 
                       self.height - 100)
        
        # Fond
        bg_rect = text.get_rect(topleft=position).inflate(20, 10)
        pygame.draw.rect(self.surface, WHITE, bg_rect)
        pygame.draw.rect(self.surface, BLACK, bg_rect, 2)
        
        self.surface.blit(text, position)
