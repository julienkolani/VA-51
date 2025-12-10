#!/usr/bin/env python3
"""
Module Keyboard Controller
"""

import pygame
import math
import config


class KeyboardController:
    """Contrôleur clavier configurable pour TurtleBot"""

    def __init__(self, vitesse_factor=None):

        cfg = config.get_keyboard_config()

        # Facteur de vitesse initial
        self.vitesse_factor = (
            vitesse_factor if vitesse_factor is not None
            else config.INITIAL_SPEED_FACTOR
        )

        # États internes
        self.vitesse_lineaire = 0.0
        self.vitesse_angulaire = 0.0
        self.tir_demande = False

        # Paramètres dépendant du facteur
        self.accel_lineaire = cfg["accel_linear"] * self.vitesse_factor
        self.accel_angulaire = cfg["accel_angular"] * self.vitesse_factor

        self.vitesse_max_lineaire = cfg["max_linear"] * self.vitesse_factor
        self.vitesse_max_angulaire = cfg["max_angular"] * self.vitesse_factor

        self.friction = cfg["friction"]
        self.friction_angulaire = cfg["friction_angular"]

    # ============================================================
    #   MISE À JOUR DU FACTEUR DE VITESSE (+ et -)
    # ============================================================
    def set_vitesse_factor(self, factor):

        ratio = factor / self.vitesse_factor
        self.vitesse_factor = factor

        cfg = config.get_keyboard_config()

        # Recalcul des paramètres
        self.accel_lineaire = cfg["accel_linear"] * factor
        self.accel_angulaire = cfg["accel_angular"] * factor
        self.vitesse_max_lineaire = cfg["max_linear"] * factor
        self.vitesse_max_angulaire = cfg["max_angular"] * factor

        # Ajustement des vitesses actuelles
        self.vitesse_lineaire *= ratio
        self.vitesse_angulaire *= ratio

    # ============================================================
    #   MISE À JOUR DES MOUVEMENTS
    # ============================================================
    def update(self, touches):

        self.tir_demande = False

        # ----------------- CONTRÔLE LINÉAIRE -----------------
        if touches[pygame.K_UP] or touches[pygame.K_w]:
            self.vitesse_lineaire = min(
                self.vitesse_lineaire + self.accel_lineaire,
                self.vitesse_max_lineaire
            )

        elif touches[pygame.K_DOWN] or touches[pygame.K_s]:
            self.vitesse_lineaire = max(
                self.vitesse_lineaire - self.accel_lineaire,
                -self.vitesse_max_lineaire
            )

        else:
            # friction linéaire
            self.vitesse_lineaire *= self.friction
            if abs(self.vitesse_lineaire) < 0.05:
                self.vitesse_lineaire = 0

        # ----------------- CONTRÔLE ANGULAIRE -----------------
        gauche  = touches[pygame.K_LEFT] or touches[pygame.K_a]
        droite  = touches[pygame.K_RIGHT] or touches[pygame.K_d]
        avant   = touches[pygame.K_UP] or touches[pygame.K_w]
        arriere = touches[pygame.K_DOWN] or touches[pygame.K_s]

        # Correction du bug : inversion du sens en marche arrière
        direction_factor = -1 if self.vitesse_lineaire < 0 else 1

        if gauche and droite:
            if avant:
                self.vitesse_angulaire = self.vitesse_max_angulaire
            elif arriere:
                self.vitesse_angulaire = -self.vitesse_max_angulaire
            else:
                self.vitesse_angulaire *= self.friction_angulaire

        elif gauche:
            intensite = 1.0 + abs(self.vitesse_lineaire) * 0.3
            self.vitesse_angulaire = max(
                self.vitesse_angulaire - self.accel_angulaire * intensite * direction_factor,
                -self.vitesse_max_angulaire * 0.6
            )

        elif droite:
            intensite = 1.0 + abs(self.vitesse_lineaire) * 0.3
            self.vitesse_angulaire = min(
                self.vitesse_angulaire + self.accel_angulaire * intensite * direction_factor,
                self.vitesse_max_angulaire * 0.6
            )

        else:
            # friction angulaire
            self.vitesse_angulaire *= self.friction_angulaire
            if abs(self.vitesse_angulaire) < 0.2:
                self.vitesse_angulaire = 0

        # ----------------- CONVERSION POUR ROS -----------------
        linear_x = self.vitesse_lineaire * 0.1
        angular_z = math.radians(self.vitesse_angulaire)

        return linear_x, angular_z, self.tir_demande

    # ============================================================
    #   ÉVÉNEMENTS (ARRÊT D'URGENCE)
    # ============================================================
    def handle_event(self, event):
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                self.tir_demande = True
                return True
        return False
