"""
Thresholding - Seuillage Image

Fonctions de seuillage pour détection obstacles.

Logs: [THRESHOLD] prefix
"""

import cv2
import numpy as np


def simple_threshold(image: np.ndarray,
                    threshold: int = 200,
                    max_value: int = 255) -> np.ndarray:
    """
    Seuillage binaire simple.
    
    Args:
        image: Image grayscale
        threshold: Valeur seuil
        max_value: Valeur maximum
        
    Returns:
        Image binaire
    """
    _, binary = cv2.threshold(image, threshold, max_value, cv2.THRESH_BINARY)
    return binary


def inverse_threshold(image: np.ndarray,
                     threshold: int = 200) -> np.ndarray:
    """
    Seuillage inverse (pour obstacles sombres sur fond clair).
    
    Args:
        image: Image grayscale
        threshold: Valeur seuil
        
    Returns:
        Image binaire (obstacles = 255)
    """
    _, binary = cv2.threshold(image, threshold, 255, cv2.THRESH_BINARY_INV)
    return binary


def otsu_threshold(image: np.ndarray) -> np.ndarray:
    """
    Seuillage automatique Otsu.
    
    Args:
        image: Image grayscale
        
    Returns:
        Image binaire
    """
    _, binary = cv2.threshold(image, 0, 255, 
                              cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return binary


def adaptive_threshold(image: np.ndarray,
                      block_size: int = 11,
                      C: int = 2) -> np.ndarray:
    """
    Seuillage adaptatif.
    
    Args:
        image: Image grayscale
        block_size: Taille bloc voisinage (impair)
        C: Constante soustraite de moyenne
        
    Returns:
        Image binaire
    """
    binary = cv2.adaptiveThreshold(
        image, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY,
        block_size, C
    )
    return binary
