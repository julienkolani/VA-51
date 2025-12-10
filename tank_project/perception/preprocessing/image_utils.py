"""
Image Utils - Utilitaires Traitement Image

Fonctions utilitaires pour traitement d'images.

Logs: [IMG_UTILS] prefix
"""

import cv2
import numpy as np
from typing import Tuple


def convert_to_grayscale(image: np.ndarray) -> np.ndarray:
    """
    Convertit image en niveaux de gris.
    
    Args:
        image: Image BGR ou RGB
        
    Returns:
        Image grayscale
    """
    if len(image.shape) == 2:
        return image  # Déjà grayscale
    
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def resize_image(image: np.ndarray,
                width: int,
                height: int,
                interpolation=cv2.INTER_LINEAR) -> np.ndarray:
    """
    Redimensionne image.
    
    Args:
        image: Image source
        width: Nouvelle largeur
        height: Nouvelle hauteur
        interpolation: Méthode interpolation
        
    Returns:
        Image redimensionnée
    """
    return cv2.resize(image, (width, height), interpolation=interpolation)


def gaussian_blur(image: np.ndarray, kernel_size: int = 5) -> np.ndarray:
    """
    Applique flou gaussien.
    
    Args:
        image: Image source
        kernel_size: Taille kernel (impair)
        
    Returns:
        Image floutée
    """
    return cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)


def morphological_open(image: np.ndarray, kernel_size: int = 5) -> np.ndarray:
    """
    Opening morphologique (erosion puis dilatation).
    
    Args:
        image: Image binaire
        kernel_size: Taille kernel
        
    Returns:
        Image traitée
    """
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    return cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)


def morphological_close(image: np.ndarray, kernel_size: int = 5) -> np.ndarray:
    """
    Closing morphologique (dilatation puis erosion).
    
    Args:
        image: Image binaire
        kernel_size: Taille kernel
        
    Returns:
        Image traitée
    """
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    return cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel)


def warp_perspective(image: np.ndarray,
                     H: np.ndarray,
                     output_size: Tuple[int, int]) -> np.ndarray:
    """
    Applique transformation perspective (warp).
    
    Args:
        image: Image source
        H: Matrice homographie 3x3
        output_size: (width, height) sortie
        
    Returns:
        Image transformée
    """
    return cv2.warpPerspective(image, H, output_size)


def equalize_histogram(image: np.ndarray) -> np.ndarray:
    """
    Égalisation histogramme.
    
    Args:
        image: Image grayscale
        
    Returns:
        Image égalisée
    """
    return cv2.equalizeHist(image)
