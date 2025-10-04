import time
import numpy as np

def distance_3d(point1, point2):
    return np.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2 + (point2[2] - point1[2])**2)

def distance_2d(point1, point2):
    return np.hypot(point1, point2)

def interpolate_position(start, end, steps):
    """
    Crée une interpolation linéaire entre deux points pour un déplacement fluide.

    Args:
        start (tuple): Coordonnées (x, y) du point de départ.
        end (tuple): Coordonnées (x, y) du point d'arrivée.
        steps (int): Nombre d'étapes pour l'interpolation.

    Returns:
        list: Liste des coordonnées interpolées.
    """
    return [(start[0] + (end[0] - start[0]) * t / steps,
             start[1] + (end[1] - start[1]) * t / steps) for t in range(steps + 1)]
