import math

def distance(point1, point2):
    """
    Calcule la distance euclidienne entre deux points 3D.

    Args:
        point1 (tuple): Coordonnées (x, y, z) du premier point.
        point2 (tuple): Coordonnées (x, y, z) du second point.

    Returns:
        float: Distance euclidienne entre les deux points.
    """
    return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2 + (point2[2] - point1[2])**2)

def distance_2d(point1, point2):
    """
    Calcule la distance euclidienne entre deux points 2D.

    Args:
        point1 (tuple): Coordonnées (x, y) du premier point.
        point2 (tuple): Coordonnées (x, y) du second point.

    Returns:
        float: Distance euclidienne entre les deux points.
    """
    return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

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