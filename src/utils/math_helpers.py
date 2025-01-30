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

def position_to_grid_index(position, x_dim, y_dim, cell_size):
    x_idx = int((position[0] / cell_size) + (x_dim // 2))
    y_idx = int((position[1] / cell_size) + (y_dim // 2))
    return y_idx, x_idx

def mark_can_on_grid(center, radius, x_dim, y_dim, cell_size):
    y_idx, x_idx = position_to_grid_index(center, x_dim, y_dim, cell_size)
    radius_cells = int(radius / cell_size)

    ox, oy = [], []
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            ny, nx = y_idx + dy, x_idx + dx
            if 0 <= ny < y_dim and 0 <= nx < x_dim:
                if np.sqrt(dx**2 + dy**2) <= radius_cells:
                    ox.append(nx)
                    oy.append(ny)
    return oy, ox

def in_bound(pos, x_dim, y_dim):
    x, y = pos
    return 0 <= x < x_dim and 0 <= y < y_dim