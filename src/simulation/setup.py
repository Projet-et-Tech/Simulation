import pybullet as p
#from simulation.pybullet_manager import PyBulletManager
from config import TABLE_HEIGHT, CAN_POSITIONS, PLANK_POSITIONS_HORIZONTAL, PLANK_POSITIONS_VERTICAL
import config as config

def load_objects(pybullet_manager, object_urdf, rotation, positions):
    """
    Charge une liste d'objets URDF à des positions données avec une rotation donnée.

    :param pybullet_manager: Instance de PyBulletManager pour charger les objets.
    :param object_urdf: Chemin vers le fichier URDF de l'objet.
    :param rotation: Rotation à appliquer aux objets.
    :param positions: Liste des positions où charger les objets.
    :return: Liste des identifiants des objets chargés.
    """
    return [pybullet_manager.load_urdf(object_urdf, pos, rotation) for pos in positions]


def create_environment(pybullet_manager):
    """
    Crée l'environnement en chargeant le sol et la table.

    :param pybullet_manager: Instance de PyBulletManager pour charger les objets.
    :return: Tuple contenant les identifiants du sol et de la table.
    """
    plane_id = pybullet_manager.load_urdf("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
    table_position = [0, 0, TABLE_HEIGHT / 2]
    table_id = pybullet_manager.load_urdf("src/urdf_models/table_eurobot2025.urdf", table_position, [0, 0, 0, 1])
    return plane_id, table_id 


def edge(xmin, xmax, ymin, ymax):
    # Create the coordinates for the edges
    ox = list(range(xmin, xmax + 1)) + [xmax] * (ymax - ymin + 1) + list(range(xmax, xmin - 1, -1)) + [xmin] * (ymax - ymin + 1)
    oy = [ymin] * (xmax - xmin + 1) + list(range(ymin, ymax + 1)) + [ymax] * (xmax - xmin + 1) + list(range(ymax, ymin - 1, -1))
    return ox, oy

def map_borders(x_dim, y_dim):
    border_width = int(config.ROBOT_RADIUS / config.CELL_SIZE)
    ox, oy = [], []
    for w in range(border_width):
        ox_w, oy_w = edge(w, x_dim-w, w, y_dim-w)
        ox += ox_w
        oy += oy_w
    return ox, oy

def initialize_map(grid):
    x_dim = grid.grid_cols
    y_dim = grid.grid_rows
    cell_size = grid.cell_size
    # Border obstacle positions
    ox, oy = map_borders(x_dim, y_dim)

    scene_corner1 = [-0.45, 1.0]  # Bottom-left corner of the scene
    scene_corner2 = [0.45, 0.55]  # Top-right corner of the scene

    y_corner_1, x_corner_1 = grid.position_to_grid_index(scene_corner1)
    y_corner_2, x_corner_2 = grid.position_to_grid_index(scene_corner2)

    x_min, x_max = min(x_corner_1, x_corner_2), max(x_corner_1, x_corner_2)
    y_min, y_max = min(y_corner_1, y_corner_2), max(y_corner_1, y_corner_2)

    for x in range(x_min, x_max + 1):
        for y in range(y_min, y_max + 1):
            ox.append(x)
            oy.append(y)

    return ox, oy

def initialize_cans(grid, can_radius):
    x_dim = grid.grid_cols
    y_dim = grid.grid_rows
    cell_size = grid.cell_size
    ox_cans, oy_cans = [], []

    for pos in CAN_POSITIONS:
        oy_can, ox_can = grid.mark_can_on_grid(pos, can_radius)
        ox_cans.extend(ox_can)
        oy_cans.extend(oy_can)

    return ox_cans, oy_cans