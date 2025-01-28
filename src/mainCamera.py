import pybullet as p
import pybullet_data
import numpy as np
import cv2

from utils.virtualCamera import init_camera, read_camera, adjust_fov_parameters, update_fov_parameters
from utils.image import imageMain
from simulation.pybullet_manager import PyBulletManager
from utils.visualization import Visualization
from config import TABLE_HEIGHT, TABLE_LENGTH, TABLE_WIDTH, CELL_SIZE, ROBOT_START_POS, ROBOT_SPEED, ROBOT_ROTATION_SPEED, DEBUG, CAN_POSITIONS, PLANK_POSITIONS_HORIZONTAL, PLANK_POSITIONS_VERTICAL
from simulation.grid import Grid
from simulation.setup import load_objects, add_scene_to_grid, initialize_grid_with_obstacles, create_environment

# ---------------- Configuration de PyBullet ----------------

debug = DEBUG

# Initialisation de PyBullet avec PyBulletManager
pybullet_manager = PyBulletManager(debug=debug)
pybullet_manager.reset_camera(distance=2.0, yaw=0, pitch=-45, target=[0, 0, 0])
pybullet_manager.set_real_time_simulation(True)

# Créer l'environnement (sol et table)
plane_id, table_id = create_environment(pybullet_manager)

# ---------------- Création du sol ----------------

rotationX90 = [0.7071, 0, 0, 0.7071] # Rotation de 90 sur x en quaternion
rotationZ90 = [0, 0, 0.7071, 0.7071] # Rotation de 90 sur z en quaternion
rotationXZ90 = [0.5, 0.5, 0.5, 0.5] # Rotation de 90 sur x et z en quaternion

# ---------------- Création des conserves ----------------

# Charger les conserves
can_ids = load_objects(pybullet_manager, "src/urdf_models/conserve.urdf", rotationX90, CAN_POSITIONS)

# ---------------- Crération des planches ----------------

# Charger les planches
plank_horizontal_ids = load_objects(pybullet_manager, "src/urdf_models/planche.urdf", rotationX90, PLANK_POSITIONS_HORIZONTAL)
plank_vertical_ids = load_objects(pybullet_manager, "src/urdf_models/planche.urdf", rotationXZ90, PLANK_POSITIONS_VERTICAL)

# ----------------  Paramètres du robot ----------------

# Charger le robot
robot_id = pybullet_manager.load_urdf("src/urdf_models/robot_cube.urdf", ROBOT_START_POS, [0, 0, 0, 1])

# Variables pour les deplacements
robot_speed = ROBOT_SPEED  # Vitesse de deplacement (m/s)
robot_rotation_speed = ROBOT_ROTATION_SPEED  # Vitesse de rotation (rad/s)

# ---------------- Boucle principale de simulation ----------------

# Creer les cameras virtuelles
cam1 = init_camera([1.5, 0, 1], [180, 135, 0])
cam2 = init_camera([-1.5, -1, 1], [0, 40, 40])
adjust_fov_parameters()

# Création de la grille et ajout des obstacles
grid = Grid()
initialize_grid_with_obstacles(grid)

# Ajout de la scène
add_scene_to_grid(grid)

# Points de départ et d'arrivée
start = grid.position_to_grid_index(ROBOT_START_POS)
goal = grid.position_to_grid_index([-1.125, 0.8, 0.1])

while True:
    update_fov_parameters()

    # Lire les cameras virtuelles
    rgb_img1 = read_camera(cam1)
    cv2.namedWindow("Camera 1", cv2.WINDOW_NORMAL)
    cv2.imshow("Camera 1", rgb_img1)
    imageMain(rgb_img1, "Cam1")

    rgb_img2 = read_camera(cam2)
    cv2.namedWindow("Camera 2", cv2.WINDOW_NORMAL)
    cv2.imshow("Camera 2", rgb_img2)
    imageMain(rgb_img2, "Cam2")

    # Ajout d'un délai pour rafraîchir la fenêtre et capturer les événements clavier
    if cv2.waitKey(1) & 0xFF == ord('q'):  # Appuyez sur 'q' pour quitter
        break

# ---------------- Deconnexion de PyBullet ----------------
cv2.destroyAllWindows()
pybullet_manager.disconnect()