import pybullet as p
import pybullet_data
import numpy as np
import cv2

from utils.virtualCamera import init_camera, read_camera, adjust_fov_parameters, update_fov_parameters
from utils.image import imageMain

# ---------------- Constantes ----------------

table_height = 0.1  # epaisseur de la table (en m)
table_length = 3 # longueur de la table (en m)
table_width = 2 # largeur de la table (en m)

# ---------------- Configuration de PyBullet ----------------

debug = False;

# Initialisation de PyBullet
physicsClient = p.connect(p.GUI)
assert physicsClient >= 0, "Erreur: Impossible de se connecter au serveur PyBullet."
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Acceder aux donnees par defaut de PyBullet
p.setGravity(0, 0, -9.81)

# R�initialiser la cam�ra pour un zoom sp�cifique et une vue appropri�e
p.resetDebugVisualizerCamera(cameraDistance = 2.0,  # Plus petit pour zoomer, plus grand pour �loigner
                             cameraYaw = 0,  # Rotation autour de l'axe Y (d�fini � 0�)
                             cameraPitch = -45,  # Inclinaison de la cam�ra (vue � 45�)
                             cameraTargetPosition = [0, 0, 0])

# Desactiver l'affichage des axes de debogage et autres objets inutiles
p.configureDebugVisualizer(p.COV_ENABLE_GUI, debug)  # Desactiver l'interface graphique de debogage
# Activez la simulation en temps r�el
p.setRealTimeSimulation(1)

# ---------------- Cr�ation du sol ----------------

plane_id = p.loadURDF("plane.urdf")

rotationX90 = [0.7071, 0, 0, 0.7071] # Rotation de 90 sur x en quaternion
rotationZ90 = [0, 0, 0.7071, 0.7071] # Rotation de 90 sur z en quaternion
rotationXZ90 = [0.5, 0.5, 0.5, 0.5] # Rotation de 90 sur x et z en quaternion

# ---------------- Cr�ation de la table ----------------

table_position = [0, 0, table_height / 2]  # La table est legerement au-dessus du sol
table_id = p.loadURDF("src/urdf_models/table_eurobot2025.urdf", table_position, [0, 0, 0, 1], globalScaling = 1.0)

# ---------------- Cr�ation des conserves ----------------

can_positions = [
                # De gauche a droite de haut en bas

                 # Moitie gauche
                 [-1.425, 0.475, 0.1],
                 [-1.425, 0.375, 0.1],
                 [-1.425, 0.275, 0.1],
                 [-1.425, 0.175, 0.1],

                 [-1.425, -0.425, 0.1],
                 [-1.425, -0.525, 0.1],
                 [-1.425, -0.625, 0.1],
                 [-1.425, -0.725, 0.1],

                 [-0.825, 0.725, 0.1],
                 [-0.725, 0.725, 0.1],
                 [-0.625, 0.725, 0.1],
                 [-0.525, 0.725, 0.1],

                 [-0.875, -0.75, 0.1],
                 [-0.775, -0.75, 0.1],
                 [-0.675, -0.75, 0.1],
                 [-0.575, -0.75, 0.1],

                 [-0.55, -0.05, 0.1],
                 [-0.45, -0.05, 0.1],
                 [-0.35, -0.05, 0.1],
                 [-0.25, -0.05, 0.1],

                 # Moitie droite
                 [1.425, 0.475, 0.1],
                 [1.425, 0.375, 0.1],
                 [1.425, 0.275, 0.1],
                 [1.425, 0.175, 0.1],

                 [1.425, -0.425, 0.1],
                 [1.425, -0.525, 0.1],
                 [1.425, -0.625, 0.1],
                 [1.425, -0.725, 0.1],

                 [0.825, 0.725, 0.1],
                 [0.725, 0.725, 0.1],
                 [0.625, 0.725, 0.1],
                 [0.525, 0.725, 0.1],

                 [0.875, -0.75, 0.1],
                 [0.775, -0.75, 0.1],
                 [0.675, -0.75, 0.1],
                 [0.575, -0.75, 0.1],

                 [0.55, -0.05, 0.1],
                 [0.45, -0.05, 0.1],
                 [0.35, -0.05, 0.1],
                 [0.25, -0.05, 0.1],
                 ]
can_ids = [0] * len(can_positions)
for i in range(len(can_positions)):
    can_id = p.loadURDF("src/urdf_models/conserve.urdf", can_positions[i], rotationX90, globalScaling = 1.0)
    can_ids[i] = can_id

# ---------------- Cr�ration des planches ----------------

plank_positions_horizontal = [
                  # De gauche a droite de haut en bas

                    # Moitie gauche
                    [-0.675, 0.725, 0.15],
                    [-0.675, 0.725, 0.2],

                    [-0.725, -0.75, 0.15],
                    [-0.725, -0.75, 0.2],

                    [-0.4, -0.05, 0.15],
                    [-0.4, -0.05, 0.2],

                    # Moitie droite
                    [0.675, 0.725, 0.15],
                    [0.675, 0.725, 0.2],

                    [0.725, -0.75, 0.15],
                    [0.725, -0.75, 0.2],

                    [0.4, -0.05, 0.15],
                    [0.4, -0.05, 0.2],
                  ]
plank_horizontal_ids = [0] * len(plank_positions_horizontal)
for i in range(len(plank_positions_horizontal)):
    plank_horizontal_id = p.loadURDF("src/urdf_models/planche.urdf", plank_positions_horizontal[i], rotationX90, globalScaling = 1.0)
    plank_horizontal_ids[i] = plank_horizontal_id

plank_positions_vertical = [
                    # De gauche a droite de haut en bas

                    # Moitie gauche
                    [-1.425, 0.325, 0.15],
                    [-1.425, 0.325, 0.2],

                    [-1.425, -0.6, 0.15],
                    [-1.425, -0.6, 0.2],

                    # Moitie droite
                    [1.425, 0.325, 0.15],
                    [1.425, 0.325, 0.2],

                    [1.425, -0.6, 0.15],
                    [1.425, -0.6, 0.2],
                  ]
plank_vertical_ids = [0] * len(plank_positions_vertical)
for i in range(len(plank_positions_vertical)):
    plank_vertical_id = p.loadURDF("src/urdf_models/planche.urdf", plank_positions_vertical[i], rotationXZ90, globalScaling = 1.0)
    plank_vertical_ids[i] = plank_vertical_id

# ----------------  Param�tres du robot ----------------

# Charger le robot rectangulaire personnalise
robot_start_pos = [0, 0, table_height + 0.1]  # Position initiale du robot
robot_id = p.loadURDF("src/urdf_models/robot_cube.urdf", robot_start_pos, [0, 0, 0, 1], globalScaling=1.0)

# Variables pour les deplacements
robot_speed = 0.5  # Vitesse de deplacement (m/s)
robot_rotation_speed = 5  # Vitesse de rotation (rad/s)

# ---------------- Boucle principale de simulation ----------------

# Creer les cameras virtuelles
cam1 = init_camera([1.5, 0, 1], [180, 135, 0])
cam2 = init_camera([-1.5, -1, 1], [0, 40, 40])
adjust_fov_parameters()

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

    # Ajout d'un d�lai pour rafra�chir la fen�tre et capturer les �v�nements clavier
    if cv2.waitKey(1) & 0xFF == ord('q'):  # Appuyez sur 'q' pour quitter
        break

# ---------------- Deconnexion de PyBullet ----------------
cv2.destroyAllWindows()
p.disconnect()