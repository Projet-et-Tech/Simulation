import pybullet as p
import pybullet_data
import time
import math
import heapq
import matplotlib.pyplot as plt
import numpy as np

# ---------------- Config Particulière Arnold ----------------

#import os
#os.chdir("C:\Ar\Education\FISE2\S7\Perso\ProjetTech\ProjetTech_simu")

# ---------------- Constantes ----------------

table_height = 0.1  # epaisseur de la table (en m)
table_length = 3 # longueur de la table (en m)
table_width = 2 # largeur de la table (en m)

cell_size = 0.05 # cellule de longeur 10 cm | 1e-3 : cellule de longueur 1 mm

# ---------------- Fonctions ----------------

def marquer_obstacle(x_idx, y_idx):
    """
    Marque les cellules voisines autour d'une cellule donnée comme obstacles.

    Args:
        x_idx (int): Indice de colonne de la cellule centrale.
        y_idx (int): Indice de ligne de la cellule centrale.
    """
    for dy in range(-1, 2):  # Balayer les lignes voisines (-1, 0, 1)
        for dx in range(-1, 2):  # Balayer les colonnes voisines (-1, 0, 1)
            ny_idx = y_idx + dy  # Nouvelle ligne
            nx_idx = x_idx + dx  # Nouvelle colonne
            
            # Vérifier si les indices sont dans les limites de la grille
            if 0 <= ny_idx < grid_rows and 0 <= nx_idx < grid_cols:
                grid[ny_idx][nx_idx] = 2  # Marquer la cellule comme un obstacle



def distance(obj1, obj2):
    return math.sqrt((obj2[0] - obj1[0])**2 + (obj2[1] - obj1[1])**2 + (obj2[2]- obj1[2])**2)

def distanceAuBord(obj):
    return 2 # → A CHANGER

def estEnContact_ancien(obj1, obj2):
    if distance(obj1, obj2) < distanceAuBord(obj1) + distanceAuBord(obj2):
        return True
    else:
        return False

def estEnContact(obj1_id, obj2_id):
    points_de_contact = p.getContactPoints(obj1_id, obj2_id)
    if (len(points_de_contact) >= 1):
        return True
    else:
        return False

def coords2coordsGrille(coords): # prend en entrée une liste de coordonnées : coords = [x, y, z]
    return [int((coords[0] + table_length / 2) / cell_size), int((coords[1] + table_width / 2) / cell_size), coords[2]]

def log_position(robot_pos, step):
    """Affiche la position actuelle du robot et la prochaine cible."""
    print(f"Robot Position: {robot_pos}")
    print(f"Next Target Grid: {step}")

def interpolate_position(start, end, steps):
    """Crée une interpolation linéaire entre deux points pour un déplacement fluide."""
    return [(start[0] + (end[0] - start[0]) * t / steps,
             start[1] + (end[1] - start[1]) * t / steps) for t in range(steps + 1)]

# ---------------- Configuration de PyBullet ----------------

debug = False;

# Initialisation de PyBullet
physicsClient = p.connect(p.GUI)
assert physicsClient >= 0, "Erreur: Impossible de se connecter au serveur PyBullet."
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Acceder aux donnees par defaut de PyBullet
p.setGravity(0, 0, -9.81)

# Réinitialiser la caméra pour un zoom spécifique et une vue appropriée
p.resetDebugVisualizerCamera(cameraDistance = 2.0,  # Plus petit pour zoomer, plus grand pour éloigner
                             cameraYaw = 0,  # Rotation autour de l'axe Y (défini à 0°)
                             cameraPitch = -45,  # Inclinaison de la caméra (vue à 45°)
                             cameraTargetPosition = [0, 0, 0])

# Desactiver l'affichage des axes de debogage et autres objets inutiles
p.configureDebugVisualizer(p.COV_ENABLE_GUI, debug)  # Desactiver l'interface graphique de debogage
# Activez la simulation en temps réel
p.setRealTimeSimulation(1)

# ---------------- Création du sol ----------------

plane_id = p.loadURDF("plane.urdf")

rotationX90 = [0.7071, 0, 0, 0.7071] # Rotation de 90 sur x en quaternion
rotationZ90 = [0, 0, 0.7071, 0.7071] # Rotation de 90 sur z en quaternion
rotationXZ90 = [0.5, 0.5, 0.5, 0.5] # Rotation de 90 sur x et z en quaternion

# ---------------- Création de la table ----------------

table_position = [0, 0, table_height / 2]  # La table est legerement au-dessus du sol
table_id = p.loadURDF("urdf_models/table_eurobot2025.urdf", table_position, [0, 0, 0, 1], globalScaling = 1.0)

# ---------------- Création des conserves ----------------

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
    can_id = p.loadURDF("urdf_models/conserve.urdf", can_positions[i], rotationX90, globalScaling = 1.0)
    can_ids[i] = can_id

# ---------------- Crération des planches ----------------

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
    plank_horizontal_id = p.loadURDF("urdf_models/planche.urdf", plank_positions_horizontal[i], rotationX90, globalScaling = 1.0)
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
    plank_vertical_id = p.loadURDF("urdf_models/planche.urdf", plank_positions_vertical[i], rotationXZ90, globalScaling = 1.0)
    plank_vertical_ids[i] = plank_vertical_id

# ----------------  Paramètres du robot ----------------

# Charger le robot rectangulaire personnalise
robot_start_pos = [0, 0, table_height + 0.1]  # Position initiale du robot
robot_id = p.loadURDF("urdf_models/robot_cube.urdf", robot_start_pos, [0, 0, 0, 1], globalScaling=1.0)

# Variables pour les deplacements
robot_speed = 0.4  # Vitesse de deplacement (m/s)
robot_rotation_speed = 5  # Vitesse de rotation (rad/s)

# ---------------- Création d'une grille de coordonnées ----------------

# Dimensions de la grille
grid_rows = int(table_width / cell_size)
grid_cols = int(table_length / cell_size)

# Initialisation de la grille (centrée sur l'origine)
grid = np.zeros((grid_rows, grid_cols), dtype=int)

# Fonction pour obtenir l'indice dans une grille centrée
def position_to_grid_index(position):
    """Convertit une position physique en indices de la grille alignée avec l'origine."""
    x_index = int((position[0] / cell_size) + (grid_cols // 2))
    y_index = int((position[1] / cell_size) + (grid_rows // 2))
    return y_index, x_index

""" Remplit tous les indices entre deux coins (indices de la grille). """
def fill_indexes_between_corners(corner1, corner2):
    x1, y1, z1 = corner1
    x2, y2, z2 = corner2

    # Trouver les bornes et arrondir pour inclure tous les points
    x_min, x_max = sorted((x1, x2))
    y_min, y_max = sorted((y1, y2))
    z_min, z_max = sorted((z1, z2))

    # Générer les points alignés sur la grille
    x_points = np.arange(x_min, x_max + cell_size, cell_size)
    z_points = np.arange(z_min, z_max + cell_size, cell_size)
    y_points = np.arange(y_min, y_max + cell_size, cell_size)

    # Produire la grille complète des points (z, y, x)
    filled_points = [
        (float(x), float(y), float(z))
        for x in x_points
        for y in y_points
        for z in z_points
    ]
    return filled_points

def grid_index_to_position(grid_index):
    """Convertit les indices de la grille en coordonnées physiques."""
    y_pos = (grid_index[0] - grid_rows // 2) * cell_size
    x_pos = (grid_index[1] - grid_cols // 2) * cell_size
    return x_pos, y_pos, table_height

# ---------------- Coordonnées des obstacles ----------------

# Placer les obstacles sur la grille directement
obstacles = can_positions + plank_positions_horizontal + plank_positions_vertical

for pos in obstacles:
    y_idx, x_idx = position_to_grid_index(pos)
    if 0 <= y_idx < grid_rows and 0 <= x_idx < grid_cols:
        grid[y_idx][x_idx] = 1  # Marquer l'obstacle

for y_idx in range(grid_rows):
    for x_idx in range(grid_cols):
        if grid[y_idx][x_idx] == 1:  # Si cette cellule est déjà un obstacle
            marquer_obstacle(x_idx, y_idx)  # Marquer les voisins


# Affichage de la scene sur la grille
scene = fill_indexes_between_corners([-0.45, 0.99, 0.1], [0.45, 0.55, 0.1])
left_ramp = fill_indexes_between_corners([-0.85, 0.99, 0.1], [-0.45, 0.8, 0.1])
right_ramp = fill_indexes_between_corners([0.45, 0.99, 0.1], [0.85, 0.8, 0.1])
scene_positions = scene + left_ramp + right_ramp

for obstacle in scene_positions:
    y_idx, x_idx = position_to_grid_index(obstacle)
    if 0 <= y_idx < grid_rows and 0 <= x_idx < grid_cols:
        grid[y_idx][x_idx] = 5  # Marquer l'obstacle

# Afficher les obstacles pour vérification
for y in range(grid_rows):
    for x in range(grid_cols):
        if grid[y][x] == 1 | grid[y][x] == 2:
            print(f"Obstacle à ({x}, {y})")

# ---------------- Algorithme A* ----------------


def heuristic(a, b):
    """Fonction heuristique pour l'algorithme A* (distance de Manhattan)."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(grid, start, goal):
    """
    Algorithme A* pour trouver le chemin le plus court dans une grille.
    
    :param grid: Liste de listes représentant la grille (0 = libre, 1 = obstacle).
    :param start: Tuple (x, y) des coordonnées de départ.
    :param goal: Tuple (x, y) des coordonnées de destination.
    :return: Liste des coordonnées formant le chemin ou None si aucun chemin n'existe.
    """
    # Dimensions de la grille
    grid_rows, grid_cols = len(grid), len(grid[0])
    
    # Initialisation des structures
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)

        # Si on atteint la destination, reconstruire le chemin
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]  # Retourne le chemin dans l'ordre

        # Parcours des voisins
        for dx, dy in [(1, 0), (0, 1), (-1, 0), (0, -1)]:  # Mouvement en 4 directions
            neighbor = (current[0] + dx, current[1] + dy)
            
            # Vérifie si le voisin est dans la grille
            if not (0 <= neighbor[0] < grid_rows and 0 <= neighbor[1] < grid_cols):
                continue
            
            # Vérifie si la cellule est un obstacle
            if grid[neighbor[0]][neighbor[1]] in (1, 2, 5):
                continue
            
            # Calcul du coût pour atteindre ce voisin
            tentative_g_score = g_score[current] + 1
            
            if tentative_g_score < g_score.get(neighbor, float('inf')):
                # Mise à jour des scores et ajout à la liste ouverte
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                
                # Ajout dans open_set si pas déjà présent
                if neighbor not in [item[1] for item in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None  # Retourne None si aucun chemin trouvé


# ---------------- Affichage initial de la grille ----------------

# Points de départ et d’arrivée
start = position_to_grid_index([robot_start_pos[0],  robot_start_pos[1]])  # Exemple de départ
goal = position_to_grid_index([-1.125, 0.8])  # Exemple de cible

# Algorithme A*
path = a_star(grid, start, goal)

# Ajouter les positions de départ et d'arrivée à la grille
grid_with_path = np.copy(grid)
grid_with_path[start[0]][start[1]] = 2  # Marquer la position de départ
grid_with_path[goal[0]][goal[1]] = 3   # Marquer la position d'arrivée

# Configurer l'affichage en mode interactif
plt.ion()
fig, ax = plt.subplots(figsize=(8, 6))





# Tracer les contours des cellules de la grille
big_cell_size = cell_size * (1 / cell_size)
for i in range(grid_cols + 1):  # +1 pour inclure la dernière ligne/colonne
    # Tracer les lignes verticales
    ax.plot([i * big_cell_size - big_cell_size / 2, i * big_cell_size - big_cell_size / 2], [0 - big_cell_size / 2, grid_rows * big_cell_size - big_cell_size / 2], color='black', lw=0.4)
    
for j in range(grid_rows + 1):  # +1 pour inclure la dernière ligne/colonne
    # Tracer les lignes horizontales
    ax.plot([0 - big_cell_size / 2, grid_cols * big_cell_size - big_cell_size / 2], [j * big_cell_size - big_cell_size / 2, j * big_cell_size - big_cell_size / 2], color='black', lw=0.4)

# Ajouter une grille et des labels pour les axes
ax.grid(False)  # Désactiver la grille par défaut





im = ax.imshow(grid_with_path, cmap="viridis", interpolation="nearest", origin="lower")
im
plt.title("Carte des obstacles avec départ et arrivée")
plt.xlabel("X (indices)")
plt.ylabel("Y (indices)")
# plt.grid(True)

# Ajouter des annotations pour `start` et `goal`
ax.text(start[1], start[0], "S", color="white", ha="center", va="center", fontsize=12, fontweight="bold")
ax.text(goal[1], goal[0], "G", color="white", ha="center", va="center", fontsize=12, fontweight="bold")

# Afficher immédiatement la grille initiale
plt.draw()
plt.show()
plt.pause(0.1)

# ---------------- Gestion du chemin et déplacement du robot ----------------

if path:
    print("Chemin trouvé :", path)

    # Configuration pour l'interpolation
    interpolation_steps = 20  # Nombre d'étapes pour un déplacement fluide

    for i, step in enumerate(path):
        # Convertir les indices de grille en coordonnées physiques
        x_pos, y_pos, z_pos = grid_index_to_position(step)
        next_pos = [x_pos, y_pos]

        # Obtenir la position actuelle du robot
        robot_pos, robot_orientation = p.getBasePositionAndOrientation(robot_id)
        robot_pos_2d = robot_pos[:2]  # Ignorer la composante Z

        # Créer une interpolation pour un déplacement fluide
        interpolated_positions = interpolate_position(robot_pos_2d, next_pos, interpolation_steps)

        for inter_pos in interpolated_positions:
            # Calculer la direction
            delta_x = inter_pos[0] - robot_pos[0]
            delta_y = inter_pos[1] - robot_pos[1]
            distance_to_travel = math.sqrt(delta_x**2 + delta_y**2)

            if distance_to_travel > 0.01:
                direction_angle = math.atan2(delta_y, delta_x)
                linear_velocity = [
                    robot_speed * math.cos(direction_angle),
                    robot_speed * math.sin(direction_angle),
                    0  # Pas de mouvement en Z
                ]
                p.resetBaseVelocity(robot_id, linearVelocity=linear_velocity)

                # Mettre à jour la simulation PyBullet
                p.stepSimulation()
                time.sleep(1 / 240.0)

            # Mettre à jour la position actuelle sur la grille
            grid_with_path[step[0]][step[1]] = 4  # Marquer temporairement la position
            im.set_data(grid_with_path)
            plt.draw()

        # Réinitialiser la cellule pour éviter des doublons visuels
        grid_with_path[step[0]][step[1]] = 0
        plt.pause(0.001)

    # Arrêter le robot à la fin
    p.resetBaseVelocity(robot_id, linearVelocity=[0, 0, 0])
else:
    print("Aucun chemin trouvé.")

# Finaliser l'affichage
plt.ioff()
plt.show()


# ---------------- Boucle principale de simulation ----------------
previous_cell = None  # Stocke la cellule précédente pour vérifier les changements

while True:
    # ---------------- Recuperer la position et orientation et vitesses actuelles du robot ----------------
    start_time = time.time()
    robot_pos, robot_orientation = p.getBasePositionAndOrientation(robot_id)
    euler_orientation = p.getEulerFromQuaternion(robot_orientation)

    # ---------------- Recuperer la position actuelle des conserves  ----------------
    for i in range(len(can_ids)):
        can_positions[i] = list(p.getBasePositionAndOrientation(can_ids[i])[0])

    # ---------------- Recuperer la position actuelle des planches ----------------
    for i in range(len(plank_horizontal_ids)):
        plank_positions_horizontal[i] = list(p.getBasePositionAndOrientation(plank_horizontal_ids[i])[0])

    for i in range(len(plank_vertical_ids)):
        plank_positions_vertical[i] = list(p.getBasePositionAndOrientation(plank_vertical_ids[i])[0])

    # ---------------- Tests de contact ----------------
    start_time = time.time()
    for i in range(len(can_ids)):
        if estEnContact(robot_id, can_ids[i]):
            print(f"Contact entre {robot_id} et {can_ids[i]}\n")
    for i in range(len(plank_horizontal_ids)):
        if estEnContact(robot_id, plank_horizontal_ids[i]):
            print(f"Contact entre {robot_id} et {plank_horizontal_ids[i]}\n")
    for i in range(len(plank_vertical_ids)):
        if estEnContact(robot_id, plank_vertical_ids[i]):
            print(f"Contact entre {robot_id} et {plank_vertical_ids[i]}\n")

    end_time = time.time()
    execution_time = end_time - start_time


# ---------------- Deconnexion de PyBullet ----------------
p.disconnect()
