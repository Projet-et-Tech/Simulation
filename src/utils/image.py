"""
Module utilitaire pour la détection et la transformation d'image basée sur les marqueurs ArUco.
Permet la calibration de perspective, la localisation du robot et le calcul de distances/angles sur la table.

Principales fonctionnalités :
- Détection des marqueurs ArUco et extraction de leurs centres.
- Calcul de la matrice de perspective pour transformer l'image.
- Calcul de la distance et de l'orientation du robot par rapport à une cible.
- Affichage annoté des résultats sur l'image.
"""

import cv2
import numpy as np
from cv2 import aruco

# Initialisation du dictionnaire ArUco et des paramètres de détection
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters()
aruco_detector = aruco.ArucoDetector(marker_dict, param_markers)

# Definir les distances en centimètres
horizontal_distance_cm = 9      # Distance horizontale entre les marqueurs en cm (depuis les centres)
vertical_distance_cm = 12       # Distance verticale entre les marqueurs en cm (depuis les centres)
horizontal_margin_cm = 2 + 4    # Marge horizontale en cm (+2cm depuis le centre)
vertical_margin_cm = 2 + 3      # Marge verticale en cm (+2cm depuis le centre)
scale_factor = 2                # Facteur d'echelle

# IDs des marqueurs d'intérêt (coins de la table)
corner_top_left_id = 22
corner_top_right_id = 20
corner_bottom_left_id = 23
corner_bottom_right_id = 21

# ID du robot et de la cible (à adapter selon le contexte)
robot_id = 2
target_id = 6

def sort_by_target_order(marker_centers):
    """
    Trie les centres des marqueurs d'intérêt selon l'ordre attendu pour la perspective.
    Retourne un tableau de 4 points (float32) ou None si un coin manque.
    """
    try:
        return np.float32([
            marker_centers[corner_top_left_id],    # Coin en haut à gauche
            marker_centers[corner_top_right_id],   # Coin en haut à droite
            marker_centers[corner_bottom_right_id], # Coin en bas à droite
            marker_centers[corner_bottom_left_id]  # Coin en bas à gauche
        ])
    except KeyError:
        return None  # Si un coin est manquant


def calculate_pixels_per_cm(corners_in_order):
    """
    Calcule le rapport pixels/cm à partir des distances entre les coins détectés.
    Utilisé pour convertir les distances pixels <-> cm.
    """
    horizontal_dist_pixels = np.linalg.norm(corners_in_order[0] - corners_in_order[1])  # Distance top-left -> top-right
    vertical_dist_pixels = np.linalg.norm(corners_in_order[1] - corners_in_order[2])    # Distance top-right -> bottom-right
    pixels_per_cm_hor = horizontal_dist_pixels / horizontal_distance_cm
    pixels_per_cm_ver = vertical_dist_pixels / vertical_distance_cm
    return (pixels_per_cm_hor + pixels_per_cm_ver) / 2


def apply_perspective_transform(frame, marker_centers, perspective_matrix=None, pixels_per_cm=None):
    """
    Applique une transformation de perspective à l'image pour obtenir une vue "vue de dessus" calibrée.
    Si la matrice de perspective n'est pas fournie, elle est calculée à partir des coins détectés.
    Retourne l'image transformée, la matrice de perspective, le ratio pixels/cm et les points de destination.
    """
    corners_in_order = sort_by_target_order(marker_centers)
    if corners_in_order is None and perspective_matrix is None:
        return frame, perspective_matrix, pixels_per_cm  # Retournez l'image d'origine si un coin est manquant et matrice inconnue

    # Si la definition n'existe pas encore, la calculer
    if pixels_per_cm is None:
        pixels_per_cm = calculate_pixels_per_cm(corners_in_order)

    # Calculer les distances en pixels
    scaled_horizontal_distance = horizontal_distance_cm * pixels_per_cm * scale_factor
    scaled_vertical_distance = vertical_distance_cm * pixels_per_cm * scale_factor
    scaled_horizontal_margin = horizontal_margin_cm * pixels_per_cm * scale_factor
    scaled_vertical_margin = vertical_margin_cm * pixels_per_cm * scale_factor

    # Si la matrice n'existe pas encore, la calculer
    if perspective_matrix is None:
        # Points de destination en pixels : Ordre classique des coins
        dst_pts = np.float32([
            [scaled_horizontal_margin, scaled_vertical_margin],  # Coin en haut-gauche
            [scaled_horizontal_margin + scaled_horizontal_distance, scaled_vertical_margin],  # Coin en haut-droite
            [scaled_horizontal_margin + scaled_horizontal_distance, scaled_vertical_margin + scaled_vertical_distance],  # Coin en bas-droit
            [scaled_horizontal_margin, scaled_vertical_margin + scaled_vertical_distance]  # Coin en bas-gauche
        ])
        perspective_matrix = cv2.getPerspectiveTransform(corners_in_order, dst_pts)

    # Application de la transformation de perspective
    warped_image = cv2.warpPerspective(
        frame, perspective_matrix,
        (int(scaled_horizontal_distance + 2 * scaled_horizontal_margin),
        int(scaled_vertical_distance + 2 * scaled_vertical_margin))
    )
    return warped_image, perspective_matrix, pixels_per_cm, dst_pts


def detect_aruco_markers(frame, aruco_detector):
    """
    Détecte les marqueurs ArUco dans une image.
    Retourne les coins détectés et les IDs.
    """
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, _ = aruco_detector.detectMarkers(gray_frame)
    return marker_corners, marker_IDs


def process_markers(frame, marker_IDs, marker_corners):
    """
    Traite les marqueurs détectés : dessine les contours et calcule les centres.
    Retourne un dictionnaire {id: centre}.
    """
    marker_centers = {}
    if marker_IDs is not None:
        for ids, corners in zip(marker_IDs, marker_corners):
            corners = corners.reshape(4, 2).astype(int)
            # Dessine le contour du marqueur
            cv2.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA)
            center = np.mean(corners, axis=0).astype(int)
            marker_centers[ids[0]] = center
            # Marque le centre du marqueur
            cv2.circle(frame, tuple(center), 5, (0, 255, 0), -1)
    return marker_centers

def draw_rotated_cross(frame, center, angle, length=30):
    """
    Dessine une croix orientée autour du centre donné.
    La branche vers le haut est rouge, les autres sont jaunes.
    """
    sin_a = np.sin(angle)
    cos_a = np.cos(angle)
    # Calcul des directions pour chaque branche de la croix
    directions = [
        (-cos_a * length, -sin_a * length),  # Haut (rouge)
        (sin_a * length, -cos_a * length),   # Droite (jaune)
        (cos_a * length, sin_a * length),    # Bas (jaune)
        (-sin_a * length, cos_a * length)    # Gauche (jaune)
    ]

    # Dessiner la branche vers le haut (rouge)
    pt1 = (int(center[0] + directions[0][0]), int(center[1] + directions[0][1]))
    cv2.line(frame, center, pt1, (0, 0, 255), 2)  # Rouge

    # Dessiner les autres branches (jaune)
    for i in range(1, 4):
        pt = (int(center[0] + directions[i][0]), int(center[1] + directions[i][1]))
        cv2.line(frame, center, pt, (0, 200, 200), 2)


def calculate_distance_and_rotation(center, target, robot_angle, pixels_per_cm):
    """
    Calcule la distance et l'angle relatif entre le robot et la cible.
    Retourne la distance (en cm si pixels_per_cm fourni) et l'angle relatif (radians).
    """
    delta_x = target[0] - center[0]
    delta_y = target[1] - center[1]
    distance = np.sqrt(delta_x**2 + delta_y**2)

    # Convertit les pixels en cm
    if pixels_per_cm is not None:
        distance = distance / pixels_per_cm
    
    # Calcul de l'angle vers le point cible (relatif a l'horizontale)
    target_angle = np.arctan2(delta_y, delta_x)
    
    # Calcul de l'angle de rotation relatif a l'orientation du robot
    relative_angle = target_angle - robot_angle
    # Normalise l'angle entre -pi et pi
    relative_angle = np.arctan2(np.sin(relative_angle), np.cos(relative_angle))
    return distance, relative_angle

def calculate_position(corner_bottom_left, robot_center, pixels_per_cm):
    """
    Calcule la position du robot par rapport au coin bas gauche en cm.
    Retourne (horizontal_position, vertical_position).
    """
    delta_x_pixels = robot_center[0] - corner_bottom_left[0]
    delta_y_pixels = robot_center[1] - corner_bottom_left[1]

    # Convertir les distances en centimetres
    horizontal_position = delta_x_pixels / pixels_per_cm
    vertical_position = delta_y_pixels / pixels_per_cm

    return horizontal_position, vertical_position


def process_robot_marker(frame, marker_IDs, marker_corners, marker_centers, pixels_per_cm=None, last_corner_bottom_left=None):
    """
    Traite le marqueur du robot : dessine la croix orientée, calcule la distance/angle à la cible,
    affiche la position relative si possible.
    Retourne l'image annotée.
    """
    annotated_frame = frame.copy()

    robot_center = marker_centers.get(robot_id)
    target_center = marker_centers.get(target_id)

    if marker_IDs is not None:
        for ids, corners in zip(marker_IDs, marker_corners):
            if ids[0] == robot_id:
                corners = corners.reshape(4, 2).astype(int)
                if robot_center is None:
                    continue  # Si le centre n'est pas trouve, passer a l'iteration suivante

                top_left = corners[0]
                top_right = corners[1]
                # Calcul de l'orientation du robot (angle du haut)
                delta_y = top_right[1] - top_left[1]
                delta_x = top_right[0] - top_left[0]
                robot_angle = np.arctan2(delta_y, delta_x)

                # Dessiner la croix orientee au centre du marqueur
                draw_rotated_cross(annotated_frame, tuple(robot_center), robot_angle)
                # Si une cible est détectée, calcule la distance et l'angle relatif
                if target_center is not None:
                    distance, relative_angle = calculate_distance_and_rotation(robot_center, target_center, robot_angle, pixels_per_cm)

                    # Dessiner une ligne entre le robot et le point cible
                    cv2.line(annotated_frame, tuple(robot_center), target_center, (0, 255, 0), 2)
                    if pixels_per_cm is None:
                        cv2.putText(annotated_frame, f"Distance: {int(distance)} pixels", (10, 30), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    else:
                        cv2.putText(annotated_frame, f"Distance: {distance:.2f} cm", (10, 30), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(annotated_frame, f"Angle : {np.degrees(relative_angle):.2f} deg", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                # Affiche la position relative si possible
                if last_corner_bottom_left is not None:
                    horizontal_position, vertical_position = calculate_position(last_corner_bottom_left, robot_center, pixels_per_cm)
                    cv2.putText(annotated_frame, f"Position: [{horizontal_position:.2f};{-vertical_position:.2f}] cm", (10, 90), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return annotated_frame

def imageMain(frame, name):
    """
    Fonction principale pour traiter une image :
    - Détecte les marqueurs ArUco
    - Applique la transformation de perspective si possible
    - Annote l'image avec la position/orientation du robot
    Affiche le résultat dans une fenêtre OpenCV.
    """
    # Variables pour la calibration
    perspective_matrix = None
    pixels_per_cm = None
    last_corner_bottom_left = None

    # Détection des marqueurs sur l'image brute
    marker_corners, marker_IDs = detect_aruco_markers(frame, aruco_detector)
    marker_centers = process_markers(frame, marker_IDs, marker_corners)

    # Vérifie si tous les coins sont détectés pour la calibration
    required_ids = [corner_top_left_id, corner_top_right_id, corner_bottom_left_id, corner_bottom_right_id]
    centers = {id: marker_centers.get(id) for id in required_ids}
        
    # Si tous les coins nécessaires sont détectés, recalculer la matrice
    if all(centers[id] is not None for id in required_ids):
        transformed_frame, perspective_matrix, pixels_per_cm, _ = apply_perspective_transform(frame, marker_centers)
    else:
        # Sinon, utiliser la matrice précédente si disponible
        transformed_frame, perspective_matrix, pixels_per_cm = apply_perspective_transform(frame, marker_centers, perspective_matrix, pixels_per_cm)

    # Traite le marqueur du robot et affiche la croix orientée
    annotated_frame = process_robot_marker(frame, marker_IDs, marker_corners, marker_centers, pixels_per_cm, last_corner_bottom_left)

    # Affichage de l'image transformée ou annotée
    if perspective_matrix is not None:
        window_name = name + " | Perspective Transformed"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.imshow(window_name, transformed_frame)
        return transformed_frame, marker_centers
    else:
        window_name = name + " | Aruco Marker Detection"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.imshow(window_name, annotated_frame)