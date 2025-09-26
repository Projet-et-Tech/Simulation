"""
Module de détection et d'analyse des marqueurs ArUco dans une image.
Fonctions principales :
- highlightDetected : met en évidence les marqueurs ArUco spécifiés et retourne leurs centres.
- drawCross : dessine une croix orientée sur un marqueur.
- getDirectionAndDistance : calcule l'angle, la distance et la direction entre deux marqueurs.
"""

import cv2
import numpy as np

def highlightDetected(frame, allowed_ids):
    """
    Détecte et met en évidence uniquement les marqueurs ArUco spécifiés dans l'image.
    Les marqueurs autorisés sont entourés en jaune et leur centre est marqué.
    Args:
        frame (ndarray): Image d'entrée (BGR).
        allowed_ids (list/set): IDs des marqueurs à mettre en évidence.
    Returns:
        frame (ndarray): Image annotée.
        centers (dict): Dictionnaire {id: centre}.
        corners (list): Liste des coins détectés.
    """
    # Convertir l'image en niveaux de gris
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Charger le dictionnaire ArUco 4x4_50
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    # Paramètres de détection optimisés
    parameters = cv2.aruco.DetectorParameters()
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    # Détection des marqueurs dans l'image
    corners, ids, _ = aruco_detector.detectMarkers(gray)

    # Dictionnaire pour stocker les centres des marqueurs
    centers = {}

    # Si des marqueurs sont détectés
    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id in allowed_ids:

                # Extraire les 4 coins du marqueur
                corner_pts = corners[i][0]
                corner_pts = corner_pts.astype(int)

                # Calculer le centre du marqueur
                center = np.mean(corner_pts, axis=0).astype(int)
                centers[marker_id] = tuple(center)

                # Dessiner le contour autour du marqueur
                cv2.polylines(frame, [corner_pts], isClosed=True, color=(0, 255, 255), thickness=3)

                # Afficher l'ID du marqueur
                text_position = tuple(corner_pts[0])
                cv2.putText(frame, str(marker_id), text_position, cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=1, color=(0, 255, 255), thickness=2, lineType=cv2.LINE_AA)

                # Dessiner le centre du marqueur en vert
                cv2.circle(frame, tuple(center), 5, (0, 255, 0), -1)

    return frame, centers, corners  # Retourne l'image annotée, les centres et les coins


def drawCross(frame, center, corners, length=30):
    """
    Dessine une croix orientée sur le marqueur à partir de ses coins.
    La branche vers le haut est rouge, les autres sont jaunes.
    Args:
        frame (ndarray): Image sur laquelle dessiner.
        center (tuple): Centre du marqueur.
        corners (ndarray): Coins du marqueur (4,2).
        length (int): Longueur des branches.
    """
    top_left = corners[0]
    top_right = corners[1]

    # Calculer l'orientation du marqueur (angle entre top_left et top_right)
    delta_y = top_right[1] - top_left[1]
    delta_x = top_right[0] - top_left[0]
    angle = np.arctan2(delta_y, delta_x)

    sin_angle = np.sin(angle)
    cos_angle = np.cos(angle)

    # Coordonnées pour les quatre directions de la croix
    directions = [
        (-cos_angle * length, -sin_angle * length),  # Haut
        (sin_angle * length, -cos_angle * length),   # Droite
        (cos_angle * length, sin_angle * length),    # Bas
        (-sin_angle * length, cos_angle * length)    # Gauche
    ]

    # Dessiner la branche vers le haut (rouge)
    branche1 = (int(center[0] + directions[0][0]), int(center[1] + directions[0][1]))
    cv2.line(frame, center, branche1, (0, 0, 255), 2)

    # Dessiner les autres branches (jaunes)
    for i in range(1, 4):
        branche = (int(center[0] + directions[i][0]), int(center[1] + directions[i][1]))
        cv2.line(frame, center, branche, (0, 200, 200), 2)


def getDirectionAndDistance(centerOrigin, cornersOrigin, centerTarget, frame_width, frame_height, meters_per_pixel):
    """
    Calcule l'angle, la distance et la direction entre deux points (origine et cible) en mètres.
    L'angle est relatif à la branche rouge (haut) de la croix du marqueur d'origine.
    Args:
        centerOrigin (tuple): Centre du marqueur d'origine (x, y).
        cornersOrigin (ndarray): Coins du marqueur d'origine.
        centerTarget (tuple): Centre du marqueur cible (x, y).
        frame_width (int): Largeur de l'image.
        frame_height (int): Hauteur de l'image.
        meters_per_pixel (float): Facteur de conversion pixel->mètre.
    Returns:
        angle (float): Angle en degrés entre l'origine et la cible.
        distance_meters (float): Distance en mètres.
        direction (str): "Left" ou "Right".
    """
    # Vérification des types d'entrée
    if not isinstance(centerOrigin, (tuple, list, np.ndarray)) or not isinstance(centerTarget, (tuple, list, np.ndarray)):
        raise ValueError("centerOrigin and centerTarget must be tuples, lists, or arrays.")

    # Calcul de la distance en pixels
    distance_pixels = np.linalg.norm(np.array(centerTarget) - np.array(centerOrigin))

    # Conversion en mètres
    distance_meters = distance_pixels * meters_per_pixel

    # Calcul de l'angle relatif à la branche rouge (haut)
    red_crossbar_vector = np.array([0, -1])  # Direction haut en image
    vectorTarget = np.array(centerTarget) - np.array(centerOrigin)
    angle = np.arctan2(vectorTarget[1], vectorTarget[0]) - np.arctan2(red_crossbar_vector[1], red_crossbar_vector[0])
    angle = np.degrees(angle)  # Conversion en degrés

    # Normalisation de l'angle dans [-180, 180]
    angle = (angle + 180) % 360 - 180

    # Détermination de la direction
    if angle > 0:
        direction = "Right"
    else:
        direction = "Left"

    return angle, distance_meters, direction