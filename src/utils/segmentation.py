"""
Module de segmentation couleur et visualisation 3D des obstacles pour la simulation.
Permet de détecter des objets par couleur, d'extraire leurs contours/corners, et de les projeter en 3D dans PyBullet.
Utilisé pour la perception des obstacles sur la table à partir d'une image caméra.
"""

import cv2
import numpy as np

from utils.perspectiveCorrection import convert_2D_to_3D
from config import TABLE_LENGTH, TABLE_WIDTH, PAMI_HEIGHT

def colorSegmentation(image, color_bounds, contour_color=(0, 255, 0), corner_color=(0, 0, 255)):
    """
    Segmente l'image selon des plages de couleurs HSV et extrait les coins des objets détectés.
    Affiche les résultats et retourne un dictionnaire des coins pour chaque objet détecté.

    Args:
        image (ndarray): Image d'entrée (BGR).
        color_bounds (dict): Dictionnaire {nom_couleur: (borne_basse, borne_haute)} en HSV.
        contour_color (tuple): Couleur pour dessiner les contours.
        corner_color (tuple): Couleur pour dessiner les coins.

    Returns:
        dict: Dictionnaire {nom_objet: [corners]}.
    """
    def segment_color(image, lower_bound, upper_bound):
        # Conversion en HSV et seuillage couleur
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        # Suppression du bruit par ouverture morphologique
        kernel = np.ones((16, 16), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return mask
    
    # Définition des plages de couleurs (en HSV)
    color_bounds = {
        "planche": (np.array([10, 100, 20]), np.array([20, 255, 200]))
    }
    
    masks = {color: segment_color(image, bounds[0], bounds[1]) for color, bounds in color_bounds.items()}

    object_corners = {}  # Dictionnaire pour stocker les coins des objets

    # Détection et annotation des objets pour chaque couleur
    for color, mask in masks.items():
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        result = image.copy()
        
        for i, contour in enumerate(contours):
            M = cv2.moments(contour)
            if M["m00"] != 0:
                # Calcul du centre de gravité du contour
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                # Dessin du contour
                cv2.drawContours(result, [contour], -1, contour_color, 2)
                # Annotation du nom de l'objet
                cv2.putText(result, f"{color}-{i}", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                # Marqueur croix au centre
                cv2.drawMarker(result, (cx, cy), corner_color, cv2.MARKER_CROSS, 10, 2)

                # Détection des coins du contour (approximation polygonale)
                epsilon = 0.02 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                # Stockage des coins
                object_corners[f"{color}-{i}"] = [tuple(point.ravel()) for point in approx]

                # Dessin des coins
                for point in approx:
                    x, y = point.ravel()
                    cv2.circle(result, (x, y), 5, corner_color, -1)

        # Affichage du résultat pour chaque couleur
        cv2.namedWindow(f"Detected {color} {contour_color}", cv2.WINDOW_NORMAL)
        cv2.imshow(f"Detected {color} {contour_color}", result)

    # Affichage console des coins détectés
    for obj, corners in object_corners.items():
        print(f"{obj}: {corners}")

    return object_corners

def drawObstacles3D(object_corners, image, cam_position, line_color=(0, 1, 0)):
    """
    Projette les coins détectés en 3D et dessine les obstacles dans PyBullet.
    Trace des lignes entre les coins pour chaque objet détecté.

    Args:
        object_corners (dict): Dictionnaire {nom_objet: [corners]}.
        image (ndarray): Image d'origine (pour la taille).
        cam_position (tuple): Position de la caméra.
        line_color (tuple): Couleur des lignes dans PyBullet.
    """
    # Conversion des coins 2D en coordonnées 3D et dessin des lignes
    for obj, corners in object_corners.items():
        transformed_corners = []
        
        for corner in corners:
            x_2d, y_2d = corner
            # Conversion 2D->3D (attention à l'ordre y, x)
            x_3d, y_3d = convert_2D_to_3D(
                y_2d, x_2d, image, cam_position, PAMI_HEIGHT
            )
            transformed_corners.append((x_3d, y_3d, PAMI_HEIGHT))

        # Trace des lignes entre chaque coin (ferme le polygone)
        for j in range(len(transformed_corners)):
            start_point = transformed_corners[j]
            end_point = transformed_corners[(j + 1) % len(transformed_corners)]
