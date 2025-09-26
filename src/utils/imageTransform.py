"""
Module utilitaire pour la calibration et la transformation de perspective d'une image à partir de marqueurs ArUco.
Permet de corriger la perspective d'une image de caméra pour obtenir une vue "vue de dessus" de la table de jeu.
Inclut la détection automatique ou manuelle des coins, le calcul du ratio pixels/cm, et la sauvegarde/rechargement de la matrice de transformation.
"""

import cv2
import numpy as np
from cv2 import aruco

from config import HORIZONTAL_DISTANCE_CM, VERTICAL_DISTANCE_CM, HORIZONTAL_MARGIN_CM, VERTICAL_MARGIN_CM, SCALE_FACTOR, CORNER_TOP_LEFT_ID, CORNER_TOP_RIGHT_ID, CORNER_BOTTOM_LEFT_ID, CORNER_BOTTOM_RIGHT_ID

from utils.perspectiveCorrection import getCornersFromUserClick
from utils.arucoDetection import highlightDetected

# ---------------------- ARUCO MARKER SETUP ---------------------- #

# Initialisation du dictionnaire ArUco et du détecteur
marker_dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters()
aruco_detector = aruco.ArucoDetector(marker_dictionary, param_markers)

# ---------------------- FUNCTIONS ---------------------- #

def calculate_pixels_per_cm(corners_in_order):
    """
    Calcule le ratio pixels/cm à partir des coins détectés.
    Prend en compte les distances horizontales et verticales connues de la table.
    """
    # Calcul des distances en pixels entre les coins
    horizontal_dist_pixels = np.linalg.norm(corners_in_order[0] - corners_in_order[1])  # Top-Left -> Top-Right
    vertical_dist_pixels = np.linalg.norm(corners_in_order[1] - corners_in_order[2])    # Top-Right -> Bottom-Right

    # Conversion en pixels/cm
    pixels_per_cm_hor = horizontal_dist_pixels / HORIZONTAL_DISTANCE_CM
    pixels_per_cm_ver = vertical_dist_pixels / VERTICAL_DISTANCE_CM

    # Retourne la moyenne des deux ratios
    return (pixels_per_cm_hor + pixels_per_cm_ver) / 2

def sort_by_target_order(marker_centers):
    """
    Trie les centres des marqueurs d'intérêt selon l'ordre attendu pour la perspective.
    Retourne un tableau de 4 points (float32) ou None si un coin manque.
    """
    try:
        return np.float32([
            marker_centers[CORNER_TOP_LEFT_ID],     # Coin en haut à gauche
            marker_centers[CORNER_TOP_RIGHT_ID],    # Coin en haut à droite
            marker_centers[CORNER_BOTTOM_RIGHT_ID], # Coin en bas à droite
            marker_centers[CORNER_BOTTOM_LEFT_ID]   # Coin en bas à gauche
        ])
    except KeyError:
        return None  # Si un coin est manquant

def apply_perspective_transform(frame, user_corners, perspective_matrix=None, pixels_per_cm=None):
    """
    Applique une transformation de perspective à l'image pour obtenir une vue corrigée.
    Si la matrice n'est pas fournie, elle est calculée à partir des coins détectés.
    Retourne l'image transformée, la matrice de perspective et le ratio pixels/cm.
    """
    # Calcul du ratio pixels/cm si besoin
    if pixels_per_cm is None:
        pixels_per_cm = calculate_pixels_per_cm(user_corners)

    # Calcul des dimensions en pixels pour la transformation
    scaled_horizontal_distance = HORIZONTAL_DISTANCE_CM * pixels_per_cm * SCALE_FACTOR
    scaled_vertical_distance = VERTICAL_DISTANCE_CM * pixels_per_cm * SCALE_FACTOR
    scaled_horizontal_margin = HORIZONTAL_MARGIN_CM * pixels_per_cm * SCALE_FACTOR
    scaled_vertical_margin = VERTICAL_MARGIN_CM * pixels_per_cm * SCALE_FACTOR

    # Calcul de la matrice de perspective si besoin
    if perspective_matrix is None:
        final_corners = np.float32([
            [scaled_horizontal_margin, scaled_vertical_margin],                                                         # Top-Left
            [scaled_horizontal_margin + scaled_horizontal_distance, scaled_vertical_margin],                            # Top-Right
            [scaled_horizontal_margin + scaled_horizontal_distance, scaled_vertical_margin + scaled_vertical_distance], # Bottom-Right
            [scaled_horizontal_margin, scaled_vertical_margin + scaled_vertical_distance]                               # Bottom-Left
        ])
        perspective_matrix = cv2.getPerspectiveTransform(user_corners, final_corners)

    # Application de la transformation de perspective
    warped_image = cv2.warpPerspective(
        frame, perspective_matrix,
        (int(scaled_horizontal_distance + 2 * scaled_horizontal_margin),
        int(scaled_vertical_distance + 2 * scaled_vertical_margin))
    )

    return warped_image, perspective_matrix, pixels_per_cm

def detect_aruco_markers(frame, aruco_detector):
    """
    Détecte les marqueurs ArUco dans une image.
    Retourne les coins détectés et les IDs.
    """
    # Conversion en niveaux de gris pour la détection
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, _ = aruco_detector.detectMarkers(gray_frame)
    return marker_corners, marker_IDs

def process_markers(frame, marker_IDs, marker_corners):
    """
    Traite les marqueurs détectés : calcule les centres et (optionnellement) dessine les contours.
    Retourne un dictionnaire {id: centre}.
    """
    marker_centers = {}
    if marker_IDs is not None:
        for ids, corners in zip(marker_IDs, marker_corners):
            corners = corners.reshape(4, 2).astype(int)
            center = np.mean(corners, axis=0).astype(int)
            marker_centers[ids[0]] = center
            # Optionnel : dessiner le contour ou le centre
            # cv2.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA)
            # cv2.circle(frame, tuple(center), 5, (0, 255, 0), -1)
    return marker_centers

# ---------------------- MAIN PROGRAM ---------------------- #

def calibrationAndTransform(frame, camNumber, enableManualCalibration=False):
    """
    Fonction principale pour gérer la calibration (auto ou manuelle) et la transformation de perspective.
    Charge la matrice de transformation depuis un fichier si possible, sinon la calcule.
    Retourne l'image transformée.
    """
    # Initialisation des variables
    perspective_matrix = None
    pixels_per_cm = None
    write_file = False  # Indique si la matrice doit être sauvegardée

    # ---------------------- LOAD PERSPECTIVE MATRIX FROM FILE ---------------------- #

    try:
        with open("src/perspective_matrix_" + str(camNumber) + ".txt", "r") as f:
            lines = f.readlines()
            # Lecture du ratio pixels/cm
            pixels_per_cm = float(lines[0].strip())
            # Lecture et conversion de la matrice de perspective
            perspective_matrix_str = "".join(lines[1:]).strip()
            perspective_matrix_str = perspective_matrix_str.replace("[", "").replace("]", "").strip()
            matrix_as_list = [float(x) for x in perspective_matrix_str.split()]
            perspective_matrix = np.float32(matrix_as_list).reshape(3, 3)
    except FileNotFoundError:
        # Fichier non trouvé : calibration nécessaire
        write_file = True

    # ---------------------- COMPUTE PRESPECTIVE MATRIX IF NOT LOADED ---------------------- #

    if perspective_matrix is None and not enableManualCalibration:
        marker_corners, marker_IDs = detect_aruco_markers(frame, aruco_detector)

        # Affiche la frame même si aucun marqueur n'est détecté
        cv2.namedWindow("Detected ArUco Markers", cv2.WINDOW_NORMAL)
        cv2.imshow("Detected ArUco Markers", frame)
        cv2.waitKey(1)  # Petite pause pour l'affichage

        if marker_IDs is None or len(marker_IDs) == 0:
            # Aucun marqueur trouvé, on saute cette itération
            return None

        marker_centers = process_markers(frame, marker_IDs, marker_corners)

        # Vérifie si tous les coins sont détectés
        required_ids = [CORNER_TOP_LEFT_ID, CORNER_TOP_RIGHT_ID, CORNER_BOTTOM_LEFT_ID, CORNER_BOTTOM_RIGHT_ID]
        centers = {id: marker_centers.get(id) for id in required_ids}

        if all(centers[id] is not None for id in required_ids):
            # Tous les coins sont présents, on calcule la matrice de perspective
            marker_centers = sort_by_target_order(marker_centers)
            transformed_frame, perspective_matrix, pixels_per_cm = apply_perspective_transform(frame, marker_centers)
            cv2.destroyWindow("Detected ArUco Markers")
        else:
            # Affiche les marqueurs détectés pour aider à la calibration
            highlighted_frame, _ = highlightDetected(frame, {CORNER_TOP_LEFT_ID, CORNER_TOP_RIGHT_ID, CORNER_BOTTOM_RIGHT_ID, CORNER_BOTTOM_LEFT_ID})

            cv2.imshow("Detected ArUco Markers", highlighted_frame)
            cv2.waitKey(1)
            return frame

    if perspective_matrix is None and enableManualCalibration:
        # Calibration manuelle par clic utilisateur
        centers = getCornersFromUserClick(frame)
        transformed_frame, perspective_matrix, pixels_per_cm = apply_perspective_transform(frame, centers)

    if perspective_matrix is not None:
        # Utilise la matrice de transformation préchargée
        centers = None
        transformed_frame, _, _ = apply_perspective_transform(frame, centers, perspective_matrix, pixels_per_cm)

    # ---------------------- SAVE PERSPECTIVE MATRIX TO FILE ---------------------- #

    if write_file and perspective_matrix is not None:
        with open("src/perspective_matrix_" + str(camNumber) + ".txt", "w") as f:
            f.write(str(pixels_per_cm) + "\n" + str(perspective_matrix))
            # print("Saved new perspective matrix into perpective_matrix_" + str(camNumber) + ".txt")

    return transformed_frame