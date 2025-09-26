"""
Module utilitaire pour la correction de perspective et la conversion des coordonnées 2D (image) en coordonnées 3D (réelles).
Permet de transformer un clic utilisateur ou une détection sur l'image en position réelle sur la table, en tenant compte de la position de la caméra et de la hauteur de l'objet.
Inclut aussi une fonction pour sélectionner manuellement les coins de la table sur une image.
"""

import math
import cv2
import numpy as np

from config import TABLE_LENGTH, TABLE_WIDTH

def convert_2D_to_3D(x, y, transformed_frame, CAM_POS, CUBE_HEIGHT):
    """
    Convertit des coordonnées 2D (image) en coordonnées 3D réelles par correction de perspective.
    Utilisé pour obtenir la position réelle sur la table à partir d'un clic ou d'une détection sur l'image.

    Args:
        x (float): Coordonnée X du point cliqué dans l'image.
        y (float): Coordonnée Y du point cliqué dans l'image.
        transformed_frame (numpy.ndarray): Image transformée (vue corrigée).
        CAM_POS (tuple): Position de la caméra (x, y, z) dans le monde réel.
        CUBE_HEIGHT (float): Hauteur de l'objet détecté.

    Returns:
        tuple: (x_true, y_true) - Coordonnées corrigées en 3D.
    """

    # ------------------ IMAGE DIMENSIONS ------------------

    windowWidth = transformed_frame.shape[0]  
    windowHeight = transformed_frame.shape[1]

    # ------------------ CONVERT IMAGE COORDINATES TO FAKE 3D ------------------

    # Convert 2D click coordinates to the estimated table coordinate system
    x_fake = ((x - windowWidth / 2) / windowWidth) * TABLE_LENGTH
    y_fake = ((y - windowHeight / 2) / windowHeight) * TABLE_WIDTH

    # ------------------ COMPUTE PERSPECTIVE CORRECTION ANGLES ------------------

    theta = math.atan2((CAM_POS[1] + y_fake), (CAM_POS[0] - x_fake))  # XY plane angle
    phi = math.atan2((CAM_POS[0] - x_fake), (CAM_POS[2]))  # XZ plane angle

    # ------------------ APPLY PERSPECTIVE CORRECTION ------------------

    x_correction = phi * (CUBE_HEIGHT / CAM_POS[2]) * CAM_POS[0]
    y_correction = theta * (CUBE_HEIGHT / CAM_POS[2]) * CAM_POS[1]

    # Calcul des coordonnées corrigées (ajustements empiriques)
    x_true = x_fake + x_correction + 0.37
    y_true = y_fake + y_correction - 0.37

    # ------------------ FINAL ADJUSTMENTS ------------------

    #if x_true <= 0.5:
     #   x_true -= (x_true - 0.5) * (0.11 / (-1 - 0.5))  # Linear adjustment for x ≤ 0.5
    #else:
     #   x_true -= 0.01  # Slight correction for x > 0.5

    y_true += y_true * 0.11  # Apply correction on y

    # ------------------ DEBUG OUTPUTS ------------------

    print(f"Angles (radians): theta = {theta:.2f}, phi = {phi:.2f}")
    print(f"Fake Detected Position in 3D: (x, y, z) = ({x_fake:.2f}, {y_fake:.2f})")
    print(f"True Detected Position in 3D: (x, y, z) = ({x_true:.2f}, {y_true:.2f})")
    print("")
  
    return x_true, y_true

def getCornersFromUserClick(frame):
    """
    Permet à l'utilisateur de sélectionner manuellement quatre coins sur une image par clic souris.
    Utilisé pour la calibration manuelle de la perspective.

    Args:
        frame (numpy.ndarray): Image sur laquelle l'utilisateur va cliquer.

    Returns:
        numpy.ndarray: Tableau de quatre coordonnées (x, y) sélectionnées.
    """
    # Copie de l'image pour affichage interactif
    frame_copy = frame.copy()
    coords = []  # Liste des coordonnées sélectionnées

    def click_event(event, x, y, flags, param):
        """
        Callback souris pour capturer les clics de l'utilisateur.
        Ajoute un point vert sur l'image à chaque clic et stocke la coordonnée.
        Ferme la fenêtre après 4 points.
        """
        if event == cv2.EVENT_LBUTTONDOWN:
            coords.append((x, y))
            cv2.circle(frame_copy, (x, y), 5, (0, 255, 0), -1)
            cv2.imshow('Manual Calibration', frame_copy)
            if len(coords) == 4:
                cv2.destroyAllWindows()

    # Affichage de l'image et activation du callback souris
    cv2.imshow('Manual Calibration', frame_copy)
    cv2.setMouseCallback('Manual Calibration', click_event)
    cv2.waitKey(0)  # Attend que l'utilisateur ait cliqué 4 fois

    # Retourne les coordonnées sélectionnées sous forme de tableau float32
    return np.float32(coords)
