"""
Module utilitaire pour la gestion d'une caméra virtuelle dans PyBullet.
Permet d'initialiser une caméra virtuelle, de capturer des images simulées, et d'ajuster dynamiquement les paramètres de champ de vision (FoV).
Utilisé pour simuler la perception caméra dans l'environnement PyBullet.
"""

import pybullet as p
import numpy as np
import cv2
import math

def init_camera(cam_position, cam_orientation_deg):
    """
    Initialise une caméra virtuelle dans PyBullet à une position et orientation donnée.

    Args:
        cam_position (list): [x, y, z] position de la caméra.
        cam_orientation_deg (list): [roll, pitch, yaw] en degrés.

    Returns:
        tuple: Matrice de vue et matrice de projection pour le rendu.
    """
    # Conversion de l'orientation en radians puis en quaternion
    cam_orientation = [math.radians(angle) for angle in cam_orientation_deg]
    cam_orientation_quat = p.getQuaternionFromEuler(cam_orientation)

    # Calcul des vecteurs forward (avant) et up (haut) à partir du quaternion
    forward_vector = p.multiplyTransforms([0, 0, 0], cam_orientation_quat, [1, 0, 0], [0, 0, 0, 1])[0]
    up_vector = p.multiplyTransforms([0, 0, 0], cam_orientation_quat, [0, 0, 1], [0, 0, 0, 1])[0]

    # Calcul de la position cible de la caméra (où elle regarde)
    cam_target_pos = [cam_position[i] + forward_vector[i] for i in range(3)]

    # ----------------- PARAMÈTRES DE LA CAMÉRA -----------------
    global h_fov, v_fov
    h_fov = 102  # Champ de vision horizontal (degrés)
    v_fov = 67   # Champ de vision vertical (degrés)
    aspect_ratio = math.tan(math.radians(h_fov) / 2) / math.tan(math.radians(v_fov) / 2)

    near_plane = 0.1
    far_plane = 100

    # Calcul du FoV diagonal à partir des FoV horizontaux et verticaux
    diagonal_fov = 2 * math.atan(math.sqrt(math.tan(math.radians(h_fov) / 2) ** 2 +
                                           math.tan(math.radians(v_fov) / 2) ** 2))
    diagonal_fov_degrees = math.degrees(diagonal_fov)

    # Calcul des matrices de vue et de projection
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=cam_position,
        cameraTargetPosition=cam_target_pos,
        cameraUpVector=up_vector
    )
    projection_matrix = p.computeProjectionMatrixFOV(diagonal_fov_degrees, aspect_ratio, near_plane, far_plane)
    
    # ------------------------------------------------------

    # Crée une représentation visuelle de la caméra (cube rouge)
    box_size = 0.01
    camera_visual_shape_id = p.createVisualShape(
        shapeType=p.GEOM_BOX, halfExtents=[1.5 * box_size, 2 * box_size, box_size], rgbaColor=[1, 0, 0, 1]
    )
    p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=camera_visual_shape_id,
        basePosition=cam_position,
        baseOrientation=cam_orientation_quat
    )

    return view_matrix, projection_matrix


def read_camera(camera_infos):
    """
    Capture une image à partir de la caméra virtuelle PyBullet.

    Args:
        camera_infos (tuple): Matrice de vue et de projection.

    Returns:
        numpy.ndarray: Image capturée au format OpenCV (BGR).
    """
    view_matrix, projection_matrix = camera_infos

    # Capture l'image simulée depuis PyBullet
    width, height, rgb_img, _, _ = p.getCameraImage(
        width=1920, height=1080, 
        viewMatrix=view_matrix, projectionMatrix=projection_matrix,
        shadow=0, flags=p.ER_NO_SEGMENTATION_MASK, renderer=p.ER_TINY_RENDERER
    )

    # Conversion en format compatible OpenCV (BGR)
    rgb_img = np.reshape(rgb_img, (height, width, 4))[:, :, :3]
    rgb_img = rgb_img.astype(np.uint8)
    rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR)

    return rgb_img


def adjust_fov_parameters():
    """
    Crée une fenêtre avec des sliders pour ajuster dynamiquement les paramètres de champ de vision (FoV).
    """

    global h_fov, v_fov

    cv2.namedWindow("Adjust FoV Parameters", cv2.WINDOW_NORMAL)

    # Création des sliders pour le FoV horizontal et vertical
    cv2.createTrackbar("Horizontal FOV (deg)", "Adjust FoV Parameters", int(h_fov), 180, lambda x: None)
    cv2.createTrackbar("Vertical FOV (deg)", "Adjust FoV Parameters", int(v_fov), 180, lambda x: None)


def update_fov_parameters():
    """
    Met à jour les paramètres de FoV en fonction des sliders.
    """

    global h_fov, v_fov

    # Lecture des valeurs depuis les sliders
    h_fov = cv2.getTrackbarPos("Horizontal FOV (deg)", "Adjust FoV Parameters")
    v_fov = cv2.getTrackbarPos("Vertical FOV (deg)", "Adjust FoV Parameters")