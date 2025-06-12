"""
Script principal pour la simulation de la détection caméra dans PyBullet.
Permet de simuler la capture d'images depuis des caméras virtuelles, la détection d'obstacles par couleur,
la détection de marqueurs ArUco, et la projection des obstacles en 3D.
Inclut la possibilité de cliquer sur l'image pour obtenir des coordonnées, et des outils de calibration et de visualisation.
"""

import pybullet as p
import cv2
import time
import numpy as np

import config as config
from simulation.pybullet_manager import PyBulletManager
from config import ROBOT_START_POS, DEBUG, CAM1_POS, CAM1_ORIENTATION_DEG, CAM2_POS, CAM2_ORIENTATION_DEG, CAN_ID, PLANK_COLOR_BOUNDS
from simulation.setup import load_objects, create_environment

from utils.virtualCamera import init_camera, read_camera
from utils.imageTransform import calibrationAndTransform
from utils.segmentation import colorSegmentation, drawObstacles3D
from utils.arucoDetection import highlightDetected


# ------------------ GLOBAL VARIABLES ------------------

clicked_point= None

# ------------------ MOUSE CLICK CALLBACK ------------------ (temp, will be auto detection by aruco on PAMIs)

def mouse_callback(event, x, y, flags, param):
    """
    Callback souris temporaire pour capturer la position d'un clic sur l'image.
    Ferme la fenêtre après le clic.
    """
    global clicked_point
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_point = (x, y)
        print(f"Clicked at: {clicked_point}")
        cv2.destroyAllWindows()  # Ferme la fenêtre après le clic

# ------------------ PYBULLET SIMULATION SETUP ------------------

debug = DEBUG

# Initialisation de PyBullet et de la caméra de debug
pybullet_manager = PyBulletManager(debug=debug)
pybullet_manager.reset_camera(distance=2.0, yaw=0, pitch=-45, target=[0, 0, 0])
pybullet_manager.set_real_time_simulation(True)

# Création de l'environnement (sol et table)
plane_id, table_id = create_environment(pybullet_manager)

# ------------------ LOAD OBSTACLES ------------------
# Chargement des conserves et planches dans la simulation
can_ids = load_objects(pybullet_manager, "src/urdf_models/conserve.urdf", [0.7071, 0, 0, 0.7071], config.CAN_POSITIONS)
plank_horizontal_ids = load_objects(pybullet_manager, "src/urdf_models/planche.urdf", [0.7071, 0, 0, 0.7071], config.PLANK_POSITIONS_HORIZONTAL)
plank_vertical_ids = load_objects(pybullet_manager, "src/urdf_models/planche.urdf", [0.5, 0.5, 0.5, 0.5], config.PLANK_POSITIONS_VERTICAL)

# ------------------ LOAD ROBOT ------------------

robot_id = pybullet_manager.load_urdf("src/urdf_models/robot_pami.urdf", ROBOT_START_POS, [0, 0, 0, 1])

# ------------------ CAMERA CAPTURE & OBJECT DETECTION ------------------
# Initialisation des caméras virtuelles
cam1 = init_camera(CAM1_POS, CAM1_ORIENTATION_DEG)
cam2 = init_camera(CAM2_POS, CAM2_ORIENTATION_DEG)

time.sleep(5)   # Attendre que le robot tombe sur la table avant la capture (temporaire)
rgb_img1 = read_camera(cam1)
rgb_img2 = read_camera(cam2)

# Affichage de la vue caméra 1
cv2.namedWindow("cam view", cv2.WINDOW_NORMAL)
cv2.imshow("cam view", rgb_img1)

# Calibration et correction de perspective sur les images capturées
transformed_frame1 = calibrationAndTransform(rgb_img1, 1, True)
transformed_frame2 = calibrationAndTransform(rgb_img2, 2, True)

# ------------------ SEGMENTATION PAR COULEUR ------------------
# Détection des planches par couleur et projection 3D
object_corners = colorSegmentation(transformed_frame1, PLANK_COLOR_BOUNDS)
drawObstacles3D(object_corners, transformed_frame1, CAM1_POS)

object_corners = colorSegmentation(transformed_frame2, PLANK_COLOR_BOUNDS, (0, 255, 255))
drawObstacles3D(object_corners, transformed_frame2, CAM2_POS, (1, 1, 0))

# ------------------------ CAN DETECTION ------------------------
# Détection et surlignage d'une conserve spécifique par son ID
highlighted_frame, _ = highlightDetected(rgb_img1, {CAN_ID})
cv2.namedWindow("Highlighted Frame", cv2.WINDOW_NORMAL)
cv2.imshow("Highlighted Frame", highlighted_frame)

# essayer color detection blanc voisin de vert jaune rouge (couleurs étiquettes)

# ---------------------------- BAZAR ----------------------------
# essayer refaire perpective matrix

# Print converted 3D corners
# for obj, corners in object_corners_3D.items():
#     print(f"{obj} (3D): {corners}")

# Visualization in PyBullet
# while True:
#     time.sleep(0.1)  # Keep visualization running


# ------------------ CONVERT 2D IMAGE COORDINATES TO 3D ------------------

# Capture the user's click position
# y, x = capture_click(transformed_frame)

# x_true, y_true = convert_2D_to_3D(x, y, transformed_frame, TABLE_LENGTH, TABLE_WIDTH, CAM1_POS, PAMI_HEIGHT)

# ------------------ VISUALIZATION IN PYBULLET ------------------

# while True:
#     # Corrected detection (Green)
#     p.addUserDebugLine([x_true, y_true, PAMI_HEIGHT], [x_true, y_true, 1], [0, 1, 0], 2, 0)
#     p.addUserDebugText("Il est la", [x_true, y_true, 1], textColorRGB=[0, 1, 0], textSize=1.5)

# ------------------ CLEANUP ------------------

cv2.waitKey(0) 
cv2.destroyAllWindows()
p.disconnect()





























# import cv2
# import numpy as np

# def detect_and_align(image1, image2):
#     detector = cv2.ORB_create()
#     keypoints1, descriptors1 = detector.detectAndCompute(image1, None)
#     keypoints2, descriptors2 = detector.detectAndCompute(image2, None)
    
#     matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
#     matches = matcher.match(descriptors1, descriptors2)
#     matches = sorted(matches, key=lambda x: x.distance)
    
#     src_pts = np.float32([keypoints2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
#     dst_pts = np.float32([keypoints1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
    
#     matrix, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
#     aligned_image2 = cv2.warpPerspective(image2, matrix, (image1.shape[1], image1.shape[0]))
    
#     return aligned_image2

# def highlight_differences(image1, image2):
#     diff = cv2.absdiff(image1, image2)
#     gray_diff = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
#     _, thresh = cv2.threshold(gray_diff, 30, 255, cv2.THRESH_BINARY)
    
#     contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     result = image1.copy()
#     cv2.drawContours(result, contours, -1, (0, 0, 255), 2)
    
#     return result

# def overlay_images(image1, image2, alpha=0.5):
#     return cv2.addWeighted(image1, alpha, image2, 1 - alpha, 0)

# def detect_differences_on_overlay(overlay_image):
#     gray_overlay = cv2.cvtColor(overlay_image, cv2.COLOR_BGR2GRAY)
#     _, thresh = cv2.threshold(gray_overlay, 30, 255, cv2.THRESH_BINARY)
    
#     contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     result = overlay_image.copy()
#     cv2.drawContours(result, contours, -1, (0, 255, 0), 2)
    
#     return result

# # Chargement des images
# image1 = transformed_frame1
# image2 = transformed_frame2

# aligned_image2 = detect_and_align(image1, image2)
# diff_image = highlight_differences(image1, aligned_image2)
# overlayed_image = overlay_images(image1, aligned_image2)
# diff_on_overlay = detect_differences_on_overlay(overlayed_image)

# cv2.namedWindow("Overlayed Image", cv2.WINDOW_NORMAL)
# cv2.imshow('Overlayed Image', overlayed_image)

# cv2.namedWindow("Differences on Overlay", cv2.WINDOW_NORMAL)
# cv2.imshow('Differences on Overlay', diff_on_overlay)

# cv2.namedWindow("Differences", cv2.WINDOW_NORMAL)
# cv2.imshow('Differences', diff_image)

# cv2.waitKey(0)
# cv2.destroyAllWindows()
