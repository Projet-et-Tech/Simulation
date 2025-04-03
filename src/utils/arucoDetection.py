import cv2
import numpy as np

def highlightDetected(frame, allowed_ids):
    """
    Highlights only the specified ArUco markers in yellow in the given frame and returns their centers.

    Args:
    - frame (ndarray): The input frame (image) from the camera.
    - allowed_ids (list or set): A list/set of marker IDs to be highlighted.

    Returns:
    - frame (ndarray): The frame with only allowed ArUco markers highlighted in yellow.
    - centers (dict): Dictionary mapping marker IDs to their center coordinates.
    """
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Load the predefined ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)  # Adjust dictionary if needed


    # Paramètres de détection optimisés
    parameters = cv2.aruco.DetectorParameters()


    parameters.minOtsuStdDev = 5.0  # Améliore le seuil Otsu pour des conditions de lumière variables
    parameters.errorCorrectionRate = 0.9  # Augmente la tolérance aux erreurs du marqueur

    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    # Detect ArUco markers in the frame

    # Prétraitement de l'image
    # gray = cv2.GaussianBlur(gray, (5, 5), 0)  # Réduction du bruit
    # gray = cv2.equalizeHist(gray)  # Amélioration du contraste
    corners, ids, _ = aruco_detector.detectMarkers(gray)

    centers = {}  # Dictionary to store marker centers

    # If markers are detected, filter them based on allowed IDs
    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id in allowed_ids:
                corner_pts = corners[i][0]  # Extract the 4 corner points

                # Convert float points to integers for OpenCV drawing functions
                corner_pts = corner_pts.astype(int)

                # Compute the center of the marker
                center = np.mean(corner_pts, axis=0).astype(int)
                centers[marker_id] = tuple(center)  # Store center in dictionary

                # Draw yellow bounding box around the marker
                cv2.polylines(frame, [corner_pts], isClosed=True, color=(0, 255, 255), thickness=3)

                # Put the marker ID as text near the top-left corner
                text_position = tuple(corner_pts[0])  # Top-left corner
                cv2.putText(frame, str(marker_id), text_position, cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=1, color=(0, 255, 255), thickness=2, lineType=cv2.LINE_AA)

                # Draw the center point
                cv2.circle(frame, tuple(center), 5, (0, 255, 0), -1)

    return frame, centers, corners  # Return both the modified frame and the marker centers


def drawCross(frame, center, corners, length=30):
    top_left = corners[0]
    top_right = corners[1]

    # Calculer l'orientation du marqueur
    delta_y = top_right[1] - top_left[1]
    delta_x = top_right[0] - top_left[0]
    angle = np.arctan2(delta_y, delta_x)

    sin_angle = np.sin(angle)
    cos_angle = np.cos(angle)

    # Coordonnees pour les quatre directions de la croix
    directions = [
        (-cos_angle * length, -sin_angle * length),  # Haut
        (sin_angle * length, -cos_angle * length),   # Droite
        (cos_angle * length, sin_angle * length),    # Bas
        (-sin_angle * length, cos_angle * length)    # Gauche
    ]

    # Dessiner la branche vers le haut
    branche1 = (int(center[0] + directions[0][0]), int(center[1] + directions[0][1]))
    cv2.line(frame, center, branche1, (0, 0, 255), 2)  # Rouge

    # Dessiner les autres branches
    for i in range(1, 4):
        branche = (int(center[0] + directions[i][0]), int(center[1] + directions[i][1]))
        cv2.line(frame, center, branche, (0, 200, 200), 2)  # Jaune


def getDirectionAndDistance(centerOrigin, cornersOrigin, centerTarget, frame_width, frame_height, meters_per_pixel):
    """
    Calculate the angle, distance, and direction between two points in meters.

    Args:
    - centerOrigin (tuple): The center of the origin marker (x, y).
    - cornersOrigin (ndarray): The corners of the origin marker.
    - centerTarget (tuple): The center of the target marker (x, y).
    - frame_width (int): The width of the frame in pixels.
    - frame_height (int): The height of the frame in pixels.
    - meters_per_pixel (float): The conversion factor from pixels to meters.

    Returns:
    - angle (float): The angle in degrees between the origin and target.
    - distance_meters (float): The distance in meters between the origin and target.
    - direction (str): The direction from the origin to the target ("Left" or "Right").
    """
    if not isinstance(centerOrigin, (tuple, list, np.ndarray)) or not isinstance(centerTarget, (tuple, list, np.ndarray)):
        raise ValueError("centerOrigin and centerTarget must be tuples, lists, or arrays.")

    # Calculate distance in pixels
    distance_pixels = np.linalg.norm(np.array(centerTarget) - np.array(centerOrigin))

    # Convert distance to meters
    distance_meters = distance_pixels * meters_per_pixel

    # Calculate angle relative to the red crossbar (upward direction)
    red_crossbar_vector = np.array([0, -1])  # Upward direction in image coordinates
    vectorTarget = np.array(centerTarget) - np.array(centerOrigin)
    angle = np.arctan2(vectorTarget[1], vectorTarget[0]) - np.arctan2(red_crossbar_vector[1], red_crossbar_vector[0])
    angle = np.degrees(angle)  # Convert angle to degrees

    # Normalize the angle to the range [-180, 180]
    angle = (angle + 180) % 360 - 180

    # Determine direction based on the angle
    if angle > 0:
        direction = "Right"
    else:
        direction = "Left"

    return angle, distance_meters, direction