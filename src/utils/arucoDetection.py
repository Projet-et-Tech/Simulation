import cv2
import numpy as np

def highlightDetected(frame, allowed_ids):
    """
    Highlights only the specified ArUco markers in yellow in the given frame.

    Args:
    - frame (ndarray): The input frame (image) from the camera.
    - allowed_ids (list or set): A list/set of marker IDs to be highlighted.

    Returns:
    - frame (ndarray): The frame with only allowed ArUco markers highlighted in yellow.
    """
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Load the predefined ArUco dictionary
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)  # Adjust dictionary if needed
    parameters = cv2.aruco.DetectorParameters_create()

    # Detect ArUco markers in the frame
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # If markers are detected, filter them based on allowed IDs
    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id in allowed_ids:
                corner_pts = corners[i][0]  # Extract the 4 corner points

                # Convert float points to integers for OpenCV drawing functions
                corner_pts = corner_pts.astype(int)

                # Draw yellow bounding box around the marker
                cv2.polylines(frame, [corner_pts], isClosed=True, color=(0, 255, 255), thickness=3)

                # Put the marker ID as text near the top-left corner
                text_position = tuple(corner_pts[0])  # Top-left corner
                cv2.putText(frame, str(marker_id), text_position, cv2.FONT_HERSHEY_SIMPLEX, 
                            fontScale=1, color=(0, 255, 255), thickness=2, lineType=cv2.LINE_AA)

    return frame  # Return the frame with yellow-highlighted markers

def detectMarkerByID(frame, target_id):
    """
    Detects a specific ArUco marker in the frame and returns its coordinates.

    Args:
    - frame (ndarray): The input image frame.
    - target_id (int): The ID of the ArUco marker to detect.

    Returns:
    - tuple: (marker_corners, center) where:
        - marker_corners (ndarray): 4 corner points of the detected marker (if found).
        - center (tuple): (x, y) center of the detected marker.
    - If the marker is not found, returns (None, None).
    """
    # Convert frame to grayscale for better ArUco detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Load the predefined ArUco dictionary (Adjust this if needed)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters_create()

    # Detect ArUco markers in the frame
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # If markers are detected
    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id == target_id:
                marker_corners = corners[i][0]  # Get 4 corner points

                # Compute the center of the marker
                center_x = int(marker_corners[:, 0].mean())
                center_y = int(marker_corners[:, 1].mean())
                center = (center_x, center_y)

                return center  # Return marker corners and center point

    return None, None  # Return None if the target marker is not found
