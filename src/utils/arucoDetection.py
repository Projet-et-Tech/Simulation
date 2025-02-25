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
    parameters = cv2.aruco.DetectorParameters()
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    # Detect ArUco markers in the frame
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

    return frame, centers  # Return both the modified frame and the marker centers
