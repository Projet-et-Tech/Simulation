import cv2
import numpy as np
from cv2 import aruco

from config import HORIZONTAL_DISTANCE_CM, VERTICAL_DISTANCE_CM, SCALE_FACTOR, CORNER_TOP_LEFT_ID, CORNER_TOP_RIGHT_ID, CORNER_BOTTOM_LEFT_ID, CORNER_BOTTOM_RIGHT_ID

from utils.perspectiveCorrection import getCornersFromUserClick

# ---------------------- ARUCO MARKER SETUP ---------------------- #

# Define the dictionary of ArUco markers and set up the detector
marker_dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters()
aruco_detector = aruco.ArucoDetector(marker_dictionary, param_markers)

# ---------------------- FUNCTIONS ---------------------- #

def calculate_pixels_per_cm(corners_in_order):
    """
    Calculate the pixel-to-centimeter ratio based on known table distances.

    Parameters:
        corners_in_order: List of detected marker corners in order (Top-Left, Top-Right, Bottom-Right, Bottom-Left)

    Returns:
        Average pixels per centimeter value
    """
    
    # Compute pixel distances between known markers
    horizontal_dist_pixels = np.linalg.norm(corners_in_order[0] - corners_in_order[1])  # Top-Left -> Top-Right
    vertical_dist_pixels = np.linalg.norm(corners_in_order[1] - corners_in_order[2])    # Top-Right -> Bottom-Right
    
    # Convert to pixels per centimeter
    pixels_per_cm_hor = horizontal_dist_pixels / HORIZONTAL_DISTANCE_CM
    pixels_per_cm_ver = vertical_dist_pixels / VERTICAL_DISTANCE_CM

    # Return the average scaling factor
    return (pixels_per_cm_hor + pixels_per_cm_ver) / 2

def apply_perspective_transform(frame, user_corners, perspective_matrix=None, pixels_per_cm=None):
    """
    Apply a perspective transformation to correct the image view.

    Parameters:
        frame: Input camera frame
        user_corners: Corners of the detected table
        perspective_matrix: Precomputed transformation matrix (optional)
        pixels_per_cm: Precomputed pixels/cm ratio (optional)
    
    Returns:
        Transformed frame, perspective matrix, pixels per cm
    """
    
    # Compute pixels/cm if not provided
    if pixels_per_cm is None:
        pixels_per_cm = calculate_pixels_per_cm(user_corners)

    # Compute scaled distances in pixels
    scaled_horizontal_distance = HORIZONTAL_DISTANCE_CM * pixels_per_cm * SCALE_FACTOR
    scaled_vertical_distance = VERTICAL_DISTANCE_CM * pixels_per_cm * SCALE_FACTOR

    # Compute the perspective transformation matrix if not provided
    if perspective_matrix is None:
        final_corners = np.float32([
            [0, 0],                                                     # Top-Left
            [scaled_horizontal_distance, 0],                            # Top-Right
            [scaled_horizontal_distance, scaled_vertical_distance],     # Bottom-Right
            [0, scaled_vertical_distance]                               # Bottom-Left
        ])
        perspective_matrix = cv2.getPerspectiveTransform(user_corners, final_corners)

    # Apply the transformation
    warped_image = cv2.warpPerspective(
        frame, perspective_matrix,
        (int(scaled_horizontal_distance), int(scaled_vertical_distance))
    )
    
    return warped_image, perspective_matrix, pixels_per_cm

def detect_aruco_markers(frame, aruco_detector):
    """ 
    Detect ArUco markers in a given image.
    
    Parameters:
        frame (numpy.ndarray): The input image where markers will be detected.
        aruco_detector: The ArUco detector object from OpenCV.
    
    Returns:
        tuple: (marker_corners, marker_IDs)
            - marker_corners (list): List of detected marker corner coordinates.
            - marker_IDs (numpy.ndarray): IDs of detected markers.
    """
    # Convert the image to grayscale (ArUco detection works better in grayscale)
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect ArUco markers in the image
    marker_corners, marker_IDs, _ = aruco_detector.detectMarkers(gray_frame)

    return marker_corners, marker_IDs

def process_markers(frame, marker_IDs, marker_corners):
    """ 
    Process detected ArUco markers and extract their centers.
    
    Parameters:
        frame (numpy.ndarray): The input image where markers are detected.
        marker_IDs (numpy.ndarray): Array of detected marker IDs.
        marker_corners (list): List of detected marker corner coordinates.
    
    Returns:
        dict: A dictionary mapping marker IDs to their respective center coordinates.
    """
    marker_centers = {}  # Dictionary to store marker IDs and their centers

    if marker_IDs is not None:
        for ids, corners in zip(marker_IDs, marker_corners):
            # Reshape corners to a 4x2 integer array (four corner points per marker)
            corners = corners.reshape(4, 2).astype(int)

            # Compute the center of the marker by averaging its corner points
            center = np.mean(corners, axis=0).astype(int)
            marker_centers[ids[0]] = center  # Store the center in the dictionary

            # Optional: Draw marker bounding box
            # cv2.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA)

            # Optional: Draw the center point of the marker
            # cv2.circle(frame, tuple(center), 5, (0, 255, 0), -1)

    return marker_centers

# ---------------------- MAIN PROGRAM ---------------------- #

def calibrationAndTransform(frame, name):
    """
    Main function to handle manual calibration and perspective transformation.

    Parameters:
        frame: Input camera frame
        name: Window name for display

    Returns:
        Transformed frame, detected corners
    """
    
    # Initialize variables
    perspective_matrix = None
    pixels_per_cm = None
    write_file = False  # Flag to indicate if matrix should be saved

    # ---------------------- LOAD PERSPECTIVE MATRIX FROM FILE ---------------------- #

    try:
        with open("src/perspective_matrix.txt", "r") as f:
            lines = f.readlines()
            print("Loading camera transformation data from perpective_matrix.txt...")
            
            # Read the first line as pixels per cm
            pixels_per_cm = float(lines[0].strip())

            # Read and convert the perspective matrix
            perspective_matrix_str = "".join(lines[1:]).strip()
            perspective_matrix_str = perspective_matrix_str.replace("[", "").replace("]", "").strip()
            matrix_as_list = [float(x) for x in perspective_matrix_str.split()]
            perspective_matrix = np.float32(matrix_as_list).reshape(3, 3)

            print("perpective_matrix.txt loaded")

    except FileNotFoundError:
        print("Perspective matrix file not found. Calibration required.")
        write_file = True  # File needs to be created

    # ---------------------- COMPUTE PRESPECTIVE MATRIX IF NOT LOADED ---------------------- #

    if perspective_matrix is None:

        marker_corners, marker_IDs = detect_aruco_markers(frame, aruco_detector)
        marker_centers = process_markers(frame, marker_IDs, marker_corners)

        # If all corners are detected
        required_ids = [CORNER_TOP_LEFT_ID, CORNER_TOP_RIGHT_ID, CORNER_BOTTOM_LEFT_ID, CORNER_BOTTOM_RIGHT_ID]
        centers = {id: marker_centers.get(id) for id in required_ids}

        if all(centers[id] is not None for id in required_ids):
            transformed_frame, perspective_matrix, pixels_per_cm, _ = apply_perspective_transform(frame, marker_centers)
        else:
            print("Error auto-calibration: could not find all corners")
            print("Switching to manual calibration")

            # Get user-defined table corners for calibration
            centers = getCornersFromUserClick(frame)
            transformed_frame, perspective_matrix, pixels_per_cm = apply_perspective_transform(frame, centers)

    else:
        # Use the preloaded transformation matrix
        centers = None
        transformed_frame, _, _ = apply_perspective_transform(frame, centers, perspective_matrix, pixels_per_cm)

    # ---------------------- SAVE PERSPECTIVE MATRIX TO FILE ---------------------- #

    if write_file:
        with open("src/perspective_matrix.txt", "w") as f:
            f.write(str(pixels_per_cm) + "\n" + str(perspective_matrix))
            print("Saved new perspective matrix into perpective_matrix.txt")

    return transformed_frame