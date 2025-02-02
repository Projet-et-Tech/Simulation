import math
import cv2
import numpy as np

def convert_2D_to_3D(x, y, transformed_frame, TABLE_LENGTH, TABLE_WIDTH, CAM_POS, CUBE_HEIGHT, robot_start_position):
    """
    Converts 2D image coordinates to 3D real-world coordinates using perspective correction.

    Parameters:
        x (float): X-coordinate of the clicked point in the image.
        y (float): Y-coordinate of the clicked point in the image.
        transformed_frame (numpy.ndarray): The transformed image/frame.
        TABLE_LENGTH (float): Length of the table in real-world units.
        TABLE_WIDTH (float): Width of the table in real-world units.
        CAM_POS (tuple): Camera position (x, y, z) in real-world coordinates.
        CUBE_HEIGHT (float): Assumed height of the detected object.
        robot_start_position (tuple): Initial robot position (x, y, z).

    Returns:
        tuple: (x_true, y_true, z_true) - The corrected 3D coordinates.
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

    x_true = x_fake + x_correction
    y_true = y_fake + y_correction

    # ------------------ FINAL ADJUSTMENTS ------------------

    if x_true <= 0.5:
        x_true += (x_true - 0.5) * (0.11 / (-1 - 0.5))  # Linear adjustment for x ≤ 0.5
    else:
        x_true -= 0.01  # Slight correction for x > 0.5

    y_true -= y_true * 0.11  # Apply correction on y

    # ------------------ DEBUG OUTPUTS ------------------

    print(f"Angles (radians): theta = {theta:.2f}, phi = {phi:.2f}")
    print(f"Fake Detected Position in 3D: (x, y, z) = ({x_fake:.2f}, {y_fake:.2f}, {CUBE_HEIGHT:.2f})")
    print(f"True Detected Position in 3D: (x, y, z) = ({x_true:.2f}, {y_true:.2f}, {CUBE_HEIGHT:.2f})")
    print(f"True Robot Position in 3D: (x, y, z) = ({robot_start_position[0]:.2f}, {robot_start_position[1]:.2f}, {robot_start_position[2]:.2f})")

    return x_true, y_true


def getCornersFromUserClick(frame):
    """
    Allows the user to manually select four corner points by clicking on an image.
    
    Parameters:
        frame (numpy.ndarray): The image or frame where the user will click points.

    Returns:
        numpy.ndarray: A list of four (x, y) coordinate pairs representing the selected points.
    """

    # Create a copy of the frame to keep the original unmodified
    frame_copy = frame.copy()

    # List to store selected coordinates
    coords = []

    def click_event(event, x, y, flags, param):
        """
        Mouse click event handler. Captures coordinates when the user clicks on the image.
        
        Parameters:
            event: The OpenCV mouse event type.
            x, y: Coordinates of the mouse click.
            flags: Any special flags (not used here).
            param: Additional parameters (not used here).
        """
        if event == cv2.EVENT_LBUTTONDOWN:  # Left mouse button click
            coords.append((x, y))  # Store clicked coordinates
            cv2.circle(frame_copy, (x, y), 5, (0, 255, 0), -1)  # Draw a green dot at the clicked location
            cv2.imshow('Manual Calibration', frame_copy)  # Update the display

            # If 4 points are selected, print them and close the window
            if len(coords) == 4:
                cv2.destroyAllWindows()

    # Display the copied image for user interaction
    cv2.imshow('Manual Calibration', frame_copy)
    
    # Set the mouse callback function to capture clicks
    cv2.setMouseCallback('Manual Calibration', click_event)

    # Wait indefinitely until a key is pressed (ensures user has time to click)
    cv2.waitKey(0)

    # Convert coordinates to a NumPy array of type float32 for further processing
    return np.float32(coords)