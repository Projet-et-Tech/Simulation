import pybullet as p
import numpy as np
import cv2
import math

def init_camera(cam_position, cam_orientation_deg):
    """
    Initializes a virtual camera in PyBullet with a given position and orientation.

    Parameters:
        cam_position (list): [x, y, z] coordinates of the camera.
        cam_orientation_deg (list): [roll, pitch, yaw] orientation in degrees.

    Returns:
        tuple: View matrix and projection matrix for rendering.
    """

    # Convert orientation from degrees to radians
    cam_orientation = [math.radians(angle) for angle in cam_orientation_deg]

    # Convert orientation to a quaternion (used in PyBullet)
    cam_orientation_quat = p.getQuaternionFromEuler(cam_orientation)

    # Extract forward and up vectors from quaternion
    forward_vector = p.multiplyTransforms([0, 0, 0], cam_orientation_quat, [1, 0, 0], [0, 0, 0, 1])[0]  # X-axis (forward)
    up_vector = p.multiplyTransforms([0, 0, 0], cam_orientation_quat, [0, 0, 1], [0, 0, 0, 1])[0]  # Z-axis (up)

    # Compute the target position based on the forward vector
    cam_target_pos = [cam_position[i] + forward_vector[i] for i in range(3)]

    # ----------------- CAMERA PARAMETERS -----------------

    global h_fov, v_fov
    h_fov = 102  # Horizontal Field of View (FoV) in degrees
    v_fov = 67   # Vertical Field of View (FoV) in degrees
    aspect_ratio = math.tan(math.radians(h_fov) / 2) / math.tan(math.radians(v_fov) / 2)  # Derived aspect ratio

    # Define near and far clipping planes
    near_plane = 0.1
    far_plane = 100

    # Convert horizontal & vertical FoV into a single diagonal FoV
    diagonal_fov = 2 * math.atan(math.sqrt(math.tan(math.radians(h_fov) / 2) ** 2 +
                                           math.tan(math.radians(v_fov) / 2) ** 2))
    diagonal_fov_degrees = math.degrees(diagonal_fov)

    # Compute view and projection matrices
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=cam_position,
        cameraTargetPosition=cam_target_pos,
        cameraUpVector=up_vector
    )
    projection_matrix = p.computeProjectionMatrixFOV(diagonal_fov_degrees, aspect_ratio, near_plane, far_plane)
    
    # ------------------------------------------------------

    # Create a visual representation of the camera as a small red box
    box_size = 0.01  # Size of the visual cube
    camera_visual_shape_id = p.createVisualShape(
        shapeType=p.GEOM_BOX, halfExtents=[1.5 * box_size, 2 * box_size, box_size], rgbaColor=[1, 0, 0, 1]
    )

    p.createMultiBody(
        baseMass=0,  # Static object
        baseVisualShapeIndex=camera_visual_shape_id,
        basePosition=cam_position,
        baseOrientation=cam_orientation_quat  # Apply the camera's orientation
    )

    return view_matrix, projection_matrix


def read_camera(camera_infos):
    """
    Captures an image using the virtual camera in PyBullet.

    Parameters:
        camera_infos (tuple): View matrix and projection matrix.

    Returns:
        numpy.ndarray: Captured image in OpenCV format.
    """

    view_matrix, projection_matrix = camera_infos

    # Capture and retrieve the rendered image
    width, height, rgb_img, _, _ = p.getCameraImage(
        width=1920, height=1080, 
        viewMatrix=view_matrix, projectionMatrix=projection_matrix,
        shadow=0, flags=p.ER_NO_SEGMENTATION_MASK, renderer=p.ER_TINY_RENDERER
    )

    # Convert to an OpenCV-compatible format
    rgb_img = np.reshape(rgb_img, (height, width, 4))[:, :, :3]  # Extract RGB channels
    rgb_img = rgb_img.astype(np.uint8)
    rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR)  # Convert from RGB to BGR for OpenCV

    return rgb_img


def adjust_fov_parameters():
    """
    Creates a window with sliders to dynamically adjust the Field of View (FoV) parameters.
    """

    global h_fov, v_fov

    cv2.namedWindow("Adjust FoV Parameters", cv2.WINDOW_NORMAL)

    # Create sliders for adjusting FoV
    cv2.createTrackbar("Horizontal FOV (deg)", "Adjust FoV Parameters", int(h_fov), 180, lambda x: None)
    cv2.createTrackbar("Vertical FOV (deg)", "Adjust FoV Parameters", int(v_fov), 180, lambda x: None)


def update_fov_parameters():
    """
    Updates the FoV parameters based on the slider positions.
    """

    global h_fov, v_fov

    # Read the values from the sliders
    h_fov = cv2.getTrackbarPos("Horizontal FOV (deg)", "Adjust FoV Parameters")
    v_fov = cv2.getTrackbarPos("Vertical FOV (deg)", "Adjust FoV Parameters")