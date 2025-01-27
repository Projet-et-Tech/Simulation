import pybullet as p
import pybullet_data
import numpy as np
import cv2
import math

def init_camera(cam_position, cam_orientation_deg):

    # Convert orientation from degrees to radians
    cam_orientation = [math.radians(angle) for angle in cam_orientation_deg]

    # Convert orientation to quaternion
    cam_orientation_quat = p.getQuaternionFromEuler(cam_orientation)

    # Extract forward and up vectors from quaternion
    forward_vector = p.multiplyTransforms(
        [0, 0, 0], cam_orientation_quat, [1, 0, 0], [0, 0, 0, 1]
    )[0]  # X-axis of the camera (forward vector)

    up_vector = p.multiplyTransforms(
        [0, 0, 0], cam_orientation_quat, [0, 0, 1], [0, 0, 0, 1]
    )[0]  # Z-axis of the camera (up vector)

    # Compute the target position based on the forward vector
    cam_target_pos = [cam_position[i] + forward_vector[i] for i in range(3)]

    # -------------------------- Camera parameters for realistic setup --------------------------
    global h_fov, v_fov
    h_fov = 43  # Horizontal FoV in degrees (43)
    v_fov = 50  # Vertical FoV in degrees (67.8)
    aspect_ratio = math.tan(math.radians(h_fov) / 2) / math.tan(math.radians(v_fov) / 2)  # Derived aspect ratio

    # Render settings
    near_plane = 0.1  # Near clipping plane
    far_plane = 100   # Far clipping plane

    # Convert horizontal and vertical FoV into a single FoV for PyBullet
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
    # -------------------------------------------------------------------------------------------

    # Convert orientation to quaternion
    cam_orientation_quat = p.getQuaternionFromEuler(cam_orientation)

    # Add a black cube to represent the camera
    box_size = 0.01
    camera_visual_shape_id = p.createVisualShape(
        shapeType=p.GEOM_BOX, halfExtents=[1.5*box_size, 2*box_size, box_size], rgbaColor=[1, 0, 0, 1]
    )

    p.createMultiBody(
        baseMass=0,  # Static object
        baseVisualShapeIndex=camera_visual_shape_id,
        basePosition=cam_position,  # Set the cube's position to the camera's position
        baseOrientation=cam_orientation_quat  # Apply the camera's orientation
    )

    return view_matrix, projection_matrix


def read_camera(camera_infos):

    view_matrix, projection_matrix = camera_infos

    #----------------------------------------------------------------------------------------
    global h_fov, v_fov
    aspect_ratio = math.tan(math.radians(h_fov) / 2) / math.tan(math.radians(v_fov) / 2)  # Derived aspect ratio

    # Render settings
    near_plane = 0.1  # Near clipping plane
    far_plane = 100   # Far clipping plane

    # Convert horizontal and vertical FoV into a single FoV for PyBullet
    diagonal_fov = 2 * math.atan(math.sqrt(math.tan(math.radians(h_fov) / 2) ** 2 +
                                           math.tan(math.radians(v_fov) / 2) ** 2))
    diagonal_fov_degrees = math.degrees(diagonal_fov)
    projection_matrix = p.computeProjectionMatrixFOV(diagonal_fov_degrees, aspect_ratio, near_plane, far_plane)
    #----------------------------------------------------------------------------------------

    # Capture and display the image
    width, height, rgb_img, _, _ = p.getCameraImage(
        width=1920, height=1080, viewMatrix=view_matrix, projectionMatrix=projection_matrix, shadow=0, flags=p.ER_NO_SEGMENTATION_MASK, renderer=p.ER_TINY_RENDERER
    )

    # Convert to OpenCV-compatible format
    rgb_img = np.reshape(rgb_img, (height, width, 4))[:, :, :3]
    rgb_img = rgb_img.astype(np.uint8)
    rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR)

    return rgb_img


def adjust_fov_parameters():
    global h_fov, v_fov
    """Creer une fenetre avec des curseurs pour ajuster les parametres de FoV dynamiquement."""
    cv2.namedWindow("Adjust FoV Parameters", cv2.WINDOW_NORMAL)

    # Creer les curseurs pour ajuster les parametres
    cv2.createTrackbar("Horizontal FOV (deg)", "Adjust FoV Parameters", int(h_fov), 180, lambda x: None)
    cv2.createTrackbar("Vertical FOV (deg)", "Adjust FoV Parameters", int(v_fov), 180, lambda x: None)


def update_fov_parameters():
    """Met a jour les parametres en fonction des positions des curseurs."""
    global h_fov, v_fov

    # Lire les valeurs des curseurs
    h_fov = cv2.getTrackbarPos("Horizontal FOV (deg)", "Adjust FoV Parameters")
    v_fov = cv2.getTrackbarPos("Vertical FOV (deg)", "Adjust FoV Parameters")