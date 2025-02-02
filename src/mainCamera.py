import pybullet as p
import cv2
import time

from simulation.pybullet_manager import PyBulletManager
from config import TABLE_LENGTH, TABLE_WIDTH, ROBOT_START_POS, DEBUG, CAM_POS, CAM_ORIENTATION_DEG, PAMI_HEIGHT
from simulation.setup import create_environment

from utils.virtualCamera import init_camera, read_camera
from utils.imageTransform import calibrationAndTransform
from utils.perspectiveCorrection import convert_2D_to_3D

# ------------------ GLOBAL VARIABLES ------------------

clicked_point= None

# ------------------ MOUSE CLICK CALLBACK ------------------ (temp, will be auto detection by aruco on PAMIs)

def mouse_callback(event, x, y, flags, param):
    """Captures the user's click position on the image."""
    global clicked_point
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_point = (x, y)
        print(f"Clicked at: {clicked_point}")
        cv2.destroyAllWindows()  # Close the window after clicking

def capture_click(frame):
    """Displays the camera feed and waits for the user to click on an object."""
    global clicked_point
    cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)
    cv2.imshow('Camera Feed', frame)
    cv2.setMouseCallback('Camera Feed', mouse_callback)

    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break  # Exit loop if 'q' is pressed
        if clicked_point:
            return clicked_point  # Return the clicked coordinates

    cv2.destroyAllWindows()

# ------------------ PYBULLET SIMULATION SETUP ------------------

debug = DEBUG

# Initialize PyBullet with PyBulletManager
pybullet_manager = PyBulletManager(debug=debug)
pybullet_manager.reset_camera(distance=2.0, yaw=0, pitch=-45, target=[0, 0, 0])
pybullet_manager.set_real_time_simulation(True)

# Create environment (ground and table)
plane_id, table_id = create_environment(pybullet_manager)

# ------------------ LOAD ROBOT ------------------

robot_id = pybullet_manager.load_urdf("src/urdf_models/robot_pami.urdf", ROBOT_START_POS, [0, 0, 0, 1])

# ------------------ CAMERA CAPTURE & OBJECT DETECTION ------------------

cam1 = init_camera(CAM_POS, CAM_ORIENTATION_DEG)
time.sleep(2)   # wait for PAMI to fall before taking picture (temp, not in video)
rgb_img1 = read_camera(cam1)

transformed_frame = calibrationAndTransform(rgb_img1, "Cam1")

# Capture the user's click position
y, x = capture_click(transformed_frame)

# ------------------ CONVERT 2D IMAGE COORDINATES TO 3D ------------------

x_true, y_true = convert_2D_to_3D(x, y, transformed_frame, TABLE_LENGTH, TABLE_WIDTH, CAM_POS, PAMI_HEIGHT, ROBOT_START_POS)

# ------------------ VISUALIZATION IN PYBULLET ------------------

while True:
    # Corrected detection (Green)
    p.addUserDebugLine([x_true, y_true, PAMI_HEIGHT], [x_true, y_true, 1], [0, 1, 0], 2, 0)
    p.addUserDebugText("Il est la", [x_true, y_true, 1], textColorRGB=[0, 1, 0], textSize=1.5)

# ------------------ CLEANUP ------------------

cv2.destroyAllWindows()
p.disconnect()