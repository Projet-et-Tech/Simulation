import os

host = ""
for host_var in ['SESSION_MANAGER', 'HOSTNAME']:
    if host_var in os.environ:
        host = os.environ[host_var].split(',')[0].split(':')[0].split('/')[1] 
print(f"Current host: {host}")
if host == 'PT-PC-OptiPlex-7060':
    print("""
#################################################
########  Le CPU ne tient pas la charge  ########
#################################################
    """)
    # Set the VK_ICD_FILENAMES environment variable
    os.environ['VK_ICD_FILENAMES'] = '/usr/share/vulkan/icd.d/lvp_icd.x86_64.json'

try:
    import time
    import cv2
    import queue
    import numpy as np
except Exception as e:
    print(f"IMPORT ERROR ! ({e})")
    print("""
> Run 'make environment' to setup the python venv and install the required modules

> Run 'make install' to install the required modules
     """)
    exit(1)


TABLE_HEIGHT = 0.1
TABLE_LENGTH = 3
TABLE_WIDTH = 2
CELL_SIZE = 0.04

# -------- ROBOT -------- #

ROBOT_ID = 2

ROBOT_START_POS = [0, 0, TABLE_HEIGHT + 0.1]
ROBOT_SPEED = 0.4
ROBOT_ROTATION_SPEED = 5
DEBUG = False

# -------- OBJECTS -------- #

BOX_HEIGHT = 0.03
BOX_LENGTH = 0.15
BOX_WIDTH = 0.05

ROBOT_RADIUS = 0.45/1.9

# -------- ARUCO MARKERS -------- #

HORIZONTAL_DISTANCE_CM = 80     # Table horizontal distance between Aruco (from centers)
VERTICAL_DISTANCE_CM = 180      # Table vertical distance between Aruco (from centers)
HORIZONTAL_MARGIN_CM = 70       # Table horizontal margin around Aruco (from centers)
VERTICAL_MARGIN_CM = 60         # Table vertical margin around Aruco (from centers)
SCALE_FACTOR = 2

CORNER_TOP_LEFT_ID = 22
CORNER_TOP_RIGHT_ID = 20
CORNER_BOTTOM_LEFT_ID = 23
CORNER_BOTTOM_RIGHT_ID = 21

ARUCO_CORNERS = {
    22: np.array([
        [2450, 1450, 0],
        [2450, 1350, 0],
        [2350, 1350, 0],
        [2350, 1450, 0]
    ]),

    20: np.array([
        [2450, 650, 0],
        [2450, 550, 0],
        [2350, 550, 0],
        [2350, 650, 0]
    ]),

    23: np.array([
        [650, 1450, 0],
        [650, 1350, 0],
        [550, 1350, 0],
        [550, 1450, 0]
    ]),

    21: np.array([
        [650, 650, 0],
        [650, 550, 0],
        [550, 550, 0],
        [550, 650, 0]
    ])
}


# -------- CAMERA -------- #

CAM1_POS = [1.5, 0, 1]
CAM1_ORIENTATION_DEG = [180, 120, 0]

CAM2_POS = [-1.5, -1, 1]
CAM2_ORIENTATION_DEG = [0, 40, 40]

# -------- PAMI -------- #

PAMI_HEIGHT = 0.1
PAMI_ID = 6

# -------- OBSTACLES -------- #

BOX_POSITIONS_HORIZONTAL = [
    # De gauche a droite de haut en bas
    # Moitie gauche
    [-1.3, 0.525, 0.1],
    [-1.3, 0.575, 0.1],
    [-1.3, 0.625, 0.1],
    [-1.3, 0.675, 0.1],

    [-1.3, -0.125, 0.1],
    [-1.3, -0.175, 0.1],
    [-1.3, -0.225, 0.1],
    [-1.3, -0.275, 0.1],

    # Moitie droite
    [1.3, 0.525, 0.1],
    [1.3, 0.575, 0.1],
    [1.3, 0.625, 0.1],
    [1.3, 0.675, 0.1],

    [1.3, -0.125, 0.1],
    [1.3, -0.175, 0.1],
    [1.3, -0.225, 0.1],
    [1.3, -0.275, 0.1],
]


BOX_POSITIONS_VERTICAL = [
    # De gauche a droite de haut en bas
    # Moitie gauche
    [0.475, 0.8, 0.1],
    [0.425, 0.8, 0.1],
    [0.375, 0.8, 0.1],
    [0.325, 0.8, 0.1],

    [0.425, 0.2, 0.1],
    [0.375, 0.2, 0.1],
    [0.325, 0.2, 0.1],
    [0.275, 0.2, 0.1],

    # Moitie droite
    [-0.475, 0.8, 0.1],
    [-0.425, 0.8, 0.1],
    [-0.375, 0.8, 0.1],
    [-0.325, 0.8, 0.1],

    [-0.425, 0.2, 0.1],
    [-0.375, 0.2, 0.1],
    [-0.325, 0.2, 0.1],
    [-0.275, 0.2, 0.1],
]