import numpy as np

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