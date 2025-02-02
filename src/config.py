TABLE_HEIGHT = 0.1
TABLE_LENGTH = 3
TABLE_WIDTH = 2
CELL_SIZE = 0.05
ROBOT_START_POS = [0, 0, TABLE_HEIGHT + 0.1]
ROBOT_SPEED = 0.4
ROBOT_ROTATION_SPEED = 5
DEBUG = False

CAN_POSITIONS = [
    # De gauche a droite de haut en bas
    # Moitie gauche
    [-1.425, 0.475, 0.1],
    [-1.425, 0.375, 0.1],
    [-1.425, 0.275, 0.1],
    [-1.425, 0.175, 0.1],
    [-1.425, -0.425, 0.1],
    [-1.425, -0.525, 0.1],
    [-1.425, -0.625, 0.1],
    [-1.425, -0.725, 0.1],
    [-0.825, 0.725, 0.1],
    [-0.725, 0.725, 0.1],
    [-0.625, 0.725, 0.1],
    [-0.525, 0.725, 0.1],
    [-0.875, -0.75, 0.1],
    [-0.775, -0.75, 0.1],
    [-0.675, -0.75, 0.1],
    [-0.575, -0.75, 0.1],
    [-0.55, -0.05, 0.1],
    [-0.45, -0.05, 0.1],
    [-0.35, -0.05, 0.1],
    [-0.25, -0.05, 0.1],
    # Moitie droite
    [1.425, 0.475, 0.1],
    [1.425, 0.375, 0.1],
    [1.425, 0.275, 0.1],
    [1.425, 0.175, 0.1],
    [1.425, -0.425, 0.1],
    [1.425, -0.525, 0.1],
    [1.425, -0.625, 0.1],
    [1.425, -0.725, 0.1],
    [0.825, 0.725, 0.1],
    [0.725, 0.725, 0.1],
    [0.625, 0.725, 0.1],
    [0.525, 0.725, 0.1],
    [0.875, -0.75, 0.1],
    [0.775, -0.75, 0.1],
    [0.675, -0.75, 0.1],
    [0.575, -0.75, 0.1],
    [0.55, -0.05, 0.1],
    [0.45, -0.05, 0.1],
    [0.35, -0.05, 0.1],
    [0.25, -0.05, 0.1],
]

PLANK_POSITIONS_HORIZONTAL = [
    # De gauche a droite de haut en bas
    # Moitie gauche
    [-0.675, 0.725, 0.15],
    [-0.675, 0.725, 0.2],
    [-0.725, -0.75, 0.15],
    [-0.725, -0.75, 0.2],
    [-0.4, -0.05, 0.15],
    [-0.4, -0.05, 0.2],
    # Moitie droite
    [0.675, 0.725, 0.15],
    [0.675, 0.725, 0.2],
    [0.725, -0.75, 0.15],
    [0.725, -0.75, 0.2],
    [0.4, -0.05, 0.15],
    [0.4, -0.05, 0.2],
]

PLANK_POSITIONS_VERTICAL = [
    # De gauche a droite de haut en bas
    # Moitie gauche
    [-1.425, 0.325, 0.15],
    [-1.425, 0.325, 0.2],
    [-1.425, -0.6, 0.15],
    [-1.425, -0.6, 0.2],
    # Moitie droite
    [1.425, 0.325, 0.15],
    [1.425, 0.325, 0.2],
    [1.425, -0.6, 0.15],
    [1.425, -0.6, 0.2],
]