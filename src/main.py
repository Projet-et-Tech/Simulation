import pybullet as p
import time
import numpy as np
from simulation.robot import Robot
from simulation.objects import SimulationObject
from simulation.pathfinding import Pathfinding
from simulation.grid import Grid
from simulation.pybullet_manager import PyBulletManager
from utils.math_helpers import interpolate_position
from utils.visualization import Visualization
import config as config

def main():
    # Initialisation de PyBullet
    pybullet_manager = PyBulletManager(debug=config.DEBUG)
    pybullet_manager.reset_camera(distance=2.0, yaw=0, pitch=-45, target=[0, 0, 0])
    pybullet_manager.set_real_time_simulation(True)

    # Création du sol
    plane_id = pybullet_manager.load_urdf("plane.urdf", [0, 0, 0], [0, 0, 0, 1])

    # Création de la table
    table_position = [0, 0, config.TABLE_HEIGHT / 2]
    table_id = pybullet_manager.load_urdf("src/urdf_models/table_eurobot2025.urdf", table_position, [0, 0, 0, 1])

    # Création des conserves
    can_positions = [
        # De gauche à droite de haut en bas
        [-1.425, 0.475, 0.1], [-1.425, 0.375, 0.1], [-1.425, 0.275, 0.1], [-1.425, 0.175, 0.1],
        [-1.425, -0.425, 0.1], [-1.425, -0.525, 0.1], [-1.425, -0.625, 0.1], [-1.425, -0.725, 0.1],
        [-0.825, 0.725, 0.1], [-0.725, 0.725, 0.1], [-0.625, 0.725, 0.1], [-0.525, 0.725, 0.1],
        [-0.875, -0.75, 0.1], [-0.775, -0.75, 0.1], [-0.675, -0.75, 0.1], [-0.575, -0.75, 0.1],
        [-0.55, -0.05, 0.1], [-0.45, -0.05, 0.1], [-0.35, -0.05, 0.1], [-0.25, -0.05, 0.1],
        [1.425, 0.475, 0.1], [1.425, 0.375, 0.1], [1.425, 0.275, 0.1], [1.425, 0.175, 0.1],
        [1.425, -0.425, 0.1], [1.425, -0.525, 0.1], [1.425, -0.625, 0.1], [1.425, -0.725, 0.1],
        [0.825, 0.725, 0.1], [0.725, 0.725, 0.1], [0.625, 0.725, 0.1], [0.525, 0.725, 0.1],
        [0.875, -0.75, 0.1], [0.775, -0.75, 0.1], [0.675, -0.75, 0.1], [0.575, -0.75, 0.1],
        [0.55, -0.05, 0.1], [0.45, -0.05, 0.1], [0.35, -0.05, 0.1], [0.25, -0.05, 0.1]
    ]
    can_ids = [SimulationObject("src/urdf_models/conserve.urdf", pos, [0.7071, 0, 0, 0.7071]).object_id for pos in can_positions]

    # Création des planches
    plank_positions_horizontal = [
        [-0.675, 0.725, 0.15], [-0.675, 0.725, 0.2], [-0.725, -0.75, 0.15], [-0.725, -0.75, 0.2],
        [-0.4, -0.05, 0.15], [-0.4, -0.05, 0.2], [0.675, 0.725, 0.15], [0.675, 0.725, 0.2],
        [0.725, -0.75, 0.15], [0.725, -0.75, 0.2], [0.4, -0.05, 0.15], [0.4, -0.05, 0.2]
    ]
    plank_horizontal_ids = [SimulationObject("src/urdf_models/planche.urdf", pos, [0.7071, 0, 0, 0.7071]).object_id for pos in plank_positions_horizontal]

    plank_positions_vertical = [
        [-1.425, 0.325, 0.15], [-1.425, 0.325, 0.2], [-1.425, -0.6, 0.15], [-1.425, -0.6, 0.2],
        [1.425, 0.325, 0.15], [1.425, 0.325, 0.2], [1.425, -0.6, 0.15], [1.425, -0.6, 0.2]
    ]
    plank_vertical_ids = [SimulationObject("src/urdf_models/planche.urdf", pos, [0.5, 0.5, 0.5, 0.5]).object_id for pos in plank_positions_vertical]

    # Initialisation du robot
    robot = Robot("src/urdf_models/robot_cube.urdf", config.ROBOT_START_POS, [0, 0, 0, 1])

    # Création de la grille et ajout des obstacles
    grid = Grid()
    grid.add_obstacles(can_positions + plank_positions_horizontal + plank_positions_vertical)

    # Ajout de la scène
    scene = grid.fill_indexes_between_corners([-0.45, 1, 0.1], [0.45, 0.55, 0.1])
    left_ramp = grid.fill_indexes_between_corners([-0.85, 1, 0.1], [-0.45, 0.8, 0.1])
    right_ramp = grid.fill_indexes_between_corners([0.45, 1, 0.1], [0.85, 0.8, 0.1])
    grid.add_scene(scene + left_ramp + right_ramp)

    # Points de départ et d’arrivée
    start = grid.position_to_grid_index(config.ROBOT_START_POS)
    goal = grid.position_to_grid_index([-1.125, 0.8, 0.1])

    # Algorithme A*
    pathfinder = Pathfinding(grid.grid)
    path = pathfinder.a_star(start, goal)

    # Visualisation
    visualization = Visualization(grid.grid, config.CELL_SIZE, start, goal)

    # Gestion du chemin et déplacement du robot
    if path:
        print("Chemin trouvé :", path)
        interpolation_steps = 20

        for step in path:
            x_pos, y_pos, z_pos = grid.grid_index_to_position(step)
            next_pos = [x_pos, y_pos]

            robot_pos, _ = robot.get_position_and_orientation()
            robot_pos_2d = robot_pos[:2]

            interpolated_positions = interpolate_position(robot_pos_2d, next_pos, interpolation_steps)

            for inter_pos in interpolated_positions:
                delta_x = inter_pos[0] - robot_pos[0]
                delta_y = inter_pos[1] - robot_pos[1]
                distance_to_travel = np.sqrt(delta_x**2 + delta_y**2)

                if distance_to_travel > 0.01:
                    direction_angle = np.arctan2(delta_y, delta_x)
                    linear_velocity = [
                        config.ROBOT_SPEED * np.cos(direction_angle),
                        config.ROBOT_SPEED * np.sin(direction_angle),
                        0
                    ]
                    robot.set_velocity(linear_velocity=linear_velocity)

                    pybullet_manager.step_simulation()
                    time.sleep(1 / 240.0)

                grid.grid[step[0]][step[1]] = 4
                visualization.update_grid(grid.grid)

            grid.grid[step[0]][step[1]] = 0
            visualization.update_grid(grid.grid)

        robot.set_velocity([0, 0, 0])
    else:
        print("Aucun chemin trouvé.")

    visualization.finalize()
    pybullet_manager.disconnect()

if __name__ == "__main__":
    main()