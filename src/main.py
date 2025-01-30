import pybullet as p
import time
import numpy as np
from simulation.robot import Robot
from simulation.pathfinding import Pathfinding
from simulation.pybullet_manager import PyBulletManager
from utils.math_helpers import interpolate_position
from utils.visualization import Visualization
import config as config
from simulation.setup import load_objects, create_environment, initialize_map, initialize_cans

def main():
    # Initialisation de PyBullet
    # ==========================
    pybullet_manager = PyBulletManager(debug=config.DEBUG)
    pybullet_manager.reset_camera(distance=2.0, yaw=0, pitch=-45, target=[0, 0, 0])
    pybullet_manager.set_real_time_simulation(True)

    # Créer l'environnement (sol et table)
    plane_id, table_id = create_environment(pybullet_manager)

    # Charger les conserves
    can_ids = load_objects(pybullet_manager, "src/urdf_models/conserve.urdf", [0.7071, 0, 0, 0.7071], config.CAN_POSITIONS)

    # Charger les planches
    plank_horizontal_ids = load_objects(pybullet_manager, "src/urdf_models/planche.urdf", [0.7071, 0, 0, 0.7071], config.PLANK_POSITIONS_HORIZONTAL)
    plank_vertical_ids = load_objects(pybullet_manager, "src/urdf_models/planche.urdf", [0.5, 0.5, 0.5, 0.5], config.PLANK_POSITIONS_VERTICAL)

    # Initialisation du robot
    robot = Robot("src/urdf_models/robot_cube.urdf", config.ROBOT_START_POS, [0, 0, 0, 1])

    # Fin initialisation pybullet
    # ===========================

    # Obstacles
    ox, oy = initialize_map(config.X_DIM, config.Y_DIM, config.CELL_SIZE)
    ox_cans, oy_cans = initialize_cans(config.X_DIM, config.Y_DIM, config.CELL_SIZE, config.CAN_RADIUS)
    spoofed_ox, spoofed_oy = [ox_cans], [oy_cans]
    obstacles = [ox, oy, spoofed_ox, spoofed_oy]
    
    # Visualisation
    visualization = Visualization(obstacles, config.X_DIM, config.Y_DIM, config.CELL_SIZE)

    # Points de départ et d’arrivée
    start, goal = visualization.get_start_goal()

    # Algorithme A*
    #pathfinder = Pathfinding(grid.grid)
    #path = pathfinder.a_star(start, goal)

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