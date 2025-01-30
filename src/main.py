import pybullet as p
import time
import numpy as np
from simulation.grid import Grid
from simulation.robot import Robot
from simulation.pathfinding import AStar, DStarLite
from simulation.pybullet_manager import PyBulletManager
from simulation.setup import load_objects, create_environment, initialize_map, initialize_cans
from utils.math_helpers import interpolate_position, extract_path
from utils.visualization import Visualization
import config as config

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

    print("""
======================================================
Fin initialisation pybullet
======================================================
    """)

    grid = Grid()

    # Obstacles
    ox, oy = initialize_map(grid)
    ox_cans, oy_cans = initialize_cans(grid, config.CAN_RADIUS)
    spoofed_ox, spoofed_oy = [ox_cans], [oy_cans]
    obstacles = [ox, oy, spoofed_ox, spoofed_oy]
    
    # Visualisation
    visualization = Visualization(obstacles)

    # Points de départ et d’arrivée
    start, goal = visualization.get_start_goal(grid)

    # Initialisation du robot
    real_start = grid.grid_index_to_position(start)
    robot = Robot("src/urdf_models/robot_cube.urdf", [real_start[0], real_start[1], config.TABLE_HEIGHT + 0.1], [0, 0, 0, 1])
    pybullet_manager.step_simulation()

    # Algorithme A*
    #a_star = AStar(grid.grid)
    #path = a_star.main(start, goal)

    dstarlite = DStarLite(ox, oy)
    path_exists, pathx, pathy, compute_time = dstarlite.main(start=start,
                                                            goal=goal,
                                                            spoofed_ox=spoofed_ox,
                                                            spoofed_oy=spoofed_oy)
    if path_exists:
        cpx, cpy, extraction_time= extract_path(pathx, pathy, threshold=4.5)

    visualization.show_path(path_exists, pathx, pathy, cpx, cpy)

    # Gestion du chemin et déplacement du robot
    if not path_exists:
        print("No path found", end=" ")
        print(f"({int(compute_time*1e3)}ms)")
    else:
        print("Path found", end=" ")

        interpolation_steps = 20

        for main_point_id in range(len(cpx)):
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

    print("""\n
======================================================

======================================================
    """)
    pybullet_manager.disconnect()

if __name__ == "__main__":
    main()