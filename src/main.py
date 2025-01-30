import pybullet as p
import time
import numpy as np
from simulation.grid import Grid
from simulation.robot import Robot
from simulation.pathfinding import AStar, DStarLite
from simulation.pybullet_manager import PyBulletManager
from simulation.setup import load_objects, create_environment, initialize_map, initialize_cans
from utils.math_helpers import interpolate_position, extract_waypoints
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
    #plank_horizontal_ids = load_objects(pybullet_manager, "src/urdf_models/planche.urdf", [0.7071, 0, 0, 0.7071], config.PLANK_POSITIONS_HORIZONTAL)
    #plank_vertical_ids = load_objects(pybullet_manager, "src/urdf_models/planche.urdf", [0.5, 0.5, 0.5, 0.5], config.PLANK_POSITIONS_VERTICAL)

    print("""
======================================================
Fin initialisation pybullet
======================================================
    """)

    grid = Grid()

    # Obstacles
    ox, oy = initialize_map(grid)
    ox_cans, oy_cans = initialize_cans(grid, config.CAN_RADIUS+config.ROBOT_RADIUS)
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
        wpx, wpy, extraction_time= extract_waypoints(pathx, pathy, threshold=4.5)

    visualization.show_path(path_exists, pathx, pathy, wpx, wpy)

    # Gestion du chemin et déplacement du robot
    if not path_exists:
        print("No path found", end=" ")
        print(f"({int(compute_time*1e3)}ms)")
    else:
        print("Path found", end=" ")
        print(f"({int(compute_time*1e3)}ms)\n")

        print("Following the path :")

        for main_point_id in range(len(wpx)):
            target_position = (wpx[main_point_id], wpy[main_point_id])
            x_pos, y_pos, z_pos = grid.grid_index_to_position(target_position)
            next_pos = [x_pos, y_pos]

            while True:
                robot_pos, _ = robot.get_position_and_orientation()
                robot_pos_2d = robot_pos[:2]

                # Calculate the direction to the target position
                delta_x = next_pos[0] - robot_pos_2d[0]
                delta_y = next_pos[1] - robot_pos_2d[1]
                distance_to_target = np.sqrt(delta_x**2 + delta_y**2)

                # Check if the robot is close enough to the target position
                if distance_to_target < 0.01: # proximity_threshold de 1cm
                    print(f"Reached waypoint {main_point_id}: {next_pos}")

                    break  # Exit the loop to move to the next waypoint

                # Normalize the direction and set the velocity
                if distance_to_target > 0:
                    direction = [delta_x / distance_to_target, delta_y / distance_to_target]
                    linear_velocity = [config.ROBOT_SPEED * direction[0], config.ROBOT_SPEED * direction[1], 0]
                    robot.set_velocity(linear_velocity=linear_velocity)
                else:
                    robot.set_velocity([0, 0, 0])  # Stop if very close to the target


                    pybullet_manager.step_simulation()
                    time.sleep(1 / 60.0)
        robot.set_velocity([0, 0, 0])

    print("""\n
======================================================

======================================================
    """)
    time.sleep(2)
    pybullet_manager.disconnect()

if __name__ == "__main__":
    main()