import os
os.environ['PYBULLET_USE_EGL'] = '1'
import pybullet as p
import time
import numpy as np
from simulation.grid import Grid
from simulation.robot import Robot
from simulation.pathfinding import AStar, DStarLite
from simulation.pathfollowing import extract_waypoints
from simulation.pybullet_manager import PyBulletManager
from simulation.setup import load_objects, create_environment, initialize_map, initialize_cans
from utils.math_helpers import interpolate_position
from utils.visualization import VisualizationMPL
import config
from utils.virtualCamera import init_camera, read_camera
import cv2
from utils.imageTransform import calibrationAndTransform

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

    cam1 = init_camera(config.CAM1_POS, config.CAM1_ORIENTATION_DEG)
    # cam2 = init_camera(config.CAM2_POS, config.CAM2_ORIENTATION_DEG)

    cv2.namedWindow("Camera 1 Stream", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Camera 1 Stream", 800, 500)
    
    cv2.namedWindow("Camera 2 Stream", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Camera 2 Stream", 800, 500)

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
    visualization = VisualizationMPL(obstacles)

    # Points de départ et d’arrivée
    start, goal = visualization.get_start_goal(grid)

    # Initialisation du robot
    real_start = grid.grid_index_to_position(start)
    robot = Robot("src/urdf_models/robot_cube.urdf", [real_start[0], real_start[1], config.TABLE_HEIGHT + 0.1], [0, 0, 0, 1])
    pybullet_manager.step_simulation()

    algo = 1 + 0

    if algo == 1:
        # Algorithme A*
        a_star = AStar(ox, oy, spoofed_ox, spoofed_oy)
        path_exists, pathx, pathy, compute_time = a_star.main(start, goal)
    else:
        # Algorithme D*Lite
        dstarlite = DStarLite(ox, oy)
        path_exists, pathx, pathy, compute_time = dstarlite.main(start=start,
                                                                goal=goal,
                                                                spoofed_ox=spoofed_ox,
                                                                spoofed_oy=spoofed_oy)
    if path_exists:
        print("Path found", end=" ")
        print(f"({int(compute_time*1e3)}ms)\n")
        wpx, wpy, extraction_time= extract_waypoints(pathx, pathy, threshold=1)
        visualization.show_path(path_exists, pathx, pathy, wpx, wpy)
    else:
        print("No path found", end=" ")
        print(f"({int(compute_time*1e3)}ms)")
        visualization.show_path(path_exists, pathx, pathy, [], [])

    # Gestion du chemin et déplacement du robot
    if path_exists:
        print("Following the path :")

        for main_point_id in range(len(wpx)):
            target_position = (wpx[main_point_id], wpy[main_point_id])
            x_pos, y_pos, z_pos = grid.grid_index_to_position(target_position)
            next_pos = (x_pos, y_pos)

            while True:
                robot_pos, _ = robot.get_position_and_orientation()
                robot_pos_2d = robot_pos[:2]

                # Calculate the direction to the target position
                delta_x = next_pos[0] - robot_pos_2d[0]
                delta_y = next_pos[1] - robot_pos_2d[1]
                distance_to_target = np.sqrt(delta_x**2 + delta_y**2)

                # Normalize the direction and set the velocity
                if distance_to_target > 0.02:
                    direction = [delta_x / distance_to_target, delta_y / distance_to_target]
                    linear_velocity = [config.ROBOT_SPEED * direction[0], config.ROBOT_SPEED * direction[1], 0]
                    robot.set_velocity(linear_velocity=linear_velocity)
                else:
                    print(f"Reached waypoint {main_point_id}: {next_pos}")
                    # robot.set_velocity([0, 0, 0])
                    break
                
                # Display cameras
                t0 = time.time()
                rgb_img1 = read_camera(cam1, divisor=3/4)
                #rgb_img2 = read_camera(cam2, divisor=1)
                t1 = time.time()
                print(f"Camera read time: {(t1 - t0)*1000:.2f} ms", end="\r")

                # rgb_img1 = calibrationAndTransform(rgb_img1, 1)
                # rgb_img2 = calibrationAndTransform(rgb_img2, 2)

                cv2.imshow("Camera 1 Stream", rgb_img1)
                #cv2.imshow("Camera 2 Stream", rgb_img2)

                c = cv2.waitKey(int(1/60.0*1000))

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