import os

host = os.environ['SESSION_MANAGER'].split(',')[0].split(':')[0].split('/')[1]
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
    import numpy as np
    import cv2
    import queue
except Exception as e:
    print(f"IMPORT ERROR ! ({e})")
    print("Run 'make environment' or 'make install' to get the required modules")
    exit(1)

from simulation.grid import Grid
from simulation.robot import Robot
from simulation.camera import (
    VirtualCamera,
    NoVirtualCamera,
    display_images
)
from simulation.pathfinding import AStar
from simulation.pathfollowing import extract_waypoints
from simulation.simulation import Simulation
from simulation.setup import initialize_map, initialize_cans
from utils.visualization import VisualizationMPL
import config

def main(fps=1000):
    # Initialisation de la simulation SAPIEN
    # =======================================================
    simulation = Simulation(with_viewer=True)
    simulation.add_ground()
    simulation.add_lights()
    simulation.add_table()
    simulation.add_cans()
    simulation.step()

    cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)

    camera_1 = NoVirtualCamera(
        scene=simulation.scene,
        img_types=['Color'],
    )
    camera_1.run()

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
    robot = Robot(scene=simulation.scene, position=[real_start[0], real_start[1], config.ROBOT_START_POS[2]])

    a_star = AStar(ox, oy, spoofed_ox, spoofed_oy)
    path_exists, pathx, pathy, compute_time = a_star.main(start, goal)

    if path_exists:
        print("Path found", end=" ")
        print(f"({int(compute_time*1e3)}ms)\n")
        wpx, wpy, extraction_time= extract_waypoints(pathx, pathy, threshold=1)
        visualization.show_path(path_exists, pathx, pathy, wpx, wpy)
    else:
        print("No path found", end=" ")
        print(f"({int(compute_time*1e3)}ms)")
        visualization.show_path(path_exists, pathx, pathy, [], [])

    target_pose = [grid.grid_index_to_position([wpx[main_point_id], wpy[main_point_id]]) \
        for main_point_id in range(len(wpx))]
    target_pose_index = 0

    t0 = time.time()
    real_fps = 0
    t_proc = 0
    while not simulation.viewer.closed:
        simulation.step()

        reached, distance = robot.move_to(
            target_pose[target_pose_index],
            speed_factor=0.8
        )
        if reached and target_pose_index < len(target_pose) - 1:
            target_pose_index += 1

        # Check for captured images (non-blocking)
        try:
            processed_images, t_proc = camera_1.image_queue.get_nowait()
            # Process or store images as needed
            key = display_images(processed_images)
        except queue.Empty:
            pass

        # FPS Tracking
        real_fps += 1
        current_time = time.time()
        if current_time - t0 >= 1.0:
            print(f"FPS: {real_fps} | Processing Time: {t_proc:.4f} seconds", end="\r")
            real_fps = 0
            t0 = current_time

        # Window and exit handling
        try:
            if cv2.getWindowProperty("Camera", cv2.WND_PROP_VISIBLE) < 1:
                break
            
            key = cv2.waitKey(1000 // fps) & 0xFF
            if key == 27:  # ESC key
                break
        except Exception:
            break
    
    # Cleanup
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
    