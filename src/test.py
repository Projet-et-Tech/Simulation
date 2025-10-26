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
    import sapien
    from sapien.utils import Viewer
    import numpy as np
    import quaternion
    import cv2
    import threading
    import queue
except Exception as e:
    print(f"IMPORT ERROR ! ({e})")
    print("""
> Run 'make environment' to setup the python venv and install the required modules

> Run 'make install' to install the required modules
     """)
    exit(1)

from simulation.simulation import Simulation
from simulation.robot import Robot
from simulation.camera import (
    VirtualCamera,
    NoVirtualCamera,
    display_images
)
import config


def main(fps=1000):
    # Initialize SAPIEN engine
    simulation = Simulation(with_viewer=False)
    simulation.add_ground()
    simulation.add_lights()
    simulation.add_table()
    simulation.add_boxes()

    robot = Robot(scene=simulation.scene)

    cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)

    camera_1 = VirtualCamera(
        scene=simulation.scene,
        img_types=['Depth'], # 'Depth', 'Segmentation', 'Color'
    )
    camera_1.run()

    real_fps = 0
    t0 = time.time()
    t_proc = 0
    while not simulation.viewer.closed:
        simulation.step()

        target_pose = [0.025, 0.5]
        robot.move_to(target_pose, speed_factor=0.5)

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
