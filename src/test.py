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
    display_images
)
from simulation.instructions import parse_instruction_file

def main(fps=1000, show_camera=False, show_render=True, instruction_file="src/routine.txt"):
    """Run the simulation.

    Args:
        fps: main-loop target frames per second for the UI waitKey.
        show_camera: if True, start the virtual camera and display the camera window.
        show_render: if True, create and show the SAPIEN viewer (render window).
    """
    # Initialize SAPIEN engine. Viewer presence depends on show_render.
    simulation = Simulation(with_viewer=show_render)
    simulation.add_ground()
    simulation.add_lights()
    simulation.add_table()
    simulation.add_boxes()

    # Initialize robot
    start_pose = [1.25, -0.75, 0.1]
    robot = Robot(scene=simulation.scene, position=start_pose)

    camera_1 = None
    if show_camera:
        cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
        camera_1 = VirtualCamera(
            scene=simulation.scene,
            img_types=['Depth'], # 'Depth', 'Segmentation', 'Color'
        )
        camera_1.run()

    # Load instructions
    instructions = parse_instruction_file(instruction_file) if instruction_file else []
    current_inst = 0

    real_fps = 0
    t0 = time.time()
    t_proc = 0
    while not simulation.viewer.closed:
        simulation.step()

        # Execute instructions sequentially
        if current_inst < len(instructions):
            cmd, args = instructions[current_inst]
            if cmd == 'MOVETO':
                reached, dist = robot.move_to(args, speed_factor=0.5)
                if reached:
                    print(f"Reached target {args} (dist={dist:.4f})")
                    current_inst += 1
            else:
                print(f"Unknown command: {cmd}")
                current_inst += 1
        else:
            # No instruction: keep robot stopped
            robot.move_to([robot.get_pose().p[0], robot.get_pose().p[1]], speed_factor=0.0)

        # Check for captured images (non-blocking) when camera is enabled
        if show_camera and camera_1 is not None:
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
            if show_camera:
                # If camera window is visible, allow camera window close to exit
                if cv2.getWindowProperty("Camera", cv2.WND_PROP_VISIBLE) < 1:
                    break
            # Use waitKey for both camera and render UI responsiveness
            key = cv2.waitKey(1000 // fps) & 0xFF
            if key == 27:  # ESC key
                break
        except Exception:
            break
    
    # Cleanup
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
