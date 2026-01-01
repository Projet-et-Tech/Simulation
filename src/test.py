from config import *
from simulation.simulation import Simulation
from simulation.robot import Robot
from simulation.camera import (
    VirtualCamera,
    display_images,
    normalize_image
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
    simulation.step()

    # Initialize robot
    start_pose = [1.25, -0.75, 0.1]
    robot = Robot(scene=simulation.scene, position=start_pose)

    if show_camera:
        cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
        camera_1 = VirtualCamera(
            scene=simulation.scene,
            img_types=['Color'], # 'Depth', 'Segmentation', 'Color'
        )
        camera_1.run()

        # Camera calibration
        cv2.namedWindow("Calibration", cv2.WINDOW_NORMAL)
        for _ in range(10):
            simulation.step()
            processed_images, t_proc = camera_1.image_queue.get()
        img_type = list(processed_images.keys())[0]
        img = processed_images[img_type]
        # Normalize and convert
        img = normalize_image(img).astype("uint8")
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGRA)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco_detector.detectMarkers(gray)

        from config import ARUCO_CORNERS

        threedpoints = []
        twodpoints = []
        for i, c in zip(ids, corners):
            if i not in [20, 21, 22, 23]:
                continue

            real_3d = ARUCO_CORNERS[int(i[0])]
            sim_2d = c[0]

            print(i)
            print(real_3d, real_3d.shape)
            print(sim_2d, sim_2d.shape)
            print()

            threedpoints.append(real_3d)
            twodpoints.append(sim_2d)

            cv2.circle(img, c[0, 0].astype(int), 4, (255, 0, 0), -1)  # Blue circle
            cv2.circle(img, c[0, 1].astype(int), 4, (0, 255, 0), -1)  # Green circle
            cv2.circle(img, c[0, 2].astype(int), 4, (0, 0, 255), -1)  # Red circle
            cv2.circle(img, c[0, 3].astype(int), 4, (255, 255, 0), -1)  # Cyan circle

        # Camera calibration
        threedpoints = np.vstack(threedpoints).astype(np.float32)
        twodpoints = np.vstack(twodpoints).astype(np.float32)

        print(threedpoints.shape) #Prints "(16, 3)"
        print(twodpoints.shape) # Prints "(16, 2)"

        ret, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera(
            objectPoints=[threedpoints],
            imagePoints=[twodpoints],
            imageSize=img[0].shape[::-1],
            cameraMatrix=None,
            distCoeffs=None
        )
        
        cv2.imshow("Calibration", img)


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
                reached, diff = robot.move_to(args, speed_factor=1)
            elif cmd == "ROTATETO":
                reached, diff = robot.rotate_to(args, angular_speed_factor=5)
            
            if reached:
                print(f"Reached target {args} (diff={diff:.4f})", 10*" ")
                current_inst += 1

        # Check for captured images (non-blocking) when camera is enabled
        if show_camera:
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
    main(
        show_camera=True,
        show_render=False
    )
