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
    print("Run 'make environment' or 'make install' to get the required modules")

import config

def normalize_image(img, min_val=0, max_val=255):
    """Flexible image normalization with custom range."""
    if img.size == 0:
        return np.zeros_like(img)
    
    img_min = img.min()
    img_max = img.max()
    
    if img_max > img_min:
        normalized_img = (
            (img - img_min) / (img_max - img_min) * 
            (max_val - min_val) + min_val
        )
        return normalized_img.astype(img.dtype)
    else:
        return np.full_like(img, min_val)


def process_images_core(rgba_camera, position, seg_labels):
    # Use a fixed-size tuple instead of a dictionary
    camera_color = np.clip(rgba_camera * 255, 0, 255).astype(np.uint8)
    depth = (-position[..., 2] * 1000.0).astype(np.uint16)
    segmentation = seg_labels[..., 0].astype(np.uint32)
    
    return (camera_color, depth, segmentation)

def optimized_image_capture(camera):
    camera.take_picture()
    rgba_camera = camera.get_picture("Color")
    camera.take_picture()
    position = camera.get_picture("Position")
    camera.take_picture()
    seg_labels = camera.get_picture("Segmentation")
    
    processed = process_images_core(rgba_camera, position, seg_labels)
    
    processed_dict = {
        'camera_color': processed[0],
        'depth': processed[1],
        'segmentation': processed[2]
    }
    
    return processed_dict


def display_images(processed_images):
    # Choose which image to display
    img = processed_images['camera_color']
    
    # Normalize and convert
    img = normalize_image(img).astype("uint8")
    img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGRA)
    
    cv2.imshow("Camera", img)
    key = cv2.waitKey(1)
    
    return key

def image_capture_thread(camera, image_queue):
    """Thread function for image capture"""
    while True:
        try:
            # Capture images
            t0 = time.time()
            processed_images = optimized_image_capture(camera)
            
            # Put images in queue (non-blocking)
            if not image_queue.full():
                image_queue.put((processed_images, time.time() - t0))
        
        except Exception as e:
            print(f"Capture thread error: {e}")
            break


def main(fps=1000):
    scene = sapien.Scene()
    scene.set_timestep(1 / 100.0)

    scene.add_ground(altitude=0)

    table_builder = scene.create_actor_builder()
    table_builder.add_convex_collision_from_file(filename="src/urdf_models/table.obj")
    table_material = sapien.render.RenderMaterial()
    table_material.base_color_texture = sapien.render.RenderTexture2D(filename="src/urdf_models/Vinyle_2025_FINAL.jpg")
    table_builder.add_visual_from_file(filename="src/urdf_models/table.obj", material=table_material)
    table = table_builder.build(name="table")
    table.set_pose(sapien.Pose(p=[0, 0, 0.025], q=[0, 0, 0.707, 0.707]))

    # robot_builder = scene.create_actor_builder()
    # half_size=[0.32/2, 0.32/2, 0.35/2]
    # robot_builder.add_box_collision(half_size=half_size)
    # robot_mt = sapien.render.RenderMaterial()
    # robot_mt.base_color = [0.8, 0.1, 0.1, 1] # Red color
    # robot_builder.add_box_visual(material=robot_mt, half_size=half_size)
    # robot = robot_builder.build_kinematic(name="robot")
    # robot.set_pose(sapien.Pose(p=[0, 0, 0.2]))

    loader = scene.create_urdf_loader()
    loader.fix_root_link = False
    robot = loader.load("src/urdf_models/red_cube_v2.urdf")
    robot.set_name("robot")

    angle = quaternion.from_euler_angles([0, 0, np.pi / 2])
    robot.set_pose(sapien.Pose(p=[0.025, 0, 0.3], q=quaternion.as_float_array(angle)))

    can_builder = scene.create_actor_builder()
    can_builder.add_cylinder_collision(radius=config.CAN_RADIUS, half_length=config.CAN_HEIGHT / 2)
    can_mt = sapien.render.RenderMaterial()
    # can_mt.base_color_texture = sapien.render.RenderTexture2D(filename="src/urdf_models/conserve.png")
    can_mt.base_color = [0.5, 0.5, 0.5, 1] # Blue color
    can_builder.add_cylinder_visual(material=can_mt, radius=config.CAN_RADIUS, half_length=config.CAN_HEIGHT / 2)
    for i, pos in enumerate(config.CAN_POSITIONS):
        can = can_builder.build(name=f"can_{i}")
        can.set_pose(sapien.Pose(p=pos, q=[0, 0.707, 0, 0.707]))

    # Add some lights so that you can observe the scene
    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])

    if 0:
        viewer = scene.create_viewer()
        viewer.set_camera_xyz(x=0, y=0, z=3.2)
        viewer.set_camera_rpy(r=0, p=-np.pi / 2, y=0)
        viewer.window.set_camera_parameters(near=0.05, far=100, fovy=1)
    else:
        class NoViewer:
            closed = False
            def render(self):
                pass
        viewer = NoViewer()

    ############################ Camera
    near, far = 0.1, 100
    # width, height = 640, 480
    width, height = 1920, 1080

    # Compute the camera pose by specifying forward(x), left(y) and up(z)
    cam_pos = np.array([-4, 0, 3])
    forward = -cam_pos / np.linalg.norm(cam_pos)
    left = np.cross([0, 0, 1], forward)
    left = left / np.linalg.norm(left)
    up = np.cross(forward, left)
    mat44 = np.eye(4)
    mat44[:3, :3] = np.stack([forward, left, up], axis=1)
    mat44[:3, 3] = cam_pos

    camera = scene.add_camera(
        name="camera",
        width=width,
        height=height,
        fovy=np.deg2rad(35),
        near=near,
        far=far,
    )
    camera.entity.set_pose(sapien.Pose(mat44))

    image_queue = queue.Queue(maxsize=10)  # Limit queue size to prevent memory buildup
    
    # Start image capture thread
    capture_thread = threading.Thread(
        target=image_capture_thread, 
        args=(camera, image_queue),
        daemon=True  # Allows thread to exit when main program exits
    )
    capture_thread.start()


    cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)

    ##########################################################

    real_fps = 0
    t0 = time.time()
    t_proc = 0
    while not viewer.closed:
        scene.step()
        scene.update_render()
        viewer.render()

        current_pose = robot.get_pose()
        target_pose = sapien.Pose(p=[0.25, 0.5, current_pose.p[2]])

        velocity_vector = target_pose.p - current_pose.p
        velocity_norm = np.linalg.norm(velocity_vector)
        velocity = velocity_vector / velocity_norm
        robot.set_root_linear_velocity(velocity * 0.8)

        # Check for captured images (non-blocking)
        try:
            processed_images, t_proc = image_queue.get_nowait()
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