import time
import sapien
from sapien.utils import Viewer
import numpy as np

# pip install opencv-python
import cv2

def main(fps=20):
    scene = sapien.Scene()  # Create an instance of simulation world (aka scene)
    scene.set_timestep(1 / 100.0)  # Set the simulation frequency

    # NOTE: How to build (rigid bodies) is elaborated in create_actors.py
    scene.add_ground(altitude=0)  # Add a ground
    


    table_builder = scene.create_actor_builder()
    table_builder.add_convex_collision_from_file(
        filename="src/urdf_models/table.obj"
    )
    table_material = sapien.render.RenderMaterial()
    table_material.base_color_texture = sapien.render.RenderTexture2D(filename="src/urdf_models/Vinyle_2025_FINAL.jpg")
    table_builder.add_visual_from_file(filename="src/urdf_models/table.obj", material=table_material)
    mesh = table_builder.build(name="mesh")
    mesh.set_pose(sapien.Pose(p=[0, 0, 0], q=[-0.5, -0.5, 0.5, 0.5]))

    robot_builder = scene.create_actor_builder()
    half_size=[0.32/2, 0.32/2, 0.35/2]
    robot_builder.add_box_collision(half_size=half_size)
    robot_mt = sapien.render.RenderMaterial()
    robot_mt.base_color = [0.8, 0.1, 0.1, 1] # Red color
    robot_builder.add_box_visual(half_size=half_size, material=robot_mt)
    robot = robot_builder.build(name="box")
    robot.set_pose(sapien.Pose(p=[0, 0, 0.2]))

    # Add some lights so that you can observe the scene
    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])

    if 1:
        viewer = scene.create_viewer()  # Create a viewer (window)
        # The coordinate frame in Sapien is: x(forward), y(left), z(upward)
        # The principle axis of the camera is the x-axis
        viewer.set_camera_xyz(x=-4, y=0, z=2)
        # The rotation of the free camera is represented as [roll(x), pitch(-y), yaw(-z)]
        # The camera now looks at the origin
        viewer.set_camera_rpy(r=0, p=-np.arctan2(2, 4), y=0)
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

    cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)

    ##########################################################
    while not viewer.closed:
        scene.step()  # Simulate the world
        scene.update_render()  # Update the world to the renderer
        viewer.render()

        t0 = time.time()
        camera.take_picture()

        t1 = time.time()
        rgba = camera.get_picture("Color")  # [H, W, 4]
        print("Camera latency:", time.time() - t0, "Read camera data latency:", time.time() - t1, end="\r")

        rgba_img = (rgba * 255).astype(np.uint8)
        rgba_img = cv2.cvtColor(rgba_img, cv2.COLOR_RGBA2BGRA)
        cv2.imshow("Camera", rgba_img)
        key = cv2.waitKey(1000 // fps)

        try:
            if cv2.getWindowProperty("Camera", cv2.WND_PROP_VISIBLE) < 1:
                break
        except Exception:
            break
        if key == 27:
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()