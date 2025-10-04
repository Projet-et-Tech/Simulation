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
    
    # actor_builder = scene.create_actor_builder()
    # actor_builder.add_box_collision(half_size=[0.5, 0.5, 0.5])
    # actor_builder.add_box_visual(half_size=[0.5, 0.5, 0.5], material=[1.0, 0.0, 0.0])
    # box = actor_builder.build(name="box")  # Add a box
    # box.set_pose(sapien.Pose(p=[0, 0, 0.5]))

    # loader = scene.create_urdf_loader()
    # table = loader.load("src/urdf_models/table_eurobot2025.urdf")
    # table.set_root_pose(sapien.Pose([0, 0, 0], [1, 0, 0, 0]))

    builder = scene.create_actor_builder()
    builder.add_convex_collision_from_file(
        filename="src/urdf_models/table.obj"
    )
    # builder.add_visual_from_file(filename="src/urdf_models/table.mtl")

    mt = sapien.render.RenderMaterial()
    mt.diffuse_texture = sapien.render.RenderTexture2D(filename="src/urdf_models/Vinyle_2025_FINAL.jpg")
    builder.add_visual_from_file(filename="src/urdf_models/table.obj", material=mt)

    mesh = builder.build(name="mesh")
    mesh.set_pose(sapien.Pose(p=[0, 0, 0], q=[0.707, 0.707, 0, 0]))

    # q is quaternion angle.
    # [1, 0, 0, 0] means no rotation
    # [0.707, 0, 0.707, 0] means 90 degree rotation around x axis
    # [0.707, 0.707, 0, 0] means 90 degree rotation around y axis
    # [0.707, 0, 0, 0.707] means 90 degree rotation around z axis

    # Add some lights so that you can observe the scene
    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])

    viewer = scene.create_viewer()  # Create a viewer (window)

    # The coordinate frame in Sapien is: x(forward), y(left), z(upward)
    # The principle axis of the camera is the x-axis
    viewer.set_camera_xyz(x=-4, y=0, z=2)
    # The rotation of the free camera is represented as [roll(x), pitch(-y), yaw(-z)]
    # The camera now looks at the origin
    viewer.set_camera_rpy(r=0, p=-np.arctan2(2, 4), y=0)
    viewer.window.set_camera_parameters(near=0.05, far=100, fovy=1)

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
    while not viewer.closed:  # Press key q to quit
        scene.step()  # Simulate the world
        scene.update_render()  # Update the world to the renderer

        t0 = time.time()
        camera.take_picture()
        print("Camera latency:", time.time() - t0)

        t1 = time.time()
        rgba = camera.get_picture("Color")  # [H, W, 4]
        print("Read camera data latency:", time.time() - t1)

        print()

        rgba_img = (rgba * 255).astype(np.uint8)
        cv2.imshow("Camera", rgba_img)
        cv2.waitKey(fps * 1000)

        viewer.render()


if __name__ == "__main__":
    main()