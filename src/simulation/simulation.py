import numpy as np
import sapien

import config

class NoViewer:
        closed = False
        def render(self):
            pass

class Simulation():
    def __init__(self, timestep=1e-2, with_viewer=True):
        self.scene = sapien.Scene()
        self.scene.set_timestep(timestep)
        self.actors = []

        if with_viewer:
            self.viewer = self.scene.create_viewer()
            self.viewer.set_camera_xyz(x=0, y=0, z=3.2)
            self.viewer.set_camera_rpy(r=0, p=-np.pi / 2, y=0)
            self.viewer.window.set_camera_parameters(near=0.05, far=100, fovy=1)
        else:
            self.viewer = NoViewer()

    def add_ground(self, altitude=0):
        self.scene.add_ground(altitude=altitude)

    def add_actor(self, actor):
        self.actors.append(actor)

    def add_table(self, position=[0,0,0.025], orientation=[0,0,0.707,0.707]):
        table_builder = self.scene.create_actor_builder()
        table_builder.add_convex_collision_from_file(filename="src/urdf_models/table.obj")
        table_material = sapien.render.RenderMaterial()
        table_material.base_color_texture = sapien.render.RenderTexture2D(filename="src/urdf_models/Vinyle_2025_FINAL.jpg")
        table_builder.add_visual_from_file(filename="src/urdf_models/table.obj", material=table_material)
        table = table_builder.build(name="table")
        table.set_pose(sapien.Pose(p=position, q=orientation))
        self.add_actor(table)

    def add_cans(self):
        can_builder = self.scene.create_actor_builder()
        can_builder.add_cylinder_collision(radius=config.CAN_RADIUS, half_length=config.CAN_HEIGHT / 2)
        can_mt = sapien.render.RenderMaterial()
        can_mt.base_color_texture = sapien.render.RenderTexture2D(filename="src/urdf_models/conserve.png")
        can_mt.base_color = [0.5, 0.5, 0.5, 1] # Blue color
        can_builder.add_cylinder_visual(material=can_mt, radius=config.CAN_RADIUS, half_length=config.CAN_HEIGHT / 2)
        for i, pos in enumerate(config.CAN_POSITIONS):
            can = can_builder.build(name=f"can_{i}")
            can.set_pose(sapien.Pose(p=pos, q=[0, 0.707, 0, 0.707]))

    def add_lights(self):
        self.scene.set_ambient_light([0.5, 0.5, 0.5])
        self.scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])

    def step(self):
        self.scene.step()
        self.scene.update_render()
        self.viewer.render()
