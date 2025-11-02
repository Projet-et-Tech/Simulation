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

    def add_table(self, position=[0,0,0.025], orientation=[0, 0, 0, 1]):
        loader = self.scene.create_urdf_loader()
        table_collision = loader.load("src/urdf_models/table_2026.urdf")
        table_collision.set_pose(sapien.Pose(p=position, q=orientation))
        
        # table_builder = self.scene.create_actor_builder()
        # table_builder.add_nonconvex_collision_from_file(filename="src/urdf_models/table.obj")
        # table_material = sapien.render.RenderMaterial()
        # table_material.base_color_texture = sapien.render.RenderTexture2D(filename="src/urdf_models/Vinyle_2026_FINAL.png")
        # table_builder.add_visual_from_file(filename="src/urdf_models/table.obj", material=table_material)
        # table = table_builder.build(name="table")
        # table.set_pose(sapien.Pose(p=position, q=orientation))
        # self.add_actor(table)

    def add_boxes(self):
        box_builder = self.scene.create_actor_builder()
        box_builder.add_convex_collision_from_file(filename="src/urdf_models/caisse_couleur.obj")
        box_material = sapien.render.RenderMaterial()
        box_material.base_color_texture = sapien.render.RenderTexture2D(filename="src/urdf_models/game_elements_2026_FINAL.png")
        box_builder.add_visual_from_file(filename="src/urdf_models/caisse_couleur.obj", material=box_material)
        for i, pos in enumerate(config.BOX_POSITIONS_HORIZONTAL):
            box = box_builder.build(name=f"box_{i}")
            choice = self.get_randomBoxColor(i)
            rotation = [0, 0, 0.707, 0.707] if choice == 0 else [0, 0, -0.707, 0.707]
            box.set_pose(sapien.Pose(p=pos, q=rotation))
        for i, pos in enumerate(config.BOX_POSITIONS_VERTICAL):
            box = box_builder.build(name=f"box_{i}")
            choice = self.get_randomBoxColor(i)
            rotation = [0.5, 0.5, 0.5, 0.5] if choice == 0 else [-0.5, 0.5, -0.5, 0.5]
            box.set_pose(sapien.Pose(p=pos, q=rotation))

    def get_randomBoxColor(self, i):
        if not hasattr(self, "_box_color_permutations"):
            self._box_color_permutations = {}
        block = i // 4
        idx_in_block = i % 4
        if block not in self._box_color_permutations:
            perm = [0, 0, 1, 1]
            rng = np.random.RandomState()
            rng.shuffle(perm)
            self._box_color_permutations[block] = perm
        return self._box_color_permutations[block][idx_in_block]

    def add_lights(self):
        self.scene.set_ambient_light([0.5, 0.5, 0.5])
        self.scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])

    def step(self):
        self.scene.step()
        self.scene.update_render()
        self.viewer.render()
