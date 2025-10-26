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
        self.actors = {}

        if with_viewer:
            self.viewer = self.scene.create_viewer()
            self.viewer.set_camera_xyz(x=0, y=0, z=3.2)
            self.viewer.set_camera_rpy(r=0, p=-np.pi / 2, y=0)
            self.viewer.window.set_camera_parameters(near=0.05, far=100, fovy=1)
        else:
            self.viewer = NoViewer()

    def add_ground(self, altitude=0):
        self.scene.add_ground(altitude=altitude)

    def add_actor(self, actor_type, actor):
        if actor_type in self.actors:
            self.actors[actor_type].append(actor)
        else:
            self.actors[actor_type] = [actor]

    def add_table(self, position=[0,0,0.025], orientation=[0,0,0.707,0.707]):
        table_builder = self.scene.create_actor_builder()
        table_builder.add_convex_collision_from_file(filename="src/urdf_models/table.obj")
        table_material = sapien.render.RenderMaterial()
        table_material.base_color_texture = sapien.render.RenderTexture2D(filename="src/urdf_models/Vinyle_2026_FINAL.png")
        table_builder.add_visual_from_file(filename="src/urdf_models/table.obj", material=table_material)
        table = table_builder.build(name="table")
        table.set_pose(sapien.Pose(p=position, q=orientation))
        self.add_actor('table', table)

    def add_boxes(self):
        box_builder = self.scene.create_actor_builder()
        box_builder.add_box_collision(half_size=[config.BOX_HEIGHT/2, config.BOX_LENGTH/2, config.BOX_WIDTH/2])

        for i, pos in enumerate(config.BOX_POSITIONS_HORIZONTAL):
            box_mt = sapien.render.RenderMaterial()
            choice = self.getRandomBoxColor(i)
            box_mt.base_color = [0, 91/256, 140/256, 1] if choice == 0 else [247/256, 181/256, 0, 1]
            box_builder.add_box_visual(material=box_mt, half_size=[config.BOX_HEIGHT/2, config.BOX_LENGTH/2, config.BOX_WIDTH/2])
            box = box_builder.build(name=f"box_{i}")
            box.set_pose(sapien.Pose(p=pos, q=[-0.5, 0.5, 0.5, 0.5]))
            self.add_actor('box', box)

        for i, pos in enumerate(config.BOX_POSITIONS_VERTICAL):
            box_mt = sapien.render.RenderMaterial()
            choice = self.getRandomBoxColor(i)
            box_mt.base_color = [0, 91/256, 140/256, 1] if choice == 0 else [247/256, 181/256, 0, 1]
            box_builder.add_box_visual(material=box_mt, half_size=[config.BOX_HEIGHT/2, config.BOX_LENGTH/2, config.BOX_WIDTH/2])
            box = box_builder.build(name=f"box_{i}")
            box.set_pose(sapien.Pose(p=pos, q=[0.707, 0, 0.707, 0]))
            self.add_actor('box', box)

    def getRandomBoxColor(self, i):
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
