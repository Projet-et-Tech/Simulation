from utils.math_helpers import interpolate_position
import math

class Robot:
    def __init__(self, pybullet_manager, urdf_path, start_position):
        self.p = pybullet_manager
        self.robot_id = self.p.load_urdf(urdf_path, start_position, [0, 0, 0, 1])
        self.speed = 0.4

    def move_to(self, path, interpolation_steps=20):
        for step in path:
            next_pos = [step[0], step[1]]
            robot_pos, _ = self.p.getBasePositionAndOrientation(self.robot_id)
            interpolated_positions = interpolate_position(robot_pos[:2], next_pos, interpolation_steps)
            for inter_pos in interpolated_positions:
                self._move_to_position(inter_pos)

    def _move_to_position(self, position):
        delta_x, delta_y = position[0], position[1]
        direction_angle = math.atan2(delta_y, delta_x)
        linear_velocity = [
            self.speed * math.cos(direction_angle),
            self.speed * math.sin(direction_angle),
            0
        ]
        self.p.resetBaseVelocity(self.robot_id, linearVelocity=linear_velocity)
