import numpy as np
import sapien

class Robot:
    """Class to load and manage a robot in the SAPIEN simulation environment."""
    def __init__(self,
            scene, 
            urdf_path="src/urdf_models/red_cube_v2.urdf", 
            position=[0.025, 0, 0.3], 
            orientation=[0, 0, 0, 1]
            ):
        loader = scene.create_urdf_loader()
        loader.fix_root_link = False
        self.robot = loader.load(urdf_path)
        self.robot.set_name("robot")
        self.robot.set_pose(sapien.Pose(p=position, q=orientation))

    def move_to(self, target_pose, speed_factor=0.8):
        """Move the robot's root link to a specified pose."""
        current_pose = self.get_pose()
        target_pose = np.array(target_pose)
        # Permute target x and y
        target_pose = np.array([target_pose[0], target_pose[1], current_pose.p[2]])

        distance_vector = target_pose - current_pose.p
        distance_norm = np.linalg.norm(distance_vector)
        if distance_norm > 1e-2:
            direction = distance_vector / distance_norm 
            self.set_root_linear_velocity(direction * speed_factor)
            return False, distance_norm
        else:
            self.set_root_linear_velocity([0, 0, 0])
            return True, distance_norm

    def get_pose(self):
        """Get the current pose of the robot's root link."""
        return self.robot.get_pose()

    def set_root_linear_velocity(self, velocity):
        """Set the linear velocity of the robot's root link."""
        self.robot.set_root_linear_velocity(velocity)

    def set_root_angular_velocity(self, angular_velocity):
        """Set the angular velocity of the robot's root link."""
        self.robot.set_root_angular_velocity(angular_velocity)

    def get_root_linear_velocity(self):
        """Get the linear velocity of the robot's root link."""
        return self.robot.get_root_linear_velocity()

    def get_root_angular_velocity(self):
        """Get the angular velocity of the robot's root link."""
        return self.robot.get_root_angular_velocity()