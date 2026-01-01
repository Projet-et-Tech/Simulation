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

    def move_to(self, target_pose, speed_factor=1):
        """Move the robot's root link to a specified pose."""
        current_pose = self.get_pose()
        target_pose = np.array(target_pose)
        # Permute target x and y
        target_pose = np.array([target_pose[0], target_pose[1], current_pose.p[2]])

        distance_vector = target_pose - current_pose.p
        distance_norm = np.linalg.norm(distance_vector)

        if distance_norm <= 1e-2:
            self.set_root_linear_velocity([0, 0, 0])
            return True, distance_norm
        
        direction = distance_vector / distance_norm 
        self.set_root_linear_velocity(direction * speed_factor)
        return False, distance_norm

    def rotate_to(self, angle, axis='z', angular_speed_factor=1):
        """ Rotate the robot by a specified angle around a chosen axis. """
        angle_rad = np.deg2rad(angle)
        current_pose = self.get_pose()
        current_orientation = current_pose.q

        axis_map = {
            'x': [1, 0, 0],
            'y': [0, 1, 0],
            'z': [0, 0, 1]
        }
        
        if axis not in axis_map:
            raise ValueError(f"Invalid axis. Choose from {list(axis_map.keys())}")
        
        rotation_axis = np.array(axis_map[axis])
        
        # Calculate current rotation around the specified axis
        # Convert quaternion to rotation matrix
        R = np.array(sapien.Pose(q=current_orientation).to_transformation_matrix()[:3, :3])
        
        # Extract current angle of rotation around the specified axis
        # This is a simplified approach and might need refinement depending on exact requirements
        if axis == 'x':
            current_angle = np.arctan2(R[2, 1], R[2, 2])
        elif axis == 'y':
            current_angle = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
        else:  # z-axis
            current_angle = np.arctan2(R[1, 0], R[0, 0])
        
        # Calculate rotation difference
        rotation_difference = np.linalg.norm(angle_rad - current_angle)
        
        # Check if rotation is close enough (within a small threshold)
        rotation_threshold = np.deg2rad(1)  # 1-degree threshold
        if abs(rotation_difference) <= rotation_threshold:
            self.set_root_angular_velocity([0, 0, 0])
            return True, rotation_difference
        
        # Determine rotation direction and set angular velocity
        # Use the sign of rotation difference to determine direction
        angular_velocity = rotation_axis * angular_speed_factor * np.sign(rotation_difference)
        
        self.set_root_angular_velocity(angular_velocity)
        
        return False, rotation_difference



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