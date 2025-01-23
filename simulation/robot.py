import pybullet as p

class Robot:
    def __init__(self, urdf_path, start_pos, start_orientation, scaling=1.0):
        """
        Initialise le robot en chargeant son modèle URDF.

        Args:
            urdf_path (str): Chemin vers le fichier URDF du robot.
            start_pos (list): Position initiale du robot [x, y, z].
            start_orientation (list): Orientation initiale du robot en quaternion [x, y, z, w].
            scaling (float): Facteur d'échelle pour le modèle URDF.
        """
        self.robot_id = p.loadURDF(urdf_path, start_pos, start_orientation, globalScaling=scaling)
        self.start_pos = start_pos
        self.start_orientation = start_orientation

    def get_position_and_orientation(self):
        """
        Retourne la position et l'orientation actuelles du robot.

        Returns:
            tuple: Position (x, y, z) et orientation quaternion (x, y, z, w).
        """
        return p.getBasePositionAndOrientation(self.robot_id)

    def set_position_and_orientation(self, position, orientation):
        """
        Définit la position et l'orientation du robot.

        Args:
            position (list): Nouvelle position [x, y, z].
            orientation (list): Nouvelle orientation en quaternion [x, y, z, w].
        """
        p.resetBasePositionAndOrientation(self.robot_id, position, orientation)

    def set_velocity(self, linear_velocity=[0, 0, 0], angular_velocity=[0, 0, 0]):
        """
        Définit la vitesse linéaire et angulaire du robot.

        Args:
            linear_velocity (list): Vitesse linéaire [x, y, z].
            angular_velocity (list): Vitesse angulaire [x, y, z].
        """
        p.resetBaseVelocity(self.robot_id, linearVelocity=linear_velocity, angularVelocity=angular_velocity)

    def apply_action(self, action):
        """
        Applique une action au robot (par exemple, définir des vitesses pour les moteurs).

        Args:
            action (dict): Dictionnaire contenant les actions à appliquer.
        """
        for joint, value in action.items():
            p.setJointMotorControl2(self.robot_id, joint, p.VELOCITY_CONTROL, targetVelocity=value)