import pybullet as p

class SimulationObject:
    def __init__(self, urdf_path, start_pos, start_orientation, scaling=1.0):
        """
        Initialise l'objet de simulation en chargeant son modèle URDF.

        Args:
            urdf_path (str): Chemin vers le fichier URDF de l'objet.
            start_pos (list): Position initiale de l'objet [x, y, z].
            start_orientation (list): Orientation initiale de l'objet en quaternion [x, y, z, w].
            scaling (float): Facteur d'échelle pour le modèle URDF.
        """
        self.object_id = p.loadURDF(urdf_path, start_pos, start_orientation, globalScaling=scaling)
        self.start_pos = start_pos
        self.start_orientation = start_orientation

    def get_position_and_orientation(self):
        """
        Retourne la position et l'orientation actuelles de l'objet.

        Returns:
            tuple: Position (x, y, z) et orientation quaternion (x, y, z, w).
        """
        return p.getBasePositionAndOrientation(self.object_id)

    def set_position_and_orientation(self, position, orientation):
        """
        Définit la position et l'orientation de l'objet.

        Args:
            position (list): Nouvelle position [x, y, z].
            orientation (list): Nouvelle orientation en quaternion [x, y, z, w].
        """
        p.resetBasePositionAndOrientation(self.object_id, position, orientation)

    def set_velocity(self, linear_velocity=[0, 0, 0], angular_velocity=[0, 0, 0]):
        """
        Définit la vitesse linéaire et angulaire de l'objet.

        Args:
            linear_velocity (list): Vitesse linéaire [x, y, z].
            angular_velocity (list): Vitesse angulaire [x, y, z].
        """
        p.resetBaseVelocity(self.object_id, linearVelocity=linear_velocity, angularVelocity=angular_velocity)

    def get_contact_points(self, other_object_id):
        """
        Retourne les points de contact entre cet objet et un autre objet.

        Args:
            other_object_id (int): Identifiant de l'autre objet.

        Returns:
            list: Liste des points de contact.
        """
        return p.getContactPoints(self.object_id, other_object_id)