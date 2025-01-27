import pybullet as p
import pybullet_data

class PyBulletManager:
    def __init__(self, debug=False):
        """
        Initialise la connexion avec PyBullet et configure les paramètres de base.
        
        Args:
            debug (bool): Active ou désactive les visualisations de débogage.
        """
        self.physics_client = p.connect(p.GUI)
        assert self.physics_client >= 0, "Erreur : Impossible de se connecter à PyBullet."
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, debug)
        self.debug = debug

    def reset_camera(self, distance=2.0, yaw=0, pitch=-45, target=[0, 0, 0]):
        """
        Configure la caméra pour une vue spécifique.
        
        Args:
            distance (float): Distance de la caméra.
            yaw (float): Rotation autour de l'axe Y.
            pitch (float): Inclinaison de la caméra.
            target (list): Position cible de la caméra.
        """
        p.resetDebugVisualizerCamera(distance, yaw, pitch, target)

    def load_urdf(self, file, position, orientation, scaling=1.0):
        """
        Charge un fichier URDF dans la simulation.
        
        Args:
            file (str): Chemin vers le fichier URDF.
            position (list): Position initiale de l'objet.
            orientation (list): Orientation quaternion de l'objet.
            scaling (float): Facteur d'échelle.
        
        Returns:
            int: Identifiant de l'objet chargé.
        """
        return p.loadURDF(file, position, orientation, globalScaling=scaling)

    def reset_base_velocity(self, body_id, linear_velocity=[0, 0, 0], angular_velocity=[0, 0, 0]):
        """
        Met à jour la vitesse linéaire et angulaire d'un corps.
        
        Args:
            body_id (int): Identifiant du corps.
            linear_velocity (list): Vitesse linéaire [x, y, z].
            angular_velocity (list): Vitesse angulaire [x, y, z].
        """
        p.resetBaseVelocity(body_id, linearVelocity=linear_velocity, angularVelocity=angular_velocity)

    def get_contact_points(self, obj1_id, obj2_id):
        """
        Retourne les points de contact entre deux objets.
        
        Args:
            obj1_id (int): Identifiant du premier objet.
            obj2_id (int): Identifiant du second objet.
        
        Returns:
            list: Liste des points de contact.
        """
        return p.getContactPoints(obj1_id, obj2_id)

    def step_simulation(self):
        """
        Fait avancer la simulation d'une étape.
        """
        p.stepSimulation()

    def disconnect(self):
        """
        Déconnecte PyBullet.
        """
        p.disconnect()

    def get_base_position_and_orientation(self, body_id):
        """
        Retourne la position et l'orientation actuelles d'un corps.
        
        Args:
            body_id (int): Identifiant du corps.
        
        Returns:
            tuple: Position (x, y, z) et orientation quaternion (x, y, z, w).
        """
        return p.getBasePositionAndOrientation(body_id)

    def set_real_time_simulation(self, enable=True):
        """
        Active ou désactive la simulation en temps réel.
        
        Args:
            enable (bool): Si True, active la simulation en temps réel.
        """
        p.setRealTimeSimulation(1 if enable else 0)
