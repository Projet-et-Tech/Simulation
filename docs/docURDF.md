# Documentation : Fichiers URDF (ajout d'objets dans PyBullet)

## Qu'est-ce que le format URDF ?
Le format URDF (Unified Robot Description Format) est un format XML utilisé pour décrire les modèles de robots et d'objets dans des environnements de simulation comme PyBullet. Il permet de définir la géométrie, les propriétés physiques et les relations entre les différentes parties d'un robot ou d'un objet.


## Structure d'un fichier URDF
Un fichier URDF est organisé en plusieurs sections principales :
- **Robot** : La balise racine qui regroupe toutes les informations sur le robot ou l'objet.
- **Link** : Définit une partie rigide du robot ou de l'objet. Chaque lien possède des propriétés comme la masse, l'inertie, la géométrie (forme et taille), la couleur, la collision, etc.
- **Joint** : Définit la connexion entre deux liens, en précisant le type de joint (`fixe`, `revolute`, `prismatic`, etc.) et les limites de mouvement.
- **Visual** : Spécifie l'apparence visuelle du lien, incluant la géométrie et les matériaux.
- **Collision** : Définit la géométrie utilisée pour les calculs de collision.
- **Inertial** : Spécifie les propriétés physiques du lien, telles que la masse et le tenseur d'inertie.


## Exemple de fichier URDF simple - Planche
Voici un exemple de fichier URDF décrivant une planche simple :

```xml
<?xml version="1.0" ?>
<robot name="plank">
    <!-- planche rectangulaire -->
    <link name="base">
        <visual>
            <origin xyz="0 0 0.175" rpy="0 0 0"/>
            <geometry>
                <box size="0.4 0.1 0.015"/>  <!-- Dimensions : 0.4 m x 0.1 m x 0.015 m -->
            </geometry>
            <material name="brown">
                <color rgba="0.6 0.3 0.1 1"/>  <!-- Couleur marron pour la planche (valeurs entre 0 et 1) -->
            </material>
        </visual>
		
        <collision>
            <origin xyz="0 0 0.175" rpy="0 0 0"/>
            <geometry>
                <box size="0.4 0.1 0.015"/> <!-- Même dimensions pour la collision -->
            </geometry>
        </collision>
		
		<!-- Masse et Inertie -->
		<inertial>
            <mass value="1.0"/>  <!-- Masse à 1kg -->
            <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/> <!-- Inertie ajustée (calculs IA) -->
        </inertial>
    </link>
</robot>
```

Cette approche est suffisante pour créer des objets simples, sans textures.


## Exemple de fichier URDF avec texture et modèle 3D : Canette
Voici un exemple de fichier URDF décrivant une canette avec une texture :

```xml
<?xml version="1.0" ?>
<robot name="can">
    <!-- conserve -->
    <link name="base">

    <!-- Rendu visuel : modèle 3D -->
    <visual>
        <geometry>
            <mesh filename="conserve.obj"/>  <!-- chemin vers le fichier .obj -->
        </geometry>
        <material name="default_material">
            <color rgba="1 1 1 1"/>  <!-- Texture par défaut si le matériau lié n'est pas trouvé -->
        </material>
    </visual>
    
    <!-- Rendu colisionel : modèle 3D (en général le même) -->
    <collision>
        <geometry>
            <mesh filename="conserve.obj"/>  <!-- chemin vers le fichier .obj -->
        </geometry>
        </collision>
    
		<!-- Masse et Inertie -->
        <inertial>
		    <mass value="0.425"/> <!-- Masse à 425g -->
		    <inertia ixx="0.000505" ixy="0" ixz="0" iyy="0.000505" iyz="0" izz="0.000283"/> <!-- Inertie ajustée (calculs IA) -->
	    </inertial>
	
    </link>
</robot>
```


Bien que les cylindres puissent être définis directement dans URDF, l'utilisation de modèles 3D permet de mapper correctement les textures. Cette technique est particulièrement utile pour des objets complexes nécessitant une apparence réaliste.

Le format OBJ est très utilisé car il est simple, compatible avec de nombreux outils de modélisation 3D, et il gère les textures facilement.

Par exemple, pour créer un modèle 3D d'une conserve, on peut utiliser un logiciel comme Blender pour concevoir la forme du cylindre et appliquer une texture réaliste (étiquette fournie par le règlement de la Coupe de France). On exporte ensuite le modèle au format OBJ, un fichier de texture MTL est généré (il fait le lien entre modèle et texture), puis on l'intègre dans le fichier URDF comme montré ci-dessus. Ici, la texture n'est pas définie directement dans le fichier URDF, mais référencée via le fichier OBJ.



## Exemple de fichier URDF complexe - Table de jeu
Voici un exemple de fichier URDF décrivant la table de jeu avec plusieurs liens et joints :

```xml
<?xml version="1.0" ?>
<robot name="obj_model">

    <!-- Module Rendu Texturé (via .obj) -->
    <link name="rendered_table">
        
        <!-- Rendu visuel : modèle 3D -->
        <visual>
            <geometry>
                <mesh filename="table.obj"/>  <!-- chemin vers le fichier .obj -->
            </geometry>
            <material name="default_material">
                <color rgba="1 1 1 1"/>  <!-- You can set the default color if no texture is available -->
            </material>
        </visual>
        
        <!-- Masse et Inertie -->
        <inertial>
        <mass value="80.0"/>  <!-- Masse à 80kg -->
        <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.2"/> <!-- Inertie ajustée (calculs IA) -->
        </inertial>
    </link>
    
    <!-- Module Collisions (jointures de formes simples) -->
    <link name="table_base">
        <collision>
        <geometry>
            <box size="3.0 2.0 0.05"/>
        </geometry>
        </collision>
        <visual>
        <geometry>
            <box size="0 0 0"/> <!-- Nécessaire de désactiver le rendu (automatique par défaut) -->
        </geometry>
        </visual>
    </link>
    
    <link name="border_front">
        <collision>
        <geometry>
            <box size="3.0 0.022 0.07"/>
        </geometry>
        </collision>
        <visual>
        <geometry>
            <box size="0 0 0"/>
        </geometry>
        </visual>
    </link>
    
    <link name="border_back">
        <collision>
        <geometry>
            <box size="3.0 0.022 0.07"/>
        </geometry>
        </collision>
        <visual>
        <geometry>
            <box size="0 0 0"/>
        </geometry>
        </visual>
    </link>
    
    <link name="border_left">
        <collision>
        <geometry>
            <box size="0.022 2.0 0.07"/>
        </geometry>
        </collision>
        <visual>
        <geometry>
            <box size="0 0 0"/>
        </geometry>
        </visual>
    </link>
    
    <link name="border_right">
        <collision>
        <geometry>
            <box size="0.022 2.0 0.07"/>
        </geometry>
        </collision>
        <visual>
        <geometry>
            <box size="0 0 0"/>
        </geometry>
        </visual>
    </link>
    
    <link name="scene">
        <collision>
        <geometry>
            <box size="0.9 0.45 0.055"/>
        </geometry>
        </collision>
        <visual>
        <geometry>
            <box size="0 0 0"/>
        </geometry>
        </visual>
    </link>
    
    <link name="ramps">
        <visual>
            <geometry>
                <mesh filename="rampes.obj"/>  <!-- chemin vers le fichier .obj -->
            </geometry>
            <material name="default_material">
                <color rgba="1 1 1 1"/>
            </material>
        </visual>
        <collision>
        <geometry>
            <mesh filename="rampes.obj"/>
        </geometry>
        </collision>
    </link>
    
    <!-- Fixer les bordures à la table -->
    <joint name="rendered_to_base" type="fixed">
        <parent link="table_base"/>
        <child link="rendered_table"/>
        <origin rpy="1.5708 0 0" xyz="0 0 0"/> <!-- 1.5708 radians = 90° -->
    </joint>
    
    <joint name="base_to_border_front" type="fixed">
        <parent link="table_base"/>
        <child link="border_front"/>
        <origin rpy="0 0 0" xyz="0 1.011 0.06"/>
    </joint>

    <joint name="base_to_border_back" type="fixed">
        <parent link="table_base"/>
        <child link="border_back"/>
        <origin rpy="0 0 0" xyz="0 -1.011 0.06"/>
    </joint>

    <joint name="base_to_border_left" type="fixed">
        <parent link="table_base"/>
        <child link="border_left"/>
        <origin rpy="0 0 0" xyz="-1.511 0 0.06"/>
    </joint>

    <joint name="base_to_border_right" type="fixed">
        <parent link="table_base"/>
        <child link="border_right"/>
        <origin rpy="0 0 0" xyz="1.511 0 0.06"/>
    </joint>
    
    <joint name="base_to_scene" type="fixed">
        <parent link="table_base"/>
        <child link="scene"/>
        <origin rpy="0 0 0" xyz="0 0.775 0.05"/>
    </joint>
    
    <joint name="base_to_ramps" type="fixed">
        <parent link="table_base"/>
        <child link="ramps"/>
        <origin rpy="1.5708 0 0" xyz="0 0 0"/>
    </joint>

    <!-- Fixer la table au monde (immobilisation) -->
    <link name="world" />
    <joint name="world_to_base_link" type="fixed">
        <parent link="world" />
        <child link="table_base" />
    </joint>

</robot>
```

Ici on combine plusieurs techniques :
- Utilisation de modèles 3D pour le rendu visuel (table et rampes) avec textures.
- Utilisation de formes simples (boîtes) pour les collisions.
- Utilisation de joints fixes pour assembler les différentes parties de la table.
- Utilisation de un lien "world" pour fixer la table dans l'environnement de simulation.

L'import direct de la table complète aurait donné des collisions imprécises. Par défaut, les collisions sont calculées à partir d'une boîte englobante du modèle 3D, ce qui n'est pas adapté pour une table avec des bordures et une scène de jeu distincte.

Les textures des modèles 3D doivent être placées dans le même répertoire que les fichiers OBJ pour être correctement chargées par PyBullet. Elles doivent aussi être mappées (UV mapping) dans le logiciel de modélisation 3D avant l'export, afin de garantir qu'elles s'affichent correctement à l'endroit souhaité sur le modèle.

Pour la table de jeu, le mapping a déjà été réalisé dans le fichier `table.obj` fourni. Il suffit donc de placer la texture dans le même dossier que le fichier URDF, puis de changer le nom dans le fichier `table.mtl` pour que PyBullet l'applique automatiquement lors du chargement du modèle.

L'utilisation de jointures permet une grande modularité : on peut ainsi modifier ou remplacer des parties de la table (comme la scène) sans avoir à recréer un modèle 3D complet. Cela facilite les ajustements futurs du modèle pour correspondre aux différentes éditions.


## Chargement dans PyBullet
Pour charger un fichier URDF dans PyBullet, on utilise la fonction `loadURDF` de l'API PyBullet. Voici un exemple de code pour charger la table de jeu :

```python
# Chargement d'un objet inanimé
can_id = pybullet_manager.load_urdf("src/urdf_models/conserve.urdf", config.CAN_POS, [0.7071, 0, 0, 0.7071]) # Quaternion pour une rotation de 90° autour de l'axe X

# Chargement du robot (mobile)
robot_id = Robot("src/urdf_models/robot_cube.urdf", config.ROBOT_START_POS, [0, 0, 0, 1])

# Chargement de la table de jeu
table_id = pybullet_manager.load_urdf("src/urdf_models/table_eurobot2025.urdf", TABLE_POS, [0, 0, 0, 1])

# Chargement du sol (damier présent par défaut dans pybullet)
plane_id = pybullet_manager.load_urdf("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
```