# Documentation : Traitement Image

## Introduction
Ce document explique le fonctionnement de l'algorithme de traitement des caméras virtuelles dans la simulation PyBullet et également en conditions réelles, utilisées pour la détection d'obstacles et de marqueurs ArUco. L'objectif est de traiter la capture d'images, la détection d'objets par couleur, la localisation 3D et la calibration des caméras.
On traitera ici les deux parties, simulation (`mainCameraSimu.py`) et réalité (`mainCameraReel.py`), car elles sont très similaires et partagent une grande partie du code.

## Algorithme de traitement caméra
L'algorithme de traitement des caméras se décompose en plusieurs étapes clés :
1. **Capture d'images** : Acquisition des images depuis la caméra (virtuelle ou réelle).
2. **Prétraitement** : Conversion des images en niveaux de gris, application de filtres pour réduire le bruit et améliorer la qualité de l'image.
3. **Calibration** : Utilisation de la calibration de la caméra pour corriger les distorsions et obtenir des mesures précises.
4. **Détection d'objets** :
   - **Détection par couleur** : Segmentation des objets basés sur des plages de couleurs définies (HSV) pour identifier les obstacles.
   - **Détection ArUco** : Utilisation de la bibliothèque OpenCV pour détecter les marqueurs ArUco dans l'image.
5. **Localisation 3D** : Calcul des positions 3D des objets détectés en utilisant les informations de profondeur (dans le cas de la caméra réelle) ou les paramètres de la caméra virtuelle.
6. **Affichage et visualisation** : Superposition des résultats de la détection sur l'image originale pour visualiser les objets détectés et leurs positions.


## Structure du code
Le code est organisé en plusieurs modules pour faciliter la maintenance et la compréhension :
- `config.py` : Contient les paramètres de configuration pour la caméra, les plages de couleurs, et les paramètres de calibration.
- `utils/virtualCamera.py` : Crée et paramètre les caméra virtuelle dans PyBullet.
- `utils/segmentation.py` : Contient les fonctions pour la segmentation des images basées sur la couleur.
- `utils/perspectiveCorrection.py` : Gère la correction de perspective des positions capturées.
- `utils/arucoDetection.py` : Implémente la détection des marqueurs ArUco.
- `utils/imageTransform.py` : Contient des fonctions utilitaires pour la transformation de perspective de la table.
- 

## Initialisation et configuration des caméras
Les caméras doivent respecter certaines contraintes de position (fixée par le réglement de la Coupe de France de Robotique), de plus nous avons des exigences supplémentaires : voir la table en entier, avoir une bonne résolution, et minimiser les occlusions. Pour cela nous avons choisi 2 caméras placées en hauteur aux coins opposés de la table, inclinées vers le centre.

Les caméras virtuelles sont paramétrées pour correspondre aux caractéristiques des caméras réelles utilisées (FOV, position, orientation).

## Connexion à la caméra
On connecte un module déporté (un ordinateur) à la caméra raspberry via SSH, le flux vidéo est transmis par Wi-Fi en partage de connexion (impossible avec eduroam, possible avec Wi-Fi Projet&Tech mais pas connecté à Internet).
Ci dessous les étapes de configuration de la connexion :
- **initialisation de la carte** : On charge un environnement sur la carte avec *Raspberry Pi Imager* sur la micro-SD en indiquant le **nom** et le **mot-de-passe** du partage de connexion. Attention au nom d'utilistaeur et mot de passe
- **connexion au partage** : Brancher la carte et attendre la connexion au partage
- **connexion SSH** : sur un terminal `ping raspberry.local -4` puis `ssh user@ip`, la connexion est établie
- **configuration vidéo**
Sur la RPi, dans `config-vid.txt` (avec nano par exemple) :

```
timeout=0
nopreview=
inline=
listen=
width=1280
height=720

codec=h264
level=4.2
bitrate=2048000
denoise=cdn_fast
framerate=30
profile=high
metering=spot

autofocus-mode=continuous
autofocus-speed=fast
```

Puis, toujours sur RPi :

```bash
rpicam-vid -c ~/config-vid.txt -o tcp://0.0.0.0:5001
```

Ensuite on peu lancer le code python (attention à bien modifier l'ip dans `RTSP_URL = 'tcp://192.168.114.241:5001'`

## Correction de perspective
Pour obtenir une vue "de dessus" de la table, nous appliquons une transformation de perspective aux images capturées. Cela permet de compenser l'angle d'inclinaison des caméras et de faciliter la détection des objets sur la table. La transformation s'appuie sur les 4 codes ArUco placés aux coins de la table. La transformation suit les étapes suivantes :
1. Utilisation de la matrice de transformation stockée dans `perspectiveMatrix.txt<camNumber>`.
2. Si absente, calcul de la matrice de transformation en détectant les marqueurs ArUco aux coins de la table.
3. Si manquant, utilisation des positions définies par l'utilisateur via des clics souris (ordre à respecter : coin haut gauche puis sens horaire).
4. Application de la transformation pour obtenir une vue corrigée de la table.

Le fichier `perspectiveMatrix.txt<camNumber>` est sauvegardé pour éviter de recalculer la matrice à chaque exécution. Il contient le ratio de pixels par mètre pour les axes X et Y, ainsi que la matrice de transformation homographique 3x3.

## Détection de codes ArUco (robot, PAMI)
En permanence, l'algorithme tente de détecter les codes ArUco dans l'image capturée, en cherchant la présence des codes robot et PAMI. Si détectés, leurs positions sont utilisées pour localiser précisément le robot et le PAMI sur la table. La détection suit les étapes suivantes :
1. Conversion de l'image en niveaux de gris.
2. Détection des marqueurs ArUco en utilisant les fonctions d'OpenCV.
3. Mise en évidence des marqueurs détectés sur l'image.
4. Calcul des positions 3D réelles des marqueurs en utilisant les paramètres de la caméra.

## Détection d'obstacles par couleur
L'algorithme segmente l'image pour détecter les obstacles basés sur des plages de couleurs définies (HSV). Chaque type d'obstacle (rouge, vert, bleu, jaune) est détecté séparément. La détection suit les étapes suivantes :
1. Conversion de l'image en espace de couleur HSV.
2. Application de masques pour chaque plage de couleur définie dans `config.py`.
3. Extraction des contours des objets détectés.
4. Mise en évidence des marqueurs détectés sur l'image.
5. Calcul des positions 3D réelles des marqueurs en utilisant les paramètres de la caméra.
