# BallFollower
## Introduction
BallFollower est un projet basé sur ROS2 qui permet à un robot mobile de détecter et suivre une balle en mouvement. Ce projet combine la vision par ordinateur, le traitement d'images, et le contrôle de robot pour créer un système capable de repérer une balle dans son environnement et de se déplacer en conséquence.

Le suivi de balle est une application couramment utilisée dans des scénarios robotiques tels que les compétitions de robotique, la recherche et l'éducation. Grâce à ce projet, vous pouvez contrôler un robot équipé d'une caméra qui détectera la position d'une balle de couleur et ajustera son mouvement pour la suivre. Le sujet est inspiré de la compétition RoboCupSoccer, qui est une commpétition de foot pour robots humanoïdes.
Nous avons eu un total de 20h allouées à la réalisation du projet, donc nous nous sommes concentrés sur la partie de détection et suivi d'une balle.

## Explication des mouvements du robot
Lorsque le robot ne détecte aucune balle dans son champ de vision, il tourne sur lui-même dans le sens anti-horaire.

Dès qu'une balle est détectée, il se dirige vers elle avec une vitesse proportionnelle à son éloignement. Sa vitesse linéaire est cependant plafonnée à 0.22 m/s pour éviter de ne plus détecter la balle à cause de la trainée engendrée par son mouvement. Plus le robot s'approche, plus il ralentit, jusqu'à atteindre la vitesse de 0.1 m/s.

> [!TIP]
> Veillez à fixer la caméra au plus proche du sol pour éviter tout angle mort.
> 
> Veillez à avoir un éclairage optimal pour favoriser la détection de la balle.

## Prérequis
Avant de commencer à configurer le projet, assurez-vous d'avoir les éléments suivants :

- Un ordinateur avec ROS2 installé (de préférence ROS2 Humble Hawksbill ou plus récent).
- Un robot mobile équipé d'une caméra compatible avec ROS2 (par exemple, TurtleBot3 ou tout autre robot équipé d'une caméra).
- Python 3.x.
- OpenCV installé sur votre environnement ROS2 (pour le traitement de l'image).

## Installation du projet
Clonez le dépôt du projet dans votre espace de travail ROS2 :
```
cd ~/ros2_ws/src
git clone https://github.com/emilie-gehl/BallFollower.git
```

Compilez le projet dans l'espace de travail :
```
cd ~/ros2_ws
colcon build
source install/setup.bash
```
  
## Utilisation
### Connexion avec la tortue (TurtleBot)
Une fois le robot allumé, se connecter à son wifi.
Ouvrir ensuite un terminal de commande et se connecter à la tortue via ssh:
```
SSH -X turtle@ip.de.la.turtle
```
L'ajout du paramètre '-X' permet d'ouvrir une fenêtre de retour caméra sur l'écran du pc connecté à la tortue.

### Lancement du programme
Lancer le fichier launch 'Ball_Launch.py' après avoir vérifier d'être dans le terminal connecté à la tortue :
```
ros2 launch BallFollower Ball_Launch.py
```

Vous êtes prêt à suivre une balle.

## Débugage
### Problème avec le fichier launch
Si le code ne se lance pas, essayer d'ouvrir un à un les fichiers pour identifier le problème. Il faut pour cela 3 terminaux différents, un pour chaque noeud et tous doivent être connectés en ssh à la tortue.
Terminal 1
```
ros2 launch turtlebot3_bringup robot.launch.py
```
Terminal 2
```
ros2 run BallFollower Ball_Publisher
```
Terminal 3
```
ros2 run BallFollower BallFollower
```
### Différente couleur de balle
Ce projet a été créé pour détecter une balle de couleur orange pâle. Il se peut que votre balle ne soit pas de la même couleur. Vous pouvez modifier la plage du masque de couleur dans le fichier Ball_Publisher.py (ligne 32) :
```
        # Plages de couleur pour l'orange
        self.lower_orange = np.array([0, 50, 150])
        self.upper_orange = np.array([25, 255, 255])
```
### ID de la caméra
Si la caméra n'est pas détectée, il est possible de changer l'ID de la caméra dans le fichier Ball_Publisher.py : 
```
ros2 run BallFollower BallFollower
```
