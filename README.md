# Projet VA50 - Tank Arena Robotique

Système complet de combat robotique en réalité mixte combinant vision par ordinateur, intelligence artificielle, et contrôle ROS2.

## Vue d'Ensemble

Le projet VA50 est composé de **3 modules interdépendants** :

1. **tank_project** - Moteur de jeu principal avec IA, vision et projection AR
2. **control_turtlebot** - Interface de contrôle manuel via WebSocket/Pygame  
3. **turtlebot_game** - Pont sécurisé WebSocket ↔ ROS2

## Architecture Globale

```
Caméra RealSense → tank_project (Perception + IA + Projection)
                        ↓
                   WebSocket/ROS2
                        ↓
                turtlebot_game (Safety Bridge)
                        ↓
                   TurtleBot Robots
```

## Prérequis Système

### Matériel

- **Ordinateur hôte** : Ubuntu 20.04+ avec GPU recommandé
- **Caméra** : Intel RealSense D435
- **Projecteur** : 1920×1080 minimum
- **Robots** : 2× TurtleBot3 Burger avec marqueurs ArUco
- **Réseau** : WiFi pour communication robot (192.168.50.x)

### Logiciels

- Python 3.8+
- ROS2 Humble
- OpenCV 4.5+
- Pygame 2.0+
- PyRealSense2

## Installation

### 1. Installation Dépendances Système

```bash
# ROS2 Humble (si pas déjà installé)
sudo apt update
sudo apt install ros-humble-desktop

# Dépendances vision
sudo apt install python3-opencv librealsense2-dev python3-pyrealsense2

# Dépendances Python
pip3 install pygame pyyaml numpy scipy websockets
```

### 2. Build Workspace ROS2

```bash
cd /home/julien/ros2_ws
colcon build --packages-select turtlebot_game
source install/setup.bash
```

### 3. Installation Modules Python

```bash
# tank_project
cd src/VA50/tank_project
pip3 install -r requirements.txt

# control_turtlebot
cd ../control_turtlebot
pip3 install -r requirements.txt
```

## Lancement des Projets

### Scénario 1 : Partie Automatique (AR Tank Combat)

**Terminal 1 - ROS2 Bridge** :
```bash
cd /home/julien/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=30
ros2 run turtlebot_game safety_bridge
```

**Terminal 2 - Tank Project** (après calibration) :
```bash
cd /home/julien/ros2_ws/src/VA50/tank_project
python3 main.py game
```

### Scénario 2 : Contrôle Manuel

**Terminal 1 - ROS2 Bridge** (même commande que ci-dessus)

**Terminal 2 - Control UI** :
```bash
cd /home/julien/ros2_ws/src/VA50/control_turtlebot  
python3 main.py
```

### Calibration Initiale (Tank Project)

**Obligatoire avant la première partie** :

```bash
cd /home/julien/ros2_ws/src/VA50/tank_project
python3 main.py calibration
```

Suivre les instructions du wizard :
1. Définir zone sécurité projecteur
2. Détecter coins ArUco projetés (IDs 0-3)
3. Mesurer marqueur physique (ID 4 ou 5)
4. Cartographier obstacles fixes

## Connexion au TurtleBot

### Configuration Réseau

Les TurtleBots doivent être sur le même réseau :
- **TurtleBot IP** : `192.168.50.1`
- **Utilisateur** : `turtlebot`
- **ROS_DOMAIN_ID** : `30`

### Connexion SSH Rapide

Utiliser le script helper fourni :

```bash
cd /home/julien/ros2_ws/src/VA50
./scripts/connect_turtlebot.sh [IP]
```

Ou connexion manuelle :

```bash
ssh turtlebot@192.168.50.1
```

Une fois connecté, configurer ROS2 :

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30
ros2 launch turtlebot3_bringup robot.launch.py
```

### Commande SSH en Une Ligne

```bash
ssh turtlebot@192.168.50.1 "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=30 && ros2 launch turtlebot3_bringup robot.launch.py"
```

## Configuration Réseau

### Sur le TurtleBot

Assurer que le fichier `~/.bashrc` contient :

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30
export TURTLEBOT3_MODEL=burger
```

### Sur l'Ordinateur Hôte

Ajouter à `~/.bashrc` :

```bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
```

Puis :

```bash
source ~/.bashrc
```

## Structure des 3 Modules

### tank_project (97 fichiers)

**Rôle** : Moteur de jeu AR avec IA et vision

- `core/` - Logique métier (jeu, IA, contrôle, monde)
- `perception/` - Vision et calibration
- `visualization/` - Rendu Pygame
- `config/` - 5 fichiers YAML

**Documentation** : Voir [tank_project/README.md](tank_project/README.md)

### control_turtlebot (7 fichiers + modules)

**Rôle** : Interface contrôle manuel

- `config/` - Configuration YAML
- `core/` - Contrôleurs et networking
- `ui/` - Interface Pygame

**Documentation** : Voir [control_turtlebot/README.md](control_turtlebot/README.md)

### turtlebot_game (8 fichiers)

**Rôle** : Pont WebSocket ↔ ROS2

- Package ROS2 (ament_python)
- Safety bridge avec validation vitesse
- Multi-clients WebSocket

**Documentation** : Voir [turtlebot_game/README.md](turtlebot_game/README.md)

## Workflows Typiques

### Démarrage Complet Partie Automatique

```bash
# 1. Démarrer ROS bridge sur robot
ssh turtlebot@192.168.50.1 "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=30 && ros2 launch turtlebot3_bringup robot.launch.py" &

# 2. Démarrer safety bridge local
cd /home/julien/ros2_ws && source install/setup.bash && export ROS_DOMAIN_ID=30
ros2 run turtlebot_game safety_bridge &

# 3. Lancer tank project
cd src/VA50/tank_project && python3 main.py game
```

### Test Contrôle Manuel

```bash
# Terminal 1
cd /home/julien/ros2_ws && source install/setup.bash && export ROS_DOMAIN_ID=30
ros2 run turtlebot_game safety_bridge

# Terminal 2
cd src/VA50/control_turtlebot
python3 main.py
```

## Troubleshooting

### Problème : Pas de connexion WebSocket

**Solutions** :
- Vérifier que `safety_bridge` est en cours d'exécution
- Vérifier URI dans `control_turtlebot/config/network.yaml`
- Tester : `telnet localhost 8765`

### Problème : ROS2 ne voit pas le robot

**Solutions** :
```bash
# Vérifier DOMAIN_ID
echo $ROS_DOMAIN_ID  # Doit être 30

# Lister topics
ros2 topic list

# Tester communication
ros2 topic echo /cmd_vel
```

### Problème : ArUco non détectés

**Solutions** :
- Vérifier éclairage de l'arène
- Réduire exposition caméra
- Imprimer markers haute résolution (200+ DPI)
- Relancer calibration : `python3 main.py calibration`

### Problème : Latence réseau élevée

**Solutions** :
- Se rapprocher du point d'accès WiFi
- Vérifier qualité signal : `iwconfig`
- Réduire `ping_interval_s` dans config
- Utiliser câble Ethernet si possible

### Problème : Robot ne bouge pas

**Checklist** :
1. ROS bridge actif sur robot ? `ros2 topic list`
2. Safety bridge reçoit commandes ? (logs)
3. Batterie robot chargée ?
4. Limites vitesse correctes ? Voir `turtlebot_game/config/`
5. Robot en mode manuel (boutons physiques) ?

## FAQ

**Q: Quelle est la différence entre control_turtlebot et tank_project ?**

A: `control_turtlebot` est une interface simple de contrôle manuel. `tank_project` est un jeu complet avec IA, vision, et arbitrage.

**Q: Peut-on utiliser plusieurs robots simultanément ?**

A: Oui, tank_project supporte 2 robots (ID 4 et 5). Pour plus, modifier la configuration.

**Q: Comment changer la vitesse max du robot ?**

A: Éditer `turtlebot_game/config/safety_config.py` ou passer paramètres ROS2.

**Q: Le projecteur est-il obligatoire ?**

A: Pour `tank_project` oui (AR). Pour `control_turtlebot` non (UI sur écran).

## Liens Utiles

- **Documentation Complète** : Voir README individuels de chaque module
- **TurtleBot3 Docs** : https://emanual.robotis.com/docs/en/platform/turtlebot3/
- **ROS2 Humble Docs** : https://docs.ros.org/en/humble/

## Support

Pour questions ou problèmes :
1. Consulter les README individuels
2. Vérifier logs : `tank_project/logs/`, ROS logs
3. Tester composants individuellement

---

**Dernière mise à jour** : 2025-12-06
