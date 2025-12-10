# Guide Rapide - Lancement du Projet

## 🚀 Méthodes de Lancement

### 1. Depuis le répertoire du projet

```bash
cd /home/julien/ros2_ws/src/tank_project

# Afficher l'aide
python3 main.py --help

# Lancer le jeu
python3 main.py
python3 main.py game

# Lancer la calibration
python3 main.py calibration

# Exporter données debug
python3 main.py export
```

### 2. En tant que module Python

```bash
cd /home/julien/ros2_ws/src

# Afficher l'aide
python3 -m tank_project --help

# Lancer le jeu
python3 -m tank_project
python3 -m tank_project game

# Lancer la calibration
python3 -m tank_project calibration

# Exporter données debug
python3 -m tank_project export
```

### 3. Via les scripts directs

```bash
cd /home/julien/ros2_ws/src/tank_project

# Calibration
python3 scripts/run_calibration.py

# Jeu
python3 scripts/run_game.py

# Export debug
python3 scripts/export_debug_data.py --output-dir logs/debug_custom
```

## 📋 Prérequis

### Installation des dépendances

```bash
cd /home/julien/ros2_ws/src/tank_project
pip3 install -r requirements.txt
```

Dépendances principales:
- `numpy` - Calculs numériques
- `opencv-python` - Vision par ordinateur
- `pyrealsense2` - Interface caméra RealSense
- `pygame` - Rendu graphique
- `scipy` - Traitement signal
- `pyyaml` - Configuration

## 🎯 Workflow Complet

### Première Utilisation

```bash
# 1. Installer dépendances
pip3 install -r requirements.txt

# 2. Lancer calibration (obligatoire la première fois)
python3 main.py calibration

# Suivre les instructions à l'écran:
# - Définir zone sécurité
# - Détecter coins projetés (ArUco 0-3)
# - Mesurer marqueur physique (ArUco 4 ou 5)
# - Cartographier obstacles

# 3. Vérifier calibration sauvegardée
cat config/arena.yaml

# 4. Lancer le jeu
python3 main.py game
```

### Utilisation Normale

```bash
# Lancer directement le jeu (calibration déjà faite)
python3 main.py
```

## 🐛 Debug & Diagnostic

### Export données debug

```bash
# Exporter snapshot complet
python3 main.py export

# Données exportées dans:
# logs/debug_YYYYMMDD_HHMMSS/
```

### Vérifier imports

```bash
# Tester que tous les modules s'importent correctement
python3 -c "
from core.game import game_engine
from core.ia import strategy
from perception.camera import aruco_detector
from visualization import pygame_renderer
print('✅ Tous les imports OK')
"
```

### Logs

Les logs sont stockés dans `logs/`:
- `runtime.log` - Logs d'exécution jeu
- `calibration.log` - Logs calibration
- `debug.log` - Logs debug général

Filtrer logs par module:
```bash
# Logs IA uniquement
python3 main.py 2>&1 | grep "\[AI\]"

# Logs vision uniquement
python3 main.py 2>&1 | grep "\[VISION\]"

# Logs calibration
python3 main.py calibration 2>&1 | tee logs/calibration.log
```

## ⚙️ Configuration

Modifier les paramètres dans `config/*.yaml`:

- `arena.yaml` - Dimensions arène, transformations
- `camera.yaml` - Paramètres RealSense, ArUco
- `game.yaml` - Règles jeu (durée, cooldowns)
- `ia.yaml` - Comportement IA (distances, seuils)
- `robot.yaml` - Specs Turtlebot (vitesses, dimensions)

Exemple:
```bash
# Éditer durée match (défaut: 180s)
nano config/game.yaml
```

## 🔧 Dépannage

### Erreur "No module named 'cv2'"
```bash
pip3 install opencv-python
```

### Erreur "No module named 'pyrealsense2'"
```bash
# Installer SDK RealSense
sudo apt-get install librealsense2-dev
pip3 install pyrealsense2
```

### Pygame ne démarre pas
```bash
pip3 install --upgrade pygame
```

### ROS bridge connection failed
Vérifier que le pont ROS est actif sur les robots:
```bash
# Sur le robot
ros2 run ros_bridge server
```

## 📚 Plus d'Informations

Voir le [README.md](README.md) complet pour:
- Architecture détaillée
- Description modules
- Mécaniques de jeu
- Détails techniques
