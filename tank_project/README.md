# Tank Arena - Projet de Combat Robotique en Realite Mixte

Systeme de jeu robotique en temps reel combinant vision par ordinateur, projection augmentee, intelligence artificielle et controle ROS. Deux robots physiques (Turtlebot Burger) s'affrontent dans une arene projetee : un robot controle par IA et un robot pilote par un humain.

---

## Vue d'ensemble du systeme

### Principe general

- **Arene physique** : Tapis au sol avec ArUco markers et obstacles
- **Vision** : Camera RealSense D435 en vue aerienne
- **Projection** : Projecteur affichant la zone de jeu, HUD, effets visuels
- **Robots** : 2x Turtlebot Burger avec marqueurs ArUco
  - Robot 4 (IA) : Controle par l'algorithme
  - Robot 5 (Humain) : Controle physiquement par le joueur
- **Communication** : ROS Bridge (WebSocket) pour envoyer commandes au robot IA

### Pipeline complet

```
RealSense Camera -> ArUco Detection -> Kalman Filtering
                         |
                   World Model (poses + occupancy grid)
                         |
            +-----------+-----------+
            |                       |
      Game Engine              AI Strategy
    (arbitre, tirs)        (behavior tree, A*)
            |                       |
      Visualization <-------- ROS Bridge Client
    (Pygame projector)        (commandes robot IA)
```

---

## Architecture du projet

```
tank_project/
|
+-- core/                      # Logique metier (independante vision/affichage)
|   +-- game/                  # Arbitre et regles de jeu
|   |   +-- game_engine.py     # Boucle principale, orchestration
|   |   +-- state.py           # Etat complet (robots, score, timer)
|   |   +-- rules.py           # Regles et parametres de jeu
|   |   +-- raycast.py         # Detection collision tirs
|   |   +-- timers.py          # Cooldowns tirs, timer match
|   |   +-- hits.py            # Gestion impacts
|   |
|   +-- ia/                    # Intelligence artificielle
|   |   +-- behavior_tree.py   # Arbre de comportement modulaire
|   |   +-- decisions.py       # Conditions tactiques (LOS, distance)
|   |   +-- strategy.py        # Combiner BT -> objectif + fire_request
|   |   +-- planners/          # Path planning
|   |       +-- a_star.py      # Algorithme A*
|   |       +-- heuristics.py  # Fonctions heuristiques
|   |       +-- path_utils.py  # Lissage, simplification trajectoires
|   |
|   +-- world/                 # Representation monde 2D metrique
|   |   +-- world_model.py     # Unification : robots, obstacles, zones
|   |   +-- occupancy_grid.py  # Grille d'occupation 2D (resolution 2cm)
|   |   +-- inflation.py       # Dilatation obstacles (securite)
|   |   +-- coordinate_frames.py  # Transformations Camera<->World<->Pygame
|   |
|   +-- control/               # Controle bas niveau
|       +-- trajectory_follower.py  # Suivi waypoints (Pure Pursuit)
|       +-- kinematics.py      # Modele cinematique Turtlebot
|       +-- motion_constraints.py  # Limites vitesse/acceleration
|       +-- ros_bridge_client.py   # Client WebSocket -> ROS
|
+-- perception/                # Vision et calibration
|   +-- camera/
|   |   +-- realsense_stream.py    # Interface RealSense D435
|   |   +-- aruco_detector.py      # Detection markers ArUco
|   |   +-- color_segmentation.py  # Seuillage obstacles
|   |   +-- kalman_filter.py       # Filtrage poses (x,y,theta,vx,vy,omega)
|   |   +-- homography.py          # Calculs H_C2AV, H_C2W
|   |
|   +-- calibration/           # Phase de calibration
|   |   +-- calibration_wizard.py  # Sequence interactive complete
|   |   +-- scale_estimator.py     # Estimation echelle metrique
|   |   +-- arena_solver.py        # Calcul dimensions arene
|   |   +-- projector_mapping.py   # Transform Monde -> Projecteur
|   |
|   +-- preprocessing/
|       +-- thresholding.py    # Pretraitement images
|       +-- contours.py        # Extraction formes obstacles
|       +-- image_utils.py     # Utilitaires CV
|
+-- visualization/             # Affichage et projection
|   +-- pygame_renderer.py     # Moteur de rendu principal
|   +-- projector_overlay.py   # Conversion coordonnees -> pixels projecteur
|   +-- ui_hud.py              # HUD (timer, score, infos)
|   +-- debug_draw.py          # Visualisation debug (paths, LOS, grid)
|   +-- colors.py              # Palette de couleurs
|
+-- config/                    # Configuration YAML
|   +-- arena.yaml             # Calibration, H_C2W, dimensions
|   +-- camera.yaml            # Parametres camera, IDs ArUco
|   +-- ia.yaml                # Seuils IA, distances securite
|   +-- game.yaml              # Duree match, cooldowns
|   +-- robot.yaml             # Dimensions robot, vitesses max
|
+-- scripts/                   # Points d'entree
|   +-- run_calibration.py     # Lancer wizard calibration
|   +-- run_game.py            # Lancer partie (boucle 30 FPS)
|   +-- export_debug_data.py   # Export snapshots debug
|
+-- assets/                    # Ressources
|   +-- aruco_markers/         # Images markers haute resolution
|   +-- fonts/                 # Polices pour HUD
|   +-- images/                # Textures, icones
|
+-- logs/                      # Logs centralises
|   +-- runtime.log
|   +-- calibration.log
|   +-- debug.log
|
+-- README.md                  # Ce fichier
```

---

## Phase 1 : Calibration (one-time setup)

La calibration s'effectue **une seule fois** avant la premiere partie, ou quand l'arene change.

### Objectif

Etablir la transformation **Camera -> World** (metrique) et cartographier les obstacles fixes.

### Etapes du wizard

#### 1.1 Definition Safe Zone

**But** : Definir la marge de projection pour eviter les bords du projecteur.

- Marge configuree : `MARGIN = 50 px`
- Zone de jeu projetee : `(50, 50) -> (974, 718)` pour projecteur 1024x768

**Logs** :
```
[CALIB] MARGIN set to 50 px
[CALIB] Arena rect in projector: (50,50) -> (974,718)
```

#### 1.2 Calibration geometrique (H_C2AV)

**But** : Obtenir l'homographie **Camera -> Arena Virtual** (espace normalise [0,1]x[0,1]).

**Procedure** :
1. Projecteur affiche 4 ArUco aux coins de l'arene (IDs 0-3)
2. Camera detecte les 4 markers
3. Correspondances etablies :
   - ArUco 0 -> (0.0, 0.0) bottom-left
   - ArUco 1 -> (1.0, 0.0) bottom-right
   - ArUco 2 -> (1.0, 1.0) top-right
   - ArUco 3 -> (0.0, 1.0) top-left
4. Calcul homographie : `H_C2AV = cv2.findHomography(src_points, dst_points)`

**Logs** :
```
[CALIB] Step 2/4: Geometric calibration
[CALIB] Detected 4 projected corners
[CALIB] H_C2AV computed successfully
```

#### 1.3 Calibration metrique (H_C2W)

**But** : Convertir l'espace Arena Virtual en **metres reels**.

**Procedure** :
1. Placer un ArUco physique de taille connue dans l'arene (ex: 10 cm)
2. Utilisateur entre la taille reelle : `marker_size_real = 0.10 m`
3. Detection ArUco robot (ID 4 ou 5)
4. Estimation taille en unites AV :
   ```
   size_av = estimate_marker_size_av(corners, H_C2AV)
   ```
5. Calcul de l'echelle :
   ```
   scale = marker_size_real / size_av
   ```
6. Construction matrice de scaling :
   ```
   S = [[scale,  0,      0],
        [0,      scale,  0],
        [0,      0,      1]]
   ```
7. Homographie finale :
   ```
   H_C2W = S * H_C2AV
   ```

**Logs** :
```
[CALIB] Real marker size: 0.10 m
[CALIB] Marker size in AV: 0.087 units
[CALIB] Scale: 1.149 m / AV_unit
[CALIB] H_C2W computed
[CALIB] Calibration OK
```

**Resultat** :
- Transformation complete : `(u, v)` pixels camera -> `(x_W, y_W)` metres monde

#### 1.4 Cartographie obstacles statiques

**But** : Detecter obstacles fixes et creer la grille d'occupation de base.

**Procedure** :
1. Projecteur affiche fond blanc uniforme
2. Placer obstacles physiques (blocs, murs)
3. Capture image camera
4. Seuillage : obstacles sombres sur fond blanc
5. Extraction contours
6. Conversion pixels -> metres via `H_C2W`
7. Remplissage grille d'occupation :
   - Resolution : `RES = 0.02 m` (2 cm)
   - Taille arene : ex. `L = 2.85 m`, `W = 1.90 m`
   - Dimensions grille : `Nx = 143, Ny = 95 cellules`
   - Valeurs : `0 = libre`, `1 = obstacle fixe`

**Logs** :
```
[CALIB] Step 4/4: Obstacle mapping
[CALIB] Arena size estimated: 2.85m x 1.90m
[CALIB] Grid resolution: 0.02m -> 143 x 95 cells
[CALIB] Static obstacles mapped
```

**Sauvegarde** : Tous les resultats dans `config/arena.yaml`

---

## Phase 2 : Jeu (boucle temps reel 30 FPS)

Une fois calibre, le systeme entre en boucle de jeu.

### 2.1 Vision et Tracking

#### 2.1.1 Acquisition
- RealSense capture frame couleur (et optionnellement profondeur)

#### 2.1.2 Detection ArUco robots
- Robot IA -> **ID 4**
- Robot Humain -> **ID 5**

Pour chaque robot detecte :
- Centre marker en pixels : `(u, v)`
- Orientation : `theta_cam` (radians)

#### 2.1.3 Transformation metrique

Conversion pixels -> monde :
```
[x_W, y_W, 1]^T = H_C2W * [u, v, 1]^T
```

Resultat : `(x_W, y_W)` en metres, `theta` en radians (repere monde)

**Logs (exemple 1 Hz)** :
```
[VISION] Robot4 raw (px): (1023,540) -> (1.23, 0.87)m, theta=1.57 rad
[VISION] Robot5 raw (px): (856,712) -> (0.95, 1.42)m, theta=-0.52 rad
```

### 2.2 Filtrage Kalman

**But** : Stabiliser poses et estimer vitesses.

#### Modele d'etat (pour chaque robot)

Etat : `X = [x, y, vx, vy, theta, omega]^T`

Modele discret (dt = 1/30 s) :
```
x_{k+1} = x_k + vx_k * dt
y_{k+1} = y_k + vy_k * dt
vx_{k+1} = vx_k
vy_{k+1} = vy_k
theta_{k+1} = theta_k + omega_k * dt
omega_{k+1} = omega_k
```

Mesures camera : `Z = [x_mes, y_mes, theta_mes]^T`

#### Cycle Kalman
1. **Predict** : Projection etat a k+1
2. **Update** : Correction avec mesure ArUco

**Avantages** :
- Positions/orientations stables
- Vecteurs vitesse pour anticipation IA

**Logs** :
```
[KALMAN] Robot4 state: x=1.21, y=0.88, vx=0.03, vy=-0.01, theta=1.60, omega=0.02
[KALMAN] Robot5 state: x=0.96, y=1.41, vx=-0.08, vy=0.05, theta=-0.50, omega=-0.10
```

### 2.3 Grille d'occupation dynamique

**Mise a jour** :
1. Partir de la grille statique (obstacles fixes)
2. Marquer robots comme obstacles dynamiques :
   - Rayon robot : `R_robot = 0.18 m`
   - Conversion cellules : `R_cells = int(0.18 / 0.02) = 9 cellules`
3. **Inflation** : Rayon securite additionnel
   - Rayon total : `0.24 m -> 12 cellules`
4. Costmap style ROS :
   - `0` = libre
   - `100` = obstacle
   - Valeurs intermediaires possibles

**Usage** : Line-of-sight IA, path planning

**Logs** :
```
[GRID] Dynamic obstacles updated (Robot4, Robot5)
[GRID] Inflated radius: 0.24m -> 12 cells
```

### 2.4 Game Engine (arbitre)

**Responsabilites** : Application stricte des regles, aucune decision "intelligente".

#### 2.4.1 Timers et cooldowns

Variables maintenues :
- `t_game` : Temps ecoule depuis debut partie
- `next_allowed_shot_human` : Prochain tir humain autorise
- `next_allowed_shot_ai` : Prochain tir IA autorise

Exemple configuration :
- Tir humain automatique : `T_human = 5.0 s`
- Cooldown tir IA : `T_ai_cooldown = 3.0 s`

#### 2.4.2 Gestion tir humain

```
Si t_now >= next_allowed_shot_human :
    1. Calcul rayon depuis (x_H, y_H) direction theta_H
    2. Raycast (detection collisions obstacles + robots)
    3. Si Robot4 touche -> hits_robot4 += 1
    4. next_allowed_shot_human = t_now + T_human
```

#### 2.4.3 Gestion tir IA

```
Si IA renvoie fire_request=True ET cooldown OK :
    1. Raycast depuis (x_A, y_A) direction theta_A
    2. Si Robot5 touche -> hits_robot5 += 1
    3. next_allowed_shot_ai = t_now + T_ai_cooldown
```

#### 2.4.4 Fin de partie

**Conditions** :
- Duree ecoulee : `t_now >= t_start + T_match` (ex: 3 min)
- OU nombre hits atteint : `hits_robot >= H_max`

**Determination vainqueur** :
- Plus grand nombre de hits infliges
- Ou moins de hits recus

**Logs** :
```
[GAME] Human fired -> HIT Robot4
[GAME] AI fired -> MISS
[GAME] Hits: R4=2, R5=3
[GAME] Time remaining: 120s
[GAME] Match end, winner: Robot5 (HUMAN)
```

### 2.5 Intelligence Artificielle (Robot 4)

L'IA **propose** des actions, n'applique rien directement.

#### 2.5.1 Entrees IA

- Pose filtree Robot4 : `(x_A, y_A, theta_A, vx_A, vy_A, omega_A)`
- Pose filtree Robot5 : `(x_H, y_H, theta_H, vx_H, vy_H, omega_H)`
- Occupancy grid gonflee (costmap)
- Etat jeu : temps restant, hits, cooldowns

#### 2.5.2 Arbre de comportement (Behavior Tree)

**Structure** :

```
Selector (priorite)
+-- Sequence "SURVIE"
|   +-- Condition: distance_ennemi < d_safe ?
|   +-- Action: RETREAT (chercher cover, s'eloigner)
|
+-- Selector "COMBAT"
    +-- Sequence "ATTAQUE"
    |   +-- Condition: line_of_sight claire ?
    |   +-- Action: maintenir distance optimale
    |   +-- Action: fire_request = True
    |
    +-- Sequence "REPOSITIONNEMENT"
        +-- Action: FLANK (contourner obstacles)
        +-- Action: chercher position tir
```

**Etats possibles** :
- `RETREAT` : Fuite, priorite survie
- `ATTACK` : Ligne de vue claire, tir actif
- `FLANK` : Contournement tactique
- `HUNT` : Recherche ennemi

#### 2.5.3 Path Planning

**Algorithme** : A* sur grille d'occupation

**Processus** :
1. IA decide objectif : `(x_goal, y_goal)`
2. Conversion en cellule grille
3. A* calcule chemin :
   - Depart : cellule Robot4
   - Arrivee : cellule proche de goal (libre)
   - Heuristique : distance euclidienne
   - Cout : costmap (eviter obstacles)
4. Resultat : liste waypoints `[(x_1, y_1), ..., (x_n, y_n)]` en metres

**Logs** :
```
[AI] State: FLANK, target=(1.85, 1.20)
[AI] Path computed: 32 waypoints
[AI] LOS: FALSE, fire_request: FALSE
```

### 2.6 Controle et ROS Bridge

#### 2.6.1 Suivi de trajectoire

**Controleur simple** (Pure Pursuit) :

```
Waypoint actuel: (x_wp, y_wp)
Erreur position:
    dx = x_wp - x_A
    dy = y_wp - y_A
    distance = sqrt(dx^2 + dy^2)

Erreur orientation:
    theta_target = atan2(dy, dx)
    dtheta = angle_diff(theta_A, theta_target)

Commandes:
    v = k_v * distance          (vitesse lineaire)
    omega = k_theta * dtheta    (vitesse angulaire)

Contraintes:
    v in [-0.22, 0.22] m/s
    omega in [-2.84, 2.84] rad/s
```

**Cas speciaux** :
- Waypoint atteint -> passer au suivant
- Plus de waypoint -> `v=0`, orientation vers ennemi si tir

#### 2.6.2 Envoi ROS Bridge

**Format message** (WebSocket JSON) :
```json
{
  "robot_id": 4,
  "linear": 0.15,
  "angular": -0.30,
  "timestamp": 1702234567.123
}
```

ROS-bridge recoit -> publie sur `/cmd_vel` (Twist)

**Logs** :
```
[CTRL] Robot4 cmd: v=0.15 m/s, omega=-0.30 rad/s
[ROS] Command sent to bridge
[ROS] Latency: 12ms
```

### 2.7 Visualisation (Pygame + Projecteur)

#### 2.7.1 Transformation Monde -> Projecteur

**Parametres** :
- Arene metrique : `Lx x Ly` metres
- Zone projetee : `ARENA_PX_WIDTH x ARENA_PX_HEIGHT` pixels
- Marge : `MARGIN` pixels

**Scale** :
```
Sx = ARENA_PX_WIDTH / Lx
Sy = ARENA_PX_HEIGHT / Ly
S = min(Sx, Sy)    # uniforme, garde aspect ratio
```

**Conversion point** :
```
Point monde: (x_W, y_W) en metres

Pixels projecteur:
    Xp = MARGIN + x_W * S
    Yp = MARGIN + (Ly - y_W) * S    # y=0 en bas
```

#### 2.7.2 Elements affiches

**1. Fond arene**
- Rectangle zone de jeu
- Bordures, grille optionnelle

**2. Obstacles**
- Rectangles/polygones alignes grille
- Couleur distincte (gris fonce)

**3. Robots**
- Cercles position `(x_W, y_W)` -> `(Xp, Yp)`
- Orientation : petit trait dans direction `theta`
- Couleurs : bleu (IA), rouge (humain)

**4. Canon virtuel**
- Ligne de visee (1 metre) :
  ```
  x_end = x_robot + cos(theta)
  y_end = y_robot + sin(theta)
  ```
- Conversion `(x_end, y_end)` -> pixels
- Couleur selon etat (blanc normal, jaune si tir imminent)

**5. Lock-on IA**
- Si IA a line-of-sight sur humain :
  - Point rouge clignotant sur Robot5
  - Indicateur "LOCKED"

**6. HUD (interface)**
- **Timer** : Temps restant (MM:SS)
- **Scores** :
  ```
  HITS IA: 3
  HITS HUMAN: 2
  ```
- **Etat IA** : ATTACK / FLANK / RETREAT
- **Cooldowns** : Barres de progression tirs

**7. Fin de partie**
- Ecran overlay : `"WINNER: HUMAN"` ou `"WINNER: AI"`
- Recapitulatif scores
- Option rejouer

**Logs** :
```
[VIS] Frame rendered @30fps
[VIS] LOS=TRUE, Lock-on active
[VIS] HumanHits=2, AIHits=3
```

---

## Espaces de coordonnees

Le systeme utilise 4 reperes principaux :

### 1. Camera (C)
- Origine : Centre optique camera
- Unites : **pixels**
- Coordonnees : `(u, v)`
- Axes : u->droite, v->bas

### 2. Arena Virtual (AV)
- Origine : Coin bas-gauche arene
- Unites : **normalise [0, 1]**
- Coordonnees : `(x_av, y_av)`
- Usage : Intermediaire calibration

### 3. World (W)
- Origine : Coin bas-gauche arene
- Unites : **metres**
- Coordonnees : `(x_W, y_W)`
- Axes : x->droite, y->haut, z->sortant (regle main droite)
- **Principal repere utilise**

### 4. Projecteur (PROJ)
- Origine : Coin haut-gauche image projetee
- Unites : **pixels projecteur**
- Coordonnees : `(Xp, Yp)`
- Resolution : ex. 1024x768

### Transformations

```
Camera -> AV:    H_C2AV  (homographie, calibration geometrique)
AV -> World:     S       (scaling metrique)
Camera -> World: H_C2W = S * H_C2AV
World -> Proj:   Scaling lineaire + translation (margin)
```

---

## Principes d'architecture

### Separation des responsabilites

Chaque module a un role clair :
- **Vision** : Capte et detecte
- **Monde** : Represente etat metrique
- **Jeu** : Applique regles
- **IA** : Decide strategie
- **Controle** : Execute mouvements
- **Visualisation** : Affiche

### Clean Architecture

- `core/` ne depend **JAMAIS** de :
  - Pygame
  - Camera
  - Sockets
- `perception/` ne depend **JAMAIS** de :
  - Logique jeu
  - IA
- `visualization/` ne prend **JAMAIS** de decisions

### Modularite

**Remplacable facilement** :
- Changer camera (ZED, webcam) -> toucher uniquement `perception/camera/`
- Changer IA -> toucher uniquement `core/ia/`
- Changer pathfinding -> toucher uniquement `core/ia/planners/`
- Ajouter robot -> configuration, pas code

### Configuration externalisee

**Tout ce qui peut varier -> YAML** :
- Dimensions arene
- Vitesse robots
- Seuils IA
- Duree partie
- IDs ArUco

**Avantage** : Reconfigurer sans recompiler

### Scalabilite

**Extensions futures faciles** :
- Mode 1v1v1 (3 robots)
- Nouveaux types obstacles
- Power-ups projetes
- Mini-map temps reel
- Multiples strategies IA switchables
- Enregistrement replay

---

## Utilisation

### Premiere utilisation : Calibration

```bash
# Activer environnement Python
pyenv activate ubuntu

# Lancer wizard calibration
python3 scripts/run_calibration.py
```

**Suivre les instructions** :
1. Definir safe zone
2. Detecter coins projetes (4 ArUco)
3. Placer marker physique connu (ex: 10cm)
4. Cartographier obstacles fixes

-> Genere `config/arena.yaml`

### Lancer une partie

```bash
# Activer environnement
pyenv activate ubuntu

# Demarrer ROS Bridge (terminal separe)
# roslaunch rosbridge_server rosbridge_websocket.launch

# Lancer le jeu
python3 scripts/run_game.py
```

**Boucle principale 30 FPS** :
- Vision tracking
- IA decisions
- Controle robot IA
- Projection visualisation

### Debugging

```bash
# Export debug snapshot (config + grille + etat)
python3 scripts/export_debug_data.py

# Capture live depuis camera (requiert RealSense connectee)
python3 scripts/export_debug_data.py --live

# Export vers repertoire specifique
python3 scripts/export_debug_data.py --output-dir ~/mon_debug

# Voir logs temps reel
tail -f logs/runtime.log

# Activer affichage debug (config/game.yaml)
debug_mode: true
```

**Contenu export debug** :
- `config/` : Tous les fichiers YAML de configuration
- `occupancy_grid.npy` / `.png` : Grille d'occupation (NumPy + visualisation)
- `game_state.json` : Etat complet du jeu (poses, scores, IA)
- `manifest.json` : Index des elements exportes
- `camera_frame.png` / `_annotated.png` : Captures camera (mode `--live`)
- `aruco_detections.json` : Detections ArUco (mode `--live`)
- `depth_frame.npy` / `_viz.png` : Donnees profondeur (mode `--live`)
- `*.log` : Copies des fichiers logs

---

## Logs et Monitoring

### Format logs

Tous les logs suivent le pattern :
```
[MODULE] Message detaille
```

**Modules** :
- `[CALIB]` : Calibration
- `[VISION]` : Detection ArUco
- `[KALMAN]` : Filtrage
- `[GRID]` : Grille occupation
- `[GAME]` : Arbitre
- `[AI]` : Strategie IA
- `[CTRL]` : Controle
- `[ROS]` : Communication bridge
- `[VIS]` : Visualisation

### Fichiers logs

- `logs/calibration.log` : Historique calibrations
- `logs/runtime.log` : Parties jouees
- `logs/debug.log` : Informations detaillees debug

---

## Configuration

### Fichiers cles

#### `config/arena.yaml`
```yaml
projector:
  width: 1024
  height: 768
  margin_px: 50

display:
  fullscreen: false
  display_index: 0

arena:
  width_m: 2.85
  height_m: 1.90

transform:
  H_C2W: [[...], [...], [...]]  # 3x3 matrix
  scale_m_per_av: 1.149

obstacles: [...]  # Liste obstacles fixes
```

#### `config/game.yaml`
```yaml
match:
  duration_seconds: 180
  tick_rate_fps: 30

cooldowns:
  human_shot_seconds: 5.0
  ai_shot_seconds: 3.0

win_conditions:
  max_hits: 10
```

#### `config/ia.yaml`
```yaml
behavior:
  danger_distance_m: 0.8
  optimal_range_min_m: 1.2
  optimal_range_max_m: 3.5

strategy:
  heuristic: euclidean
  path_simplify: true

decision_rate:
  replan_interval: 10
```

---

## Dependances

### Python
- Python 3.8+
- OpenCV (cv2) avec module aruco
- NumPy
- Pygame
- PyYAML
- pyrealsense2 (Intel RealSense SDK)
- scipy (pour distance transform)

### Installation

```bash
pip install -r requirements.txt
```

### Materiel requis
- Intel RealSense D435 ou D455
- Projecteur 1024x768 ou superieur
- 2x Turtlebot Burger avec marqueurs ArUco
- ROS Bridge Server

---

## Auteurs

Projet VA-51 - Systeme de combat robotique en realite mixte.
