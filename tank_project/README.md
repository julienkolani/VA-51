# 🎮 Tank Arena - Projet de Combat Robotique en Réalité Mixte

Système de jeu robotique en temps réel combinant vision par ordinateur, projection augmentée, intelligence artificielle et contrôle ROS. Deux robots physiques (Turtlebot Burger) s'affrontent dans une arène projetée : un robot contrôlé par IA et un robot piloté par un humain.

---

## 📐 Vue d'ensemble du système

### Principe général

- **Arène physique** : Tapis au sol avec ArUco markers et obstacles
- **Vision** : Caméra RealSense D435 en vue aérienne
- **Projection** : Projecteur affichant la zone de jeu, HUD, effets visuels
- **Robots** : 2x Turtlebot Burger avec marqueurs ArUco
  - Robot 4 (IA) : Contrôlé par l'algorithme
  - Robot 5 (Humain) : Contrôlé physiquement par le joueur
- **Communication** : ROS Bridge (WebSocket) pour envoyer commandes au robot IA

### Pipeline complet

```
RealSense Camera → ArUco Detection → Kalman Filtering
                         ↓
                   World Model (poses + occupancy grid)
                         ↓
            ┌────────────┴────────────┐
            ↓                         ↓
      Game Engine              AI Strategy
    (arbitre, tirs)        (behavior tree, A*)
            ↓                         ↓
      Visualization ←──────── ROS Bridge Client
    (Pygame projector)        (commandes robot IA)
```

---

## 🗂️ Architecture du projet

```
tanker_project/
│
├── core/                      # Logique métier (indépendante vision/affichage)
│   ├── game/                  # Arbitre et règles de jeu
│   │   ├── game_engine.py     # Boucle principale, orchestration
│   │   ├── state.py           # État complet (robots, score, timer)
│   │   ├── rules.py           # Règles et paramètres de jeu
│   │   ├── raycast.py         # Détection collision tirs
│   │   ├── timers.py          # Cooldowns tirs, timer match
│   │   └── hits.py            # Gestion impacts
│   │
│   ├── ia/                    # Intelligence artificielle
│   │   ├── behavior_tree.py   # Arbre de comportement modulaire
│   │   ├── decisions.py       # Conditions tactiques (LOS, distance)
│   │   ├── strategy.py        # Combiner BT → objectif + fire_request
│   │   └── planners/          # Path planning
│   │       ├── a_star.py      # Algorithme A*
│   │       ├── heuristics.py  # Fonctions heuristiques
│   │       └── path_utils.py  # Lissage, simplification trajectoires
│   │
│   ├── world/                 # Représentation monde 2D métrique
│   │   ├── world_model.py     # Unification : robots, obstacles, zones
│   │   ├── occupancy_grid.py  # Grille d'occupation 2D (résolution 2cm)
│   │   ├── inflation.py       # Dilatation obstacles (sécurité)
│   │   └── coordinate_frames.py  # Transformations Camera↔World↔Pygame
│   │
│   └── control/               # Contrôle bas niveau
│       ├── trajectory_follower.py  # Suivi waypoints (Pure Pursuit)
│       ├── kinematics.py      # Modèle cinématique Turtlebot
│       ├── motion_constraints.py  # Limites vitesse/accélération
│       └── ros_bridge_client.py   # Client WebSocket → ROS
│
├── perception/                # Vision et calibration
│   ├── camera/
│   │   ├── realsense_stream.py    # Interface RealSense D435
│   │   ├── aruco_detector.py      # Détection markers ArUco
│   │   ├── color_segmentation.py  # Seuillage obstacles
│   │   ├── kalman_filter.py       # Filtrage poses (x,y,θ,ẋ,ẏ,ω)
│   │   └── homography.py          # Calculs H_C2AV, H_C2W
│   │
│   ├── calibration/           # Phase de calibration
│   │   ├── calibration_wizard.py  # Séquence interactive complète
│   │   ├── scale_estimator.py     # Estimation échelle métrique
│   │   ├── arena_solver.py        # Calcul dimensions arène
│   │   └── projector_mapping.py   # Transform Monde → Projecteur
│   │
│   └── preprocessing/
│       ├── thresholding.py    # Prétraitement images
│       ├── contours.py        # Extraction formes obstacles
│       └── image_utils.py     # Utilitaires CV
│
├── visualization/             # Affichage et projection
│   ├── pygame_renderer.py     # Moteur de rendu principal
│   ├── projector_overlay.py   # Conversion coordonnées → pixels projecteur
│   ├── ui_hud.py              # HUD (timer, score, infos)
│   ├── debug_draw.py          # Visualisation debug (paths, LOS, grid)
│   └── colors.py              # Palette de couleurs
│
├── config/                    # Configuration YAML
│   ├── arena.yaml             # Calibration, H_C2W, dimensions
│   ├── camera.yaml            # Paramètres caméra, IDs ArUco
│   ├── ia.yaml                # Seuils IA, distances sécurité
│   ├── game.yaml              # Durée match, cooldowns
│   └── robot.yaml             # Dimensions robot, vitesses max
│
├── scripts/                   # Points d'entrée
│   ├── run_calibration.py     # Lancer wizard calibration
│   ├── run_game.py            # Lancer partie (boucle 30 FPS)
│   └── export_debug_data.py   # Export snapshots debug
│
├── assets/                    # Ressources
│   ├── aruco_markers/         # Images markers haute résolution
│   ├── fonts/                 # Polices pour HUD
│   └── images/                # Textures, icônes
│
├── logs/                      # Logs centralisés
│   ├── runtime.log
│   ├── calibration.log
│   └── debug.log
│
└── README.md                  # Ce fichier
```

---

## 🔧 Phase 1 : Calibration (one-time setup)

La calibration s'effectue **une seule fois** avant la première partie, ou quand l'arène change.

### Objectif

Établir la transformation **Camera → World** (métrique) et cartographier les obstacles fixes.

### Étapes du wizard

#### 1.1 Définition Safe Zone

**But** : Définir la marge de projection pour éviter les bords du projecteur.

- Marge configurée : `MARGIN = 50 px`
- Zone de jeu projetée : `(50, 50) → (1870, 1030)` pour projecteur 1920×1080

**Logs** :
```
[CALIB] MARGIN set to 50 px
[CALIB] Arena rect in projector: (50,50) -> (1870,1030)
```

#### 1.2 Calibration géométrique (H_C2AV)

**But** : Obtenir l'homographie **Camera → Arena Virtual** (espace normalisé [0,1]×[0,1]).

**Procédure** :
1. Projecteur affiche 4 ArUco aux coins de l'arène (IDs 0-3)
2. Caméra détecte les 4 markers
3. Correspondances établies :
   - ArUco 0 → (0.0, 0.0) bottom-left
   - ArUco 1 → (1.0, 0.0) bottom-right
   - ArUco 2 → (1.0, 1.0) top-right
   - ArUco 3 → (0.0, 1.0) top-left
4. Calcul homographie : `H_C2AV = cv2.findHomography(src_points, dst_points)`

**Logs** :
```
[CALIB] Step 2/4: Geometric calibration
[CALIB] Detected 4 projected corners
[CALIB] H_C2AV computed successfully
```

**Option raffinement** :
- Répéter avec plusieurs captures
- Moyenner les homographies ou utiliser RANSAC

#### 1.3 Calibration métrique (H_C2W)

**But** : Convertir l'espace Arena Virtual en **mètres réels**.

**Procédure** :
1. Placer un ArUco physique de taille connue dans l'arène (ex: 10 cm)
2. Utilisateur entre la taille réelle : `marker_size_real = 0.10 m`
3. Détection ArUco robot (ID 4 ou 5)
4. Estimation taille en unités AV :
   ```
   size_av = estimate_marker_size_av(corners, H_C2AV)
   ```
5. Calcul de l'échelle :
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
   H_C2W = S · H_C2AV
   ```

**Logs** :
```
[CALIB] Real marker size: 0.10 m
[CALIB] Marker size in AV: 0.087 units
[CALIB] Scale: 1.149 m / AV_unit
[CALIB] H_C2W computed
[CALIB] Calibration OK
```

**Résultat** :
- Transformation complète : `(u, v)` pixels caméra → `(x_W, y_W)` mètres monde

#### 1.4 Cartographie obstacles statiques

**But** : Détecter obstacles fixes et créer la grille d'occupation de base.

**Procédure** :
1. Projecteur affiche fond blanc uniforme
2. Placer obstacles physiques (blocs, murs)
3. Capture image caméra
4. Seuillage : obstacles sombres sur fond blanc
5. Extraction contours
6. Conversion pixels → mètres via `H_C2W`
7. Remplissage grille d'occupation :
   - Résolution : `RES = 0.02 m` (2 cm)
   - Taille arène : ex. `L = 2.85 m`, `W = 1.90 m`
   - Dimensions grille : `Nx = 143, Ny = 95 cellules`
   - Valeurs : `0 = libre`, `1 = obstacle fixe`

**Logs** :
```
[CALIB] Step 4/4: Obstacle mapping
[CALIB] Arena size estimated: 2.85m x 1.90m
[CALIB] Grid resolution: 0.02m -> 143 x 95 cells
[CALIB] Static obstacles mapped
```

**Sauvegarde** : Tous les résultats dans `config/arena.yaml`

---

## 🎯 Phase 2 : Jeu (boucle temps réel 30 FPS)

Une fois calibré, le système entre en boucle de jeu.

### 2.1 Vision & Tracking

#### 2.1.1 Acquisition
- RealSense capture frame couleur (et optionnellement profondeur)

#### 2.1.2 Détection ArUco robots
- Robot IA → **ID 4**
- Robot Humain → **ID 5**

Pour chaque robot détecté :
- Centre marker en pixels : `(u, v)`
- Orientation : `theta_cam` (radians)

#### 2.1.3 Transformation métrique

Conversion pixels → monde :
```
[x_W, y_W, 1]^T = H_C2W * [u, v, 1]^T
```

Résultat : `(x_W, y_W)` en mètres, `theta` en radians (repère monde)

**Logs (exemple 1 Hz)** :
```
[VISION] Robot4 raw (px): (1023,540) -> (1.23, 0.87)m, theta=1.57 rad
[VISION] Robot5 raw (px): (856,712) -> (0.95, 1.42)m, theta=-0.52 rad
```

### 2.2 Filtrage Kalman

**But** : Stabiliser poses et estimer vitesses.

#### Modèle d'état (pour chaque robot)

État : `X = [x, y, vx, vy, θ, ω]^T`

Modèle discret (dt = 1/30 s) :
```
x_{k+1} = x_k + vx_k · dt
y_{k+1} = y_k + vy_k · dt
vx_{k+1} = vx_k
vy_{k+1} = vy_k
θ_{k+1} = θ_k + ω_k · dt
ω_{k+1} = ω_k
```

Mesures caméra : `Z = [x_mes, y_mes, θ_mes]^T`

#### Cycle Kalman
1. **Predict** : Projection état à k+1
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

**Mise à jour** :
1. Partir de la grille statique (obstacles fixes)
2. Marquer robots comme obstacles dynamiques :
   - Rayon robot : `R_robot ≈ 0.18 m`
   - Conversion cellules : `R_cells = int(0.18 / 0.02) ≈ 9 cellules`
3. **Inflation** : Rayon sécurité additionnel
   - Rayon total : `0.24 m → 12 cellules`
4. Costmap style ROS :
   - `0` = libre
   - `100` = obstacle
   - Valeurs intermédiaires possibles

**Usage** : Line-of-sight IA, path planning

**Logs** :
```
[GRID] Dynamic obstacles updated (Robot4, Robot5)
[GRID] Inflated radius: 0.24m -> 12 cells
```

### 2.4 Game Engine (arbitre)

**Responsabilités** : Application stricte des règles, aucune décision "intelligente".

#### 2.4.1 Timers & cooldowns

Variables maintenues :
- `t_game` : Temps écoulé depuis début partie
- `next_allowed_shot_human` : Prochain tir humain autorisé
- `next_allowed_shot_ai` : Prochain tir IA autorisé

Exemple configuration :
- Tir humain automatique : `T_human = 5.0 s`
- Cooldown tir IA : `T_ai_cooldown = 3.0 s`

#### 2.4.2 Gestion tir humain

```
Si t_now >= next_allowed_shot_human :
    1. Calcul rayon depuis (x_H, y_H) direction theta_H
    2. Raycast (détection collisions obstacles + robots)
    3. Si Robot4 touché → hits_robot4 += 1
    4. next_allowed_shot_human = t_now + T_human
```

#### 2.4.3 Gestion tir IA

```
Si IA renvoie fire_request=True ET cooldown OK :
    1. Raycast depuis (x_A, y_A) direction theta_A
    2. Si Robot5 touché → hits_robot5 += 1
    3. next_allowed_shot_ai = t_now + T_ai_cooldown
```

#### 2.4.4 Fin de partie

**Conditions** :
- Durée écoulée : `t_now >= t_start + T_match` (ex: 3 min)
- OU nombre hits atteint : `hits_robot >= H_max`

**Détermination vainqueur** :
- Plus grand nombre de hits infligés
- Ou moins de hits reçus

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

#### 2.5.1 Entrées IA

- Pose filtrée Robot4 : `(x_A, y_A, theta_A, vx_A, vy_A, omega_A)`
- Pose filtrée Robot5 : `(x_H, y_H, theta_H, vx_H, vy_H, omega_H)`
- Occupancy grid gonflée (costmap)
- État jeu : temps restant, hits, cooldowns

#### 2.5.2 Arbre de comportement (Behavior Tree)

**Structure exemple** :

```
Selector (priorité)
├─ Sequence "SURVIE"
│  ├─ Condition: distance_ennemi < d_safe ?
│  └─ Action: RETREAT (chercher cover, s'éloigner)
│
└─ Selector "COMBAT"
   ├─ Sequence "ATTAQUE"
   │  ├─ Condition: line_of_sight claire ?
   │  ├─ Action: maintenir distance optimale
   │  └─ Action: fire_request = True
   │
   └─ Sequence "REPOSITIONNEMENT"
      ├─ Action: FLANK (contourner obstacles)
      └─ Action: chercher position tir
```

**États possibles** :
- `RETREAT` : Fuite, priorité survie
- `ATTACK` : Ligne de vue claire, tir actif
- `FLANK` : Contournement tactique
- `SEEK_COVER` : Recherche couverture

#### 2.5.3 Path Planning

**Algorithme** : A* sur grille d'occupation

**Processus** :
1. IA décide objectif : `(x_goal, y_goal)`
2. Conversion en cellule grille
3. A* calcule chemin :
   - Départ : cellule Robot4
   - Arrivée : cellule proche de goal (libre)
   - Heuristique : distance euclidienne
   - Coût : costmap (éviter obstacles)
4. Résultat : liste waypoints `[(x_1, y_1), ..., (x_n, y_n)]` en mètres

**Logs** :
```
[AI] State: FLANK, target=(1.85, 1.20)
[AI] Path computed: 32 waypoints
[AI] LOS: FALSE, fire_request: FALSE
```

### 2.6 Contrôle & ROS Bridge

#### 2.6.1 Suivi de trajectoire

**Contrôleur simple** (Pure Pursuit) :

```
Waypoint actuel: (x_wp, y_wp)
Erreur position:
    dx = x_wp - x_A
    dy = y_wp - y_A
    distance = sqrt(dx² + dy²)

Erreur orientation:
    theta_target = atan2(dy, dx)
    dtheta = angle_diff(theta_A, theta_target)

Commandes:
    v = k_v * distance          (vitesse linéaire)
    ω = k_theta * dtheta        (vitesse angulaire)

Contraintes:
    v ∈ [-0.22, 0.22] m/s
    ω ∈ [-2.84, 2.84] rad/s
```

**Cas spéciaux** :
- Waypoint atteint → passer au suivant
- Plus de waypoint → `v=0`, orientation vers ennemi si tir

#### 2.6.2 Envoi ROS Bridge

**Format message** (WebSocket JSON) :
```json
{
  "robot_id": 4,
  "v": 0.15,
  "omega": -0.30
}
```

ROS-bridge reçoit → publie sur `/cmd_vel` (Twist)

**Logs** :
```
[CTRL] Robot4 cmd: v=0.15 m/s, omega=-0.30 rad/s
[ROS] Command sent to bridge
[ROS] Latency: 12ms
```

### 2.7 Visualisation (Pygame + Projecteur)

#### 2.7.1 Transformation Monde → Projecteur

**Paramètres** :
- Arène métrique : `Lx × Ly` mètres
- Zone projetée : `ARENA_PX_WIDTH × ARENA_PX_HEIGHT` pixels
- Marge : `MARGIN` pixels

**Scale** :
```
Sx = ARENA_PX_WIDTH / Lx
Sy = ARENA_PX_HEIGHT / Ly
S = min(Sx, Sy)    # uniforme, garde aspect ratio
```

**Conversion point** :
```
Point monde: (x_W, y_W) en mètres

Pixels projecteur:
    Xp = MARGIN + x_W * S
    Yp = MARGIN + (Ly - y_W) * S    # y=0 en bas
```

#### 2.7.2 Éléments affichés

**1. Fond arène**
- Rectangle zone de jeu
- Bordures, grille optionnelle

**2. Obstacles**
- Rectangles/polygones alignés grille
- Couleur distincte (gris foncé)

**3. Robots**
- Cercles position `(x_W, y_W)` → `(Xp, Yp)`
- Orientation : petit trait dans direction `theta`
- Couleurs : bleu (IA), rouge (humain)

**4. Canon virtuel**
- Ligne de visée (1 mètre) :
  ```
  x_end = x_robot + cos(theta)
  y_end = y_robot + sin(theta)
  ```
- Conversion `(x_end, y_end)` → pixels
- Couleur selon état (blanc normal, jaune si tir imminent)

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
- **État IA** : ATTACK / FLANK / RETREAT
- **Cooldowns** : Barres de progression tirs

**7. Fin de partie**
- Écran overlay : `"WINNER: HUMAN"` ou `"WINNER: AI"`
- Récapitulatif scores
- Option rejouer

**Logs** :
```
[VIS] Frame rendered @30fps
[VIS] LOS=TRUE, Lock-on active
[VIS] HumanHits=2, AIHits=3
```

---

## 🌐 Espaces de coordonnées

Le système utilise 4 repères principaux :

### 1. Camera (C)
- Origine : Centre optique caméra
- Unités : **pixels**
- Coordonnées : `(u, v)`
- Axes : u→droite, v→bas

### 2. Arena Virtual (AV)
- Origine : Coin bas-gauche arène
- Unités : **normalisé [0, 1]**
- Coordonnées : `(x_av, y_av)`
- Usage : Intermédiaire calibration

### 3. World (W)
- Origine : Coin bas-gauche arène
- Unités : **mètres**
- Coordonnées : `(x_W, y_W)`
- Axes : x→droite, y→haut, z→sortant (règle main droite)
- **Principal repère utilisé**

### 4. Projecteur (PROJ)
- Origine : Coin haut-gauche image projetée
- Unités : **pixels projecteur**
- Coordonnées : `(Xp, Yp)`
- Résolution : ex. 1920×1080

### Transformations

```
Camera → AV:    H_C2AV  (homographie, calibration géométrique)
AV → World:     S       (scaling métrique)
Camera → World: H_C2W = S · H_C2AV
World → Proj:   Scaling linéaire + translation (margin)
```

---

## 🏗️ Principes d'architecture

### ✅ Séparation des responsabilités

Chaque module a un rôle clair :
- **Vision** : Capte et détecte
- **Monde** : Représente état métrique
- **Jeu** : Applique règles
- **IA** : Décide stratégie
- **Contrôle** : Exécute mouvements
- **Visualisation** : Affiche

### ✅ Clean Architecture

- `core/` ne dépend **JAMAIS** de :
  - Pygame
  - Caméra
  - Sockets
- `perception/` ne dépend **JAMAIS** de :
  - Logique jeu
  - IA
- `visualization/` ne prend **JAMAIS** de décisions

### ✅ Modularité

**Remplaçable facilement** :
- Changer caméra (ZED, webcam) → toucher uniquement `perception/camera/`
- Changer IA → toucher uniquement `core/ia/`
- Changer pathfinding → toucher uniquement `core/ia/planners/`
- Ajouter robot → configuration, pas code

### ✅ Configuration externalisée

**Tout ce qui peut varier → YAML** :
- Dimensions arène
- Vitesse robots
- Seuils IA
- Durée partie
- IDs ArUco

**Avantage** : Reconfigurer sans recompiler

### ✅ Scalabilité

**Extensions futures faciles** :
- Mode 1v1v1 (3 robots)
- Nouveaux types obstacles
- Power-ups projetés
- Mini-map temps réel
- Multiples stratégies IA switchables
- Enregistrement replay

---

## 🚀 Utilisation

### Première utilisation : Calibration

```bash
# Activer environnement Python
pyenv activate ubuntu

# Lancer wizard calibration
python3 scripts/run_calibration.py
```

**Suivre les instructions** :
1. Définir safe zone
2. Détecter coins projetés (4 ArUco)
3. Placer marker physique connu (ex: 10cm)
4. Cartographier obstacles fixes

→ Génère `config/arena.yaml`

### Lancer une partie

```bash
# Activer environnement
pyenv activate ubuntu

# Démarrer ROS Bridge (terminal séparé)
# roslaunch rosbridge_server rosbridge_websocket.launch

# Lancer le jeu
python3 scripts/run_game.py
```

**Boucle principale 30 FPS** :
- Vision tracking
- IA décisions
- Contrôle robot IA
- Projection visualisation

### Debugging

```bash
# Export debug snapshot (config + grille + état)
python3 scripts/export_debug_data.py

# Capture live depuis caméra (requiert RealSense connectée)
python3 scripts/export_debug_data.py --live

# Export vers répertoire spécifique
python3 scripts/export_debug_data.py --output-dir ~/mon_debug

# Voir logs temps réel
tail -f logs/runtime.log

# Activer affichage debug (config/game.yaml)
debug_mode: true
```

**Contenu export debug** :
- `config/` : Tous les fichiers YAML de configuration
- `occupancy_grid.npy` / `.png` : Grille d'occupation (NumPy + visualisation)
- `game_state.json` : État complet du jeu (poses, scores, IA)
- `manifest.json` : Index des éléments exportés
- `camera_frame.png` / `_annotated.png` : Captures caméra (mode `--live`)
- `aruco_detections.json` : Détections ArUco (mode `--live`)
- `depth_frame.npy` / `_viz.png` : Données profondeur (mode `--live`)
- `*.log` : Copies des fichiers logs

---

## 📊 Logs & Monitoring

### Format logs

Tous les logs suivent le pattern :
```
[MODULE] Message détaillé
```

**Modules** :
- `[CALIB]` : Calibration
- `[VISION]` : Détection ArUco
- `[KALMAN]` : Filtrage
- `[GRID]` : Grille occupation
- `[GAME]` : Arbitre
- `[AI]` : Stratégie IA
- `[CTRL]` : Contrôle
- `[ROS]` : Communication bridge
- `[VIS]` : Visualisation

### Fichiers logs

- `logs/calibration.log` : Historique calibrations
- `logs/runtime.log` : Parties jouées
- `logs/debug.log` : Informations détaillées debug

---

## 🔧 Configuration

### Fichiers clés

#### `config/arena.yaml`
```yaml
projector:
  width: 1920
  height: 1080
  margin: 50

arena:
  width_m: 2.85
  height_m: 1.90

transform:
  H_C2W: [[...], [...], [...]]  # 3x3 matrix
  scale: 1.149

obstacles: [...]  # Liste obstacles fixes
```

#### `config/game.yaml`
```yaml
match_duration_s: 180
human_fire_cooldown_s: 5.0
ai_fire_cooldown_s: 3.0
max_hits: 10
fps: 30
```

#### `config/ia.yaml`
```yaml
safe_distance_m: 0.8
attack_distance_m: 1.5
retreat_threshold_m: 0.5
path_replan_interval_s: 2.0
```

---

## 📚 Dépendances

### Python (requirements.txt)

```
opencv-contrib-python>=4.8.0
pyrealsense2>=2.54.0
pygame>=2.5.0
numpy>=1.24.0
scipy>=1.11.0
pyyaml>=6.0
websocket-client>=1.6.0
```

### Système

- **ROS Noetic** (ou ROS2 Humble)
- **rosbridge_server** pour communication WebSocket
- **RealSense SDK 2.0**

### Hardware

- Intel RealSense D435 (caméra RGB-D)
- Projecteur (recommandé ≥1920×1080)
- 2× Turtlebot Burger avec ArUco markers
- PC Linux (Ubuntu 20.04/22.04 recommandé)

---

## 🎓 Ressources techniques

### Calibration
- Zhang's camera calibration
- Homography estimation (OpenCV docs)
- ArUco marker detection

### IA
- Behavior Trees (article: "Behavior Trees for Robotics")
- A* pathfinding
- Occupancy grid navigation

### Contrôle
- Pure Pursuit controller
- Differential drive kinematics
- ROS navigation stack concepts

### Vision
- Kalman filtering for tracking
- Perspective transformation
- Color-based segmentation

---

## 👥 Contribution

Développé dans le cadre d'un projet de robotique mobile avec vision par ordinateur et IA temps réel.

**Structure respectant** :
- PEP 8 (Python style)
- Clean Architecture
- SOLID principles
- Modularité maximale

---

## 📝 License

Projet académique - À définir selon contexte institutionnel.

---

## 🆘 Troubleshooting

### Import Error: "attempted relative import beyond top-level package"
**Solution** : Vérifier que imports absolus sont utilisés (`from core.world...` et non `from ...core.world...`)

### RuntimeError: No device connected (RealSense)
**Cause** : Caméra non branchée ou drivers manquants  
**Solution** : 
```bash
# Vérifier connexion
rs-enumerate-devices

# Réinstaller drivers si besoin
sudo apt install librealsense2-utils
```

### WebSocket connection failed
**Cause** : ROS Bridge non lancé  
**Solution** :
```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

### IA path planning échoue
**Cause** : Grid mal initialisée ou goal inaccessible  
**Solution** : Activer `debug_draw` pour visualiser costmap

---

**Version** : 1.0  
**Dernière mise à jour** : 2025-12-06
