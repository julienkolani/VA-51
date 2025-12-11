# TurtleBot Game

Package ROS2 contenant le Safety Bridge WebSocket pour contrôler le TurtleBot.

## Installation

```bash
cd ~/ros2_ws
colcon build --packages-select turtlebot_game
source install/setup.bash
```

## Lancement

```bash
ros2 run turtlebot_game safety_bridge
```

Ou via le script unifié :
```bash
./launch_game.sh --bridge
```

## Configuration

Paramètres ROS2 :
| Paramètre         | Défaut    | Description          |
| ----------------- | --------- | -------------------- |
| `ws_host`         | `0.0.0.0` | Interface d'écoute   |
| `ws_port`         | `8765`    | Port WebSocket       |
| `max_linear_vel`  | `0.5`     | Vitesse max (m/s)    |
| `max_angular_vel` | `1.5`     | Rotation max (rad/s) |

## Protocole WebSocket

### Messages Entrants

```json
{"type": "cmd_vel", "linear_x": 0.2, "angular_z": 0.5}
{"type": "emergency_stop"}
{"type": "get_status"}
{"type": "ping"}
```

### Messages Sortants

```json
{"type": "connected", "robot": "robot_controler", "config": {...}}
{"type": "cmd_accepted", "linear_x": 0.2, "angular_z": 0.5}
{"type": "pong", "timestamp": "..."}
```

## Structure

```
turtlebot_game/
├── package.xml
├── setup.py
├── launch/
│   └── safety_bridge.launch.py
└── turtlebot_game/
    └── safety_bridge.py     # Nœud ROS2 principal
```

Voir [DOCUMENTATION.md](DOCUMENTATION.md) pour l'architecture complète.
