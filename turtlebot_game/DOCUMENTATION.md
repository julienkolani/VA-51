# TurtleBot Game - Documentation

Pont ROS2 entre les clients WebSocket et le robot TurtleBot physique.

## Architecture

```mermaid
flowchart LR
    subgraph CLIENTS["Clients WebSocket"]
        TANK[Tank Project<br/>IA Robot]
        CTRL[Control TurtleBot<br/>Manuel]
    end

    subgraph BRIDGE["Safety Bridge"]
        WS[Serveur WebSocket<br/>Port 8765]
        LIM[VelocityLimiter]
        ROS[ROS2 Node]
    end

    subgraph ROBOT["TurtleBot"]
        CMD["/cmd_vel Topic"]
    end

    TANK -->|JSON| WS
    CTRL -->|JSON| WS
    WS --> LIM
    LIM --> ROS
    ROS -->|Twist| CMD
```

## Flux de Traitement

```mermaid
sequenceDiagram
    participant CLIENT as Client WebSocket
    participant WS as WebSocket Server
    participant LIM as VelocityLimiter
    participant ROS as ROS2 Publisher
    participant BOT as TurtleBot

    CLIENT->>WS: {"type": "cmd_vel", "linear_x": 0.2, "angular_z": 0.5}
    WS->>WS: Parse JSON
    WS->>LIM: validate_command(0.2, 0.5)
    LIM->>LIM: Clamp aux limites
    LIM-->>WS: (True, 0.2, 0.5, "OK")
    WS->>ROS: Twist(linear.x=0.2, angular.z=0.5)
    ROS->>BOT: Publish /cmd_vel
    WS-->>CLIENT: {"type": "cmd_accepted", ...}
```

## Protocole WebSocket

### Messages Entrants

| Type             | Description      | Champs Requis           |
| ---------------- | ---------------- | ----------------------- |
| `cmd_vel`        | Commande vitesse | `linear_x`, `angular_z` |
| `emergency_stop` | Arrêt immédiat   | -                       |
| `get_status`     | Demande d'état   | -                       |
| `ping`           | Test connexion   | -                       |

### Messages Sortants

| Type                 | Description              |
| -------------------- | ------------------------ |
| `connected`          | Accueil + config limites |
| `cmd_accepted`       | Vitesses appliquées      |
| `emergency_stop_ack` | Confirmation arrêt       |
| `status`             | État du bridge           |
| `pong`               | Réponse ping             |
| `error`              | Erreur traitement        |

## Limites de Vitesse

Le VelocityLimiter applique des contraintes de sécurité :

| Paramètre         | Valeur Par Défaut |
| ----------------- | ----------------- |
| `max_linear_vel`  | 0.5 m/s           |
| `min_linear_vel`  | -0.3 m/s          |
| `max_angular_vel` | ±1.5 rad/s        |

## Paramètres ROS2

```yaml
robot_namespace: 'robot_controler'
ws_host: '0.0.0.0'
ws_port: 8765
max_linear_vel: 0.5
min_linear_vel: -0.3
max_angular_vel: 1.5
```

## Structure des Fichiers

```
turtlebot_game/
├── package.xml
├── setup.py
├── setup.cfg
├── config/
│   └── bridge_params.yaml
├── launch/
│   └── safety_bridge.launch.py
└── turtlebot_game/
    ├── __init__.py
    └── safety_bridge.py      # Nœud ROS2 principal
```

## Lancement

```bash
# Build le package
cd ~/ros2_ws
colcon build --packages-select turtlebot_game

# Source et lancer
source install/setup.bash
ros2 run turtlebot_game safety_bridge
```

Ou via launch file :
```bash
ros2 launch turtlebot_game safety_bridge.launch.py
```

## Intégration

Le Safety Bridge est le point central de communication :

```mermaid
graph TB
    subgraph "Tank Project"
        AI[AIStrategy]
        RBC[ROSBridgeClient]
    end

    subgraph "Control TurtleBot"
        UI[IntegratedUI]
        WSC[WebSocketClient]
    end

    subgraph "TurtleBot Game"
        SB[Safety Bridge<br/>:8765]
    end

    subgraph "Robot"
        TB[TurtleBot<br/>/cmd_vel]
    end

    AI --> RBC
    RBC -->|Port 8765| SB
    UI --> WSC
    WSC -->|Port 8765| SB
    SB --> TB
```

Les deux clients (Tank Project pour l'IA et Control TurtleBot pour le contrôle manuel) communiquent avec le même Safety Bridge qui publie sur le topic `/cmd_vel`.
