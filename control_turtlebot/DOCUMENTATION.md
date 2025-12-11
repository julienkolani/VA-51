# Control TurtleBot - Documentation

Interface de contrôle manuel pour piloter un robot TurtleBot via WebSocket.

## Architecture

```mermaid
flowchart TB
    subgraph INPUT["Entrées"]
        KB[Clavier<br/>WASD/Flèches]
        PS3[Manette PS3]
    end

    subgraph UI["IntegratedUI"]
        direction TB
        LOOP[Boucle Pygame]
        VR[VisualRobot]
        STATS[Panneau Stats]
    end

    subgraph WS["WebSocketClient"]
        QUEUE[Queue Envoi]
        RECV[Réception Async]
    end

    subgraph OUTPUT["Sortie"]
        ROS[Safety Bridge<br/>Port 8765]
    end

    KB --> LOOP
    PS3 --> LOOP
    LOOP --> QUEUE
    LOOP --> VR
    LOOP --> STATS
    QUEUE --> ROS
    ROS --> RECV
    RECV --> STATS
```

## Flux de Données

```mermaid
sequenceDiagram
    participant USER as Utilisateur
    participant UI as IntegratedUI
    participant WS as WebSocketClient
    participant BRIDGE as Safety Bridge

    USER->>UI: Input (clavier/manette)
    UI->>UI: Calcul vitesses (v, ω)
    UI->>WS: send_cmd_vel(linear_x, angular_z)
    WS->>BRIDGE: {"type": "cmd_vel", "linear_x": v, "angular_z": ω}
    BRIDGE->>WS: {"type": "cmd_accepted", ...}
    WS->>UI: Mise à jour stats
    UI->>UI: Rendu frame
```

## Protocole WebSocket

Le client envoie des messages JSON au Safety Bridge :

| Type             | Description      | Payload                 |
| ---------------- | ---------------- | ----------------------- |
| `cmd_vel`        | Commande vitesse | `linear_x`, `angular_z` |
| `emergency_stop` | Arrêt d'urgence  | -                       |
| `get_status`     | Demande status   | -                       |
| `ping`           | Test connexion   | `timestamp`             |

Réponses du serveur :

| Type           | Description                   |
| -------------- | ----------------------------- |
| `connected`    | Message d'accueil avec config |
| `cmd_accepted` | Confirmation commande         |
| `cmd_rejected` | Commande refusée              |
| `pong`         | Réponse ping                  |

## Configuration

### network.yaml
```yaml
websocket:
  uri: "ws://localhost:8765"
  reconnect_delay_s: 2.0

command:
  frequency_hz: 30
  k_linear: 6.0   # Amplification linéaire
  k_angular: 4.0  # Amplification angulaire
```

### ui.yaml
Paramètres d'interface : dimensions, couleurs, polices.

### controls.yaml
Mapping des touches clavier et boutons manette.

## Structure des Fichiers

```
control_turtlebot/
├── main.py                 # Point d'entrée
├── integrated_ui.py        # Interface Pygame principale
├── websocket_client.py     # Client WebSocket async
├── keyboard_controller.py  # Contrôle clavier
├── ps3_controller.py       # Contrôle manette PS3
├── visual_robot.py         # Représentation visuelle robot
├── config.py              # Constantes et thème
└── config/
    ├── network.yaml       # Configuration réseau
    ├── ui.yaml            # Configuration interface
    ├── controls.yaml      # Mapping contrôles
    └── robot.yaml         # Paramètres robot
```

## Lancement

```bash
cd control_turtlebot
python3 main.py
```

Prérequis : Safety Bridge actif sur `ws://localhost:8765`.
