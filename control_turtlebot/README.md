# Control TurtleBot

Interface Pygame professionnelle pour contrôle manuel du robot TurtleBot via WebSocket.

## Lancement

```bash
cd control_turtlebot
python3 main.py
```

Ou via le script unifié :
```bash
./launch_game.sh --control
```

## Structure

```
control_turtlebot/
├── main.py               # Point d'entrée
├── integrated_ui.py      # Interface Pygame
├── websocket_client.py   # Client WebSocket async
├── keyboard_controller.py
├── ps3_controller.py
├── visual_robot.py
└── config/
    ├── network.yaml      # URI WebSocket (port 8765)
    ├── ui.yaml           # Interface
    └── controls.yaml     # Mapping touches
```

## Configuration Réseau

```yaml
# config/network.yaml
websocket:
  uri: "ws://localhost:8765"
```

## Contrôles

### Clavier
| Touche | Action                         |
| ------ | ------------------------------ |
| `W/↑`  | Avancer                        |
| `S/↓`  | Reculer                        |
| `A/←`  | Tourner gauche                 |
| `D/→`  | Tourner droite                 |
| `+/-`  | Ajuster vitesse                |
| `M`    | Changer mode (clavier/manette) |
| `ESC`  | Quitter                        |

### Manette PS3
- Stick gauche : Déplacement
- L2/R2 : Vitesse

## Protocole

Envoie au Safety Bridge :
```json
{"type": "cmd_vel", "linear_x": 0.2, "angular_z": 0.5}
```

Voir [DOCUMENTATION.md](DOCUMENTATION.md) pour les détails.
