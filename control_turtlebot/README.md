# TurtleBot Controller

Professional Pygame-based interface for manual control of TurtleBot robots via WebSocket.

## Features

- **Dual Control**: Keyboard and PS3 gamepad support
- **Real-time Monitoring**: WebSocket connection status and latency
- **Visual Simulation**: Real-time robot trajectory visualization
- **Professional UI**: Modern dark theme with status panels  
- **Modular Architecture**: Clean separation of concerns (config, core, UI)
- **Configurable**: YAML-based configuration management

## Quick Start

### Installation

```bash
cd /home/julien/ros2_ws/src/VA50/control_turtlebot
pip3 install -r requirements.txt
```

### Basic Usage

```bash
python3 main.py
```

The controller will:
1. Load configuration from `config/` directory
2. Attempt WebSocket connection to `ws://localhost:8765`
3. Display UI with control panels
4. Accept keyboard or PS3 gamepad input
5. Send velocity commands to robot via WebSocket

## Controls

### Keyboard

| Key | Action |
|-----|--------|
| **Arrow Keys** / **WASD** | Move robot (forward/backward/left/right) |
| **Space** | Emergency stop |
| **+** / **-** | Increase/decrease speed factor |
| **ESC** | Quit application |

### PS3 Gamepad

| Control | Action |
|---------|--------|
| **Left Stick** | Move robot (Y-axis: forward/back, X-axis: turn) |
| **D-Pad** | Override movement |
| **X Button** | Emergency stop |

## Configuration

All configuration is in YAML files under `config/`:

### `config/network.yaml`

WebSocket connection settings:

```yaml
websocket:
  uri: "ws://localhost:8765"  # Change to robot IP
  reconnect_delay_s: 2.0
  ping_interval_s: 3.0
```

### `config/controls.yaml`

Keyboard and gamepad parameters:

```yaml
keyboard:
  max_linear_mps: 3.5
  max_angular_dps: 120
  
ps3:
  deadzone: 0.15
```

### `config/ui.yaml`

UI appearance and behavior:

```yaml
window:
  default_width: 1200
  default_height: 700
  target_fps: 60

theme:
  colors:
    background: [15, 15, 20]
    accent: [0, 180, 220]
```

### `config/robot.yaml`

Robot physical limits:

```yaml
velocity_limits:
  max_linear_mps: 0.22
  max_angular_radps: 2.84
```

## Architecture

```
control_turtlebot/
├── config/              # YAML configuration files
├── core/
│   ├── controllers/     # Input handlers (keyboard, PS3)
│   └── networking/      # WebSocket client
├── ui/                  # Pygame rendering
└── utils/               # Config loader
```

### Key Components

- **BaseController**: Abstract base class for all controllers
- **KeyboardController**: Physics-based keyboard control
- **PS3Controller**: Joystick control with deadzone
- **Theme**: Professional UI styling
- **ConfigLoader**: YAML configuration management

## Connecting to TurtleBot

1. Ensure `turtlebot_game/safety_bridge` is running on the robot or bridge server
2. Update `config/network.yaml` with correct WebSocket URI
3. Launch controller:

```bash
python3 main.py
```

4. Check connection status in UI (top-right panel)

## Troubleshooting

### No WebSocket Connection

- Verify `turtlebot_game/safety_bridge` is running
- Check WebSocket URI in `config/network.yaml`
- Test network connectivity to robot: `ping 192.168.50.1`

### PS3 Controller Not Detected

- Connect gamepad before launching
- Check gamepad with: `ls /dev/input/js*`
- Reconnect during runtime (hot-plug supported)

### High Latency

- Check network quality
- Reduce `ping_interval_s` in `config/network.yaml`
- Move closer to robot WiFi access point

## Development

### Adding New Controller

1. Create new class in `core/controllers/`
2. Inherit from `BaseController`
3. Implement required methods: `update()`, `get_command()`, `emergency_stop()`
4. Add configuration to `config/controls.yaml`

### Customizing UI

Edit `config/ui.yaml` to modify:
- Window size and FPS
- Theme colors
- Font sizes
- Panel layout

## Dependencies

- `pygame` - UI and input handling
- `websockets` - WebSocket client
- `pyyaml` - Configuration loading

## See Also

- [VA50 Main README](../README.md) - Complete project documentation
- [turtlebot_game](../turtlebot_game/) - ROS2 WebSocket bridge
- [tank_project](../tank_project/) - AR tank combat game
