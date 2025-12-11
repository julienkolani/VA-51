#!/bin/bash
# =============================================================================
# VA-51 Tank Arena - Unified Launch Script
# =============================================================================
# Lance le Safety Bridge ROS2 et le jeu Tank Arena en une seule commande.
#
# Usage:
#   ./launch_game.sh           # Lance bridge + jeu
#   ./launch_game.sh --bridge  # Lance seulement le bridge
#   ./launch_game.sh --game    # Lance seulement le jeu
#   ./launch_game.sh --help    # Affiche l'aide
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="$SCRIPT_DIR/.."

# Couleurs
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_banner() {
    echo -e "${BLUE}"
    echo "╔══════════════════════════════════════════════════════════════╗"
    echo "║              VA-51 TANK ARENA - LAUNCHER                     ║"
    echo "╚══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --bridge    Lance seulement le Safety Bridge ROS2"
    echo "  --game      Lance seulement le jeu Tank Arena"
    echo "  --control   Lance seulement l'interface de contrôle manuel"
    echo "  --help      Affiche cette aide"
    echo ""
    echo "Sans option: lance le bridge puis le jeu automatiquement."
}

check_ros2() {
    if ! command -v ros2 &> /dev/null; then
        echo -e "${RED}[ERREUR] ROS2 n'est pas installé ou sourcé.${NC}"
        echo "Exécutez: source /opt/ros/humble/setup.bash"
        exit 1
    fi
}

source_workspace() {
    if [ -f "$ROS2_WS/install/setup.bash" ]; then
        echo -e "${YELLOW}[INFO] Source du workspace ROS2...${NC}"
        source "$ROS2_WS/install/setup.bash"
    else
        echo -e "${YELLOW}[WARN] Workspace non buildé. Build en cours...${NC}"
        cd "$ROS2_WS"
        colcon build --packages-select turtlebot_game
        source "$ROS2_WS/install/setup.bash"
    fi
}

launch_bridge() {
    echo -e "${GREEN}[BRIDGE] Lancement du Safety Bridge...${NC}"
    echo -e "${BLUE}         Port WebSocket: 8765${NC}"
    ros2 run turtlebot_game safety_bridge &
    BRIDGE_PID=$!
    echo -e "${GREEN}[BRIDGE] PID: $BRIDGE_PID${NC}"
    
    # Attendre que le bridge soit prêt
    sleep 3
    echo -e "${GREEN}[BRIDGE] Ready!${NC}"
}

launch_game() {
    echo -e "${GREEN}[GAME] Lancement de Tank Arena...${NC}"
    cd "$SCRIPT_DIR/tank_project"
    python3 scripts/run_game.py
}

launch_control() {
    echo -e "${GREEN}[CONTROL] Lancement de l'interface de contrôle...${NC}"
    cd "$SCRIPT_DIR/control_turtlebot"
    python3 main.py
}

cleanup() {
    echo ""
    echo -e "${YELLOW}[CLEANUP] Arrêt des processus...${NC}"
    if [ ! -z "$BRIDGE_PID" ]; then
        kill $BRIDGE_PID 2>/dev/null || true
        echo -e "${GREEN}[CLEANUP] Bridge arrêté${NC}"
    fi
    exit 0
}

# Trap Ctrl+C
trap cleanup SIGINT SIGTERM

# Main
print_banner

case "${1:-}" in
    --help|-h)
        print_help
        exit 0
        ;;
    --bridge)
        check_ros2
        source_workspace
        launch_bridge
        echo -e "${GREEN}[BRIDGE] En attente de connexions... (Ctrl+C pour arrêter)${NC}"
        wait $BRIDGE_PID
        ;;
    --game)
        launch_game
        ;;
    --control)
        launch_control
        ;;
    "")
        # Full launch: bridge + game
        check_ros2
        source_workspace
        launch_bridge
        launch_game
        cleanup
        ;;
    *)
        echo -e "${RED}[ERREUR] Option inconnue: $1${NC}"
        print_help
        exit 1
        ;;
esac
