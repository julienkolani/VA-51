#!/bin/bash
# TurtleBot SSH Connection and Launch Helper
# 
# Usage:
#   ./connect_turtlebot.sh [IP_ADDRESS]
#   
# Default IP: 192.168.50.1

set -e

# Configuration
TURTLEBOT_IP="${1:-192.168.50.1}"
TURTLEBOT_USER="turtlebot"
ROS_DOMAIN_ID="30"

echo "================================================"
echo " TurtleBot Connection Helper"
echo "================================================"
echo "Connecting to: ${TURTLEBOT_USER}@${TURTLEBOT_IP}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "================================================"
echo ""

# Test connectivity
echo "[1/3] Testing connectivity..."
if ! ping -c 1 -W 2 "${TURTLEBOT_IP}" > /dev/null 2>&1; then
    echo "ERROR: Cannot reach TurtleBot at ${TURTLEBOT_IP}"
    echo "Please check:"
    echo "  - TurtleBot is powered on"
    echo "  - WiFi connection is active"
    echo "  - IP address is correct"
    exit 1
fi
echo "  OK - TurtleBot is reachable"
echo ""

# Connect and launch
echo "[2/3] Connecting via SSH..."
echo "  (Password required for user '${TURTLEBOT_USER}')"
echo ""

# SSH with bringup launch
ssh -t ${TURTLEBOT_USER}@${TURTLEBOT_IP} << ENDSSH
    echo "================================================"
    echo " Connected to TurtleBot"
    echo "================================================"
    
    # Source ROS2 environment
    echo "[3/3] Setting up ROS2 environment..."
    source /opt/ros/humble/setup.bash
    
    # Set ROS_DOMAIN_ID
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
    
    echo "  ROS2 sourced: /opt/ros/humble/setup.bash"
    echo "  ROS_DOMAIN_ID set to: \${ROS_DOMAIN_ID}"
    echo ""
    
    # Launch TurtleBot bringup
    echo "Launching TurtleBot3 bringup..."
    echo "  (Press Ctrl+C to stop)"
    echo "================================================"
    echo ""
    
    ros2 launch turtlebot3_bringup robot.launch.py
ENDSSH

echo ""
echo "================================================"
echo " Disconnected from TurtleBot"
echo "================================================"
