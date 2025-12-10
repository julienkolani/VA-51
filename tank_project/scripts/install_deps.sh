#!/bin/bash
# Script pour installer les dépendances dans l'environnement pyenv ubuntu

echo "Installation des dépendances Python pour tank_project..."
echo "Environnement: pyenv ubuntu"
echo ""

# S'assurer d'être dans le bon répertoire
cd /home/julien/ros2_ws/src/VA50/tank_project

# Installer les dépendances
pip3 install -r requirements.txt

echo ""
echo "Installation terminée!"
echo "Vous pouvez maintenant lancer la calibration avec:"
echo "  python -m tank_project.main calibration"
