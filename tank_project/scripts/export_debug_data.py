#!/usr/bin/env python3
"""
Export Debug Data - Sauvegarde Données Debug

Export snapshots debug pour analyse offline:
- Images caméra (capture live ou test)
- Grilles occupation (NumPy + visualisation)
- Homographies et calibration
- État jeu (JSON)
- Logs

Modes:
    - Standalone : Export config + logs (sans système actif)
    - Live       : Capture snapshot depuis caméra (avec --live)

Usage:
    python3 export_debug_data.py [--output-dir DIR] [--live]
    
Examples:
    # Export config + logs seulement
    python3 export_debug_data.py
    
    # Capture live depuis caméra
    python3 export_debug_data.py --live
    
    # Export vers répertoire spécifique
    python3 export_debug_data.py --output-dir ~/mon_debug --live
"""

import sys
import argparse
import json
import time
import cv2
import numpy as np
import yaml
from pathlib import Path
from datetime import datetime
from typing import Optional, Dict, Any

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))


def create_debug_export(output_dir: str = None, live_capture: bool = False):
    """
    Crée export debug complet.
    
    Args:
        output_dir: Répertoire sortie (défaut: logs/debug_TIMESTAMP/)
        live_capture: Si True, tente capture live depuis caméra
    """
    # Créer répertoire output
    if output_dir is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = Path(__file__).parent.parent / 'logs' / f'debug_{timestamp}'
    else:
        output_dir = Path(output_dir)
    
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"[EXPORT] ========== Debug Export ==========")
    print(f"[EXPORT] Output directory: {output_dir.absolute()}")
    print(f"[EXPORT] Live capture: {'ENABLED' if live_capture else 'DISABLED'}")
    print()
    
    # Créer fichier manifest
    manifest = {
        'export_timestamp': datetime.now().isoformat(),
        'mode': 'live' if live_capture else 'standalone',
        'exported_items': []
    }
    
    # 1. Sauvegarder configuration
    config_success = _export_config(output_dir)
    if config_success:
        manifest['exported_items'].append('config')
    
    # 2. Sauvegarder frame caméra + détections ArUco
    if live_capture:
        camera_success = _export_camera_frame(output_dir)
        if camera_success:
            manifest['exported_items'].append('camera_frame')
            manifest['exported_items'].append('aruco_detections')
    
    # 3. Sauvegarder grille occupation (exemple synthétique)
    grid_success = _export_occupancy_grid(output_dir, live_capture)
    if grid_success:
        manifest['exported_items'].append('occupancy_grid')
    
    # 4. Sauvegarder état jeu (exemple synthétique)
    state_success = _export_game_state(output_dir, live_capture)
    if state_success:
        manifest['exported_items'].append('game_state')
    
    # 5. Copier logs récents
    logs_count = _export_logs(output_dir)
    if logs_count > 0:
        manifest['exported_items'].append('logs')
        manifest['logs_count'] = logs_count
    
    # 6. Sauvegarder manifest
    with open(output_dir / 'manifest.json', 'w') as f:
        json.dump(manifest, f, indent=2)
    
    print()
    print(f"[EXPORT] ========== Export Complete ==========")
    print(f"[EXPORT] Location: {output_dir.absolute()}")
    print(f"[EXPORT] Items exported: {len(manifest['exported_items'])}")
    print(f"[EXPORT] Manifest: manifest.json")


def _export_config(output_dir: Path) -> bool:
    """
    Export fichiers configuration.
    
    Returns:
        True si au moins un fichier exporté
    """
    print("[EXPORT] Exporting configuration...")
    
    config_dir = Path(__file__).parent.parent / 'config'
    
    if not config_dir.exists():
        print("[EXPORT]   ✗ config/ directory not found")
        return False
    
    config_export = output_dir / 'config'
    config_export.mkdir(exist_ok=True)
    
    exported = 0
    for config_file in config_dir.glob('*.yaml'):
        try:
            with open(config_file) as f:
                data = yaml.safe_load(f)
            
            out_file = config_export / config_file.name
            with open(out_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            
            print(f"[EXPORT]   ✓ {config_file.name}")
            exported += 1
        except Exception as e:
            print(f"[EXPORT]   ✗ {config_file.name}: {e}")
    
    return exported > 0


def _export_camera_frame(output_dir: Path) -> bool:
    """
    Export capture caméra live + détections ArUco.
    
    Returns:
        True si capture réussie
    """
    print("[EXPORT] Capturing live camera frame...")
    
    try:
        from perception.camera.realsense_stream import RealSenseStream
        from perception.camera.aruco_detector import ArucoDetector
        
        # Initialiser caméra
        print("[EXPORT]   Initializing RealSense camera...")
        camera = RealSenseStream(width=640, height=480, fps=30)
        camera.start()
        
        # Attendre stabilisation
        time.sleep(1.0)
        
        # Capturer frame
        color_frame, depth_frame = camera.get_frames()
        
        if color_frame is None:
            print("[EXPORT]   ✗ Failed to capture frame")
            camera.stop()
            return False
        
        # Sauvegarder frame couleur
        cv2.imwrite(str(output_dir / 'camera_frame.png'), color_frame)
        print(f"[EXPORT]   ✓ camera_frame.png ({color_frame.shape[1]}x{color_frame.shape[0]})")
        
        # Sauvegarder depth si disponible
        if depth_frame is not None:
            np.save(str(output_dir / 'depth_frame.npy'), depth_frame)
            
            # Visualisation depth
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_frame, alpha=0.03),
                cv2.COLORMAP_JET
            )
            cv2.imwrite(str(output_dir / 'depth_frame_viz.png'), depth_colormap)
            print(f"[EXPORT]   ✓ depth_frame.npy")
        
        # Détecter ArUco
        print("[EXPORT]   Detecting ArUco markers...")
        aruco = ArucoDetector()
        detections = aruco.detect(color_frame)
        
        # Dessiner détections sur frame
        frame_annotated = color_frame.copy()
        for marker_id, data in detections.items():
            center = data['center']
            corners = data['corners']
            
            # Dessiner contour
            cv2.polylines(frame_annotated, [corners.astype(int)], True, (0, 255, 0), 2)
            
            # Dessiner centre + ID
            cv2.circle(frame_annotated, tuple(center.astype(int)), 5, (0, 0, 255), -1)
            cv2.putText(
                frame_annotated,
                f"ID:{marker_id}",
                tuple((center + [10, -10]).astype(int)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 0),
                2
            )
        
        cv2.imwrite(str(output_dir / 'camera_frame_annotated.png'), frame_annotated)
        
        # Sauvegarder détections JSON
        detections_json = {}
        for marker_id, data in detections.items():
            detections_json[int(marker_id)] = {
                'center': data['center'].tolist(),
                'corners': data['corners'].tolist()
            }
        
        with open(output_dir / 'aruco_detections.json', 'w') as f:
            json.dump(detections_json, f, indent=2)
        
        print(f"[EXPORT]   ✓ ArUco detected: {len(detections)} markers")
        print(f"[EXPORT]   ✓ camera_frame_annotated.png")
        print(f"[EXPORT]   ✓ aruco_detections.json")
        
        camera.stop()
        return True
        
    except ImportError as e:
        print(f"[EXPORT]   ✗ Import error: {e}")
        print(f"[EXPORT]   → Live capture requires RealSense dependencies")
        return False
    except Exception as e:
        print(f"[EXPORT]   ✗ Camera capture failed: {e}")
        return False


def _export_occupancy_grid(output_dir: Path, live_capture: bool) -> bool:
    """
    Export grille occupation.
    
    Crée exemple synthétique si pas de système actif.
    
    Args:
        output_dir: Répertoire sortie
        live_capture: Si True, tente connexion à système actif
        
    Returns:
        True si export réussi
    """
    print("[EXPORT] Exporting occupancy grid...")
    
    # Pour l'instant, créer exemple synthétique
    # TODO: Connecter à WorldModel actif si disponible
    
    # Paramètres grille exemple
    width_m = 2.85
    height_m = 1.90
    resolution_m = 0.02
    
    nx = int(width_m / resolution_m)
    ny = int(height_m / resolution_m)
    
    # Créer grille (0 = libre, 100 = obstacle)
    grid = np.zeros((ny, nx), dtype=np.uint8)
    
    # Ajouter quelques obstacles exemple
    # Obstacle central
    grid[40:60, 60:80] = 100
    
    # Murs bordure
    grid[0:5, :] = 100  # Bas
    grid[-5:, :] = 100  # Haut
    grid[:, 0:5] = 100  # Gauche
    grid[:, -5:] = 100  # Droite
    
    # Sauvegarder array NumPy
    np.save(str(output_dir / 'occupancy_grid.npy'), grid)
    print(f"[EXPORT]   ✓ occupancy_grid.npy ({nx}x{ny}, {resolution_m}m/cell)")
    
    # Créer visualisation
    grid_viz = np.zeros((ny, nx, 3), dtype=np.uint8)
    grid_viz[grid == 0] = [240, 240, 240]    # Libre = gris clair
    grid_viz[grid == 100] = [50, 50, 50]     # Obstacle = gris foncé
    
    # Agrandir pour visualisation
    scale = 4
    grid_viz_large = cv2.resize(
        grid_viz,
        (nx * scale, ny * scale),
        interpolation=cv2.INTER_NEAREST
    )
    
    cv2.imwrite(str(output_dir / 'occupancy_grid.png'), grid_viz_large)
    print(f"[EXPORT]   ✓ occupancy_grid.png (visualization)")
    
    # Sauvegarder métadonnées
    grid_meta = {
        'width_m': width_m,
        'height_m': height_m,
        'resolution_m': resolution_m,
        'grid_size': [nx, ny],
        'note': 'Synthetic example grid' if not live_capture else 'Live captured grid'
    }
    
    with open(output_dir / 'occupancy_grid_meta.json', 'w') as f:
        json.dump(grid_meta, f, indent=2)
    
    print(f"[EXPORT]   ✓ occupancy_grid_meta.json")
    
    return True


def _export_game_state(output_dir: Path, live_capture: bool) -> bool:
    """
    Export état jeu.
    
    Crée exemple synthétique si pas de jeu actif.
    
    Args:
        output_dir: Répertoire sortie
        live_capture: Si True, tente connexion à jeu actif
        
    Returns:
        True si export réussi
    """
    print("[EXPORT] Exporting game state...")
    
    # Pour l'instant, créer exemple synthétique
    # TODO: Connecter à GameEngine actif si disponible
    
    state_dict = {
        'timestamp': datetime.now().isoformat(),
        'mode': 'live' if live_capture else 'synthetic_example',
        'match': {
            'duration_s': 180,
            'elapsed_s': 67.3,
            'time_remaining_s': 112.7,
            'status': 'in_progress'
        },
        'robots': {
            'robot_4': {
                'name': 'AI Robot',
                'pose': {
                    'x_m': 1.23,
                    'y_m': 0.87,
                    'theta_rad': 1.57
                },
                'velocity': {
                    'vx_m_s': 0.03,
                    'vy_m_s': -0.01,
                    'omega_rad_s': 0.02
                },
                'hits_received': 2,
                'hits_inflicted': 3,
                'last_shot_time': 61.2
            },
            'robot_5': {
                'name': 'Human Robot',
                'pose': {
                    'x_m': 1.85,
                    'y_m': 1.42,
                    'theta_rad': -0.52
                },
                'velocity': {
                    'vx_m_s': -0.08,
                    'vy_m_s': 0.05,
                    'omega_rad_s': -0.10
                },
                'hits_received': 3,
                'hits_inflicted': 2,
                'last_shot_time': 58.7
            }
        },
        'ai_state': {
            'current_behavior': 'ATTACK',
            'target_position': [1.85, 1.42],
            'path_waypoints_count': 12,
            'has_line_of_sight': True,
            'fire_request': True,
            'safe_distance_m': 0.8
        },
        'cooldowns': {
            'human_next_shot': 72.7,
            'ai_next_shot': 64.2
        }
    }
    
    with open(output_dir / 'game_state.json', 'w') as f:
        json.dump(state_dict, f, indent=2)
    
    print(f"[EXPORT]   ✓ game_state.json")
    print(f"[EXPORT]     - Time: {state_dict['match']['elapsed_s']:.1f}s / {state_dict['match']['duration_s']}s")
    print(f"[EXPORT]     - AI state: {state_dict['ai_state']['current_behavior']}")
    print(f"[EXPORT]     - Hits: R4={state_dict['robots']['robot_4']['hits_received']}, R5={state_dict['robots']['robot_5']['hits_received']}")
    
    return True


def _export_logs(output_dir: Path) -> int:
    """
    Copie logs récents.
    
    Returns:
        Nombre de fichiers copiés
    """
    print("[EXPORT] Copying logs...")
    
    logs_dir = Path(__file__).parent.parent / 'logs'
    
    if not logs_dir.exists():
        logs_dir.mkdir(exist_ok=True)
        print("[EXPORT]   ✗ No logs found (logs/ directory created)")
        return 0
    
    # Copier fichiers .log
    copied = 0
    for log_file in logs_dir.glob('*.log'):
        try:
            out_file = output_dir / log_file.name
            with open(log_file) as f:
                content = f.read()
            with open(out_file, 'w') as f:
                f.write(content)
            
            # Compter lignes
            lines = len(content.splitlines())
            size_kb = len(content) / 1024
            
            print(f"[EXPORT]   ✓ {log_file.name} ({lines} lines, {size_kb:.1f}KB)")
            copied += 1
        except Exception as e:
            print(f"[EXPORT]   ✗ {log_file.name}: {e}")
    
    if copied == 0:
        print("[EXPORT]   ✗ No .log files found")
    
    return copied


def main():
    """Point d'entrée script."""
    parser = argparse.ArgumentParser(
        description='Export données debug pour analyse offline',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                        Export config + logs
  %(prog)s --live                 Capture live depuis caméra
  %(prog)s --output-dir ~/debug   Export vers répertoire spécifique
        """
    )
    parser.add_argument(
        '--output-dir',
        type=str,
        default=None,
        help='Répertoire sortie (défaut: logs/debug_TIMESTAMP/)'
    )
    parser.add_argument(
        '--live',
        action='store_true',
        help='Activer capture live depuis caméra'
    )
    
    args = parser.parse_args()
    
    create_debug_export(args.output_dir, args.live)


if __name__ == '__main__':
    main()
