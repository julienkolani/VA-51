#!/usr/bin/env python3
"""
Test script for ProjectorDisplay - displays ArUco markers without camera.

Usage:
    pyenv activate ubuntu
    python test_projector.py [--width WIDTH] [--height HEIGHT]
    
    Examples:
        python test_projector.py                    # Use default 1024x768
        python test_projector.py --width 1280 --height 720
        python test_projector.py --width 1920 --height 1080
    
Controls:
    - F11: Toggle fullscreen
    - ESC: Exit fullscreen (or quit if windowed)
    - Space: Switch between marker display and white screen
    - Q: Quit
"""

import sys
import argparse
from pathlib import Path

# Add tank_project to path
sys.path.insert(0, str(Path(__file__).parent))

from tank_project.perception.calibration.projector_display import ProjectorDisplay
import time


def main():
    """Test projector display without camera."""
    
    # Parse arguments
    parser = argparse.ArgumentParser(description='Test ArUco marker projection')
    parser.add_argument('--width', type=int, default=1024, 
                        help='Projector width in pixels (default: 1024)')
    parser.add_argument('--height', type=int, default=768, 
                        help='Projector height in pixels (default: 768)')
    args = parser.parse_args()
    
    print("=" * 60)
    print("  TANK ARENA - Projector Display Test")
    print("=" * 60)
    print()
    
    # Detect available displays
    import pygame
    pygame.init()
    displays = pygame.display.get_desktop_sizes()
    print(f"Detected {len(displays)} display(s):")
    for i, (w, h) in enumerate(displays):
        name = "Primary" if i == 0 else f"Display {i}"
        print(f"  {name}: {w}x{h} px")
    print()
    
    print(f"Using resolution: {args.width}x{args.height} px")
    print()
    
    print("Controls:")
    print("  SPACE - Switch display mode")
    print("  Q / ESC - Quit")
    print()
    print("Instructions:")
    print("  1. Drag the window to your projector screen")
    print("  2. Press SPACE to toggle between modes")
    print()
    
    # Create display
    display = ProjectorDisplay(
        width=args.width,
        height=args.height,
        margin=50
    )
    
    try:
        # Start display
        display.start()
        
        # Show corner markers
        display.show_corner_markers(marker_size_px=200)
        
        print()
        print("[TEST] ArUco markers displayed!")
        print("[TEST] Press SPACE to switch modes, Q to quit")
        
        mode = "markers"  # or "white"
        
        # Event loop
        import pygame
        while display.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    display.stop()
                    break
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                        display.stop()
                        break
                    elif event.key == pygame.K_SPACE:
                        # Toggle display mode
                        if mode == "markers":
                            mode = "white"
                            display.show_white_screen()
                            print("[TEST] Switched to white screen mode")
                        else:
                            mode = "markers"
                            display.show_corner_markers(marker_size_px=200)
                            print("[TEST] Switched to marker mode")
            
            # Small delay to prevent CPU spinning
            time.sleep(0.01)
        
        print("[TEST] Display closed")
        
    except Exception as e:
        print(f"[TEST] ERROR: {e}")
        import traceback
        traceback.print_exc()
    finally:
        display.stop()


if __name__ == "__main__":
    main()
