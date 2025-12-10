#!/usr/bin/env python3
"""
Detect Projector Resolution

This script helps detect the actual resolution of your projector.
It displays available monitors and their resolutions.

Usage:
    pyenv activate ubuntu
    python detect_projector_resolution.py
"""

import pygame
import sys


def main():
    print("=" * 60)
    print("  PROJECTOR RESOLUTION DETECTOR")
    print("=" * 60)
    print()
    
    pygame.init()
    
    # Get all available display sizes
    displays = pygame.display.get_desktop_sizes()
    
    print(f"Detected {len(displays)} display(s):\n")
    
    for i, (width, height) in enumerate(displays):
        display_name = "Primary Monitor" if i == 0 else f"Secondary Display {i}"
        print(f"  Display {i}: {width} x {height} px ({display_name})")
    
    print()
    print("=" * 60)
    print()
    
    # Test: Create a window and show its actual size
    print("Creating a test window...")
    print("1. The window will open on your primary display")
    print("2. Drag it to the projector")
    print("3. Press F11 to go fullscreen")
    print("4. Check if the display info shown matches reality")
    print()
    
    # Create resizable window
    screen = pygame.display.set_mode((800, 600), pygame.RESIZABLE)
    pygame.display.set_caption("Projector Resolution Test - Drag me to projector, press F11")
    
    font_large = pygame.font.Font(None, 48)
    font_small = pygame.font.Font(None, 32)
    
    running = True
    is_fullscreen = False
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_F11:
                    # Toggle fullscreen
                    if is_fullscreen:
                        screen = pygame.display.set_mode((800, 600), pygame.RESIZABLE)
                        is_fullscreen = False
                    else:
                        screen = pygame.display.set_mode((0, 0), pygame.NOFRAME | pygame.FULLSCREEN)
                        is_fullscreen = True
                elif event.key == pygame.K_ESCAPE:
                    if is_fullscreen:
                        screen = pygame.display.set_mode((800, 600), pygame.RESIZABLE)
                        is_fullscreen = False
                    else:
                        running = False
                elif event.key == pygame.K_q:
                    running = False
        
        # Clear screen
        screen.fill((30, 30, 50))
        
        # Get current window size
        width, height = screen.get_size()
        
        # Display info
        mode_text = "FULLSCREEN MODE" if is_fullscreen else "WINDOWED MODE"
        mode_surface = font_large.render(mode_text, True, (255, 255, 100))
        mode_rect = mode_surface.get_rect(center=(width // 2, height // 3))
        screen.blit(mode_surface, mode_rect)
        
        # Display resolution
        res_text = f"Current Resolution: {width} x {height} px"
        res_surface = font_large.render(res_text, True, (255, 255, 255))
        res_rect = res_surface.get_rect(center=(width // 2, height // 2))
        screen.blit(res_surface, res_rect)
        
        # Instructions
        instructions = [
            "F11 - Toggle Fullscreen",
            "ESC - Exit Fullscreen / Quit",
            "Q - Quit"
        ]
        
        y_offset = height // 2 + 80
        for instruction in instructions:
            inst_surface = font_small.render(instruction, True, (200, 200, 200))
            inst_rect = inst_surface.get_rect(center=(width // 2, y_offset))
            screen.blit(inst_surface, inst_rect)
            y_offset += 40
        
        # Note at bottom
        if is_fullscreen:
            note = "THIS IS YOUR PROJECTOR RESOLUTION!"
            note_color = (100, 255, 100)
        else:
            note = "Drag window to projector, then press F11"
            note_color = (255, 200, 100)
        
        note_surface = font_small.render(note, True, note_color)
        note_rect = note_surface.get_rect(center=(width // 2, height - 50))
        screen.blit(note_surface, note_rect)
        
        pygame.display.flip()
    
    pygame.quit()
    
    print()
    print("=" * 60)
    print("NEXT STEPS:")
    print("=" * 60)
    print()
    print("If your projector resolution is NOT 1024x768, you need to update:")
    print()
    print("1. Edit: tank_project/config/arena.yaml")
    print("   Change projector width and height values")
    print()
    print("2. Or run test_projector.py with custom resolution:")
    print("   python test_projector.py --width <WIDTH> --height <HEIGHT>")
    print()


if __name__ == "__main__":
    main()
