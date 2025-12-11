#!/usr/bin/env python3
"""
TurtleBot Controller - Main Entry Point

Professional Pygame-based interface for manual robot control via WebSocket.
"""

import sys
import logging
from pathlib import Path

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='[%(levelname)s] %(name)s: %(message)s'
)

logger = logging.getLogger(__name__)


def main():
    """Main entry point."""
    try:
        # Add current directory to path for imports
        sys.path.insert(0, str(Path(__file__).parent))
        
        # Import after path setup
        from utils.config_loader import ConfigLoader
        
        logger.info("=" * 60)
        logger.info(" TurtleBot Controller")
        logger.info("=" * 60)
        
        # Load all configurations
        logger.info("Loading configuration...")
        config_loader = ConfigLoader()
        
        try:
            all_config = config_loader.load_all()
            logger.info(f"Loaded {len(all_config)} configuration files")
        except Exception as e:
            logger.error(f"Failed to load configuration: {e}")
            logger.error("Please check config/ directory contains valid YAML files")
            return 1
        
        # Import and launch UI
        from integrated_ui import IntegratedUI
        
        logger.info("Configuration loaded successfully!")
        logger.info("")
        logger.info("Configuration summary:")
        for name, cfg in all_config.items():
            logger.info("  - %s.yaml: %d top-level keys", name, len(cfg))
        
        logger.info("")
        logger.info("Launching Integrated UI...")
        
        # Get UI configuration
        ui_cfg = all_config.get('ui', {})
        network_cfg = all_config.get('network', {})
        
        # Build WebSocket URI from config (network.yaml has 'uri' directly)
        ws_cfg = network_cfg.get('websocket', {})
        ws_uri = ws_cfg.get('uri', 'ws://localhost:8765')
        
        # Create and launch UI
        ui = IntegratedUI(
            width=ui_cfg.get('default_width', 1400),
            height=ui_cfg.get('default_height', 900),
            ws_uri=ws_uri
        )
        ui.run()
        
        logger.info("UI closed normally")
        return 0
        
    except KeyboardInterrupt:
        logger.info("Interrupted by user (Ctrl+C)")
        return 0
        
    except Exception as e:
        logger.error(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())