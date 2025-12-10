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
        
        # TODO: Import and launch UI
        # For now, just print success
        logger.info("Configuration loaded successfully!")
        logger.info("")
        logger.info("Next steps:")
        logger.info("  1. Implement UI renderer")
        logger.info("  2. Integrate controllers")
        logger.info("  3. Connect WebSocket client")
        logger.info("")
        logger.info("Configuration summary:")
        for name, cfg in all_config.items():
            logger.info(f"  - {name}.yaml: {len(cfg)} top-level keys")
        
        # TODO: Launch actual application
        # ui = ControllerUI(all_config)
        # ui.run()
        
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