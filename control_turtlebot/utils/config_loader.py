"""Configuration loader utility for YAML files."""

import yaml
from pathlib import Path
from typing import Dict, Any
import logging

logger = logging.getLogger(__name__)


class ConfigLoader:
    """Load and manage YAML configuration files."""
    
    def __init__(self, config_dir: Path = None):
        """
        Initialize configuration loader.
        
        Args:
            config_dir: Directory containing config files. If None, uses ./config
        """
        if config_dir is None:
            # Get config dir relative to this file's location
            self.config_dir = Path(__file__).parent.parent / 'config'
        else:
            self.config_dir = Path(config_dir)
        
        if not self.config_dir.exists():
            raise FileNotFoundError(f"Config directory not found: {self.config_dir}")
        
        logger.info(f"Config directory: {self.config_dir}")
    
    def load(self, config_name: str) -> Dict[str, Any]:
        """
        Load a YAML configuration file.
        
        Args:
            config_name: Name of config file (without .yaml extension)
            
        Returns:
            Dictionary containing configuration
            
        Raises:
            FileNotFoundError: If config file doesn't exist
            yaml.YAMLError: If config file is malformed
        """
        config_path = self.config_dir / f"{config_name}.yaml"
        
        if not config_path.exists():
            raise FileNotFoundError(f"Config file not found: {config_path}")
        
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            logger.info(f"Loaded config: {config_name}")
            return config
        except yaml.YAMLError as e:
            logger.error(f"Error parsing {config_name}.yaml: {e}")
            raise
    
    def load_all(self) -> Dict[str, Dict[str, Any]]:
        """
        Load all YAML configuration files from config directory.
        
        Returns:
            Dictionary mapping config names to their contents
        """
        all_configs = {}
        
        for config_file in self.config_dir.glob("*.yaml"):
            config_name = config_file.stem
            try:
                all_configs[config_name] = self.load(config_name)
            except Exception as e:
                logger.warning(f"Failed to load {config_name}: {e}")
        
        return all_configs
    
    def get_value(self, config_name: str, key_path: str, default: Any = None) -> Any:
        """
        Get a specific value from a config using dot notation.
        
        Args:
            config_name: Name of config file
            key_path: Path to value using dot notation (e.g., 'websocket.uri')
            default: Default value if key not found
            
        Returns:
            Value at key_path or default
            
        Example:
            loader.get_value('network', 'websocket.uri')  # Returns 'ws://localhost:8765'
        """
        config = self.load(config_name)
        
        keys = key_path.split('.')
        value = config
        
        for key in keys:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return default
        
        return value
