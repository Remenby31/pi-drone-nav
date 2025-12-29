"""
Interface modules for Pi Drone Navigation

Note: CLI has been moved to src.cli module.
REST API has been moved to src.server module.
This file is kept for backward compatibility.
"""

from .rest_api import create_api_server

__all__ = ['create_api_server']
