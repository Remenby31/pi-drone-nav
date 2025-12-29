"""
Mission Store

Handles persistent storage of missions as JSON files.
"""

import json
import logging
import uuid
from pathlib import Path
from typing import Dict, List, Optional

from .models import Mission, ValidationError

logger = logging.getLogger(__name__)


class MissionStore:
    """
    Persistent storage for missions

    Stores missions as JSON files in a directory.
    Each file is named with the mission's UUID.
    """

    def __init__(self, missions_dir: str = "~/missions"):
        """
        Initialize mission store

        Args:
            missions_dir: Directory to store mission files
        """
        self.missions_dir = Path(missions_dir).expanduser()
        self._ensure_directory()

        # Cache of loaded missions (uuid -> Mission)
        self._cache: Dict[str, Mission] = {}

    def _ensure_directory(self):
        """Create missions directory if it doesn't exist"""
        self.missions_dir.mkdir(parents=True, exist_ok=True)
        logger.info(f"Mission store initialized at {self.missions_dir}")

    def _get_mission_path(self, mission_uuid: str) -> Path:
        """Get file path for a mission UUID"""
        return self.missions_dir / f"{mission_uuid}.json"

    def create(self, data: Dict) -> Mission:
        """
        Create a new mission from data

        Args:
            data: Mission data dictionary (without UUID)

        Returns:
            Created Mission instance

        Raises:
            ValidationError: If mission data is invalid
        """
        # Generate UUID
        mission_uuid = str(uuid.uuid4())

        # Parse and validate
        mission = Mission.from_dict(data, mission_uuid=mission_uuid)

        # Save to file
        self._save_to_file(mission)

        # Add to cache
        self._cache[mission_uuid] = mission

        logger.info(f"Created mission '{mission.name}' ({mission_uuid})")
        return mission

    def get(self, mission_uuid: str) -> Optional[Mission]:
        """
        Get a mission by UUID

        Args:
            mission_uuid: Mission UUID

        Returns:
            Mission instance or None if not found
        """
        # Check cache first
        if mission_uuid in self._cache:
            return self._cache[mission_uuid]

        # Try to load from file
        mission_path = self._get_mission_path(mission_uuid)
        if not mission_path.exists():
            return None

        try:
            mission = self._load_from_file(mission_path, mission_uuid)
            self._cache[mission_uuid] = mission
            return mission
        except (json.JSONDecodeError, ValidationError) as e:
            logger.error(f"Failed to load mission {mission_uuid}: {e}")
            return None

    def list_all(self) -> List[Dict]:
        """
        List all stored missions

        Returns:
            List of mission summaries
        """
        missions = []

        for path in self.missions_dir.glob("*.json"):
            mission_uuid = path.stem

            # Try to get from cache or load
            mission = self.get(mission_uuid)
            if mission:
                missions.append(mission.get_summary())

        return missions

    def delete(self, mission_uuid: str) -> bool:
        """
        Delete a mission

        Args:
            mission_uuid: Mission UUID

        Returns:
            True if deleted, False if not found
        """
        mission_path = self._get_mission_path(mission_uuid)

        if not mission_path.exists():
            return False

        # Remove from cache
        if mission_uuid in self._cache:
            del self._cache[mission_uuid]

        # Delete file
        mission_path.unlink()
        logger.info(f"Deleted mission {mission_uuid}")

        return True

    def update(self, mission_uuid: str, data: Dict) -> Optional[Mission]:
        """
        Update an existing mission

        Args:
            mission_uuid: Mission UUID
            data: New mission data

        Returns:
            Updated Mission instance or None if not found

        Raises:
            ValidationError: If new data is invalid
        """
        if not self._get_mission_path(mission_uuid).exists():
            return None

        # Parse and validate with existing UUID
        mission = Mission.from_dict(data, mission_uuid=mission_uuid)

        # Save to file
        self._save_to_file(mission)

        # Update cache
        self._cache[mission_uuid] = mission

        logger.info(f"Updated mission '{mission.name}' ({mission_uuid})")
        return mission

    def _save_to_file(self, mission: Mission):
        """Save mission to JSON file"""
        mission_path = self._get_mission_path(mission.uuid)

        with open(mission_path, "w") as f:
            json.dump(mission.to_dict(), f, indent=2)

    def _load_from_file(self, path: Path, mission_uuid: str) -> Mission:
        """Load mission from JSON file"""
        with open(path, "r") as f:
            data = json.load(f)

        return Mission.from_dict(data, mission_uuid=mission_uuid)

    def clear_cache(self):
        """Clear the mission cache"""
        self._cache.clear()

    def get_count(self) -> int:
        """Get total number of stored missions"""
        return len(list(self.missions_dir.glob("*.json")))
