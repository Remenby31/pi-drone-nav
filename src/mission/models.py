"""
Mission models

Defines the mission format v1.0 with action-based structure.
"""

from __future__ import annotations

import uuid
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, Dict, List, Optional, Union


class ValidationError(Exception):
    """Raised when mission validation fails"""
    pass


class ActionType(Enum):
    """Available action types"""
    TAKEOFF = "takeoff"
    GOTO = "goto"
    HOVER = "hover"
    ORIENT = "orient"
    DELAY = "delay"
    PHOTO = "photo"
    LAND = "land"
    RTH = "rth"


@dataclass
class Action(ABC):
    """Base class for all mission actions"""

    @property
    @abstractmethod
    def action_type(self) -> ActionType:
        """Return the action type"""
        pass

    @abstractmethod
    def to_dict(self) -> Dict[str, Any]:
        """Convert action to dictionary"""
        pass

    @classmethod
    @abstractmethod
    def from_dict(cls, data: Dict[str, Any], defaults: 'MissionDefaults') -> 'Action':
        """Create action from dictionary"""
        pass

    def validate(self) -> List[str]:
        """
        Validate action parameters.

        Returns:
            List of validation error messages (empty if valid)
        """
        return []


@dataclass
class TakeoffAction(Action):
    """Take off to specified altitude"""
    alt: float  # meters, relative to ground

    @property
    def action_type(self) -> ActionType:
        return ActionType.TAKEOFF

    def to_dict(self) -> Dict[str, Any]:
        return {"type": "takeoff", "alt": self.alt}

    @classmethod
    def from_dict(cls, data: Dict[str, Any], defaults: 'MissionDefaults') -> 'TakeoffAction':
        alt = data.get("alt", defaults.alt)
        return cls(alt=alt)

    def validate(self) -> List[str]:
        errors = []
        if self.alt <= 0:
            errors.append("takeoff: altitude must be positive")
        if self.alt > 120:
            errors.append("takeoff: altitude exceeds 120m limit")
        return errors


@dataclass
class GotoAction(Action):
    """Fly to GPS position"""
    lat: float
    lon: float
    alt: Optional[float] = None  # None = use default/current
    speed: Optional[float] = None  # None = use default

    @property
    def action_type(self) -> ActionType:
        return ActionType.GOTO

    def to_dict(self) -> Dict[str, Any]:
        d = {"type": "goto", "lat": self.lat, "lon": self.lon}
        if self.alt is not None:
            d["alt"] = self.alt
        if self.speed is not None:
            d["speed"] = self.speed
        return d

    @classmethod
    def from_dict(cls, data: Dict[str, Any], defaults: 'MissionDefaults') -> 'GotoAction':
        return cls(
            lat=data["lat"],
            lon=data["lon"],
            alt=data.get("alt"),  # Keep None if not specified
            speed=data.get("speed"),
        )

    def validate(self) -> List[str]:
        errors = []
        if not (-90 <= self.lat <= 90):
            errors.append(f"goto: invalid latitude {self.lat}")
        if not (-180 <= self.lon <= 180):
            errors.append(f"goto: invalid longitude {self.lon}")
        if self.alt is not None and self.alt < 0:
            errors.append("goto: altitude cannot be negative")
        if self.speed is not None and self.speed <= 0:
            errors.append("goto: speed must be positive")
        return errors


@dataclass
class HoverAction(Action):
    """Hover in place for specified duration"""
    duration: float  # seconds

    @property
    def action_type(self) -> ActionType:
        return ActionType.HOVER

    def to_dict(self) -> Dict[str, Any]:
        return {"type": "hover", "duration": self.duration}

    @classmethod
    def from_dict(cls, data: Dict[str, Any], defaults: 'MissionDefaults') -> 'HoverAction':
        return cls(duration=data["duration"])

    def validate(self) -> List[str]:
        errors = []
        if self.duration <= 0:
            errors.append("hover: duration must be positive")
        if self.duration > 300:
            errors.append("hover: duration exceeds 5 minute limit")
        return errors


@dataclass
class OrientAction(Action):
    """Orient to specified heading (yaw)"""
    heading: float  # degrees, 0-360, 0=North

    @property
    def action_type(self) -> ActionType:
        return ActionType.ORIENT

    def to_dict(self) -> Dict[str, Any]:
        return {"type": "orient", "heading": self.heading}

    @classmethod
    def from_dict(cls, data: Dict[str, Any], defaults: 'MissionDefaults') -> 'OrientAction':
        return cls(heading=data["heading"])

    def validate(self) -> List[str]:
        errors = []
        if not (0 <= self.heading < 360):
            errors.append(f"orient: heading must be 0-360, got {self.heading}")
        return errors


@dataclass
class DelayAction(Action):
    """Wait without any action"""
    duration: float  # seconds

    @property
    def action_type(self) -> ActionType:
        return ActionType.DELAY

    def to_dict(self) -> Dict[str, Any]:
        return {"type": "delay", "duration": self.duration}

    @classmethod
    def from_dict(cls, data: Dict[str, Any], defaults: 'MissionDefaults') -> 'DelayAction':
        return cls(duration=data["duration"])

    def validate(self) -> List[str]:
        errors = []
        if self.duration <= 0:
            errors.append("delay: duration must be positive")
        return errors


@dataclass
class PhotoAction(Action):
    """Trigger camera shutter"""

    @property
    def action_type(self) -> ActionType:
        return ActionType.PHOTO

    def to_dict(self) -> Dict[str, Any]:
        return {"type": "photo"}

    @classmethod
    def from_dict(cls, data: Dict[str, Any], defaults: 'MissionDefaults') -> 'PhotoAction':
        return cls()


@dataclass
class LandAction(Action):
    """Land at current position"""

    @property
    def action_type(self) -> ActionType:
        return ActionType.LAND

    def to_dict(self) -> Dict[str, Any]:
        return {"type": "land"}

    @classmethod
    def from_dict(cls, data: Dict[str, Any], defaults: 'MissionDefaults') -> 'LandAction':
        return cls()


@dataclass
class RthAction(Action):
    """Return to home and land"""

    @property
    def action_type(self) -> ActionType:
        return ActionType.RTH

    def to_dict(self) -> Dict[str, Any]:
        return {"type": "rth"}

    @classmethod
    def from_dict(cls, data: Dict[str, Any], defaults: 'MissionDefaults') -> 'RthAction':
        return cls()


# Action type mapping
ACTION_CLASSES: Dict[str, type] = {
    "takeoff": TakeoffAction,
    "goto": GotoAction,
    "hover": HoverAction,
    "orient": OrientAction,
    "delay": DelayAction,
    "photo": PhotoAction,
    "land": LandAction,
    "rth": RthAction,
}


@dataclass
class MissionDefaults:
    """Default values for mission actions"""
    speed: float = 5.0  # m/s
    alt: float = 10.0   # meters

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'MissionDefaults':
        return cls(
            speed=data.get("speed", 5.0),
            alt=data.get("alt", 10.0),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {"speed": self.speed, "alt": self.alt}


@dataclass
class Mission:
    """
    Complete mission definition (v1.0 format)

    A mission is a sequence of actions executed in order.
    Segments are implicitly defined as consecutive goto actions.
    """
    uuid: str
    name: str
    version: str
    defaults: MissionDefaults
    actions: List[Action]

    @classmethod
    def from_dict(cls, data: Dict[str, Any], mission_uuid: Optional[str] = None) -> 'Mission':
        """
        Create mission from dictionary (JSON data)

        Args:
            data: Mission data dictionary
            mission_uuid: UUID to use (generated if None)

        Returns:
            Mission instance

        Raises:
            ValidationError: If mission data is invalid
        """
        # Generate UUID if not provided
        if mission_uuid is None:
            mission_uuid = str(uuid.uuid4())

        # Parse defaults
        defaults = MissionDefaults.from_dict(data.get("defaults", {}))

        # Parse actions
        actions: List[Action] = []
        actions_data = data.get("actions", [])

        if not actions_data:
            raise ValidationError("Mission must have at least one action")

        for i, action_data in enumerate(actions_data):
            action_type = action_data.get("type")
            if action_type not in ACTION_CLASSES:
                raise ValidationError(f"Action {i}: unknown type '{action_type}'")

            action_cls = ACTION_CLASSES[action_type]
            try:
                action = action_cls.from_dict(action_data, defaults)
                actions.append(action)
            except KeyError as e:
                raise ValidationError(f"Action {i} ({action_type}): missing required field {e}")
            except (TypeError, ValueError) as e:
                raise ValidationError(f"Action {i} ({action_type}): {e}")

        mission = cls(
            uuid=mission_uuid,
            name=data.get("name", "Unnamed Mission"),
            version=data.get("version", "1.0"),
            defaults=defaults,
            actions=actions,
        )

        # Validate mission
        errors = mission.validate()
        if errors:
            raise ValidationError(f"Mission validation failed: {'; '.join(errors)}")

        return mission

    def to_dict(self) -> Dict[str, Any]:
        """Convert mission to dictionary (for JSON serialization)"""
        return {
            "version": self.version,
            "name": self.name,
            "defaults": self.defaults.to_dict(),
            "actions": [action.to_dict() for action in self.actions],
        }

    def validate(self) -> List[str]:
        """
        Validate entire mission

        Returns:
            List of validation error messages (empty if valid)
        """
        errors = []

        # Must have actions
        if not self.actions:
            errors.append("Mission must have at least one action")
            return errors

        # First action should be takeoff (warning, not error)
        if self.actions[0].action_type != ActionType.TAKEOFF:
            errors.append("First action should be 'takeoff'")

        # Last action must be land or rth
        last_action = self.actions[-1]
        if last_action.action_type not in (ActionType.LAND, ActionType.RTH):
            errors.append("Last action must be 'land' or 'rth'")

        # Validate individual actions
        for i, action in enumerate(self.actions):
            action_errors = action.validate()
            for err in action_errors:
                errors.append(f"Action {i}: {err}")

        # Check for duplicate consecutive goto (might be intentional, just warn)
        # No error here, user might want to change altitude at same position

        return errors

    def get_segments(self) -> List[List[GotoAction]]:
        """
        Get path segments (consecutive goto actions)

        Returns:
            List of segments, each segment is a list of GotoAction
        """
        segments: List[List[GotoAction]] = []
        current_segment: List[GotoAction] = []

        for action in self.actions:
            if isinstance(action, GotoAction):
                current_segment.append(action)
            else:
                if current_segment:
                    segments.append(current_segment)
                    current_segment = []

        # Don't forget last segment
        if current_segment:
            segments.append(current_segment)

        return segments

    @property
    def action_count(self) -> int:
        """Number of actions in mission"""
        return len(self.actions)

    @property
    def goto_count(self) -> int:
        """Number of goto actions (waypoints)"""
        return sum(1 for a in self.actions if isinstance(a, GotoAction))

    def get_summary(self) -> Dict[str, Any]:
        """Get mission summary for API responses"""
        return {
            "uuid": self.uuid,
            "name": self.name,
            "version": self.version,
            "action_count": self.action_count,
            "goto_count": self.goto_count,
            "segment_count": len(self.get_segments()),
        }
