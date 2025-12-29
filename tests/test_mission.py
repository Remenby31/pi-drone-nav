"""
Tests for mission module

Tests mission models, store, and executor.
"""

import json
import pytest
import tempfile
import time
from pathlib import Path

from src.mission import (
    Mission,
    MissionDefaults,
    ValidationError,
    TakeoffAction,
    GotoAction,
    HoverAction,
    OrientAction,
    DelayAction,
    PhotoAction,
    LandAction,
    RthAction,
    MissionStore,
    MissionExecutor,
    ExecutorState,
)


class TestMissionModels:
    """Test mission model classes"""

    def test_takeoff_action_from_dict(self):
        """Test creating TakeoffAction from dict"""
        defaults = MissionDefaults()
        action = TakeoffAction.from_dict({"alt": 15}, defaults)

        assert action.alt == 15
        assert action.action_type.value == "takeoff"

    def test_takeoff_action_default_alt(self):
        """Test TakeoffAction uses default altitude"""
        defaults = MissionDefaults(alt=20.0)
        action = TakeoffAction.from_dict({}, defaults)

        assert action.alt == 20.0

    def test_goto_action_from_dict(self):
        """Test creating GotoAction from dict"""
        defaults = MissionDefaults()
        data = {"lat": 43.484, "lon": 1.391, "alt": 15, "speed": 3}
        action = GotoAction.from_dict(data, defaults)

        assert action.lat == 43.484
        assert action.lon == 1.391
        assert action.alt == 15
        assert action.speed == 3

    def test_goto_action_optional_fields(self):
        """Test GotoAction with optional fields omitted"""
        defaults = MissionDefaults()
        data = {"lat": 43.484, "lon": 1.391}
        action = GotoAction.from_dict(data, defaults)

        assert action.alt is None
        assert action.speed is None

    def test_hover_action(self):
        """Test HoverAction"""
        defaults = MissionDefaults()
        action = HoverAction.from_dict({"duration": 10}, defaults)

        assert action.duration == 10

    def test_orient_action(self):
        """Test OrientAction"""
        defaults = MissionDefaults()
        action = OrientAction.from_dict({"heading": 90}, defaults)

        assert action.heading == 90

    def test_photo_action(self):
        """Test PhotoAction"""
        defaults = MissionDefaults()
        action = PhotoAction.from_dict({}, defaults)

        assert action.action_type.value == "photo"

    def test_land_action(self):
        """Test LandAction"""
        defaults = MissionDefaults()
        action = LandAction.from_dict({}, defaults)

        assert action.action_type.value == "land"

    def test_rth_action(self):
        """Test RthAction"""
        defaults = MissionDefaults()
        action = RthAction.from_dict({}, defaults)

        assert action.action_type.value == "rth"


class TestMissionValidation:
    """Test mission validation"""

    def test_valid_mission(self):
        """Test valid mission passes validation"""
        data = {
            "version": "1.0",
            "name": "Test Mission",
            "defaults": {"speed": 5, "alt": 10},
            "actions": [
                {"type": "takeoff", "alt": 10},
                {"type": "goto", "lat": 43.484, "lon": 1.391},
                {"type": "land"}
            ]
        }

        mission = Mission.from_dict(data)
        assert mission.name == "Test Mission"
        assert mission.action_count == 3

    def test_mission_must_end_with_land_or_rth(self):
        """Test mission validation requires land/rth at end"""
        data = {
            "actions": [
                {"type": "takeoff", "alt": 10},
                {"type": "goto", "lat": 43.484, "lon": 1.391},
                {"type": "hover", "duration": 5}  # Invalid ending
            ]
        }

        with pytest.raises(ValidationError) as exc_info:
            Mission.from_dict(data)

        assert "Last action must be 'land' or 'rth'" in str(exc_info.value)

    def test_mission_should_start_with_takeoff(self):
        """Test mission validation warns about missing takeoff"""
        data = {
            "actions": [
                {"type": "goto", "lat": 43.484, "lon": 1.391},  # No takeoff
                {"type": "land"}
            ]
        }

        with pytest.raises(ValidationError) as exc_info:
            Mission.from_dict(data)

        assert "First action should be 'takeoff'" in str(exc_info.value)

    def test_mission_invalid_action_type(self):
        """Test mission with unknown action type"""
        data = {
            "actions": [
                {"type": "takeoff", "alt": 10},
                {"type": "invalid_action"},
                {"type": "land"}
            ]
        }

        with pytest.raises(ValidationError) as exc_info:
            Mission.from_dict(data)

        assert "unknown type" in str(exc_info.value)

    def test_goto_validation_invalid_lat(self):
        """Test goto with invalid latitude"""
        data = {
            "actions": [
                {"type": "takeoff", "alt": 10},
                {"type": "goto", "lat": 100, "lon": 1.391},  # Invalid lat
                {"type": "land"}
            ]
        }

        with pytest.raises(ValidationError) as exc_info:
            Mission.from_dict(data)

        assert "invalid latitude" in str(exc_info.value)

    def test_mission_segments(self):
        """Test segment detection"""
        data = {
            "actions": [
                {"type": "takeoff", "alt": 10},
                {"type": "goto", "lat": 43.484, "lon": 1.391},
                {"type": "goto", "lat": 43.485, "lon": 1.392},
                {"type": "hover", "duration": 5},
                {"type": "goto", "lat": 43.486, "lon": 1.393},
                {"type": "land"}
            ]
        }

        mission = Mission.from_dict(data)
        segments = mission.get_segments()

        assert len(segments) == 2
        assert len(segments[0]) == 2  # First segment has 2 gotos
        assert len(segments[1]) == 1  # Second segment has 1 goto


class TestMissionStore:
    """Test mission storage"""

    def test_create_and_get_mission(self):
        """Test creating and retrieving a mission"""
        with tempfile.TemporaryDirectory() as tmpdir:
            store = MissionStore(tmpdir)

            data = {
                "name": "Test Mission",
                "actions": [
                    {"type": "takeoff", "alt": 10},
                    {"type": "land"}
                ]
            }

            mission = store.create(data)
            assert mission.uuid is not None
            assert mission.name == "Test Mission"

            # Retrieve mission
            retrieved = store.get(mission.uuid)
            assert retrieved is not None
            assert retrieved.name == "Test Mission"

    def test_list_missions(self):
        """Test listing all missions"""
        with tempfile.TemporaryDirectory() as tmpdir:
            store = MissionStore(tmpdir)

            # Create two missions
            store.create({
                "name": "Mission 1",
                "actions": [{"type": "takeoff", "alt": 10}, {"type": "land"}]
            })
            store.create({
                "name": "Mission 2",
                "actions": [{"type": "takeoff", "alt": 10}, {"type": "rth"}]
            })

            missions = store.list_all()
            assert len(missions) == 2

    def test_delete_mission(self):
        """Test deleting a mission"""
        with tempfile.TemporaryDirectory() as tmpdir:
            store = MissionStore(tmpdir)

            mission = store.create({
                "name": "To Delete",
                "actions": [{"type": "takeoff", "alt": 10}, {"type": "land"}]
            })

            assert store.delete(mission.uuid) is True
            assert store.get(mission.uuid) is None

    def test_mission_persistence(self):
        """Test mission survives store restart"""
        with tempfile.TemporaryDirectory() as tmpdir:
            # Create mission with first store instance
            store1 = MissionStore(tmpdir)
            mission = store1.create({
                "name": "Persistent",
                "actions": [{"type": "takeoff", "alt": 10}, {"type": "land"}]
            })
            uuid = mission.uuid

            # Create new store instance (simulates restart)
            store2 = MissionStore(tmpdir)
            store2.clear_cache()

            # Should still find the mission
            retrieved = store2.get(uuid)
            assert retrieved is not None
            assert retrieved.name == "Persistent"


class TestMissionExecutor:
    """Test mission executor"""

    def test_executor_initial_state(self):
        """Test executor starts in IDLE state"""
        executor = MissionExecutor()
        assert executor.state == ExecutorState.IDLE

    def test_load_mission(self):
        """Test loading a mission"""
        executor = MissionExecutor()

        mission = Mission.from_dict({
            "name": "Test",
            "actions": [
                {"type": "takeoff", "alt": 10},
                {"type": "land"}
            ]
        })

        executor.load(mission)
        assert executor.state == ExecutorState.READY
        assert executor.mission.name == "Test"

    def test_start_mission(self):
        """Test starting a mission"""
        executor = MissionExecutor()

        mission = Mission.from_dict({
            "name": "Test",
            "actions": [
                {"type": "takeoff", "alt": 10},
                {"type": "land"}
            ]
        })

        executor.load(mission)
        executor.set_home(43.0, 1.0, 0.0)

        assert executor.start() is True
        assert executor.state == ExecutorState.RUNNING

    def test_pause_resume(self):
        """Test pausing and resuming"""
        executor = MissionExecutor()

        mission = Mission.from_dict({
            "name": "Test",
            "actions": [
                {"type": "takeoff", "alt": 10},
                {"type": "land"}
            ]
        })

        executor.load(mission)
        executor.set_home(43.0, 1.0, 0.0)
        executor.start()

        executor.pause()
        assert executor.state == ExecutorState.PAUSED

        executor.resume()
        assert executor.state == ExecutorState.RUNNING

    def test_stop_mission(self):
        """Test stopping a mission"""
        executor = MissionExecutor()

        mission = Mission.from_dict({
            "name": "Test",
            "actions": [
                {"type": "takeoff", "alt": 10},
                {"type": "land"}
            ]
        })

        executor.load(mission)
        executor.set_home(43.0, 1.0, 0.0)
        executor.start()

        executor.stop()
        assert executor.state == ExecutorState.STOPPED

    def test_takeoff_action_output(self):
        """Test takeoff action produces correct output"""
        executor = MissionExecutor()

        mission = Mission.from_dict({
            "name": "Test",
            "actions": [
                {"type": "takeoff", "alt": 15},
                {"type": "land"}
            ]
        })

        executor.load(mission)
        executor.set_home(43.0, 1.0, 0.0)
        executor.start()

        output = executor.update(0.02)

        assert output.should_takeoff is True
        assert output.target_alt == 15

    def test_goto_action_output(self):
        """Test goto action produces correct output"""
        executor = MissionExecutor()

        mission = Mission.from_dict({
            "name": "Test",
            "actions": [
                {"type": "takeoff", "alt": 10},
                {"type": "goto", "lat": 43.5, "lon": 1.5, "alt": 20, "speed": 5},
                {"type": "land"}
            ]
        })

        executor.load(mission)
        executor.set_home(43.0, 1.0, 0.0)
        executor.start()

        # Simulate takeoff complete
        executor.update_position(43.0, 1.0, 10.0)
        executor.update(0.02)  # Skip takeoff action
        executor.current_action_index = 1
        executor.action_state = executor.action_state.PENDING

        output = executor.update(0.02)

        assert output.target_lat == 43.5
        assert output.target_lon == 1.5
        assert output.target_alt == 20
        assert output.target_speed == 5

    def test_hover_action_output(self):
        """Test hover action produces correct output"""
        executor = MissionExecutor()

        mission = Mission.from_dict({
            "name": "Test",
            "actions": [
                {"type": "takeoff", "alt": 10},
                {"type": "hover", "duration": 2},
                {"type": "land"}
            ]
        })

        executor.load(mission)
        executor.set_home(43.0, 1.0, 0.0)
        executor.start()

        # Skip to hover action
        executor.current_action_index = 1
        executor.action_state = executor.action_state.PENDING

        output = executor.update(0.02)

        assert output.should_hold is True

    def test_skip_action(self):
        """Test skipping current action"""
        executor = MissionExecutor()

        mission = Mission.from_dict({
            "name": "Test",
            "actions": [
                {"type": "takeoff", "alt": 10},
                {"type": "hover", "duration": 100},  # Long hover
                {"type": "land"}
            ]
        })

        executor.load(mission)
        executor.set_home(43.0, 1.0, 0.0)
        executor.start()

        # Skip to hover
        executor.current_action_index = 1
        executor.action_state = executor.action_state.EXECUTING

        # Skip the hover
        executor.skip()

        assert executor.current_action_index == 2

    def test_goto_action_jump(self):
        """Test jumping to specific action"""
        executor = MissionExecutor()

        mission = Mission.from_dict({
            "name": "Test",
            "actions": [
                {"type": "takeoff", "alt": 10},
                {"type": "goto", "lat": 43.1, "lon": 1.1},
                {"type": "goto", "lat": 43.2, "lon": 1.2},
                {"type": "goto", "lat": 43.3, "lon": 1.3},
                {"type": "land"}
            ]
        })

        executor.load(mission)
        executor.set_home(43.0, 1.0, 0.0)
        executor.start()

        # Jump to action 3
        assert executor.goto_action(3) is True
        assert executor.current_action_index == 3

    def test_get_status(self):
        """Test getting executor status"""
        executor = MissionExecutor()

        mission = Mission.from_dict({
            "name": "Status Test",
            "actions": [
                {"type": "takeoff", "alt": 10},
                {"type": "land"}
            ]
        })

        executor.load(mission)
        executor.set_home(43.0, 1.0, 0.0)
        executor.start()

        status = executor.get_status_dict()

        assert status["state"] == "RUNNING"
        assert status["mission_name"] == "Status Test"
        assert status["action_count"] == 2
        assert status["current_action_index"] == 0


class TestMissionIntegration:
    """Integration tests for mission system"""

    def test_full_mission_flow(self):
        """Test complete mission execution flow"""
        with tempfile.TemporaryDirectory() as tmpdir:
            # Create store and executor
            store = MissionStore(tmpdir)
            executor = MissionExecutor()

            # Upload mission via store
            mission = store.create({
                "name": "Full Flow Test",
                "defaults": {"speed": 5, "alt": 10},
                "actions": [
                    {"type": "takeoff", "alt": 10},
                    {"type": "goto", "lat": 43.484, "lon": 1.391},
                    {"type": "hover", "duration": 0.1},
                    {"type": "orient", "heading": 90},
                    {"type": "photo"},
                    {"type": "land"}
                ]
            })

            # Load and start
            executor.load(mission)
            executor.set_home(43.0, 1.0, 0.0)
            executor.start()

            assert executor.state == ExecutorState.RUNNING
            assert executor.current_action_index == 0

            # Verify mission was persisted
            retrieved = store.get(mission.uuid)
            assert retrieved is not None
            assert retrieved.action_count == 6
