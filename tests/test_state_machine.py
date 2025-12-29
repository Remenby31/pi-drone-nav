"""
Tests for Flight State Machine
"""

import unittest

import sys
sys.path.insert(0, '/home/remenby/projects/pi_drone_nav')

# Try to import state machine
try:
    from src.flight.state_machine import FlightStateMachine, FlightState
    STATE_MACHINE_AVAILABLE = True
except ImportError:
    STATE_MACHINE_AVAILABLE = False


@unittest.skipUnless(STATE_MACHINE_AVAILABLE, "State machine not available")
class TestFlightStateMachine(unittest.TestCase):
    """Test flight state machine transitions"""

    def setUp(self):
        """Set up test state machine"""
        self.sm = FlightStateMachine()

    def test_initial_state(self):
        """Test initial state is IDLE"""
        self.assertEqual(self.sm.state, FlightState.IDLE)
        self.assertFalse(self.sm.is_flying)

    # ==================== Valid Transitions ====================

    def test_idle_to_armed(self):
        """Test transition from IDLE to ARMED"""
        success = self.sm.transition_to(FlightState.ARMED)

        self.assertTrue(success)
        self.assertEqual(self.sm.state, FlightState.ARMED)
        self.assertFalse(self.sm.is_flying)

    def test_armed_to_takeoff(self):
        """Test transition from ARMED to TAKEOFF"""
        self.sm.transition_to(FlightState.ARMED)
        success = self.sm.transition_to(FlightState.TAKEOFF)

        self.assertTrue(success)
        self.assertEqual(self.sm.state, FlightState.TAKEOFF)
        self.assertTrue(self.sm.is_flying)

    def test_takeoff_to_hover(self):
        """Test transition from TAKEOFF to HOVER"""
        self.sm.transition_to(FlightState.ARMED)
        self.sm.transition_to(FlightState.TAKEOFF)
        success = self.sm.transition_to(FlightState.HOVER)

        self.assertTrue(success)
        self.assertEqual(self.sm.state, FlightState.HOVER)

    def test_hover_to_position_hold(self):
        """Test transition from HOVER to POSITION_HOLD"""
        self._get_to_hover()
        success = self.sm.transition_to(FlightState.POSITION_HOLD)

        self.assertTrue(success)
        self.assertEqual(self.sm.state, FlightState.POSITION_HOLD)

    def test_hover_to_flying(self):
        """Test transition from HOVER to FLYING"""
        self._get_to_hover()
        success = self.sm.transition_to(FlightState.FLYING)

        self.assertTrue(success)
        self.assertEqual(self.sm.state, FlightState.FLYING)

    def test_hover_to_mission(self):
        """Test transition from HOVER to MISSION"""
        self._get_to_hover()
        success = self.sm.transition_to(FlightState.MISSION)

        self.assertTrue(success)
        self.assertEqual(self.sm.state, FlightState.MISSION)

    def test_flying_to_landing(self):
        """Test transition from FLYING to LANDING"""
        self._get_to_flying()
        success = self.sm.transition_to(FlightState.LANDING)

        self.assertTrue(success)
        self.assertEqual(self.sm.state, FlightState.LANDING)

    def test_landing_to_landed(self):
        """Test transition from LANDING to LANDED"""
        self._get_to_flying()
        self.sm.transition_to(FlightState.LANDING)
        success = self.sm.transition_to(FlightState.LANDED)

        self.assertTrue(success)
        self.assertEqual(self.sm.state, FlightState.LANDED)
        self.assertFalse(self.sm.is_flying)

    def test_landed_to_idle(self):
        """Test transition from LANDED to IDLE"""
        self._get_to_flying()
        self.sm.transition_to(FlightState.LANDING)
        self.sm.transition_to(FlightState.LANDED)
        success = self.sm.transition_to(FlightState.IDLE)

        self.assertTrue(success)
        self.assertEqual(self.sm.state, FlightState.IDLE)

    def test_armed_to_idle(self):
        """Test transition from ARMED back to IDLE"""
        self.sm.transition_to(FlightState.ARMED)
        success = self.sm.transition_to(FlightState.IDLE)

        self.assertTrue(success)
        self.assertEqual(self.sm.state, FlightState.IDLE)

    # ==================== Invalid Transitions ====================

    def test_idle_to_takeoff_invalid(self):
        """Test cannot go directly from IDLE to TAKEOFF"""
        success = self.sm.transition_to(FlightState.TAKEOFF)

        self.assertFalse(success)
        self.assertEqual(self.sm.state, FlightState.IDLE)

    def test_idle_to_flying_invalid(self):
        """Test cannot go directly from IDLE to FLYING"""
        success = self.sm.transition_to(FlightState.FLYING)

        self.assertFalse(success)
        self.assertEqual(self.sm.state, FlightState.IDLE)

    def test_flying_to_idle_invalid(self):
        """Test cannot go directly from FLYING to IDLE"""
        self._get_to_flying()
        success = self.sm.transition_to(FlightState.IDLE)

        self.assertFalse(success)
        self.assertEqual(self.sm.state, FlightState.FLYING)

    def test_takeoff_to_idle_abort(self):
        """Test can abort from TAKEOFF to IDLE (for takeoff abort on ground)"""
        self.sm.transition_to(FlightState.ARMED)
        self.sm.transition_to(FlightState.TAKEOFF)
        success = self.sm.transition_to(FlightState.IDLE)

        # TAKEOFF → IDLE is now valid for abort scenarios
        self.assertTrue(success)
        self.assertEqual(self.sm.state, FlightState.IDLE)

    # ==================== Emergency Transitions ====================

    def test_any_to_failsafe(self):
        """Test FAILSAFE can be reached from any flying state"""
        self._get_to_flying()
        success = self.sm.transition_to(FlightState.FAILSAFE)

        self.assertTrue(success)
        self.assertEqual(self.sm.state, FlightState.FAILSAFE)

    def test_failsafe_from_mission(self):
        """Test FAILSAFE can be reached from MISSION"""
        self._get_to_hover()
        self.sm.transition_to(FlightState.MISSION)
        success = self.sm.transition_to(FlightState.FAILSAFE)

        self.assertTrue(success)
        self.assertEqual(self.sm.state, FlightState.FAILSAFE)

    def test_any_to_error(self):
        """Test ERROR can be reached from any state"""
        self._get_to_flying()
        success = self.sm.transition_to(FlightState.ERROR)

        self.assertTrue(success)
        self.assertEqual(self.sm.state, FlightState.ERROR)

    def test_error_from_idle(self):
        """Test ERROR can be reached from IDLE"""
        success = self.sm.transition_to(FlightState.ERROR)

        self.assertTrue(success)
        self.assertEqual(self.sm.state, FlightState.ERROR)

    # ==================== RTH Transitions ====================

    def test_flying_to_rth(self):
        """Test transition from FLYING to RTH"""
        self._get_to_flying()
        success = self.sm.transition_to(FlightState.RTH)

        self.assertTrue(success)
        self.assertEqual(self.sm.state, FlightState.RTH)

    def test_mission_to_rth(self):
        """Test transition from MISSION to RTH"""
        self._get_to_hover()
        self.sm.transition_to(FlightState.MISSION)
        success = self.sm.transition_to(FlightState.RTH)

        self.assertTrue(success)
        self.assertEqual(self.sm.state, FlightState.RTH)

    def test_rth_to_landing(self):
        """Test transition from RTH to LANDING"""
        self._get_to_flying()
        self.sm.transition_to(FlightState.RTH)
        success = self.sm.transition_to(FlightState.LANDING)

        self.assertTrue(success)
        self.assertEqual(self.sm.state, FlightState.LANDING)

    # ==================== State Properties ====================

    def test_is_flying_states(self):
        """Test is_flying property returns correct values"""
        # FAILSAFE is not considered flying (drone may be in emergency descent)
        non_flying = [FlightState.IDLE, FlightState.ARMED, FlightState.LANDED,
                      FlightState.ERROR, FlightState.FAILSAFE]
        flying = [FlightState.TAKEOFF, FlightState.HOVER, FlightState.POSITION_HOLD,
                  FlightState.FLYING, FlightState.MISSION, FlightState.RTH,
                  FlightState.LANDING]

        for state in non_flying:
            self.sm._state = state
            self.assertFalse(self.sm.is_flying, f"{state} should not be flying")

        for state in flying:
            self.sm._state = state
            self.assertTrue(self.sm.is_flying, f"{state} should be flying")

    def test_can_arm_states(self):
        """Test arming is only valid from IDLE"""
        # Can only arm from IDLE
        self.sm._state = FlightState.IDLE
        self.assertTrue(self.sm.can_transition_to(FlightState.ARMED))

        for state in [FlightState.ARMED, FlightState.FLYING, FlightState.LANDING]:
            self.sm._state = state
            # Cannot transition to ARMED from these states
            self.assertFalse(self.sm.can_transition_to(FlightState.ARMED))

    def test_can_disarm_states(self):
        """Test disarming is only valid from certain states"""
        # Can disarm (go to IDLE) from ARMED
        self.sm._state = FlightState.ARMED
        self.assertTrue(self.sm.can_transition_to(FlightState.IDLE))

        # Can disarm from LANDED
        self.sm._state = FlightState.LANDED
        self.assertTrue(self.sm.can_transition_to(FlightState.IDLE))

        # Cannot disarm while flying
        self.sm._state = FlightState.FLYING
        self.assertFalse(self.sm.can_transition_to(FlightState.IDLE))

    # ==================== State Status ====================

    def test_get_status(self):
        """Test get_status() returns complete info"""
        self._get_to_flying()

        status = self.sm.get_status()

        self.assertEqual(status['state'], 'FLYING')
        self.assertTrue(status['is_flying'])
        self.assertIn('time_in_state', status)
        self.assertIsNotNone(status['previous_state'])

    def test_state_history(self):
        """Test state transitions are tracked"""
        self.sm.transition_to(FlightState.ARMED)
        self.sm.transition_to(FlightState.TAKEOFF)

        status = self.sm.get_status()

        self.assertEqual(status['previous_state'], 'ARMED')

    # ==================== Callbacks ====================

    def test_transition_callback(self):
        """Test transition callbacks are called"""
        callback_called = []

        def transition_callback(old_state, new_state):
            callback_called.append((old_state, new_state))

        self.sm.on_transition(transition_callback)

        self.sm.transition_to(FlightState.ARMED)

        self.assertEqual(len(callback_called), 1)
        self.assertEqual(callback_called[0], (FlightState.IDLE, FlightState.ARMED))

    # ==================== Helper Methods ====================

    def _get_to_hover(self):
        """Helper to get state machine to HOVER"""
        self.sm.transition_to(FlightState.ARMED)
        self.sm.transition_to(FlightState.TAKEOFF)
        self.sm.transition_to(FlightState.HOVER)

    def _get_to_flying(self):
        """Helper to get state machine to FLYING"""
        self._get_to_hover()
        self.sm.transition_to(FlightState.FLYING)


@unittest.skipUnless(STATE_MACHINE_AVAILABLE, "State machine not available")
class TestFlightStateEnum(unittest.TestCase):
    """Test FlightState enum"""

    def test_all_states_defined(self):
        """Test all expected states are defined"""
        expected_states = [
            'IDLE', 'ARMED', 'TAKEOFF', 'HOVER', 'POSITION_HOLD',
            'FLYING', 'MISSION', 'RTH', 'LANDING', 'LANDED',
            'FAILSAFE', 'ERROR'
        ]

        actual_states = [s.name for s in FlightState]

        for state in expected_states:
            self.assertIn(state, actual_states)


if __name__ == '__main__':
    unittest.main()
