"""
REST API for Pi Drone Navigation Server

Provides HTTP endpoints for drone monitoring and mission management.
"""

import threading
import time
from typing import TYPE_CHECKING, Optional
import logging

try:
    from flask import Flask, request, jsonify
    from flask_cors import CORS
    FLASK_AVAILABLE = True
except ImportError:
    FLASK_AVAILABLE = False

from ..mission import MissionStore, MissionExecutor, ValidationError

if TYPE_CHECKING:
    from ..flight.flight_controller import FlightController

logger = logging.getLogger(__name__)

# Default missions directory
DEFAULT_MISSIONS_DIR = '~/.pidrone/missions'


def create_api_server(flight_controller: 'FlightController',
                      port: int = 8080,
                      host: str = '0.0.0.0',
                      missions_dir: str = None) -> Optional['APIServer']:
    """
    Create and start REST API server

    Args:
        flight_controller: FlightController instance
        port: HTTP port
        host: Host address
        missions_dir: Directory for mission storage

    Returns:
        APIServer instance or None if Flask not available
    """
    if not FLASK_AVAILABLE:
        logger.warning("Flask not installed - REST API disabled")
        return None

    if missions_dir is None:
        missions_dir = DEFAULT_MISSIONS_DIR

    server = APIServer(flight_controller, port, host, missions_dir)
    flight_controller.set_mission_executor(server.mission_executor)
    server.start()
    return server


class APIServer:
    """REST API Server"""

    def __init__(self, flight_controller: 'FlightController',
                 port: int = 8080, host: str = '0.0.0.0',
                 missions_dir: str = DEFAULT_MISSIONS_DIR):
        self.fc = flight_controller
        self.port = port
        self.host = host

        self.app = Flask(__name__)
        CORS(self.app)

        self.mission_store = MissionStore(missions_dir)
        self.mission_executor = MissionExecutor()

        self._thread: Optional[threading.Thread] = None
        self._setup_routes()

    def _setup_routes(self):
        """Setup API routes"""

        # ==================== Health ====================

        @self.app.route('/api/health', methods=['GET'])
        def health():
            """Health check endpoint"""
            return jsonify({
                'status': 'ok',
                'armed': self.fc._armed if hasattr(self.fc, '_armed') else False,
                'state': self.fc.state_machine.state.name
            })

        # ==================== Status ====================

        @self.app.route('/api/status', methods=['GET'])
        def get_status():
            """Get complete drone status and telemetry"""
            return jsonify(self.fc.get_telemetry())

        # ==================== Missions ====================

        @self.app.route('/api/missions', methods=['GET'])
        def list_missions():
            """List all stored missions"""
            missions = self.mission_store.list_all()
            return jsonify({'missions': missions})

        @self.app.route('/api/missions', methods=['POST'])
        def create_mission():
            """
            Upload a new mission

            Request body: Mission JSON
            Returns: {uuid, name, valid, action_count}
            """
            data = request.get_json()
            if not data:
                return jsonify({'error': 'No data provided'}), 400

            try:
                mission = self.mission_store.create(data)
                return jsonify({
                    'uuid': mission.uuid,
                    'name': mission.name,
                    'valid': True,
                    'action_count': mission.action_count
                }), 201

            except ValidationError as e:
                return jsonify({'error': str(e), 'valid': False}), 400

        @self.app.route('/api/missions/<mission_uuid>', methods=['GET'])
        def get_mission(mission_uuid: str):
            """Get mission details by UUID or name"""
            mission = self._resolve_mission(mission_uuid)
            if not mission:
                return jsonify({'error': 'Mission not found'}), 404

            result = mission.to_dict()
            result['uuid'] = mission.uuid
            return jsonify(result)

        @self.app.route('/api/missions/<mission_uuid>', methods=['DELETE'])
        def delete_mission(mission_uuid: str):
            """Delete a mission"""
            if (self.mission_executor.mission and
                self.mission_executor.mission.uuid == mission_uuid and
                self.mission_executor.state.name in ('RUNNING', 'PAUSED')):
                return jsonify({'error': 'Cannot delete active mission'}), 409

            if self.mission_store.delete(mission_uuid):
                return jsonify({'success': True})
            return jsonify({'error': 'Mission not found'}), 404

        # ==================== Mission Execution ====================

        @self.app.route('/api/missions/<mission_uuid>/start', methods=['POST'])
        def start_mission(mission_uuid: str):
            """
            Start a mission

            Arms the drone automatically before takeoff.
            Auto-disarms when mission completes.
            """
            if self.mission_executor.state.name in ('RUNNING', 'PAUSED'):
                return jsonify({'error': 'Another mission is active'}), 409

            mission = self._resolve_mission(mission_uuid)
            if not mission:
                return jsonify({'error': 'Mission not found'}), 404

            # Load mission
            self.mission_executor.load(mission)

            # Set home from current position
            if self.fc._last_gps_fix:
                self.mission_executor.set_home(
                    self.fc._last_gps_fix.latitude,
                    self.fc._last_gps_fix.longitude,
                    self.fc.alt_controller.current_altitude
                )

            # Arm and start
            if not self.fc.arm():
                return jsonify({'error': 'Failed to arm'}), 400

            if self.mission_executor.start():
                # Note: State transitions are handled by flight controller
                # based on mission actions (takeoff -> flying -> etc)
                # Don't force MISSION state here - let the control loop handle it
                return jsonify({
                    'success': True,
                    'message': f"Mission '{mission.name}' started"
                })

            # Failed to start, disarm
            self.fc.disarm()
            return jsonify({'error': 'Failed to start mission'}), 400

        @self.app.route('/api/missions/active', methods=['GET'])
        def get_active_mission():
            """Get active mission execution status"""
            status = self.mission_executor.get_status_dict()

            if self.fc._last_gps_fix:
                status['position'] = {
                    'lat': self.fc._last_gps_fix.latitude,
                    'lon': self.fc._last_gps_fix.longitude,
                    'alt': self.fc.alt_controller.current_altitude
                }

            current_action = status.get('current_action')
            if current_action and current_action.get('type') == 'goto':
                if self.fc._last_gps_fix:
                    from ..utils.geo import haversine_distance
                    dist = haversine_distance(
                        self.fc._last_gps_fix.latitude,
                        self.fc._last_gps_fix.longitude,
                        current_action['lat'],
                        current_action['lon']
                    )
                    status['distance_to_next_m'] = dist

            return jsonify(status)

        @self.app.route('/api/missions/active/pause', methods=['POST'])
        def pause_mission():
            """Pause active mission (hover in place)"""
            if self.mission_executor.state.name != 'RUNNING':
                return jsonify({'error': 'No running mission'}), 400

            self.mission_executor.pause()
            return jsonify({'success': True, 'message': 'Mission paused'})

        @self.app.route('/api/missions/active/resume', methods=['POST'])
        def resume_mission():
            """Resume paused mission"""
            if self.mission_executor.state.name != 'PAUSED':
                return jsonify({'error': 'Mission not paused'}), 400

            self.mission_executor.resume()
            return jsonify({'success': True, 'message': 'Mission resumed'})

        @self.app.route('/api/missions/active/stop', methods=['POST'])
        def stop_mission():
            """
            Stop active mission

            Lands the drone and disarms.
            """
            if self.mission_executor.state.name not in ('RUNNING', 'PAUSED'):
                return jsonify({'error': 'No active mission'}), 400

            self.mission_executor.stop()

            # Land if flying, then always disarm
            if self.fc.state_machine.is_flying:
                self.fc.land()
                # Wait a bit for landing to initiate
                time.sleep(0.5)

            # Always disarm when stopping mission
            self.fc.disarm()

            return jsonify({'success': True, 'message': 'Mission stopped, drone disarmed'})

        # ==================== Diagnostics ====================

        @self.app.route('/api/diagnostics', methods=['GET'])
        def get_diagnostics():
            """Get full system diagnostics"""
            diag = {
                'flight_state': {
                    'state': self.fc.state_machine.state.name,
                    'previous_state': self.fc.state_machine.previous_state.name,
                    'time_in_state': self.fc.state_machine.time_in_state,
                    'is_flying': self.fc.state_machine.is_flying
                },
                'msp': {
                    'connected': self.fc.msp is not None
                },
                'gps': {},
                'controllers': {},
                'takeoff': {},
                'hover_learner': {}
            }

            # MSP details
            if self.fc.msp:
                try:
                    api = self.fc.msp.get_api_version()
                    diag['msp']['api_version'] = api
                except:
                    diag['msp']['api_version'] = 'error'

            # GPS
            gps_source = getattr(self.fc, '_gps_source', 'unknown')
            diag['gps']['source'] = gps_source
            if self.fc.msp:
                try:
                    gps = self.fc.msp.get_gps_as_fix()
                    if gps:
                        diag['gps']['satellites'] = gps.num_satellites
                        diag['gps']['fix_valid'] = gps.fix_valid
                        diag['gps']['latitude'] = gps.latitude
                        diag['gps']['longitude'] = gps.longitude
                except:
                    diag['gps']['error'] = 'Failed to read GPS'

            # Altitude controller
            if self.fc.alt_controller:
                debug = self.fc.alt_controller.get_debug_info()
                diag['controllers']['altitude'] = {
                    'target': debug['target_alt'],
                    'current': debug['current_alt'],
                    'hover_throttle': debug['hover_throttle']
                }

            # Takeoff controller
            if self.fc.takeoff_controller:
                status = self.fc.takeoff_controller.get_status()
                diag['takeoff'] = status

            # Hover learner
            if self.fc.hover_learner:
                diag['hover_learner'] = self.fc.hover_learner.get_status()

            return jsonify(diag)

        @self.app.route('/api/test/<component>', methods=['POST'])
        def run_test(component: str):
            """
            Run diagnostic test for a component

            Components: msp, gps, sensors, motors, rc, preflight
            """
            valid_components = ['msp', 'gps', 'sensors', 'motors', 'rc', 'preflight']
            if component not in valid_components:
                return jsonify({
                    'error': f'Unknown component: {component}',
                    'valid_components': valid_components
                }), 400

            result = {'component': component, 'tests': [], 'passed': True}

            if component == 'msp':
                result['tests'] = self._test_msp()
            elif component == 'gps':
                result['tests'] = self._test_gps()
            elif component == 'sensors':
                result['tests'] = self._test_sensors()
            elif component == 'motors':
                # Motors require confirmation
                data = request.get_json() or {}
                if not data.get('confirm'):
                    return jsonify({
                        'error': 'Motor test requires confirmation. Send {"confirm": true} in request body.',
                        'warning': 'Motors will spin! Remove propellers first.'
                    }), 400
                result['tests'] = self._test_motors()
            elif component == 'rc':
                result['tests'] = self._test_rc()
            elif component == 'preflight':
                result['tests'] = self._test_preflight()

            # Check if any test failed
            for test in result['tests']:
                if test.get('result') == 'FAIL':
                    result['passed'] = False
                    break

            return jsonify(result)

    # ==================== Test Methods ====================

    def _test_msp(self) -> list:
        """Test MSP communication"""
        tests = []
        if not self.fc.msp:
            return [{'name': 'MSP Connection', 'result': 'FAIL', 'message': 'MSP not initialized'}]

        try:
            api = self.fc.msp.get_api_version()
            tests.append({'name': 'API Version', 'result': 'PASS', 'value': api})

            variant = self.fc.msp.get_fc_variant()
            tests.append({'name': 'FC Variant', 'result': 'PASS', 'value': variant})

            status = self.fc.msp.get_status()
            if status:
                tests.append({
                    'name': 'Status',
                    'result': 'PASS',
                    'value': f'cycle={status.cycle_time}us, cpu={status.cpu_load}%'
                })
            else:
                tests.append({'name': 'Status', 'result': 'FAIL', 'message': 'Could not read status'})

        except Exception as e:
            tests.append({'name': 'MSP Communication', 'result': 'FAIL', 'message': str(e)})

        return tests

    def _test_gps(self) -> list:
        """Test GPS reception"""
        tests = []
        if not self.fc.msp:
            return [{'name': 'GPS', 'result': 'FAIL', 'message': 'MSP not initialized'}]

        try:
            gps = self.fc.msp.get_gps_as_fix()
            if gps:
                tests.append({'name': 'Satellites', 'result': 'PASS', 'value': gps.num_satellites})
                tests.append({'name': 'Fix Valid', 'result': 'PASS' if gps.fix_valid else 'WARN',
                              'value': gps.fix_valid})
                tests.append({'name': 'Position', 'result': 'PASS',
                              'value': f'{gps.latitude:.6f}, {gps.longitude:.6f}'})
                tests.append({'name': 'Altitude', 'result': 'PASS', 'value': f'{gps.altitude_msl:.1f}m'})

                if gps.num_satellites >= 5 and gps.fix_valid:
                    tests.append({'name': 'GPS Quality', 'result': 'PASS', 'message': 'Good fix'})
                elif gps.num_satellites >= 3:
                    tests.append({'name': 'GPS Quality', 'result': 'WARN', 'message': 'Weak fix'})
                else:
                    tests.append({'name': 'GPS Quality', 'result': 'FAIL', 'message': 'No fix'})
            else:
                tests.append({'name': 'GPS Data', 'result': 'FAIL', 'message': 'Could not read GPS'})

        except Exception as e:
            tests.append({'name': 'GPS', 'result': 'FAIL', 'message': str(e)})

        return tests

    def _test_sensors(self) -> list:
        """Test sensors (attitude, altitude, IMU, battery)"""
        tests = []
        if not self.fc.msp:
            return [{'name': 'Sensors', 'result': 'FAIL', 'message': 'MSP not initialized'}]

        try:
            # Attitude
            att = self.fc.msp.get_attitude()
            if att:
                tests.append({
                    'name': 'Attitude',
                    'result': 'PASS' if abs(att.roll) < 45 and abs(att.pitch) < 45 else 'WARN',
                    'value': f'roll={att.roll:.1f}, pitch={att.pitch:.1f}, yaw={att.yaw:.1f}'
                })
            else:
                tests.append({'name': 'Attitude', 'result': 'FAIL', 'message': 'Could not read'})

            # Altitude
            alt = self.fc.msp.get_altitude()
            if alt:
                tests.append({
                    'name': 'Altitude',
                    'result': 'PASS',
                    'value': f'{alt.altitude_cm/100:.2f}m, vario={alt.vario_cms/100:.2f}m/s'
                })
            else:
                tests.append({'name': 'Altitude', 'result': 'FAIL', 'message': 'Could not read'})

            # IMU
            imu = self.fc.msp.get_raw_imu()
            if imu:
                tests.append({
                    'name': 'IMU Acc',
                    'result': 'PASS',
                    'value': f'({imu.acc_x}, {imu.acc_y}, {imu.acc_z})'
                })
                tests.append({
                    'name': 'IMU Gyro',
                    'result': 'PASS',
                    'value': f'({imu.gyro_x}, {imu.gyro_y}, {imu.gyro_z})'
                })
            else:
                tests.append({'name': 'IMU', 'result': 'FAIL', 'message': 'Could not read'})

            # Battery
            analog = self.fc.msp.get_analog()
            if analog:
                result = 'PASS' if analog.vbat > 10.0 else 'WARN'
                tests.append({
                    'name': 'Battery',
                    'result': result,
                    'value': f'{analog.vbat:.1f}V, {analog.mah_drawn}mAh used'
                })
            else:
                tests.append({'name': 'Battery', 'result': 'FAIL', 'message': 'Could not read'})

        except Exception as e:
            tests.append({'name': 'Sensors', 'result': 'FAIL', 'message': str(e)})

        return tests

    def _test_motors(self) -> list:
        """Test individual motors"""
        tests = []
        if not self.fc.msp:
            return [{'name': 'Motors', 'result': 'FAIL', 'message': 'MSP not initialized'}]

        try:
            # Read initial values
            motors = self.fc.msp.get_motor_values()
            tests.append({'name': 'Initial State', 'result': 'PASS', 'value': str(motors[:4])})

            # Test each motor
            for i in range(4):
                test_values = [1000, 1000, 1000, 1000]
                test_values[i] = 1100
                self.fc.msp.set_motor_test(test_values)
                time.sleep(0.3)

                motors = self.fc.msp.get_motor_values()
                if motors[i] > 1000:
                    tests.append({
                        'name': f'Motor {i+1}',
                        'result': 'PASS',
                        'value': motors[i]
                    })
                else:
                    tests.append({
                        'name': f'Motor {i+1}',
                        'result': 'FAIL',
                        'value': motors[i]
                    })

            # Stop all motors
            self.fc.msp.stop_all_motors()
            tests.append({'name': 'Motor Stop', 'result': 'PASS'})

        except Exception as e:
            # Emergency stop
            if self.fc.msp:
                self.fc.msp.stop_all_motors()
            tests.append({'name': 'Motors', 'result': 'FAIL', 'message': str(e)})

        return tests

    def _test_rc(self) -> list:
        """Test RC channel control"""
        tests = []
        if not self.fc.msp:
            return [{'name': 'RC', 'result': 'FAIL', 'message': 'MSP not initialized'}]

        try:
            # Read current RC
            rc_before = self.fc.msp.get_rc_channels()
            tests.append({'name': 'RC Read', 'result': 'PASS', 'value': str(rc_before[:8])})

            # Send test RC (neutral values, disarmed)
            test_values = [1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500]
            for _ in range(10):
                self.fc.msp.set_raw_rc(test_values)
                time.sleep(0.02)

            rc_after = self.fc.msp.get_rc_channels()
            tests.append({'name': 'RC After', 'result': 'PASS', 'value': str(rc_after[:8])})

            if rc_after[2] == 1000:
                tests.append({'name': 'RC Control', 'result': 'PASS', 'message': 'RC values applied'})
            else:
                tests.append({
                    'name': 'RC Control',
                    'result': 'WARN',
                    'message': 'RC values may not be applied - check RX_MSP'
                })

        except Exception as e:
            tests.append({'name': 'RC', 'result': 'FAIL', 'message': str(e)})

        return tests

    def _test_preflight(self) -> list:
        """Run pre-flight checks"""
        tests = []
        try:
            from ..flight.preflight_checks import PreflightChecker
            from ..config import TakeoffConfig

            config = TakeoffConfig()
            checker = PreflightChecker(config)

            gps = self.fc.msp.get_gps_as_fix() if self.fc.msp else None
            att = self.fc.msp.get_attitude() if self.fc.msp else None

            result = checker.run_checks(self.fc.msp, gps, att)

            for check in result.checks:
                tests.append({
                    'name': check.name,
                    'result': check.result.name,
                    'message': check.message
                })

            tests.append({
                'name': 'Overall',
                'result': 'PASS' if result.passed else 'FAIL',
                'message': 'Ready for flight' if result.passed else 'Do not fly!'
            })

        except Exception as e:
            tests.append({'name': 'Preflight', 'result': 'FAIL', 'message': str(e)})

        return tests

    # ==================== Helpers ====================

    def _resolve_mission(self, name_or_uuid: str):
        """Resolve mission by name or UUID"""
        # Try direct UUID lookup
        mission = self.mission_store.get(name_or_uuid)
        if mission:
            return mission

        # Try by name
        missions = self.mission_store.list_all()
        for m in missions:
            if m['name'].lower() == name_or_uuid.lower():
                return self.mission_store.get(m['uuid'])

        return None

    def start(self):
        """Start API server in background thread"""
        self._thread = threading.Thread(
            target=lambda: self.app.run(
                host=self.host,
                port=self.port,
                debug=False,
                use_reloader=False
            ),
            daemon=True
        )
        self._thread.start()
        logger.info(f"REST API started on http://{self.host}:{self.port}")

    def stop(self):
        """Stop API server"""
        pass

    def get_executor(self) -> MissionExecutor:
        """Get mission executor"""
        return self.mission_executor
