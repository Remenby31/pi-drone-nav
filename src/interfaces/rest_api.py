"""
REST API for Pi Drone Navigation

Provides HTTP endpoints for drone monitoring and mission management.
All flight control goes through missions.
"""

import threading
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


def create_api_server(flight_controller: 'FlightController',
                      port: int = 8080,
                      host: str = '0.0.0.0',
                      missions_dir: str = '~/missions') -> Optional['APIServer']:
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

    server = APIServer(flight_controller, port, host, missions_dir)
    flight_controller.set_mission_executor(server.mission_executor)
    server.start()
    return server


class APIServer:
    """REST API Server"""

    def __init__(self, flight_controller: 'FlightController',
                 port: int = 8080, host: str = '0.0.0.0',
                 missions_dir: str = '~/missions'):
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

        # ==================== Status ====================

        @self.app.route('/api/status', methods=['GET'])
        def get_status():
            """Get complete drone status and telemetry"""
            return jsonify(self.fc.get_telemetry())

        @self.app.route('/api/state', methods=['GET'])
        def get_state():
            """Get flight state machine status"""
            return jsonify(self.fc.state_machine.get_status())

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
            """Get mission details"""
            mission = self.mission_store.get(mission_uuid)
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
            """
            if self.mission_executor.state.name in ('RUNNING', 'PAUSED'):
                return jsonify({'error': 'Another mission is active'}), 409

            mission = self.mission_store.get(mission_uuid)
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
                from ..flight.state_machine import FlightState
                self.fc.state_machine.transition_to(FlightState.MISSION)
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

            Stops execution and hovers in place. Does NOT disarm.
            Use land action or RTH to safely end flight.
            """
            if self.mission_executor.state.name not in ('RUNNING', 'PAUSED'):
                return jsonify({'error': 'No active mission'}), 400

            self.mission_executor.stop()
            self.fc.hold_position()

            return jsonify({'success': True, 'message': 'Mission stopped'})

        @self.app.route('/api/missions/active/skip', methods=['POST'])
        def skip_action():
            """Skip current action"""
            if self.mission_executor.state.name != 'RUNNING':
                return jsonify({'error': 'No running mission'}), 400

            self.mission_executor.skip()
            return jsonify({
                'success': True,
                'new_action_index': self.mission_executor.current_action_index
            })

        @self.app.route('/api/missions/active/goto/<int:index>', methods=['POST'])
        def goto_action(index: int):
            """Jump to specific action index"""
            if self.mission_executor.state.name not in ('RUNNING', 'PAUSED'):
                return jsonify({'error': 'No active mission'}), 400

            if self.mission_executor.goto_action(index):
                return jsonify({'success': True, 'action_index': index})

            return jsonify({'error': f'Invalid action index {index}'}), 400

        # ==================== Serial Ports ====================

        @self.app.route('/api/ports', methods=['GET'])
        def list_ports():
            """
            List available serial ports

            Returns:
                {
                    "ports": [...],          # All available ports
                    "betaflight_ports": [...], # Ports matching Betaflight signature
                    "current_port": str|null   # Currently connected port
                }
            """
            from ..drivers.serial_manager import SerialManager

            all_ports = SerialManager.list_available_ports()
            bf_ports = SerialManager.find_betaflight_ports()

            current = None
            if self.fc.serial_manager:
                current = self.fc.serial_manager.current_port

            return jsonify({
                'ports': all_ports,
                'betaflight_ports': bf_ports,
                'current_port': current
            })

        @self.app.route('/api/ports/connect', methods=['POST'])
        def connect_port():
            """
            Connect to a specific serial port

            Request body: {"port": "/dev/ttyACM0"} or {} for auto-detect

            Returns:
                {"success": true, "port": str} on success
                {"error": str} on failure
            """
            data = request.get_json() or {}
            port = data.get('port')

            if not self.fc.serial_manager:
                return jsonify({'error': 'Serial manager not initialized'}), 500

            # Check if armed - don't allow port change while armed
            if self.fc._armed:
                return jsonify({'error': 'Cannot change port while armed'}), 409

            # Disconnect current connection
            self.fc.serial_manager.disconnect()

            if port:
                # Connect to specific port
                if self.fc.serial_manager.connect(port):
                    # Re-initialize MSP
                    if self.fc.msp.connect():
                        return jsonify({'success': True, 'port': port})
                    else:
                        return jsonify({'error': 'MSP handshake failed'}), 500
                else:
                    return jsonify({'error': f'Failed to connect to {port}'}), 400
            else:
                # Auto-detect
                if self.fc.serial_manager.auto_detect_and_connect():
                    connected_port = self.fc.serial_manager.current_port
                    # Re-initialize MSP
                    if self.fc.msp.connect():
                        return jsonify({
                            'success': True,
                            'port': connected_port,
                            'auto_detected': True
                        })
                    else:
                        return jsonify({'error': 'MSP handshake failed'}), 500
                else:
                    return jsonify({'error': 'Auto-detection failed'}), 400

        @self.app.route('/api/ports/disconnect', methods=['POST'])
        def disconnect_port():
            """Disconnect from current serial port"""
            if not self.fc.serial_manager:
                return jsonify({'error': 'Serial manager not initialized'}), 500

            if self.fc._armed:
                return jsonify({'error': 'Cannot disconnect while armed'}), 409

            self.fc.serial_manager.disconnect()
            return jsonify({'success': True})

        # ==================== Configuration ====================

        @self.app.route('/api/config', methods=['GET'])
        def get_config():
            """Get system configuration"""
            from ..config import get_config
            config = get_config()

            result = {}
            for section in ['serial', 'gps', 'navigation', 'altitude', 'failsafe', 'interface']:
                sect = getattr(config, section)
                result[section] = {k: v for k, v in sect.__dict__.items()}

            return jsonify(result)

        @self.app.route('/api/config', methods=['POST'])
        def set_config():
            """Update system configuration"""
            from ..config import get_config
            config = get_config()
            data = request.get_json()

            if not data:
                return jsonify({'error': 'No data provided'}), 400

            config._update_from_dict(data)
            return jsonify({'success': True})

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
