"""
REST API for Pi Drone Navigation

Provides HTTP endpoints for drone control, monitoring, and mission management.
"""

import json
import threading
from dataclasses import asdict
from typing import TYPE_CHECKING, Optional
import logging

try:
    from flask import Flask, request, jsonify, Response
    from flask_cors import CORS
    FLASK_AVAILABLE = True
except ImportError:
    FLASK_AVAILABLE = False

from ..mission import MissionStore, MissionExecutor, Mission, ValidationError

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

    # Connect mission executor to flight controller
    flight_controller.set_mission_executor(server.mission_executor)

    server.start()
    return server


class APIServer:
    """REST API Server wrapper"""

    def __init__(self, flight_controller: 'FlightController',
                 port: int = 8080, host: str = '0.0.0.0',
                 missions_dir: str = '~/missions'):
        self.fc = flight_controller
        self.port = port
        self.host = host

        self.app = Flask(__name__)
        CORS(self.app)  # Enable CORS for web clients

        # Mission management
        self.mission_store = MissionStore(missions_dir)
        self.mission_executor = MissionExecutor()

        self._thread: Optional[threading.Thread] = None
        self._setup_routes()

    def _setup_routes(self):
        """Setup API routes"""

        # ==================== Status ====================

        @self.app.route('/api/status', methods=['GET'])
        def get_status():
            """Get complete drone status"""
            return jsonify(self.fc.get_telemetry())

        @self.app.route('/api/telemetry', methods=['GET'])
        def get_telemetry():
            """Get telemetry data"""
            return jsonify(self.fc.get_telemetry())

        @self.app.route('/api/state', methods=['GET'])
        def get_state():
            """Get flight state"""
            return jsonify(self.fc.state_machine.get_status())

        # ==================== Arming ====================

        @self.app.route('/api/arm', methods=['POST'])
        def arm():
            """Arm the drone"""
            success = self.fc.arm()
            return jsonify({
                'success': success,
                'message': 'Armed' if success else 'Arming failed'
            })

        @self.app.route('/api/disarm', methods=['POST'])
        def disarm():
            """Disarm the drone"""
            success = self.fc.disarm()
            return jsonify({
                'success': success,
                'message': 'Disarmed' if success else 'Disarm failed'
            })

        # ==================== Flight Control ====================

        @self.app.route('/api/takeoff', methods=['POST'])
        def takeoff():
            """Initiate takeoff"""
            data = request.get_json() or {}
            altitude = data.get('altitude', 3.0)

            success = self.fc.takeoff(altitude)
            return jsonify({
                'success': success,
                'message': f'Taking off to {altitude}m' if success else 'Takeoff failed'
            })

        @self.app.route('/api/land', methods=['POST'])
        def land():
            """Initiate landing"""
            success = self.fc.land()
            return jsonify({
                'success': success,
                'message': 'Landing' if success else 'Landing failed'
            })

        @self.app.route('/api/hold', methods=['POST'])
        def hold():
            """Hold current position"""
            success = self.fc.hold_position()
            return jsonify({
                'success': success,
                'message': 'Position hold' if success else 'Hold failed'
            })

        @self.app.route('/api/rth', methods=['POST'])
        def rth():
            """Return to home"""
            success = self.fc.return_to_home()
            return jsonify({
                'success': success,
                'message': 'Returning home' if success else 'RTH failed'
            })

        # ==================== Navigation ====================

        @self.app.route('/api/goto', methods=['POST'])
        def goto():
            """Go to GPS position"""
            data = request.get_json()
            if not data:
                return jsonify({'success': False, 'message': 'No data provided'}), 400

            lat = data.get('latitude') or data.get('lat')
            lon = data.get('longitude') or data.get('lon')
            alt = data.get('altitude') or data.get('alt', 10.0)

            if lat is None or lon is None:
                return jsonify({
                    'success': False,
                    'message': 'Missing latitude or longitude'
                }), 400

            success = self.fc.goto(lat, lon, alt)
            return jsonify({
                'success': success,
                'message': f'Flying to {lat}, {lon}' if success else 'Goto failed'
            })

        # ==================== Missions (v1.0 API) ====================

        @self.app.route('/api/missions', methods=['GET'])
        def list_missions():
            """
            List all stored missions

            Returns:
                {missions: [{uuid, name, action_count, ...}, ...]}
            """
            missions = self.mission_store.list_all()
            return jsonify({'missions': missions})

        @self.app.route('/api/missions', methods=['POST'])
        def create_mission():
            """
            Upload a new mission

            Request body: Mission JSON (without UUID)
            Returns:
                {uuid, name, valid: true} or error
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
                return jsonify({
                    'error': str(e),
                    'valid': False
                }), 400

        @self.app.route('/api/missions/<mission_uuid>', methods=['GET'])
        def get_mission(mission_uuid: str):
            """
            Get mission details

            Returns:
                Full mission JSON with UUID
            """
            mission = self.mission_store.get(mission_uuid)
            if not mission:
                return jsonify({'error': 'Mission not found'}), 404

            result = mission.to_dict()
            result['uuid'] = mission.uuid
            return jsonify(result)

        @self.app.route('/api/missions/<mission_uuid>', methods=['DELETE'])
        def delete_mission(mission_uuid: str):
            """
            Delete a mission

            Returns:
                {success: true} or 404
            """
            # Prevent deletion of active mission
            if (self.mission_executor.mission and
                self.mission_executor.mission.uuid == mission_uuid and
                self.mission_executor.state.name in ('RUNNING', 'PAUSED')):
                return jsonify({
                    'error': 'Cannot delete active mission'
                }), 409

            if self.mission_store.delete(mission_uuid):
                return jsonify({'success': True})
            return jsonify({'error': 'Mission not found'}), 404

        # ==================== Active Mission Control ====================

        @self.app.route('/api/missions/active', methods=['GET'])
        def get_active_mission():
            """
            Get active mission status

            Returns:
                Detailed status of current mission execution
            """
            status = self.mission_executor.get_status_dict()

            # Add position info if available
            if self.fc._last_gps_fix:
                status['position'] = {
                    'lat': self.fc._last_gps_fix.latitude,
                    'lon': self.fc._last_gps_fix.longitude,
                    'alt': self.fc.alt_controller.current_altitude
                }

            # Calculate distance to next waypoint if goto action
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
                    if status.get('target_speed', 0) > 0:
                        status['eta_seconds'] = dist / status['target_speed']

            return jsonify(status)

        @self.app.route('/api/missions/<mission_uuid>/start', methods=['POST'])
        def start_mission(mission_uuid: str):
            """
            Start a mission

            Returns:
                {success: true} or error
            """
            # Check if another mission is running
            if self.mission_executor.state.name in ('RUNNING', 'PAUSED'):
                return jsonify({
                    'error': 'Another mission is active. Stop it first.'
                }), 409

            # Load mission
            mission = self.mission_store.get(mission_uuid)
            if not mission:
                return jsonify({'error': 'Mission not found'}), 404

            # Load into executor
            self.mission_executor.load(mission)

            # Set home from current position
            if self.fc._last_gps_fix:
                self.mission_executor.set_home(
                    self.fc._last_gps_fix.latitude,
                    self.fc._last_gps_fix.longitude,
                    self.fc.alt_controller.current_altitude
                )

            # Start execution
            if self.mission_executor.start():
                return jsonify({
                    'success': True,
                    'message': f"Mission '{mission.name}' started"
                })

            return jsonify({
                'success': False,
                'error': 'Failed to start mission'
            }), 400

        @self.app.route('/api/missions/active/pause', methods=['POST'])
        def pause_mission():
            """Pause active mission"""
            if self.mission_executor.state.name != 'RUNNING':
                return jsonify({'error': 'No running mission'}), 400

            self.mission_executor.pause()
            return jsonify({
                'success': True,
                'message': 'Mission paused'
            })

        @self.app.route('/api/missions/active/resume', methods=['POST'])
        def resume_mission():
            """Resume paused mission"""
            if self.mission_executor.state.name != 'PAUSED':
                return jsonify({'error': 'Mission not paused'}), 400

            self.mission_executor.resume()
            return jsonify({
                'success': True,
                'message': 'Mission resumed'
            })

        @self.app.route('/api/missions/active/stop', methods=['POST'])
        def stop_mission():
            """Stop active mission"""
            if self.mission_executor.state.name not in ('RUNNING', 'PAUSED'):
                return jsonify({'error': 'No active mission'}), 400

            self.mission_executor.stop()

            # Command drone to hold position
            self.fc.hold_position()

            return jsonify({
                'success': True,
                'message': 'Mission stopped'
            })

        @self.app.route('/api/missions/active/skip', methods=['POST'])
        def skip_action():
            """Skip current action"""
            if self.mission_executor.state.name != 'RUNNING':
                return jsonify({'error': 'No running mission'}), 400

            self.mission_executor.skip()
            return jsonify({
                'success': True,
                'message': 'Action skipped',
                'new_action_index': self.mission_executor.current_action_index
            })

        @self.app.route('/api/missions/active/goto/<int:index>', methods=['POST'])
        def goto_action(index: int):
            """Jump to specific action"""
            if self.mission_executor.state.name not in ('RUNNING', 'PAUSED'):
                return jsonify({'error': 'No active mission'}), 400

            if self.mission_executor.goto_action(index):
                return jsonify({
                    'success': True,
                    'message': f'Jumped to action {index}'
                })

            return jsonify({
                'error': f'Invalid action index {index}'
            }), 400

        # ==================== Configuration ====================

        @self.app.route('/api/config', methods=['GET'])
        def get_config():
            """Get configuration"""
            from ..config import get_config
            config = get_config()

            result = {}
            for section in ['serial', 'gps', 'navigation', 'altitude', 'failsafe', 'interface']:
                sect = getattr(config, section)
                result[section] = {k: v for k, v in sect.__dict__.items()}

            return jsonify(result)

        @self.app.route('/api/config', methods=['POST'])
        def set_config():
            """Update configuration"""
            from ..config import get_config
            config = get_config()
            data = request.get_json()

            if not data:
                return jsonify({'success': False, 'message': 'No data'}), 400

            config._update_from_dict(data)
            return jsonify({'success': True, 'message': 'Configuration updated'})

        # ==================== Legacy Mission API (deprecated) ====================

        @self.app.route('/api/mission', methods=['GET'])
        def get_mission_legacy():
            """Get current mission status (deprecated, use /api/missions/active)"""
            return jsonify(self.mission_executor.get_status_dict())

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
        # Flask doesn't have a clean shutdown mechanism in threaded mode
        pass

    def get_executor(self) -> MissionExecutor:
        """Get mission executor for integration with flight controller"""
        return self.mission_executor
