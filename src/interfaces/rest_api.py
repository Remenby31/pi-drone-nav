"""
REST API for Pi Drone Navigation

Provides HTTP endpoints for drone control and monitoring.
"""

import json
import threading
from typing import TYPE_CHECKING, Optional
import logging

try:
    from flask import Flask, request, jsonify
    from flask_cors import CORS
    FLASK_AVAILABLE = True
except ImportError:
    FLASK_AVAILABLE = False

if TYPE_CHECKING:
    from ..flight.flight_controller import FlightController

logger = logging.getLogger(__name__)


def create_api_server(flight_controller: 'FlightController',
                      port: int = 8080,
                      host: str = '0.0.0.0') -> Optional['APIServer']:
    """
    Create and start REST API server

    Args:
        flight_controller: FlightController instance
        port: HTTP port
        host: Host address

    Returns:
        APIServer instance or None if Flask not available
    """
    if not FLASK_AVAILABLE:
        logger.warning("Flask not installed - REST API disabled")
        return None

    server = APIServer(flight_controller, port, host)
    server.start()
    return server


class APIServer:
    """REST API Server wrapper"""

    def __init__(self, flight_controller: 'FlightController',
                 port: int = 8080, host: str = '0.0.0.0'):
        self.fc = flight_controller
        self.port = port
        self.host = host

        self.app = Flask(__name__)
        CORS(self.app)  # Enable CORS for web clients

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

        # ==================== Mission ====================

        @self.app.route('/api/mission', methods=['GET'])
        def get_mission():
            """Get current mission status"""
            if self.fc.waypoint_nav:
                return jsonify(self.fc.waypoint_nav.get_status())
            return jsonify({'mission': None})

        @self.app.route('/api/mission', methods=['POST'])
        def load_mission():
            """Load a mission"""
            data = request.get_json()
            if not data:
                return jsonify({'success': False, 'message': 'No data'}), 400

            try:
                from ..navigation.waypoint_navigator import Mission, Waypoint, WaypointAction

                waypoints = []
                for wp_data in data.get('waypoints', []):
                    waypoints.append(Waypoint(
                        latitude=wp_data['latitude'],
                        longitude=wp_data['longitude'],
                        altitude=wp_data.get('altitude', 10.0),
                        action=WaypointAction[wp_data.get('action', 'FLY_THROUGH').upper()],
                        radius=wp_data.get('radius', 2.0)
                    ))

                mission = Mission(
                    name=data.get('name', 'API Mission'),
                    waypoints=waypoints,
                    default_speed=data.get('default_speed', 10.0),
                    return_to_home=data.get('return_to_home', True)
                )

                self.fc.waypoint_nav.load_mission(mission)

                return jsonify({
                    'success': True,
                    'message': f'Mission loaded with {len(waypoints)} waypoints'
                })

            except Exception as e:
                return jsonify({
                    'success': False,
                    'message': str(e)
                }), 400

        @self.app.route('/api/mission/start', methods=['POST'])
        def start_mission():
            """Start loaded mission"""
            if self.fc.waypoint_nav and self.fc.waypoint_nav.mission:
                success = self.fc.start_mission(self.fc.waypoint_nav.mission)
                return jsonify({
                    'success': success,
                    'message': 'Mission started' if success else 'Start failed'
                })
            return jsonify({'success': False, 'message': 'No mission loaded'})

        @self.app.route('/api/mission/pause', methods=['POST'])
        def pause_mission():
            """Pause mission"""
            if self.fc.waypoint_nav:
                self.fc.waypoint_nav.pause()
                return jsonify({'success': True, 'message': 'Mission paused'})
            return jsonify({'success': False, 'message': 'No mission'})

        @self.app.route('/api/mission/resume', methods=['POST'])
        def resume_mission():
            """Resume mission"""
            if self.fc.waypoint_nav:
                self.fc.waypoint_nav.resume()
                return jsonify({'success': True, 'message': 'Mission resumed'})
            return jsonify({'success': False, 'message': 'No mission'})

        @self.app.route('/api/mission/abort', methods=['POST'])
        def abort_mission():
            """Abort mission"""
            if self.fc.waypoint_nav:
                self.fc.waypoint_nav.abort()
                return jsonify({'success': True, 'message': 'Mission aborted'})
            return jsonify({'success': False, 'message': 'No mission'})

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

    def stop(self):
        """Stop API server"""
        # Flask doesn't have a clean shutdown mechanism in threaded mode
        pass
