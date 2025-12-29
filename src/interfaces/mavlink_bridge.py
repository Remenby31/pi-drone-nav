"""
MAVLink Bridge for Pi Drone Navigation

Provides MAVLink protocol compatibility for ground control stations
like QGroundControl, Mission Planner, etc.
"""

import socket
import struct
import threading
import time
from typing import TYPE_CHECKING, Optional
import logging

try:
    from pymavlink import mavutil
    from pymavlink.dialects.v20 import common as mavlink
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False

if TYPE_CHECKING:
    from ..flight.flight_controller import FlightController

logger = logging.getLogger(__name__)


class MAVLinkBridge:
    """
    MAVLink protocol bridge

    Translates between Pi Drone Navigation and MAVLink protocol
    for compatibility with standard ground control stations.

    Implements:
    - HEARTBEAT
    - GLOBAL_POSITION_INT
    - ATTITUDE
    - GPS_RAW_INT
    - SYS_STATUS
    - Command handling (arm, takeoff, land, waypoints)
    """

    # MAVLink system types
    MAV_TYPE_QUADROTOR = 2
    MAV_AUTOPILOT_GENERIC = 0
    MAV_STATE_ACTIVE = 4
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
    MAV_MODE_FLAG_SAFETY_ARMED = 128

    def __init__(self, flight_controller: 'FlightController',
                 port: int = 14550, system_id: int = 1):
        """
        Initialize MAVLink bridge

        Args:
            flight_controller: FlightController instance
            port: UDP port for MAVLink
            system_id: MAVLink system ID
        """
        if not MAVLINK_AVAILABLE:
            raise ImportError("pymavlink not installed")

        self.fc = flight_controller
        self.port = port
        self.system_id = system_id
        self.component_id = 1

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._socket: Optional[socket.socket] = None
        self._mav: Optional[mavutil.mavlink_connection] = None

        # MAVLink state
        self._sequence = 0
        self._last_heartbeat = 0

        # Ground station address
        self._gcs_addr = None

    def start(self):
        """Start MAVLink bridge"""
        if self._running:
            return

        # Create UDP socket
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind(('0.0.0.0', self.port))
        self._socket.setblocking(False)

        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

        logger.info(f"MAVLink bridge started on UDP:{self.port}")

    def stop(self):
        """Stop MAVLink bridge"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._socket:
            self._socket.close()

    def _run_loop(self):
        """Main MAVLink loop"""
        last_telemetry = 0
        last_heartbeat = 0

        while self._running:
            try:
                # Receive incoming messages
                try:
                    data, addr = self._socket.recvfrom(1024)
                    self._gcs_addr = addr
                    self._handle_incoming(data)
                except BlockingIOError:
                    pass

                current_time = time.time()

                # Send heartbeat at 1Hz
                if current_time - last_heartbeat >= 1.0:
                    self._send_heartbeat()
                    last_heartbeat = current_time

                # Send telemetry at 10Hz
                if current_time - last_telemetry >= 0.1:
                    self._send_telemetry()
                    last_telemetry = current_time

                time.sleep(0.01)  # 100Hz loop

            except Exception as e:
                logger.error(f"MAVLink error: {e}")
                time.sleep(0.1)

    def _handle_incoming(self, data: bytes):
        """Handle incoming MAVLink message"""
        if not MAVLINK_AVAILABLE:
            return

        try:
            # Parse MAVLink message
            mav = mavlink.MAVLink(None)
            msg = mav.parse_char(data)

            if msg:
                self._process_message(msg)

        except Exception as e:
            logger.debug(f"Failed to parse MAVLink: {e}")

    def _process_message(self, msg):
        """Process parsed MAVLink message"""
        msg_type = msg.get_type()

        if msg_type == 'HEARTBEAT':
            # GCS heartbeat - just acknowledge
            pass

        elif msg_type == 'COMMAND_LONG':
            self._handle_command_long(msg)

        elif msg_type == 'SET_MODE':
            self._handle_set_mode(msg)

        elif msg_type == 'MISSION_COUNT':
            self._handle_mission_count(msg)

        elif msg_type == 'MISSION_ITEM':
            self._handle_mission_item(msg)

        elif msg_type == 'MISSION_REQUEST_LIST':
            self._handle_mission_request_list(msg)

    def _handle_command_long(self, msg):
        """Handle COMMAND_LONG message"""
        command = msg.command

        # MAV_CMD_COMPONENT_ARM_DISARM (400)
        if command == 400:
            if msg.param1 == 1:
                self.fc.arm()
            else:
                self.fc.disarm()

        # MAV_CMD_NAV_TAKEOFF (22)
        elif command == 22:
            altitude = msg.param7
            self.fc.takeoff(altitude)

        # MAV_CMD_NAV_LAND (21)
        elif command == 21:
            self.fc.land()

        # MAV_CMD_NAV_RETURN_TO_LAUNCH (20)
        elif command == 20:
            self.fc.return_to_home()

        # MAV_CMD_DO_SET_MODE (176)
        elif command == 176:
            mode = int(msg.param1)
            if mode == 4:  # GUIDED
                self.fc.hold_position()

    def _handle_set_mode(self, msg):
        """Handle SET_MODE message"""
        # Simplified mode handling
        pass

    def _handle_mission_count(self, msg):
        """Handle MISSION_COUNT - GCS sending mission"""
        # TODO: Implement mission upload
        pass

    def _handle_mission_item(self, msg):
        """Handle MISSION_ITEM message"""
        # TODO: Implement mission item reception
        pass

    def _handle_mission_request_list(self, msg):
        """Handle MISSION_REQUEST_LIST - GCS requesting mission"""
        # TODO: Send current mission
        pass

    def _send(self, data: bytes):
        """Send MAVLink data to GCS"""
        if self._gcs_addr and self._socket:
            try:
                self._socket.sendto(data, self._gcs_addr)
            except Exception as e:
                logger.debug(f"Failed to send MAVLink: {e}")

    def _send_heartbeat(self):
        """Send HEARTBEAT message"""
        if not MAVLINK_AVAILABLE:
            return

        # Determine mode flags
        base_mode = self.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if self.fc._armed:
            base_mode |= self.MAV_MODE_FLAG_SAFETY_ARMED

        # Create heartbeat
        mav = mavlink.MAVLink(None, srcSystem=self.system_id, srcComponent=self.component_id)

        msg = mav.heartbeat_encode(
            type=self.MAV_TYPE_QUADROTOR,
            autopilot=self.MAV_AUTOPILOT_GENERIC,
            base_mode=base_mode,
            custom_mode=0,
            system_status=self.MAV_STATE_ACTIVE
        )

        self._send(msg.pack(mav))

    def _send_telemetry(self):
        """Send telemetry messages"""
        if not MAVLINK_AVAILABLE:
            return

        telemetry = self.fc.get_telemetry()

        mav = mavlink.MAVLink(None, srcSystem=self.system_id, srcComponent=self.component_id)

        # GLOBAL_POSITION_INT
        pos = telemetry['position']
        msg = mav.global_position_int_encode(
            time_boot_ms=int(time.time() * 1000) & 0xFFFFFFFF,
            lat=int(pos['lat'] * 1e7),
            lon=int(pos['lon'] * 1e7),
            alt=int(pos['alt'] * 1000),
            relative_alt=int(pos['alt'] * 1000),
            vx=0, vy=0, vz=0,  # TODO: Add velocity
            hdg=int(telemetry['attitude']['yaw'] * 100)
        )
        self._send(msg.pack(mav))

        # ATTITUDE
        att = telemetry['attitude']
        import math
        msg = mav.attitude_encode(
            time_boot_ms=int(time.time() * 1000) & 0xFFFFFFFF,
            roll=math.radians(att['roll']),
            pitch=math.radians(att['pitch']),
            yaw=math.radians(att['yaw']),
            rollspeed=0,
            pitchspeed=0,
            yawspeed=0
        )
        self._send(msg.pack(mav))

        # GPS_RAW_INT
        msg = mav.gps_raw_int_encode(
            time_usec=int(time.time() * 1e6),
            fix_type=3 if pos['satellites'] >= 6 else 0,
            lat=int(pos['lat'] * 1e7),
            lon=int(pos['lon'] * 1e7),
            alt=int(pos['alt'] * 1000),
            eph=100,
            epv=100,
            vel=0,
            cog=int(att['yaw'] * 100),
            satellites_visible=pos['satellites']
        )
        self._send(msg.pack(mav))

        # SYS_STATUS
        msg = mav.sys_status_encode(
            onboard_control_sensors_present=0,
            onboard_control_sensors_enabled=0,
            onboard_control_sensors_health=0,
            load=0,
            voltage_battery=0,
            current_battery=-1,
            battery_remaining=-1,
            drop_rate_comm=0,
            errors_comm=0,
            errors_count1=0,
            errors_count2=0,
            errors_count3=0,
            errors_count4=0
        )
        self._send(msg.pack(mav))
