#!/usr/bin/env python3
"""
Simulation script for Pi Drone Navigation

This script provides a complete simulation environment using:
- Betaflight SITL (Software In The Loop)
- Simulated GPS data
- Flight physics simulation

Usage:
    python scripts/simulate.py [options]

Options:
    --sitl          Start Betaflight SITL automatically
    --no-physics    Use simple kinematics (no physics)
    --headless      Run without visualization
    --mission FILE  Load and execute mission file
"""

import argparse
import socket
import struct
import threading
import time
import math
import sys
import os
from dataclasses import dataclass
from typing import Optional, Tuple

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


@dataclass
class DroneState:
    """Simulated drone state"""
    # Position (NED from origin)
    north: float = 0.0
    east: float = 0.0
    down: float = 0.0

    # Velocity (NED)
    vel_north: float = 0.0
    vel_east: float = 0.0
    vel_down: float = 0.0

    # Attitude (degrees)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    # Angular rates (deg/s)
    roll_rate: float = 0.0
    pitch_rate: float = 0.0
    yaw_rate: float = 0.0

    # State
    armed: bool = False
    motors_running: bool = False

    # Origin GPS
    origin_lat: float = 48.8566
    origin_lon: float = 2.3522
    origin_alt: float = 0.0


class PhysicsSimulator:
    """Simple quadcopter physics simulation"""

    def __init__(self):
        # Physical parameters
        self.mass = 1.5  # kg
        self.gravity = 9.81  # m/s²
        self.drag_coef = 0.3
        self.max_thrust = 25.0  # N (total for all motors)

        # Time step
        self.dt = 0.002  # 500Hz physics

        # State
        self.state = DroneState()

        # Lock for thread safety
        self._lock = threading.Lock()

    def update(self, roll_cmd: float, pitch_cmd: float, yaw_rate_cmd: float,
               throttle_cmd: float) -> DroneState:
        """
        Update physics simulation

        Args:
            roll_cmd: Roll angle command (-30 to 30 degrees)
            pitch_cmd: Pitch angle command (-30 to 30 degrees)
            yaw_rate_cmd: Yaw rate command (deg/s)
            throttle_cmd: Throttle (0.0 to 1.0)

        Returns:
            Updated drone state
        """
        with self._lock:
            if not self.state.armed:
                return self.state

            # Attitude response (simplified first-order)
            attitude_tau = 0.1  # time constant
            self.state.roll += (roll_cmd - self.state.roll) * self.dt / attitude_tau
            self.state.pitch += (pitch_cmd - self.state.pitch) * self.dt / attitude_tau
            self.state.yaw += yaw_rate_cmd * self.dt

            # Wrap yaw
            self.state.yaw = self.state.yaw % 360

            # Calculate thrust vector
            thrust = throttle_cmd * self.max_thrust

            # Convert angles to radians
            roll_rad = math.radians(self.state.roll)
            pitch_rad = math.radians(self.state.pitch)
            yaw_rad = math.radians(self.state.yaw)

            # Thrust in body frame, then rotate to NED
            # Simplified: assume small angles
            thrust_north = thrust * (-math.sin(pitch_rad))
            thrust_east = thrust * math.sin(roll_rad) * math.cos(pitch_rad)
            thrust_down = -thrust * math.cos(roll_rad) * math.cos(pitch_rad) + self.mass * self.gravity

            # Accelerations (F = ma)
            accel_north = thrust_north / self.mass
            accel_east = thrust_east / self.mass
            accel_down = thrust_down / self.mass

            # Add drag
            speed = math.sqrt(self.state.vel_north**2 + self.state.vel_east**2 + self.state.vel_down**2)
            if speed > 0.1:
                drag = self.drag_coef * speed * speed / self.mass
                accel_north -= drag * self.state.vel_north / speed
                accel_east -= drag * self.state.vel_east / speed
                accel_down -= drag * self.state.vel_down / speed

            # Integrate velocity
            self.state.vel_north += accel_north * self.dt
            self.state.vel_east += accel_east * self.dt
            self.state.vel_down += accel_down * self.dt

            # Integrate position
            self.state.north += self.state.vel_north * self.dt
            self.state.east += self.state.vel_east * self.dt
            self.state.down += self.state.vel_down * self.dt

            # Ground constraint
            if self.state.down > 0:
                self.state.down = 0
                self.state.vel_down = 0
                if not self.state.motors_running:
                    self.state.vel_north *= 0.9  # friction
                    self.state.vel_east *= 0.9

            return self.state

    def arm(self):
        """Arm the drone"""
        with self._lock:
            self.state.armed = True
            self.state.motors_running = True

    def disarm(self):
        """Disarm the drone"""
        with self._lock:
            self.state.armed = False
            self.state.motors_running = False

    def get_gps_position(self) -> Tuple[float, float, float]:
        """Get GPS position (lat, lon, alt)"""
        with self._lock:
            # Convert NED to GPS
            lat = self.state.origin_lat + self.state.north / 111000.0
            lon = self.state.origin_lon + self.state.east / (111000.0 * math.cos(math.radians(self.state.origin_lat)))
            alt = self.state.origin_alt - self.state.down

            return lat, lon, alt


class SimulatedMSPServer:
    """Simulated MSP server (mimics Betaflight SITL)"""

    def __init__(self, physics: PhysicsSimulator, port: int = 5761):
        self.physics = physics
        self.port = port
        self._socket: Optional[socket.socket] = None
        self._running = False
        self._client: Optional[socket.socket] = None

        # RC channels
        self.rc_channels = [1500] * 16
        self.rc_channels[2] = 1000  # Throttle at minimum

    def start(self):
        """Start MSP server"""
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind(('127.0.0.1', self.port))
        self._socket.listen(1)
        self._socket.settimeout(1.0)

        self._running = True

        # Accept thread
        threading.Thread(target=self._accept_loop, daemon=True).start()

        print(f"Simulated MSP server listening on port {self.port}")

    def stop(self):
        """Stop MSP server"""
        self._running = False
        if self._client:
            self._client.close()
        if self._socket:
            self._socket.close()

    def _accept_loop(self):
        """Accept client connections"""
        while self._running:
            try:
                self._client, addr = self._socket.accept()
                print(f"MSP client connected from {addr}")
                threading.Thread(target=self._handle_client, daemon=True).start()
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    print(f"Accept error: {e}")

    def _handle_client(self):
        """Handle MSP client connection"""
        buffer = b''

        while self._running and self._client:
            try:
                data = self._client.recv(1024)
                if not data:
                    break

                buffer += data

                # Parse MSP messages
                while len(buffer) >= 6:
                    # Look for $M< header
                    if buffer[:3] != b'$M<':
                        buffer = buffer[1:]
                        continue

                    size = buffer[3]
                    cmd = buffer[4]

                    total_len = 6 + size
                    if len(buffer) < total_len:
                        break

                    payload = buffer[5:5+size]
                    # checksum = buffer[5+size]

                    self._handle_command(cmd, payload)
                    buffer = buffer[total_len:]

            except Exception as e:
                print(f"Client error: {e}")
                break

        print("MSP client disconnected")

    def _handle_command(self, cmd: int, payload: bytes):
        """Handle MSP command"""
        response = None

        if cmd == 1:  # MSP_API_VERSION
            response = bytes([2, 5, 0])  # API 2.5.0

        elif cmd == 2:  # MSP_FC_VARIANT
            response = b'BTFL'

        elif cmd == 101:  # MSP_STATUS
            # mode flags, cycle time, errors, etc.
            response = struct.pack('<HHHIBBH', 500, 0, 0, 1, 0, 0, 0)

        elif cmd == 108:  # MSP_ATTITUDE
            state = self.physics.state
            roll = int(state.roll * 10)
            pitch = int(state.pitch * 10)
            yaw = int(state.yaw)
            response = struct.pack('<hhh', roll, pitch, yaw)

        elif cmd == 109:  # MSP_ALTITUDE
            state = self.physics.state
            alt_cm = int(-state.down * 100)
            vario = int(-state.vel_down * 100)
            response = struct.pack('<ih', alt_cm, vario)

        elif cmd == 106:  # MSP_RAW_GPS
            lat, lon, alt = self.physics.get_gps_position()
            state = self.physics.state
            speed = int(math.sqrt(state.vel_north**2 + state.vel_east**2) * 100)
            response = struct.pack('<BBiiHHH',
                                  3,  # fix
                                  12,  # sats
                                  int(lat * 1e7),
                                  int(lon * 1e7),
                                  int(alt),
                                  speed,
                                  int(state.yaw * 10))

        elif cmd == 200:  # MSP_SET_RAW_RC
            # Parse RC channels
            num_channels = len(payload) // 2
            channels = struct.unpack(f'<{num_channels}H', payload)

            for i, val in enumerate(channels):
                if i < len(self.rc_channels):
                    self.rc_channels[i] = val

            # Empty response for SET commands
            response = b''

            # Update physics based on RC
            self._apply_rc_to_physics()

        if response is not None:
            self._send_response(cmd, response)

    def _send_response(self, cmd: int, data: bytes):
        """Send MSP response"""
        if not self._client:
            return

        size = len(data)
        checksum = size ^ cmd
        for b in data:
            checksum ^= b

        msg = b'$M>' + bytes([size, cmd]) + data + bytes([checksum])

        try:
            self._client.send(msg)
        except Exception as e:
            print(f"Send error: {e}")

    def _apply_rc_to_physics(self):
        """Apply RC commands to physics simulation"""
        # Convert RC values to commands
        # Roll: 1000-2000 -> -30 to +30 degrees
        roll_cmd = (self.rc_channels[0] - 1500) / 500 * 30

        # Pitch: 1000-2000 -> -30 to +30 degrees
        pitch_cmd = (self.rc_channels[1] - 1500) / 500 * 30

        # Throttle: 1000-2000 -> 0 to 1
        throttle = (self.rc_channels[2] - 1000) / 1000

        # Yaw rate: 1000-2000 -> -180 to +180 deg/s
        yaw_rate = (self.rc_channels[3] - 1500) / 500 * 180

        # Check arm switch (AUX1 / channel 5)
        if self.rc_channels[4] > 1700:
            if not self.physics.state.armed:
                self.physics.arm()
                print("ARMED")
        else:
            if self.physics.state.armed:
                self.physics.disarm()
                print("DISARMED")

        # Update physics
        if self.physics.state.armed:
            self.physics.update(roll_cmd, pitch_cmd, yaw_rate, throttle)


class SimulatedGPSServer:
    """Simulated GPS server (UBX protocol)"""

    def __init__(self, physics: PhysicsSimulator, port: int = 5762):
        self.physics = physics
        self.port = port
        self._socket: Optional[socket.socket] = None
        self._running = False

    def start(self):
        """Start GPS server"""
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind(('127.0.0.1', self.port))
        self._socket.listen(1)
        self._socket.settimeout(1.0)

        self._running = True
        threading.Thread(target=self._server_loop, daemon=True).start()

        print(f"Simulated GPS server listening on port {self.port}")

    def stop(self):
        """Stop GPS server"""
        self._running = False
        if self._socket:
            self._socket.close()

    def _server_loop(self):
        """Server loop"""
        while self._running:
            try:
                client, addr = self._socket.accept()
                print(f"GPS client connected from {addr}")
                threading.Thread(target=self._handle_client, args=(client,), daemon=True).start()
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    print(f"GPS accept error: {e}")

    def _handle_client(self, client: socket.socket):
        """Send NAV-PVT messages to client"""
        try:
            while self._running:
                msg = self._build_nav_pvt()
                client.send(msg)
                time.sleep(0.1)  # 10Hz
        except Exception as e:
            print(f"GPS client error: {e}")
        finally:
            client.close()

    def _build_nav_pvt(self) -> bytes:
        """Build UBX NAV-PVT message"""
        lat, lon, alt = self.physics.get_gps_position()
        state = self.physics.state

        # NAV-PVT payload (92 bytes)
        payload = bytearray(92)

        # iTOW
        struct.pack_into('<I', payload, 0, int(time.time() * 1000) & 0xFFFFFFFF)
        # Year, month, day, hour, min, sec
        import datetime
        now = datetime.datetime.utcnow()
        struct.pack_into('<HBBBBB', payload, 4,
                        now.year, now.month, now.day,
                        now.hour, now.minute, now.second)
        # valid
        payload[11] = 0x07
        # fixType
        payload[20] = 3
        # flags
        payload[21] = 0x01
        # numSV
        payload[23] = 12
        # lon, lat (1e-7 deg)
        struct.pack_into('<ii', payload, 24, int(lon * 1e7), int(lat * 1e7))
        # height, hMSL (mm)
        struct.pack_into('<ii', payload, 32, int(alt * 1000), int(alt * 1000))
        # velN, velE, velD (mm/s)
        struct.pack_into('<iii', payload, 48,
                        int(state.vel_north * 1000),
                        int(state.vel_east * 1000),
                        int(state.vel_down * 1000))

        # Build complete message
        msg_class = 0x01
        msg_id = 0x07

        # Checksum
        ck_a = 0
        ck_b = 0
        header = bytes([msg_class, msg_id]) + struct.pack('<H', len(payload))
        for b in header + payload:
            ck_a = (ck_a + b) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF

        return b'\xb5\x62' + header + bytes(payload) + bytes([ck_a, ck_b])


def run_simulation(args):
    """Run the simulation"""
    print("=" * 60)
    print("  Pi Drone Navigation - Simulation Mode")
    print("=" * 60)
    print()

    # Create physics simulator
    physics = PhysicsSimulator()
    physics.state.origin_lat = args.lat
    physics.state.origin_lon = args.lon

    # Create simulated servers
    msp_server = SimulatedMSPServer(physics, port=args.msp_port)
    gps_server = SimulatedGPSServer(physics, port=args.gps_port)

    # Start servers
    msp_server.start()
    gps_server.start()

    print()
    print("Simulation running. Press Ctrl+C to stop.")
    print()
    print("Connect Pi Drone Navigation with:")
    print(f"  --msp-port tcp:127.0.0.1:{args.msp_port}")
    print(f"  --gps-port tcp:127.0.0.1:{args.gps_port}")
    print()

    # Physics loop
    physics_thread = threading.Thread(target=_physics_loop, args=(physics, msp_server), daemon=True)
    physics_thread.start()

    # Status display
    try:
        while True:
            state = physics.state
            lat, lon, alt = physics.get_gps_position()

            print(f"\rArmed: {state.armed:5} | "
                  f"Pos: N{state.north:6.1f} E{state.east:6.1f} D{state.down:6.1f} | "
                  f"Vel: {math.sqrt(state.vel_north**2 + state.vel_east**2):5.1f} m/s | "
                  f"Alt: {alt:5.1f}m | "
                  f"Att: R{state.roll:5.1f} P{state.pitch:5.1f} Y{state.yaw:5.1f}",
                  end='', flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n\nStopping simulation...")

    msp_server.stop()
    gps_server.stop()


def _physics_loop(physics: PhysicsSimulator, msp_server: SimulatedMSPServer):
    """Physics update loop"""
    last_time = time.time()

    while True:
        current_time = time.time()
        dt = current_time - last_time

        if dt >= physics.dt:
            last_time = current_time

            if physics.state.armed:
                # Get current RC from MSP server
                rc = msp_server.rc_channels
                roll_cmd = (rc[0] - 1500) / 500 * 30
                pitch_cmd = (rc[1] - 1500) / 500 * 30
                throttle = (rc[2] - 1000) / 1000
                yaw_rate = (rc[3] - 1500) / 500 * 180

                physics.update(roll_cmd, pitch_cmd, yaw_rate, throttle)

        time.sleep(0.001)


def main():
    parser = argparse.ArgumentParser(description='Pi Drone Navigation Simulator')
    parser.add_argument('--msp-port', type=int, default=5761,
                       help='MSP server port (default: 5761)')
    parser.add_argument('--gps-port', type=int, default=5762,
                       help='GPS server port (default: 5762)')
    parser.add_argument('--lat', type=float, default=48.8566,
                       help='Starting latitude (default: 48.8566)')
    parser.add_argument('--lon', type=float, default=2.3522,
                       help='Starting longitude (default: 2.3522)')
    parser.add_argument('--mission', type=str,
                       help='Mission file to load')

    args = parser.parse_args()
    run_simulation(args)


if __name__ == '__main__':
    main()
