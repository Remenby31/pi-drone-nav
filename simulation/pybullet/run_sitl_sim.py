#!/usr/bin/env python3
"""
Complete SITL + PyBullet Simulation

This script:
1. Launches Betaflight SITL
2. Starts PyBullet physics bridge
3. Connects MSP driver via TCP
4. Runs flight control loop

Usage:
    python simulation/pybullet/run_sitl_sim.py
"""

import sys
import time
import signal
import subprocess
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from simulation.pybullet.sitl_bridge import SITLBridge
from simulation.pybullet.tcp_serial import TCPSerial
from src.drivers.msp import MSPClient


class SITLSimulation:
    """Complete SITL + PyBullet simulation environment"""

    def __init__(
        self,
        sitl_path: str = None,
        msp_port: int = 5761,
        gui: bool = True
    ):
        # Find SITL binary
        if sitl_path is None:
            # Try different possible names
            base = Path.home() / "projects/betaflight/obj"
            candidates = list(base.glob("betaflight_*SITL*"))
            if candidates:
                sitl_path = candidates[0]
            else:
                sitl_path = base / "betaflight_SITL"
        self.sitl_path = Path(sitl_path)

        self.msp_port = msp_port
        self.gui = gui

        # Components
        self.sitl_process: subprocess.Popen = None
        self.bridge: SITLBridge = None
        self.tcp_serial: TCPSerial = None
        self.msp: MSPClient = None

        # State
        self.running = False
        self.armed = False

        # Control
        self.rc_channels = [1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500]
        # [Roll, Pitch, Throttle, Yaw, AUX1(arm), AUX2, AUX3, AUX4]

    def start(self) -> bool:
        """Start all simulation components"""
        print("=" * 60)
        print("  Betaflight SITL + PyBullet Simulation")
        print("=" * 60)
        print()

        # 1. Start SITL
        print("[1/3] Starting Betaflight SITL...")
        if not self._start_sitl():
            return False
        print("      SITL started")

        # 2. Start PyBullet bridge
        print("[2/3] Starting PyBullet physics bridge...")
        self.bridge = SITLBridge(gui=self.gui)
        self.bridge.start()
        print("      Bridge started")

        # 3. Connect MSP
        print(f"[3/3] Connecting MSP on TCP port {self.msp_port}...")
        time.sleep(1)  # Give SITL time to start TCP server

        self.tcp_serial = TCPSerial(port=self.msp_port, timeout=0.5)
        if not self.tcp_serial.open():
            print("      ERROR: Could not connect to SITL MSP port")
            print("      Make sure SITL is running and port is available")
            return False

        self.msp = MSPClient(self.tcp_serial, timeout=0.5)
        if not self.msp.connect():
            print("      ERROR: MSP connection failed")
            return False

        print(f"      MSP connected")

        self.running = True
        print("\nSimulation ready!")
        return True

    def _start_sitl(self) -> bool:
        """Start Betaflight SITL process"""
        if not self.sitl_path.exists():
            print(f"ERROR: SITL binary not found at {self.sitl_path}")
            print("Build it with: cd ~/projects/betaflight && make TARGET=SITL")
            return False

        try:
            # Start SITL with 127.0.0.1 as target IP for UDP
            self.sitl_process = subprocess.Popen(
                [str(self.sitl_path), "127.0.0.1"],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                cwd=self.sitl_path.parent
            )

            # Wait for startup
            time.sleep(2)

            if self.sitl_process.poll() is not None:
                print("ERROR: SITL process exited immediately")
                return False

            return True

        except Exception as e:
            print(f"ERROR: Failed to start SITL: {e}")
            return False

    def stop(self):
        """Stop all components"""
        self.running = False

        # Disarm first
        if self.armed:
            self.disarm()

        if self.msp:
            self.msp.close()

        if self.tcp_serial:
            self.tcp_serial.close()

        if self.bridge:
            self.bridge.stop()

        if self.sitl_process:
            self.sitl_process.terminate()
            try:
                self.sitl_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.sitl_process.kill()

        print("Simulation stopped")

    def arm(self) -> bool:
        """Arm the drone via UDP RC"""
        print("\nArming...")
        self.rc_channels[4] = 1800  # AUX1 high = armed
        self.rc_channels[2] = 1000  # Throttle low

        # Send arm command multiple times
        for _ in range(50):
            self._step()
            time.sleep(0.02)

        self.armed = True
        print("Armed!")
        return True

    def disarm(self) -> bool:
        """Disarm the drone via UDP RC"""
        print("\nDisarming...")
        self.rc_channels[4] = 1000  # AUX1 low = disarmed
        self.rc_channels[2] = 1000  # Throttle low

        for _ in range(50):
            self._step()
            time.sleep(0.02)

        self.armed = False
        print("Disarmed!")
        return True

    def set_rc(self, roll=1500, pitch=1500, throttle=1000, yaw=1500):
        """Set RC channel values"""
        self.rc_channels[0] = roll
        self.rc_channels[1] = pitch
        self.rc_channels[2] = throttle
        self.rc_channels[3] = yaw

    def _step(self):
        """Run one simulation step"""
        # Send RC to SITL via UDP (native SITL protocol)
        self.bridge.send_rc(self.rc_channels)

        # Step physics (sends sensors, receives motors)
        motors = self.bridge.step()

        return motors

    def run_hover_test(self, target_alt: float = 2.0, duration: float = 5.0):
        """Run a simple hover test"""
        print(f"\n--- Hover Test: {target_alt}m for {duration}s ---\n")

        if not self.armed:
            self.arm()
            time.sleep(0.5)

        # Hover throttle estimate
        hover_throttle = 1590  # ~59%

        print("Taking off...")
        start_time = time.time()

        while time.time() - start_time < duration + 5:  # +5s for takeoff
            state = self.bridge.get_state()

            # Simple altitude controller
            error = target_alt - state.z
            throttle = hover_throttle + int(error * 100)
            throttle = max(1100, min(1800, throttle))

            self.set_rc(throttle=throttle)

            motors = self._step()
            self.bridge.follow_camera()

            # Display
            elapsed = time.time() - start_time
            print(f"\r  t={elapsed:5.1f}s | Alt: {state.z:5.2f}m | "
                  f"Thr: {throttle} | Motors: [{motors[0]:.2f} {motors[1]:.2f} {motors[2]:.2f} {motors[3]:.2f}]  ",
                  end="", flush=True)

            time.sleep(1/50)  # 50Hz

        print("\n\nLanding...")
        # Simple landing - reduce throttle
        for i in range(200):
            state = self.bridge.get_state()
            throttle = max(1000, hover_throttle - i * 3)
            self.set_rc(throttle=throttle)
            self._step()
            self.bridge.follow_camera()

            if state.z < 0.1:
                break

            time.sleep(0.02)

        self.disarm()
        print("\nHover test complete!")

    def run_interactive(self):
        """Run interactive control mode"""
        print("\n--- Interactive Mode ---")
        print("Commands:")
        print("  a = arm")
        print("  d = disarm")
        print("  t = takeoff hover test")
        print("  q = quit")
        print()

        def input_thread():
            while self.running:
                try:
                    cmd = input("> ").strip().lower()
                    if cmd == 'a':
                        self.arm()
                    elif cmd == 'd':
                        self.disarm()
                    elif cmd == 't':
                        self.run_hover_test()
                    elif cmd == 'q':
                        self.running = False
                        break
                except EOFError:
                    break

        input_t = threading.Thread(target=input_thread, daemon=True)
        input_t.start()

        # Main loop
        while self.running:
            self._step()
            self.bridge.follow_camera()

            state = self.bridge.get_state()
            motors = self.bridge.motor_values

            # Status every second
            print(f"\r[{'ARMED' if self.armed else 'SAFE'}] "
                  f"Alt: {state.z:5.2f}m | "
                  f"Motors: [{motors[0]:.2f} {motors[1]:.2f} {motors[2]:.2f} {motors[3]:.2f}]  ",
                  end="", flush=True)

            time.sleep(1/60)


def main():
    sim = SITLSimulation(gui=True)

    # Handle Ctrl+C
    def signal_handler(sig, frame):
        print("\n\nInterrupted!")
        sim.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    if not sim.start():
        print("\nFailed to start simulation")
        sys.exit(1)

    try:
        # Run hover test
        sim.run_hover_test(target_alt=2.0, duration=5.0)

        # Keep running for a bit to see result
        time.sleep(2)

    finally:
        sim.stop()


if __name__ == "__main__":
    main()
