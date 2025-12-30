"""
Serial port manager for UART and USB connections

Handles automatic detection and switching between UART and USB ports.
Provides reconnection logic and port monitoring.
"""

import serial
import serial.tools.list_ports
import time
import threading
from typing import Optional, List, Callable
import logging

logger = logging.getLogger(__name__)


class SerialManager:
    """
    Manages serial port connections with automatic detection and reconnection

    Features:
    - Automatic port detection (UART and USB)
    - Reconnection on disconnect
    - Thread-safe access
    - Connection status callbacks
    """

    # Common Betaflight USB identifiers
    BETAFLIGHT_USB_VIDS = [0x0483]  # STMicroelectronics
    BETAFLIGHT_USB_PIDS = [0x5740, 0xDF11]  # Virtual COM Port, DFU mode

    def __init__(self,
                 uart_port: str = "/dev/ttyAMA0",
                 usb_port: str = "/dev/ttyACM0",
                 baudrate: int = 115200,
                 timeout: float = 0.1,
                 prefer_usb: bool = False):
        """
        Initialize serial manager

        Args:
            uart_port: Primary UART port path
            usb_port: USB serial port path
            baudrate: Connection baudrate
            timeout: Read timeout in seconds
            prefer_usb: If True, prefer USB over UART when both available
        """
        self.uart_port = uart_port
        self.usb_port = usb_port
        self.baudrate = baudrate
        self.timeout = timeout
        self.prefer_usb = prefer_usb

        self._serial: Optional[serial.Serial] = None
        self._current_port: Optional[str] = None
        self._lock = threading.Lock()
        self._connected = False

        # Reconnection
        self._auto_reconnect = True
        self._reconnect_thread: Optional[threading.Thread] = None
        self._running = False

        # Callbacks
        self._connect_callbacks: List[Callable[[str], None]] = []
        self._disconnect_callbacks: List[Callable[[], None]] = []

    @property
    def is_connected(self) -> bool:
        return self._connected and self._serial is not None

    @property
    def current_port(self) -> Optional[str]:
        return self._current_port

    @property
    def serial(self) -> Optional[serial.Serial]:
        """Get the underlying serial port (thread-safe)"""
        with self._lock:
            return self._serial

    def connect(self, port: Optional[str] = None) -> bool:
        """
        Connect to serial port

        Args:
            port: Specific port to connect to, or None for auto-detect

        Returns:
            True if connection successful
        """
        with self._lock:
            if self._serial and self._serial.is_open:
                self._serial.close()

            ports_to_try = []

            if port:
                ports_to_try = [port]
            elif self.prefer_usb:
                ports_to_try = [self.usb_port, self.uart_port]
            else:
                ports_to_try = [self.uart_port, self.usb_port]

            for p in ports_to_try:
                try:
                    self._serial = serial.Serial(
                        port=p,
                        baudrate=self.baudrate,
                        timeout=self.timeout,
                        write_timeout=self.timeout
                    )
                    self._current_port = p
                    self._connected = True
                    logger.info(f"Connected to {p} at {self.baudrate} baud")

                    # Notify callbacks
                    for callback in self._connect_callbacks:
                        try:
                            callback(p)
                        except Exception as e:
                            logger.error(f"Connect callback error: {e}")

                    return True

                except serial.SerialException as e:
                    logger.debug(f"Failed to connect to {p}: {e}")
                    continue

            logger.error("Failed to connect to any serial port")
            self._connected = False
            return False

    def disconnect(self):
        """Disconnect from serial port"""
        with self._lock:
            if self._serial:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None

            self._connected = False
            old_port = self._current_port
            self._current_port = None

            if old_port:
                for callback in self._disconnect_callbacks:
                    try:
                        callback()
                    except Exception as e:
                        logger.error(f"Disconnect callback error: {e}")

    def start_auto_reconnect(self, interval: float = 2.0):
        """Start automatic reconnection monitoring"""
        if self._running:
            return

        self._running = True
        self._reconnect_thread = threading.Thread(
            target=self._reconnect_loop,
            args=(interval,),
            daemon=True
        )
        self._reconnect_thread.start()

    def stop_auto_reconnect(self):
        """Stop automatic reconnection"""
        self._running = False
        if self._reconnect_thread:
            self._reconnect_thread.join(timeout=3.0)

    def _reconnect_loop(self, interval: float):
        """Background reconnection loop"""
        while self._running:
            if not self._connected:
                logger.info("Attempting reconnection...")
                self.connect()

            # Check if still connected
            elif self._serial:
                try:
                    # Try a simple operation to verify connection
                    if not self._serial.is_open:
                        raise serial.SerialException("Port closed")
                except serial.SerialException:
                    logger.warning("Connection lost, will attempt reconnect")
                    self.disconnect()

            time.sleep(interval)

    def add_connect_callback(self, callback: Callable[[str], None]):
        """Add callback for connection events"""
        self._connect_callbacks.append(callback)

    def add_disconnect_callback(self, callback: Callable[[], None]):
        """Add callback for disconnection events"""
        self._disconnect_callbacks.append(callback)

    # ==================== Port Detection ====================

    @classmethod
    def find_betaflight_ports(cls) -> List[str]:
        """
        Find potential Betaflight USB ports

        Returns:
            List of port device paths (sorted: ACM0, ACM1, ...)
        """
        ports = []

        for port in serial.tools.list_ports.comports():
            # Check for known Betaflight USB identifiers
            if port.vid in cls.BETAFLIGHT_USB_VIDS:
                ports.append(port.device)
                continue

            # Check for common naming patterns
            if 'ACM' in port.device or 'USB' in port.device:
                ports.append(port.device)

        # Sort for deterministic order
        return sorted(ports)

    def auto_detect_and_connect(self, timeout: float = 1.0) -> bool:
        """
        Auto-detect Betaflight port by testing MSP communication

        Scans all potential ports and tries to get a valid MSP response.

        Args:
            timeout: Timeout per port test in seconds

        Returns:
            True if a valid Betaflight port was found and connected
        """
        candidates = self.find_betaflight_ports()

        if not candidates:
            logger.warning("No potential Betaflight ports found")
            return False

        logger.info(f"Auto-detecting Betaflight on {len(candidates)} port(s): {candidates}")

        for port in candidates:
            logger.debug(f"Testing port {port}...")

            try:
                # Try to connect
                test_serial = serial.Serial(
                    port=port,
                    baudrate=self.baudrate,
                    timeout=timeout,
                    write_timeout=timeout
                )

                # Send MSP_API_VERSION request (code 1)
                # MSP v1 frame: $M< + length + cmd + crc
                msp_request = bytes([ord('$'), ord('M'), ord('<'), 0, 1, 1])
                test_serial.write(msp_request)
                test_serial.flush()

                # Wait for response
                time.sleep(0.1)

                # Read response - expect $M> header
                response = test_serial.read(32)
                test_serial.close()

                if len(response) >= 3 and response[0:3] == b'$M>':
                    logger.info(f"✓ Betaflight detected on {port}")

                    # Now do actual connection
                    with self._lock:
                        self._serial = serial.Serial(
                            port=port,
                            baudrate=self.baudrate,
                            timeout=self.timeout,
                            write_timeout=self.timeout
                        )
                        self._current_port = port
                        self._connected = True

                    # Notify callbacks
                    for callback in self._connect_callbacks:
                        try:
                            callback(port)
                        except Exception as e:
                            logger.error(f"Connect callback error: {e}")

                    return True
                else:
                    logger.debug(f"  {port}: No valid MSP response")

            except serial.SerialException as e:
                logger.debug(f"  {port}: {e}")
            except Exception as e:
                logger.debug(f"  {port}: Unexpected error: {e}")

        logger.error("Auto-detection failed: no Betaflight port found")
        return False

    @classmethod
    def list_available_ports(cls) -> List[dict]:
        """
        List all available serial ports with details

        Returns:
            List of port info dictionaries
        """
        result = []

        for port in serial.tools.list_ports.comports():
            result.append({
                'device': port.device,
                'name': port.name,
                'description': port.description,
                'vid': port.vid,
                'pid': port.pid,
                'serial_number': port.serial_number,
                'manufacturer': port.manufacturer
            })

        return result

    # ==================== Pass-through Methods ====================

    def read(self, size: int = 1) -> bytes:
        """Read from serial port"""
        with self._lock:
            if not self._serial:
                raise serial.SerialException("Not connected")
            return self._serial.read(size)

    def write(self, data: bytes) -> int:
        """Write to serial port"""
        with self._lock:
            if not self._serial:
                raise serial.SerialException("Not connected")
            return self._serial.write(data)

    def flush(self):
        """Flush write buffer"""
        with self._lock:
            if self._serial:
                self._serial.flush()

    def reset_input_buffer(self):
        """Clear input buffer"""
        with self._lock:
            if self._serial:
                self._serial.reset_input_buffer()

    def reset_output_buffer(self):
        """Clear output buffer"""
        with self._lock:
            if self._serial:
                self._serial.reset_output_buffer()

    @property
    def in_waiting(self) -> int:
        """Number of bytes in input buffer"""
        with self._lock:
            if self._serial:
                return self._serial.in_waiting
            return 0

    def close(self):
        """Close connection and stop monitoring"""
        self.stop_auto_reconnect()
        self.disconnect()
