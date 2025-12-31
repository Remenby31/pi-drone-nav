#!/usr/bin/env python3
"""
TCP Serial Wrapper

Provides a pyserial-compatible interface over TCP socket.
Used to connect MSP driver to Betaflight SITL.
"""

import socket
import time
from typing import Optional


class TCPSerial:
    """
    TCP socket wrapper that mimics pyserial Serial interface.

    This allows the MSP driver to connect to Betaflight SITL
    which exposes UART over TCP (port 576x).
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 5761,
        timeout: float = 1.0
    ):
        """
        Initialize TCP serial connection.

        Args:
            host: SITL host address
            port: SITL UART port (5761 for UART1, etc.)
            timeout: Socket timeout in seconds
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock: Optional[socket.socket] = None
        self._buffer = b''

    def open(self) -> bool:
        """Open TCP connection"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.timeout)
            self.sock.connect((self.host, self.port))
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            return True
        except Exception as e:
            print(f"[TCPSerial] Connection failed: {e}")
            self.sock = None
            return False

    @property
    def is_open(self) -> bool:
        """Check if connection is open"""
        return self.sock is not None

    @property
    def in_waiting(self) -> int:
        """Return number of bytes waiting to be read"""
        # For TCP, we don't have a reliable way to know without blocking
        # Return length of internal buffer
        return len(self._buffer)

    def read(self, size: int = 1) -> bytes:
        """
        Read bytes from TCP socket.

        Args:
            size: Number of bytes to read

        Returns:
            Bytes read (may be less than requested)
        """
        if not self.sock:
            return b''

        # First, return from buffer if available
        if len(self._buffer) >= size:
            data = self._buffer[:size]
            self._buffer = self._buffer[size:]
            return data

        # Need to read more from socket
        try:
            # Try to read remaining bytes
            remaining = size - len(self._buffer)
            self.sock.settimeout(self.timeout)
            chunk = self.sock.recv(remaining)

            if chunk:
                result = self._buffer + chunk
                self._buffer = b''

                if len(result) >= size:
                    self._buffer = result[size:]
                    return result[:size]
                return result
            return self._buffer

        except socket.timeout:
            # Return what we have
            data = self._buffer
            self._buffer = b''
            return data
        except Exception:
            return b''

    def write(self, data: bytes) -> int:
        """
        Write bytes to TCP socket.

        Args:
            data: Bytes to write

        Returns:
            Number of bytes written
        """
        if not self.sock:
            return 0

        try:
            return self.sock.send(data)
        except Exception:
            return 0

    def flush(self):
        """Flush output buffer (no-op for TCP)"""
        pass

    def reset_input_buffer(self):
        """Clear input buffer"""
        self._buffer = b''
        if self.sock:
            # Drain any pending data
            try:
                self.sock.setblocking(False)
                while True:
                    try:
                        data = self.sock.recv(1024)
                        if not data:
                            break
                    except BlockingIOError:
                        break
            except Exception:
                pass
            finally:
                self.sock.setblocking(True)
                self.sock.settimeout(self.timeout)

    def close(self):
        """Close TCP connection"""
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
        self._buffer = b''

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *args):
        self.close()


def test_connection(host: str = "127.0.0.1", port: int = 5761):
    """Test TCP connection to SITL"""
    print(f"Testing connection to {host}:{port}...")

    tcp = TCPSerial(host, port, timeout=2.0)
    if not tcp.open():
        print("Failed to connect!")
        return False

    print("Connected!")

    # Try to send MSP_API_VERSION request
    # MSP V1: $M< + size(0) + cmd(1) + checksum
    msp_request = b'$M<\x00\x01\x01'  # API_VERSION request
    tcp.write(msp_request)
    print(f"Sent: {msp_request.hex()}")

    time.sleep(0.1)

    response = tcp.read(20)
    print(f"Received: {response.hex() if response else '(nothing)'}")

    tcp.close()
    return bool(response)


if __name__ == "__main__":
    import sys
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 5761
    test_connection(port=port)
