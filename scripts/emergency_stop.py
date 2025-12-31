#!/usr/bin/env python3
"""
EMERGENCY STOP - Disarm drone immediately

Usage:
    python emergency_stop.py [raspberry_ip]

Or with keyboard shortcut (recommended):
    Bind this script to a key combination for instant access
"""

import sys
import requests
import time
from concurrent.futures import ThreadPoolExecutor

# Configuration
DEFAULT_IP = "192.168.1.114"
API_PORT = 8080

# ANSI colors
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"


def emergency_disarm(ip: str) -> bool:
    """Send emergency disarm command"""
    url = f"http://{ip}:{API_PORT}/api/emergency/disarm"

    try:
        response = requests.post(url, timeout=2)
        return response.status_code == 200
    except Exception:
        return False


def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_IP
    url = f"http://{ip}:{API_PORT}/api/emergency/disarm"

    print(f"{RED}")
    print("=" * 50)
    print("        !!! EMERGENCY DISARM !!!")
    print("=" * 50)
    print(f"{RESET}")
    print(f"Target: {url}")
    print()

    # Send multiple requests in parallel for reliability
    success = False

    with ThreadPoolExecutor(max_workers=5) as executor:
        # Fire 5 requests simultaneously
        futures = [executor.submit(emergency_disarm, ip) for _ in range(5)]

        for future in futures:
            if future.result():
                success = True
                break

    if success:
        print(f"{GREEN}")
        print("=" * 50)
        print("         DISARMED SUCCESSFULLY")
        print("=" * 50)
        print(f"{RESET}")
        return 0
    else:
        print(f"{RED}")
        print("=" * 50)
        print("         DISARM FAILED!")
        print("=" * 50)
        print(f"{RESET}")
        print()
        print(f"{YELLOW}Retrying...{RESET}")

        # Retry loop
        for i in range(5):
            time.sleep(0.1)
            if emergency_disarm(ip):
                print(f"{GREEN}Retry {i+1}: DISARMED{RESET}")
                return 0
            else:
                print(f"{RED}Retry {i+1}: Failed{RESET}")

        print()
        print(f"{RED}!!! ALL RETRIES FAILED !!!{RESET}")
        print(f"Check connection to Raspberry Pi at {ip}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
