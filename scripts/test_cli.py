#!/usr/bin/env python3
"""Test CLI functions with connected drone"""

import sys
sys.path.insert(0, '/home/remenby/projects/pi_drone_nav')

from src.flight.flight_controller import FlightController
from src.interfaces.cli import CLI

def main():
    print("=" * 60)
    print("   TEST CLI AVEC DRONE CONNECTÉ")
    print("=" * 60)
    print()

    # Initialize flight controller
    print("Initialisation du FlightController...")
    fc = FlightController(simulation=False)

    if not fc.initialize():
        print("ERREUR: Impossible d'initialiser le FlightController")
        return 1

    fc.start()
    print("FlightController démarré!")
    print()

    # Create CLI instance
    cli = CLI(fc)

    try:
        # Run tests
        print("=" * 60)
        print("TEST 1: test msp")
        print("=" * 60)
        cli._test_msp()
        print()

        print("=" * 60)
        print("TEST 2: test sensors")
        print("=" * 60)
        cli._test_sensors()
        print()

        print("=" * 60)
        print("TEST 3: test gps")
        print("=" * 60)
        cli._test_gps()
        print()

        print("=" * 60)
        print("TEST 4: test rc")
        print("=" * 60)
        cli._test_rc()
        print()

        print("=" * 60)
        print("TEST 5: test preflight")
        print("=" * 60)
        cli._test_preflight()
        print()

        print("=" * 60)
        print("TEST 6: status")
        print("=" * 60)
        cli.do_status("")
        print()

        print("=" * 60)
        print("TEST 7: diag")
        print("=" * 60)
        cli.do_diag("")
        print()

    finally:
        # Cleanup
        print("=" * 60)
        print("Arrêt...")
        fc.shutdown()

    print("TESTS TERMINÉS!")
    return 0


if __name__ == "__main__":
    sys.exit(main())
