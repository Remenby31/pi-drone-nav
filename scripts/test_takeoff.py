#!/usr/bin/env python3
"""
Test de la procédure de décollage améliorée
ATTENTION: Tester SANS HELICES uniquement!
"""

import sys
import time
sys.path.insert(0, '/home/remenby/projects/pi_drone_nav')

from src.drivers.msp import MSPClient
from src.drivers.serial_manager import SerialManager
from src.config import get_config, TakeoffConfig
from src.flight.preflight_checks import PreflightChecker, CheckResult
from src.flight.takeoff_controller import TakeoffController, TakeoffPhase
from src.navigation.altitude_controller import AltitudeController

# Configuration
PORT = "/dev/ttyACM0"
BAUDRATE = 115200

def test_connection():
    """Test MSP connection"""
    print("\n" + "="*50)
    print("1. TEST CONNEXION MSP")
    print("="*50)

    serial_mgr = SerialManager(
        uart_port=PORT,
        usb_port=PORT,
        baudrate=BAUDRATE,
        prefer_usb=True
    )

    if not serial_mgr.connect():
        print("❌ Échec connexion série")
        return None, None

    msp = MSPClient(serial_mgr, timeout=0.5)
    if not msp.connect():
        print("❌ Échec connexion MSP")
        return None, None

    # Get info
    api = msp.get_api_version()
    print(f"✓ API Version: {api}")

    status = msp.get_status()
    if status:
        print(f"✓ Status: cycle={status.cycle_time}µs, CPU={status.cpu_load}%")
    else:
        print("✓ Connexion OK (status non disponible)")

    return msp, serial_mgr


def test_preflight(msp):
    """Test pre-flight checks"""
    print("\n" + "="*50)
    print("2. TEST PRE-FLIGHT CHECKS")
    print("="*50)

    config = TakeoffConfig()
    checker = PreflightChecker(config)

    # Get current data
    gps_fix = msp.get_gps_as_fix()
    attitude = msp.get_attitude()

    print(f"\nDonnées actuelles:")
    if gps_fix:
        print(f"  GPS: {gps_fix.num_satellites} sats, fix={gps_fix.has_fix}")
        print(f"  Position: {gps_fix.latitude:.6f}, {gps_fix.longitude:.6f}")
    else:
        print("  GPS: Non disponible")

    if attitude:
        print(f"  Attitude: roll={attitude.roll:.1f}°, pitch={attitude.pitch:.1f}°, yaw={attitude.yaw:.1f}°")

    # Run checks
    result = checker.run_checks(msp, gps_fix, attitude)

    print(f"\nRésultat des checks:")
    for check in result.checks:
        symbol = "✓" if check.result == CheckResult.PASS else "⚠" if check.result == CheckResult.WARN else "❌"
        print(f"  {symbol} {check.name}: {check.message}")

    print(f"\n{'✓ PASSED' if result.passed else '❌ FAILED'}")
    return result.passed, gps_fix, attitude


def test_motor_spinup(msp):
    """Test motor spinup phase"""
    print("\n" + "="*50)
    print("3. TEST MOTOR SPINUP")
    print("="*50)

    input("\n⚠️  ATTENTION: Les moteurs vont tourner!\nAppuyez sur Entrée pour continuer (Ctrl+C pour annuler)...")

    config = TakeoffConfig()
    spinup_time = config.spinup_time_ms / 1000.0
    initial_throttle = config.initial_throttle

    print(f"\nConfiguration:")
    print(f"  Durée spinup: {config.spinup_time_ms}ms")
    print(f"  Throttle initial: {initial_throttle}")

    # RC channels array
    # [Roll, Pitch, Throttle, Yaw, AUX1, AUX2, AUX3, AUX4]
    RC_CENTER = 1500
    RC_LOW = 1000
    RC_HIGH = 1800

    def set_rc(throttle_pct):
        """Set RC with throttle percentage (0.0-1.0)"""
        throttle_rc = int(throttle_pct * 1000 + 1000)
        channels = [RC_CENTER, RC_CENTER, throttle_rc, RC_CENTER, RC_HIGH, RC_HIGH, RC_CENTER, RC_CENTER]
        msp.set_raw_rc(channels)
        return channels

    try:
        print("\nPhase SPINUP:")
        start_time = time.time()

        while True:
            elapsed = time.time() - start_time
            progress = min(1.0, elapsed / spinup_time)
            throttle = initial_throttle * progress

            channels = set_rc(throttle)
            motors = msp.get_motor_values()

            print(f"\r  Progress: {progress*100:5.1f}% | Throttle: {throttle:.3f} | RC: {channels[2]} | Motors: {motors}", end="")

            if progress >= 1.0:
                break

            time.sleep(0.02)  # 50Hz

        print("\n\n✓ Spinup terminé")

        # Hold for 1 second
        print("\nMaintien 1 seconde...")
        hold_start = time.time()
        while time.time() - hold_start < 1.0:
            set_rc(initial_throttle)
            time.sleep(0.02)

        return True

    except KeyboardInterrupt:
        print("\n\n⚠️  Interrompu par l'utilisateur")
        return False
    finally:
        # Stop motors
        print("\nArrêt des moteurs...")
        for _ in range(10):
            msp.set_raw_rc([RC_CENTER, RC_CENTER, RC_LOW, RC_CENTER, RC_LOW, RC_CENTER, RC_CENTER, RC_CENTER])
            time.sleep(0.02)
        print("✓ Moteurs arrêtés")


def test_throttle_ramp(msp):
    """Test throttle ramp phase"""
    print("\n" + "="*50)
    print("4. TEST THROTTLE RAMP")
    print("="*50)

    input("\n⚠️  Les moteurs vont accélérer progressivement!\nAppuyez sur Entrée pour continuer (Ctrl+C pour annuler)...")

    config = TakeoffConfig()

    print(f"\nConfiguration:")
    print(f"  Initial throttle: {config.initial_throttle}")
    print(f"  Ramp rate: {config.ramp_rate_per_sec}/s")
    print(f"  Max ramp throttle: {config.max_ramp_throttle}")

    RC_CENTER = 1500
    RC_LOW = 1000
    RC_HIGH = 1800

    def set_rc(throttle_pct):
        throttle_rc = int(throttle_pct * 1000 + 1000)
        channels = [RC_CENTER, RC_CENTER, throttle_rc, RC_CENTER, RC_HIGH, RC_HIGH, RC_CENTER, RC_CENTER]
        msp.set_raw_rc(channels)
        return channels

    try:
        throttle = config.initial_throttle
        throttle_rate = 0.0
        max_jerk = 0.5
        dt = 0.02

        print("\nPhase THROTTLE_RAMP (5 secondes max):")
        start_time = time.time()

        while time.time() - start_time < 5.0:
            # Jerk-limited ramp
            target_rate = config.ramp_rate_per_sec
            rate_error = target_rate - throttle_rate
            max_rate_change = max_jerk * dt
            rate_change = max(-max_rate_change, min(max_rate_change, rate_error))
            throttle_rate += rate_change

            throttle += throttle_rate * dt
            throttle = min(throttle, config.max_ramp_throttle)

            channels = set_rc(throttle)
            motors = msp.get_motor_values()

            elapsed = time.time() - start_time
            print(f"\r  Time: {elapsed:4.1f}s | Throttle: {throttle:.3f} | Rate: {throttle_rate:.3f}/s | RC: {channels[2]} | Motors: {motors}", end="")

            if throttle >= config.max_ramp_throttle:
                print("\n\n⚠️  Max throttle atteint!")
                break

            time.sleep(dt)

        print("\n\n✓ Ramp terminé")
        return True

    except KeyboardInterrupt:
        print("\n\n⚠️  Interrompu par l'utilisateur")
        return False
    finally:
        # Stop motors
        print("\nArrêt des moteurs...")
        for _ in range(10):
            msp.set_raw_rc([RC_CENTER, RC_CENTER, RC_LOW, RC_CENTER, RC_LOW, RC_CENTER, RC_CENTER, RC_CENTER])
            time.sleep(0.02)
        print("✓ Moteurs arrêtés")


def test_full_sequence(msp):
    """Test full takeoff sequence with TakeoffController"""
    print("\n" + "="*50)
    print("5. TEST SÉQUENCE COMPLÈTE (sans vol)")
    print("="*50)

    print("\nCe test simule la séquence complète mais sans décoller")
    print("(le drone reste au sol, pas de détection liftoff)")

    input("\nAppuyez sur Entrée pour continuer (Ctrl+C pour annuler)...")

    config = get_config()
    alt_controller = AltitudeController()
    takeoff = TakeoffController(config.takeoff, alt_controller)

    RC_CENTER = 1500
    RC_LOW = 1000
    RC_HIGH = 1800

    def set_rc(throttle_pct):
        throttle_rc = int(throttle_pct * 1000 + 1000)
        channels = [RC_CENTER, RC_CENTER, throttle_rc, RC_CENTER, RC_HIGH, RC_HIGH, RC_CENTER, RC_CENTER]
        msp.set_raw_rc(channels)
        return channels

    # Callbacks
    completed = [False]
    aborted = [False]
    abort_reason = [""]

    def on_complete():
        completed[0] = True
        print("\n✓ TAKEOFF COMPLETE callback!")

    def on_abort(reason):
        aborted[0] = True
        abort_reason[0] = reason
        print(f"\n❌ TAKEOFF ABORTED: {reason}")

    takeoff.on_complete(on_complete)
    takeoff.on_abort(on_abort)

    try:
        # Start takeoff
        takeoff.start(target_altitude_m=3.0)
        print(f"\nTarget altitude: 3.0m")
        print(f"Starting phase: {takeoff.phase.name}")

        dt = 0.02
        start_time = time.time()
        max_time = 10.0  # Max 10 seconds for test

        while time.time() - start_time < max_time:
            # Get sensor data
            attitude = msp.get_attitude()
            altitude_data = msp.get_altitude()

            roll = attitude.roll if attitude else 0
            pitch = attitude.pitch if attitude else 0
            altitude_m = altitude_data.altitude_cm / 100.0 if altitude_data else 0
            climb_rate = altitude_data.vario_cms / 100.0 if altitude_data else 0

            # Update takeoff controller
            throttle = takeoff.update(
                dt=dt,
                altitude_m=altitude_m,
                climb_rate_ms=climb_rate,
                roll_deg=roll,
                pitch_deg=pitch
            )

            # Send RC
            if takeoff.is_active:
                channels = set_rc(throttle)
            else:
                set_rc(0)

            # Print status
            elapsed = time.time() - start_time
            status = takeoff.get_status()
            print(f"\r  [{elapsed:5.1f}s] Phase: {status['phase']:15} | Throttle: {throttle:.3f} | Alt: {altitude_m:5.2f}m | Climb: {climb_rate:+5.2f}m/s", end="")

            # Check if done
            if completed[0] or aborted[0]:
                break

            if not takeoff.is_active:
                print(f"\n\nTakeoff terminé - Phase finale: {takeoff.phase.name}")
                break

            time.sleep(dt)

        print("\n")
        return True

    except KeyboardInterrupt:
        print("\n\n⚠️  Interrompu par l'utilisateur")
        return False
    finally:
        # Stop motors
        print("Arrêt des moteurs...")
        for _ in range(10):
            msp.set_raw_rc([RC_CENTER, RC_CENTER, RC_LOW, RC_CENTER, RC_LOW, RC_CENTER, RC_CENTER, RC_CENTER])
            time.sleep(0.02)
        print("✓ Moteurs arrêtés")


def main():
    print("="*50)
    print("  TEST PROCÉDURE DE DÉCOLLAGE AMÉLIORÉE")
    print("  ⚠️  SANS HÉLICES UNIQUEMENT!")
    print("="*50)

    # Test connection
    msp, serial_mgr = test_connection()
    if not msp:
        return 1

    try:
        # Test preflight
        passed, gps_fix, attitude = test_preflight(msp)

        # Menu
        while True:
            print("\n" + "-"*50)
            print("Options de test:")
            print("  1. Re-test pre-flight checks")
            print("  2. Test motor spinup (500ms)")
            print("  3. Test throttle ramp (5s max)")
            print("  4. Test séquence complète")
            print("  5. Lire valeurs moteurs")
            print("  q. Quitter")
            print("-"*50)

            choice = input("Choix: ").strip().lower()

            if choice == '1':
                test_preflight(msp)
            elif choice == '2':
                test_motor_spinup(msp)
            elif choice == '3':
                test_throttle_ramp(msp)
            elif choice == '4':
                test_full_sequence(msp)
            elif choice == '5':
                motors = msp.get_motor_values()
                print(f"Valeurs moteurs: {motors}")
            elif choice == 'q':
                break
            else:
                print("Choix invalide")

    finally:
        # Cleanup
        print("\nFermeture...")
        if msp:
            # Ensure motors stopped
            for _ in range(5):
                try:
                    msp.set_raw_rc([1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500])
                except:
                    pass
                time.sleep(0.02)
            msp.close()
        if serial_mgr:
            serial_mgr.close()

    print("✓ Test terminé")
    return 0


if __name__ == "__main__":
    sys.exit(main())
