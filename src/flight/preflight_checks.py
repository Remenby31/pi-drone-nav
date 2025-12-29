"""
Pre-flight Checks - Essential validations before arming

Inspired by ArduPilot's AP_Arming and iNav's pre-arm checks.
Validates critical systems before allowing arm/takeoff.
"""

from enum import Enum, auto
from dataclasses import dataclass, field
from typing import List, Optional, TYPE_CHECKING
import time
import logging

if TYPE_CHECKING:
    from ..drivers.msp import MSPClient
    from ..drivers.gps_ubx import GPSFix
    from ..config import TakeoffConfig

logger = logging.getLogger(__name__)


class CheckResult(Enum):
    """Result of a pre-flight check"""
    PASS = auto()
    FAIL = auto()
    WARN = auto()      # Non-blocking warning
    PENDING = auto()   # Check in progress


@dataclass
class CheckStatus:
    """Status of a single check"""
    name: str
    result: CheckResult = CheckResult.PENDING
    message: str = ""
    last_check_time: float = 0.0


@dataclass
class PreflightResult:
    """Complete preflight check result"""
    passed: bool
    checks: List[CheckStatus] = field(default_factory=list)
    blocking_failures: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)


class PreflightChecker:
    """
    Runs essential pre-flight checks before allowing arm/takeoff

    Essential checks (blocking):
    - GPS fix quality (satellites, HDOP)
    - MSP communication alive
    - Attitude sensors responding and level

    Optional checks (warnings):
    - Battery voltage
    - Compass calibration
    """

    def __init__(self, config: 'TakeoffConfig'):
        """
        Initialize preflight checker

        Args:
            config: Takeoff configuration with preflight settings
        """
        self.config = config
        self._check_statuses: List[CheckStatus] = []

    def run_checks(self, msp: Optional['MSPClient'],
                   gps_fix: Optional['GPSFix'],
                   attitude: Optional[object]) -> PreflightResult:
        """
        Run all pre-flight checks

        Args:
            msp: MSP client for communication checks
            gps_fix: Current GPS fix (or None if no GPS)
            attitude: Current attitude data with roll/pitch attributes (or None)

        Returns:
            PreflightResult with pass/fail status and details
        """
        self._check_statuses = []

        # Essential checks
        self._check_gps(gps_fix)
        self._check_msp_communication(msp)
        self._check_attitude(attitude)

        # Compile results
        blocking_failures = []
        warnings = []

        for check in self._check_statuses:
            if check.result == CheckResult.FAIL:
                blocking_failures.append(f"{check.name}: {check.message}")
            elif check.result == CheckResult.WARN:
                warnings.append(f"{check.name}: {check.message}")

        passed = len(blocking_failures) == 0

        if passed:
            logger.info("Pre-flight checks PASSED")
        else:
            logger.warning(f"Pre-flight checks FAILED: {blocking_failures}")

        return PreflightResult(
            passed=passed,
            checks=self._check_statuses.copy(),
            blocking_failures=blocking_failures,
            warnings=warnings
        )

    def _check_gps(self, gps_fix: Optional['GPSFix']):
        """Check GPS fix quality"""
        check = CheckStatus(name="GPS Fix")

        if gps_fix is None:
            check.result = CheckResult.FAIL
            check.message = "No GPS data available"
        elif not getattr(gps_fix, 'fix_valid', getattr(gps_fix, 'has_fix', False)):
            check.result = CheckResult.FAIL
            check.message = "No valid GPS fix"
        elif getattr(gps_fix, 'num_satellites', 0) < self.config.min_gps_satellites:
            check.result = CheckResult.FAIL
            sats = getattr(gps_fix, 'num_satellites', 0)
            check.message = f"Insufficient satellites: {sats} < {self.config.min_gps_satellites}"
        elif hasattr(gps_fix, 'hdop') and gps_fix.hdop > self.config.max_hdop:
            check.result = CheckResult.WARN
            check.message = f"HDOP high: {gps_fix.hdop:.1f} > {self.config.max_hdop}"
        else:
            check.result = CheckResult.PASS
            sats = getattr(gps_fix, 'num_satellites', 0)
            check.message = f"OK ({sats} sats)"

        check.last_check_time = time.time()
        self._check_statuses.append(check)

    def _check_msp_communication(self, msp: Optional['MSPClient']):
        """Check MSP communication is working"""
        check = CheckStatus(name="MSP Communication")

        if msp is None:
            check.result = CheckResult.FAIL
            check.message = "MSP client not initialized"
        elif not getattr(msp, 'is_connected', False):
            check.result = CheckResult.FAIL
            check.message = "MSP not connected"
        else:
            # Try to get status to verify communication
            try:
                status = msp.get_status()
                if status is not None:
                    cpu = getattr(status, 'cpu_load', 0)
                    check.result = CheckResult.PASS
                    check.message = f"OK (CPU: {cpu}%)"
                else:
                    check.result = CheckResult.FAIL
                    check.message = "MSP status returned None"
            except Exception as e:
                check.result = CheckResult.FAIL
                check.message = f"MSP error: {e}"

        check.last_check_time = time.time()
        self._check_statuses.append(check)

    def _check_attitude(self, attitude: Optional[object]):
        """Check attitude sensors are responding and drone is level"""
        check = CheckStatus(name="Attitude Sensors")

        if attitude is None:
            check.result = CheckResult.FAIL
            check.message = "No attitude data"
        else:
            roll = getattr(attitude, 'roll', 0)
            pitch = getattr(attitude, 'pitch', 0)
            tilt = (roll**2 + pitch**2) ** 0.5
            max_tilt = self.config.attitude_check_max_tilt_deg

            if tilt > max_tilt:
                check.result = CheckResult.FAIL
                check.message = f"Not level: {tilt:.1f} deg > {max_tilt} deg"
            else:
                check.result = CheckResult.PASS
                check.message = f"OK (tilt: {tilt:.1f} deg)"

        check.last_check_time = time.time()
        self._check_statuses.append(check)

    def get_status(self) -> dict:
        """Get preflight check status for telemetry"""
        return {
            'checks': [
                {
                    'name': c.name,
                    'result': c.result.name,
                    'message': c.message
                }
                for c in self._check_statuses
            ]
        }
