"""
HTTP Client for Pi Drone Navigation CLI

Communicates with pidrone-server via REST API.
"""

import json
from pathlib import Path
from typing import Optional, Dict, Any, List

try:
    import requests
    REQUESTS_AVAILABLE = True
except ImportError:
    REQUESTS_AVAILABLE = False


class ServerError(Exception):
    """Error from server response"""
    pass


class ConnectionError(Exception):
    """Server connection error"""
    pass


class PidroneClient:
    """HTTP client for pidrone-server"""

    def __init__(self, base_url: str = "http://localhost:8080"):
        if not REQUESTS_AVAILABLE:
            raise ImportError("requests library is required: pip install requests")

        self.base_url = base_url.rstrip('/')
        self.timeout = 10.0

    def is_server_running(self) -> bool:
        """Check if server is accessible"""
        try:
            r = requests.get(f"{self.base_url}/api/health", timeout=2)
            return r.status_code == 200
        except:
            return False

    def _get(self, endpoint: str) -> Dict[str, Any]:
        """Make GET request"""
        try:
            r = requests.get(f"{self.base_url}{endpoint}", timeout=self.timeout)
            data = r.json()
            if r.status_code >= 400:
                raise ServerError(data.get('error', f'HTTP {r.status_code}'))
            return data
        except requests.exceptions.ConnectionError:
            raise ConnectionError("Cannot connect to server")
        except requests.exceptions.Timeout:
            raise ConnectionError("Request timeout")

    def _post(self, endpoint: str, json_data: Dict = None) -> Dict[str, Any]:
        """Make POST request"""
        try:
            r = requests.post(
                f"{self.base_url}{endpoint}",
                json=json_data,
                timeout=self.timeout
            )
            data = r.json()
            if r.status_code >= 400:
                raise ServerError(data.get('error', f'HTTP {r.status_code}'))
            return data
        except requests.exceptions.ConnectionError:
            raise ConnectionError("Cannot connect to server")
        except requests.exceptions.Timeout:
            raise ConnectionError("Request timeout")

    def _delete(self, endpoint: str) -> Dict[str, Any]:
        """Make DELETE request"""
        try:
            r = requests.delete(f"{self.base_url}{endpoint}", timeout=self.timeout)
            data = r.json()
            if r.status_code >= 400:
                raise ServerError(data.get('error', f'HTTP {r.status_code}'))
            return data
        except requests.exceptions.ConnectionError:
            raise ConnectionError("Cannot connect to server")
        except requests.exceptions.Timeout:
            raise ConnectionError("Request timeout")

    # ==================== Status ====================

    def get_status(self) -> Dict[str, Any]:
        """Get complete drone telemetry"""
        return self._get("/api/status")

    def get_health(self) -> Dict[str, Any]:
        """Get health check"""
        return self._get("/api/health")

    # ==================== Missions ====================

    def list_missions(self) -> List[Dict[str, Any]]:
        """List all stored missions"""
        data = self._get("/api/missions")
        return data.get('missions', [])

    def get_mission(self, name_or_uuid: str) -> Dict[str, Any]:
        """Get mission details by name or UUID"""
        return self._get(f"/api/missions/{name_or_uuid}")

    def upload_mission(self, filepath: str) -> Dict[str, Any]:
        """Upload a mission from JSON file"""
        path = Path(filepath)
        if not path.exists():
            raise FileNotFoundError(f"File not found: {filepath}")

        with open(path) as f:
            data = json.load(f)

        return self._post("/api/missions", json_data=data)

    def delete_mission(self, name_or_uuid: str) -> Dict[str, Any]:
        """Delete a mission"""
        return self._delete(f"/api/missions/{name_or_uuid}")

    # ==================== Mission Execution ====================

    def start_mission(self, name_or_uuid: str) -> Dict[str, Any]:
        """Start a mission (arms automatically)"""
        return self._post(f"/api/missions/{name_or_uuid}/start")

    def get_active_mission(self) -> Dict[str, Any]:
        """Get active mission status"""
        return self._get("/api/missions/active")

    def pause_mission(self) -> Dict[str, Any]:
        """Pause active mission"""
        return self._post("/api/missions/active/pause")

    def resume_mission(self) -> Dict[str, Any]:
        """Resume paused mission"""
        return self._post("/api/missions/active/resume")

    def stop_mission(self) -> Dict[str, Any]:
        """Stop active mission (lands and disarms)"""
        return self._post("/api/missions/active/stop")

    # ==================== Diagnostics ====================

    def get_diagnostics(self) -> Dict[str, Any]:
        """Get full system diagnostics"""
        return self._get("/api/diagnostics")

    def run_test(self, component: str, confirm: bool = False) -> Dict[str, Any]:
        """
        Run diagnostic test for a component

        Args:
            component: msp, gps, sensors, motors, rc, preflight
            confirm: For motors test, must be True
        """
        json_data = None
        if component == 'motors':
            json_data = {'confirm': confirm}

        return self._post(f"/api/test/{component}", json_data=json_data)

    # ==================== Helpers ====================

    def resolve_mission_uuid(self, name_or_uuid: str) -> Optional[str]:
        """Resolve mission name to UUID"""
        missions = self.list_missions()

        # Check if it's a UUID
        for m in missions:
            if m['uuid'] == name_or_uuid:
                return m['uuid']

        # Check by name
        for m in missions:
            if m['name'].lower() == name_or_uuid.lower():
                return m['uuid']

        return None
