"""
Geographic utilities

Coordinate conversions and distance calculations.
"""

import math
from typing import Tuple

# WGS84 ellipsoid parameters
EARTH_RADIUS_M = 6378137.0              # Equatorial radius
EARTH_FLATTENING = 1.0 / 298.257223563  # WGS84 flattening


def gps_to_ned(lat: float, lon: float, alt: float,
               ref_lat: float, ref_lon: float, ref_alt: float) -> Tuple[float, float, float]:
    """
    Convert GPS coordinates to local NED (North-East-Down) frame

    Uses flat Earth approximation, accurate for distances < 10km

    Args:
        lat, lon: Current position in degrees
        alt: Current altitude in meters (MSL)
        ref_lat, ref_lon, ref_alt: Reference/origin point

    Returns:
        Tuple of (north, east, down) in meters
    """
    # Calculate deltas in radians
    d_lat = math.radians(lat - ref_lat)
    d_lon = math.radians(lon - ref_lon)
    ref_lat_rad = math.radians(ref_lat)

    # Meridional radius of curvature
    # For small distances, we can use the approximation
    north = d_lat * EARTH_RADIUS_M

    # Prime vertical radius of curvature (simplified)
    east = d_lon * EARTH_RADIUS_M * math.cos(ref_lat_rad)

    # Down is negative altitude difference
    down = -(alt - ref_alt)

    return north, east, down


def ned_to_gps(north: float, east: float, down: float,
               ref_lat: float, ref_lon: float, ref_alt: float) -> Tuple[float, float, float]:
    """
    Convert local NED coordinates to GPS

    Args:
        north, east, down: Local position in meters
        ref_lat, ref_lon, ref_alt: Reference/origin point

    Returns:
        Tuple of (lat, lon, alt) in degrees and meters
    """
    ref_lat_rad = math.radians(ref_lat)

    # Inverse of gps_to_ned
    d_lat_rad = north / EARTH_RADIUS_M
    d_lon_rad = east / (EARTH_RADIUS_M * math.cos(ref_lat_rad))

    lat = ref_lat + math.degrees(d_lat_rad)
    lon = ref_lon + math.degrees(d_lon_rad)
    alt = ref_alt - down

    return lat, lon, alt


def haversine_distance(lat1: float, lon1: float,
                       lat2: float, lon2: float) -> float:
    """
    Calculate great-circle distance between two points

    Uses haversine formula for accuracy over long distances.

    Args:
        lat1, lon1: First point in degrees
        lat2, lon2: Second point in degrees

    Returns:
        Distance in meters
    """
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)

    a = (math.sin(d_lat / 2) ** 2 +
         math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(d_lon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return EARTH_RADIUS_M * c


def bearing(lat1: float, lon1: float,
            lat2: float, lon2: float) -> float:
    """
    Calculate initial bearing from point 1 to point 2

    Args:
        lat1, lon1: Start point in degrees
        lat2, lon2: End point in degrees

    Returns:
        Bearing in degrees (0-360, 0=North)
    """
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    d_lon = math.radians(lon2 - lon1)

    x = math.sin(d_lon) * math.cos(lat2_rad)
    y = (math.cos(lat1_rad) * math.sin(lat2_rad) -
         math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(d_lon))

    bearing_rad = math.atan2(x, y)
    bearing_deg = math.degrees(bearing_rad)

    # Normalize to 0-360
    return (bearing_deg + 360) % 360


def destination_point(lat: float, lon: float,
                      bearing_deg: float, distance_m: float) -> Tuple[float, float]:
    """
    Calculate destination point given start, bearing and distance

    Args:
        lat, lon: Start point in degrees
        bearing_deg: Bearing in degrees
        distance_m: Distance in meters

    Returns:
        Tuple of (lat, lon) in degrees
    """
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    bearing_rad = math.radians(bearing_deg)

    angular_dist = distance_m / EARTH_RADIUS_M

    lat2_rad = math.asin(
        math.sin(lat_rad) * math.cos(angular_dist) +
        math.cos(lat_rad) * math.sin(angular_dist) * math.cos(bearing_rad)
    )

    lon2_rad = lon_rad + math.atan2(
        math.sin(bearing_rad) * math.sin(angular_dist) * math.cos(lat_rad),
        math.cos(angular_dist) - math.sin(lat_rad) * math.sin(lat2_rad)
    )

    return math.degrees(lat2_rad), math.degrees(lon2_rad)


def wrap_angle_180(angle: float) -> float:
    """Wrap angle to -180 to 180 degrees"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def wrap_angle_360(angle: float) -> float:
    """Wrap angle to 0 to 360 degrees"""
    while angle >= 360:
        angle -= 360
    while angle < 0:
        angle += 360
    return angle
