"""
Elevation and Azimuth Calculation Module

This module provides functions to calculate satellite elevation and azimuth angles
from ECEF coordinates of satellites and receivers.

Functions:
    ecef_to_geodetic: Convert ECEF coordinates to geodetic (lat, lon, height)
    ecef_to_enu: Transform ECEF vector to East-North-Up coordinates
    enu_to_elevation_azimuth: Calculate elevation and azimuth from ENU coordinates
    calculate_elevation_azimuth: Main function to compute angles from ECEF positions
"""

import numpy as np

# WGS-84 ellipsoid parameters
WGS84_A = 6378137.0  # Semi-major axis [meters]
WGS84_E2 = 6.69437999014e-3  # First eccentricity squared


def ecef_to_geodetic(x, y, z):
    """
    Convert ECEF coordinates to geodetic coordinates (latitude, longitude, height).

    Uses iterative algorithm for latitude calculation with WGS-84 ellipsoid.

    Parameters:
        x (float): ECEF X coordinate [meters]
        y (float): ECEF Y coordinate [meters]
        z (float): ECEF Z coordinate [meters]

    Returns:
        tuple: (latitude [radians], longitude [radians], height [meters])
    """
    # Longitude is straightforward
    lon = np.arctan2(y, x)

    # Calculate initial latitude estimate
    p = np.sqrt(x ** 2 + y ** 2)
    lat = np.arctan2(z, p * (1 - WGS84_E2))

    # Iterate to refine latitude (typically converges in 3-5 iterations)
    for _ in range(5):
        N = WGS84_A / np.sqrt(1 - WGS84_E2 * np.sin(lat) ** 2)
        h = p / np.cos(lat) - N
        lat = np.arctan2(z, p * (1 - WGS84_E2 * N / (N + h)))

    # Calculate final height
    N = WGS84_A / np.sqrt(1 - WGS84_E2 * np.sin(lat) ** 2)
    h = p / np.cos(lat) - N

    return lat, lon, h


def ecef_to_enu(dx, dy, dz, lat, lon):
    """
    Transform a vector from ECEF to East-North-Up (ENU) topocentric coordinates.

    The ENU coordinate system is centered at the observer location with:
    - E axis pointing East
    - N axis pointing North
    - U axis pointing Up (perpendicular to ellipsoid)

    Parameters:
        dx (float): ECEF X component of vector [meters]
        dy (float): ECEF Y component of vector [meters]
        dz (float): ECEF Z component of vector [meters]
        lat (float): Observer geodetic latitude [radians]
        lon (float): Observer geodetic longitude [radians]

    Returns:
        tuple: (east [meters], north [meters], up [meters])
    """
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_lon = np.sin(lon)
    cos_lon = np.cos(lon)

    # Apply rotation matrix from ECEF to ENU
    e = -sin_lon * dx + cos_lon * dy
    n = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    u = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz

    return e, n, u


def enu_to_elevation_azimuth(e, n, u):
    """
    Calculate elevation and azimuth angles from ENU coordinates.

    Parameters:
        e (float): East component [meters]
        n (float): North component [meters]
        u (float): Up component [meters]

    Returns:
        tuple: (elevation [degrees], azimuth [degrees])
            - elevation: 0° at horizon, 90° at zenith, negative below horizon
            - azimuth: 0° = North, 90° = East, 180° = South, 270° = West
    """
    # Calculate horizontal distance
    horizontal_dist = np.sqrt(e ** 2 + n ** 2)

    # Elevation angle (angle above horizon)
    elevation_rad = np.arctan2(u, horizontal_dist)
    elevation_deg = np.degrees(elevation_rad)

    # Azimuth angle (clockwise from North)
    azimuth_rad = np.arctan2(e, n)
    azimuth_deg = np.degrees(azimuth_rad)

    # Normalize azimuth to 0-360 degrees
    if azimuth_deg < 0:
        azimuth_deg += 360.0

    return elevation_deg, azimuth_deg


def calculate_elevation_azimuth(sat_ecef, rx_ecef):
    """
    Calculate satellite elevation and azimuth angles as seen from a receiver.

    This is the main function that combines all steps:
    1. Calculate satellite-to-receiver vector in ECEF
    2. Convert receiver position to geodetic coordinates
    3. Transform vector to ENU coordinates
    4. Calculate elevation and azimuth angles

    Parameters:
        sat_ecef (tuple or array): Satellite ECEF position (x, y, z) [meters]
        rx_ecef (tuple or array): Receiver ECEF position (x, y, z) [meters]

    Returns:
        tuple: (elevation [degrees], azimuth [degrees])
            - elevation: 0° at horizon, 90° at zenith, negative below horizon
            - azimuth: 0° = North, 90° = East, 180° = South, 270° = West

    Example:
        >>> sat_pos = (20000000, 15000000, 10000000)  # Satellite ECEF [m]
        >>> rx_pos = (4194304, 0, 4487348)  # Receiver ECEF [m]
        >>> elev, az = calculate_elevation_azimuth(sat_pos, rx_pos)
        >>> print(f"Elevation: {elev:.2f}°, Azimuth: {az:.2f}°")
    """
    # Step 1: Calculate vector from receiver to satellite
    dx = sat_ecef[0] - rx_ecef[0]
    dy = sat_ecef[1] - rx_ecef[1]
    dz = sat_ecef[2] - rx_ecef[2]

    # Step 2: Convert receiver ECEF position to geodetic
    lat, lon, _ = ecef_to_geodetic(rx_ecef[0], rx_ecef[1], rx_ecef[2])

    # Step 3: Transform ECEF vector to ENU coordinates
    e, n, u = ecef_to_enu(dx, dy, dz, lat, lon)

    # Step 4: Calculate elevation and azimuth angles
    elevation, azimuth = enu_to_elevation_azimuth(e, n, u)

    return elevation, azimuth


def calculate_slant_range(sat_ecef, rx_ecef):
    """
    Calculate the straight-line distance from receiver to satellite.

    Parameters:
        sat_ecef (tuple or array): Satellite ECEF position (x, y, z) [meters]
        rx_ecef (tuple or array): Receiver ECEF position (x, y, z) [meters]

    Returns:
        float: Slant range [meters]
    """
    dx = sat_ecef[0] - rx_ecef[0]
    dy = sat_ecef[1] - rx_ecef[1]
    dz = sat_ecef[2] - rx_ecef[2]

    return np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)


if __name__ == "__main__":
    # Example usage
    print("Elevation and Azimuth Calculation Module")
    print("=" * 50)

    # Example: Receiver at approximate location (45°N, 0°E)
    rx_ecef = (4194304.0, 0.0, 4487348.0)

    # Example: Satellite at GNSS orbit altitude
    sat_ecef = (20000000.0, 15000000.0, 10000000.0)

    # Calculate angles
    elev, az = calculate_elevation_azimuth(sat_ecef, rx_ecef)
    slant_range = calculate_slant_range(sat_ecef, rx_ecef)

    print(f"\nReceiver ECEF: {rx_ecef}")
    print(f"Satellite ECEF: {sat_ecef}")
    print(f"\nElevation: {elev:.2f}°")
    print(f"Azimuth: {az:.2f}°")
    print(f"Slant Range: {slant_range / 1000:.2f} km")

    # Convert receiver to geodetic for reference
    lat, lon, h = ecef_to_geodetic(rx_ecef[0], rx_ecef[1], rx_ecef[2])
    print(f"\nReceiver Geodetic:")
    print(f"  Latitude: {np.degrees(lat):.6f}°")
    print(f"  Longitude: {np.degrees(lon):.6f}°")
    print(f"  Height: {h:.2f} m")