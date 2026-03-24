"""
Elevation and Azimuth Calculations
"""

import numpy as np
from typing import Tuple
from .coordinates import ecef_to_geodetic, ecef_to_enu

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


def calculate_elevation_azimuth(sat_ecef: Tuple[float, float, float], rx_ecef: Tuple[float, float, float]):
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
    lat, lon, _ = ecef_to_geodetic(rx_ecef)

    # Step 3: Transform ECEF vector to ENU coordinates
    e, n, u = ecef_to_enu(dx, dy, dz, np.radians(lat), np.radians(lon))

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
    lat, lon, h = ecef_to_geodetic(rx_ecef)
    print(f"\nReceiver Geodetic:")
    print(f"  Latitude: {np.degrees(lat):.6f}°")
    print(f"  Longitude: {np.degrees(lon):.6f}°")
    print(f"  Height: {h:.2f} m")