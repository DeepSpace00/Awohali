import numpy as np
from typing import Tuple

class EarthParameters:
    # WGS84 parameters
    A = 6378137.0  # semi-major axis (m)
    B = 6356752.314245  # semi-minor axis (m)
    F = 1.0 / 298.257223563  # flattening

    # Standard gravity parameters
    G_E = 9.7803253359  # equatorial gravity (m/s^2)
    G_P = 9.8321849378  # polar gravity (m/s^2)

def ecef_to_geodetic(rcv_pos: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """
    Convert ECEF coordinates to geodetic coordinates (latitude, longitude, height).

    Uses iterative algorithm for latitude calculation with WGS-84 ellipsoid.

    Args:
        @type rcv_pos: Tuple[float, float, float]

    Returns:
        tuple: (latitude [radians], longitude [radians], height [meters])
    """

    x = rcv_pos[0]
    y = rcv_pos[1]
    z = rcv_pos[2]

    e2 = 1 - (EarthParameters.B / EarthParameters.A) ** 2  # First eccentricity squared

    # Longitude is straightforward
    lon = np.arctan2(y, x)

    # Calculate initial latitude estimate
    p = np.sqrt(x ** 2 + y ** 2)
    lat = np.arctan2(z, p * (1 - e2))

    # Iterate to refine latitude (typically converges in 3-5 iterations)
    for _ in range(5):
        N = EarthParameters.A / np.sqrt(1 - e2 * np.sin(lat) ** 2)
        h = p / np.cos(lat) - N
        lat = np.arctan2(z, p * (1 - e2 * N / (N + h)))

    # Calculate final height
    N = EarthParameters.A / np.sqrt(1 - e2 * np.sin(lat) ** 2)
    h = p / np.cos(lat) - N

    lat = np.degrees(lat)
    lon = np.degrees(lon)

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