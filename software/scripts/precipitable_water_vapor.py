"""
Calculate precipitable water vapor from tropospheric delay
"""

import math
from typing import Tuple

class EarthParameters:
    """Physical values and constants for PWV calculations"""
    # Atmospheric refraction constants
    K1 = 77.604 # [K/mbar]
    K2 = 64.79  # [K/mbar]
    K3 = 377600 # [K^2/mbar]

    # Gas constants
    Rd = 287.0  # [J/(K*kg)]
    Rw = 461.0  # [J/(K*kg)]

    # Molar mass of water vapor
    Md = 28.9644    # [g/mol]
    Mw = 15.9994    # [g/mol]

    # Density of liquid water
    RHOw = 977  # [kg/m^3]

    # WGS84 parameters
    A = 6378137.0  # semi-major axis (m)
    B = 6356752.314245  # semi-minor axis (m)
    F = 1.0 / 298.257223563  # flattening

    # Standard gravity parameters
    G_E = 9.7803253359  # equatorial gravity (m/s^2)
    G_P = 9.8321849378  # polar gravity (m/s^2)

    #

def calculate_ellipsoidal_height(rcv_pos: Tuple[float, float, float]) -> Tuple[float, float]:
    """
    Calculate acceleration due to gravity at the center of mass of the vertical column (of atmosphere)

    Args:
        @type rcv_pos: Tuple[float, float, float]
        @param rcv_pos: Receiver ECEF coordinates (x, y, z) in meters
    """

    # Calculate geodetic latitude from ECEF
    p = math.sqrt(rcv_pos[0]**2 + rcv_pos[1]**2)
    e2 = 1 - (EarthParameters.B / EarthParameters.A)**2 # First eccentricity squared

    # Iterative solution for geodetic latitude
    lat = math.atan2(rcv_pos[2], p * (1 - e2))
    for _ in range(5):
        N = EarthParameters.A / math.sqrt(1 - e2 * math.sin(lat)**2)
        lat = math.atan2(rcv_pos[2] + e2 * N * math.sin(lat), p)

    # Calculate ellipsoidal height
    N = EarthParameters.A / math.sqrt(1 - e2 * math.sin(lat)**2)
    h = p / math.cos(lat) - N

    return h, lat

def calculate_zenith_hydrostatic_delay(rcv_pos: Tuple[float, float, float], p: float) -> float:
    """
    Calculate zenith hydrostatic delay

    Args:
        @type rcv_pos: Tuple[float, float, float]
        @param rcv_pos: Receiver ECEF coordinates (x, y, z) in meters

        @type p: float
        @param p: Ground air pressure in Pa
    """

    h_ellipsoid, lat = calculate_ellipsoidal_height(rcv_pos)

    zhd = (0.0022768 * p) / (1 - 0.00266 * math.cos(2*lat) - 0.28/1e6 * h_ellipsoid)

    return zhd

def calculate_conversion_coefficient(t:float) -> float:
    k1 = EarthParameters.K1
    k2 = EarthParameters.K2
    k3 = EarthParameters.K3
    Rd = EarthParameters.Rd
    Rw = EarthParameters.Rw
    Md = EarthParameters.Md
    Mw = EarthParameters.Mw
    rho_w = EarthParameters.RHOw

    k2_prime = k2 - k1 * Rd/Rw

    # Calculate weighted mean temperature of tha atmospheric air column (rough calc)
    t_m = 70.2 + 0.72 * (t + 273.15)

    conversion_coeff = 1e6 / ((k3/t_m + k2_prime - k1 * Mw/Md) * Rw * rho_w)

    return conversion_coeff

def calculate_mapping_function(elevation: float, a: float, b: float, c: float) -> float:
    """
    Calculate mapping function using VMF3 coefficients
    """

    el_rad = math.radians(elevation)

    mf = (1 + a/(1 + b/(1 + c))) / (math.sin(el_rad) + a/(math.sin(el_rad) + b/(math.sin(el_rad) + c)))

    return mf

def calculate_precipitable_water_vapor(rcv_pos: Tuple[float, float, float], elevation: float, troposphericDelay: float, t: float, p: float, mapping_coeffs: Tuple[float, float, float, float, float, float]) -> Tuple[float, float]:
    """
    Calculate precipitable water vapor

    Args:
        @type rcv_pos: Tuple[float, float, float]
        @param rcv_pos: Receiver ECEF coordinates (x, y, z) in meters

        @type elevation: float
        @param elevation: satellite elevation in degrees

        @type troposphericDelay: float
        @param troposphericDelay: tropospheric delay in meters

        @type t: float
        @param t: Ground air temperature in degrees Celsius

        @type p: float
        @param p: Ground air pressure in Pa

        @type mapping_coeffs: Tuple[float, float, float, float, float, float]
        @param mapping_coeffs: VMF3 air pressure, temperature, and water vapor mapping coefficients

    Returns:
        @rtype: float
        @return: precipitable water vapor
    """

    ah = mapping_coeffs[0]
    aw = mapping_coeffs[1]
    bh = mapping_coeffs[2]
    bw = mapping_coeffs[3]
    ch = mapping_coeffs[4]
    cw = mapping_coeffs[5]

    # Compute mapping functions
    mf_h = calculate_mapping_function(elevation, ah, bh, ch)
    mf_w = calculate_mapping_function(elevation, aw, bw, cw)

    # Compute conversion coefficient
    conversion_coeff = calculate_conversion_coefficient(t)

    # Compute zenith hydrostatic delay
    zhd = calculate_zenith_hydrostatic_delay(rcv_pos, p)

    pwv = conversion_coeff * (troposphericDelay - mf_h * zhd) / mf_w

    ztd = zhd + pwv/conversion_coeff

    return pwv, ztd

