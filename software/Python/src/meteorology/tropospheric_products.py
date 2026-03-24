"""
Calculate precipitable water vapor from tropospheric delay
"""

import math
from typing import Tuple
import numpy as np

class EarthParameters:
    """Physical values and constants for PWV calculations"""
    # Atmospheric refraction constants
    K1 = 77.689 # [K/mbar]
    K2 = 71.2952  # [K/mbar]
    K3 = 375463 # [K^2/mbar]

    # Gas constants
    Rd = 287.0  # [J/(K*kg)]
    Rw = 461.0  # [J/(K*kg)]

    # Molar mass of dry air
    Md = 28.9644    # [g/mol]

    # Molar mass of water vapor
    Mw = 18.01528    # [g/mol]

    # Density of liquid water
    RHOw = 997  # [kg/m^3]

    # WGS84 parameters
    A = 6378137.0  # semi-major axis (m)
    B = 6356752.314245  # semi-minor axis (m)
    F = 1.0 / 298.257223563  # flattening

    # Standard gravity parameters
    G_E = 9.7803253359  # equatorial gravity (m/s^2)
    G_P = 9.8321849378  # polar gravity (m/s^2)

def bevis_tm(surface_temp_k):
    """Bevis (1992) mean atmospheric temperature from surface temp."""
    return 70.2 + 0.72 * surface_temp_k

def saastamoinen_zhd(station_lat: float, station_hgt: float, p: float) -> float:
    """
    Calculate zenith hydrostatic delay

    Args:
        @type rcv_pos: Tuple[float, float, float]
        @param rcv_pos: Receiver ECEF coordinates (x, y, z) in meters

        @type p: float
        @param p: Ground air pressure in hPa
    """

    zhd = (0.0022768 * p) / (1 - 0.00266 * math.cos(2*math.radians(station_lat)) - 0.28/1e6 * station_hgt)

    return zhd

def conversion_coefficient(t:float) -> float:
    k1 = EarthParameters.K1
    k2 = EarthParameters.K2
    k3 = EarthParameters.K3
    Rd = EarthParameters.Rd
    Rw = EarthParameters.Rw
    Md = EarthParameters.Md
    Mw = EarthParameters.Mw
    rho_w = EarthParameters.RHOw

    k2_prime = k2 - k1 * Rd/Rw

    # Calculate weighted mean temperature of the atmospheric air column (rough calc)
    t_m = 70.2 + 0.72 * (t + 273.15)

    Pi = 1e8 / ((k3/t_m + k2_prime - k1 * Mw/Md) * Rw * rho_w)

    return Pi

def calculate_mapping_function(elevation, a: float, b: float, c: float):
    """
    Calculate Marini continued-fraction mapping function using VMF3 coefficients
    """

    el_rad = np.radians(elevation)

    mf = (1 + a/(1 + b/(1 + c))) / (np.sin(el_rad) + a/(np.sin(el_rad) + b/(np.sin(el_rad) + c)))

    return mf

def zwd_to_pwv(zwd_m, tm_k) -> float:
    k1 = EarthParameters.K1
    k2 = EarthParameters.K2
    k3 = EarthParameters.K3
    Rd = EarthParameters.Rd
    Rw = EarthParameters.Rw
    Md = EarthParameters.Md
    Mw = EarthParameters.Mw
    rho_w = EarthParameters.RHOw

    k2_prime = k2 - k1 * Rd / Rw

    Pi = 1e8 / ((k3 / tm_k + k2_prime - k1 * Mw / Md) * Rw * rho_w)

    return (zwd_m / Pi) * 1000.0 # -> mm

def calculate_pwv(station_lat: float, station_hgt: float, elevation: float, troposphericDelay: float, t: float,
                  p: float, mapping_coeffs: Tuple[float, float, float, float, float, float]) -> Tuple[float, float]:
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
    conversion_coeff = conversion_coefficient(t)

    # Compute zenith hydrostatic delay
    zhd = saastamoinen_zhd(station_lat, station_hgt, p)

    # Compute zenith wet delay
    zwd = (troposphericDelay - mf_h * zhd) / mf_w

    pwv = zwd * conversion_coeff

    ztd = zhd + pwv/conversion_coeff

    return pwv, ztd

