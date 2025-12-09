"""
Calculate hydrostatic and wet temperature / water vapor coefficient for VMF mapping functions

This subroutine determines the coefficients bh, bw, ch, and cw of VMF3o. These coefficients are of empirical nature
containing a geographical and temporal dependence, represented in spherical harmonics. The spherical harmonics
coefficients are developed to degree and order 12 and are based on a 5°x5° grid containing ray-tracing data from
2001-2010.

History:

2025-12-09 python file created by Madison Gleydura
2018-12-07 m-file created by Janina Boisits
"""

from datetime import datetime, timedelta
import math
import numpy as np
import pandas as pd
from typing import Tuple
from software.scripts.geometric_range import TimeConverter
from software.scripts.ephemeris_classes import GNSSConstants

def vmf3o_b_c(rcv_pos: Tuple[float, float, float], utc_time: datetime = None, gps_week: int = None, gps_tow: float = None) -> Tuple[float, float, float, float]:
    """
    Calculate b and c coefficients

    Args:
        @type rcv_pos: Tuple[float, float, float]
        @param rcv_pos: Receiver ECEF coordinates (x, y, z) in meters

        @type utc_time: datetime
        @param utc_time: Time in system time (seconds)

        @type gps_week: int
        @param gps_week: GPS week number (optional if utc_time is provided)

        @type gps_tow: float
        @param gps_tow: GPS time of week in seconds (optional if utc_time is provided)

    Returns:
        @rtype: tuple
        @return: Tuple with bh, bw, ch, and cw coefficients of VMF3o
    """

    if gps_week is not None and gps_tow is not None:
        week = gps_week
        tow = gps_tow
    elif utc_time is not None:
        week, tow = TimeConverter.utc_to_gps_time(utc_time)
    else:
        raise ValueError("Must provide either utc_time or (gps_week, gps_tow)")

    a = GNSSConstants.GPS_A         # semi-major axis [m]
    f = 1 / GNSSConstants.GPS_FF    # flattening

    e2 = 2*f - f**2 # first eccentricity squared

    # Longitude
    lon = math.atan2(rcv_pos[1], rcv_pos[0])

    # Initial guess for latitude
    p = math.sqrt(rcv_pos[0]**2 + rcv_pos[1]**2)
    lat = math.atan2(rcv_pos[2], p*(1 - e2)) # Initial guess

    # Iterative solution
    for _ in range(5):
        N = a / math.sqrt(1 - e2 * math.sin(lat)**2)
        lat = math.atan2(rcv_pos[2] + e2 * N * math.sin(lat), p)

    gps_epoch = GNSSConstants.GPS_EPOCH

    # Convert week + TOW to datetime
    dt = gps_epoch + timedelta(weeks=week, seconds=tow)

    # Day-of-year
    doy = dt.timetuple().tm_yday

    # Fractional part of the day
    frac_day = (dt.hour + dt.minute/60 + dt.second/3600) / 24
    doy = doy + frac_day

    polar_distance = math.pi / 2 - lat # polar distance in radians

    # Unit vector of the station
    x = math.sin(polar_distance) * math.cos(lon)
    y = math.sin(polar_distance) * math.sin(lon)
    z = math.cos(polar_distance)

    # Read the coefficients of spherical harmonics
    column_names = ['A0', 'A1', 'B1', 'A2', 'B2']

    anm_bh = pd.read_csv("../../data/vmf3o_SH/anm_bh.txt", sep=r'\s+', names=column_names, dtype=float)
    bnm_bh = pd.read_csv('../../data/vmf3o_SH/bnm_bh.txt', sep=r'\s+', names=column_names, dtype=float)

    anm_bw = pd.read_csv("../../data/vmf3o_SH/anm_bw.txt", sep=r'\s+', names=column_names, dtype=float)
    bnm_bw = pd.read_csv( "../../data/vmf3o_SH/bnm_bw.txt", sep=r'\s+', names=column_names, dtype=float)

    anm_ch = pd.read_csv("../../data/vmf3o_SH/anm_ch.txt", sep=r'\s+', names=column_names, dtype=float)
    bnm_ch = pd.read_csv("../../data/vmf3o_SH/bnm_ch.txt", sep=r'\s+', names=column_names, dtype=float)

    anm_cw = pd.read_csv( "../../data/vmf3o_SH/anm_cw.txt", sep=r'\s+', names=column_names, dtype=float)
    bnm_cw = pd.read_csv( "../../data/vmf3o_SH/bnm_cw.txt", sep=r'\s+', names=column_names, dtype=float)

    # Compute the Legendre polynomials
    # degree/order of SH
    nmax = 12

    # Legendre polynomials
    V = np.zeros((nmax + 1, nmax + 1), dtype=float)
    W = np.zeros((nmax + 1, nmax + 1), dtype=float)

    V[0,0] = 1
    W[0,0] = 0
    V[1,0] = z * V[0,0]
    W[1,0] = 0

    for n in range(2, nmax + 1):
        V[n, 0] = ((2*n - 1) * z * V[n - 1, 0] - (n - 1) * V[n - 2, 0]) / n
        W[n, 0] = 0

    for m in range(1, nmax + 1):
        V[m, m] = (2*m - 1) * (x*V[m - 1, m - 1] - y*W[m - 1, m - 1])
        W[m, m] = (2*m - 1) * (x*W[m - 1, m - 1] + y*V[m - 1, m - 1])

        if m < nmax:
            V[m + 1, m] = (2*m + 1) * z * V[m, m]
            W[m + 1, m] = (2*m + 1) * z * W[m, m]

        for n in range(m + 2, nmax + 1):
            V[n, m] = ((2 * n - 1) * z * V[n - 1, m] - (n + m - 1) * V[n - 2, m]) / (n - m)
            W[n, m] = ((2 * n - 1) * z * W[n - 1, m] - (n + m - 1) * W[n - 2, m]) / (n - m)

    # Evaluate SH
    fields = column_names
    prefixes = ['bh', 'bw', 'ch', 'cw']

    # Organize dataframes into dictionaries
    anm = {'bh': anm_bh, 'bw': anm_bw, 'ch': anm_ch, 'cw': anm_cw}
    bnm = {'bh': bnm_bh, 'bw': bnm_bw, 'ch': bnm_ch, 'cw': bnm_cw}

    # Initialize coefficients dictionary
    coeffs = {p: {f: 0 for f in fields} for p in prefixes}

    # Compute sums
    i = -1

    for n in range(nmax + 1):
        for m in range(n + 1):
            i += 1
            for p in prefixes:
                for f in fields:
                    coeffs[p][f] += anm[p].at[i, f] * V[n, m] + bnm[p].at[i, f] * W[n, m]

    # Compute final coefficients
    omega1 = 2 * math.pi / 365.25
    omega2 = 4 * math.pi / 365.25

    cos1 = math.cos(doy * omega1)
    sin1 = math.sin(doy * omega1)
    cos2 = math.cos(doy * omega2)
    sin2 = math.sin(doy * omega2)

    results = []
    for key in ('bh', 'bw', 'ch', 'cw'):
        c = coeffs[key]
        value = c['A0'] + c['A1']*cos1 + c['B1']*sin1 + c['A2']*cos2 + c['B2']*sin2
        results.append(value)

    return tuple(results)

def main():
    """Example usage"""

    receiver_ecef = (867068.487, -5504812.066, 3092176.505)
    gps_week = 2394
    gps_tow = 228560

    print("\n" + "=" * 80)
    print("VMF3 Coefficients Calculation")
    print("=" * 80)

    bh, bw, ch, cw = vmf3o_b_c(receiver_ecef, gps_week=gps_week, gps_tow=gps_tow)

    print(f"bh: {bh:.8f}")
    print(f"bw: {bw:.8f}")
    print(f"ch: {ch:.8f}")
    print(f"cw: {cw:.8f}")

if __name__ == "__main__":
    main()