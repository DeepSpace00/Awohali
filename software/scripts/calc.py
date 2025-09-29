import numpy as np
from datetime import datetime, timezone

# Constants
MU = 3.986005e14          # Earth's universal gravitational parameter, m^3/s^2
OMEGA_E = 7.2921151467e-5 # Earth rotation rate, rad/s
SECONDS_IN_WEEK = 604800

# ------------------------------
# Helpers: time conversions
# ------------------------------
def datetime_to_gpsweek_seconds(dt):
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    else:
        dt = dt.astimezone(timezone.utc)
    gps_epoch = datetime(1980, 1, 6, tzinfo=timezone.utc)
    delta = dt - gps_epoch
    total_s = delta.total_seconds()
    week = int(total_s // SECONDS_IN_WEEK)
    sow = total_s - week * SECONDS_IN_WEEK
    return week, sow

def tow_wrap(dt_sec):
    if dt_sec > 302400:
        return dt_sec - 604800
    if dt_sec < -302400:
        return dt_sec + 604800
    return dt_sec

# ------------------------------
# Kepler solver
# ------------------------------
def solve_kepler(M, e, tol=1e-12, max_iter=60):
    M = np.mod(M, 2*np.pi)
    E = M if e < 0.8 else np.pi
    for _ in range(max_iter):
        f = E - e*np.sin(E) - M
        fprime = 1 - e*np.cos(E)
        dE = -f / fprime
        E += dE
        if abs(dE) < tol:
            break
    return E

# ------------------------------
# UBX dwrds -> ephemeris
# ------------------------------
def eph_from_dwrds(dwrd):
    eph = {}
    eph['sqrtA'] = ((dwrd[7] << 16) + dwrd[8]) * 2**-19
    eph['e'] = ((dwrd[4] << 16) + dwrd[5]) * 2**-33
    eph['i0'] = ((dwrd[8] >> 16) + (dwrd[9] & 0xFFFF)) * 2**-19
    eph['Omega0'] = ((dwrd[1] << 16) + dwrd[2]) * 2**-31
    eph['omega'] = ((dwrd[6] << 16) + (dwrd[7] & 0xFFFF)) * 2**-31
    eph['M0'] = ((dwrd[3] << 16) + dwrd[4]) * 2**-31
    eph['delta_n'] = ((dwrd[5] << 16) + dwrd[6]) * 2**-43
    eph['IDOT'] = ((dwrd[7] >> 16) + (dwrd[7] & 0xFFFF)) * 2**-43
    eph['OMEGA_DOT'] = ((dwrd[2] >> 16) + (dwrd[2] & 0xFFFF)) * 2**-43
    eph['Cuc'] = 0.0; eph['Cus'] = 0.0
    eph['Crc'] = 0.0; eph['Crs'] = 0.0
    eph['Cic'] = 0.0; eph['Cis'] = 0.0
    eph['toe'] = dwrd[0] * 2**4
    eph['af0'] = 0.0; eph['af1'] = 0.0; eph['af2'] = 0.0
    return eph

# ------------------------------
# Ephemeris -> ECEF
# ------------------------------
def sat_ecef_from_eph(eph, target_dt=None, gps_week=None, gps_sow=None):
    if target_dt is not None:
        week, sow = datetime_to_gpsweek_seconds(target_dt)
    else:
        if gps_week is None or gps_sow is None:
            raise ValueError("Provide target_dt or gps_week+gps_sow")
        week, sow = gps_week, gps_sow
    tk = sow - eph['toe']
    tk = tow_wrap(tk)
    a = eph['sqrtA']**2
    n0 = np.sqrt(MU / a**3)
    n = n0 + eph.get('delta_n', 0.0)
    M = eph['M0'] + n * tk
    E = solve_kepler(M, eph['e'])
    sin_v = np.sqrt(1 - eph['e']**2) * np.sin(E) / (1 - eph['e']*np.cos(E))
    cos_v = (np.cos(E) - eph['e']) / (1 - eph['e']*np.cos(E))
    v = np.arctan2(sin_v, cos_v)
    u0 = v + eph['omega']
    du = eph['Cuc'] * np.cos(2*u0) + eph['Cus'] * np.sin(2*u0)
    dr = eph['Crc'] * np.cos(2*u0) + eph['Crs'] * np.sin(2*u0)
    di = eph['Cic'] * np.cos(2*u0) + eph['Cis'] * np.sin(2*u0)
    u = u0 + du
    r = a * (1 - eph['e'] * np.cos(E)) + dr
    i = eph['i0'] + di + eph.get('IDOT', 0.0) * tk
    Omega = eph['Omega0'] + (eph.get('OMEGA_DOT', 0.0) - OMEGA_E) * tk - OMEGA_E * eph['toe']
    x_orb = r * np.cos(u)
    y_orb = r * np.sin(u)
    cos_O = np.cos(Omega); sin_O = np.sin(Omega)
    cos_i = np.cos(i); sin_i = np.sin(i)
    x = x_orb * cos_O - y_orb * cos_i * sin_O
    y = x_orb * sin_O + y_orb * cos_i * cos_O
    z = y_orb * sin_i
    return np.array([x, y, z])

# ------------------------------
# Slant range
# ------------------------------
def slant_range_from_eph(eph, r_ground_ecef, target_dt=None, gps_week=None, gps_sow=None):
    r_sat = sat_ecef_from_eph(eph, target_dt=target_dt, gps_week=gps_week, gps_sow=gps_sow)
    rng = np.linalg.norm(r_sat - np.asarray(r_ground_ecef))
    return rng, r_sat

# ------------------------------
# Example usage
# ------------------------------
if __name__ == "__main__":
    # Example dwrds (decimal integers) from UBX-RXM-SFRBX
    dwrd00, dwrd01, dwrd02, dwrd03, dwrd04, dwrd05, dwrd06, dwrd07, dwrd08, dwrd09 = \
        41040, 12345, 67890, 11111, 22222, 33333, 44444, 55555, 66666, 77777

    eph = eph_from_dwrds([dwrd00, dwrd01, dwrd02, dwrd03, dwrd04, dwrd05, dwrd06, dwrd07, dwrd08, dwrd09])

    # Ground ECEF point [m]
    r_ground = np.array([863780.5246, -5503166.7389, 3095957.7455])  # example

    target_dt = datetime(2025, 9, 29, 10, 7, 27, tzinfo=timezone.utc)
    rng, r_sat = slant_range_from_eph(eph, r_ground, target_dt=target_dt)
    print("Datetime target: slant range [km]:", rng/1000.0)
    print("Satellite ECEF [m]:", r_sat)

    # Or compute slant range by GPS week + sow
    gps_week = 2386
    gps_sow  = 122866.988
    rng2, r_sat2 = slant_range_from_eph(eph, r_ground, gps_week=gps_week, gps_sow=gps_sow)
    print("Week+SOW target: slant range [km]:", rng2/1000.0)

    pseudorange_m = 17325363.9148477
    pseudorange_km = pseudorange_m / 1000

    print("Pseudorange [km]: ", pseudorange_km)

    print("Semi-major axis a [km]:", eph['sqrtA']**2 / 1000)
    print("eccentricity:", eph['e'])
    print("inclination [rad]:", eph['i0'])
    print("right_ascension:", eph['Omega0'])
    print("argument_of_perigee:", eph['omega'])
    print("mean_anomaly:", eph['M0'])