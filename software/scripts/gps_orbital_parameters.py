import numpy as np
from datetime import datetime, timezone

# Constants
MU = 3.986005e14           # Earth's gravitational parameter [m^3/s^2]
OMEGA_E = 7.2921151467e-5  # Earth rotation rate [rad/s]
SECONDS_IN_WEEK = 604800

# Example ephemeris for GPS G21 (from your RINEX file)
eph = {
    'af0': 0.00026156893000006676,
    'af1': 1.830358087318018e-11,
    'af2': 0.0,
    'Crs': -54.46875,
    'delta_n': 1.4798615666222759e-09,
    'M0': 0.6415577214211226,
    'Cuc': -2.7418136596679688e-06,
    'e': 0.00011470273602753878,
    'Cus': 5.023553967475891e-06,
    'sqrtA': 5153.678485870361,
    'toe': 129600,
    'Cic': 2.2351741790771484e-08,
    'Omega0': 0.19263618672266603,
    'Cis': 4.470348358154297e-08,
    'i0': 0.30584465991705656,
    'Crc': 282.90625,
    'omega': 0.058211587369441986,
    'OMEGA_DOT': -2.5937652026186697e-09,
    'IDOT': 2.773958840407431e-11
}

def solve_kepler(M, e, tol=1e-12, max_iter=100):
    E = M
    for _ in range(max_iter):
        E_new = E - (E - e*np.sin(E) - M)/(1 - e*np.cos(E))
        if abs(E_new - E) < tol:
            return E_new
        E = E_new
    return E

def sat_ecef_from_eph(eph, target_dt):
    # Convert datetime to GPS seconds of week
    gps_epoch = datetime(1980, 1, 6, tzinfo=timezone.utc)
    dt = target_dt.replace(tzinfo=timezone.utc)
    sow = (dt - gps_epoch).total_seconds() % SECONDS_IN_WEEK

    tk = sow - eph['toe']
    # wrap to ±302400s
    if tk > 302400: tk -= SECONDS_IN_WEEK
    if tk < -302400: tk += SECONDS_IN_WEEK

    a = eph['sqrtA']**2
    n0 = np.sqrt(MU / a**3)
    n = n0 + eph['delta_n']
    M = eph['M0'] + n * tk
    E = solve_kepler(M, eph['e'])

    # True anomaly
    v = np.arctan2(np.sqrt(1 - eph['e']**2)*np.sin(E), np.cos(E)-eph['e'])
    u = v + eph['omega']  # argument of latitude
    r = a*(1 - eph['e']*np.cos(E))
    i = eph['i0'] + eph['IDOT']*tk
    Omega = eph['Omega0'] + (eph['OMEGA_DOT'] - OMEGA_E)*tk - OMEGA_E*eph['toe']

    x_orb = r*np.cos(u)
    y_orb = r*np.sin(u)
    cos_O = np.cos(Omega); sin_O = np.sin(Omega)
    cos_i = np.cos(i); sin_i = np.sin(i)
    x = x_orb*cos_O - y_orb*cos_i*sin_O
    y = x_orb*sin_O + y_orb*cos_i*cos_O
    z = y_orb*sin_i
    return np.array([x, y, z])

def slant_range(eph, r_ground_ecef, target_dt):
    r_sat = sat_ecef_from_eph(eph, target_dt)
    rng = np.linalg.norm(r_sat - np.asarray(r_ground_ecef))
    return rng, r_sat

# Ground station ECEF [m]
r_ground = np.array([863780.3953, -5503166.7779, 3095958.0275])

# Target datetime
from datetime import datetime, timezone
target_dt = datetime(2025, 9, 29, 10, 7, 27, tzinfo=timezone.utc)

# Compute slant range
rng, r_sat = slant_range(eph, r_ground, target_dt)
print(f"Slant range [km]: {rng/1000:.3f}")
print(f"Satellite ECEF [m]: {r_sat}")

17325.3639148477