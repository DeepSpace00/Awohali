from collections import defaultdict
import numpy as np
from pathlib import Path
import pandas as pd
import sqlite3
import sys
import time

from calculations import clock_correction, geometric_range
from ephemerides.ephemeris import load_ephemeris, load_ephemeris_data
from calculations.elevation_azimuth import calculate_elevation_azimuth

c = 299792458.0  # Speed of light (m/s)

# ─────────────────────────────────────────────
# File / database paths
# ─────────────────────────────────────────────
_DATA_data = Path("/mnt/NAS/Documents/Awohali/data")

_DATA = Path(__file__).parent.parent / "data"

rawx_file   = _DATA_data / "ubx_data/2025-11-25/2025-11-25_93138_serial-COM3_RXM_RAWX.csv"
clock_file  = _DATA_data / "ubx_data/2025-11-25/2025-11-25_93138_serial-COM3_NAV_CLOCK.csv"
ephemeris   = _DATA_data / "ephemerides/ephemeris_2025-11-25_RINEX.json"
results_dir = _DATA_data / "ubx_data/2025-11-25/2025-11-25_serial-COM3_pwv_testing_fast"


results_dir.mkdir(parents=True, exist_ok=True)

conn = sqlite3.connect(_DATA / "test_raw.db")

# ─────────────────────────────────────────────
# Station / met parameters
# ─────────────────────────────────────────────
receiver_ecef = (867068.487, -5504812.066, 3092176.505)  # Campus quad
# receiver_ecef = (867068.487, -5504812.066, 3092176.505) # Apartment

surface_temperature = 25.0                  # °C
surface_pressure    = 1021.3348218666767    # hPa

# ─────────────────────────────────────────────
# VMF3 mapping coefficients
# ─────────────────────────────────────────────
ah, aw = 1.2451128509e-03, 6.0843905260e-04
bh, bw = 2.7074805444e-03, 1.4082272735e-03
ch, cw = 5.6279169098e-02, 4.0289703115e-02
coeffs = (ah, aw, bh, bw, ch, cw)

ELEV_CUTOFF_DEG = 20.0  # degrees — applied AFTER geometry computation

ephemeris_data = load_ephemeris_data(ephemeris)

# ─────────────────────────────────────────────
# UBX signal plan (gnssId → sigId → service/freq)
# ─────────────────────────────────────────────
signal_plan = {
    0: {
        0:  {'service': 'L1_C/A',   'freq': 1575.42},   # L1 Civilian signal [MHz]
        3:  {'service': 'L2_CL',    'freq': 1227.60},   # L2 Pilot signal
        4:  {'service': 'L2_CM',    'freq': 1227.60},   # L2 Data signal
        6:  {'service': 'L5_I',     'freq': 1176.45},   # L5 Data signal
        7:  {'service': 'L5_Q',     'freq': 1176.45}    # L5 Pilot signal
    },
    2: {
        0:  {'service': 'E1_C',     'freq': 1575.42},   # E1 Pilot signal [MHz]
        1:  {'service': 'E1_B',     'freq': 1575.42},   # E1 Data signal
        3:  {'service': 'E5_aI',    'freq': 1176.45},   # E5a Data signal
        4:  {'service': 'E5_aQ',    'freq': 1176.45},   # E5a Pilot signal
        5:  {'service': 'E5_bI',    'freq': 1207.14},   # E5b Data signal
        6:  {'service': 'E5_bQ',    'freq': 1207.14},   # E5b Pilot signal
        8:  {'service': 'E6_B',     'freq': 1278.75},   # E6 Data signal
        9:  {'service': 'E6_C',     'freq': 1278.75},   # E6 Pilot signal
        10: {'service': 'E6_A',     'freq': 1278.75}    # E6 PRS signal
    }
}


# ══════════════════════════════════════════════
# Helpers — signal processing
# ══════════════════════════════════════════════

def make_if_pseudorange(pr1, pr2, f1_mhz, f2_mhz):
    """Ionosphere-free pseudorange combination (metres)."""
    f1 = f1_mhz * 1e6
    f2 = f2_mhz * 1e6
    return (pr1 * f1 ** 2 - pr2 * f2 ** 2) / (f1 ** 2 - f2 ** 2)


def make_if_carrier(cp1, cp2, f1_mhz, f2_mhz):
    """Ionosphere-free carrier phase combination (cycles)."""
    f1 = f1_mhz * 1e6
    f2 = f2_mhz * 1e6
    return (cp1 * f1 ** 2 - cp2 * f2 ** 2) / (f1 ** 2 - f2 ** 2)


def parse_ubx_signals(satellite_data, constellation, signal_plan):
    """
    Parse UBX RAWX signals for one satellite at one epoch.
    Returns (pseudorange_m, carrierPhase, freq_label) or (None, None, None).
    Preference: highest+lowest freq IF combination > single-frequency fallback.
    """
    sigIDs = satellite_data['sigId'].unique()

    pr_low = 0
    pr_high = 0
    cp_low = 0
    cp_high = 0

    if len(sigIDs) > 1:
        freq_sig_pairs = []
        for sig in sigIDs:
            freq = float(signal_plan[constellation][sig]['freq'])
            row = satellite_data[satellite_data['sigId'] == sig].iloc[0]
            freq_sig_pairs.append((freq, float(row['prMes']), float(row['cpMes'])))

        freq_sig_pairs.sort(key=lambda x: x[0])

        freq_low, pr_low, cp_low = freq_sig_pairs[0]
        freq_high, pr_high, cp_high = freq_sig_pairs[-1]

        if freq_low == freq_high:
            pseudorange_m = pr_high
            carrier_phase = cp_high
            freq_label = signal_plan[constellation][sigIDs[0]]['service']
        else:
            pseudorange_m = make_if_pseudorange(pr_high, pr_low, freq_high, freq_low)
            carrier_phase = make_if_carrier(cp_high, cp_low, freq_high, freq_low)
            svc_high = signal_plan[constellation][sigIDs[-1]]['service'].split('_')[0]
            svc_low = signal_plan[constellation][sigIDs[0]]['service'].split('_')[0]
            freq_label = f"{svc_high}_{svc_low}"
    else:
        pseudorange_m = float(satellite_data.iloc[0]['prMes'])
        carrier_phase = float(satellite_data.iloc[0]['cpMes'])
        freq_label = signal_plan[constellation][sigIDs[0]]['service']

    return pseudorange_m, carrier_phase, freq_label, pr_low, pr_high, cp_low, cp_high


def correct_rcvtow(rcvTow_s, clkBias_ns):
    """
    rcvTOW and iTOW become out of sync when clkBias_ns exceeds ±0.5 ms
    (rcvTOW rounds because it only has 3 decimal places).
    """
    if clkBias_ns >= 500000:
        return rcvTow_s - 0.001
    elif clkBias_ns <= -500000:
        return rcvTow_s + 0.001
    return rcvTow_s


# ══════════════════════════════════════════════
# Helpers — mapping functions & estimation
# ══════════════════════════════════════════════

def marini(elevation_deg, a, b, c_coeff):
    """Marini continued-fraction mapping function."""
    sin_el = np.sin(np.radians(elevation_deg))
    num = 1.0 + a / (1.0 + b / (1.0 + c_coeff))
    den = sin_el + a / (sin_el + b / (sin_el + c_coeff))
    return num / den


# ══════════════════════════════════════════════
# Helpers — geodetic & meteorological
# ══════════════════════════════════════════════

def ecef_to_geodetic(x, y, z):
    """Bowring iterative ECEF → geodetic (WGS-84). Returns (lat_deg, lon_deg, height_m)."""
    a  = 6378137.0
    f  = 1.0 / 298.257223563
    b  = a * (1.0 - f)
    e2 = 2*f - f**2

    p   = np.sqrt(x**2 + y**2)
    lon = np.arctan2(y, x)

    lat = np.arctan2(z, p * (1.0 - e2))
    for _ in range(10):
        sin_lat = np.sin(lat)
        N       = a / np.sqrt(1.0 - e2 * sin_lat**2)
        lat_new = np.arctan2(z + e2 * N * sin_lat, p)
        if abs(lat_new - lat) < 1e-12:
            break
        lat = lat_new

    sin_lat = np.sin(lat)
    N = a / np.sqrt(1.0 - e2 * sin_lat**2)
    h = p / np.cos(lat) - N if abs(lat) < np.radians(45) else \
        z / np.sin(lat) - N * (1.0 - e2)

    return np.degrees(lat), np.degrees(lon), h


def saastamoinen_zhd(pressure_hpa, lat_deg, height_km):
    """Saastamoinen (1972) / Davis et al. (1985) ZHD in metres."""
    return (0.0022768 * pressure_hpa) / (
        1.0 - 0.00266 * np.cos(2.0 * np.radians(lat_deg))
            - 0.00028 * height_km
    )


def bevis_tm(surface_temp_k):
    """Bevis (1992) mean atmospheric temperature from surface temp."""
    return 70.2 + 0.72 * surface_temp_k


# ══════════════════════════════════════════════
# Load data
# ══════════════════════════════════════════════
rawx  = pd.read_csv(rawx_file)
clock = pd.read_csv(clock_file)
rawx_length = len(rawx)
total_measurements = len(rawx)

# Compute station geodetic coordinates and a priori ZHD once
station_lat, station_lon, station_hgt = ecef_to_geodetic(*receiver_ecef)
station_hgt_km = station_hgt / 1000.0
zhd_prior = saastamoinen_zhd(surface_pressure, station_lat, station_hgt_km)
tm = bevis_tm(surface_temperature + 273.15)

print(f"Station geodetic: lat={station_lat:.6f}°  lon={station_lon:.6f}°  h={station_hgt:.3f} m")
print(f"A priori ZHD (Saastamoinen): {zhd_prior:.4f} m")
print(f"Mean atmospheric temperature Tm: {tm:.1f} K")

start_time = time.time()

epoch_buffer = defaultdict(list)

i = 0

while True:
    progress = (i + 1) / total_measurements
    elapsed = time.time() - start_time
    if i > 0:
        eta_s = (elapsed / (i + 1)) * (total_measurements - i - 1)
        eta_str = (f"{eta_s:.0f}s" if eta_s < 60 else
                   f"{eta_s / 60:.1f}m" if eta_s < 3600 else
                   f"{eta_s / 3600:.1f}h")
    else:
        eta_str = "calculating..."
    bar = '█' * int(40 * progress) + '-' * (40 - int(40 * progress))
    sys.stdout.write(f'\r|{bar}| {progress * 100:.1f}% ({i + 1}/{total_measurements}) ETA: {eta_str}')
    sys.stdout.flush()

    numMeas = rawx.iloc[i]['numMeas']
    epoch_data = rawx.loc[i:(i + numMeas - 1), [
        'rcvTow', 'week', 'prMes', 'cpMes', 'doMes', 'gnssId', 'svId',
        'sigId', 'locktime', 'cno', 'prStdev', 'cpStdev',
        'prValid', 'cpValid', 'halfCyc', 'subHalfCyc']]

    rcvTow_s = epoch_data['rcvTow'].values[0]
    gpsWeek  = epoch_data['week'].values[0]

    closest_iTow = clock.iloc[(clock['iTOW'] / 1000.0 - rcvTow_s).abs().argsort()[:1]]
    iTow_ms      = closest_iTow['iTOW'].values[0]
    clkBias_ns   = closest_iTow['clkB'].values[0]
    clkDrift_ns  = closest_iTow['clkD'].values[0]

    rcvTow_sat_s = correct_rcvtow(rcvTow_s, clkBias_ns)
    dt_iTow_s    = rcvTow_sat_s - (iTow_ms / 1000.0)
    dt_bias_s    = (clkBias_ns + clkDrift_ns * dt_iTow_s) / 1e9
    rcv_clkBias_nav_s = dt_iTow_s + dt_bias_s
    gpsTow_s     = rcvTow_sat_s - rcv_clkBias_nav_s

    rcv_clock_m = rcv_clkBias_nav_s * c

    gnssIDs = epoch_data['gnssId'].unique()
    for constellation in gnssIDs:

        epoch_constellation_data = epoch_data[epoch_data['gnssId'] == constellation]
        svIDs = epoch_constellation_data['svId'].unique()
        sat_buffer = []
        for satellite in svIDs:
            if constellation == 0:
                pvn = f'G{satellite:02d}'
            elif constellation == 2:
                pvn = f'E{satellite:02d}'
            else:
                continue

            satellite_data = epoch_constellation_data[epoch_constellation_data['svId'] == satellite]

            pseudorange_m, carrier_phase, freq_label, pr_low, pr_high, cp_low, cp_high = parse_ubx_signals(
                satellite_data, constellation, signal_plan)
            if pseudorange_m is None:
                continue

            try:
                sat = load_ephemeris(ephemeris_data, pvn, gps_tow=gpsTow_s)
            except KeyError:
                continue
            except ValueError:
                continue

            try:
                gr = geometric_range.calculate_satellite_position_and_range(
                    ephemeris_data, pvn, receiver_ecef,
                    gps_week=gpsWeek, gps_tow=gpsTow_s)
            except Exception:
                continue

            geo_range_m = gr['geometric_range_m']
            t_tx_s      = gr['transmission_time']
            sat_ecef    = (gr['satellite_ecef_m']['x'],
                           gr['satellite_ecef_m']['y'],
                           gr['satellite_ecef_m']['z'])

            dt_sv_s  = clock_correction.calculate_satellite_clock_offset(sat, t_tx_s)
            dt_rel_s = clock_correction.calculate_relativistic_clock_correction(sat, t_tx_s)

            raw_residual = pseudorange_m - geo_range_m + (dt_sv_s + dt_rel_s) * c - rcv_clock_m

            elevation_deg, azimuth_deg = calculate_elevation_azimuth(sat_ecef, receiver_ecef)

            sat_buffer.append({
                'pvn':            pvn,
                'rcvTOW':         rcvTow_s,
                'gpsTOW':         gpsTow_s,
                'iTOW':           iTow_ms / 1000.0,
                'rcvTOW_sat':     rcvTow_sat_s,
                'clkBias_ns':     clkBias_ns,
                'clkDrift_ns':    clkDrift_ns,
                'gpsWeek':        gpsWeek,
                't_tx':           t_tx_s,
                'geoRange':       geo_range_m,
                'elevation':      elevation_deg,
                'azimuth':        azimuth_deg,
                'satClockBias':   dt_sv_s * c,
                'relBias':        dt_rel_s * c,
                'pseudorange_combined':    pseudorange_m,
                'pseudorange_L1': pr_low,
                'pseudorange_L5': pr_high,
                'carrierPhase_combined':   carrier_phase,
                'carrierPhase_L1': cp_low,
                'carrierPhase_L5': cp_high,
                'freqLabel':      freq_label,
                'raw_residual':   raw_residual,
                'rcv_clock_m':      rcv_clock_m
            })

        for entry in sat_buffer:
            epoch_buffer[entry['pvn']].append(entry)

    i += numMeas
    if i >= rawx_length:
        break

sys.stdout.write('\n')

# ══════════════════════════════════════════════
# Save to SQLite and print summary
# ══════════════════════════════════════════════
total_time = time.time() - start_time
print(f"\nCompleted in {total_time:.1f} seconds")

# Per-satellite tables
gnss_results = {}
for pvn in epoch_buffer:
    gnss_results[pvn] = pd.DataFrame(epoch_buffer[pvn])
    gnss_results[pvn].to_sql(pvn, conn, if_exists='replace', index=False)

print(f"Saved {len(gnss_results)} satellite tables to database.")