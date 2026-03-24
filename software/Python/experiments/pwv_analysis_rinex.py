from collections import defaultdict
from datetime import datetime
import numpy as np
from pathlib import Path
import pandas as pd
import sqlite3
import sys
import time

from calculations import clock_correction, geometric_range
from ephemerides.ephemeris import load_ephemeris, load_ephemeris_data
from calculations.datetime_conversion import datetime_to_gps_tow
from calculations.elevation_azimuth import calculate_elevation_azimuth
from meteorology.old_trop_products import calculate_precipitable_water_vapor

c = 299792458.0  # Speed of light (m/s)

gps_frequency_plan = {
    'L1': 1575.42,   # MHz
    'L2': 1227.6,    # MHz
    'L5': 1176.45    # MHz
}

# ─────────────────────────────────────────────
# File / database paths
# ─────────────────────────────────────────────
_DATA = Path(__file__).parent.parent / "data"

# rinex_file   = _DATA / "RINEX_data/ORMD/2025-11-25/ormd3290_excerpt.csv"
# ephemeris    = _DATA / "ephemerides/ephemeris_2025-11-25_RINEX.json"
# results_dir  = _DATA / "RINEX_data/ORMD/2025-11-25/RINEX_pwv_test_new"

rinex_file   = _DATA / "RINEX_data/ORMD/2026-02-28/ormd0590_GPS_Galileo_excerpt.csv"
ephemeris    = _DATA / "ephemerides/ephemeris_2026-02-28_RINEX.json"
results_dir  = _DATA / "RINEX_data/ORMD/2026-02-28/ormd0590_GPS_Galileo_excerpt"

results_dir.mkdir(parents=True, exist_ok=True)

# conn = sqlite3.connect(_DATA / "RINEX_data/ORMD/2025-09-30/ormd2730.db")

conn = sqlite3.connect(_DATA / "RINEX_data/ORMD/2026-02-28/ormd0590_GPS_Galileo_excerpt.db")

# ─────────────────────────────────────────────
# Station / met parameters
# ─────────────────────────────────────────────
receiver_ecef       = (860376.4154, -5499833.4036, 3102756.9385)  # CORS ORMD
surface_temperature = 25.0                   # °C
surface_pressure    = 846.597166666675       # hPa

# ─────────────────────────────────────────────
# VMF3 mapping coefficients
# ─────────────────────────────────────────────
ah, aw = 1.2566852371e-03, 6.7752025554e-04
bh, bw = 2.7038091444e-03, 1.4050786566e-03
ch, cw = 5.6422851553e-02, 4.0585677414e-02
coeffs = (ah, aw, bh, bw, ch, cw)

ELEV_CUTOFF_DEG = 20.0   # degrees — applied AFTER receiver clock estimation

ephemeris_data = load_ephemeris_data(ephemeris)


# ══════════════════════════════════════════════
# Helpers — signal processing
# ══════════════════════════════════════════════

def make_if_pseudorange(pr1, pr2, f1_mhz, f2_mhz):
    """Ionosphere-free pseudorange combination (metres)."""
    f1 = f1_mhz * 1e6
    f2 = f2_mhz * 1e6
    return (pr1 * f1 ** 2 - pr2 * f2 ** 2) / (f1 ** 2 - f2 ** 2)


def parse_pseudorange(row):
    """
    Return (pseudorange_m, freq_label) for one RINEX row.
    Preference order: L1+L2 > L1+L5 > L1 only.
    Returns (None, None) if no usable pseudorange.
    """
    L1 = row['C1'] if pd.notna(row['C1']) else None
    L2 = row['C2'] if pd.notna(row['C2']) else None
    L5 = row['C5'] if pd.notna(row['C5']) else None

    if L1 is not None and L2 is not None:
        return make_if_pseudorange(L1, L2,
                                   gps_frequency_plan['L1'],
                                   gps_frequency_plan['L2']), 'L1_L2'
    if L1 is not None and L5 is not None:
        return make_if_pseudorange(L1, L5,
                                   gps_frequency_plan['L1'],
                                   gps_frequency_plan['L5']), 'L1_L5'
    if L1 is not None:
        return float(L1), 'L1'

    return None, None


# ══════════════════════════════════════════════
# Helpers — mapping functions & estimation
# ══════════════════════════════════════════════

def marini(elevation_deg, a, b, c_coeff):
    """Marini continued-fraction mapping function."""
    sin_el = np.sin(np.radians(elevation_deg))
    num = 1.0 + a / (1.0 + b / (1.0 + c_coeff))
    den = sin_el + a / (sin_el + b / (sin_el + c_coeff))
    return num / den


def estimate_clock_ztd_isb(sats, ah, bh, ch, elev_cutoff_deg, has_galileo):
    """
    Jointly estimate receiver clock (c*dt_r), ZTD, and optionally ISB
    from per-satellite raw residuals using weighted least squares.

    Observation model for each satellite i:
        raw_residual_i = c*dt_r + mf_h(el_i) * ZTD  [+ ISB if Galileo]

    Returns dict or None if insufficient observations.
    """
    above = [s for s in sats if s['elevation'] >= elev_cutoff_deg]
    n_obs = len(above)

    n_params = 3 if has_galileo else 2
    if n_obs < n_params + 1:
        return None

    y = np.zeros(n_obs)
    A = np.zeros((n_obs, n_params))
    W = np.zeros(n_obs)

    for i, s in enumerate(above):
        y[i] = s['raw_residual']
        mf_h = marini(s['elevation'], ah, bh, ch)

        A[i, 0] = 1.0      # receiver clock
        A[i, 1] = mf_h     # ZTD

        if has_galileo and s['pvn'].startswith('E'):
            A[i, 2] = 1.0  # ISB

        W[i] = np.sin(np.radians(s['elevation'])) ** 2

    Wd = np.diag(W)
    try:
        x = np.linalg.solve(A.T @ Wd @ A, A.T @ Wd @ y)
    except np.linalg.LinAlgError:
        return None

    return {
        'rcv_clock_m': x[0],
        'ztd_m':       x[1],
        'isb_m':       x[2] if has_galileo else 0.0,
        'n_obs':       n_obs,
        'n_params':    n_params,
    }


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


def zwd_to_pwv(zwd_m, tm_k):
    """Convert ZWD (metres) to PWV (mm)."""
    k2p = 22.1        # K/hPa
    k3  = 3.739e5     # K²/hPa
    rho = 1000.0      # kg/m³
    rv  = 461.51      # J/(kg·K)
    Pi  = 1e-6 * rho * rv * (k2p * 100.0 / tm_k + k3 * 100.0 / tm_k**2)
    return (zwd_m / Pi) * 1000.0   # → mm


def estimate_zwd_epoch(slant_delays, elevations, zhd_m):
    """
    Estimate ZWD for one epoch from per-satellite slant tropospheric delays
    using WLS with Saastamoinen ZHD as a priori constraint.

    Model:  T_slant_i = mf_h(el_i) * ZHD + mf_w(el_i) * ZWD

    ZHD is fixed from Saastamoinen; only ZWD is estimated.

    Returns dict with 'ZHD_m', 'ZWD_m', 'ZTD_m', 'ZTD_sigma_m', 'n_sats'
    or None if insufficient observations.
    """
    n = len(slant_delays)
    if n < 2:
        return None

    elev = np.asarray(elevations)
    std  = np.asarray(slant_delays)
    w    = np.sin(np.radians(elev)) ** 2
    W    = np.diag(w)

    mf_h = marini(elev, ah, bh, ch)
    mf_w = marini(elev, aw, bw, cw)

    # Remove the known hydrostatic contribution
    z_reduced = std - mf_h * zhd_m

    # Single-parameter WLS for ZWD
    H    = mf_w.reshape(-1, 1)
    HtWH = float((H.T @ W @ H)[0, 0])
    HtWz = float((H.T @ W @ z_reduced)[0])

    if abs(HtWH) < 1e-20:
        return None

    zwd = HtWz / HtWH

    # Formal uncertainty
    resid     = std - (mf_h * zhd_m + mf_w * zwd)
    sigma2    = np.sum(w * resid**2) / max(n - 1, 1)
    H_full    = np.column_stack([mf_h, mf_w])
    try:
        cov_x     = sigma2 * np.linalg.inv(H_full.T @ W @ H_full)
        ztd_sigma = float(np.sqrt(np.array([1., 1.]) @ cov_x @ np.array([1., 1.])))
    except np.linalg.LinAlgError:
        ztd_sigma = np.nan

    return {
        'ZHD_m':       zhd_m,
        'ZWD_m':       zwd,
        'ZTD_m':       zhd_m + zwd,
        'ZTD_sigma_m': ztd_sigma,
        'n_sats':      n,
    }


# ══════════════════════════════════════════════
# Load data & station parameters
# ══════════════════════════════════════════════

rinex = pd.read_csv(rinex_file)
total_rows = len(rinex)

station_lat, station_lon, station_hgt = ecef_to_geodetic(*receiver_ecef)
station_hgt_km = station_hgt / 1000.0
zhd_prior = saastamoinen_zhd(surface_pressure, station_lat, station_hgt_km)
tm = bevis_tm(surface_temperature + 273.15)

print(f"Station geodetic: lat={station_lat:.6f}°  lon={station_lon:.6f}°  h={station_hgt:.3f} m")
print(f"A priori ZHD (Saastamoinen): {zhd_prior:.4f} m")
print(f"Mean atmospheric temperature Tm: {tm:.1f} K")

# ══════════════════════════════════════════════
# Pass 1 — compute geometry for every satellite
# ══════════════════════════════════════════════

start_time = time.time()
print("\nPass 1: computing satellite geometry and raw residuals...")

epoch_buffer = defaultdict(list)

for idx in range(total_rows):
    progress = (idx + 1) / total_rows
    elapsed = time.time() - start_time
    eta_s = (elapsed / (idx + 1)) * (total_rows - idx - 1)
    eta_str = (f"{eta_s:.0f}s" if eta_s < 60 else
               f"{eta_s / 60:.1f}m" if eta_s < 3600 else
               f"{eta_s / 3600:.1f}h")
    bar = '█' * int(40 * progress) + '-' * (40 - int(40 * progress))
    sys.stdout.write(f'\r  |{bar}| {progress * 100:.1f}% ETA: {eta_str}')
    sys.stdout.flush()

    row = rinex.iloc[idx]

    pseudorange_m, freq_label = parse_pseudorange(row)
    if pseudorange_m is None:
        continue

    pvn = row['sat_id']
    rcvTime = datetime.strptime(row['epoch'], "%Y-%m-%d %H:%M:%S")
    rcvTow, gpsWeek = datetime_to_gps_tow(rcvTime)

    try:
        sat = load_ephemeris(ephemeris_data, pvn, gps_tow=rcvTow)
    except (KeyError, ValueError):
        continue

    try:
        gr = geometric_range.calculate_satellite_position_and_range(
            ephemeris_data, pvn, receiver_ecef,
            gps_week=gpsWeek, gps_tow=rcvTow
        )
    except Exception:
        continue

    geo_range_m = gr['geometric_range_m']
    t_tx = gr['transmission_time']
    sat_ecef = (gr['satellite_ecef_m']['x'],
                gr['satellite_ecef_m']['y'],
                gr['satellite_ecef_m']['z'])

    dt_sv = clock_correction.calculate_satellite_clock_offset(sat, t_tx)
    dt_rel = clock_correction.calculate_relativistic_clock_correction(sat, t_tx)

    raw_residual = pseudorange_m - geo_range_m + (dt_sv + dt_rel) * c

    elevation, azimuth = calculate_elevation_azimuth(sat_ecef, receiver_ecef)

    epoch_buffer[rcvTow].append({
        'pvn': pvn,
        'rcvTOW': rcvTow,
        'gpsWeek': gpsWeek,
        't_tx': t_tx,
        'geoRange': geo_range_m,
        'elevation': elevation,
        'azimuth': azimuth,
        'satClockBias': dt_sv * c,
        'relBias': dt_rel * c,
        'pseudorange': pseudorange_m,
        'freqLabel': freq_label,
        'raw_residual': raw_residual,
    })

sys.stdout.write('\n')

# ══════════════════════════════════════════════
# Pass 2 — per-epoch joint estimation of
#           receiver clock, ZTD, and ISB
# ══════════════════════════════════════════════

print("Pass 2: joint least-squares estimation of clock + ZTD [+ ISB]...")

gnss_results_lists = defaultdict(list)
epoch_summary_list = []  # ← collects one row per epoch for the summary table
epochs_total = len(epoch_buffer)
epochs_done = 0
skipped_epochs = 0

for rcvTow, sats in sorted(epoch_buffer.items()):
    epochs_done += 1
    sys.stdout.write(f'\r  Epoch {epochs_done}/{epochs_total}  '
                     f'({len(sats)} satellites)')
    sys.stdout.flush()

    has_galileo = any(s['pvn'].startswith('E') and s['elevation'] >= ELEV_CUTOFF_DEG
                      for s in sats)

    solution = estimate_clock_ztd_isb(sats, ah, bh, ch, ELEV_CUTOFF_DEG, has_galileo)

    if solution is None:
        skipped_epochs += 1
        continue

    rcv_clock_m = solution['rcv_clock_m']
    ztd_m       = solution['ztd_m']
    isb_m       = solution['isb_m']

    # ── Per-satellite slant tropospheric delay ────────────────
    above = [s for s in sats if s['elevation'] >= ELEV_CUTOFF_DEG]
    epoch_slant_delays = []
    epoch_elevations   = []
    epoch_svids        = []

    for s in above:
        clock_to_remove = rcv_clock_m
        if s['pvn'].startswith('E'):
            clock_to_remove += isb_m

        tropospheric_delay_m = s['raw_residual'] - clock_to_remove

        pwv, ztd = calculate_precipitable_water_vapor(
            receiver_ecef,
            s['elevation'],
            tropospheric_delay_m,
            surface_temperature,
            surface_pressure,
            coeffs
        )

        epoch_slant_delays.append(tropospheric_delay_m)
        epoch_elevations.append(s['elevation'])
        epoch_svids.append(s['pvn'])

        gnss_results_lists[s['pvn']].append({
            'svId': s['pvn'],
            'rcvTOW': s['rcvTOW'],
            't_tx': s['t_tx'],
            'WN': s['gpsWeek'],
            'geoRange': s['geoRange'],
            'elevation': s['elevation'],
            'azimuth': s['azimuth'],
            'satClockBias': s['satClockBias'],
            'relBias': s['relBias'],
            'rcvClockBias_m': rcv_clock_m,
            'ztd_est_m': ztd_m,
            'ISB_m': isb_m,
            'pseudorange': s['pseudorange'],
            'freqLabel': s['freqLabel'],
            'troposphericDelay': tropospheric_delay_m,
            'pwv': pwv * 1000.0,
            'ztd': ztd * 1000.0,
        })

    # ── Epoch-level ZWD / PWV estimate ────────────────────────
    zwd_result = estimate_zwd_epoch(epoch_slant_delays, epoch_elevations, zhd_prior)

    if zwd_result is not None:
        pwv_mm = zwd_to_pwv(zwd_result['ZWD_m'], tm)
        epoch_summary_list.append({
            'rcvTOW':       rcvTow,
            'ZHD_m':        zwd_result['ZHD_m'],
            'ZWD_m':        zwd_result['ZWD_m'],
            'ZTD_m':        zwd_result['ZTD_m'],
            'ZTD_sigma_m':  zwd_result['ZTD_sigma_m'],
            'PWV_mm':       pwv_mm,
            'rcvClockBias_m': rcv_clock_m,
            'ISB_m':        isb_m,
            'n_sats':       zwd_result['n_sats'],
            'sats_used':    ','.join(epoch_svids),
        })

sys.stdout.write('\n')
if skipped_epochs:
    print(f"  Skipped {skipped_epochs} epochs with insufficient satellites above {ELEV_CUTOFF_DEG}°")

# ══════════════════════════════════════════════
# Save to SQLite and print summary
# ══════════════════════════════════════════════
total_time = time.time() - start_time
print(f"\nCompleted in {total_time:.1f} seconds")

# Per-satellite tables
gnss_results = {}
for pvn in gnss_results_lists:
    gnss_results[pvn] = pd.DataFrame(gnss_results_lists[pvn])
    gnss_results[pvn].to_sql(pvn, conn, if_exists='replace', index=False)

print(f"Saved {len(gnss_results)} satellite tables to database.")

# Epoch summary table
epoch_summary = pd.DataFrame(epoch_summary_list)
epoch_summary.to_sql('epoch_summary', conn, if_exists='replace', index=False)
print(f"Saved epoch_summary table ({len(epoch_summary)} epochs) to database.")

# Print summary statistics
all_tropo = pd.concat(
    [df[['rcvTOW', 'elevation', 'troposphericDelay']]
     for df in gnss_results.values()],
    ignore_index=True
)
print(f"\nTropospheric delay summary (all satellites ≥{ELEV_CUTOFF_DEG}°):")
print(f"  mean  = {all_tropo['troposphericDelay'].mean():.3f} m")
print(f"  std   = {all_tropo['troposphericDelay'].std():.3f} m")
print(f"  min   = {all_tropo['troposphericDelay'].min():.3f} m")
print(f"  max   = {all_tropo['troposphericDelay'].max():.3f} m")
print(f"  (expected: ~2–12 m slant delay for 10–90° elevation)")

if len(epoch_summary) > 0:
    valid = epoch_summary.dropna(subset=['ZTD_m'])
    print(f"\nEpoch summary (Saastamoinen-constrained ZWD):")
    print(f"  Valid epochs: {len(valid)} / {len(epoch_summary)}")
    print(f"  ZHD (fixed):  {zhd_prior*1000:.1f} mm")
    print(f"  ZWD mean:     {valid['ZWD_m'].mean()*1000:.1f} mm")
    print(f"  ZWD std:      {valid['ZWD_m'].std()*1000:.1f} mm")
    print(f"  ZTD mean:     {valid['ZTD_m'].mean()*1000:.1f} mm")
    print(f"  ZTD range:    {valid['ZTD_m'].min()*1000:.1f} – {valid['ZTD_m'].max()*1000:.1f} mm")
    if valid['PWV_mm'].notna().any():
        print(f"  PWV mean:     {valid['PWV_mm'].mean():.2f} mm")
        print(f"  PWV std:      {valid['PWV_mm'].std():.2f} mm")
        print(f"  PWV range:    {valid['PWV_mm'].min():.2f} – {valid['PWV_mm'].max():.2f} mm")