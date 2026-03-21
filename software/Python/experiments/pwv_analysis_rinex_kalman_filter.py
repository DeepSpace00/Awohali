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
from tropospheric_products.precipitable_water_vapor import calculate_precipitable_water_vapor

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

rinex_file   = _DATA / "RINEX_data/ORMD/2025-11-25/ormd3290_excerpt.csv"
ephemeris    = _DATA / "ephemerides/ephemeris_2025-11-25_RINEX.json"
results_dir  = _DATA / "RINEX_data/ORMD/2025-11-25/RINEX_pwv_test_new_filter"

# rinex_file   = _DATA / "RINEX_data/ORMD/2025-09-30/ormd2730_GPS_Galileo.csv"
# ephemeris    = _DATA / "ephemerides/ephemeris_2025-09-30.json"
# results_dir  = _DATA / "RINEX_data/ORMD/2025-09-30/ormd2730_GPS_Galileo"

results_dir.mkdir(parents=True, exist_ok=True)

conn = sqlite3.connect(_DATA / "RINEX_data/ORMD/2025-11-25/RINEX_pwv_test_new_filter.db")

# conn = sqlite3.connect(_DATA / "RINEX_data/ORMD/2025-09-30/ormd2730.db")

# ─────────────────────────────────────────────
# Station / met parameters
# ─────────────────────────────────────────────
receiver_ecef       = (860376.4154, -5499833.4036, 3102756.9385)  # CORS ORMD
surface_temperature = 25.0                   # °C
surface_pressure    = 846.597166666675       # hPa

# ─────────────────────────────────────────────
# VMF3 mapping coefficients
# ─────────────────────────────────────────────
ah, aw = 1.2451128509e-03, 6.0843905260e-04
bh, bw = 2.7074805444e-03, 1.4082272735e-03
ch, cw = 5.6279169098e-02, 4.0289703115e-02
coeffs = (ah, aw, bh, bw, ch, cw)

ELEV_CUTOFF_DEG = 20.0   # degrees — applied AFTER receiver clock estimation

ephemeris_data = load_ephemeris_data(ephemeris)

# ─────────────────────────────────────────────
# Kalman filter tuning parameters
# ─────────────────────────────────────────────
# Process noise standard deviations (per second of elapsed time):
#   - Clock:  effectively unconstrained (receiver clock can jump freely)
#   - ZTD:    ~0.5 mm/√s — ZTD changes physically at ~0.1-1 mm/min
#   - ISB:    ~0.01 mm/√s — hardware bias is very stable over a session
#
# Measurement noise: base sigma² scaled by 1/sin²(el) so low-elevation
# satellites get higher noise.  R_BASE_M2 is the variance at zenith.

Q_CLOCK_M2_PER_S = 1e12        # (m²/s) — essentially white noise each epoch
Q_ZTD_M2_PER_S   = 2.5e-7      # (m²/s) — (0.5 mm)² / s = 2.5e-7
Q_ISB_M2_PER_S   = 1e-10       # (m²/s) — (0.01 mm)² / s = 1e-10
R_BASE_M2         = 1.0         # (m²)   — measurement noise variance at zenith


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
# Helpers — mapping functions
# ══════════════════════════════════════════════

def marini(elevation_deg, a, b, c_coeff):
    """Marini continued-fraction mapping function."""
    sin_el = np.sin(np.radians(elevation_deg))
    num = 1.0 + a / (1.0 + b / (1.0 + c_coeff))
    den = sin_el + a / (sin_el + b / (sin_el + c_coeff))
    return num / den


# ══════════════════════════════════════════════
# Kalman filter for clock + ZTD + ISB
# ══════════════════════════════════════════════
#
# State vector (always 3 elements):
#   x = [c·dt_r,  ZTD,  ISB]
#
# State transition: identity (states persist, noise added via Q).
#
# Process noise Q: diagonal, scaled by dt between epochs.
#   - Clock gets enormous variance → effectively re-estimated each epoch.
#   - ZTD gets small variance → constrained to evolve slowly.
#   - ISB gets tiny variance → nearly constant across the session.
#
# Observation model at each epoch (n satellites above cutoff):
#   y_i = [1,  mf_h(el_i),  δ_Galileo_i] · x + v_i
#
# Measurement noise R: diagonal, R_ii = R_BASE / sin²(el_i).
#
# The filter is initialized from the first epoch's WLS solution.
# If the first epoch has insufficient satellites, initialization is
# deferred until a valid WLS solution is available.

class KalmanFilterCZI:
    """
    Kalman filter for receiver clock, ZTD, and ISB.

    State:  x = [clock_m, ztd_m, isb_m]   (3 × 1)
    """

    def __init__(self):
        self.x = None           # state vector (3,)
        self.P = None           # state covariance (3, 3)
        self.t_prev = None      # previous epoch time (s)
        self.initialized = False

    def initialize(self, x0, P0, t0):
        """Set the initial state from a WLS solution."""
        self.x = np.array(x0, dtype=float)
        self.P = np.array(P0, dtype=float)
        self.t_prev = t0
        self.initialized = True

    def predict(self, t_now):
        """
        Time update (prediction step).
        State transition is identity; process noise Q is added
        proportionally to elapsed time.
        """
        dt = abs(t_now - self.t_prev) if self.t_prev is not None else 1.0
        dt = max(dt, 0.01)  # guard against zero dt

        Q = np.diag([
            Q_CLOCK_M2_PER_S * dt,
            Q_ZTD_M2_PER_S   * dt,
            Q_ISB_M2_PER_S   * dt,
        ])

        # x_predicted = F · x  (F = I, so no change)
        # P_predicted = F · P · F' + Q
        self.P = self.P + Q
        self.t_prev = t_now

    def update(self, sats, elev_cutoff_deg):
        """
        Measurement update using satellites above the elevation cutoff.

        Returns the updated state dict or None if no valid measurements.
        """
        above = [s for s in sats if s['elevation'] >= elev_cutoff_deg]
        n_obs = len(above)
        if n_obs < 2:
            return None

        # Build observation vector and design matrix
        y = np.zeros(n_obs)
        H = np.zeros((n_obs, 3))
        R = np.zeros((n_obs, n_obs))

        for i, s in enumerate(above):
            y[i] = s['raw_residual']
            mf_h = marini(s['elevation'], ah, bh, ch)

            H[i, 0] = 1.0      # clock
            H[i, 1] = mf_h     # ZTD
            if s['pvn'].startswith('E'):
                H[i, 2] = 1.0  # ISB

            # Elevation-dependent measurement noise
            sin_el = np.sin(np.radians(s['elevation']))
            R[i, i] = R_BASE_M2 / (sin_el ** 2)

        # Innovation
        z = y - H @ self.x

        # Innovation covariance
        S = H @ self.P @ H.T + R

        # Kalman gain
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return None

        # State update
        self.x = self.x + K @ z

        # Covariance update (Joseph form for numerical stability)
        I_KH = np.eye(3) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

        # Ensure symmetry
        self.P = 0.5 * (self.P + self.P.T)

        return {
            'rcv_clock_m': self.x[0],
            'ztd_m':       self.x[1],
            'isb_m':       self.x[2],
            'n_obs':       n_obs,
        }


def wls_initial_solution(sats, elev_cutoff_deg):
    """
    Compute a WLS solution for Kalman filter initialization.
    Returns (x0, P0) or (None, None) if insufficient satellites.
    """
    above = [s for s in sats if s['elevation'] >= elev_cutoff_deg]
    n_obs = len(above)
    if n_obs < 4:  # need at least 4 for a 3-parameter fit
        return None, None

    y = np.zeros(n_obs)
    A = np.zeros((n_obs, 3))
    W = np.zeros(n_obs)

    for i, s in enumerate(above):
        y[i] = s['raw_residual']
        mf_h = marini(s['elevation'], ah, bh, ch)
        A[i, 0] = 1.0
        A[i, 1] = mf_h
        if s['pvn'].startswith('E'):
            A[i, 2] = 1.0
        W[i] = np.sin(np.radians(s['elevation'])) ** 2

    Wd = np.diag(W)
    try:
        ATWA = A.T @ Wd @ A
        ATWy = A.T @ Wd @ y
        x0   = np.linalg.solve(ATWA, ATWy)

        # Initial covariance from WLS: P0 = sigma² · (A'WA)^{-1}
        v = y - A @ x0
        sigma2 = np.sum(W * v**2) / max(n_obs - 3, 1)
        P0 = sigma2 * np.linalg.inv(ATWA)
    except np.linalg.LinAlgError:
        return None, None

    return x0, P0


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

    Returns dict or None if insufficient observations.
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
print(f"\nKalman filter process noise:")
print(f"  Clock: {Q_CLOCK_M2_PER_S:.0e} m²/s (unconstrained)")
print(f"  ZTD:   {Q_ZTD_M2_PER_S:.1e} m²/s ({np.sqrt(Q_ZTD_M2_PER_S)*1e3:.2f} mm/√s)")
print(f"  ISB:   {Q_ISB_M2_PER_S:.1e} m²/s ({np.sqrt(Q_ISB_M2_PER_S)*1e3:.4f} mm/√s)")
print(f"  Measurement noise at zenith: {np.sqrt(R_BASE_M2):.2f} m")

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
# Pass 2 — Kalman-filtered clock + ZTD + ISB
# ══════════════════════════════════════════════
# Instead of solving each epoch independently, the Kalman filter
# carries state forward:
#   - Clock: huge process noise → effectively re-estimated each epoch
#   - ZTD:   small process noise → constrained to evolve slowly
#   - ISB:   tiny process noise  → nearly constant
#
# When a satellite appears or disappears, the measurement geometry
# changes, but the ZTD state barely moves because the filter trusts
# its prediction (from the previous epoch) more than the single-epoch
# measurement perturbation.
#
# Initialization: the first epoch with ≥4 satellites is solved via
# standard WLS to seed the filter state and covariance.

print("Pass 2: Kalman-filtered estimation of clock + ZTD + ISB...")

kf = KalmanFilterCZI()
gnss_results_lists = defaultdict(list)
epoch_summary_list = []
epochs_total = len(epoch_buffer)
epochs_done  = 0
skipped_epochs = 0

for rcvTow, sats in sorted(epoch_buffer.items()):
    epochs_done += 1
    sys.stdout.write(f'\r  Epoch {epochs_done}/{epochs_total}  '
                     f'({len(sats)} satellites)')
    sys.stdout.flush()

    # ── Initialize filter from first valid WLS solution ──────
    if not kf.initialized:
        x0, P0 = wls_initial_solution(sats, ELEV_CUTOFF_DEG)
        if x0 is None:
            skipped_epochs += 1
            continue
        kf.initialize(x0, P0, rcvTow)
        # Fall through to process this epoch normally

    # ── Kalman predict ───────────────────────────────────────
    kf.predict(rcvTow)

    # ── Kalman update ────────────────────────────────────────
    solution = kf.update(sats, ELEV_CUTOFF_DEG)

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
            'raw_residual': s['raw_residual'],
            'troposphericDelay': tropospheric_delay_m,
            'pwv': pwv * 1000.0,
            'ztd': ztd * 1000.0,
        })

    # ── Epoch-level ZWD / PWV estimate ────────────────────────
    zwd_result = estimate_zwd_epoch(epoch_slant_delays, epoch_elevations, zhd_prior)

    if zwd_result is not None:
        pwv_mm = zwd_to_pwv(zwd_result['ZWD_m'], tm)
        epoch_summary_list.append({
            'rcvTOW':         rcvTow,
            'ZHD_m':          zwd_result['ZHD_m'],
            'ZWD_m':          zwd_result['ZWD_m'],
            'ZTD_m':          zwd_result['ZTD_m'],
            'ZTD_sigma_m':    zwd_result['ZTD_sigma_m'],
            'PWV_mm':         pwv_mm,
            'rcvClockBias_m': rcv_clock_m,
            'ISB_m':          isb_m,
            'n_sats':         zwd_result['n_sats'],
            'sats_used':      ','.join(epoch_svids),
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