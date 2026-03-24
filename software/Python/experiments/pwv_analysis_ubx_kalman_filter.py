from collections import defaultdict
import numpy as np
from pathlib import Path
import pandas as pd
import sqlite3
import sys
import time

from calculations.coordinates import ecef_to_geodetic
from calculations import clock_correction, geometric_range
from ephemerides.ephemeris import load_ephemeris, load_ephemeris_data
from calculations.elevation_azimuth import calculate_elevation_azimuth
from meteorology.tropospheric_products import (bevis_tm, calculate_pwv, saastamoinen_zhd,
                                               calculate_mapping_function, zwd_to_pwv)

c = 299792458.0  # Speed of light (m/s)

bdg_corr = False

# ─────────────────────────────────────────────
# File / database paths
# ─────────────────────────────────────────────
_DATA = Path(__file__).parent.parent / "data"

rawx_file   = _DATA / "ubx_data/2025-11-25/2025-11-25_93138_serial-COM3_RXM_RAWX.csv"
clock_file  = _DATA / "ubx_data/2025-11-25/2025-11-25_93138_serial-COM3_NAV_CLOCK.csv"
utctime_file = _DATA / "ubx_data/2025-11-25/2025-11-25_93138_serial-COM3_NAV_TIMEUTC.csv"
ephemeris   = _DATA / "ephemerides/ephemeris_2025-11-25_RINEX.json"
results_dir = _DATA / "ubx_data/2025-11-25/2025-11-25_serial-COM3_pwv_filter_gpsTOW_fixed"

results_dir.mkdir(parents=True, exist_ok=True)

conn = sqlite3.connect(_DATA / "ubx_data/2025-11-25/2025-11-25_serial-COM3_pwv_filter_gpsTOW_fixed_9.db")

# ─────────────────────────────────────────────
# Station / met parameters
# ─────────────────────────────────────────────
receiver_ecef = (867068.487, -5504812.066, 3092176.505)  # Campus quad
# receiver_ecef = (867068.487, -5504812.066, 3092176.505) # Apartment

surface_temperature_C   = 25.0              # °C
surface_pressure_hPa    = 1021.3348218666767  # hPa

# ─────────────────────────────────────────────
# VMF3 mapping coefficients
# ─────────────────────────────────────────────
ah, aw = 1.2451128509e-03, 6.0843905260e-04
bh, bw = 2.7074805444e-03, 1.4082272735e-03
ch, cw = 5.6279169098e-02, 4.0289703115e-02
mapping_coeffs = (ah, aw, bh, bw, ch, cw)

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
Q_ZTD_M2_PER_S   = 2.5e-8      # (m²/s) — (0.5 mm)² / s = 2.5e-7
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

    return pseudorange_m, carrier_phase, freq_label


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
# Kalman filter for clock + ZTD + ISB
# ══════════════════════════════════════════════

class KalmanFilterCZI:
    """
    Kalman filter for receiver clock, ZTD, and ISB.

    State:  x = [clock_m, ztd_m, isb_m]   (3 × 1)

    State transition is identity; process noise Q scales with dt.
    Clock gets enormous Q (re-estimated each epoch). ZTD gets small Q
    (constrained to evolve slowly). ISB gets tiny Q (nearly constant).

    Measurement model per satellite:
        y_i = [1, mf_h(el_i), δ_Galileo_i] · x + v_i

    Measurement noise R_ii = R_BASE / sin²(el_i).
    """

    def __init__(self):
        self.x = None
        self.P = None
        self.t_prev = None
        self.initialized = False

    def initialize(self, x0, P0, t0):
        """Set the initial state from a WLS solution."""
        self.x = np.array(x0, dtype=float)
        self.P = np.array(P0, dtype=float)
        self.t_prev = t0
        self.initialized = True

    def predict(self, t_now):
        """Time update: add process noise proportional to elapsed time."""
        dt = abs(t_now - self.t_prev) if self.t_prev is not None else 1.0
        dt = max(dt, 0.01)

        Q = np.diag([
            Q_CLOCK_M2_PER_S * dt,
            Q_ZTD_M2_PER_S   * dt,
            Q_ISB_M2_PER_S   * dt,
        ])

        self.P = self.P + Q
        self.t_prev = t_now

    def update(self, sats, elev_cutoff_deg):
        """Measurement update. Returns state dict or None."""
        above = [s for s in sats if s['elevation'] >= elev_cutoff_deg]
        n_obs = len(above)
        if n_obs < 2:
            return None

        y = np.zeros(n_obs)
        H = np.zeros((n_obs, 3))
        R = np.zeros((n_obs, n_obs))

        for i, s in enumerate(above):
            y[i] = s['raw_residual']
            mf_h = calculate_mapping_function(s['elevation'], ah, bh, ch)

            H[i, 0] = 1.0
            H[i, 1] = mf_h
            if s['pvn'].startswith('E'):
                H[i, 2] = 1.0

            # if 'prStdev' in s and s['prStdev'] > 0:
            #     R[i, i] = s['prStdev'] ** 2
            # else:
                # sin_el = np.sin(np.radians(s['elevation']))
                # R[i, i] = R_BASE_M2 / (sin_el ** 2)

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
    if n_obs < 4:
        return None, None

    y = np.zeros(n_obs)
    A = np.zeros((n_obs, 3))
    W = np.zeros(n_obs)

    for i, s in enumerate(above):
        y[i] = s['raw_residual']
        mf_h = calculate_mapping_function(s['elevation'], ah, bh, ch)
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

        v = y - A @ x0
        sigma2 = np.sum(W * v**2) / max(n_obs - 3, 1)
        P0 = sigma2 * np.linalg.inv(ATWA)
    except np.linalg.LinAlgError:
        return None, None

    return x0, P0


# ══════════════════════════════════════════════
# Helpers — geodetic & meteorological
# ══════════════════════════════════════════════
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

    mf_h = calculate_mapping_function(elev, ah, bh, ch)
    mf_w = calculate_mapping_function(elev, aw, bw, cw)

    z_reduced = std - mf_h * zhd_m

    H    = mf_w.reshape(-1, 1)
    HtWH = float((H.T @ W @ H)[0, 0])
    HtWz = float((H.T @ W @ z_reduced)[0])

    if abs(HtWH) < 1e-20:
        return None

    zwd = HtWz / HtWH

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
# Load data
# ══════════════════════════════════════════════
rawx  = pd.read_csv(rawx_file)
clock = pd.read_csv(clock_file)
utctime = pd.read_csv(utctime_file)
rawx_length = len(rawx)
total_measurements = len(rawx)

station_lat, station_lon, station_hgt = ecef_to_geodetic(receiver_ecef)
zhd_prior = saastamoinen_zhd(station_lat, station_hgt, surface_pressure_hPa)
tm = bevis_tm(surface_temperature_C + 273.15)

print(f"Station geodetic: lat={station_lat:.6f}°  lon={station_lon:.6f}°  h={station_hgt:.3f} m")
print(f"A priori ZHD (Saastamoinen): {zhd_prior:.4f} m")
print(f"Mean atmospheric temperature Tm: {tm:.1f} K")
print(f"Station atmospheric pressure: {surface_pressure_hPa:.2f} hPa")
print(f"\nKalman filter process noise:")
print(f"  Clock: {Q_CLOCK_M2_PER_S:.0e} m²/s (unconstrained)")
print(f"  ZTD:   {Q_ZTD_M2_PER_S:.1e} m²/s ({np.sqrt(Q_ZTD_M2_PER_S)*1e3:.2f} mm/√s)")
print(f"  ISB:   {Q_ISB_M2_PER_S:.1e} m²/s ({np.sqrt(Q_ISB_M2_PER_S)*1e3:.4f} mm/√s)")
print(f"  Measurement noise at zenith: {np.sqrt(R_BASE_M2):.2f} m")

# ══════════════════════════════════════════════
# Pass 1 — compute geometry and raw residuals
# ══════════════════════════════════════════════

print("\nPass 1: computing satellite geometry and raw residuals...")
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
    sys.stdout.write(f'\r  |{bar}| {progress * 100:.1f}% ({i + 1}/{total_measurements}) ETA: {eta_str}')
    sys.stdout.flush()

    numMeas = rawx.iloc[i]['numMeas']
    epoch_data = rawx.loc[i:(i + numMeas - 1), [
        'rcvTow', 'week', 'prMes', 'cpMes', 'doMes', 'gnssId', 'svId',
        'sigId', 'locktime', 'cno', 'prStdev', 'cpStdev',
        'prValid', 'cpValid', 'halfCyc', 'subHalfCyc']]

    rcvTow_s = epoch_data['rcvTow'].values[0]
    gpsWeek  = epoch_data['week'].values[0]

    rcvTow_ms = round(rcvTow_s) * 1e3
    utc_row = utctime[utctime['iTOW'] == rcvTow_ms]

    # if len(utc_row) > 0:
    #     iTow_ms = utc_row['iTOW'].values[0]
    #     nano = utc_row['nano'].values[0]
    #     gpsTow_s = iTow_ms * 1e-3 + nano * 1e-9

    # else:
    closest_iTow = clock.iloc[(clock['iTOW'] / 1000.0 - rcvTow_s).abs().argsort()[:1]]
    iTow_ms      = closest_iTow['iTOW'].values[0]
    clkBias_ns   = closest_iTow['clkB'].values[0]
    clkDrift_ns  = closest_iTow['clkD'].values[0]
    rcvTow_sat_s = correct_rcvtow(rcvTow_s, clkBias_ns)
    dt_iTow_s    = rcvTow_sat_s - (iTow_ms / 1000.0)
    dt_bias_s    = (clkBias_ns + clkDrift_ns * dt_iTow_s) / 1e9
    rcv_clkBias_nav_s = dt_iTow_s + dt_bias_s
    gpsTow_s     = rcvTow_sat_s - rcv_clkBias_nav_s

    gnssIDs = epoch_data['gnssId'].unique()

    for constellation in gnssIDs:
        epoch_constellation_data = epoch_data[epoch_data['gnssId'] == constellation]
        svIDs = epoch_constellation_data['svId'].unique()

        for satellite in svIDs:
            if constellation == 0:
                pvn = f'G{satellite:02d}'
            elif constellation == 2:
                pvn = f'E{satellite:02d}'
            else:
                continue

            satellite_data = epoch_constellation_data[epoch_constellation_data['svId'] == satellite]

            pseudorange_m, carrier_phase, freq_label = parse_ubx_signals(
                satellite_data, constellation, signal_plan)
            if pseudorange_m is None:
                continue

            try:
                sat = load_ephemeris(ephemeris_data, pvn, gps_tow=gpsTow_s)
            except KeyError:
                continue
            except ValueError:
                continue

            # ── broadcast group delay correction ─────────
            if bdg_corr:
                if pvn.startswith('G'):
                    tgd = sat.eph_data.get('TGD', 0.0)
                    if 'L5' in freq_label:
                        gamma = (1575.42 ** 2 - 1227.60 ** 2) / (1575.42 ** 2 - 1176.45 ** 2)
                        tgd_correction_s = -tgd * gamma
                    # L1/L2 IF: no correction needed
                    else:
                        tgd_correction_s = 0
                elif pvn.startswith('E'):
                    if 'E5_a' in freq_label or 'L5' in freq_label:
                        bgd = sat.eph_data.get('BGD_E1E5a', 0.0)
                        tgd_correction_s = -bgd
                    elif 'E5_b' in freq_label:
                        bgd = sat.eph_data.get('BGD_E1E5b', 0.0)
                        tgd_correction_s = -bgd
                    else:
                        tgd_correction_s = 0
                else:
                    tgd_correction_s = 0
            else:
                tgd_correction_s = 0

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

            raw_residual = pseudorange_m - geo_range_m + (dt_sv_s + dt_rel_s) * c

            elevation_deg, azimuth_deg = calculate_elevation_azimuth(sat_ecef, receiver_ecef)

            epoch_buffer[rcvTow_s].append({
                'pvn':            pvn,
                'rcvTOW':         rcvTow_s,
                'gpsTOW':         gpsTow_s,
                'iTOW':           iTow_ms / 1000.0,
                # 'rcvTOW_sat':     rcvTow_sat_s,
                # 'clkBias_ns':     clkBias_ns,
                # 'clkDrift_ns':    clkDrift_ns,
                'gpsWeek':        gpsWeek,
                't_tx':           t_tx_s,
                'geoRange':       geo_range_m,
                'elevation':      elevation_deg,
                'azimuth':        azimuth_deg,
                'satClockBias':   dt_sv_s * c,
                'relBias':        dt_rel_s * c,
                'pseudorange':    pseudorange_m,
                'prStdev':        float(satellite_data.iloc[0]['prStdev']),
                'carrierPhase':   carrier_phase,
                'freqLabel':      freq_label,
                'raw_residual':   raw_residual,
            })

    i += numMeas
    if i >= rawx_length:
        break

sys.stdout.write('\n')
pass1_time = time.time() - start_time
print(f"Pass 1 completed in {pass1_time:.1f} seconds")

# ══════════════════════════════════════════════
# Pass 2 — Kalman-filtered clock + ZTD + ISB
# ══════════════════════════════════════════════

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

        pwv, ztd = calculate_pwv(
            station_lat,
            station_hgt,
            s['elevation'],
            tropospheric_delay_m,
            surface_temperature_C,
            surface_pressure_hPa,
            mapping_coeffs
        )

        epoch_slant_delays.append(tropospheric_delay_m)
        epoch_elevations.append(s['elevation'])
        epoch_svids.append(s['pvn'])

        gnss_results_lists[s['pvn']].append({
            'svId':              s['pvn'],
            'iTOW':              s['iTOW'],
            'gpsTOW':            s['gpsTOW'],
            'rcvTOW':            s['rcvTOW'],
            # 'rcvTOW_sat':        s['rcvTOW_sat'],
            # 'clkBias_ns':        s['clkBias_ns'],
            # 'clkDrift_ns':       s['clkDrift_ns'],
            't_tx':              s['t_tx'],
            'WN':                s['gpsWeek'],
            'geoRange':          s['geoRange'],
            'elevation':         s['elevation'],
            'azimuth':           s['azimuth'],
            'satClockBias':      s['satClockBias'],
            'relBias':           s['relBias'],
            'rcvClockBias_m':    rcv_clock_m,
            'ztd_est_m':         ztd_m,
            'ISB_m':             isb_m,
            'pseudorange':       s['pseudorange'],
            'carrierPhase':      s['carrierPhase'],
            'freqLabel':         s['freqLabel'],
            'raw_residual':      s['raw_residual'],
            'troposphericDelay': tropospheric_delay_m,
            'pwv':               pwv * 1000.0,
            'ztd':               ztd * 1000.0,
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

gnss_results = {}
for pvn in gnss_results_lists:
    gnss_results[pvn] = pd.DataFrame(gnss_results_lists[pvn])
    gnss_results[pvn].to_sql(pvn, conn, if_exists='replace', index=False)

print(f"Saved {len(gnss_results)} satellite tables to database.")

epoch_summary = pd.DataFrame(epoch_summary_list)
epoch_summary.to_sql('epoch_summary', conn, if_exists='replace', index=False)
print(f"Saved epoch_summary table ({len(epoch_summary)} epochs) to database.")

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