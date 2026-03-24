from collections import defaultdict
import numpy as np
from pathlib import Path
import pandas as pd
import sqlite3
import sys
import time

from calculations import clock_correction, geometric_range, coordinates
from ephemerides.ephemeris import load_ephemeris, load_ephemeris_data
from calculations.elevation_azimuth import calculate_elevation_azimuth
from meteorology.tropospheric_products import calculate_mapping_function
from meteorology.tropospheric_products import saastamoinen_zhd, bevis_tm, zwd_to_pwv

c = 299792458.0  # Speed of light (m/s)

# ─────────────────────────────────────────────
# File / database paths
# ─────────────────────────────────────────────
_DATA = Path(__file__).parent.parent / "data"

rawx_file   = _DATA / "ubx_data/2025-11-25/2025-11-25_93138_serial-COM3_RXM_RAWX.csv"
clock_file  = _DATA / "ubx_data/2025-11-25/2025-11-25_93138_serial-COM3_NAV_CLOCK.csv"
ephemeris   = _DATA / "ephemerides/ephemeris_2025-11-25_RINEX.json"
results_dir = _DATA / "ubx_data/2025-11-25/2025-11-25_serial-COM3_pwv_single_difference_3"

results_dir.mkdir(parents=True, exist_ok=True)

conn = sqlite3.connect(_DATA / "ubx_data/2025-09-28/COM3_pwv_single_difference_3.db")

# ─────────────────────────────────────────────
# Station / met parameters
# ─────────────────────────────────────────────
receiver_ecef = (867068.487, -5504812.066, 3092176.505)  # Campus quad
# receiver_ecef = (867068.487, -5504812.066, 3092176.505) # Apartment

surface_temperature_C   = 25.0           # °C
surface_pressure_hPa    = 1013.0         # hPa

# ─────────────────────────────────────────────
# VMF3 mapping coefficients
# ─────────────────────────────────────────────
ah, aw = 1.2451128509e-03, 6.0843905260e-04
bh, bw = 2.7074805444e-03, 1.4082272735e-03
ch, cw = 5.6279169098e-02, 4.0289703115e-02
mapping_coeffs = (ah, aw, bh, bw, ch, cw)

ELEV_CUTOFF_DEG = 20.0  # degrees — applied AFTER geometry computation
GPS_ONLY        = True # True = process GPS only, False = GPS + Galileo

ephemeris_data = load_ephemeris_data(ephemeris)

# ─────────────────────────────────────────────
# Kalman filter tuning (ZTD only — no clock state)
# ─────────────────────────────────────────────
Q_ZTD_M2_PER_S = 2.5e-7    # (m²/s) — ZTD process noise
R_BASE_M2      = 2.0        # (m²)   — SD measurement noise at zenith (√2 higher than undifferenced)

# ─────────────────────────────────────────────
# UBX signal plan (gnssId → sigId → service/freq)
# ─────────────────────────────────────────────
signal_plan = {
    0: {
        0:  {'service': 'L1_C/A',   'freq': 1575.42},
        3:  {'service': 'L2_CL',    'freq': 1227.60},
        4:  {'service': 'L2_CM',    'freq': 1227.60},
        6:  {'service': 'L5_I',     'freq': 1176.45},
        7:  {'service': 'L5_Q',     'freq': 1176.45}
    },
    2: {
        0:  {'service': 'E1_C',     'freq': 1575.42},
        1:  {'service': 'E1_B',     'freq': 1575.42},
        3:  {'service': 'E5_aI',    'freq': 1176.45},
        4:  {'service': 'E5_aQ',    'freq': 1176.45},
        5:  {'service': 'E5_bI',    'freq': 1207.14},
        6:  {'service': 'E5_bQ',    'freq': 1207.14},
        8:  {'service': 'E6_B',     'freq': 1278.75},
        9:  {'service': 'E6_C',     'freq': 1278.75},
        10: {'service': 'E6_A',     'freq': 1278.75}
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
# Helpers — mapping functions
# ══════════════════════════════════════════════

# def marini(elevation_deg, a, b, c_coeff):
#     """Marini continued-fraction mapping function."""
#     sin_el = np.sin(np.radians(elevation_deg))
#     num = 1.0 + a / (1.0 + b / (1.0 + c_coeff))
#     den = sin_el + a / (sin_el + b / (sin_el + c_coeff))
#     return num / den


# ══════════════════════════════════════════════
# Single-difference ZTD estimation
# ══════════════════════════════════════════════
#
# For satellites i and j at the same epoch:
#   raw_resid_i - raw_resid_j = (mf_h(el_i) - mf_h(el_j)) * ZTD
#
# The receiver clock c*dt_r cancels exactly.
# ISB cancels if both satellites are from the same constellation.
#
# Reference satellite selection: pick the GPS satellite with the
# highest elevation (best pseudorange quality, smallest mapping
# function error). All other GPS satellites are differenced against it.
# Galileo satellites are differenced against the highest-elevation
# Galileo satellite separately (avoids ISB contamination).

def select_reference_satellite(sats, constellation_prefix):
    """
    Select the highest-elevation satellite from a given constellation
    as the single-difference reference.
    """
    candidates = [s for s in sats if s['pvn'].startswith(constellation_prefix)]
    if not candidates:
        return None
    return max(candidates, key=lambda s: s['elevation'])


def estimate_ztd_single_difference(sats, elev_cutoff_deg):
    """
    Estimate ZTD from single-differenced pseudoranges.

    For satellites i and reference j (same constellation):
        (P_i - P_j) = (rho_i - rho_j)
                     + c*(dt_sv_j - dt_sv_i + dt_rel_j - dt_rel_i)
                     + (mf_h(el_i) - mf_h(el_j)) * ZTD

    The receiver clock c*dt_r cancels exactly in P_i - P_j.
    """
    above = [s for s in sats if s['elevation'] >= elev_cutoff_deg]

    ref_gps = select_reference_satellite(above, 'G')
    ref_gal = select_reference_satellite(above, 'E')

    sd_obs = []

    if ref_gps is not None:
        mf_ref = calculate_mapping_function(ref_gps['elevation'], ah, bh, ch)
        w_ref  = np.sin(np.radians(ref_gps['elevation'])) ** 2

        for s in above:
            if s['pvn'].startswith('G') and s['pvn'] != ref_gps['pvn']:
                mf_i = calculate_mapping_function(s['elevation'], ah, bh, ch)

                # Single-differenced pseudorange
                delta_P   = s['pseudorange'] - ref_gps['pseudorange']

                # Single-differenced geometric range
                delta_rho = s['geoRange'] - ref_gps['geoRange']

                # Single-differenced satellite clock + relativistic correction
                # Note sign: we subtract ref from sat_i in P, so we add ref clock minus sat_i clock
                delta_clk = (ref_gps['satClockBias'] + ref_gps['relBias']) \
                          - (s['satClockBias'] + s['relBias'])

                # Corrected SD observation = delta_T = (mf_i - mf_ref) * ZTD
                delta_obs = delta_P - delta_rho - delta_clk

                # Mapping function difference
                delta_mf = mf_i - mf_ref

                # Weight (SD noise = sum of both variances)
                w_i = np.sin(np.radians(s['elevation'])) ** 2
                sd_var = R_BASE_M2 / w_i + R_BASE_M2 / w_ref
                sd_obs.append((delta_mf, delta_obs, 1.0 / sd_var, s['pvn']))

    if ref_gal is not None:
        mf_ref = calculate_mapping_function(ref_gal['elevation'], ah, bh, ch)
        w_ref  = np.sin(np.radians(ref_gal['elevation'])) ** 2

        for s in above:
            if s['pvn'].startswith('E') and s['pvn'] != ref_gal['pvn']:
                mf_i = calculate_mapping_function(s['elevation'], ah, bh, ch)

                delta_P   = s['pseudorange'] - ref_gal['pseudorange']
                delta_rho = s['geoRange'] - ref_gal['geoRange']
                delta_clk = (ref_gal['satClockBias'] + ref_gal['relBias']) \
                          - (s['satClockBias'] + s['relBias'])

                delta_obs = delta_P - delta_rho - delta_clk
                delta_mf  = mf_i - mf_ref

                w_i = np.sin(np.radians(s['elevation'])) ** 2
                sd_var = R_BASE_M2 / w_i + R_BASE_M2 / w_ref
                sd_obs.append((delta_mf, delta_obs, 1.0 / sd_var, s['pvn']))

    n_sd = len(sd_obs)
    if n_sd < 2:
        return None

    # WLS for ZTD: delta_obs = delta_mf * ZTD
    y = np.array([o[1] for o in sd_obs])
    H = np.array([o[0] for o in sd_obs]).reshape(-1, 1)
    W = np.diag([o[2] for o in sd_obs])

    HtWH = float((H.T @ W @ H)[0, 0])
    HtWy = float((H.T @ W @ y)[0])

    if abs(HtWH) < 1e-20:
        return None

    ztd = HtWy / HtWH

    v = y - H.ravel() * ztd
    sigma2 = np.sum(np.diag(W) * v**2) / max(n_sd - 1, 1)
    ztd_sigma = np.sqrt(sigma2 / HtWH) if HtWH > 0 else np.nan

    return {
        'ztd_m':      ztd,
        'ztd_sigma':  ztd_sigma,
        'ref_gps':    ref_gps['pvn'] if ref_gps else None,
        'ref_gal':    ref_gal['pvn'] if ref_gal else None,
        'n_sd_obs':   n_sd,
    }


# ══════════════════════════════════════════════
# Kalman filter for ZTD only (no clock state)
# ══════════════════════════════════════════════

class KalmanFilterZTD:
    """
    Scalar Kalman filter for ZTD estimated from single differences.

    State: x = ZTD (scalar, meters)
    Since the receiver clock is eliminated by differencing,
    the only state to track is ZTD.
    """

    def __init__(self):
        self.x = None       # ZTD estimate (m)
        self.P = None       # ZTD variance (m²)
        self.t_prev = None
        self.initialized = False

    def initialize(self, x0, P0, t0):
        self.x = float(x0)
        self.P = float(P0)
        self.t_prev = t0
        self.initialized = True

    def predict(self, t_now):
        dt = abs(t_now - self.t_prev) if self.t_prev is not None else 1.0
        dt = max(dt, 0.01)
        self.P += Q_ZTD_M2_PER_S * dt
        self.t_prev = t_now

    def update(self, ztd_meas, ztd_var):
        """
        Scalar Kalman update with a ZTD measurement and its variance.

        Parameters
        ----------
        ztd_meas : float
            ZTD from single-difference WLS (m)
        ztd_var : float
            Variance of the ZTD measurement (m²)
        """
        if ztd_var <= 0 or np.isnan(ztd_var):
            return

        # Innovation
        z = ztd_meas - self.x

        # Innovation variance
        S = self.P + ztd_var

        # Kalman gain
        K = self.P / S

        # Update
        self.x = self.x + K * z
        self.P = (1.0 - K) * self.P

    def state(self):
        return self.x, self.P


# ══════════════════════════════════════════════
# Helpers — geodetic & meteorological
# ══════════════════════════════════════════════

# def ecef_to_geodetic(x, y, z):
#     """Bowring iterative ECEF → geodetic (WGS-84). Returns (lat_deg, lon_deg, height_m)."""
#     a  = 6378137.0
#     f  = 1.0 / 298.257223563
#     b  = a * (1.0 - f)
#     e2 = 2*f - f**2

#     p   = np.sqrt(x**2 + y**2)
#     lon = np.arctan2(y, x)

#     lat = np.arctan2(z, p * (1.0 - e2))
#     for _ in range(10):
#         sin_lat = np.sin(lat)
#         N       = a / np.sqrt(1.0 - e2 * sin_lat**2)
#         lat_new = np.arctan2(z + e2 * N * sin_lat, p)
#         if abs(lat_new - lat) < 1e-12:
#             break
#         lat = lat_new

#     sin_lat = np.sin(lat)
#     N = a / np.sqrt(1.0 - e2 * sin_lat**2)
#     h = p / np.cos(lat) - N if abs(lat) < np.radians(45) else \
#         z / np.sin(lat) - N * (1.0 - e2)

#     return np.degrees(lat), np.degrees(lon), h


# def saastamoinen_zhd(pressure_hpa, lat_deg, height_km):
#     """Saastamoinen (1972) / Davis et al. (1985) ZHD in metres."""
#     return (0.0022768 * pressure_hpa) / (
#         1.0 - 0.00266 * np.cos(2.0 * np.radians(lat_deg))
#             - 0.00028 * height_km
#     )


# def bevis_tm(surface_temp_k):
#     """Bevis (1992) mean atmospheric temperature from surface temp."""
#     return 70.2 + 0.72 * surface_temp_k


# def zwd_to_pwv(zwd_m, tm_k):
#     """Convert ZWD (metres) to PWV (mm)."""
#     k2p = 22.1        # K/hPa
#     k3  = 3.739e5     # K²/hPa
#     rho = 1000.0      # kg/m³
#     rv  = 461.51      # J/(kg·K)
#     Pi  = 1e-6 * (k2p + k3 / tm_k) * rv * rho / 100.0
#     return (zwd_m / Pi) * 1000.0   # → mm


# def estimate_zwd_from_ztd(ztd_m, zhd_m):
#     """Decompose ZTD into ZHD (fixed) + ZWD (residual)."""
#     return ztd_m - zhd_m


# ══════════════════════════════════════════════
# Load data
# ══════════════════════════════════════════════
rawx  = pd.read_csv(rawx_file)
clock = pd.read_csv(clock_file)
rawx_length = len(rawx)
total_measurements = len(rawx)

station_lat, station_lon, station_hgt = coordinates.ecef_to_geodetic(receiver_ecef)
zhd_prior = saastamoinen_zhd(station_lat, station_hgt, surface_pressure_hPa)
tm = bevis_tm(surface_temperature_C + 273.15)

print(f"Station geodetic: lat={station_lat:.6f}°  lon={station_lon:.6f}°  h={station_hgt:.3f} m")
print(f"A priori ZHD (Saastamoinen): {zhd_prior:.4f} m")
print(f"Mean atmospheric temperature Tm: {tm:.1f} K")
print(f"Station atmospheric pressure: {surface_pressure_hPa:.2f} hPa")
print(f"Constellation mode: {'GPS only' if GPS_ONLY else 'GPS + Galileo'}")
print(f"\nSingle-difference Kalman filter:")
print(f"  ZTD process noise: {Q_ZTD_M2_PER_S:.1e} m²/s ({np.sqrt(Q_ZTD_M2_PER_S)*1e3:.2f} mm/√s)")
print(f"  SD measurement noise at zenith: {np.sqrt(R_BASE_M2):.2f} m")

# ══════════════════════════════════════════════
# Pass 1 — compute geometry and raw residuals
# ══════════════════════════════════════════════
# Same as before: for each satellite, compute
#   raw_residual = P_IF - rho + c*(dt_sv + dt_rel) = c*dt_r + T_slant
#
# The receiver clock is still present in each raw_residual.
# It will be eliminated in Pass 2 via single differencing.

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
                if GPS_ONLY:
                    continue
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
                'pseudorange':    pseudorange_m,
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
# Pass 2 — single-difference ZTD estimation
#           with Kalman filter smoothing
# ══════════════════════════════════════════════
# For each epoch:
#   1. Select highest-elevation GPS sat as reference (separately for GAL)
#   2. Difference all other same-constellation sats against the reference
#   3. Fit ZTD from the differenced observations (clock cancels exactly)
#   4. Feed the SD-derived ZTD into a scalar Kalman filter for smoothing
#   5. Recover per-satellite slant delays using filtered ZTD
#
# Advantages over the clock+ZTD joint estimation:
#   - Receiver clock eliminated exactly, not estimated
#   - ISB eliminated by differencing within each constellation
#   - Satellite DCBs cancel if same-constellation sats use same signal combo
#   - No numerical conditioning issues from million-meter clock values

print("Pass 2: single-difference ZTD estimation with Kalman filter...")

kf = KalmanFilterZTD()
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

    # ── Single-difference ZTD estimate ────────────────────────
    sd_result = estimate_ztd_single_difference(sats, ELEV_CUTOFF_DEG)

    if sd_result is None:
        skipped_epochs += 1
        continue

    ztd_sd = sd_result['ztd_m']
    ztd_sd_var = sd_result['ztd_sigma'] ** 2 if not np.isnan(sd_result['ztd_sigma']) else R_BASE_M2

    # ── Kalman filter ─────────────────────────────────────────
    if not kf.initialized:
        # Initialize from first valid SD solution
        kf.initialize(ztd_sd, ztd_sd_var, rcvTow)
    else:
        kf.predict(rcvTow)
        kf.update(ztd_sd, ztd_sd_var)

    ztd_filtered, ztd_P = kf.state()

    # ── Decompose ZTD into ZHD + ZWD ─────────────────────────
    zwd_m = ztd_filtered - zhd_prior
    pwv_mm = zwd_to_pwv(zwd_m, tm)

    # ── Recover per-satellite slant delays ────────────────────
    # T_slant_i = mf_h(el_i) * ZTD_filtered
    above = [s for s in sats if s['elevation'] >= ELEV_CUTOFF_DEG]
    epoch_svids = []

    for s in above:
        mf_h_i = calculate_mapping_function(s['elevation'], ah, bh, ch)
        tropospheric_delay_m = mf_h_i * ztd_filtered

        # Per-satellite PWV from the modeled slant delay
        mf_w_i = calculate_mapping_function(s['elevation'], aw, bw, cw)
        slant_zwd = mf_w_i * zwd_m
        sat_pwv_mm = zwd_to_pwv(zwd_m, tm)

        epoch_svids.append(s['pvn'])

        gnss_results_lists[s['pvn']].append({
            'svId':              s['pvn'],
            'iTOW':              s['iTOW'],
            'gpsTOW':            s['gpsTOW'],
            'rcvTOW':            s['rcvTOW'],
            'rcvTOW_sat':        s['rcvTOW_sat'],
            'clkBias_ns':        s['clkBias_ns'],
            'clkDrift_ns':       s['clkDrift_ns'],
            't_tx':              s['t_tx'],
            'WN':                s['gpsWeek'],
            'geoRange':          s['geoRange'],
            'elevation':         s['elevation'],
            'azimuth':           s['azimuth'],
            'satClockBias':      s['satClockBias'],
            'relBias':           s['relBias'],
            'ztd_est_m':         ztd_filtered,
            'pseudorange':       s['pseudorange'],
            'carrierPhase':      s['carrierPhase'],
            'freqLabel':         s['freqLabel'],
            'raw_residual':      s['raw_residual'],
            'troposphericDelay': tropospheric_delay_m,
            'pwv':               sat_pwv_mm,
            'ztd':               ztd_filtered * 1000.0,
        })

    # ── Epoch summary ─────────────────────────────────────────
    epoch_summary_list.append({
        'rcvTOW':         rcvTow,
        'ZHD_m':          zhd_prior,
        'ZWD_m':          zwd_m,
        'ZTD_m':          ztd_filtered,
        'ZTD_sd_m':       ztd_sd,
        'ZTD_sigma_m':    np.sqrt(ztd_P),
        'PWV_mm':         pwv_mm,
        'ref_gps':        sd_result['ref_gps'] or '',
        'ref_gal':        sd_result['ref_gal'] or '',
        'n_sd_obs':       sd_result['n_sd_obs'],
        'n_sats':         len(above),
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
if len(epoch_summary) > 0:
    valid = epoch_summary.dropna(subset=['ZTD_m'])
    print(f"\nEpoch summary (single-difference + Kalman filter):")
    print(f"  Valid epochs: {len(valid)} / {len(epoch_summary)}")
    print(f"  ZHD (fixed):  {zhd_prior*1000:.1f} mm")
    print(f"  ZTD mean:     {valid['ZTD_m'].mean()*1000:.1f} mm")
    print(f"  ZTD std:      {valid['ZTD_m'].std()*1000:.1f} mm")
    print(f"  ZTD range:    {valid['ZTD_m'].min()*1000:.1f} – {valid['ZTD_m'].max()*1000:.1f} mm")
    print(f"  ZWD mean:     {valid['ZWD_m'].mean()*1000:.1f} mm")
    print(f"  ZWD std:      {valid['ZWD_m'].std()*1000:.1f} mm")
    if valid['PWV_mm'].notna().any():
        print(f"  PWV mean:     {valid['PWV_mm'].mean():.2f} mm")
        print(f"  PWV std:      {valid['PWV_mm'].std():.2f} mm")
        print(f"  PWV range:    {valid['PWV_mm'].min():.2f} – {valid['PWV_mm'].max():.2f} mm")
    print(f"\n  ZTD_sd (unfiltered) mean: {valid['ZTD_sd_m'].mean()*1000:.1f} mm")
    print(f"  ZTD_sd (unfiltered) std:  {valid['ZTD_sd_m'].std()*1000:.1f} mm")