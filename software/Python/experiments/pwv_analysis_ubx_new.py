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
A_IONO = 40.28e16  # Ionospheric constant (m·Hz²/TECU)

# ─────────────────────────────────────────────
# File / database paths
# ─────────────────────────────────────────────
_DATA = Path(__file__).parent.parent / "data"
_DATA_data = Path("/mnt/NAS/Documents/Awohali/data")

rawx_file   = _DATA_data / "ubx_data/2025-11-25/2025-11-25_93138_serial-COM3_RXM_RAWX.csv"
clock_file  = _DATA_data / "ubx_data/2025-11-25/2025-11-25_93138_serial-COM3_NAV_CLOCK.csv"
ephemeris   = _DATA_data / "ephemerides/ephemeris_2025-11-25_RINEX.json"
results_dir = _DATA_data / "ubx_data/2025-11-25/2025-11-25_serial-COM3_pwv_testing_fast"

results_dir.mkdir(parents=True, exist_ok=True)

conn = sqlite3.connect(_DATA / "2025-11-25_serial-COM3_pwv_new_method.db")

# ─────────────────────────────────────────────
# Station / met parameters
# ─────────────────────────────────────────────
receiver_ecef = (867068.487, -5504812.066, 3092176.505)  # Campus quad

surface_temperature = 25.0           # °C
surface_pressure    = 1013.0         # hPa

# ─────────────────────────────────────────────
# VMF3 mapping coefficients
# ─────────────────────────────────────────────
ah, aw = 1.2451128509e-03, 6.0843905260e-04
bh, bw = 2.7074805444e-03, 1.4082272735e-03
ch, cw = 5.6279169098e-02, 4.0289703115e-02
coeffs = (ah, aw, bh, bw, ch, cw)

ELEV_CUTOFF_DEG = 20.0
GPS_ONLY        = False

ephemeris_data = load_ephemeris_data(ephemeris)

# ─────────────────────────────────────────────
# Kalman filter tuning
# ─────────────────────────────────────────────
Q_CLOCK_M2_PER_S = 1e12        # (m²/s) — re-estimated each epoch
Q_ZTD_M2_PER_S   = 2.5e-7      # (m²/s) — 0.5 mm/√s
Q_STEC_TECU2_PER_S = 1e-2      # (TECU²/s) — allows slow iono variation
R_CODE_M2         = 0.5**2      # (m²) — raw pseudorange noise at zenith

# ─────────────────────────────────────────────
# UBX signal plan (gnssId → sigId → service/freq)
# ─────────────────────────────────────────────
signal_plan = {
    0: {
        0:  {'service': 'L1_CA',   'freq': 1575.42},
        3:  {'service': 'L2_CL',   'freq': 1227.60},
        4:  {'service': 'L2_CM',   'freq': 1227.60},
        6:  {'service': 'L5_I',    'freq': 1176.45},
        7:  {'service': 'L5_Q',    'freq': 1176.45}
    },
    2: {
        0:  {'service': 'E1_C',    'freq': 1575.42},
        1:  {'service': 'E1_B',    'freq': 1575.42},
        3:  {'service': 'E5a_I',   'freq': 1176.45},
        4:  {'service': 'E5a_Q',   'freq': 1176.45},
        5:  {'service': 'E5b_I',   'freq': 1207.14},
        6:  {'service': 'E5b_Q',   'freq': 1207.14},
        8:  {'service': 'E6_B',    'freq': 1278.75},
        9:  {'service': 'E6_C',    'freq': 1278.75},
        10: {'service': 'E6_A',    'freq': 1278.75}
    }
}


# ══════════════════════════════════════════════
# Helpers — signal processing (raw, no IF)
# ══════════════════════════════════════════════

def parse_ubx_signals_raw(satellite_data, constellation, signal_plan):
    """
    Parse UBX RAWX signals for one satellite at one epoch.
    Returns a list of per-signal observations, one per distinct frequency.

    Each entry: {'pseudorange': m, 'carrierPhase': cycles, 'freq_mhz': MHz,
                 'signal': label, 'cno': dB-Hz, 'prStdev': m}

    If multiple tracking modes exist on the same frequency (e.g. L5-I and L5-Q),
    only the first is kept (one observation per frequency).
    """
    sigIDs = satellite_data['sigId'].unique()
    seen_freqs = {}  # freq_mhz → observation dict

    for sig in sorted(sigIDs):
        if sig not in signal_plan[constellation]:
            continue
        info = signal_plan[constellation][sig]
        freq_mhz = info['freq']

        if freq_mhz in seen_freqs:
            continue  # already have an observation on this frequency

        row = satellite_data[satellite_data['sigId'] == sig].iloc[0]
        pr = float(row['prMes'])
        cp = float(row['cpMes'])
        cno = float(row['cno']) if 'cno' in row.index else 0.0
        prStdev = float(row['prStdev']) if 'prStdev' in row.index else 0.0

        seen_freqs[freq_mhz] = {
            'pseudorange':  pr,
            'carrierPhase': cp,
            'freq_mhz':     freq_mhz,
            'signal':        info['service'],
            'cno':           cno,
            'prStdev':       prStdev,
        }

    return list(seen_freqs.values())


def correct_rcvtow(rcvTow_s, clkBias_ns):
    if clkBias_ns >= 500000:
        return rcvTow_s - 0.001
    elif clkBias_ns <= -500000:
        return rcvTow_s + 0.001
    return rcvTow_s


# ══════════════════════════════════════════════
# Helpers — mapping functions
# ══════════════════════════════════════════════

def marini(elevation_deg, a, b, c_coeff):
    """Marini continued-fraction mapping function."""
    sin_el = np.sin(np.radians(elevation_deg))
    num = 1.0 + a / (1.0 + b / (1.0 + c_coeff))
    den = sin_el + a / (sin_el + b / (sin_el + c_coeff))
    return num / den


def iono_coeff(freq_mhz):
    """Ionospheric delay coefficient: A_IONO / f² in metres/TECU."""
    f_hz = freq_mhz * 1e6
    return A_IONO / (f_hz ** 2)


# ══════════════════════════════════════════════
# Kalman filter for clock + ZTD + per-satellite STEC
# ══════════════════════════════════════════════

class KalmanFilterRaw:
    """
    Kalman filter for raw GNSS observation processing.

    State vector: [clock_m, ztd_m, stec_1, stec_2, ..., stec_n]
      - clock_m:  receiver clock bias in metres (re-estimated each epoch)
      - ztd_m:    zenith total delay in metres (slowly varying)
      - stec_i:   slant TEC for satellite i in TECU (absorbs true iono + ISC)

    The STEC states are dynamic: new satellites are added with large
    initial uncertainty, departed satellites are removed.
    """

    def __init__(self):
        self.x = None           # state vector
        self.P = None           # covariance matrix
        self.t_prev = None
        self.sat_indices = {}   # pvn → index in state vector (starting at 2)
        self.initialized = False

    def _n_states(self):
        return 2 + len(self.sat_indices)

    def _get_or_add_sat(self, pvn):
        """Return the state index for this satellite, adding it if new."""
        if pvn in self.sat_indices:
            return self.sat_indices[pvn]

        # Add new satellite STEC state
        new_idx = self._n_states()
        self.sat_indices[pvn] = new_idx

        # Extend state vector
        self.x = np.append(self.x, 0.0)  # initial STEC = 0 TECU

        # Extend covariance
        n = len(self.x)
        P_new = np.zeros((n, n))
        P_new[:n-1, :n-1] = self.P
        P_new[n-1, n-1] = 100.0**2  # 100 TECU initial uncertainty
        self.P = P_new

        return new_idx

    def initialize(self, clock0, ztd0, P_clock, P_ztd, t0):
        """Initialize with clock and ZTD from first WLS solution."""
        self.x = np.array([clock0, ztd0], dtype=float)
        self.P = np.diag([P_clock, P_ztd])
        self.t_prev = t0
        self.sat_indices = {}
        self.initialized = True

    def predict(self, t_now):
        """Time update: add process noise proportional to elapsed time."""
        dt = abs(t_now - self.t_prev) if self.t_prev is not None else 1.0
        dt = max(dt, 0.01)

        n = self._n_states()
        Q = np.zeros(n)
        Q[0] = Q_CLOCK_M2_PER_S * dt
        Q[1] = Q_ZTD_M2_PER_S * dt
        for i in range(2, n):
            Q[i] = Q_STEC_TECU2_PER_S * dt

        self.P += np.diag(Q)
        self.t_prev = t_now

    def update(self, observations):
        """
        Measurement update from raw observations.

        Parameters
        ----------
        observations : list of dict
            Each dict has: 'pvn', 'raw_residual', 'elevation', 'freq_mhz', 'prStdev'

        Returns dict with 'rcv_clock_m', 'ztd_m', 'stec' dict, 'n_obs'
        or None if insufficient observations.
        """
        if len(observations) < 3:
            return None

        # Ensure all satellites have state entries
        for obs in observations:
            self._get_or_add_sat(obs['pvn'])

        n_obs = len(observations)
        n_states = self._n_states()

        y = np.zeros(n_obs)
        H = np.zeros((n_obs, n_states))
        R = np.zeros((n_obs, n_obs))

        for i, obs in enumerate(observations):
            y[i] = obs['raw_residual']
            mf_h = marini(obs['elevation'], ah, bh, ch)
            alpha = iono_coeff(obs['freq_mhz'])
            sat_idx = self.sat_indices[obs['pvn']]

            H[i, 0] = 1.0      # clock
            H[i, 1] = mf_h     # ZTD
            H[i, sat_idx] = alpha  # STEC for this satellite

            # Measurement noise
            sin_el = np.sin(np.radians(obs['elevation']))
            if obs.get('prStdev', 0) > 0:
                R[i, i] = obs['prStdev'] ** 2
            else:
                R[i, i] = R_CODE_M2 / (sin_el ** 2)

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

        # Covariance update (Joseph form)
        I_KH = np.eye(n_states) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

        # Pack results
        stec_dict = {}
        for pvn, idx in self.sat_indices.items():
            stec_dict[pvn] = self.x[idx]

        return {
            'rcv_clock_m': self.x[0],
            'ztd_m':       self.x[1],
            'stec':        stec_dict,
            'n_obs':       n_obs,
        }

    def prune_satellites(self, active_pvns):
        """Remove STEC states for satellites no longer visible."""
        departed = [pvn for pvn in self.sat_indices if pvn not in active_pvns]
        if not departed:
            return

        # Indices to keep (always keep clock=0, ztd=1, plus active sats)
        keep_indices = [0, 1]
        new_sat_indices = {}
        for pvn, idx in sorted(self.sat_indices.items(), key=lambda x: x[1]):
            if pvn not in departed:
                new_idx = len(keep_indices)
                keep_indices.append(idx)
                new_sat_indices[pvn] = new_idx

        keep = np.array(keep_indices)
        self.x = self.x[keep]
        self.P = self.P[np.ix_(keep, keep)]
        self.sat_indices = new_sat_indices


def wls_raw_initial(observations):
    """
    Compute initial WLS solution for Kalman filter initialization.
    Estimates clock + ZTD + per-satellite STEC jointly.
    """
    n_obs = len(observations)
    if n_obs < 4:
        return None

    sat_set = sorted(set(o['pvn'] for o in observations))
    sat_map = {pvn: 2 + i for i, pvn in enumerate(sat_set)}
    n_params = 2 + len(sat_set)

    if n_obs < n_params + 1:
        return None

    y = np.zeros(n_obs)
    A = np.zeros((n_obs, n_params))
    W = np.zeros(n_obs)

    for i, obs in enumerate(observations):
        y[i] = obs['raw_residual']
        mf_h = marini(obs['elevation'], ah, bh, ch)
        alpha = iono_coeff(obs['freq_mhz'])

        A[i, 0] = 1.0                      # clock
        A[i, 1] = mf_h                     # ZTD
        A[i, sat_map[obs['pvn']]] = alpha   # STEC

        sin_el = np.sin(np.radians(obs['elevation']))
        W[i] = sin_el ** 2

    Wd = np.diag(W)
    try:
        ATWA = A.T @ Wd @ A
        ATWy = A.T @ Wd @ y
        x0   = np.linalg.solve(ATWA, ATWy)

        v = y - A @ x0
        sigma2 = np.sum(W * v**2) / max(n_obs - n_params, 1)
        P0 = sigma2 * np.linalg.inv(ATWA)
    except np.linalg.LinAlgError:
        return None

    return {
        'clock': x0[0],
        'ztd':   x0[1],
        'P_clock': P0[0, 0],
        'P_ztd':   P0[1, 1],
        'stec':  {pvn: x0[sat_map[pvn]] for pvn in sat_set},
    }


# ══════════════════════════════════════════════
# Helpers — geodetic & meteorological
# ══════════════════════════════════════════════

def ecef_to_geodetic(x, y, z):
    """Bowring iterative ECEF → geodetic (WGS-84). Returns (lat_deg, lon_deg, height_m)."""
    a  = 6378137.0
    f  = 1.0 / 298.257223563
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
    Pi  = 1e-6 * (k2p + k3 / tm_k) * rv * rho / 100.0
    return (zwd_m / Pi) * 1000.0   # → mm


# ══════════════════════════════════════════════
# Load data
# ══════════════════════════════════════════════
rawx  = pd.read_csv(rawx_file)
clock = pd.read_csv(clock_file)
rawx_length = len(rawx)
total_measurements = len(rawx)

station_lat, station_lon, station_hgt = ecef_to_geodetic(*receiver_ecef)
station_hgt_km = station_hgt / 1000.0
zhd_prior = saastamoinen_zhd(surface_pressure, station_lat, station_hgt_km)
tm = bevis_tm(surface_temperature + 273.15)

print(f"Station geodetic: lat={station_lat:.6f}°  lon={station_lon:.6f}°  h={station_hgt:.3f} m")
print(f"A priori ZHD (Saastamoinen): {zhd_prior:.4f} m ({zhd_prior*1000:.1f} mm)")
print(f"Mean atmospheric temperature Tm: {tm:.1f} K")
print(f"Station atmospheric pressure: {surface_pressure:.2f} hPa")
print(f"Constellation mode: {'GPS only' if GPS_ONLY else 'GPS + Galileo'}")
print(f"\nRaw observation Kalman filter:")
print(f"  Clock process noise: {Q_CLOCK_M2_PER_S:.0e} m²/s (unconstrained)")
print(f"  ZTD process noise:   {Q_ZTD_M2_PER_S:.1e} m²/s ({np.sqrt(Q_ZTD_M2_PER_S)*1e3:.2f} mm/√s)")
print(f"  STEC process noise:  {Q_STEC_TECU2_PER_S:.1e} TECU²/s")
print(f"  Code noise at zenith: {np.sqrt(R_CODE_M2):.2f} m")

# Precompute ionospheric coefficients for common frequencies
freq_L1 = 1575.42
freq_L5 = 1176.45
print(f"\nIonospheric coefficients:")
print(f"  L1 ({freq_L1} MHz): {iono_coeff(freq_L1):.6e} m/TECU")
print(f"  L5 ({freq_L5} MHz): {iono_coeff(freq_L5):.6e} m/TECU")
print(f"  Ratio L5/L1: {iono_coeff(freq_L5)/iono_coeff(freq_L1):.3f}")

# ══════════════════════════════════════════════
# Pass 1 — per-signal geometry and raw residuals
# ══════════════════════════════════════════════
# For each satellite, for each signal (L1, L5, etc.):
#   raw_residual = P_f - rho + c*(dt_sv + dt_rel)
#                = c*dt_r + mf_h(el)*ZTD + (A/f²)*STEC + epsilon
#
# Each satellite with two frequencies produces TWO observations.
# No ionosphere-free combination is formed.

print("\nPass 1: computing per-signal geometry and raw residuals...")
start_time = time.time()

epoch_buffer = defaultdict(list)  # rcvTow → list of per-signal observations
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

            # Parse all raw signals (one per frequency, no IF combination)
            raw_signals = parse_ubx_signals_raw(satellite_data, constellation, signal_plan)
            if not raw_signals:
                continue

            # Ephemeris and geometry (computed once per satellite, shared by all signals)
            try:
                sat = load_ephemeris(ephemeris_data, pvn, gps_tow=gpsTow_s)
            except (KeyError, ValueError):
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

            elevation_deg, azimuth_deg = calculate_elevation_azimuth(sat_ecef, receiver_ecef)

            # One raw residual per signal
            for sig in raw_signals:
                raw_residual = sig['pseudorange'] - geo_range_m + (dt_sv_s + dt_rel_s) * c

                epoch_buffer[rcvTow_s].append({
                    'pvn':            pvn,
                    'rcvTOW':         rcvTow_s,
                    'gpsTOW':         gpsTow_s,
                    'iTOW':           iTow_ms / 1000.0,
                    'gpsWeek':        gpsWeek,
                    't_tx':           t_tx_s,
                    'geoRange':       geo_range_m,
                    'elevation':      elevation_deg,
                    'azimuth':        azimuth_deg,
                    'satClockBias':   dt_sv_s * c,
                    'relBias':        dt_rel_s * c,
                    'pseudorange':    sig['pseudorange'],
                    'carrierPhase':   sig['carrierPhase'],
                    'freq_mhz':      sig['freq_mhz'],
                    'signal':         sig['signal'],
                    'cno':            sig['cno'],
                    'prStdev':        sig['prStdev'],
                    'raw_residual':   raw_residual,
                })

    i += numMeas
    if i >= rawx_length:
        break

sys.stdout.write('\n')
pass1_time = time.time() - start_time

# Count observations
n_total_obs = sum(len(obs_list) for obs_list in epoch_buffer.values())
n_epochs = len(epoch_buffer)
print(f"Pass 1 completed in {pass1_time:.1f} seconds")
print(f"  {n_total_obs:,} raw observations across {n_epochs:,} epochs")
print(f"  Average {n_total_obs/max(n_epochs,1):.1f} observations per epoch")

# ══════════════════════════════════════════════
# Pass 2 — joint clock/ZTD/STEC estimation
#           from raw observations
# ══════════════════════════════════════════════
# Observation model per signal:
#   r_{i,f} = c·dt_r + mf_h(el_i)·ZTD + (A/f²)·STEC_i
#
# The receiver clock is common to all observations (frequency-independent).
# ZTD is common to all observations (frequency-independent, elevation-dependent).
# STEC is per-satellite (frequency-dependent via A/f²).
#
# With 7 satellites × 2 frequencies = 14 observations and 9 unknowns
# (1 clock + 1 ZTD + 7 STECs), the system is overdetermined.
#
# ISC biases are absorbed by the per-satellite STEC estimates.
# ZTD is clean because it is frequency-independent.

print("Pass 2: raw observation joint estimation (clock + ZTD + STEC)...")

kf = KalmanFilterRaw()
gnss_results_lists = defaultdict(list)
epoch_summary_list = []
epochs_total = len(epoch_buffer)
epochs_done  = 0
skipped_epochs = 0

for rcvTow, obs_list in sorted(epoch_buffer.items()):
    epochs_done += 1

    # Filter by elevation
    above = [o for o in obs_list if o['elevation'] >= ELEV_CUTOFF_DEG]

    # Need at least 2 satellites with 2 frequencies each
    sat_freqs = defaultdict(set)
    for o in above:
        sat_freqs[o['pvn']].add(o['freq_mhz'])
    dual_freq_sats = {pvn for pvn, freqs in sat_freqs.items() if len(freqs) >= 2}

    # Keep only observations from dual-frequency satellites
    above = [o for o in above if o['pvn'] in dual_freq_sats]

    n_sats = len(dual_freq_sats)
    n_obs = len(above)

    sys.stdout.write(f'\r  Epoch {epochs_done}/{epochs_total}  '
                     f'({n_sats} sats, {n_obs} obs)')
    sys.stdout.flush()

    if n_sats < 3 or n_obs < (n_sats + 3):
        skipped_epochs += 1
        continue

    # ── Initialize Kalman filter from first WLS ──────────────
    if not kf.initialized:
        wls = wls_raw_initial(above)
        if wls is None:
            skipped_epochs += 1
            continue
        kf.initialize(wls['clock'], wls['ztd'], wls['P_clock'], wls['P_ztd'], rcvTow)
        # Pre-populate STEC states from WLS
        for pvn, stec_val in wls['stec'].items():
            idx = kf._get_or_add_sat(pvn)
            kf.x[idx] = stec_val

    # ── Kalman predict ────────────────────────────────────────
    kf.predict(rcvTow)

    # ── Prune departed satellites ─────────────────────────────
    active_pvns = set(o['pvn'] for o in above)
    kf.prune_satellites(active_pvns)

    # ── Kalman update ─────────────────────────────────────────
    solution = kf.update(above)

    if solution is None:
        skipped_epochs += 1
        continue

    rcv_clock_m = solution['rcv_clock_m']
    ztd_m       = solution['ztd_m']
    stec_dict   = solution['stec']

    # ── Decompose ZTD ─────────────────────────────────────────
    zwd_m = ztd_m - zhd_prior
    pwv_mm = zwd_to_pwv(zwd_m, tm)

    # ── Store per-signal results ──────────────────────────────
    epoch_svids = []
    seen_sats = set()

    for o in above:
        mf_h = marini(o['elevation'], ah, bh, ch)
        alpha = iono_coeff(o['freq_mhz'])
        stec_est = stec_dict.get(o['pvn'], 0.0)

        # Modeled observation
        model = rcv_clock_m + mf_h * ztd_m + alpha * stec_est
        post_fit_resid = o['raw_residual'] - model

        # Modeled slant tropospheric delay
        tropo_slant = mf_h * ztd_m

        if o['pvn'] not in seen_sats:
            epoch_svids.append(o['pvn'])
            seen_sats.add(o['pvn'])

        gnss_results_lists[o['pvn']].append({
            'svId':              o['pvn'],
            'iTOW':              o['iTOW'],
            'gpsTOW':            o['gpsTOW'],
            'rcvTOW':            o['rcvTOW'],
            'WN':                o['gpsWeek'],
            'geoRange':          o['geoRange'],
            'elevation':         o['elevation'],
            'azimuth':           o['azimuth'],
            'satClockBias':      o['satClockBias'],
            'relBias':           o['relBias'],
            'rcvClockBias_m':    rcv_clock_m,
            'ztd_est_m':         ztd_m,
            'STEC_TECU':         stec_est,
            'pseudorange':       o['pseudorange'],
            'carrierPhase':      o['carrierPhase'],
            'freq_mhz':         o['freq_mhz'],
            'signal':            o['signal'],
            'cno':               o['cno'],
            'prStdev':           o['prStdev'],
            'raw_residual':      o['raw_residual'],
            'post_fit_residual': post_fit_resid,
            'troposphericDelay': tropo_slant,
            'pwv':               pwv_mm,
            'ztd':               ztd_m * 1000.0,
        })

    # ── Epoch summary ─────────────────────────────────────────
    epoch_summary_list.append({
        'rcvTOW':         rcvTow,
        'gpsTOW':         above[0]['gpsTOW'] if above else np.nan,
        'ZHD_m':          zhd_prior,
        'ZWD_m':          zwd_m,
        'ZTD_m':          ztd_m,
        'ZTD_sigma_m':    np.sqrt(kf.P[1, 1]),
        'PWV_mm':         pwv_mm,
        'rcvClockBias_m': rcv_clock_m,
        'n_sats':         n_sats,
        'n_obs':          n_obs,
        'sats_used':      ','.join(sorted(epoch_svids)),
    })

sys.stdout.write('\n')
if skipped_epochs:
    print(f"  Skipped {skipped_epochs} epochs with insufficient dual-frequency satellites above {ELEV_CUTOFF_DEG}°")

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
    print(f"\nEpoch summary (raw observation processing):")
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
        print(f"  (Expected:    ~10-30 mm for Florida)")

    # Post-fit residual statistics
    all_resids = []
    for df in gnss_results.values():
        if 'post_fit_residual' in df.columns:
            all_resids.extend(df['post_fit_residual'].dropna().tolist())
    if all_resids:
        resids = np.array(all_resids)
        print(f"\n  Post-fit residuals ({len(resids):,} observations):")
        print(f"    RMS:    {np.sqrt(np.mean(resids**2)):.3f} m")
        print(f"    Mean:   {np.mean(resids):.3f} m")
        print(f"    Std:    {np.std(resids):.3f} m")