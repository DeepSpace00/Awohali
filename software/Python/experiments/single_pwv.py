import sqlite3
import numpy as np
import pandas as pd
from pathlib import Path

# ─────────────────────────────────────────────
# VMF3 coefficients for your session/site
# ─────────────────────────────────────────────
ah, aw = 1.2451128509e-03, 6.0843905260e-04
bh, bw = 2.7074805444e-03, 1.4082272735e-03
ch, cw = 5.6279169098e-02, 4.0289703115e-02

# ─────────────────────────────────────────────
# Station ECEF coordinates (metres)
# ─────────────────────────────────────────────
# STATION_X =  867068.487   # ← replace with your values
# STATION_Y = -5504812.066
# STATION_Z =  3092176.505

STATION_X = 860376.4154
STATION_Y = -5499833.4036
STATION_Z = 3102756.9385

# ─────────────────────────────────────────────
# Surface met (from your barometer/thermometer)
# ─────────────────────────────────────────────
PRESSURE_HPA   = 846.597166666675    # hPa
SURFACE_TEMP_K = 298.15    # Kelvin  (°C + 273.15)

# ─────────────────────────────────────────────
# Elevation cutoff
# ─────────────────────────────────────────────
ELEV_CUTOFF_DEG  = 10.0

# DB_PATH = Path("../data/ubx_data/2025-11-25/2025-11-25_serial-COM3_pwv_testing_fast.db")
DB_PATH = Path("../data/RINEX_data/ORMD/2025-11-25/RINEX_pwv_test_new.db")

# ══════════════════════════════════════════════
# Helpers
# ══════════════════════════════════════════════

def ecef_to_geodetic(x, y, z):
    """
    Bowring / Zhu (1994) iterative ECEF → geodetic conversion.
    Returns (lat_deg, lon_deg, height_m) on WGS-84.
    """
    # WGS-84 ellipsoid parameters
    a  =  6378137.0          # semi-major axis (m)
    f  =  1.0 / 298.257223563
    b  =  a * (1.0 - f)      # semi-minor axis
    e2 =  2*f - f**2         # first eccentricity squared
    ep2 = (a**2 - b**2) / b**2  # second eccentricity squared

    p   = np.sqrt(x**2 + y**2)  # distance from Z-axis
    lon = np.arctan2(y, x)

    # Iterative solution for latitude (Bowring, converges in 2–3 iterations)
    lat = np.arctan2(z, p * (1.0 - e2))   # initial estimate
    for _ in range(10):
        sin_lat = np.sin(lat)
        N       = a / np.sqrt(1.0 - e2 * sin_lat**2)   # radius of curvature
        lat_new = np.arctan2(z + e2 * N * sin_lat, p)
        if abs(lat_new - lat) < 1e-12:
            break
        lat = lat_new

    sin_lat = np.sin(lat)
    N       = a / np.sqrt(1.0 - e2 * sin_lat**2)
    h       = p / np.cos(lat) - N if abs(lat) < np.radians(45) else \
              z / np.sin(lat) - N * (1.0 - e2)

    return np.degrees(lat), np.degrees(lon), h   # h in metres

def marini(elev_deg, a, b, c):
    """Marini continued-fraction mapping function."""
    s   = np.sin(np.radians(elev_deg))
    num = 1.0 + a / (1.0 + b / (1.0 + c))
    den = s   + a / (s   + b / (s   + c))
    return num / den


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


# ══════════════════════════════════════════════
# Database loading
# ══════════════════════════════════════════════

def get_satellite_tables(conn):
    """Return all table names matching G_## or E_## pattern."""
    query = """
        SELECT name FROM sqlite_master
        WHERE type='table'
          AND (name LIKE 'G_%' OR name LIKE 'E_%')
        ORDER BY name
    """
    tables = pd.read_sql_query(query, conn)['name'].tolist()
    return tables


def load_all_satellites(conn):
    """
    Load every G_## and E_## table, tag each row with its svId,
    and concatenate into a single DataFrame.
    """
    tables = get_satellite_tables(conn)
    if not tables:
        raise RuntimeError("No satellite tables (G_## / E_##) found in database.")

    frames = []
    for tbl in tables:
        df = pd.read_sql_query(f'SELECT * FROM "{tbl}"', conn)

        # Ensure svId column reflects the table name
        # (in case the stored svId differs or is missing)
        df['svId'] = tbl

        frames.append(df)

    combined = pd.concat(frames, ignore_index=True)
    print(f"Loaded {len(tables)} satellite tables → {len(combined):,} rows total")
    return combined


# ══════════════════════════════════════════════
# ZTD estimation
# ══════════════════════════════════════════════

def estimate_ztd_epoch(group, zhd_prior=None):
    """
    Weighted least-squares ZTD estimate for one epoch.

    If zhd_prior is given (metres), only ZWD is estimated (recommended).
    Otherwise ZHD and ZWD are estimated jointly (less stable).
    """
    g = group[group['elevation'] >= ELEV_CUTOFF_DEG].copy()

    min_sats = 3 if zhd_prior is not None else 4
    if len(g) < min_sats:
        return pd.Series({
            'ZHD_m': np.nan, 'ZWD_m': np.nan,
            'ZTD_m': np.nan, 'ZTD_sigma_m': np.nan,
            'n_sats': len(g), 'sats_used': ''
        })

    elev = g['elevation'].values
    std  = g['troposphericDelay'].values
    w    = np.sin(np.radians(elev)) ** 2      # elevation-dependent weights
    W    = np.diag(w)

    mh = marini(elev, ah, bh, ch)
    mw = marini(elev, aw, bw, cw)

    if zhd_prior is not None:
        # Fix ZHD from Saastamoinen, estimate ZWD only
        z_red  = std - mh * zhd_prior
        H      = mw.reshape(-1, 1)
        HtWH = float((H.T @ W @ H)[0, 0])  # H.T@W@H is (1,1) → needs [0,0]
        HtWz = float((H.T @ W @ z_red)[0])  # H.T@W@z is (1,)  → needs [0]
        zwd = float(HtWz / HtWH)
        zhd    = zhd_prior
    else:
        # Estimate ZHD and ZWD jointly
        H      = np.column_stack([mh, mw])
        HtWH   = H.T @ W @ H
        HtWz   = H.T @ W @ std
        cond   = np.linalg.cond(HtWH)
        if cond > 1e6:
            print(f"  ⚠ Epoch {group['rcvTOW'].iloc[0]:.0f}: "
                  f"H'WH condition number {cond:.1e} — solution unreliable")
        x_hat  = np.linalg.solve(HtWH, HtWz)
        zhd, zwd = x_hat[0], x_hat[1]

    # Residuals and formal ZTD uncertainty
    resid     = std - (mh * zhd + mw * zwd)
    sigma2    = np.sum(w * resid**2) / max(len(g) - 1, 1)
    H_full    = np.column_stack([mh, mw])
    cov_x     = sigma2 * np.linalg.inv(H_full.T @ W @ H_full)
    ztd_sigma = float(np.sqrt(np.array([1., 1.]) @ cov_x @ np.array([1., 1.])))

    return pd.Series({
        'ZHD_m':       zhd,
        'ZWD_m':       zwd,
        'ZTD_m':       zhd + zwd,
        'ZTD_sigma_m': ztd_sigma,
        'n_sats':      len(g),
        'sats_used':   ','.join(g['svId'].astype(str).values)
    })


# ══════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════

def main():
    # ── 1. Load from SQLite ──────────────────
    with sqlite3.connect(DB_PATH) as conn:
        df = load_all_satellites(conn)

    # Compute once at startup
    STATION_LAT_DEG, STATION_LON_DEG, STATION_HGT_M = ecef_to_geodetic(
        STATION_X, STATION_Y, STATION_Z
    )
    STATION_HGT_KM = STATION_HGT_M / 1000.0

    print(f"Station geodetic: "
          f"lat={STATION_LAT_DEG:.6f}°  "
          f"lon={STATION_LON_DEG:.6f}°  "
          f"h={STATION_HGT_M:.3f} m")

    # ── 2. Optional: filter to a time window ─
    # Uncomment and adjust iTOW bounds (seconds) if needed:
    # df = df[(df['iTOW'] >= 345600) & (df['iTOW'] < 356400)]

    # ── 3. Drop rows missing critical columns ─
    required = ['rcvTOW', 'elevation', 'troposphericDelay', 'svId']
    before   = len(df)
    df       = df.dropna(subset=required)
    dropped  = before - len(df)
    if dropped:
        print(f"Dropped {dropped} rows with NaN in required columns")

    # ── 4. Summary of what we have ───────────
    constellations = df['svId'].str[0].value_counts()
    print(f"\nConstellation breakdown:\n{constellations.to_string()}")
    print(f"Epochs: {df['rcvTOW'].nunique()}")
    print(f"Elevation range: {df['elevation'].min():.1f}° – "
          f"{df['elevation'].max():.1f}°\n")

    # ── 5. ZHD from Saastamoinen (if pressure available) ─
    zhd_prior = None
    if PRESSURE_HPA is not None:
        zhd_prior = saastamoinen_zhd(PRESSURE_HPA, STATION_LAT_DEG, STATION_HGT_KM)
        print(f"A priori ZHD (Saastamoinen): {zhd_prior:.4f} m")

    # ── 6. Estimate ZTD per epoch ─────────────
    print("Estimating ZTD per epoch...")
    ztd_ts = (
        df.groupby('rcvTOW')
          .apply(estimate_ztd_epoch, zhd_prior=zhd_prior)
          .reset_index()
    )

    if zhd_prior is not None and SURFACE_TEMP_K is not None:
        tm  = bevis_tm(SURFACE_TEMP_K)
        ztd_ts['PWV_mm'] = ztd_ts['ZWD_m'].apply(
            lambda zwd: zwd_to_pwv(zwd, tm) if pd.notna(zwd) else np.nan
        )
        print(f"Mean atmospheric temperature Tm = {tm:.1f} K")
    else:
        ztd_ts['PWV_mm'] = np.nan

    # ── 8. Print summary ─────────────────────
    valid = ztd_ts.dropna(subset=['ZTD_m'])
    print(f"\nValid epochs:   {len(valid)} / {len(ztd_ts)}")
    print(f"ZTD mean:       {valid['ZTD_m'].mean()*1000:.1f} mm")
    print(f"ZTD std:        {valid['ZTD_m'].std()*1000:.1f} mm")
    print(f"ZTD range:      {valid['ZTD_m'].min()*1000:.1f} – "
          f"{valid['ZTD_m'].max()*1000:.1f} mm")
    if zhd_prior is not None:
        print(f"ZWD mean:       {valid['ZWD_m'].mean()*1000:.1f} mm")
    if 'PWV_mm' in valid and valid['PWV_mm'].notna().any():
        print(f"PWV mean:       {valid['PWV_mm'].mean():.2f} mm")

    # ── 9. Save results ──────────────────────
    out_path = DB_PATH.with_name(DB_PATH.stem + '_ztd.csv')
    ztd_ts.to_csv(out_path, index=False, float_format='%.6f')
    print(f"\nResults saved → {out_path}")

    return ztd_ts


if __name__ == '__main__':
    ztd_ts = main()