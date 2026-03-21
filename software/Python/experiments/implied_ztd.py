"""
Diagnostic: compute implied ZTD from pairs of satellites at each epoch.

The receiver clock cancels in the difference of two raw residuals:
    raw_resid_i - raw_resid_j = mf_h(el_i)*ZTD - mf_h(el_j)*ZTD
                               = (mf_h(el_i) - mf_h(el_j)) * ZTD

So:
    implied_ZTD = (raw_resid_i - raw_resid_j) / (mf_h(el_i) - mf_h(el_j))

If the raw residuals are clean (only clock + troposphere), every pair
should give ~2.2-2.5 m regardless of the actual clock value.

Large deviations indicate per-satellite biases (multipath, IF combination
errors, ephemeris issues, etc.) that the least-squares cannot absorb
into a single clock parameter.
"""

import sqlite3
import numpy as np
import pandas as pd
from pathlib import Path
from itertools import combinations

# ─────────────────────────────────────────────
# Configuration — change these to match your setup
# ─────────────────────────────────────────────

# UBX database
UBX_DB = Path("../data/ubx_data/2025-11-25/2025-11-25_serial-COM3_pwv_testing_fast_filter.db")

# RINEX database (for comparison)
RINEX_DB = Path("../data/RINEX_data/ORMD/2025-11-25/RINEX_pwv_test_new_filter.db")

# VMF3 hydrostatic mapping function coefficients
ah = 1.2451128509e-03
bh = 2.7074805444e-03
ch = 5.6279169098e-02

# Elevation cutoff
ELEV_CUTOFF_DEG = 20.0

# How many epochs to analyze (None = all)
MAX_EPOCHS = 50

# Minimum elevation spread between a pair (degrees) for it to be useful
MIN_ELEV_SPREAD = 15.0


# ─────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────

def marini(elevation_deg, a, b, c_coeff):
    """Marini continued-fraction mapping function."""
    sin_el = np.sin(np.radians(elevation_deg))
    num = 1.0 + a / (1.0 + b / (1.0 + c_coeff))
    den = sin_el + a / (sin_el + b / (sin_el + c_coeff))
    return num / den


def get_satellite_tables(conn):
    """Return all satellite table names (G## or E##)."""
    query = """
        SELECT name FROM sqlite_master
        WHERE type='table'
          AND (name GLOB 'G[0-9][0-9]' OR name GLOB 'E[0-9][0-9]')
        ORDER BY name
    """
    return pd.read_sql_query(query, conn)['name'].tolist()


def load_all_satellites(db_path):
    """Load all satellite tables into one DataFrame."""
    conn = sqlite3.connect(db_path)
    tables = get_satellite_tables(conn)
    if not tables:
        print(f"  No satellite tables found in {db_path}")
        conn.close()
        return None

    frames = []
    for tbl in tables:
        df = pd.read_sql_query(f'SELECT * FROM "{tbl}"', conn)
        if 'svId' not in df.columns:
            df['svId'] = tbl
        frames.append(df)

    conn.close()
    combined = pd.concat(frames, ignore_index=True)
    print(f"  Loaded {len(tables)} satellite tables, {len(combined):,} rows")
    return combined


def analyze_implied_ztd(df, label, max_epochs=None):
    """
    For each epoch, compute implied ZTD from all satellite pairs
    and report statistics.
    """
    if 'raw_residual' not in df.columns:
        print(f"  ERROR: 'raw_residual' column not found in {label} database.")
        print(f"  Available columns: {list(df.columns)}")
        return None

    # Filter by elevation
    df = df[df['elevation'] >= ELEV_CUTOFF_DEG].copy()

    # Get unique epochs
    epochs = sorted(df['rcvTOW'].unique())
    if max_epochs is not None:
        # Sample evenly across the session
        step = max(1, len(epochs) // max_epochs)
        epochs = epochs[::step][:max_epochs]

    print(f"\n{'='*70}")
    print(f"  Implied ZTD analysis: {label}")
    print(f"  Analyzing {len(epochs)} epochs, elevation cutoff {ELEV_CUTOFF_DEG}°")
    print(f"  Minimum elevation spread for pairs: {MIN_ELEV_SPREAD}°")
    print(f"{'='*70}")

    all_implied_ztd = []
    epoch_medians = []
    problem_epochs = []

    for epoch_idx, tow in enumerate(epochs):
        epoch_df = df[df['rcvTOW'] == tow]

        sats = []
        for _, row in epoch_df.iterrows():
            sats.append({
                'svId': row['svId'],
                'elevation': row['elevation'],
                'raw_residual': row['raw_residual'],
                'mf_h': marini(row['elevation'], ah, bh, ch),
            })

        if len(sats) < 2:
            continue

        # Compute implied ZTD for all pairs with sufficient elevation spread
        pair_ztds = []
        for s1, s2 in combinations(sats, 2):
            elev_spread = abs(s1['elevation'] - s2['elevation'])
            if elev_spread < MIN_ELEV_SPREAD:
                continue

            dmf = s1['mf_h'] - s2['mf_h']
            if abs(dmf) < 1e-6:
                continue

            dresid = s1['raw_residual'] - s2['raw_residual']
            implied_ztd = dresid / dmf

            pair_ztds.append({
                'rcvTOW': tow,
                'sat_i': s1['svId'],
                'sat_j': s2['svId'],
                'el_i': s1['elevation'],
                'el_j': s2['elevation'],
                'elev_spread': elev_spread,
                'resid_i': s1['raw_residual'],
                'resid_j': s2['raw_residual'],
                'mf_i': s1['mf_h'],
                'mf_j': s2['mf_h'],
                'implied_ztd': implied_ztd,
            })

        if not pair_ztds:
            continue

        ztd_values = [p['implied_ztd'] for p in pair_ztds]
        median_ztd = np.median(ztd_values)
        epoch_medians.append(median_ztd)
        all_implied_ztd.extend(pair_ztds)

        # Flag epochs with high spread or unreasonable values
        ztd_std = np.std(ztd_values)
        if ztd_std > 1.0 or median_ztd < 0 or median_ztd > 5:
            problem_epochs.append({
                'rcvTOW': tow,
                'median_ztd': median_ztd,
                'std_ztd': ztd_std,
                'n_pairs': len(pair_ztds),
                'n_sats': len(sats),
            })

        # Print detail for first few epochs
        if epoch_idx < 5:
            print(f"\n  Epoch rcvTOW={tow:.3f}  ({len(sats)} sats, "
                  f"{len(pair_ztds)} pairs with >{MIN_ELEV_SPREAD}° spread)")
            print(f"  {'Pair':<12} {'El_i':>6} {'El_j':>6} {'Spread':>7} "
                  f"{'Resid_i':>14} {'Resid_j':>14} {'Impl ZTD':>10}")
            print(f"  {'-'*12} {'-'*6} {'-'*6} {'-'*7} {'-'*14} {'-'*14} {'-'*10}")
            for p in sorted(pair_ztds, key=lambda x: -x['elev_spread']):
                print(f"  {p['sat_i']+'/'+p['sat_j']:<12} "
                      f"{p['el_i']:6.1f} {p['el_j']:6.1f} {p['elev_spread']:7.1f} "
                      f"{p['resid_i']:14.3f} {p['resid_j']:14.3f} "
                      f"{p['implied_ztd']:10.3f}")
            print(f"  → Median implied ZTD: {median_ztd:.3f} m  "
                  f"(std: {ztd_std:.3f} m)")

    # Overall summary
    if epoch_medians:
        print(f"\n  {'─'*50}")
        print(f"  Overall implied ZTD across {len(epoch_medians)} epochs:")
        print(f"    Mean of medians:  {np.mean(epoch_medians):.3f} m")
        print(f"    Std of medians:   {np.std(epoch_medians):.3f} m")
        print(f"    Min median:       {np.min(epoch_medians):.3f} m")
        print(f"    Max median:       {np.max(epoch_medians):.3f} m")
        print(f"    Expected:         ~2.2 – 2.5 m")

        if np.mean(epoch_medians) < 1.5 or np.mean(epoch_medians) > 4.0:
            print(f"\n    ⚠ Mean implied ZTD is outside expected range!")
            print(f"    This suggests systematic per-satellite biases in")
            print(f"    the raw residuals beyond clock + troposphere.")

        if np.std(epoch_medians) > 0.5:
            print(f"\n    ⚠ High epoch-to-epoch variability in implied ZTD.")
            print(f"    This suggests noisy pseudoranges or changing biases.")

    if problem_epochs:
        print(f"\n  Flagged {len(problem_epochs)} problem epochs "
              f"(negative ZTD, ZTD>5m, or std>1m):")
        for p in problem_epochs[:10]:
            print(f"    TOW={p['rcvTOW']:.3f}  median={p['median_ztd']:.3f} m  "
                  f"std={p['std_ztd']:.3f} m  ({p['n_sats']} sats)")
        if len(problem_epochs) > 10:
            print(f"    ... and {len(problem_epochs)-10} more")

    # Per-satellite bias check: for each satellite, what's its average
    # deviation from the epoch median?
    if all_implied_ztd:
        print(f"\n  {'─'*50}")
        print(f"  Per-satellite bias check:")
        print(f"  (Positive = this satellite's pairs give higher ZTD than median)")
        print(f"  {'Satellite':<10} {'Mean offset':>12} {'N pairs':>8}")

        pair_df = pd.DataFrame(all_implied_ztd)

        # Compute epoch medians
        epoch_med = pair_df.groupby('rcvTOW')['implied_ztd'].median()
        pair_df['epoch_median'] = pair_df['rcvTOW'].map(epoch_med)
        pair_df['offset'] = pair_df['implied_ztd'] - pair_df['epoch_median']

        # Average offset per satellite (appearing in either position)
        sat_offsets = {}
        for sat in set(pair_df['sat_i'].tolist() + pair_df['sat_j'].tolist()):
            as_i = pair_df[pair_df['sat_i'] == sat]['offset']
            as_j = pair_df[pair_df['sat_j'] == sat]['offset'] * -1  # flip sign
            all_offsets = pd.concat([as_i, as_j])
            if len(all_offsets) > 0:
                sat_offsets[sat] = (all_offsets.mean(), len(all_offsets))

        for sat in sorted(sat_offsets.keys()):
            mean_off, n = sat_offsets[sat]
            flag = "  ← OUTLIER" if abs(mean_off) > 0.5 else ""
            print(f"  {sat:<10} {mean_off:>+12.3f} m {n:>8}{flag}")

    return all_implied_ztd


# ─────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────

if __name__ == '__main__':

    if UBX_DB.exists():
        print(f"\nLoading UBX data from {UBX_DB}...")
        ubx_df = load_all_satellites(UBX_DB)
        if ubx_df is not None:
            analyze_implied_ztd(ubx_df, "UBX", max_epochs=MAX_EPOCHS)

    if RINEX_DB.exists():
        print(f"\nLoading RINEX data from {RINEX_DB}...")
        rinex_df = load_all_satellites(RINEX_DB)
        if rinex_df is not None:
            analyze_implied_ztd(rinex_df, "RINEX", max_epochs=MAX_EPOCHS)

    if not UBX_DB.exists() and not RINEX_DB.exists():
        print("No databases found. Update UBX_DB and/or RINEX_DB paths.")