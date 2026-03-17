import sqlite3
import numpy as np
import math
import pandas as pd

from tropospheric_products.precipitable_water_vapor import calculate_zenith_hydrostatic_delay, calculate_conversion_coefficient, calculate_mapping_function

ah = 1.2451128509e-03
aw = 6.0843905260e-04
bh = 2.7074805444e-03
bw = 1.4082272735e-03
ch = 5.6279169098e-02
cw = 4.0289703115e-02

c = 299792458.0  # Speed of light (m/s)

# Your existing values
surface_pressure = 846.597  # hPa
surface_temperature = 25.0  # Celsius
receiver_ecef = (867068.487, -5504812.066, 3092176.505)

db_path = '../data/ubx_data/2025-11-25/2025-11-25_serial-COM3_pwv_testing.db'
output_csv = '../data/ubx_data/2025-11-25/2025-11-25_pwv_testing_epoch.csv'

# Precompute constants
zhd = calculate_zenith_hydrostatic_delay(receiver_ecef, surface_pressure)
conversion_coeff = calculate_conversion_coefficient(surface_temperature)

conn = sqlite3.connect(db_path)

# Dynamically get all satellite tables (G01, G02, E01, etc.)
tables = pd.read_sql("SELECT name FROM sqlite_master WHERE type='table'", conn)
sat_tables = tables['name'].tolist()
print(f"Found {len(sat_tables)} satellite tables: {sat_tables}")

# Load and concatenate all satellite data
all_data = []
for table in sat_tables:
    df = pd.read_sql(f"SELECT rcvTOW, pseudorange, geoRange, biases, elevation FROM '{table}'", conn)
    df['svId'] = table
    all_data.append(df)

combined = pd.concat(all_data, ignore_index=True)
print(f"Total observations: {len(combined)}")

# Process epoch by epoch
epochs = sorted(combined['rcvTOW'].unique())
print(f"Total epochs: {len(epochs)}")

results = []
for epoch in epochs:
    epoch_data = combined[combined['rcvTOW'] == epoch].copy()

    # Drop any rows with NaN in required columns
    epoch_data = epoch_data.dropna(subset=['pseudorange', 'geoRange', 'biases', 'elevation'])

    # Optionally apply elevation cutoff
    epoch_data = epoch_data[epoch_data['elevation'] >= 10.0]

    n_sats = len(epoch_data)
    if n_sats < 2:
        continue

    mf_w_vals = []
    mf_h_vals = []
    y_vals = []
    elevations = []

    for _, row in epoch_data.iterrows():
        el = row['elevation']
        mf_h = calculate_mapping_function(el, ah, bh, ch)
        mf_w = calculate_mapping_function(el, aw, bw, cw)

        slant_total = row['pseudorange'] - row['geoRange'] - row['biases'] * c
        slant_hyd = mf_h * zhd
        slant_wet = slant_total - slant_hyd
        zwd_estimate = slant_wet / mf_w

        print(f"el={el:.1f} | STD={slant_total:.3f} | mf_h*ZHD={slant_hyd:.3f} | SWD={slant_wet:.3f} | ZWD={zwd_estimate:.3f}")

        y = row['pseudorange'] - row['geoRange'] - row['biases'] * c - mf_h * zhd

        mf_w_vals.append(mf_w)
        mf_h_vals.append(mf_h)
        y_vals.append(y)
        elevations.append(el)

    mf_w_arr = np.array(mf_w_vals)
    y_arr = np.array(y_vals)
    el_arr = np.array(elevations)
    weights = np.sin(np.radians(el_arr)) ** 2

    # 2-unknown WLS: estimate ZWD and residual clock correction together
    A = np.column_stack([mf_w_arr, np.ones(len(mf_w_arr))])
    W = np.diag(weights)

    try:
        AtWA = A.T @ W @ A
        AtWy = A.T @ W @ y_arr
        solution = np.linalg.solve(AtWA, AtWy)
        ZWD = solution[0]
        clk_residual = solution[1]
    except np.linalg.LinAlgError:
        print(f"Singular matrix at epoch {epoch:.3f}, skipping")
        continue

    ZTD = zhd + ZWD
    PWV_mm = conversion_coeff * ZWD * 1000.0

    # Compute post-fit residuals for quality check
    y_hat = A @ solution
    residuals = y_arr - y_hat
    rms = np.sqrt(np.mean(residuals ** 2))

    results.append({
        'rcvTOW': epoch,
        'ZHD_m': zhd,
        'ZWD_m': ZWD,
        'ZTD_m': ZTD,
        'PWV_mm': PWV_mm,
        'clk_residual_m': clk_residual,
        'n_sats': n_sats,
        'rms_m': rms
    })

conn.close()

results_df = pd.DataFrame(results)
results_df.to_csv(output_csv, index=False)
print(f"Saved {len(results_df)} epoch estimates to {output_csv}")
print(results_df[['rcvTOW', 'ZTD_m', 'ZWD_m', 'PWV_mm', 'n_sats', 'rms_m']].describe())