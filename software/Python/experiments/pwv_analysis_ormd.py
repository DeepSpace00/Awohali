from collections import defaultdict
from datetime import datetime
import os
import pandas as pd
import sqlite3
import sys
import time

from calculations import clock_correction, geometric_range
from ephemerides.ephemeris import load_ephemeris
from calculations.datetime_conversion import datetime_to_gps_tow
from calculations.elevation_azimuth import calculate_elevation_azimuth
from meteorology.tropospheric_products import calculate_precipitable_water_vapor

c = 299792458.0 # Speed of light (m/s)

gps_frequency_plan = {
    'L1': 1575.42,  # MHz
    'L2': 1227.6,   # MHz
    'L5': 1176.45   # MHz
}

rinex_file = "../data/RINEX_data/ORMD/2025-11-25/ormd3290_excerpt.csv"
ephemeris = "../data/ephemerides/ephemeris_2025-11-25_RINEX.json"
results_dir = "../data/RINEX_data/ORMD/2025-11-25/RINEX_pwv_test"

if not os.path.exists(results_dir):
    os.makedirs(results_dir)

conn = sqlite3.connect('../data/RINEX_data/ORMD/2025-11-25/RINEX_pwv_test.db')

receiver_ecef = (860376.4154, -5499833.4036, 3102756.9385) # CORS ORMD

surface_temperature = 25.0 # [Celsius]
surface_pressure = 846.597166666675 # [hPa]

ah = 1.2451128509e-03
aw = 6.0843905260e-04
bh = 2.7074805444e-03
bw = 1.4082272735e-03
ch = 5.6279169098e-02
cw = 4.0289703115e-02

coeffs = (ah, aw, bh, bw, ch, cw)

rinex = pd.read_csv(rinex_file)
total_rows = len(rinex)
start_time = time.time()

gnss_results_lists = defaultdict(list)
for idx in range(len(rinex)):
    # Skip line if Pseudorange validity flag is not True
    # if rawx.iloc[_]['prValid'] != True:
    #    continue

    # ── progress bar ─────────────────────────────────────────
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

    # Assign data to variables
    pvn = rinex.iloc[idx]['sat_id']
    rcvDatetime = rinex.iloc[idx]['epoch']
    pseudorange_L1 = rinex.iloc[idx]['C1']
    pseudorange_L2 = rinex.iloc[idx]['C2']
    pseudorange_L5 = rinex.iloc[idx]['C5']

    if pd.notna(pseudorange_L1) and pd.notna(pseudorange_L2):
        pseudorange_combined_m = (pseudorange_L1 * (gps_frequency_plan['L1'] * 1e6) ** 2 - pseudorange_L2 *
                                  (gps_frequency_plan['L2'] * 1e6) ** 2) / ((gps_frequency_plan['L1'] * 1e6) ** 2 -
                                                                            (gps_frequency_plan['L2'] * 1e6) ** 2)
        freqId = 'L1_L2'

    elif pd.notna(pseudorange_L1) and pd.notna(pseudorange_L5):
        pseudorange_combined_m = (pseudorange_L1 * (gps_frequency_plan['L1'] * 1e6) ** 2 - pseudorange_L5 *
                                  (gps_frequency_plan['L5'] * 1e6) ** 2) / ((gps_frequency_plan['L1'] * 1e6) ** 2 -
                                                                            (gps_frequency_plan['L5'] * 1e6) ** 2)
        freqId = 'L1_L5'

    elif pd.notna(pseudorange_L1):
        pseudorange_combined_m = pseudorange_L1
        freqId = 'L1'

    else:
        # print("Missing pseudorange...")
        continue

    rcvTime_stripped = datetime.strptime(rcvDatetime, "%Y-%m-%d %H:%M:%S")
    rcvTow_s, gpsWeek = datetime_to_gps_tow(rcvTime_stripped)

    try:
        sat = load_ephemeris(ephemeris, pvn, gps_tow=rcvTow_s)
    except KeyError:
        # print("Missing ephemeris...")
        continue

    geometric_range_results = geometric_range.calculate_satellite_position_and_range(ephemeris, pvn, receiver_ecef,
                                                                                     gps_tow=rcvTow_s, gps_week=gpsWeek)

    # Assign results to variable
    geo_range_m = geometric_range_results['geometric_range_m']
    t_tx_s = geometric_range_results['transmission_time']
    sat_x = geometric_range_results['satellite_ecef_m']['x']
    sat_y = geometric_range_results['satellite_ecef_m']['y']
    sat_z = geometric_range_results['satellite_ecef_m']['z']

    sat_ecef = (sat_x, sat_y, sat_z)

    # Calculate satellite clock correction
    dt_sv_s = clock_correction.calculate_satellite_clock_offset(sat, t_tx_s)

    # Calculate relativistic clock correction
    dt_rel_s = clock_correction.calculate_relativistic_clock_correction(sat, t_tx_s)

    # Calculate total bias and total tropospheric delay
    residuals_m = pseudorange_combined_m - geo_range_m
    biases_m = (dt_sv_s - dt_rel_s) * c
    tropospheric_delay_m = residuals_m - biases_m

    elevation_deg, azimuth_deg = calculate_elevation_azimuth(sat_ecef, receiver_ecef)

    # Skip data below 10 deg elevation
    if elevation_deg < 10.0:
        continue

    pwv, ztd = calculate_precipitable_water_vapor(receiver_ecef, elevation_deg, tropospheric_delay_m, surface_temperature, surface_pressure, coeffs)

    pwv_mm = pwv * 1000
    ztd_mm = ztd * 1000

    gnss_results_lists[pvn].append({
        'svId': pvn,
        'rcvTOW': rcvTow_s,
        't_tx': t_tx_s,
        'WN': gpsWeek,
        'geoRange': geo_range_m,
        'elevation': elevation_deg,
        'azimuth': azimuth_deg,
        'satClockBias': dt_sv_s,
        'relBias': dt_rel_s,
        'residuals': residuals_m,
        'biases': biases_m,
        'pseudorange': pseudorange_combined_m,
        'troposphericDelay': tropospheric_delay_m,
        'pwv': pwv_mm,
        'ztd': ztd_mm
    })

sys.stdout.write('\n')
total_time = time.time() - start_time
print(f"Completed in {total_time:.1f} seconds")

gnss_results = {}
for pvn in gnss_results_lists:
    gnss_results[pvn] = pd.DataFrame(gnss_results_lists[pvn])
    table_name = f'{pvn}'
    gnss_results[pvn].to_sql(table_name, conn, if_exists='replace', index=False)