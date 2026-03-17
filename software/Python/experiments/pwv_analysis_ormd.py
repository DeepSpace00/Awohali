from collections import defaultdict
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
import os
import pandas as pd
import sqlite3
import sys
import time

from calculations import clock_correction, geometric_range
from ephemerides.ephemeris import load_ephemeris
from calculations.datetime_conversion import datetime_to_gps_tow
from calculations.elevation_azimuth import calculate_elevation_azimuth
from tropospheric_products.precipitable_water_vapor import calculate_precipitable_water_vapor
from calculations.plot_satellite_angles import plot_elevation_azimuth_combined

from software.Python.src.calculations.datetime_conversion import datetime_to_gps_tow_precise

print_all_plots = False
print_sky_plot = False

c = 299792458.0 # Speed of light (m/s)

gps_frequency_plan = {
    'L1': 1575.42,  # MHz
    'L2': 1227.6,   # MHz
    'L5': 1176.45   # MHz
}

rinex_file = "../data/RINEX_data/ORMD/2025-11-25/ormd3290_GPS_excerpt.csv"
ephemeris = "../data/ephemerides/ephemeris_2025-11-25_RINEX.json"
# ephemeris = "../data/ephemerides/ephemeris_2025-11-25.json"
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

i = 0
total_measurements = len(rinex)
start_time = time.time()

gnss_results_lists = defaultdict(list)
for _ in range(len(rinex)):
    # Skip line if Pseudorange validity flag is not True
    # if rawx.iloc[_]['prValid'] != True:
    #    continue

    progress = (i + 1) / total_measurements
    elapsed_time = time.time() - start_time

    if i > 0:
        avg_time_per_item = elapsed_time / (i + 1)
        remaining_items = total_measurements - (i + 1)
        eta_seconds = avg_time_per_item * remaining_items

        # Format ETA
        if eta_seconds < 60:
            eta_str = f"{eta_seconds:.0f}s"
        elif eta_seconds < 3600:
            eta_str = f"{eta_seconds / 60:.1f}m"
        else:
            eta_str = f"{eta_seconds / 3600:.1f}h"
    else:
        eta_str = "calculating..."

    # Print progress bar
    bar_length = 40
    filled_length = int(bar_length * progress)
    bar = '█' * filled_length + '-' * (bar_length - filled_length)

    sys.stdout.write(f'\rProcessing GNSS Data: |{bar}| {progress * 100:.1f}% ({i + 1}/{total_measurements}) ETA:'
                     f' {eta_str}')
    sys.stdout.flush()

    # Assign data to variables
    pvn = rinex.iloc[_]['sat_id']
    rcvDatetime = rinex.iloc[_]['epoch']
    pseudorange_L1 = rinex.iloc[_]['C1']
    pseudorange_L2 = rinex.iloc[_]['C2']
    pseudorange_L5 = rinex.iloc[_]['C5']

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
        i += 1
        continue

    rcvTime_stripped = datetime.strptime(rcvDatetime, "%Y-%m-%d %H:%M:%S.%f")
    rcvTow, gpsWeek = datetime_to_gps_tow(rcvTime_stripped)
    # print("{}:\t{} s".format(pvn, rcvTow))

    try:
        sat = load_ephemeris(ephemeris, pvn, gps_tow=rcvTow)
    except KeyError:
        # print("Missing ephemeris...")
        i += 1
        continue

    geometric_range_results = geometric_range.calculate_satellite_position_and_range(ephemeris, pvn, receiver_ecef, gps_tow=rcvTow, gps_week=gpsWeek)

    # Assign results to variable
    geo_range = geometric_range_results['geometric_range_m']
    t_tx = geometric_range_results['transmission_time']
    sat_x = geometric_range_results['satellite_ecef_m']['x']
    sat_y = geometric_range_results['satellite_ecef_m']['y']
    sat_z = geometric_range_results['satellite_ecef_m']['z']

    sat_ecef = (sat_x, sat_y, sat_z)

    # Calculate satellite clock correction
    dt_sv = clock_correction.calculate_satellite_clock_offset(sat, t_tx)

    # Calculate relativistic clock correction
    dt_rel = clock_correction.calculate_relativistic_clock_correction(sat, t_tx)

    # Calculate Biases in meters
    sat_clkBias_m = dt_sv * c
    relativistic_bias_m = dt_rel * c

    biases = sat_clkBias_m + relativistic_bias_m
    tropospheric_delay_m = pseudorange_combined_m - geo_range + biases

    elevation, azimuth = calculate_elevation_azimuth(sat_ecef, receiver_ecef)

    # Skip data below 10 deg elevation
    if elevation < 10.0:
        i += 1
        continue

    pwv, ztd = calculate_precipitable_water_vapor(receiver_ecef, elevation, tropospheric_delay_m, surface_temperature, surface_pressure, coeffs)

    pwv_mm = pwv * 1000
    ztd_mm = ztd * 1000

    iwv = pwv_mm

    gnss_results_lists[pvn].append({
        'svId': pvn,
        'rcvTOW': rcvTow,
        't_tx': t_tx,
        'WN': gpsWeek,
        'geoRange': geo_range,
        'elevation': elevation,
        'azimuth': azimuth,
        'satClockBias': sat_clkBias_m,
        'relBias': relativistic_bias_m,
        'biases': biases,
        'pseudorange': pseudorange_combined_m,
        'troposphericDelay': tropospheric_delay_m,
        'pwv': pwv_mm,
        'ztd': ztd_mm
    })

    i += 1

sys.stdout.write('\n')
total_time = time.time() - start_time
print(f"Completed in {total_time:.1f} seconds")

gnss_results = {}
for pvn in gnss_results_lists:
    gnss_results[pvn] = pd.DataFrame(gnss_results_lists[pvn])
    table_name = f'{pvn}'
    gnss_results[pvn].to_sql(table_name, conn, if_exists='replace', index=False)


if print_sky_plot:
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='polar')
    ax.set_theta_zero_location('N')  # 0° at top (North)
    ax.set_theta_direction(-1)  # Clockwise
    for pvn in gnss_results:
        df = gnss_results[pvn]
        if df.empty:
            continue
        azimuth = df['azimuth'].to_numpy()
        elevation = df['elevation'].to_numpy()
        theta = np.radians(azimuth)
        # print(theta)
        r = 90 - elevation
        # print(r)
        lines = ax.plot(theta, r, marker='.', markersize=2, linewidth=1, label=pvn, alpha=0.7)
        for line in lines[1:]:  # suppress legend for any extra segments
            line.set_label('_nolegend_')
        color = lines[0].get_color()
        ax.plot(theta[0], r[0], 'o', markersize=8, color=color, alpha=0.8, label='_nolegend_')
        ax.plot(theta[-1], r[-1], 's', markersize=6, color=color, alpha=0.8, label='_nolegend_')

    ax.set_ylim(0, 90)
    ax.set_yticks([0, 15, 30, 45, 60, 75, 90])
    for elev in [30, 60]:
        circle = Circle((0, 0), 90 - elev, transform=ax.transData._b,
                        fill=False, edgecolor='gray', linewidth=1,
                        linestyle='--', alpha=0.3)
        ax.add_patch(circle)

    # Add cardinal direction labels
    ax.set_xticks(np.radians([0, 45, 90, 135, 180, 225, 270, 315]))
    ax.set_xticklabels(['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'])

    ax.set_title('Satellite Sky Plot',
                 fontsize=14, fontweight='bold', pad=20)
    for handle, label in zip(*ax.get_legend_handles_labels()):
        print(label)
    ax.legend(bbox_to_anchor=(1.15, 1.0), loc='upper left', fontsize=9)

    plt.tight_layout()

    plt.savefig(f"{results_dir}/el_az_gps_satellites.png")

if print_all_plots:
    for pvn in gnss_results:
        df = gnss_results[pvn]
        plt.figure(figsize=(10,6))
        plt.plot(df['rcvTOW'], df['pwv'], label=pvn)
        plt.xlabel("rcvTOW (s)")
        plt.ylabel("PWV (mm)")
        plt.title(f"Precipitable Water Vapor (PWV) - {pvn}")
        plt.grid()
        plt.tight_layout()

        plt.savefig(f"{results_dir}/{pvn}_pwv.png")

    for pvn in gnss_results:
        df = gnss_results[pvn]
        plt.plot(df['rcvTOW'], df['pwv'], label=pvn)

    plt.xlabel("rcvTOW (s)")
    plt.ylabel("ZTD (mm)")
    plt.title(f"Precipitable Water Vapor (PWV)")
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.savefig(f"{results_dir}/pwv_all_gps_satellites.png")

plt.show()