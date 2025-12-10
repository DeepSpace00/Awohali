from collections import defaultdict
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
import os
import pandas as pd
import sqlite3

import software.scripts.clock_correction as clock_correction
from software.scripts.datetime_conversion import datetime_to_gps_tow
from software.scripts.ephemeris_classes import load_ephemeris
from software.scripts.elevation_azimuth import calculate_elevation_azimuth
import software.scripts.geometric_range as geometric_range

print_all_plots = True
print_sky_plot = True

c = 299792458.0 # Speed of light (m/s)

gps_frequency_plan = {
    'L1': 1575.42,  # MHz
    'L2': 1227.6,   # MHz
    'L5': 1176.45   # MHz
}

rinex_file = "../../data/RINEX_data/ORMD/2025-11-25/ormd3290_GPS_excerpt.csv"
ephemeris = "../ephemerides/ephemeris_2025-11-25_RINEX.json"
results_dir = "../../data/RINEX_data/ORMD/2025-11-25/results_combined_pseudorange"

if not os.path.exists(results_dir):
    os.makedirs(results_dir)

conn = sqlite3.connect('../../data/RINEX_data/ORMD/2025-11-25/ormd3290_GPS_combined_pseudorange.db')

receiver_ecef = (860376.4154, -5499833.4036, 3102756.9385) # CORS ORMD

rinex = pd.read_csv(rinex_file)

gnss_results_lists = defaultdict(list)
for _ in range(len(rinex)):
    # Assign data to variables
    pvn = rinex.iloc[_]['sat_id']
    rcvDatetime = rinex.iloc[_]['epoch']
    pseudorange_L1 = rinex.iloc[_]['C1']
    pseudorange_L2 = rinex.iloc[_]['C2']
    pseudorange_L5 = rinex.iloc[_]['C5']

    if pseudorange_L1 is not None and pseudorange_L2 is not None:
        pseudorange_combined_m =    (pseudorange_L1 * (gps_frequency_plan['L1'] * 1e6)**2 - pseudorange_L2 *
                                    (gps_frequency_plan['L2'] * 1e6)**2) / ((gps_frequency_plan['L1'] * 1e6)**2 -
                                    (gps_frequency_plan['L2'] * 1e6)**2)
        freqId = 'L1_L2'

    elif pseudorange_L1 is not None and pseudorange_L5 is not None:
        pseudorange_combined_m =    (pseudorange_L1 * (gps_frequency_plan['L1'] * 1e6) ** 2 - pseudorange_L5 *
                                    (gps_frequency_plan['L5'] * 1e6) ** 2) / ((gps_frequency_plan['L1'] * 1e6) ** 2 -
                                    (gps_frequency_plan['L5'] * 1e6) ** 2)
        freqId = 'L1_L5'

    elif pseudorange_L1 is not None:
        pseudorange_combined_m =    pseudorange_L1
        freqId = 'L1'

    else:
        print("Missing pseudorange...")
        continue

    rcvTime_stripped = datetime.strptime(rcvDatetime, "%Y-%m-%d %H:%M:%S.%f")
    rcvTow_s, gpsWeek = datetime_to_gps_tow(rcvTime_stripped)
    print("{}:\t{} s".format(pvn, rcvTow_s))

    gpsTow_s = rcvTow_s

    try:
        sat = load_ephemeris(ephemeris, pvn, gps_tow=rcvTow_s)
    except KeyError:
        print("Missing ephemeris...")
        continue

    geometric_range_results = geometric_range.calculate_satellite_position_and_range(ephemeris, pvn, receiver_ecef, gps_tow=rcvTow_s, gps_week=gpsWeek)

    # Assign results to variable
    geo_range_m = geometric_range_results['geometric_range_m']
    t_tx_s = geometric_range_results['transmission_time']
    sat_x = geometric_range_results['satellite_ecef_m']['x']
    sat_y = geometric_range_results['satellite_ecef_m']['y']
    sat_z = geometric_range_results['satellite_ecef_m']['z']

    sat_ecef = (sat_x, sat_y, sat_z)

    # Calculate satellite clock correction
    dt_sv = clock_correction.calculate_satellite_clock_offset(sat, t_tx_s)

    # Calculate relativistic clock correction
    dt_rel = clock_correction.calculate_relativistic_clock_correction(sat, t_tx_s)

    # Calculate Biases in meters
    sat_clkBias_m = dt_sv * c
    relativistic_bias_m = dt_rel * c

    biases_m = sat_clkBias_m + relativistic_bias_m
    tropospheric_delay_m = pseudorange_combined_m - geo_range_m + biases_m

    elevation_deg, azimuth_deg = calculate_elevation_azimuth(sat_ecef, receiver_ecef)

    gnss_results_lists[pvn].append({
        'svId': pvn,
        'freqId': freqId,
        'gpsTOW': gpsTow_s,
        'rcvTOW': rcvTow_s,
        't_tx': t_tx_s,
        'WN': gpsWeek,
        'geoRange': geo_range_m,
        'elevation': elevation_deg,
        'azimuth': azimuth_deg,
        'satClockBias': sat_clkBias_m,
        'relBias': relativistic_bias_m,
        'biases': biases_m,
        'pseudorange': pseudorange_combined_m,
        'rangeDiff': pseudorange_combined_m - geo_range_m,
        'troposphericDelay': tropospheric_delay_m
    })

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
        azimuth = df['azimuth'].to_numpy()
        elevation = df['elevation'].to_numpy()
        theta = np.radians(azimuth)
        # print(theta)
        r = 90 - elevation
        # print(r)
        lines = ax.plot(theta, r, marker='.', markersize=2, linewidth=1, label=pvn, alpha=0.7)
        color = lines[0].get_color()
        ax.plot(theta[0], r[0], 'o', markersize=8, color=color, alpha=0.8)
        ax.plot(theta[-1], r[-1], 's', markersize=6, color=color, alpha=0.8)

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
    ax.legend(bbox_to_anchor=(1.15, 1.0), loc='upper left', fontsize=9)

    plt.tight_layout()

if print_all_plots:
    for pvn in gnss_results:
        df = gnss_results[pvn]
        plt.figure(figsize=(10,6))
        plt.plot(df['rcvTOW'], df['troposphericDelay'], label=pvn)
        plt.xlabel("rcvTOW (s)")
        plt.ylabel("Tropospheric Delay (m)")
        plt.title(f"Tropospheric Delay - {pvn}")
        plt.grid()
        plt.tight_layout()

        plt.savefig(f"{results_dir}/{pvn}.png")

for pvn in gnss_results:
    df = gnss_results[pvn]
    plt.plot(df['rcvTOW'], df['troposphericDelay'], label=pvn)

plt.xlabel("rcvTOW (s)")
plt.ylabel("Tropospheric Delay (m)")
plt.title(f"Tropospheric Delay")
plt.legend()
plt.grid()
plt.tight_layout()
plt.savefig(f"{results_dir}/all_gps_satellites.png")
plt.show()