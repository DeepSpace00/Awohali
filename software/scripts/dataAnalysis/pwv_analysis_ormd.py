from collections import defaultdict
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
import pandas as pd
import sqlite3

import software.scripts.geometric_range as geometric_range
import software.scripts.clock_correction as clock_correction
from software.scripts.ephemeris_classes import load_ephemeris
from software.scripts.datetime_conversion import datetime_to_gps_tow

from software.scripts.elevation_azimuth import calculate_elevation_azimuth
from software.scripts.precipitable_water_vapor import calculate_precipitable_water_vapor


print_all_plots = False
print_sky_plot = False

c = 299792458.0 # Speed of light (m/s)

rinex_file = "../../data/RINEX_data/ORMD/2025-11-25/ormd3290_GPS_excerpt.csv"
ephemeris = "../ephemerides/ephemeris_2025-11-25_RINEX.json"
results_dir = "../../data/RINEX_data/ORMD/2025-11-25/results_pwv"

conn = sqlite3.connect('../../data/RINEX_data/ORMD/2025-11-25/pwv.db')

receiver_ecef = (860376.4154, -5499833.4036, 3102756.9385) # CORS ORMD

surface_temperature = 25.0 # [Celsius]
surface_pressure = 84659.7166666675 # [Pa]

ah = 0.00126969
aw = 0.00053219
bh = 0.00270650
bw = 0.00150246
ch = 0.05637877
cw = 0.04173512

coeffs = (ah, aw, bh, bw, ch, cw)

rinex = pd.read_csv(rinex_file)

gnss_results_lists = defaultdict(list)
for _ in range(len(rinex)):
    # Assign data to variables
    pvn = rinex.iloc[_]['sat_id']
    rcvDatetime = rinex.iloc[_]['epoch']
    pseudorange = rinex.iloc[_]['C1']

    rcvTime_stripped = datetime.strptime(rcvDatetime, "%Y-%m-%d %H:%M:%S.%f")
    rcvTow, gpsWeek = datetime_to_gps_tow(rcvTime_stripped)
    print("{}:\t{} s".format(pvn, rcvTow))

    try:
        sat = load_ephemeris(ephemeris, pvn, gps_tow=rcvTow)
    except KeyError:
        print("Missing ephemeris...")
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
    tropospheric_delay_m = pseudorange - geo_range + biases

    elevation, azimuth = calculate_elevation_azimuth(sat_ecef, receiver_ecef)

    pwv, ztd = calculate_precipitable_water_vapor(receiver_ecef, elevation, tropospheric_delay_m, surface_temperature, surface_pressure, coeffs)

    gnss_results_lists[pvn].append({
        'svId': pvn,
        'freqId': 'E1 C',
        'rcvTOW': rcvTow,
        't_tx': t_tx,
        'WN': gpsWeek,
        'geoRange': geo_range,
        'satClockBias': sat_clkBias_m,
        'relBias': relativistic_bias_m,
        'biases': biases,
        'pseudorange': pseudorange,
        'rangeDiff': pseudorange - geo_range,
        'troposphericDelay': tropospheric_delay_m,
        'elevation': elevation,
        'azimuth': azimuth,
        'pwv': pwv,
        'ztd': ztd
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
# plt.show()