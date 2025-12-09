from collections import defaultdict
import matplotlib.pyplot as plt
import pandas as pd
import sqlite3

import software.scripts.geometric_range as geometric_range
import software.scripts.clock_correction as clock_correction
from software.scripts.ephemeris_classes import load_ephemeris
from software.scripts.elevation_azimuth import calculate_elevation_azimuth

print_all_plots = False

c = 299792458.0 # Speed of light (m/s)

rawx_file = "../../data/ubx_data/2025-11-25/GNSS001_RXM_RAWX.csv"
clock_file = "../../data/ubx_data/2025-11-25/GNSS001_NAV_CLOCK.csv"
ephemeris = "../ephemerides/ephemeris_2025-11-25_RINEX.json"
results_dir = "../../data/ubx_data/2025-11-25/GNSS001"

receiver_ecef = (867068.487, -5504812.066, 3092176.505) # Campus quad
# receiver_ecef = (867068.487, -5504812.066, 3092176.505) # Apartment

rawx = pd.read_csv(rawx_file)
clock = pd.read_csv(clock_file)

conn = sqlite3.connect('../../data/ubx_data/2025-11-25/GNSS001_10.db')

freqId = ''

gnss_results = defaultdict(lambda: defaultdict(list))
for _ in range(len(rawx)):
    # Skip line if Pseudorange validity flag is not True
    # if rawx.iloc[_]['prValid'] != True:
    #    continue

    if rawx.iloc[_]['gnssId'] != 0:
        continue

    # Assign data to variables
    gnssId = rawx.iloc[_]['gnssId']
    svId = rawx.iloc[_]['svId']
    sigId = rawx.iloc[_]['sigId']
    rcvTow_s = rawx.iloc[_]['rcvTow']
    gpsWeek = rawx.iloc[_]['week']
    pseudorange_m = rawx.iloc[_]['prMes']

    # Locate receiver bias and drift values
    closest_iTow = clock.iloc[(clock['iTOW'] / 1000.0 - rcvTow_s).abs().argsort()[:1]]
    iTow_ms = closest_iTow['iTOW'].values[0]
    clkBias_ns = closest_iTow['clkB'].values[0]
    clkDrift_ns = closest_iTow['clkD'].values[0]

    # rcvTOW and iTOW become out of sync when clkBias_ns is greater than 0.5 ms (rcvTOW rounds up b/c 3 decimals)
    # Fix by keeping rcvTOW the same until the clkBias_ns resets to zero
    if clkBias_ns >= 500000:
        rcvTow_s = rcvTow_s - 0.001

    iTOW_s = iTow_ms / 1000.0

    # Calculate receiver clock correction
    dt_iTow_s = rcvTow_s - (iTow_ms / 1000.0)
    dt_bias_s = (clkBias_ns + clkDrift_ns * dt_iTow_s) / 1e9

    gpsTow_s = rcvTow_s + (dt_iTow_s + dt_bias_s)

    # Create new dictionary
    if gnssId == 0:
        pvn = 'G' + str('{:02d}'.format(svId))
        if sigId == 0:
            freqId = 'L1 C'
        elif sigId == 3:
            freqId = 'L2 CL'
        elif sigId == 4:
            freqId = 'L2 CM'
        elif sigId == 6:
            freqId = 'L5 I'
        elif sigId == 7:
            freqId = 'L5 Q'
        else:
            continue
    elif gnssId == 2:
        pvn = 'E' + str('{:02d}'.format(svId))
        if sigId == 0:
            freqId = 'E1 C'
        elif sigId == 1:
            freqId = 'E1 B'
        elif sigId == 3:
            freqId = 'E5 al'
        elif sigId == 4:
            freqId = 'E5 aQ'
        elif sigId == 6:
            freqId = 'E5 bQ'
        elif sigId == 8:
            freqId = 'E6 B'
        elif sigId == 9:
            freqId = 'E6 C'
        elif sigId == 10:
            freqId = 'E6 A'
        else:
            continue
    else:
        continue # Skip if sat is not GPS or Galileo

    print("{}:\t{:.3f} s".format(pvn, gpsTow_s))

    try:
        sat = load_ephemeris(ephemeris, pvn, gps_tow=gpsTow_s)
    except KeyError:
        # print("Missing ephemeris...")
        continue
    except ValueError:
        print("No valid ephemeris...")
        continue

    geometric_range_results = geometric_range.calculate_satellite_position_and_range(ephemeris, pvn, receiver_ecef, gps_week=gpsWeek, gps_tow=gpsTow_s) # rcvTow includes receiver bias, so doesn't need to be accounted for below

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

    # Calculate Biases in meters
    rcv_clkBias_m = (dt_iTow_s + dt_bias_s) * c
    sat_clkBias_m = dt_sv_s * c
    relativistic_bias_m = dt_rel_s * c

    # Calculate total bias and total tropospheric delay
    biases_m = rcv_clkBias_m - sat_clkBias_m - relativistic_bias_m
    tropospheric_delay_m = pseudorange_m - geo_range_m - biases_m

    elevation, azimuth = calculate_elevation_azimuth(sat_ecef, receiver_ecef)

    gnss_results[pvn][freqId].append({
        'svId': pvn,
        'freqId': freqId,
        'gpsTOW': gpsTow_s,
        'rcvTOW': rcvTow_s,
        'iTOW': iTOW_s,
        'clkBias': clkBias_ns,
        'clkDrift': clkDrift_ns,
        't_tx': t_tx_s,
        'WN': gpsWeek,
        'geoRange': geo_range_m,
        'elevation': elevation,
        'azimuth': azimuth,
        'satClockBias': sat_clkBias_m,
        'rcvClockBias': rcv_clkBias_m,
        'relBias': relativistic_bias_m,
        'biases': biases_m,
        'pseudorange': pseudorange_m,
        'rangeDiff': pseudorange_m - geo_range_m,
        'troposphericDelay': tropospheric_delay_m
    })

gnss_results_dict = {}
for pvn in gnss_results:
    gnss_results_dict[pvn] = {}
    for freq in gnss_results[pvn]:
        gnss_results_dict[pvn][freq] = pd.DataFrame(gnss_results[pvn][freq])
        
for pvn in gnss_results_dict:
    for freq in gnss_results_dict[pvn]:
        table_name = f'{pvn}_{freq}'
        gnss_results_dict[pvn][freq].to_sql(table_name, conn, if_exists='replace', index=False)

if print_all_plots:
    for pvn in gnss_results_dict:
        plt.figure(figsize=(10, 6))
        if pvn.startswith('G') and 'L1 C' in gnss_results[pvn]:
            df = gnss_results_dict[pvn]['L1 C']
            plt.plot(df['gpsTOW'], df['troposphericDelay'], label=pvn + '_L1C')
            plt.title(f"Tropospheric Delay - {pvn + '_L1C'}")
        elif pvn.startswith('E') and 'E1 C' in gnss_results[pvn]:
            df = gnss_results_dict[pvn]['E1 C']
            plt.plot(df['gpsTOW'], df['troposphericDelay'], label=pvn + '_E1C')
            plt.title(f"Tropospheric Delay - {pvn + '_E1C'}")
        else:
            continue

        plt.xlabel("rcvTOW (s)")
        plt.ylabel("Tropospheric Delay (m)")
        plt.grid()
        plt.tight_layout()
        plt.savefig(f"{results_dir}/{pvn}.png")

for pvn in gnss_results_dict:
    if pvn.startswith('G') and 'L1 C' in gnss_results[pvn]:
        df = gnss_results_dict[pvn]['L1 C']
        plt.plot(df['gpsTOW'], df['troposphericDelay'], label=pvn+'_L1C')
    elif pvn.startswith('E') and 'E1 C' in gnss_results[pvn]:
        df = gnss_results_dict[pvn]['E1 C']
        plt.plot(df['gpsTOW'], df['troposphericDelay'], label=pvn+'_E1C')
    else:
        continue

plt.xlabel("rcvTOW (s)")
plt.ylabel("Tropospheric Delay (m)")
plt.title(f"Tropospheric Delay")
plt.legend()
plt.grid()
plt.tight_layout()
plt.savefig(f"{results_dir}/all_satellites.png")
plt.show()