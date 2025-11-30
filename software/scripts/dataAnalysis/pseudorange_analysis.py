from collections import defaultdict
import matplotlib.pyplot as plt
import pandas as pd
import sqlite3

import software.scripts.geometric_range as geometric_range
import software.scripts.clock_correction as clock_correction
from software.scripts.ephemeris_classes import load_ephemeris, get_available_satellites

print_all_plots = True

c = 299792458.0 # Speed of light (m/s)

rawx_file = "../../data/ubx_data/2025-11-25/2025-11-25_93138_serial-COM3_RXM_RAWX.csv"
clock_file = "../../data/ubx_data/2025-11-25/2025-11-25_93138_serial-COM3_NAV_CLOCK.csv"
ephemeris = "../ephemerides/ephemeris_2025-11-25.json"
results_dir = "../../data/ubx_data/2025-11-25/results4"

receiver_ecef = (867068.487, -5504812.066, 3092176.505) # Campus quad

# sats = get_available_satellites(ephemeris)

rawx = pd.read_csv(rawx_file)
clock = pd.read_csv(clock_file)

conn = sqlite3.connect('../../data/ubx_data/2025-11-25/GNSS001_3.db')

freqId = ''

gnss_results = defaultdict(lambda: defaultdict(list))
for _ in range(len(rawx)):
    # Skip line if Pseudorange validity flag is not True
    # if rawx.iloc[_]['prValid'] != True:
    #    continue

    if rawx.iloc[_]['gnssId'] != 0 | rawx.iloc[_]['gnssId'] != 1:
        continue

    # Assign data to variables
    gnssId = rawx.iloc[_]['gnssId']
    svId = rawx.iloc[_]['svId']
    sigId = rawx.iloc[_]['sigId']
    rcvTow = rawx.iloc[_]['rcvTow']
    gpsWeek = rawx.iloc[_]['week']
    pseudorange = rawx.iloc[_]['prMes']

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

    print("{}:\t{} s".format(pvn, rcvTow))

    try:
        sat = load_ephemeris(ephemeris, pvn, gps_tow=rcvTow)
    except KeyError:
        continue

    geometric_range_results = geometric_range.calculate_satellite_position_and_range(ephemeris, pvn, receiver_ecef, gps_week=gpsWeek, gps_tow=rcvTow)

    # Assign results to variable
    geo_range = geometric_range_results['geometric_range_m']
    t_tx = geometric_range_results['transmission_time']

    # Locate receiver bias and drift values
    closest_iTow = clock.iloc[(clock['iTOW'] - rcvTow).abs().argsort()[:1]]
    iTow = closest_iTow['iTOW'].values[0]
    clkBias = closest_iTow['clkB'].values[0]
    clkDrift = closest_iTow['clkD'].values[0]

    # Calculate receiver clock correction
    dt = rcvTow - (iTow / 1000.0)
    dt_rcv = (clkBias + clkDrift * dt) / 1e9

    # Calculate satellite clock correction
    dt_sv = clock_correction.calculate_satellite_clock_offset(sat, t_tx)

    # Calculate relativistic clock correction
    dt_rel = clock_correction.calculate_relativistic_clock_correction(sat, t_tx)

    # Calculate Biases in meters
    rcv_clkBias_m = dt_rcv * c
    sat_clkBias_m = dt_sv * c
    relativistic_bias_m = dt_rel * c

    biases = rcv_clkBias_m + sat_clkBias_m + relativistic_bias_m #+ (2.69671938470657*rcvTow)
    tropospheric_delay_m = pseudorange - geo_range - biases

    gnss_results[pvn][freqId].append({
        'svId': pvn,
        'freqId': freqId,
        'rcvTOW': rcvTow,
        'WN': gpsWeek,
        'geoRange': geo_range,
        'satClockBias': sat_clkBias_m,
        'rcvClockBias': rcv_clkBias_m,
        'relBias': relativistic_bias_m,
        'biases': biases,
        'pseudorange': pseudorange,
        'rangeDiff': pseudorange - geo_range,
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
            plt.plot(df['rcvTOW'], df['troposphericDelay'], label=pvn + '_L1C')
            plt.title(f"Tropospheric Delay - {pvn + '_L1C'}")
        elif pvn.startswith('E') and 'E1 C' in gnss_results[pvn]:
            df = gnss_results_dict[pvn]['E1 C']
            plt.plot(df['rcvTOW'], df['troposphericDelay'], label=pvn + '_E1C')
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
        plt.plot(df['rcvTOW'], df['troposphericDelay'], label=pvn+'_L1C')
    elif pvn.startswith('E') and 'E1 C' in gnss_results[pvn]:
        df = gnss_results_dict[pvn]['E1 C']
        plt.plot(df['rcvTOW'], df['troposphericDelay'], label=pvn+'_E1C')
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