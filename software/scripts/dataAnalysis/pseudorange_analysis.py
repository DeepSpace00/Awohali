import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

import software.scripts.geometric_range as geometric_range
import software.scripts.clock_correction as clock_correction
from software.scripts.ephemeris_classes import load_ephemeris, get_available_satellites

c = 299792458.0 # Speed of light (m/s)

keys = ['rcvTOW', 'geoRange', 'sat_clockBias', 'rcv_clockBias', 'relativisticBias', 'pseudorange', 'troposphericDelay']

rawx_file = "../../data/ubx_data/2025-11-25/GNSS001_RXM_RAWX_wo_timestamp.csv"
clock_file = "../../data/ubx_data/2025-11-25/GNSS001_NAV_CLOCK_wo_timestamp.csv"
ephemeris = "../ephemerides/ephemeris_2025-11-25.json"

receiver_ecef = (867068.487, -5504812.066, 3092176.505) # Campus quad
gps_week = 2394
sat_id = 'G32'

sats = get_available_satellites(ephemeris)
gps_sats = sats['GPS']
galileo_sats = sats['Galileo']
print(gps_sats)
print(galileo_sats)

rawx = pd.read_csv(rawx_file)
clock = pd.read_csv(clock_file)

gps_results = []
for pvn in gps_sats:
    sat_data = rawx[(rawx['gnssId'] == 0) & (rawx['svId'] == int(pvn.split('G')[1]))]
    sat_results = []

    for _ in range(len(sat_data)):
        rcvTow = sat_data.iloc[_]['rcvTow']
        # print(rcvTow)
        pseudorange = sat_data.iloc[_]['prMes']
        sat = load_ephemeris(ephemeris, sat_id, gps_tow=rcvTow)
        range_results = geometric_range.calculate_satellite_position_and_range(ephemeris, sat_id, receiver_ecef, gps_week=gps_week, gps_tow=rcvTow)
        geo_range = range_results['geometric_range_m']
        t_tx = range_results['transmission_time']

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

        biases = rcv_clkBias_m - sat_clkBias_m - relativistic_bias_m
        tropospheric_delay_m = pseudorange - geo_range - biases

        row = [rcvTow, geo_range, sat_clkBias_m, rcv_clkBias_m, relativistic_bias_m, pseudorange, tropospheric_delay_m]

        sat_results.append(row)

    if len(sat_results) > 0:
        df = pd.DataFrame(sat_results, columns=keys)
        gps_results.append([pvn, df])

gps_results_df = pd.DataFrame(gps_results, columns=['PVNs', 'Data'])

for pvn, df in gps_results:
    plt.figure(figsize=(10,6))
    plt.plot(df['rcvTOW'], df['troposphericDelay'], label=pvn)
    plt.xlabel("rcvTOW (s)")
    plt.ylabel("Tropospheric Delay (m)")
    plt.title(f"Tropospheric Delay - {pvn}")
    plt.grid()
    plt.tight_layout()

for pvn, df in gps_results:
    plt.plot(df['rcvTOW'], df['troposphericDelay'], label=pvn)

plt.xlabel("rcvTOW (s)")
plt.ylabel("Tropospheric Delay (m)")
plt.title(f"Tropospheric Delay")
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()