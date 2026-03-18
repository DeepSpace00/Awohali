from collections import defaultdict
import matplotlib.pyplot as plt
import os
import pandas as pd
import sqlite3
import sys
import time

from calculations import clock_correction, geometric_range
from ephemerides.ephemeris import load_ephemeris
from calculations.elevation_azimuth import calculate_elevation_azimuth

from software.Python.src.tropospheric_products.precipitable_water_vapor import calculate_precipitable_water_vapor

print_all_plots = False

c = 299792458.0  # Speed of light (m/s)

rawx_file = "../data/ubx_data/2025-11-25/2025-11-25_93138_serial-COM3_RXM_RAWX.csv"
clock_file = "../data/ubx_data/2025-11-25/2025-11-25_93138_serial-COM3_NAV_CLOCK.csv"
ephemeris = "../data/ephemerides/ephemeris_2025-11-25_RINEX.json"
# ephemeris = "../data/ephemerides/ephemeris_2025-11-25.json"
results_dir = "../data/ubx_data/2025-11-25/2025-11-25_serial-COM3_pwv_testing"

if not os.path.exists(results_dir):
    os.makedirs(results_dir)

receiver_ecef = (867068.487, -5504812.066, 3092176.505)  # Campus quad
# receiver_ecef = (867068.487, -5504812.066, 3092176.505) # Apartment

surface_temperature = 25.0 # [Celsius]
surface_pressure = 846.597166666675 # [hPa]

ah = 1.2451128509e-03
aw = 6.0843905260e-04
bh = 2.7074805444e-03
bw = 1.4082272735e-03
ch = 5.6279169098e-02
cw = 4.0289703115e-02

coeffs = (ah, aw, bh, bw, ch, cw)

rawx = pd.read_csv(rawx_file)
clock = pd.read_csv(clock_file)

rawx_length = len(rawx)

conn = sqlite3.connect('../data/ubx_data/2025-11-25/2025-11-25_serial-COM3_pwv_testing.db')
signal_plan = {
    0: {
        0:  {'service': 'L1_C/A',   'freq': 1575.42},   # L1 Civilian signal [Mhz]
        3:  {'service': 'L2_CL',    'freq': 1227.60},   # L2 Pilot signal
        4:  {'service': 'L2_CM',    'freq': 1227.60},   # L2 Data signal
        6:  {'service': 'L5_I',     'freq': 1176.45},   # L5 Data signal
        7:  {'service': 'L5_Q',     'freq': 1176.45}    # L5 Pilot signal
    },
    2: {
        0:  {'service': 'E1_C',     'freq': 1575.42},   # E1 Pilot signal [MHz]
        1:  {'service': 'E1_B',     'freq': 1575.42},   # E1 Data signal
        3:  {'service': 'E5_aI',    'freq': 1176.45},   # E5a Data signal
        4:  {'service': 'E5_aQ',    'freq': 1176.45},   # E5a Pilot signal
        5:  {'service': 'E5_bI',    'freq': 1207.14},   # E5b Data signal
        6:  {'service': 'E5_bQ',    'freq': 1207.14},   # E5b Pilot signal
        8:  {'service': 'E6_B',     'freq': 1278.75},   # E6 Data signal
        9:  {'service': 'E6_C',     'freq': 1278.75},   # E6 Pilot signal
        10: {'service': 'E6_A',     'freq': 1278.75}    # E6 PRS signal
    }
}

current_epoch = rawx.iloc[0]['rcvTow']
i = 0

total_measurements = len(rawx)
start_time = time.time()

gnss_results = defaultdict(list)
while True:

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

    numMeas = rawx.iloc[i]['numMeas']

    epoch_data = rawx.loc[i:(i + numMeas - 1), [
        'rcvTow', 'week', 'prMes', 'cpMes', 'doMes', 'gnssId', 'svId', 'sigId', 'locktime', 'cno', 'prStdev', 'cpStdev', 'prValid', 'cpValid', 'halfCyc', 'subHalfCyc']]

    # print(epoch_data)

    rcvTow_s = epoch_data['rcvTow'].values[0]
    gpsWeek = epoch_data['week'].values[0]

    gnssIDs = epoch_data['gnssId'].unique()

    for constellation in gnssIDs:
        epoch_constellation_data = pd.DataFrame(epoch_data[epoch_data['gnssId'] == constellation])

        svIDs = epoch_constellation_data['svId'].unique()

        # satellite_dict = defaultdict(lambda: defaultdict(list))
        for satellite in svIDs:
            if constellation == 0:
                pvn = 'G' + str('{:02d}'.format(satellite))
                satellite_data = pd.DataFrame(epoch_constellation_data[epoch_constellation_data['svId'] == satellite])
            elif constellation == 2:
                pvn = 'E' + str('{:02d}'.format(satellite))
                satellite_data = pd.DataFrame(epoch_constellation_data[epoch_constellation_data['svId'] == satellite])
            else:
                print('Invalid satellite')
                continue

            sigIDs = satellite_data['sigId'].unique()

            services_list = []
            for signal in sigIDs:
                services_list.append(signal_plan[constellation][signal]['service'])

            services = '&'.join(services_list)

            if len(sigIDs) > 1:
                # Build list of (freq, pseudorange, carrierPhase) for all signals
                freq_sig_pairs = []
                for sig in sigIDs:
                    freq = float(signal_plan[constellation][sig]['freq'])
                    row = satellite_data[satellite_data['sigId'] == sig].iloc[0]
                    freq_sig_pairs.append((freq, float(row['prMes']), float(row['cpMes'])))

                # Sort by frequency
                freq_sig_pairs.sort(key=lambda x: x[0])

                freq_low, pr_low, cp_low = freq_sig_pairs[0]
                freq_high, pr_high, cp_high = freq_sig_pairs[-1]

                if freq_low == freq_high:
                    # All signals on same frequency, fall back to single freq
                    pseudorange_combined_m = pr_high
                    carrierPhase_combined = cp_high
                else:
                    f1 = freq_high * 1e6
                    f2 = freq_low * 1e6
                    pseudorange_combined_m = (pr_high * f1 ** 2 - pr_low * f2 ** 2) / (f1 ** 2 - f2 ** 2)
                    carrierPhase_combined = (cp_high * f1 ** 2 - cp_low * f2 ** 2) / (f1 ** 2 - f2 ** 2)
            else:
                pseudorange_combined_m = float(satellite_data.iloc[0]['prMes'])
                carrierPhase_combined = float(satellite_data.iloc[0]['cpMes'])

            # Locate receiver bias and drift values
            closest_iTow = clock.iloc[(clock['iTOW'] / 1000.0 - rcvTow_s).abs().argsort()[:1]]
            iTow_ms = closest_iTow['iTOW'].values[0]
            clkBias_ns = closest_iTow['clkB'].values[0]
            clkDrift_ns = closest_iTow['clkD'].values[0]
            tAcc_ns = closest_iTow['tAcc'].values[0]
            fAcc = closest_iTow['fAcc'].values[0]

            # rcvTOW and iTOW become out of sync when clkBias_ns is greater than 0.5 ms (rcvTOW rounds up b/c 3 decimals)
            # Fix by keeping rcvTOW the same until the clkBias_ns resets to zero

            if clkBias_ns >= 500000:
                rcvTow_sat_s = rcvTow_s - 0.001
            elif clkBias_ns <= -500000:
                rcvTow_sat_s = rcvTow_s + 0.001
            else:
                rcvTow_sat_s = rcvTow_s

            iTOW_s = iTow_ms / 1000.0

            # Calculate receiver clock correction
            dt_iTow_s = rcvTow_sat_s - (iTow_ms / 1000.0)
            dt_bias_s = (clkBias_ns + clkDrift_ns * dt_iTow_s) / 1e9

            rcv_clkBias_s = dt_iTow_s + dt_bias_s

            gpsTow_s = rcvTow_sat_s - rcv_clkBias_s

            # print("{}:\t{:.3f} s".format(pvn, gpsTow_s))

            try:
                sat = load_ephemeris(ephemeris, pvn, gps_tow=gpsTow_s)
            except KeyError:
                # print("Missing ephemeris...")
                continue
            except ValueError:
                print("No valid ephemeris...")
                continue

            geometric_range_results = geometric_range.calculate_satellite_position_and_range(ephemeris, pvn,
                                                                                             receiver_ecef,
                                                                                             gps_week=gpsWeek,
                                                                                             gps_tow=gpsTow_s)  # rcvTow includes receiver bias, so doesn't need to be accounted for below

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
            biases_m = (rcv_clkBias_s - dt_sv_s - dt_rel_s) * c
            tropospheric_delay_m = residuals_m - biases_m

            elevation_deg, azimuth_deg = calculate_elevation_azimuth(sat_ecef, receiver_ecef)

            pwv, ztd = calculate_precipitable_water_vapor(receiver_ecef, elevation_deg, tropospheric_delay_m,
                                                          surface_temperature, surface_pressure, coeffs)

            pwv_mm = pwv * 1000.0
            ztd_mm = ztd * 1000.0

            gnss_results[pvn].append({
                'svId': pvn,
                'gpsTOW': gpsTow_s,
                'rcvTOW': rcvTow_s,
                'rcvTOW_sat': rcvTow_sat_s,
                'clkBias': clkBias_ns,
                'clkDrift': clkDrift_ns,
                'iTOW': iTOW_s,
                't_tx': t_tx_s,
                'WN': gpsWeek,
                'geoRange': geo_range_m,
                'elevation': elevation_deg,
                'azimuth': azimuth_deg,
                'rcvClockBias': rcv_clkBias_s,
                'satClockBias': dt_sv_s,
                'relTimeBias': dt_rel_s,
                'residuals': residuals_m,
                'biases': biases_m,
                'pseudorange': pseudorange_combined_m,
                'carrierPhase': carrierPhase_combined,
                'troposphericDelay': tropospheric_delay_m,
                'pwv': pwv_mm,
                'ztd': ztd_mm
            })

    i += numMeas

    if i >= rawx_length:
        break

sys.stdout.write('\n')
total_time = time.time() - start_time
print(f"Completed in {total_time:.1f} seconds")

final_results = {}
for pvn in gnss_results:
    final_results[pvn] = pd.DataFrame(gnss_results[pvn])
    table_name = f'{pvn}'
    final_results[pvn].to_sql(table_name, conn, if_exists='replace', index=False)

if print_all_plots:
    for pvn in final_results:
        df = final_results[pvn]
        plt.figure(figsize=(10,6))
        plt.plot(df['rcvTOW'], df['troposphericDelay'], label=pvn)
        plt.xlabel("rcvTOW (s)")
        plt.ylabel("Tropospheric Delay (m)")
        plt.title(f"Tropospheric Delay - {pvn}")
        plt.grid()
        plt.tight_layout()

        plt.savefig(f"{results_dir}/{pvn}.png")

for pvn in final_results:
    df = final_results[pvn]
    plt.plot(df['rcvTOW'], df['troposphericDelay'], label=pvn)

plt.xlabel("rcvTOW (s)")
plt.ylabel("Tropospheric Delay (m)")
plt.title(f"Tropospheric Delay")
plt.legend()
plt.grid()
plt.tight_layout()
plt.savefig(f"{results_dir}/all_satellites.png")
plt.show()
