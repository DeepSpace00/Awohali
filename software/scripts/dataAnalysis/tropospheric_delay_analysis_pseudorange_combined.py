from collections import defaultdict
import matplotlib.pyplot as plt
import os
import pandas as pd
import sqlite3
import software.scripts.clock_correction as clock_correction
from software.scripts.elevation_azimuth import calculate_elevation_azimuth
from software.scripts.ephemeris_classes import load_ephemeris
import software.scripts.geometric_range as geometric_range

print_all_plots = False

c = 299792458.0  # Speed of light (m/s)

rawx_file = "../../data/ubx_data/2025-11-25/2025-11-25_93138_serial-COM3_RXM_RAWX.csv"
clock_file = "../../data/ubx_data/2025-11-25/2025-11-25_93138_serial-COM3_NAV_CLOCK.csv"
ephemeris = "../ephemerides/ephemeris_2025-11-25_RINEX.json"
results_dir = "../../data/ubx_data/2025-11-25/2025-11-25_93138_serial-COM3_pseudorange_combined"

if not os.path.exists(results_dir):
    os.makedirs(results_dir)

receiver_ecef = (867068.487, -5504812.066, 3092176.505)  # Campus quad
# receiver_ecef = (867068.487, -5504812.066, 3092176.505) # Apartment

rawx = pd.read_csv(rawx_file)
clock = pd.read_csv(clock_file)

rawx_length = len(rawx)

conn = sqlite3.connect('../../data/ubx_data/2025-11-25/2025-11-25_93138_serial-COM3_pseudorange_combined.db')

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

gnss_results = defaultdict(list)
while True:
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
                signalA_data = satellite_data.iloc[0][:]
                signalB_data = satellite_data.iloc[1][:]

                freqA = float(signal_plan[constellation][sigIDs[0]]['freq'])
                freqB = float(signal_plan[constellation][sigIDs[1]]['freq'])

                if freqA > freqB:
                    freq_1 = freqA
                    freq_2 = freqB
                    pseudorange_1 = float(signalA_data['prMes'])
                    pseudorange_2 = float(signalB_data['prMes'])
                    carrierPhase_1 = float(signalA_data['cpMes'])
                    carrierPhase_2 = float(signalB_data['cpMes'])

                else:
                    freq_1 = freqB
                    freq_2 = freqA
                    pseudorange_1 = float(signalB_data['prMes'])
                    pseudorange_2 = float(signalA_data['prMes'])
                    carrierPhase_1 = float(signalB_data['cpMes'])
                    carrierPhase_2 = float(signalA_data['cpMes'])

                pseudorange_combined_m = ((pseudorange_1 * (freq_1 * 1e6) ** 2 - pseudorange_2 * (freq_2 * 1e6) ** 2) /
                                          ((freq_1 * 1e6) ** 2 - (freq_2 * 1e6) ** 2))

                carrierPhase_combined = ((carrierPhase_1 * (freq_1 * 1e6) ** 2 - carrierPhase_2 * (freq_2 * 1e6) ** 2) /
                                          ((freq_1 * 1e6) ** 2 - (freq_2 * 1e6) ** 2))

            else:
                pseudorange_combined_m = satellite_data.iloc[0]['prMes']
                carrierPhase_combined = satellite_data[:]['cpMes']

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
                rcvTow_s = rcvTow_s - 0.001

            iTOW_s = iTow_ms / 1000.0

            # Calculate receiver clock correction
            dt_iTow_s = rcvTow_s - (iTow_ms / 1000.0)
            dt_bias_s = (clkBias_ns + clkDrift_ns * dt_iTow_s) / 1e9

            rcv_clkBias_s = dt_iTow_s + dt_bias_s

            gpsTow_s = rcvTow_s + rcv_clkBias_s

            print("{}:\t{:.3f} s".format(pvn, gpsTow_s))

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
            biases_s = rcv_clkBias_s - dt_sv_s - dt_rel_s
            tropospheric_delay_m = residuals_m - biases_s * c

            elevation_deg, azimuth_deg = calculate_elevation_azimuth(sat_ecef, receiver_ecef)

            gnss_results[pvn].append({
                'svId': pvn,
                'gpsTOW': gpsTow_s,
                'WN': gpsWeek,
                'rcvTOW': rcvTow_s,
                't_tx': t_tx_s,
                'geoRange': geo_range_m,
                'el': elevation_deg,
                'az': azimuth_deg,
                'rcvClockBias': rcv_clkBias_s,
                'satClockBias': dt_sv_s,
                'relTimeBias': dt_rel_s,
                'biases': biases_s,
                'pseudorange': pseudorange_combined_m,
                'carrierPhase': carrierPhase_combined,
                'residuals': residuals_m,
                'tropoDelay': tropospheric_delay_m
            })

    i += numMeas

    if i >= rawx_length:
        break

final_results = {}
for pvn in gnss_results:
    final_results[pvn] = pd.DataFrame(gnss_results[pvn])
    table_name = f'{pvn}'
    final_results[pvn].to_sql(table_name, conn, if_exists='replace', index=False)

if print_all_plots:
    for pvn in final_results:
        df = final_results[pvn]
        plt.figure(figsize=(10,6))
        plt.plot(df['rcvTOW'], df['tropoDelay'], label=pvn)
        plt.xlabel("rcvTOW (s)")
        plt.ylabel("Tropospheric Delay (m)")
        plt.title(f"Tropospheric Delay - {pvn}")
        plt.grid()
        plt.tight_layout()

        plt.savefig(f"{results_dir}/{pvn}.png")

for pvn in final_results:
    df = final_results[pvn]
    plt.plot(df['rcvTOW'], df['tropoDelay'], label=pvn)

plt.xlabel("rcvTOW (s)")
plt.ylabel("Tropospheric Delay (m)")
plt.title(f"Tropospheric Delay")
plt.legend()
plt.grid()
plt.tight_layout()
plt.savefig(f"{results_dir}/all_satellites.png")
plt.show()
