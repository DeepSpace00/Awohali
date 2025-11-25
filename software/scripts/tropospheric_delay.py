"""
Calculate tropospheric delay from pseudorange, geometric range, and clock corrections.

Does not account for ionosphere or multipath biases
"""

from datetime import datetime, timezone
import geometric_range
import clock_correction
from ephemeris_classes import SatelliteEphemeris, GNSSConstants, load_ephemeris

f_Galileo_E1_Hz = 1575.420e6
f_Galileo_E5a_Hz = 1176.450e6
f_GPS_L1_Hz = 1575.420e6
f_GPS_L5_Hz = 1176.450e6

c = 299792458.0 # Speed of light (m/s)

json_file = "ephemerides/ephemeris_sample_2025-11-25.json"
utc_time = datetime(2025, 11, 25, 15, 29, 32, tzinfo=timezone.utc)
receiver_ecef = (867068.487, -5504812.066, 3092176.505) # Apartment

# RXM-RAWX receiver time of week and week number
rcvTOW = 228590.993
gps_week = 2394
sat_id = 'E36'

# Convert receiver time to GPS system time
gps_tow = rcvTOW

# Receiver clock biases (from NAV-CLOCK)
receiver_clock_bias = 352931 # ns
receiver_clock_drift = 485 # ns/s
iTOW = 228590000

measured_pseudorange_L1 = 22121216.9695235 # meters
measured_pseudorange_L5 = 22121211.5424739 # meters

measured_pseudorange_E1 = 24638154.25013 # meters
measured_pseudorange_E5a = 24638147.5472601 # meters

# Calculate receiver clock bias at measurement time (RXM-RAWX and NAV-CLOCK messages may be slightly out of sync)
dt = rcvTOW - (iTOW / 1000.0)
dt_rcv = (receiver_clock_bias + receiver_clock_drift * dt) / 1e9

sat, constellation = load_ephemeris(json_file, sat_id)

range_results = geometric_range.calculate_satellite_position_and_range(json_file, sat_id, receiver_ecef,
                                                                       utc_time, gps_week, gps_tow)

geometric_range = range_results['geometric_range_m']
t_tx = range_results['transmission_time']

# Calculate satellite clock correction
dt_sv = clock_correction.calculate_satellite_clock_offset(sat, t_tx)

# Calculate relativistic clock correction
dt_rel = clock_correction.calculate_relativistic_clock_correction(sat, t_tx)

# Calculate Biases in meters
rcv_clkBias_m = dt_rcv * c
sat_clkBias_m = dt_sv * c
relativistic_bias_m = dt_rel * c

biases = rcv_clkBias_m - sat_clkBias_m - relativistic_bias_m

# Calculate combined pseudorange for multi-band measurements
combined_pseudorange_GAL = (((f_Galileo_E1_Hz ** 2) * measured_pseudorange_E1) - (f_Galileo_E5a_Hz ** 2) * measured_pseudorange_E5a) / (f_Galileo_E1_Hz ** 2 - f_Galileo_E5a_Hz ** 2)
combined_pseudorange_GPS = (((f_GPS_L1_Hz ** 2) * measured_pseudorange_L1) - (f_GPS_L5_Hz ** 2) * measured_pseudorange_L5) / (f_GPS_L1_Hz ** 2 - f_GPS_L5_Hz ** 2)
combined_pseudorange = combined_pseudorange_GAL

# Calculate tropospheric delay (residuals)
tropospheric_delay_m = combined_pseudorange - geometric_range - biases

# Print results
print(f"\n{range_results['satellite_id']} ({range_results['constellation']}):")
print(f"  Geometric Range:        {geometric_range:>15,.2f} m")
print(f"  Sat Clock Bias:         {sat_clkBias_m:>15,.2f} m")
print(f"  Receiver Clock Bias:    {rcv_clkBias_m:>15,.2f} m")
print(f"  Relativistic Bias:      {relativistic_bias_m:>15,.2f} m")
print(f"\n  Combined Pseudorange:   {combined_pseudorange:>15,.2f} m")
print(f"  Tropospheric Delay*:    {tropospheric_delay_m:>15,.2f} m")