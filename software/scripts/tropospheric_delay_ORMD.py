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

json_file = "ephemerides/ephemeris_ormondBeach.json"
utc_time = datetime(2025, 9, 30, 18, 45, 30, tzinfo=timezone.utc)
receiver_ecef = (860376.4150, -5499833.3840, 3102756.9330) # Ormond Beach Municipal Airport CORS Receiver

# RXM-RAWX receiver time of week and week number
rcvTOW = 513845.005
gps_week = 2386
sat_id = 'E10'

# Convert receiver time to GPS system time
gps_tow = rcvTOW

# Receiver clock biases
dt_rcv = -8e-6

measured_pseudorange_E1 = 26383980.780 # meters
measured_pseudorange_E5a = 26383996.120 # meters

sat, constellation = load_ephemeris(json_file, sat_id)

range_results = geometric_range.calculate_satellite_position_and_range(json_file, sat_id, receiver_ecef,
                                                                       utc_time)

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