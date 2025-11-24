from datetime import datetime, timezone
import geometric_range as geo_range

f_Galileo_E1_Hz = 1575.420e6
f_Galileo_E5a_Hz = 1176.450e6
f_GPS_L1_Hz = 1575.420e6
f_GPS_L5_Hz = 1176.450e6

json_file = "ephemeris_sample_2025-10-03.json"
utc_time = datetime(2025, 10, 3, 22, 43, 45, tzinfo=timezone.utc)
receiver_ecef = (863798.932, -5503182.842, 3095969.847) # Apartment
receiver_clock_bias = 579918 # ns
receiver_clock_drift = 545 # ns/s
iTOW = 513844000
rcvTOW = 513845.005
gnss_week = 2386

measured_pseudorange_L1 = 21585931.4032699 # meters
measured_pseudorange_L5 = 21585913.2487915 # meters

measured_pseudorange_E1 = 25332868.9768342 # meters
measured_pseudorange_E5a = 25332853.5543619 # meters

combined_pseudorange_GAL = (((f_Galileo_E1_Hz ** 2) * measured_pseudorange_E1) - (f_Galileo_E5a_Hz ** 2) * measured_pseudorange_E5a) / (f_Galileo_E1_Hz ** 2 - f_Galileo_E5a_Hz ** 2)
combined_pseudorange_GPS = (((f_GPS_L1_Hz ** 2) * measured_pseudorange_L1) - (f_GPS_L5_Hz ** 2) * measured_pseudorange_L5) / (f_GPS_L1_Hz ** 2 - f_GPS_L5_Hz ** 2)

dt = rcvTOW - (iTOW / 1000.0)
clock_bias_at_rcvTOW_ns = receiver_clock_bias + receiver_clock_drift * dt
clock_bias_at_rcvTOW_s = clock_bias_at_rcvTOW_ns / 1e9
receiver_clock_bias = clock_bias_at_rcvTOW_s

gnss_tow = rcvTOW + clock_bias_at_rcvTOW_s

results = geo_range.calculate_satellite_position_and_range(json_file, "G04", receiver_ecef, utc_time, gnss_week, rcvTOW, receiver_clock_bias=clock_bias_at_rcvTOW_s)

residual = combined_pseudorange_GPS - results['expected_pseudorange_m']

print(f"\n{results['satellite_id']} ({results['constellation']}):")
print(f"  Geometric Range:        {results['geometric_range_m']:>15,.2f} m")
print(f"  Sat Clock Correction:   {results['clock_correction_m']:>15,.2f} m")
print(f"  Corrected Range:        {results['corrected_range_m']:>15,.2f} m")
print(f"  Receiver Clock Bias:    {results['receiver_clock_bias_m']:>15,.2f} m")
print(f"  Expected Pseudorange:   {results['expected_pseudorange_m']:>15,.2f} m")
print(f"  Measured Pseudorange:   {combined_pseudorange_GPS:>15,.2f} m")
print(f"  Residual:               {residual:>15,.2f} m")
print(f"  → Residual is ionosphere + troposphere + multipath + noise")