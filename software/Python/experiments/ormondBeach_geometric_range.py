from datetime import datetime, timezone
from calculations import geometric_range_full as geo_range

f_Galileo_E1_Hz = 1575.420e6
f_Galileo_E5a_Hz = 1176.450e6
f_GPS_L1_Hz = 1575.420e6
f_GPS_L5_Hz = 1176.450e6

json_file = "../data/ephemerides/ephemeris_ormondBeach.json"
utc_time = datetime(2025, 9, 30, 18, 45, 30, tzinfo=timezone.utc)
receiver_ecef = (860376.4150, -5499833.3840, 3102756.9330) # Ormond Beach Municipal Airport CORS Receiver
receiver_clock_bias = 0 # seconds

measured_pseudorange_E1 = 26383980.780 # meters
measured_pseudorange_E5a = 26383996.120 # meters

combined_pseudorange = (((f_Galileo_E1_Hz ** 2) * measured_pseudorange_E1) - (f_Galileo_E5a_Hz ** 2) * measured_pseudorange_E5a) / (f_Galileo_E1_Hz ** 2 - f_Galileo_E5a_Hz ** 2)

results = geo_range.calculate_satellite_position_and_range(json_file, "E10", receiver_ecef, utc_time, receiver_clock_bias=receiver_clock_bias)

residual = combined_pseudorange - results['expected_pseudorange_m']

print(f"\n{results['satellite_id']} ({results['constellation']}):")
print(f"  Geometric Range:        {results['geometric_range_m']:>15,.2f} m")
print(f"  Sat Clock Correction:   {results['clock_correction_m']:>15,.2f} m")
print(f"  Corrected Range:        {results['corrected_range_m']:>15,.2f} m")
print(f"  Receiver Clock Bias:    {results['receiver_clock_bias_m']:>15,.2f} m")
print(f"  Expected Pseudorange:   {results['expected_pseudorange_m']:>15,.2f} m")
print(f"  Measured Pseudorange:   {combined_pseudorange:>15,.2f} m")
print(f"  Residual:               {residual:>15,.2f} m")
print(f"  → Residual is ionosphere + troposphere + multipath + noise")