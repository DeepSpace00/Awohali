import json
import math
from datetime import datetime, timezone
from typing import Tuple, Dict, Any
import geometric_range as geo_range

json_file = "ephemeris_sample.json"
utc_time = datetime(2025, 9, 30, 18, 49, 24, tzinfo=timezone.utc)
receiver_ecef = (863794.228, -5503182.971, 3095968.875) # Apartment
receiver_clock_bias = 582534e-9 + 0.000289 # seconds
gnss_tow = 240595.004
gnss_week = 2386

measured_pseudorange_L1 = 25049745.1839653 # meters
measured_pseudorange_L5 = 25049744.1574245 # meters

results = geo_range.calculate_satellite_position_and_range(json_file, "G01", receiver_ecef, utc_time, gps_week=gnss_week, gps_tow=gnss_tow, receiver_clock_bias=receiver_clock_bias)

residual = measured_pseudorange - results['expected_pseudorange_m']

print(f"\n{results['satellite_id']} ({results['constellation']}):")
print(f"  Geometric Range:        {results['geometric_range_m']:>15,.2f} m")
print(f"  Sat Clock Correction:   {results['clock_correction_m']:>15,.2f} m")
print(f"  Corrected Range:        {results['corrected_range_m']:>15,.2f} m")
print(f"  Receiver Clock Bias:    {results['receiver_clock_bias_m']:>15,.2f} m")
print(f"  Expected Pseudorange:   {results['expected_pseudorange_m']:>15,.2f} m")
print(f"  Measured Pseudorange:   {measured_pseudorange:>15,.2f} m")
print(f"  Residual:               {residual:>15,.2f} m")
print(f"  → Residual is ionosphere + troposphere + multipath + noise")