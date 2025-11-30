"""
Diagnostic script to debug elevation/azimuth calculation issues.

This will help identify why all satellites are showing the same angles.
"""

import numpy as np
import pandas as pd
from software.scripts.elevation_azimuth import calculate_elevation_azimuth, ecef_to_geodetic


def diagnose_elevation_azimuth_calculations(df, sat_id_col='PVNs', timeseries_col='Data'):
    """
    Diagnose issues with elevation and azimuth calculations.

    This will check:
    1. Are satellite ECEF positions different for different satellites?
    2. Is the receiver ECEF position reasonable?
    3. Are the calculated angles actually different?
    """
    print("=" * 80)
    print("ELEVATION/AZIMUTH CALCULATION DIAGNOSTICS")
    print("=" * 80)

    # Check first few satellites
    num_sats_to_check = min(5, len(df))

    for i in range(num_sats_to_check):
        row = df.iloc[i]
        sat_id = row[sat_id_col]
        ts_data = row[timeseries_col]

        print(f"\n{'=' * 80}")
        print(f"Satellite: {sat_id}")
        print(f"{'=' * 80}")

        if not isinstance(ts_data, pd.DataFrame) or ts_data.empty:
            print("  ⚠️  No valid timeseries data")
            continue

        print(f"  Number of epochs: {len(ts_data)}")

        # Check for required columns
        required_cols = ['sat_x', 'sat_y', 'sat_z', 'rx_x', 'rx_y', 'rx_z',
                         'elevation', 'azimuth']
        missing_cols = [col for col in required_cols if col not in ts_data.columns]

        if missing_cols:
            print(f"  ⚠️  Missing columns: {missing_cols}")
            print(f"  Available columns: {ts_data.columns.tolist()}")
            continue

        # Check first epoch in detail
        print(f"\n  --- First Epoch Analysis ---")
        first_epoch = ts_data.iloc[0]

        # Satellite position
        sat_pos = (first_epoch['sat_x'], first_epoch['sat_y'], first_epoch['sat_z'])
        print(f"  Satellite ECEF: ({sat_pos[0]:.1f}, {sat_pos[1]:.1f}, {sat_pos[2]:.1f}) m")

        sat_distance = np.sqrt(sat_pos[0] ** 2 + sat_pos[1] ** 2 + sat_pos[2] ** 2)
        print(f"  Satellite distance from Earth center: {sat_distance / 1e6:.3f} km")

        expected_gnss_radius = 26560e3  # GPS orbital radius ~26,560 km
        if sat_distance < 20e6 or sat_distance > 30e6:
            print(f"  ⚠️  WARNING: Distance seems wrong (expected ~26,560 km for GNSS)")

        # Receiver position
        rx_pos = (first_epoch['rx_x'], first_epoch['rx_y'], first_epoch['rx_z'])
        print(f"\n  Receiver ECEF: ({rx_pos[0]:.1f}, {rx_pos[1]:.1f}, {rx_pos[2]:.1f}) m")

        rx_distance = np.sqrt(rx_pos[0] ** 2 + rx_pos[1] ** 2 + rx_pos[2] ** 2)
        print(f"  Receiver distance from Earth center: {rx_distance / 1e3:.3f} km")

        earth_radius = 6371e3  # ~6,371 km
        if rx_distance < 6.3e6 or rx_distance > 6.5e6:
            print(f"  ⚠️  WARNING: Distance seems wrong (expected ~6,371 km for Earth surface)")

        # Convert receiver to geodetic
        lat, lon, h = ecef_to_geodetic(rx_pos[0], rx_pos[1], rx_pos[2])
        print(f"  Receiver Geodetic: ({np.degrees(lat):.6f}°, {np.degrees(lon):.6f}°, {h:.1f} m)")

        # Stored angles
        stored_elev = first_epoch['elevation']
        stored_azim = first_epoch['azimuth']
        print(f"\n  Stored elevation: {stored_elev:.2f}°")
        print(f"  Stored azimuth: {stored_azim:.2f}°")

        # Recalculate angles to verify
        calc_elev, calc_azim = calculate_elevation_azimuth(sat_pos, rx_pos)
        print(f"\n  Recalculated elevation: {calc_elev:.2f}°")
        print(f"  Recalculated azimuth: {calc_azim:.2f}°")

        # Check if they match
        elev_diff = abs(stored_elev - calc_elev)
        azim_diff = abs(stored_azim - calc_azim)

        if elev_diff > 0.1:
            print(f"  ⚠️  Elevation mismatch: {elev_diff:.3f}° difference")
        else:
            print(f"  ✓ Elevation matches")

        if azim_diff > 0.1:
            print(f"  ⚠️  Azimuth mismatch: {azim_diff:.3f}° difference")
        else:
            print(f"  ✓ Azimuth matches")

        # Check variation over time
        if len(ts_data) > 1:
            print(f"\n  --- Variation Over Time ---")
            print(f"  Elevation range: [{ts_data['elevation'].min():.2f}°, {ts_data['elevation'].max():.2f}°]")
            print(f"  Azimuth range: [{ts_data['azimuth'].min():.2f}°, {ts_data['azimuth'].max():.2f}°]")

            elev_std = ts_data['elevation'].std()
            azim_std = ts_data['azimuth'].std()
            print(f"  Elevation std dev: {elev_std:.2f}°")
            print(f"  Azimuth std dev: {azim_std:.2f}°")

            if elev_std < 0.1:
                print(f"  ⚠️  Elevation barely changes (std dev < 0.1°)")
            if azim_std < 0.1:
                print(f"  ⚠️  Azimuth barely changes (std dev < 0.1°)")

    # Compare between satellites
    print(f"\n{'=' * 80}")
    print("COMPARISON BETWEEN SATELLITES")
    print(f"{'=' * 80}")

    comparison_data = []
    for i in range(min(10, len(df))):
        row = df.iloc[i]
        sat_id = row[sat_id_col]
        ts_data = row[timeseries_col]

        if isinstance(ts_data, pd.DataFrame) and not ts_data.empty:
            if 'elevation' in ts_data.columns and 'azimuth' in ts_data.columns:
                first_epoch = ts_data.iloc[0]
                comparison_data.append({
                    'sat_id': sat_id,
                    'elevation': first_epoch['elevation'],
                    'azimuth': first_epoch['azimuth'],
                    'sat_x': first_epoch.get('sat_x', np.nan),
                    'sat_y': first_epoch.get('sat_y', np.nan),
                    'sat_z': first_epoch.get('sat_z', np.nan),
                })

    comp_df = pd.DataFrame(comparison_data)

    print("\nFirst epoch angles for all satellites:")
    print(comp_df[['sat_id', 'elevation', 'azimuth']].to_string(index=False))

    # Check if all elevations are the same
    if len(comp_df) > 1:
        elev_unique = comp_df['elevation'].nunique()
        azim_unique = comp_df['azimuth'].nunique()

        print(f"\nUnique elevation values: {elev_unique}")
        print(f"Unique azimuth values: {azim_unique}")

        if elev_unique == 1:
            print("⚠️  PROBLEM: All satellites have identical elevation!")
        if azim_unique == 1:
            print("⚠️  PROBLEM: All satellites have identical azimuth!")

        # Check if satellite positions are different
        sat_pos_unique = comp_df[['sat_x', 'sat_y', 'sat_z']].drop_duplicates()
        print(f"\nUnique satellite positions: {len(sat_pos_unique)}")

        if len(sat_pos_unique) == 1:
            print("⚠️  PROBLEM: All satellites have identical ECEF positions!")
            print("   This means your satellite position calculation is wrong.")
        elif len(sat_pos_unique) < len(comp_df):
            print(f"⚠️  WARNING: Only {len(sat_pos_unique)}/{len(comp_df)} unique positions")
        else:
            print("✓ All satellites have different positions")


def check_calculation_code_pattern():
    """
    Show common mistakes in elevation/azimuth calculation loops.
    """
    print("\n" + "=" * 80)
    print("COMMON MISTAKES TO CHECK IN YOUR CODE")
    print("=" * 80)

    print("""
1. WRONG: Using the same satellite position for all satellites

   # ❌ BAD - sat_pos defined outside loop
   sat_pos = calculate_satellite_position(...)  
   for sat_id in satellites:
       elev, az = calculate_elevation_azimuth(sat_pos, rx_pos)  # Always same sat_pos!

   # ✓ GOOD - sat_pos calculated per satellite
   for sat_id in satellites:
       sat_pos = calculate_satellite_position(sat_id, ...)
       elev, az = calculate_elevation_azimuth(sat_pos, rx_pos)

2. WRONG: Using the same receiver position (but this would be correct)

   # ✓ This is OK - receiver position is the same for all satellites
   rx_pos = (rx_x, rx_y, rx_z)
   for sat_id in satellites:
       sat_pos = calculate_satellite_position(sat_id, ...)
       elev, az = calculate_elevation_azimuth(sat_pos, rx_pos)

3. WRONG: Overwriting results from previous satellite

   # ❌ BAD - results get overwritten
   for sat_id in satellites:
       results = calculate_angles(...)  # Same variable name

   # ✓ GOOD - store in dictionary or DataFrame
   results = {}
   for sat_id in satellites:
       results[sat_id] = calculate_angles(...)

4. WRONG: Using wrong PRN/SVID when looking up ephemeris

   # ❌ BAD - always using first satellite's ephemeris
   ephemeris = ephemeris_dict[1]  # Always PRN 1!
   for sat_id in satellites:
       sat_pos = calculate_position(ephemeris, ...)

   # ✓ GOOD - look up each satellite's ephemeris
   for sat_id in satellites:
       ephemeris = ephemeris_dict[sat_id]
       sat_pos = calculate_position(ephemeris, ...)
""")


if __name__ == "__main__":
    print("Elevation/Azimuth Calculation Diagnostics")
    print("\nTo use this script:")
    print("  1. Load your DataFrame: df = ...")
    print("  2. Run: diagnose_elevation_azimuth_calculations(df)")
    print("\nThis will help identify why all satellites show the same angles.")
    print("\n" + "=" * 80)
    check_calculation_code_pattern()