#!/usr/bin/env python3
"""
Example usage of the UBX Navigation Data Parser

This script demonstrates how to use the ubx_nav_parser module
programmatically for custom applications.
"""

from ubx_nav_parser import (
    parse_ubx_file, 
    save_to_json,
    GPSNavDataParser,
    UBXParser,
    RXMSFRBXParser,
    GNSSId
)
import json


def example_basic_usage():
    """Example 1: Basic file parsing"""
    print("=" * 60)
    print("Example 1: Basic Usage")
    print("=" * 60)
    
    # Parse UBX file with all ephemerides
    ephemerides = parse_ubx_file(
        'data.ubx',
        save_all_ephemerides=True,
        json_indent=2
    )
    
    # Save to JSON
    save_to_json(ephemerides, 'output.json', indent=2)
    
    print("Parsed ephemerides saved to output.json\n")


def example_most_recent_only():
    """Example 2: Only save most recent ephemeris per satellite"""
    print("=" * 60)
    print("Example 2: Most Recent Ephemeris Only")
    print("=" * 60)
    
    # Parse and keep only most recent ephemeris
    ephemerides = parse_ubx_file(
        'data.ubx',
        save_all_ephemerides=False,  # Only most recent
        json_indent=2
    )
    
    save_to_json(ephemerides, 'latest_ephemerides.json', indent=2)
    
    print("Latest ephemerides saved to latest_ephemerides.json\n")


def example_access_data():
    """Example 3: Access ephemeris data programmatically"""
    print("=" * 60)
    print("Example 3: Programmatic Data Access")
    print("=" * 60)
    
    # Parse file
    ephemerides = parse_ubx_file('data.ubx', save_all_ephemerides=True)
    
    # Iterate through satellites
    gps_data = ephemerides.get('GPS', {})
    
    for sat_id, sat_ephemerides in gps_data.items():
        print(f"\nSatellite: {sat_id}")
        print(f"  Number of ephemerides: {len(sat_ephemerides)}")
        
        for toe_key, eph in sat_ephemerides.items():
            print(f"\n  Ephemeris: {toe_key}")
            print(f"    Reference time (toe): {eph['toe']} seconds")
            print(f"    Semi-major axis: {eph['a']:.3f} meters")
            print(f"    Eccentricity: {eph['e']:.6f}")
            print(f"    Inclination: {eph['i0']:.6f} radians")
            print(f"    RAAN: {eph['Omega0']:.6f} radians")
            print(f"    Argument of perigee: {eph['omega']:.6f} radians")
            print(f"    Mean anomaly: {eph['M0']:.6f} radians")
            print(f"    Clock bias (af0): {eph['af0']:.6e} seconds")
            print(f"    Clock drift (af1): {eph['af1']:.6e} sec/sec")


def example_custom_processing():
    """Example 4: Custom processing with direct parser access"""
    print("=" * 60)
    print("Example 4: Custom Processing")
    print("=" * 60)
    
    nav_parser = GPSNavDataParser()
    message_stats = {
        'total_messages': 0,
        'gps_messages': 0,
        'message_types': {}
    }
    
    # Open UBX file and process manually
    with UBXParser('data.ubx') as ubx:
        for msg_class, msg_id, payload in ubx.read_messages():
            message_stats['total_messages'] += 1
            
            # Only RXM-SFRBX messages
            if msg_class == UBXParser.CLASS_RXM and msg_id == UBXParser.MSG_RXM_SFRBX:
                sfrbx = RXMSFRBXParser.parse(payload)
                
                if sfrbx and sfrbx['gnssId'] == GNSSId.GPS:
                    message_stats['gps_messages'] += 1
                    
                    # Parse navigation data
                    msg_type = nav_parser.parse_gps_words(sfrbx['words'], sfrbx['svId'])
                    
                    if msg_type is not None:
                        # Track message type statistics
                        if msg_type not in message_stats['message_types']:
                            message_stats['message_types'][msg_type] = 0
                        message_stats['message_types'][msg_type] += 1
    
    # Print statistics
    print(f"Total UBX messages: {message_stats['total_messages']}")
    print(f"GPS navigation messages: {message_stats['gps_messages']}")
    print("\nMessage type breakdown:")
    for msg_type, count in sorted(message_stats['message_types'].items()):
        print(f"  Type {msg_type}: {count} messages")
    
    # Get final ephemerides
    ephemerides = nav_parser.get_ephemerides_dict(save_all=True)
    print(f"\nComplete ephemerides: {sum(len(s) for s in ephemerides['GPS'].values())}")


def example_filter_by_satellite():
    """Example 5: Filter ephemerides for specific satellites"""
    print("=" * 60)
    print("Example 5: Filter by Satellite")
    print("=" * 60)
    
    # Parse all data
    ephemerides = parse_ubx_file('data.ubx', save_all_ephemerides=True)
    
    # Filter for specific satellites (e.g., G01, G05, G10)
    satellites_of_interest = ['G01', 'G05', 'G10']
    
    filtered_data = {'GPS': {}}
    for sat_id in satellites_of_interest:
        if sat_id in ephemerides['GPS']:
            filtered_data['GPS'][sat_id] = ephemerides['GPS'][sat_id]
            print(f"Found ephemerides for {sat_id}")
    
    # Save filtered data
    save_to_json(filtered_data, 'filtered_ephemerides.json', indent=2)


def example_compute_satellite_position():
    """Example 6: Use ephemeris to compute satellite position (simplified)"""
    print("=" * 60)
    print("Example 6: Compute Satellite Position (Concept)")
    print("=" * 60)
    
    import math
    
    # Parse data
    ephemerides = parse_ubx_file('data.ubx', save_all_ephemerides=False)
    
    # Get first satellite with ephemeris
    gps_data = ephemerides.get('GPS', {})
    if not gps_data:
        print("No ephemerides found")
        return
    
    sat_id = list(gps_data.keys())[0]
    sat_data = gps_data[sat_id]
    toe_key = list(sat_data.keys())[0]
    eph = sat_data[toe_key]
    
    print(f"\nUsing ephemeris from {sat_id} at toe={eph['toe']}")
    
    # Example: Compute mean motion
    mu = 3.986005e14  # Earth's gravitational parameter (m^3/s^2)
    n0 = math.sqrt(mu / (eph['a'] ** 3))  # Mean motion (rad/s)
    
    print(f"Semi-major axis: {eph['a']:.3f} m")
    print(f"Mean motion: {n0:.6e} rad/s")
    print(f"Orbital period: {2 * math.pi / n0 / 3600:.3f} hours")
    
    print("\nNote: This is a simplified example.")
    print("Full position computation requires implementing Kepler's equations")
    print("and applying all harmonic corrections.")


def main():
    """Run all examples"""
    print("\n" + "=" * 60)
    print("UBX Navigation Data Parser - Usage Examples")
    print("=" * 60 + "\n")
    
    print("Note: These examples assume you have a 'data.ubx' file.")
    print("Modify the filename to match your actual UBX file.\n")
    
    # Uncomment the examples you want to run:
    
    # example_basic_usage()
    # example_most_recent_only()
    # example_access_data()
    # example_custom_processing()
    # example_filter_by_satellite()
    # example_compute_satellite_position()
    
    print("\nUncomment the examples in main() to run them.")


if __name__ == "__main__":
    main()
