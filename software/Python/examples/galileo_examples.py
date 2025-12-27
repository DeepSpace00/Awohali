import json

from parsers.galileo_sfrbx_parser import GalileoINavParser, parse_ubx_file

def example_1_basic_parsing():
    """Example 1: Basic file parsing"""
    print("=" * 70)
    print("Example 1: Basic UBX File Parsing")
    print("=" * 70)
    
    # Parse UBX file
    results = parse_ubx_file("GNSS002.ubx")
    
    # Display results
    print(f"\nFound {len(results.get('Galileo', {}))} Galileo satellites")
    
    for sat_id, eph in results.get('Galileo', {}).items():
        print(f"\n{sat_id}:")
        print(f"  IODnav: {eph['IODnav']}")
        print(f"  Time of Ephemeris: {eph['toe']} seconds")
        print(f"  Eccentricity: {eph['e']:.6e}")
        print(f"  Semi-major axis: {(eph['sqrtA']**2):.3f} meters")


def example_2_direct_message_parsing():
    """Example 2: Parse individual SFRBX messages"""
    print("\n" + "=" * 70)
    print("Example 2: Direct Message Parsing")
    print("=" * 70)
    
    parser = GalileoINavParser()
    
    # Assume you have an SFRBX message (48 bytes for 8 words)
    # sfrbx_message = b'...'  # Your actual message bytes
    
    print("\nTo parse a single SFRBX message:")
    print("  inav = parser.extract_inav_from_sfrbx(sfrbx_message)")
    print("  word_data = parser.parse_inav_message(inav, svid)")


def example_3_parameter_extraction():
    """Example 3: Extract specific parameters"""
    print("\n" + "=" * 70)
    print("Example 3: Parameter Extraction")
    print("=" * 70)
    
    # Parse file
    results = parse_ubx_file("GNSS002.ubx")
    
    print("\nKeplerian Parameters:")
    for sat_id, eph in results.get('Galileo', {}).items():
        print(f"\n{sat_id}:")
        print(f"  M₀ (Mean Anomaly): {eph['M0']:.6e} semi-circles")
        print(f"  e (Eccentricity): {eph['e']:.6e}")
        print(f"  √A (Semi-major axis): {eph['sqrtA']:.6f} m^(1/2)")
        print(f"  Ω₀ (Long. Asc. Node): {eph['OMEGA0']:.6e} semi-circles")
        print(f"  i₀ (Inclination): {eph['i0']:.6e} semi-circles")
        print(f"  ω (Arg. of Perigee): {eph['omega']:.6e} semi-circles")


def example_4_clock_corrections():
    """Example 4: Clock correction parameters"""
    print("\n" + "=" * 70)
    print("Example 4: Clock Corrections")
    print("=" * 70)
    
    results = parse_ubx_file("GNSS002.ubx")
    
    print("\nClock Correction Parameters:")
    for sat_id, eph in results.get('Galileo', {}).items():
        print(f"\n{sat_id}:")
        print(f"  toc: {eph['toc']} seconds")
        print(f"  af0 (Clock Bias): {eph['af0']:.6e} seconds")
        print(f"  af1 (Clock Drift): {eph['af1']:.6e} sec/sec")
        print(f"  af2 (Clock Drift Rate): {eph['af2']:.6e} sec/sec²")


def example_5_harmonic_corrections():
    """Example 5: Harmonic correction coefficients"""
    print("\n" + "=" * 70)
    print("Example 5: Harmonic Corrections")
    print("=" * 70)
    
    results = parse_ubx_file("GNSS002.ubx")
    
    print("\nHarmonic Correction Coefficients:")
    for sat_id, eph in results.get('Galileo', {}).items():
        print(f"\n{sat_id}:")
        print(f"  Cuc: {eph['Cuc']:.6e} radians")
        print(f"  Cus: {eph['Cus']:.6e} radians")
        print(f"  Crc: {eph['Crc']:.6f} meters")
        print(f"  Crs: {eph['Crs']:.6f} meters")
        print(f"  Cic: {eph['Cic']:.6e} radians")
        print(f"  Cis: {eph['Cis']:.6e} radians")


def example_6_custom_processing():
    """Example 6: Custom word type processing"""
    print("\n" + "=" * 70)
    print("Example 6: Custom Processing")
    print("=" * 70)
    
    parser = GalileoINavParser()
    all_words = []
    
    # Read UBX file
    with open("GNSS002.ubx", 'rb') as f:
        data = f.read()
    
    i = 0
    while i < len(data) - 8:
        if data[i] == 0xB5 and data[i+1] == 0x62:
            msg_class = data[i+2]
            msg_id = data[i+3]
            
            if msg_class == 0x02 and msg_id == 0x13:  # RXM-SFRBX
                import struct
                length = struct.unpack('<H', data[i+4:i+6])[0]
                msg = data[i:i+6+length+2]
                
                if len(msg) == 6 + length + 2:
                    gnssId = msg[6]
                    svId = msg[7]
                    
                    if gnssId == 2:  # Galileo
                        inav = parser.extract_inav_from_sfrbx(msg)
                        if inav:
                            word_data = parser.parse_inav_message(inav, svId)
                            if word_data:
                                all_words.append(word_data)
                
                i += 6 + length + 2
            else:
                i += 1
        else:
            i += 1
    
    # Count word types
    word_counts = {}
    for word in all_words:
        wt = word['word_type']
        word_counts[wt] = word_counts.get(wt, 0) + 1
    
    print("\nWord Type Distribution:")
    for wt in sorted(word_counts.keys()):
        print(f"  Word Type {wt}: {word_counts[wt]} messages")


def example_7_save_formatted_output():
    """Example 7: Save formatted output"""
    print("\n" + "=" * 70)
    print("Example 7: Save Formatted Output")
    print("=" * 70)
    
    results = parse_ubx_file("your_recording.ubx")
    
    # Save as pretty JSON
    with open("galileo_ephemerides.json", 'w') as f:
        json.dump(results, f, indent=2)
    
    print("\nSaved formatted JSON to: galileo_ephemerides.json")
    
    # Save as compact JSON
    with open("galileo_ephemerides_compact.json", 'w') as f:
        json.dump(results, f)
    
    print("Saved compact JSON to: galileo_ephemerides_compact.json")


def example_8_quality_checks():
    """Example 8: Data quality checks"""
    print("\n" + "=" * 70)
    print("Example 8: Quality Checks")
    print("=" * 70)
    
    results = parse_ubx_file("GNSS002.ubx")
    
    print("\nData Quality Assessment:")
    for sat_id, eph in results.get('Galileo', {}).items():
        print(f"\n{sat_id}:")
        
        # Check SISA (Signal-in-Space Accuracy)
        sisa = eph.get('SISA', 255)
        if sisa < 50:
            print(f"  ✓ SISA: {sisa} (Good)")
        else:
            print(f"  ✗ SISA: {sisa} (Poor)")
        
        # Check health status if available
        if 'E1B_HS' in eph and 'E5b_HS' in eph:
            e1b_health = eph['E1B_HS']
            e5b_health = eph['E5b_HS']
            
            if e1b_health == 0 and e5b_health == 0:
                print(f"  ✓ Health: OK")
            else:
                print(f"  ✗ Health: E1B={e1b_health}, E5b={e5b_health}")
        
        # Check eccentricity range (should be 0-0.03 for GPS/Galileo)
        if 0 <= eph['e'] <= 0.03:
            print(f"  ✓ Eccentricity: {eph['e']:.6f} (Valid range)")
        else:
            print(f"  ⚠ Eccentricity: {eph['e']:.6f} (Unusual)")
        
        # Check semi-major axis (should be ~5,153 km for Galileo)
        a = eph['sqrtA'] ** 2
        if 26_000_000 < a < 30_000_000:  # ~26,000-30,000 km
            print(f"  ✓ Semi-major axis: {a/1000:.1f} km")
        else:
            print(f"  ⚠ Semi-major axis: {a/1000:.1f} km (Unusual)")


def main():
    """Run all examples"""
    print("\nGalileo I/NAV Parser - Usage Examples")
    print("=" * 70)
    print("\nNote: Replace 'your_recording.ubx' with your actual UBX file")
    print("=" * 70)
    
    # Uncomment the examples you want to run:
    
    # example_1_basic_parsing()
    example_2_direct_message_parsing()
    # example_3_parameter_extraction()
    # example_4_clock_corrections()
    # example_5_harmonic_corrections()
    # example_6_custom_processing()
    # example_7_save_formatted_output()
    # example_8_quality_checks()
    
    print("\n" + "=" * 70)
    print("Examples complete!")
    print("=" * 70)


if __name__ == '__main__':
    main()
