import struct
import json

class UBXSFRBXParser:
    def __init__(self):
        self.ephemeris_data = {}
        self.almanac_data = {}
    
    def parse_message(self, data, debug=False):
        """Parse UBX-RXM-SFRBX message"""
        # Skip sync chars (0xB5 0x62), class (0x02), and id (0x13)
        if len(data) < 14:
            if debug:
                print(f"  Message too short: {len(data)} bytes")
            return None
        
        # Parse header
        length = struct.unpack('<H', data[4:6])[0]
        
        # Parse payload
        payload = data[6:6+length]
        gnss_id = payload[0]
        sv_id = payload[1]
        freq_id = payload[3]
        num_words = payload[4]
        
        if debug:
            gnss_names = {0: 'GPS', 1: 'SBAS', 2: 'Galileo', 3: 'BeiDou', 5: 'QZSS', 6: 'GLONASS'}
            print(f"  GNSS: {gnss_names.get(gnss_id, gnss_id)}, SV: {sv_id}, Words: {num_words}")
        
        # Extract data words (each is 4 bytes)
        # Try big-endian first (standard), then little-endian if needed
        offset = 8
        words = []
        words_le = []
        for i in range(num_words):
            if offset + 4 > len(payload):
                if debug:
                    print(f"  Incomplete word data at offset {offset}")
                return None
            # Big-endian (standard)
            word_be = struct.unpack('>I', payload[offset:offset+4])[0]
            words.append(word_be)
            # Little-endian (for troubleshooting)
            word_le = struct.unpack('<I', payload[offset:offset+4])[0]
            words_le.append(word_le)
            offset += 4
        
        return {
            'gnss_id': gnss_id,
            'sv_id': sv_id,
            'freq_id': freq_id,
            'words': words,
            'words_le': words_le
        }
    
    def extract_gps_subframe(self, words, words_le=None, debug=False):
        """Extract GPS subframe data from words
        
        Args:
            words: Big-endian words (standard)
            words_le: Little-endian words (for non-standard receivers)
            debug: Enable debug output
        """
        if len(words) < 10:
            if debug:
                print(f"  Not enough words: {len(words)}")
            return None
        
        # Try both big-endian and little-endian
        for endian_name, word_list in [('big-endian', words), ('little-endian', words_le)]:
            if word_list is None:
                continue
                
            # GPS uses 30-bit words, need to strip parity bits
            # Each word has 24 data bits + 6 parity bits
            subframe_data = []
            for word in word_list:
                # Extract 24 MSBs (ignore 6 LSB parity bits for data extraction)
                data_bits = (word >> 6) & 0xFFFFFF
                subframe_data.append(data_bits)
            
            # First word contains preamble and TLM
            preamble = (subframe_data[0] >> 16) & 0xFF
            
            if debug and endian_name == 'big-endian':
                print(f"  Preamble: 0x{preamble:02X} (expected 0x8B)")
                print(f"  First word raw: 0x{word_list[0]:08X}")
            
            if preamble == 0x8B:  # GPS preamble found!
                # Second word contains TOW and subframe ID
                tow = (subframe_data[1] >> 7) & 0x1FFFF
                subframe_id = (subframe_data[1] >> 2) & 0x7
                
                if debug:
                    print(f"  ✓ Valid preamble found using {endian_name}!")
                    print(f"  Subframe ID: {subframe_id}, TOW: {tow}")
                
                return {
                    'subframe_id': subframe_id,
                    'tow': tow,
                    'words': subframe_data,
                    'endian': endian_name
                }
        
        if debug:
            print(f"  ✗ No valid preamble found in either endianness")
        
        return None
    
    def parse_gps_ephemeris(self, sv_id, subframe_id, words):
        """Parse GPS ephemeris from subframes 1-3"""
        if sv_id not in self.ephemeris_data:
            self.ephemeris_data[sv_id] = {}
        
        if subframe_id == 1:
            # Subframe 1: Clock corrections and health
            self.ephemeris_data[sv_id]['week'] = (words[2] >> 14) & 0x3FF
            self.ephemeris_data[sv_id]['ura'] = (words[2] >> 8) & 0xF
            self.ephemeris_data[sv_id]['health'] = (words[2] >> 2) & 0x3F
            self.ephemeris_data[sv_id]['iodc'] = ((words[2] & 0x3) << 8) | ((words[7] >> 16) & 0xFF)
            self.ephemeris_data[sv_id]['tgd'] = self._twos_complement((words[6] >> 6) & 0xFF, 8) * (2**-31)
            self.ephemeris_data[sv_id]['toc'] = ((words[7] >> 0) & 0xFFFF) * 16
            self.ephemeris_data[sv_id]['af2'] = self._twos_complement((words[8] >> 16) & 0xFF, 8) * (2**-55)
            self.ephemeris_data[sv_id]['af1'] = self._twos_complement((words[8] >> 0) & 0xFFFF, 16) * (2**-43)
            self.ephemeris_data[sv_id]['af0'] = self._twos_complement(((words[9] >> 2) & 0x3FFFFF), 22) * (2**-31)
        
        elif subframe_id == 2:
            # Subframe 2: Ephemeris data (orbit 1)
            self.ephemeris_data[sv_id]['iode2'] = (words[2] >> 16) & 0xFF
            self.ephemeris_data[sv_id]['crs'] = self._twos_complement((words[2] >> 0) & 0xFFFF, 16) * (2**-5)
            self.ephemeris_data[sv_id]['delta_n'] = self._twos_complement((words[3] >> 8) & 0xFFFF, 16) * (2**-43)
            self.ephemeris_data[sv_id]['m0'] = self._twos_complement(((words[3] & 0xFF) << 24) | (words[4] >> 0), 32) * (2**-31)
            self.ephemeris_data[sv_id]['cuc'] = self._twos_complement((words[5] >> 8) & 0xFFFF, 16) * (2**-29)
            self.ephemeris_data[sv_id]['e'] = ((words[5] & 0xFF) << 24) | (words[6] >> 0)
            self.ephemeris_data[sv_id]['e'] = self.ephemeris_data[sv_id]['e'] * (2**-33)
            self.ephemeris_data[sv_id]['cus'] = self._twos_complement((words[7] >> 8) & 0xFFFF, 16) * (2**-29)
            self.ephemeris_data[sv_id]['sqrt_a'] = ((words[7] & 0xFF) << 24) | (words[8] >> 0)
            self.ephemeris_data[sv_id]['sqrt_a'] = self.ephemeris_data[sv_id]['sqrt_a'] * (2**-19)
            self.ephemeris_data[sv_id]['toe'] = ((words[9] >> 8) & 0xFFFF) * 16
            
        elif subframe_id == 3:
            # Subframe 3: Ephemeris data (orbit 2)
            self.ephemeris_data[sv_id]['cic'] = self._twos_complement((words[2] >> 8) & 0xFFFF, 16) * (2**-29)
            self.ephemeris_data[sv_id]['omega0'] = self._twos_complement(((words[2] & 0xFF) << 24) | (words[3] >> 0), 32) * (2**-31)
            self.ephemeris_data[sv_id]['cis'] = self._twos_complement((words[4] >> 8) & 0xFFFF, 16) * (2**-29)
            self.ephemeris_data[sv_id]['i0'] = self._twos_complement(((words[4] & 0xFF) << 24) | (words[5] >> 0), 32) * (2**-31)
            self.ephemeris_data[sv_id]['crc'] = self._twos_complement((words[6] >> 8) & 0xFFFF, 16) * (2**-5)
            self.ephemeris_data[sv_id]['omega'] = self._twos_complement(((words[6] & 0xFF) << 24) | (words[7] >> 0), 32) * (2**-31)
            self.ephemeris_data[sv_id]['omega_dot'] = self._twos_complement(words[8], 24) * (2**-43)
            self.ephemeris_data[sv_id]['iode3'] = (words[9] >> 16) & 0xFF
            self.ephemeris_data[sv_id]['idot'] = self._twos_complement((words[9] >> 2) & 0x3FFF, 14) * (2**-43)
    
    def parse_gps_almanac(self, sv_id, subframe_id, words):
        """Parse GPS almanac from subframes 4 and 5"""
        if subframe_id not in [4, 5]:
            return
        
        # Page ID is in word 3
        page_id = (words[2] >> 16) & 0x3F
        
        # Pages 1-24 contain almanac data for different satellites
        if 1 <= page_id <= 24:
            almanac_sv = page_id
            if almanac_sv not in self.almanac_data:
                self.almanac_data[almanac_sv] = {}
            
            self.almanac_data[almanac_sv]['e'] = ((words[2] >> 0) & 0xFFFF) * (2**-21)
            self.almanac_data[almanac_sv]['toa'] = ((words[3] >> 16) & 0xFF) * 4096
            self.almanac_data[almanac_sv]['delta_i'] = self._twos_complement((words[3] >> 0) & 0xFFFF, 16) * (2**-19)
            self.almanac_data[almanac_sv]['omega_dot'] = self._twos_complement((words[4] >> 8) & 0xFFFF, 16) * (2**-38)
            self.almanac_data[almanac_sv]['health'] = (words[4] >> 0) & 0xFF
            self.almanac_data[almanac_sv]['sqrt_a'] = ((words[5] >> 0) & 0xFFFFFF) * (2**-11)
            self.almanac_data[almanac_sv]['omega0'] = self._twos_complement(words[6], 24) * (2**-23)
            self.almanac_data[almanac_sv]['omega'] = self._twos_complement(words[7], 24) * (2**-23)
            self.almanac_data[almanac_sv]['m0'] = self._twos_complement(words[8], 24) * (2**-23)
            self.almanac_data[almanac_sv]['af0'] = self._twos_complement(((words[9] >> 13) & 0x7FF), 11) * (2**-20)
            self.almanac_data[almanac_sv]['af1'] = self._twos_complement(((words[9] >> 2) & 0x7FF), 11) * (2**-38)
    
    def _twos_complement(self, val, bits):
        """Convert unsigned value to signed using two's complement"""
        if val & (1 << (bits - 1)):
            return val - (1 << bits)
        return val
    
    def process_ubx_message(self, data, debug=False, failed_reasons=None):
        """Main processing function"""
        parsed = self.parse_message(data, debug=debug)
        if not parsed:
            if failed_reasons:
                failed_reasons['parsing_error'] += 1
            return None
        
        if parsed['gnss_id'] == 0:  # GPS
            subframe = self.extract_gps_subframe(parsed['words'], parsed.get('words_le'), debug=debug)
            if not subframe:
                if failed_reasons:
                    # Check if it's a preamble issue
                    if len(parsed['words']) >= 10:
                        data_bits = (parsed['words'][0] >> 6) & 0xFFFFFF
                        preamble = (data_bits >> 16) & 0xFF
                        if preamble != 0x8B:
                            failed_reasons['bad_preamble'] += 1
                        else:
                            failed_reasons['no_subframe'] += 1
                    else:
                        failed_reasons['no_subframe'] += 1
                return None
            
            sv_id = parsed['sv_id']
            subframe_id = subframe['subframe_id']
            words = subframe['words']
            
            if 1 <= subframe_id <= 3:
                self.parse_gps_ephemeris(sv_id, subframe_id, words)
                return {'type': 'ephemeris', 'sv_id': sv_id, 'subframe_id': subframe_id}
            elif subframe_id in [4, 5]:
                self.parse_gps_almanac(sv_id, subframe_id, words)
                return {'type': 'almanac', 'sv_id': sv_id, 'subframe_id': subframe_id}
        else:
            if failed_reasons:
                failed_reasons['non_gps'] += 1
        
        return None

    def calculate_checksum(self, data):
        """Calculate UBX checksum (Fletcher-8)"""
        ck_a = 0
        ck_b = 0
        for byte in data:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        return ck_a, ck_b
    
    def read_ubx_file(self, filename, verbose=False, skip_checksum=False, debug=False):
        """Read and process all UBX-RXM-SFRBX messages from a file
        
        Args:
            filename: Path to UBX file
            verbose: Print detailed processing information
            skip_checksum: Skip checksum verification (use for corrupted files)
            debug: Print detailed debug information about why messages fail
        """
        with open(filename, 'rb') as f:
            data = f.read()
        
        messages_processed = 0
        messages_found = 0
        checksum_failed = 0
        failed_reasons = {
            'no_subframe': 0,
            'bad_preamble': 0,
            'non_gps': 0,
            'parsing_error': 0
        }
        gnss_types = {}
        pos = 0
        
        while pos < len(data):
            # Look for UBX sync characters
            if pos + 1 >= len(data):
                break
                
            if data[pos] == 0xB5 and data[pos+1] == 0x62:
                # Found potential UBX message
                if pos + 6 > len(data):
                    break
                
                msg_class = data[pos+2]
                msg_id = data[pos+3]
                length = struct.unpack('<H', data[pos+4:pos+6])[0]
                
                # Check if we have the complete message
                total_length = 6 + length + 2  # header + payload + checksum
                if pos + total_length > len(data):
                    break
                
                # Extract message
                message = data[pos:pos+total_length]
                
                # Verify checksum unless skipped
                checksum_valid = True
                if not skip_checksum:
                    expected_ck_a, expected_ck_b = self.calculate_checksum(message[2:6+length])
                    actual_ck_a = message[6+length]
                    actual_ck_b = message[6+length+1]
                    checksum_valid = (expected_ck_a == actual_ck_a and expected_ck_b == actual_ck_b)
                    
                    if not checksum_valid:
                        checksum_failed += 1
                        if verbose:
                            print(f"Checksum failed at position {pos} (Class: 0x{msg_class:02X}, ID: 0x{msg_id:02X})")
                
                if checksum_valid or skip_checksum:
                    # Valid message - check if it's RXM-SFRBX
                    if msg_class == 0x02 and msg_id == 0x13:
                        messages_found += 1
                        
                        # Debug: Check GNSS type
                        if debug and len(message) > 6:
                            payload = message[6:6+length]
                            if len(payload) > 0:
                                gnss_id = payload[0]
                                gnss_types[gnss_id] = gnss_types.get(gnss_id, 0) + 1
                        
                        result = self.process_ubx_message(message, debug=debug, failed_reasons=failed_reasons)
                        if result:
                            messages_processed += 1
                            if verbose:
                                print(f"Processed {result['type']} for SV {result['sv_id']}, subframe {result['subframe_id']}")
                
                pos += total_length
            else:
                pos += 1
        
        print(f"\nFound {messages_found} RXM-SFRBX messages, successfully processed {messages_processed}")
        if checksum_failed > 0:
            print(f"⚠ {checksum_failed} messages failed checksum verification")
            if not skip_checksum:
                print(f"  Tip: Use skip_checksum=True to process them anyway")
        
        if debug:
            print(f"\n=== DEBUG INFO ===")
            print(f"GNSS types found:")
            gnss_names = {0: 'GPS', 1: 'SBAS', 2: 'Galileo', 3: 'BeiDou', 4: 'IMES', 5: 'QZSS', 6: 'GLONASS'}
            for gnss_id, count in gnss_types.items():
                name = gnss_names.get(gnss_id, f'Unknown ({gnss_id})')
                print(f"  {name}: {count} messages")
            
            print(f"\nFailure reasons:")
            print(f"  Non-GPS satellites: {failed_reasons['non_gps']}")
            print(f"  Bad preamble: {failed_reasons['bad_preamble']}")
            print(f"  No subframe extracted: {failed_reasons['no_subframe']}")
            print(f"  Parsing errors: {failed_reasons['parsing_error']}")
            
        return messages_processed
    
    def diagnose_data(self):
        """Diagnose why ephemeris data might be incomplete"""
        print("\n=== DIAGNOSTIC REPORT ===\n")
        
        if not self.ephemeris_data:
            print("⚠ NO EPHEMERIS DATA COLLECTED!")
            print("\nPossible reasons:")
            print("  1. File contains no RXM-SFRBX messages")
            print("  2. Messages are corrupted or have wrong format")
            print("  3. GNSS system is not GPS (only GPS supported in current version)")
            print("  4. Receiver was not tracking any satellites")
            return
        
        print(f"Found ephemeris data for {len(self.ephemeris_data)} satellites\n")
        
        for sv_id, data in sorted(self.ephemeris_data.items()):
            has_sf1 = 'week' in data
            has_sf2 = 'iode2' in data
            has_sf3 = 'iode3' in data
            
            status = "✓ COMPLETE" if (has_sf1 and has_sf2 and has_sf3) else "✗ INCOMPLETE"
            print(f"SV {sv_id:2d}: {status}")
            
            if not (has_sf1 and has_sf2 and has_sf3):
                missing = []
                if not has_sf1:
                    missing.append("Subframe 1 (clock data)")
                if not has_sf2:
                    missing.append("Subframe 2 (orbit part 1)")
                if not has_sf3:
                    missing.append("Subframe 3 (orbit part 2)")
                print(f"         Missing: {', '.join(missing)}")
                print(f"         Keys present: {list(data.keys())}")
        
        if self.almanac_data:
            print(f"\nAlmanac data available for {len(self.almanac_data)} satellites")
        else:
            print("\n⚠ No almanac data collected (requires subframes 4-5)")
        
        print("\n=== COMMON ISSUES ===")
        print("• Recording too short: Need at least 30 seconds to get all 3 subframes")
        print("• Weak signals: Receiver may decode some subframes but not others")
        print("• Non-GPS satellites: This parser only handles GPS (GNSS ID = 0)")
        print("• Wrong message type: Ensure receiver is outputting RXM-SFRBX, not RXM-RAW")
    
    def get_ephemeris_summary(self):
        """Get summary of collected ephemeris data"""
        summary = {}
        for sv_id, data in self.ephemeris_data.items():
            # Check if we have all three subframes
            has_sf1 = 'week' in data
            has_sf2 = 'iode2' in data
            has_sf3 = 'iode3' in data
            
            summary[sv_id] = {
                'complete': has_sf1 and has_sf2 and has_sf3,
                'subframes': {
                    1: has_sf1,
                    2: has_sf2,
                    3: has_sf3
                },
                'health': data.get('health', 'N/A')
            }
        return summary
    
    def get_almanac_summary(self):
        """Get summary of collected almanac data"""
        return {sv_id: {'health': data.get('health', 'N/A')} 
                for sv_id, data in self.almanac_data.items()}
    
    def export_to_json(self, filename):
        """Export all ephemeris and almanac data to JSON file"""
        output_data = {
            'ephemeris': {},
            'almanac': {},
            'metadata': {
                'total_satellites_ephemeris': len(self.ephemeris_data),
                'total_satellites_almanac': len(self.almanac_data),
                'complete_ephemeris': []
            }
        }
        
        # Export ephemeris data
        for sv_id, data in self.ephemeris_data.items():
            output_data['ephemeris'][f'SV{sv_id}'] = data
            
            # Check if ephemeris is complete
            has_sf1 = 'week' in data
            has_sf2 = 'iode2' in data
            has_sf3 = 'iode3' in data
            if has_sf1 and has_sf2 and has_sf3:
                output_data['metadata']['complete_ephemeris'].append(sv_id)
        
        # Export almanac data
        for sv_id, data in self.almanac_data.items():
            output_data['almanac'][f'SV{sv_id}'] = data
        
        # Write to file
        with open(filename, 'w') as f:
            json.dump(output_data, f, indent=2)
        
        print(f"Data exported to {filename}")
        print(f"  Ephemeris: {len(self.ephemeris_data)} satellites")
        print(f"  Complete ephemeris: {len(output_data['metadata']['complete_ephemeris'])} satellites")
        print(f"  Almanac: {len(self.almanac_data)} satellites")
        
        return output_data
    
    def export_ephemeris_rinex(self, sv_id, output_file=None):
        """Export ephemeris data for a satellite (simplified format)"""
        if sv_id not in self.ephemeris_data:
            return None
        
        eph = self.ephemeris_data[sv_id]
        
        output = []
        output.append(f"Ephemeris for GPS SV {sv_id}")
        output.append("=" * 40)
        
        if 'week' in eph:
            output.append(f"GPS Week: {eph['week']}")
            output.append(f"TOC: {eph.get('toc', 'N/A')} seconds")
            output.append(f"Clock Bias (af0): {eph.get('af0', 'N/A')}")
            output.append(f"Clock Drift (af1): {eph.get('af1', 'N/A')}")
            output.append(f"Clock Drift Rate (af2): {eph.get('af2', 'N/A')}")
            output.append(f"Health: {eph.get('health', 'N/A')}")
        
        if 'sqrt_a' in eph:
            output.append(f"\nOrbital Parameters:")
            output.append(f"Semi-major axis (sqrt): {eph.get('sqrt_a', 'N/A')}")
            output.append(f"Eccentricity: {eph.get('e', 'N/A')}")
            output.append(f"TOE: {eph.get('toe', 'N/A')} seconds")
            output.append(f"Mean Anomaly (M0): {eph.get('m0', 'N/A')}")
            output.append(f"Longitude of Ascending Node (Omega0): {eph.get('omega0', 'N/A')}")
            output.append(f"Argument of Perigee (omega): {eph.get('omega', 'N/A')}")
            output.append(f"Inclination (i0): {eph.get('i0', 'N/A')}")
        
        result = "\n".join(output)
        
        if output_file:
            with open(output_file, 'w') as f:
                f.write(result)
        
        return result


# Usage example
if __name__ == "__main__":
    parser = UBXSFRBXParser()
    
    # Process a UBX file with debug info
    # parser.read_ubx_file('your_file.ubx', debug=True)
    # 
    # # If you have checksum errors, try bypassing them:
    # # num_messages = parser.read_ubx_file('your_file.ubx', verbose=True, skip_checksum=True)
    # 
    # # Run diagnostics to see what's missing
    # parser.diagnose_data()
    # 
    # # Export all data to JSON
    # parser.export_to_json('navigation_data.json')
    
    # Check what ephemeris data was collected
    # eph_summary = parser.get_ephemeris_summary()
    # print("\nEphemeris Summary:")
    # for sv_id, info in eph_summary.items():
    #     status = "COMPLETE" if info['complete'] else "INCOMPLETE"
    #     print(f"  SV {sv_id}: {status} - Health: {info['health']}")
    
    # Check almanac data
    # alm_summary = parser.get_almanac_summary()
    # print(f"\nAlmanac data collected for {len(alm_summary)} satellites")
    
    # Export ephemeris for a specific satellite
    # if 1 in parser.ephemeris_data:
    #     print("\n" + parser.export_ephemeris_rinex(1))
    
    # Access raw data
    # print("\nRaw ephemeris data:")
    # print(parser.ephemeris_data)
    # print("\nRaw almanac data:")
    # print(parser.almanac_data)
    
    print("UBX-RXM-SFRBX Parser ready.")
    print("Usage:")
    print("  parser = UBXSFRBXParser()")
    print("  parser.read_ubx_file('your_file.ubx', debug=True)  # See why messages fail!")
    print("  parser.diagnose_data()  # See what's missing!")
    print("  parser.export_to_json('output.json')")

parser = UBXSFRBXParser()
# Try with checksum skipping since those are failing too
parser.read_ubx_file('GNSS015.ubx', skip_checksum=True, debug=False)
parser.diagnose_data()
parser.export_to_json('navigation_data.json')