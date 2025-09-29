#!./.venv/bin/python Python 3.13.5
"""
UBX-RXM-SFRBX Parser for GPS and Galileo Ephemeris Data
Extracts orbital parameters from raw navigation messages and timestamps them
based on the most recent NAV-TIMEUTC message at the time of logging.
"""

import struct
import json
from datetime import datetime, timezone
from collections import defaultdict


class UBXParser:
    # UBX message constants
    UBX_SYNC1 = 0xB5
    UBX_SYNC2 = 0x62
    UBX_CLASS_RXM = 0x02
    UBX_ID_SFRBX = 0x13
    UBX_CLASS_NAV = 0x01
    UBX_ID_TIMEUTC = 0x21
    
    # GNSS IDs
    GNSS_GPS = 0
    GNSS_GALILEO = 2
    
    # GPS epoch
    GPS_EPOCH = datetime(1980, 1, 6, 0, 0, 0, tzinfo=timezone.utc)
    
    def __init__(self, filename):
        self.filename = filename
        self.gps_subframes = defaultdict(dict)  # {svid: {subframe_id: (data, timestamp)}}
        self.galileo_words = defaultdict(list)   # {svid: [(word_data, timestamp)]}
        
        # Current timestamp from most recent NAV-TIMEUTC
        self.current_timestamp = None
        
        # Store when each satellite's ephemeris was completed
        self.ephemeris_timestamps = {}  # {svid: utc_timestamp}
        
    def parse_ubx_file(self):
        """Parse UBX file and extract SFRBX messages with timestamps"""
        with open(self.filename, 'rb') as f:
            data = f.read()
        
        idx = 0
        while idx < len(data) - 8:
            # Look for UBX header
            if data[idx] == self.UBX_SYNC1 and data[idx+1] == self.UBX_SYNC2:
                msg_class = data[idx+2]
                msg_id = data[idx+3]
                length = struct.unpack('<H', data[idx+4:idx+6])[0]
                
                if idx + 8 + length <= len(data):
                    payload = data[idx+6:idx+6+length]
                    
                    # Process NAV-TIMEUTC to update current timestamp
                    if msg_class == self.UBX_CLASS_NAV and msg_id == self.UBX_ID_TIMEUTC:
                        self.parse_timeutc(payload)
                    elif msg_class == self.UBX_CLASS_RXM and msg_id == self.UBX_ID_SFRBX:
                        # Use the current timestamp for this SFRBX message
                        self.parse_sfrbx(payload)
                
                idx += 8 + length
            else:
                idx += 1
    
    def parse_sfrbx(self, payload):
        """Parse UBX-RXM-SFRBX message"""
        if len(payload) < 8:
            return
        
        gnss_id = payload[0]
        svid = payload[1]
        freq_id = payload[3]
        num_words = payload[4]
        version = payload[6]
        
        # Extract raw navigation data words
        word_data = payload[8:]
        
        if gnss_id == self.GNSS_GPS:
            self.parse_gps_subframe(svid, word_data)
        elif gnss_id == self.GNSS_GALILEO:
            self.parse_galileo_page(svid, word_data)
    
    def parse_timeutc(self, payload):
        """Parse UBX-NAV-TIMEUTC message and update current timestamp"""
        if len(payload) < 20:
            return
        
        # Parse according to UBX protocol specification
        year = struct.unpack('<H', payload[12:14])[0]
        month = payload[14]
        day = payload[15]
        hour = payload[16]
        minute = payload[17]
        second = payload[18]
        valid = payload[19]
        
        # Check if time is valid (bit 2 of valid field)
        if valid & 0x04:
            try:
                self.current_timestamp = datetime(year, month, day, hour, minute, second, tzinfo=timezone.utc)
            except ValueError:
                pass  # Invalid date/time values
    
    def parse_gps_subframe(self, svid, data):
        """Parse GPS subframe (300 bits = 10 words of 30 bits each)"""
        if len(data) < 40:  # Need at least 10 words * 4 bytes
            return
        
        # Extract subframe ID from word 2 (bits 50-52)
        word2 = struct.unpack('>I', data[4:8])[0]
        subframe_id = (word2 >> 8) & 0x7
        
        if subframe_id in [1, 2, 3]:
            # Store subframe data with current timestamp
            self.gps_subframes[svid][subframe_id] = (data[:40], self.current_timestamp)
            
            # If we have all 3 subframes, timestamp the complete ephemeris set
            if len(self.gps_subframes[svid]) == 3:
                # Use the timestamp from when we received the last subframe
                _, last_timestamp = self.gps_subframes[svid][subframe_id]
                if last_timestamp:
                    self.ephemeris_timestamps[svid] = last_timestamp.isoformat().replace('+00:00', 'Z')
    
    def parse_galileo_page(self, svid, data):
        """Parse Galileo I/NAV page"""
        if len(data) < 32:  # Galileo page is 128 bits
            return
        
        self.galileo_words[svid].append((data[:32], self.current_timestamp))
        
        # Galileo ephemeris requires word types 1-4
        if len(self.galileo_words[svid]) >= 4:
            # Use the timestamp from the most recent page
            _, last_timestamp = self.galileo_words[svid][-1]
            if last_timestamp:
                self.ephemeris_timestamps[f"E{svid}"] = last_timestamp.isoformat().replace('+00:00', 'Z')
    
    def extract_gps_ephemeris(self, svid):
        """Extract GPS orbital parameters from subframes 1-3"""
        subframes = self.gps_subframes[svid]
        if not all(k in subframes for k in [1, 2, 3]):
            return None
        
        # Extract data from tuples (ignore timestamps in extraction)
        sf1_data, _ = subframes[1]
        sf2_data, _ = subframes[2]
        sf3_data, _ = subframes[3]
        
        # Subframe 1
        w3 = struct.unpack('>I', sf1_data[8:12])[0]
        w7 = struct.unpack('>I', sf1_data[24:28])[0]
        w8 = struct.unpack('>I', sf1_data[28:32])[0]
        w9 = struct.unpack('>I', sf1_data[32:36])[0]
        w10 = struct.unpack('>I', sf1_data[36:40])[0]
        
        # Extract GPS week number from subframe 1, word 3
        week_number = (w3 >> 14) & 0x3FF  # 10 bits
        
        toc = ((w7 >> 6) & 0xFFFF) * 16  # Time of clock
        af0 = self.twos_complement((w8 >> 8) & 0x3FFFFF, 22) * 2**-31
        af1 = self.twos_complement(((w8 & 0xFF) << 8) | ((w9 >> 24) & 0xFF), 16) * 2**-43
        
        # Subframe 2
        w3 = struct.unpack('>I', sf2_data[8:12])[0]
        w4 = struct.unpack('>I', sf2_data[12:16])[0]
        w5 = struct.unpack('>I', sf2_data[16:20])[0]
        w6 = struct.unpack('>I', sf2_data[20:24])[0]
        w7 = struct.unpack('>I', sf2_data[24:28])[0]
        w8 = struct.unpack('>I', sf2_data[28:32])[0]
        w9 = struct.unpack('>I', sf2_data[32:36])[0]
        w10 = struct.unpack('>I', sf2_data[36:40])[0]
        
        M0 = self.twos_complement(((w3 & 0xFF) << 24) | ((w4 >> 6) & 0xFFFFFF), 32) * 2**-31
        e = (((w4 & 0x3F) << 26) | ((w5 >> 6) & 0x3FFFFFF)) * 2**-33
        sqrt_a = (((w5 & 0x3F) << 26) | ((w6 >> 6) & 0x3FFFFFF)) * 2**-19
        
        # Subframe 3
        w3 = struct.unpack('>I', sf3_data[8:12])[0]
        w4 = struct.unpack('>I', sf3_data[12:16])[0]
        w5 = struct.unpack('>I', sf3_data[16:20])[0]
        w6 = struct.unpack('>I', sf3_data[20:24])[0]
        w7 = struct.unpack('>I', sf3_data[24:28])[0]
        w8 = struct.unpack('>I', sf3_data[28:32])[0]
        w10 = struct.unpack('>I', sf3_data[36:40])[0]
        
        omega0 = self.twos_complement(((w3 & 0xFF) << 24) | ((w4 >> 6) & 0xFFFFFF), 32) * 2**-31
        i0 = self.twos_complement(((w5 & 0xFF) << 24) | ((w6 >> 6) & 0xFFFFFF), 32) * 2**-31
        omega = self.twos_complement(((w7 & 0xFF) << 24) | ((w8 >> 6) & 0xFFFFFF), 32) * 2**-31
        
        toe = ((w10 >> 6) & 0xFFFF) * 16  # Time of ephemeris
        
        # Get timestamp when this ephemeris was received
        received_timestamp = self.ephemeris_timestamps.get(svid, None)
        
        return {
            'semi_major_axis': sqrt_a ** 2,
            'eccentricity': e,
            'inclination': i0,
            'right_ascension': omega0,
            'argument_of_perigee': omega,
            'mean_anomaly': M0,
            'toe': toe,
            'week_number': week_number,
            'received_utc': received_timestamp,  # When we received/logged this ephemeris
            'toe_utc': self.gps_time_to_utc(toe, week_number)  # Reference time in the ephemeris
        }
    
    def extract_galileo_ephemeris(self, svid):
        """Extract Galileo orbital parameters from I/NAV words"""
        # This is a simplified version - full Galileo parsing is complex
        # Would need to parse word types and extract specific bits
        return None
    
    def twos_complement(self, val, bits):
        """Convert to two's complement signed integer"""
        if val & (1 << (bits - 1)):
            return val - (1 << bits)
        return val
    
    def gps_time_to_utc(self, toe, week_number):
        """
        Convert GPS time to UTC
        
        Args:
            toe: Time of ephemeris in seconds of week
            week_number: GPS week number
            
        Returns:
            UTC time string
        """
        from datetime import timedelta
        
        # Calculate time since GPS epoch
        gps_seconds = week_number * 604800 + toe
        
        # Subtract leap seconds (GPS time is ahead of UTC)
        leap_seconds = 18  # As of 2017
        utc_seconds = gps_seconds - leap_seconds
        
        utc_time = self.GPS_EPOCH + timedelta(seconds=utc_seconds)
        return utc_time.isoformat().replace('+00:00', 'Z')
    
    def export_to_json(self, output_file='ephemeris.json'):
        """Export parsed ephemeris data to JSON"""
        ephemeris_data = {}
        
        # Extract GPS ephemeris
        for svid in self.gps_subframes.keys():
            if len(self.gps_subframes[svid]) == 3:  # Only export complete sets
                eph = self.extract_gps_ephemeris(svid)
                if eph:
                    sat_id = f"G{svid:02d}"
                    ephemeris_data[sat_id] = eph
        
        # Extract Galileo ephemeris (placeholder)
        for svid in self.galileo_words.keys():
            eph = self.extract_galileo_ephemeris(svid)
            if eph:
                sat_id = f"E{svid:02d}"
                ephemeris_data[sat_id] = eph
        
        # Use the most recent timestamp as the file timestamp
        file_timestamp = None
        if self.current_timestamp:
            file_timestamp = self.current_timestamp.isoformat().replace('+00:00', 'Z')
        else:
            file_timestamp = datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')
        
        output = {
            'file_timestamp': file_timestamp,
            'ephemeris': ephemeris_data
        }
        
        with open(output_file, 'w') as f:
            json.dump(output, f, indent=2)
        
        num_sats = len(ephemeris_data)
        print(f"Exported ephemeris data for {num_sats} satellites to {output_file}")
        
        # Print summary of received ephemeris
        if ephemeris_data:
            print("\nReceived Ephemeris Summary:")
            print(f"{'Sat ID':<8} {'Received UTC':<28} {'TOE (GPS sec)':<15} {'Week'}")
            print("-" * 75)
            for sat_id, eph in sorted(ephemeris_data.items()):
                received = eph.get('received_utc') or 'Unknown'
                toe = eph.get('toe', 0)
                week = eph.get('week_number', 0)
                print(f"{sat_id:<8} {received:<28} {toe:<15.1f} {week}")


# Usage example
if __name__ == '__main__':
    import argparse
    import sys
    import os
    
    arg_parser = argparse.ArgumentParser(
        description='Parse UBX-RXM-SFRBX ephemeris data from GPS and Galileo satellites'
    )
    arg_parser.add_argument('ubx_file', help='Input UBX file path')
    arg_parser.add_argument('json_file', nargs='?', default='ephemeris.json',
                           help='Output JSON file path (default: ephemeris.json)')
    
    args = arg_parser.parse_args()
    
    # Check if input file exists
    if not os.path.exists(args.ubx_file):
        print(f"Error: Input file '{args.ubx_file}' not found", file=sys.stderr)
        sys.exit(1)
    
    # Parse and export
    try:
        parser = UBXParser(args.ubx_file)
        parser.parse_ubx_file()
        parser.export_to_json(args.json_file)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)