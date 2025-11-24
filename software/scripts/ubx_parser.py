#!./.venv/bin/python Python 3.13.5
"""
UBX File Parser and CSV Splitter - Manual Parser

This script manually decodes .ubx files and splits different UBX message types into separate CSV files.
Supports UBX-NAV-TIMEUTC, UBX-RXM-RAWX, UBX-RXM-SFRBX, UBX-NAV-CLOCK, 
UBX-NAV-HPPOSECEF, and UBX-NAV-HPPOSLLH messages.

Requirements:
    pip install pandas
"""

import struct
import sys
from datetime import datetime, timezone
from pathlib import Path
import pandas as pd


class ManualUBXParser:
    def __init__(self, ubx_file_path, output_dir=None, column_orders=None):
        """Initialize manual UBX parser."""
        print(f"Initializing Manual UBX Parser...")
        
        self.ubx_file_path = Path(str(ubx_file_path))
        print(f"Input file: {self.ubx_file_path}")
        
        if output_dir is None:
            self.output_dir = self.ubx_file_path.parent
        else:
            self.output_dir = Path(str(output_dir))
        
        print(f"Output directory: {self.output_dir}")
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Message storage
        self.message_data = {
            'NAV-TIMEUTC': [],
            'RXM-RAWX': [],
            'RXM-SFRBX': [],
            'NAV-CLOCK': [],
            'NAV-HPPOSECEF': [],
            'NAV-HPPOSLLH': []
        }
        
        # Current UTC time for timestamping
        self.current_utc_time = None
        
        # Exclusion functionality
        self.excluded_columns = set({'version', 'reserved0', 'reserved1', 'reserved2'})  # Global exclusions
        self.message_type_exclusions = {
            'NAV-TIMEUTC': set(),
            'RXM-RAWX': set(),
            'RXM-SFRBX': set(),
            'NAV-CLOCK': set(),
            'NAV-HPPOSECEF': set(),
            'NAV-HPPOSLLH': set()
        }
        
        # Column orders
        self.column_orders = self._get_default_column_orders()
        if column_orders:
            self.column_orders.update(column_orders)
        
        # UBX message class and ID mappings
        self.message_types = {
            (0x01, 0x21): 'NAV-TIMEUTC',
            (0x02, 0x15): 'RXM-RAWX', 
            (0x02, 0x13): 'RXM-SFRBX',
            (0x01, 0x22): 'NAV-CLOCK',
            (0x01, 0x13): 'NAV-HPPOSECEF',
            (0x01, 0x14): 'NAV-HPPOSLLH'
        }
        
        print(f"Initialization complete.")

    def _get_default_column_orders(self):
        """Define default column orders for each message type."""
        return {
            'NAV-TIMEUTC': [
                'timestamp_utc', 'iTOW', 'tAcc', 'nano', 'year', 'month', 'day', 
                'hour', 'min', 'sec', 'valid', 'validTOW', 'validWKN', 'validUTC', 
                'authStatus', 'utcStandard'
            ],
            'RXM-RAWX': [
                'timestamp_utc', 'rcvTow', 'week', 'leapS', 'numMeas', 'recStat', 
                'version', 'reserved1', 'prMes', 'cpMes', 'doMes', 'gnssId', 
                'svId', 'sigId', 'freqId', 'locktime', 'cno', 'prStdev', 
                'cpStdev', 'doStdev', 'trkStat', 'prValid', 'cpValid', 'halfCyc', 
                'subHalfCyc', 'reserved2'
            ],
            'RXM-SFRBX': [
                'timestamp_utc', 'gnssId', 'svId', 'reserved1', 'freqId', 
                'numWords', 'chn', 'version', 'reserved2', 'dwrd'
            ],
            'NAV-CLOCK': [
                'timestamp_utc', 'iTOW', 'clkB', 'clkD', 'tAcc', 'fAcc'
            ],
            'NAV-HPPOSECEF': [
                'timestamp_utc', 'version', 'reserved1', 'iTOW', 'ecefX', 'ecefY', 
                'ecefZ', 'ecefXHp', 'ecefYHp', 'ecefZHp', 'reserved2', 'pAcc',
                'invalidEcef', 'ecefX_full_m', 'ecefY_full_m', 'ecefZ_full_m'
            ],
            'NAV-HPPOSLLH': [
                'timestamp_utc', 'version', 'reserved1', 'iTOW', 'lon', 'lat', 
                'height', 'hMSL', 'lonHp', 'latHp', 'heightHp', 'hMSLHp', 
                'hAcc', 'vAcc', 'invalidLlh', 'lon_full_deg', 'lat_full_deg', 
                'height_full_m', 'hMSL_full_m'
            ]
        }
    
    def set_column_order(self, msg_type, column_order):
        """Set custom column order for a specific message type."""
        if msg_type in self.message_data:
            self.column_orders[msg_type] = column_order
            print(f"Set custom column order for {msg_type}")
        else:
            print(f"Warning: Unknown message type '{msg_type}'")
    
    def add_excluded_column(self, column_name, msg_type=None):
        """Add a column to the exclusion list.
        
        Args:
            column_name: Name of column to exclude
            msg_type: If specified, exclude only for this message type. 
                     If None, exclude for all message types.
        """
        if msg_type is None:
            self.excluded_columns.add(column_name)
            print(f"Added '{column_name}' to global excluded columns")
        elif msg_type in self.message_type_exclusions:
            self.message_type_exclusions[msg_type].add(column_name)
            print(f"Added '{column_name}' to excluded columns for {msg_type}")
        else:
            print(f"Warning: Unknown message type '{msg_type}'. Use one of: {list(self.message_type_exclusions.keys())}")
    
    def remove_excluded_column(self, column_name, msg_type=None):
        """Remove a column from the exclusion list.
        
        Args:
            column_name: Name of column to remove from exclusions
            msg_type: If specified, remove only for this message type.
                     If None, remove from global exclusions.
        """
        if msg_type is None:
            if column_name in self.excluded_columns:
                self.excluded_columns.remove(column_name)
                print(f"Removed '{column_name}' from global excluded columns")
            else:
                print(f"'{column_name}' was not in global excluded columns")
        elif msg_type in self.message_type_exclusions:
            if column_name in self.message_type_exclusions[msg_type]:
                self.message_type_exclusions[msg_type].remove(column_name)
                print(f"Removed '{column_name}' from excluded columns for {msg_type}")
            else:
                print(f"'{column_name}' was not excluded for {msg_type}")
        else:
            print(f"Warning: Unknown message type '{msg_type}'")
    
    def set_excluded_columns(self, columns, msg_type=None):
        """Set the complete list of excluded columns.
        
        Args:
            columns: List or set of column names to exclude
            msg_type: If specified, set exclusions only for this message type.
                     If None, set global exclusions.
        """
        if msg_type is None:
            self.excluded_columns = set(columns)
            print(f"Set global excluded columns to: {self.excluded_columns}")
        elif msg_type in self.message_type_exclusions:
            self.message_type_exclusions[msg_type] = set(columns)
            print(f"Set excluded columns for {msg_type} to: {self.message_type_exclusions[msg_type]}")
        else:
            print(f"Warning: Unknown message type '{msg_type}'")
    
    def get_excluded_columns(self, msg_type=None):
        """Get the current list of excluded columns.
        
        Args:
            msg_type: If specified, get exclusions for this message type.
                     If None, get global exclusions.
        """
        if msg_type is None:
            return self.excluded_columns
        elif msg_type in self.message_type_exclusions:
            return self.excluded_columns.union(self.message_type_exclusions[msg_type])
        else:
            print(f"Warning: Unknown message type '{msg_type}'")
            return set()
    
    def show_all_exclusions(self):
        """Display all current exclusion settings."""
        print(f"\nGlobal excluded columns: {self.excluded_columns}")
        print("Message-type-specific exclusions:")
        for msg_type, exclusions in self.message_type_exclusions.items():
            if exclusions:
                print(f"  {msg_type}: {exclusions}")
            else:
                print(f"  {msg_type}: none")
        print()
    
    def apply_exclusions(self, data, msg_type):
        """Apply exclusions to a data dictionary."""
        if not isinstance(data, dict):
            return data
            
        # Get all exclusions for this message type
        type_specific_exclusions = self.message_type_exclusions.get(msg_type, set())
        all_exclusions = self.excluded_columns.union(type_specific_exclusions)
        
        if not all_exclusions:
            return data
            
        # Create new dictionary without excluded columns
        filtered_data = {}
        for key, value in data.items():
            if key not in all_exclusions:
                filtered_data[key] = value
                
        return filtered_data
    
    def parse_nav_timeutc(self, payload):
        """Parse UBX-NAV-TIMEUTC message."""
        if len(payload) < 20:
            return None
            
        data = {}
        data['iTOW'] = struct.unpack('<I', payload[0:4])[0]
        data['tAcc'] = struct.unpack('<I', payload[4:8])[0]
        data['nano'] = struct.unpack('<i', payload[8:12])[0]  # signed
        data['year'] = struct.unpack('<H', payload[12:14])[0]
        data['month'] = payload[14]
        data['day'] = payload[15]
        data['hour'] = payload[16]
        data['min'] = payload[17]
        data['sec'] = payload[18]
        data['valid'] = payload[19]
        
        # Parse validity flags from 'valid' byte (bit field)
        valid_byte = payload[19]
        data['validTOW'] = bool(valid_byte & 0x01)      # bit 0: validTOW
        data['validWKN'] = bool(valid_byte & 0x02)      # bit 1: validWKN  
        data['validUTC'] = bool(valid_byte & 0x04)      # bit 2: validUTC
        data['authStatus'] = (valid_byte & 0x30) >> 4   # bits 4-5: authStatus (0=none, 1=authenticated, 2=not authenticated)
        data['utcStandard'] = (valid_byte & 0xF0) >> 4  # bits 4-7: UTC standard (0=n/a, 1=CRL, 2=NIST, 3=USNO, 4=BIPM, 5=EU, 6=SU, 15=unknown)
        
        # Calculate UTC timestamp
        try:
            if (1 <= data['month'] <= 12 and 1 <= data['day'] <= 31 and 
                0 <= data['hour'] <= 23 and 0 <= data['min'] <= 59 and 
                0 <= data['sec'] <= 59):
                
                microsecond = 0
                if data['nano'] != 0:
                    microsecond = abs(data['nano']) // 1000
                    microsecond = min(999999, microsecond)
                
                utc_time = datetime(
                    data['year'], data['month'], data['day'],
                    data['hour'], data['min'], data['sec'],
                    microsecond, tzinfo=timezone.utc
                )
                data['timestamp_utc'] = utc_time.isoformat()
                
                # Apply exclusions
                data = self.apply_exclusions(data, 'NAV-TIMEUTC')
                return data, utc_time
        except:
            pass
            
        data['timestamp_utc'] = ''
        data = self.apply_exclusions(data, 'NAV-TIMEUTC')
        return data, None
    
    def parse_nav_hpposecef(self, payload):
        """Parse UBX-NAV-HPPOSECEF message."""
        if len(payload) < 28:
            return None
            
        data = {}
        data['version'] = payload[0]
        data['reserved1'] = struct.unpack('<I', payload[1:4] + b'\x00')[0]  # 3 bytes
        data['iTOW'] = struct.unpack('<I', payload[4:8])[0]
        data['ecefX'] = struct.unpack('<i', payload[8:12])[0]   # cm
        data['ecefY'] = struct.unpack('<i', payload[12:16])[0]  # cm
        data['ecefZ'] = struct.unpack('<i', payload[16:20])[0]  # cm
        data['ecefXHp'] = struct.unpack('<b', payload[20:21])[0]  # 0.1mm
        data['ecefYHp'] = struct.unpack('<b', payload[21:22])[0]  # 0.1mm
        data['ecefZHp'] = struct.unpack('<b', payload[22:23])[0]  # 0.1mm
        data['reserved2'] = payload[23]
        data['pAcc'] = struct.unpack('<I', payload[24:28])[0]   # 0.1mm
        
        # Parse validity flags (version-dependent)
        if data['version'] >= 1 and len(payload) >= 29:
            flags_byte = payload[28]
            data['invalidEcef'] = bool(flags_byte & 0x01)  # bit 0: invalidEcef
        else:
            data['invalidEcef'] = False  # Default for older versions
        
        # Calculate full-resolution coordinates in meters
        data['ecefX_full_m'] = (data['ecefX'] / 100.0) + (data['ecefXHp'] * 0.0001)
        data['ecefY_full_m'] = (data['ecefY'] / 100.0) + (data['ecefYHp'] * 0.0001)
        data['ecefZ_full_m'] = (data['ecefZ'] / 100.0) + (data['ecefZHp'] * 0.0001)
        
        if self.current_utc_time:
            data['timestamp_utc'] = self.current_utc_time.isoformat()
        else:
            data['timestamp_utc'] = ''
        
        # Apply exclusions
        data = self.apply_exclusions(data, 'NAV-HPPOSECEF')
        return data
    
    def parse_nav_hpposllh(self, payload):
        """Parse UBX-NAV-HPPOSLLH message."""
        if len(payload) < 36:
            return None
            
        data = {}
        data['version'] = payload[0]
        data['reserved1'] = struct.unpack('<I', payload[1:4] + b'\x00')[0]  # 3 bytes
        data['iTOW'] = struct.unpack('<I', payload[4:8])[0]
        data['lon'] = struct.unpack('<i', payload[8:12])[0]     # 1e-7 deg
        data['lat'] = struct.unpack('<i', payload[12:16])[0]    # 1e-7 deg
        data['height'] = struct.unpack('<i', payload[16:20])[0] # mm
        data['hMSL'] = struct.unpack('<i', payload[20:24])[0]   # mm
        data['lonHp'] = struct.unpack('<b', payload[24:25])[0]  # 1e-9 deg
        data['latHp'] = struct.unpack('<b', payload[25:26])[0]  # 1e-9 deg
        data['heightHp'] = struct.unpack('<b', payload[26:27])[0] # 0.1mm
        data['hMSLHp'] = struct.unpack('<b', payload[27:28])[0]   # 0.1mm
        data['hAcc'] = struct.unpack('<I', payload[28:32])[0]   # 0.1mm
        data['vAcc'] = struct.unpack('<I', payload[32:36])[0]   # 0.1mm
        
        # Parse validity flags (version-dependent)  
        if data['version'] >= 1 and len(payload) >= 37:
            flags_byte = payload[36]
            data['invalidLlh'] = bool(flags_byte & 0x01)  # bit 0: invalidLlh
        else:
            data['invalidLlh'] = False  # Default for older versions
        
        # Calculate full-resolution coordinates
        data['lon_full_deg'] = (data['lon'] * 1e-7) + (data['lonHp'] * 1e-9)
        data['lat_full_deg'] = (data['lat'] * 1e-7) + (data['latHp'] * 1e-9)
        data['height_full_m'] = (data['height'] / 1000.0) + (data['heightHp'] * 0.0001)
        data['hMSL_full_m'] = (data['hMSL'] / 1000.0) + (data['hMSLHp'] * 0.0001)
        
        if self.current_utc_time:
            data['timestamp_utc'] = self.current_utc_time.isoformat()
        else:
            data['timestamp_utc'] = ''
        
        # Apply exclusions
        data = self.apply_exclusions(data, 'NAV-HPPOSLLH')
        return data
    
    def parse_nav_clock(self, payload):
        """Parse UBX-NAV-CLOCK message."""
        if len(payload) < 20:
            return None
            
        data = {}
        data['iTOW'] = struct.unpack('<I', payload[0:4])[0]
        data['clkB'] = struct.unpack('<i', payload[4:8])[0]   # ns
        data['clkD'] = struct.unpack('<i', payload[8:12])[0]  # ns/s
        data['tAcc'] = struct.unpack('<I', payload[12:16])[0] # ns
        data['fAcc'] = struct.unpack('<I', payload[16:20])[0] # ps/s

        print(payload)

        if self.current_utc_time:
            data['timestamp_utc'] = self.current_utc_time.isoformat()
        else:
            data['timestamp_utc'] = ''
        
        # Apply exclusions
        data = self.apply_exclusions(data, 'NAV-CLOCK')
        return data
    
    def parse_rxm_rawx(self, payload):
        """Parse UBX-RXM-RAWX message (raw measurement data)."""
        if len(payload) < 16:
            return []
            
        # Parse header
        rcvTow = struct.unpack('<d', payload[0:8])[0]  # double
        week = struct.unpack('<H', payload[8:10])[0]
        leapS = struct.unpack('<b', payload[10:11])[0]
        numMeas = payload[11]
        recStat = payload[12]
        version = payload[13]
        reserved1 = struct.unpack('<H', payload[14:16])[0]
        
        measurements = []
        
        # Parse each measurement (32 bytes each)
        for i in range(numMeas):
            meas_offset = 16 + (i * 32)
            if meas_offset + 32 > len(payload):
                break
                
            meas_data = payload[meas_offset:meas_offset + 32]
            
            data = {
                'timestamp_utc': self.current_utc_time.isoformat() if self.current_utc_time else '',
                'rcvTow': rcvTow,
                'week': week,
                'leapS': leapS,
                'numMeas': numMeas,
                'recStat': recStat,
                'version': version,
                'reserved1': reserved1,
                'prMes': struct.unpack('<d', meas_data[0:8])[0],     # pseudorange (m)
                'cpMes': struct.unpack('<d', meas_data[8:16])[0],    # carrier phase (cycles)
                'doMes': struct.unpack('<f', meas_data[16:20])[0],   # doppler (Hz)
                'gnssId': meas_data[20],                             # GNSS identifier
                'svId': meas_data[21],                               # satellite identifier
                'sigId': meas_data[22],                              # signal identifier
                'freqId': meas_data[23],                             # frequency identifier
                'locktime': struct.unpack('<H', meas_data[24:26])[0], # carrier phase locktime
                'cno': meas_data[26],                                # C/N0 (dBHz)
                'prStdev': meas_data[27] & 0x0F,                     # pseudorange stdev
                'cpStdev': (meas_data[27] & 0xF0) >> 4,              # carrier phase stdev  
                'doStdev': meas_data[28] & 0x0F,                     # doppler stdev
                'trkStat': meas_data[29],                            # tracking status
                'reserved2': struct.unpack('<H', meas_data[30:32])[0]
            }
            
            # Parse tracking status flags from trkStat byte
            trk_stat = meas_data[29]
            data['prValid'] = bool(trk_stat & 0x01)      # bit 0: prValid (pseudorange valid)
            data['cpValid'] = bool(trk_stat & 0x02)      # bit 1: cpValid (carrier phase valid) 
            data['halfCyc'] = bool(trk_stat & 0x04)      # bit 2: halfCyc (half cycle valid)
            data['subHalfCyc'] = bool(trk_stat & 0x08)   # bit 3: subHalfCyc (sub half cycle valid)
            
            # Apply exclusions
            data = self.apply_exclusions(data, 'RXM-RAWX')
            measurements.append(data)
        
        return measurements
    
    def parse_rxm_sfrbx(self, payload):
        """Parse UBX-RXM-SFRBX message (subframe buffer)."""
        if len(payload) < 8:
            return None
            
        gnssId = payload[0]
        svId = payload[1]
        reserved1 = payload[2]
        freqId = payload[3]
        numWords = payload[4]
        chn = payload[5]
        version = payload[6]
        reserved2 = payload[7]
        
        # Extract subframe data words (4 bytes each)
        dwrd_data = []
        for i in range(numWords):
            word_offset = 8 + (i * 4)
            if word_offset + 4 <= len(payload):
                word = struct.unpack('<I', payload[word_offset:word_offset + 4])[0]
                dwrd_data.append(word)
        
        data = {
            'timestamp_utc': self.current_utc_time.isoformat() if self.current_utc_time else '',
            'gnssId': gnssId,
            'svId': svId,
            'reserved1': reserved1,
            'freqId': freqId,
            'numWords': numWords,
            'chn': chn,
            'version': version,
            'reserved2': reserved2,
            'dwrd': dwrd_data  # List of data words
        }
        
        # Apply exclusions
        data = self.apply_exclusions(data, 'RXM-SFRBX')
        return data
    
    def find_ubx_messages(self, data):
        """Find UBX messages in binary data."""
        messages = []
        i = 0
        
        while i < len(data) - 8:
            # Look for UBX sync bytes (0xB5, 0x62)
            if data[i] == 0xB5 and data[i+1] == 0x62:
                try:
                    msg_class = data[i+2]
                    msg_id = data[i+3]
                    length = struct.unpack('<H', data[i+4:i+6])[0]
                    
                    # Check if we have enough data for the complete message
                    total_length = 6 + length + 2  # header + payload + checksum
                    if i + total_length <= len(data):
                        payload = data[i+6:i+6+length]
                        
                        # Verify checksum
                        calc_checksum = self.calculate_checksum(data[i+2:i+6+length])
                        msg_checksum = struct.unpack('<H', data[i+6+length:i+8+length])[0]
                        
                        if calc_checksum == msg_checksum:
                            messages.append({
                                'class': msg_class,
                                'id': msg_id,
                                'length': length,
                                'payload': payload
                            })
                            i += total_length
                        else:
                            i += 1
                    else:
                        break
                except:
                    i += 1
            else:
                i += 1
                
        return messages
    
    def calculate_checksum(self, data):
        """Calculate UBX checksum."""
        ck_a = 0
        ck_b = 0
        for byte in data:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        return (ck_b << 8) | ck_a
    
    def parse_ubx_file(self):
        """Parse the UBX file and extract supported message types."""
        print(f"Starting to parse UBX file: {self.ubx_file_path}")
        
        if not self.ubx_file_path.exists():
            print(f"Error: File not found: {self.ubx_file_path}")
            return False
        
        try:
            with open(self.ubx_file_path, 'rb') as file:
                data = file.read()
            
            print(f"Read {len(data)} bytes from file")
            messages = self.find_ubx_messages(data)
            print(f"Found {len(messages)} UBX messages")
            
            supported_count = 0
            
            for msg in messages:
                msg_type = self.message_types.get((msg['class'], msg['id']))
                
                if msg_type == 'NAV-TIMEUTC':
                    result = self.parse_nav_timeutc(msg['payload'])
                    if result:
                        data, utc_time = result
                        if utc_time:
                            self.current_utc_time = utc_time
                        self.message_data['NAV-TIMEUTC'].append(data)
                        supported_count += 1
                
                elif msg_type == 'NAV-HPPOSECEF':
                    data = self.parse_nav_hpposecef(msg['payload'])
                    if data:
                        self.message_data['NAV-HPPOSECEF'].append(data)
                        supported_count += 1
                
                elif msg_type == 'NAV-HPPOSLLH':
                    data = self.parse_nav_hpposllh(msg['payload'])
                    if data:
                        self.message_data['NAV-HPPOSLLH'].append(data)
                        supported_count += 1
                
                elif msg_type == 'NAV-CLOCK':
                    data = self.parse_nav_clock(msg['payload'])
                    if data:
                        self.message_data['NAV-CLOCK'].append(data)
                        supported_count += 1
                
                elif msg_type == 'RXM-RAWX':
                    measurements = self.parse_rxm_rawx(msg['payload'])
                    if measurements:
                        # RXM-RAWX returns a list of measurements
                        for measurement in measurements:
                            self.message_data['RXM-RAWX'].append(measurement)
                            supported_count += 1
                
                elif msg_type == 'RXM-SFRBX':
                    data = self.parse_rxm_sfrbx(msg['payload'])
                    if data:
                        self.message_data['RXM-SFRBX'].append(data)
                        supported_count += 1
            
            print(f"Parsing complete. Processed {supported_count} supported messages.")
            return True
            
        except Exception as e:
            print(f"Error parsing UBX file: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def save_to_csv(self):
        """Save parsed message data to separate CSV files."""
        base_filename = self.ubx_file_path.stem
        
        for msg_type, data_list in self.message_data.items():
            if not data_list:
                print(f"No {msg_type} messages found, skipping CSV creation.")
                continue
                
            csv_filename = f"{base_filename}_{msg_type.replace('-', '_')}.csv"
            csv_path = self.output_dir / csv_filename
            
            try:
                # Handle special case for RXM-SFRBX with list data
                if msg_type == 'RXM-SFRBX':
                    # Flatten the dwrd lists into separate columns
                    processed_data = []
                    for item in data_list:
                        new_item = item.copy()
                        dwrd_list = new_item.pop('dwrd', [])
                        # Add each data word as a separate column
                        for i, word in enumerate(dwrd_list):
                            new_item[f'dwrd_{i:02d}'] = word
                        processed_data.append(new_item)
                    df = pd.DataFrame(processed_data)
                else:
                    df = pd.DataFrame(data_list)
                
                # Sort by timestamp if available
                if 'timestamp_utc' in df.columns and not df['timestamp_utc'].isna().all():
                    df = df.sort_values('timestamp_utc')
                
                # Apply custom column ordering
                desired_order = self.column_orders.get(msg_type, [])
                if desired_order:
                    available_ordered_cols = [col for col in desired_order if col in df.columns]
                    remaining_cols = [col for col in df.columns if col not in available_ordered_cols]
                    final_column_order = available_ordered_cols + remaining_cols
                    df = df[final_column_order]
                
                df.to_csv(csv_path, index=False)
                print(f"Saved {len(data_list)} {msg_type} messages to: {csv_path}")
                
            except Exception as e:
                print(f"Error saving {msg_type} data to CSV: {e}")
    
    def get_summary(self):
        """Get summary of parsed messages."""
        return {msg_type: len(data) for msg_type, data in self.message_data.items()}


def main():
    """Main function to run the UBX parser."""
    if len(sys.argv) < 2:
        print("Usage: python ubx_parser.py <ubx_file_path> [output_directory]")
        print("\nSupported message types:")
        print("  - UBX-NAV-TIMEUTC (for UTC timestamping)")
        print("  - UBX-RXM-RAWX")
        print("  - UBX-RXM-SFRBX") 
        print("  - UBX-NAV-CLOCK")
        print("  - UBX-NAV-HPPOSECEF (with full-resolution coordinates)")
        print("  - UBX-NAV-HPPOSLLH (with full-resolution coordinates)")
        sys.exit(1)
    
    ubx_file = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else None
    
    try:
        parser = ManualUBXParser(ubx_file, output_dir)
        
        # Example customizations (uncomment as needed)
        # parser.set_column_order('NAV-TIMEUTC', ['timestamp_utc', 'year', 'month', 'day'])
        # parser.add_excluded_column('reserved1')  # Exclude from all message types
        # parser.add_excluded_column('version', 'RXM-RAWX')  # Exclude only from RXM-RAWX
        # parser.show_all_exclusions()  # Show current exclusion settings
        
        if parser.parse_ubx_file():
            parser.save_to_csv()
            
            summary = parser.get_summary()
            print("\n=== PARSING SUMMARY ===")
            for msg_type, count in summary.items():
                if count > 0:
                    print(f"{msg_type}: {count} messages")
            print("=======================")
        else:
            print("Failed to parse UBX file.")
            sys.exit(1)
            
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()