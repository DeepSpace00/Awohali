#!/usr/bin/env python3
"""
UBX File Decoder to CSV
Converts UBX binary files (from ZEDF9P logger) to CSV format
Focuses on RAWX and HPPOSLLH messages with comprehensive data extraction

Dependencies: pip install pyubx2 pandas
"""

import argparse
import csv
import sys
from pathlib import Path
from datetime import datetime, timezone
import pandas as pd
from pyubx2 import UBXReader, UBX_MSGIDS

def decode_rawx_message(msg):
    """Extract RAWX message data into dictionary format"""
    import struct
    
    rawx_data = {
        'message_type': 'RAWX',
        'timestamp': datetime.now(timezone.utc).isoformat(),
        'iTOW': getattr(msg, 'iTOW', 0) / 1000.0,  # Convert ms to seconds
        'week': getattr(msg, 'week', 0),
        'leapS': getattr(msg, 'leapS', 0),
        'numMeas': getattr(msg, 'numMeas', 0),
        'recStat': getattr(msg, 'recStat', 0),
        'version': getattr(msg, 'version', 0)
    }
    
    # Parse the payload manually to get all measurement fields including standard deviations
    payload = getattr(msg, 'payload', b'')
    num_meas = getattr(msg, 'numMeas', 0)
    
    print(f"RAWX payload length: {len(payload)}, numMeas: {num_meas}")
    
    measurements = []
    
    if len(payload) >= 16 and num_meas > 0:  # RAWX header is 16 bytes, then measurements
        try:
            # Parse header (first 16 bytes)
            # rcvTow(8) + week(2) + leapS(1) + numMeas(1) + recStat(1) + version(1) + reserved(2)
            header = struct.unpack('<dHBBBBH', payload[:16])
            
            # Each measurement is 32 bytes
            measurement_size = 32
            expected_payload_size = 16 + (num_meas * measurement_size)
            
            print(f"Expected payload size: {expected_payload_size}, actual: {len(payload)}")
            
            if len(payload) >= expected_payload_size:
                for i in range(num_meas):
                    offset = 16 + (i * measurement_size)
                    meas_payload = payload[offset:offset + measurement_size]
                    
                    if len(meas_payload) >= 32:
                        # Unpack measurement data (32 bytes each)
                        # prMes(8) + cpMes(8) + doMes(4) + gnssId(1) + svId(1) + sigId(1) + freqId(1) + locktime(2) + cno(1) + prStdev(1) + cpStdev(1) + doStdev(1) + trkStat(1) + reserved(1)
                        meas_data = struct.unpack('<ddfBBBBHBBBBBB', meas_payload)
                        
                        meas_entry = {
                            'meas_index': i,
                            'prMes': meas_data[0],      # Pseudorange (m)
                            'cpMes': meas_data[1],      # Carrier phase (cycles)
                            'doMes': meas_data[2],      # Doppler (Hz)
                            'gnssId': meas_data[3],     # GNSS ID
                            'svId': meas_data[4],       # Satellite ID
                            'sigId': meas_data[5],      # Signal ID
                            'freqId': meas_data[6],     # Frequency ID
                            'locktime': meas_data[7],   # Lock time (ms)
                            'cno': meas_data[8],        # C/N0 (dB-Hz)
                            'prStdev': meas_data[9],    # Pseudorange std dev (encoded)
                            'cpStdev': meas_data[10],   # Carrier phase std dev (encoded)
                            'doStdev': meas_data[11],   # Doppler std dev (encoded)
                            'trkStat': meas_data[12],   # Tracking status
                        }
                        
                        if i < 3:  # Debug first 3 measurements
                            print(f"  Meas {i}: prStdev={meas_entry['prStdev']}, cpStdev={meas_entry['cpStdev']}, doStdev={meas_entry['doStdev']}")
                        
                        measurements.append({**rawx_data, **meas_entry})
                        
        except struct.error as e:
            print(f"Error unpacking RAWX payload: {e}")
            # Fallback to header data only
            measurements = [rawx_data]
    
    return measurements if measurements else [rawx_data]

def decode_hpposllh_message(msg):
    """Extract HPPOSLLH message data into dictionary format"""
    import struct
    
    # Parse the payload manually since pyubx2 doesn't extract high-precision fields
    payload = getattr(msg, 'payload', b'')
    
    print(f"HPPOSLLH payload length: {len(payload)}")
    print(f"HPPOSLLH payload hex: {payload.hex()}")
    
    if len(payload) >= 36:  # HPPOSLLH payload is 36 bytes
        # Unpack the binary payload according to UBX NAV-HPPOSLLH format
        try:
            # Correct format: version(1B) + reserved(3B) + iTOW(4B) + lon(4B) + lat(4B) + height(4B) + hMSL(4B) + lonHp(1b) + latHp(1b) + heightHp(1b) + hMSLHp(1b) + hAcc(4B) + vAcc(4B) = 36 bytes
            unpacked = struct.unpack('<BBBBLllllbbbbLL', payload)
            print(f"Unpacked values: {unpacked}")
            
            version = unpacked[0]
            reserved1_1 = unpacked[1] 
            reserved1_2 = unpacked[2]
            invalidLlh = unpacked[3]
            iTOW = unpacked[4]
            lon_raw = unpacked[5]      # 1e-7 degrees
            lat_raw = unpacked[6]      # 1e-7 degrees  
            height_raw = unpacked[7]   # mm
            hMSL_raw = unpacked[8]     # mm
            lonHp_raw = unpacked[9]    # 1e-9 degrees (signed byte)
            latHp_raw = unpacked[10]   # 1e-9 degrees (signed byte)
            heightHp_raw = unpacked[11] # 0.1 mm (signed byte)
            hMSLHp_raw = unpacked[12]  # 0.1 mm (signed byte)
            hAcc_raw = unpacked[13]    # 0.1 mm
            vAcc_raw = unpacked[14]    # 0.1 mm
            
            print(f"High precision values: lonHp={lonHp_raw}, latHp={latHp_raw}, heightHp={heightHp_raw}, hMSLHp={hMSLHp_raw}")
            
        except struct.error as e:
            print(f"Error unpacking HPPOSLLH payload: {e}")
            # Fallback to pyubx2 parsed values without HP fields
            return [{
                'message_type': 'HPPOSLLH',
                'timestamp': datetime.now(timezone.utc).isoformat(),
                'version': getattr(msg, 'version', 0),
                'invalidLlh': getattr(msg, 'invalidLlh', 0),
                'iTOW': getattr(msg, 'iTOW', 0) / 1000.0,
                'lon': getattr(msg, 'lon', 0.0),
                'lat': getattr(msg, 'lat', 0.0),
                'height': getattr(msg, 'height', 0.0) / 1000.0,
                'hMSL': getattr(msg, 'hMSL', 0.0) / 1000.0,
                'lonHp': 0.0,  # Not available from pyubx2
                'latHp': 0.0,  # Not available from pyubx2
                'heightHp': 0.0,  # Not available from pyubx2
                'hMSLHp': 0.0,  # Not available from pyubx2
                'hAcc': getattr(msg, 'hAcc', 0.0) / 1000.0,
                'vAcc': getattr(msg, 'vAcc', 0.0) / 1000.0,
                'lon_full': getattr(msg, 'lon', 0.0),
                'lat_full': getattr(msg, 'lat', 0.0),
                'height_full': getattr(msg, 'height', 0.0) / 1000.0,
            }]
    else:
        print("Payload too short, using pyubx2 values")
        # Use pyubx2 parsed values
        lon_raw = int(getattr(msg, 'lon', 0.0) * 1e7)
        lat_raw = int(getattr(msg, 'lat', 0.0) * 1e7)
        height_raw = int(getattr(msg, 'height', 0.0) * 1000)
        hMSL_raw = int(getattr(msg, 'hMSL', 0.0) * 1000)
        lonHp_raw = 0
        latHp_raw = 0
        heightHp_raw = 0
        hMSLHp_raw = 0
        hAcc_raw = int(getattr(msg, 'hAcc', 0.0) * 10)
        vAcc_raw = int(getattr(msg, 'vAcc', 0.0) * 10)
        iTOW = getattr(msg, 'iTOW', 0)
        version = getattr(msg, 'version', 0)
        invalidLlh = getattr(msg, 'invalidLlh', 0)
    
    return [{
        'message_type': 'HPPOSLLH',
        'timestamp': datetime.now(timezone.utc).isoformat(),
        'version': version,
        'invalidLlh': invalidLlh,
        'iTOW': iTOW / 1000.0,  # Convert ms to seconds
        'lon': lon_raw * 1e-7,      # Convert to degrees
        'lat': lat_raw * 1e-7,      # Convert to degrees
        'height': height_raw / 1000.0,  # Convert mm to m
        'hMSL': hMSL_raw / 1000.0,      # Convert mm to m
        'lonHp': lonHp_raw * 1e-9,      # High precision part (1e-9 degrees)
        'latHp': latHp_raw * 1e-9,      # High precision part (1e-9 degrees)
        'heightHp': heightHp_raw * 0.1, # Convert to mm (0.1 mm resolution)
        'hMSLHp': hMSLHp_raw * 0.1,     # Convert to mm (0.1 mm resolution)  
        'hAcc': hAcc_raw * 0.1,         # Convert 0.1mm to mm
        'vAcc': vAcc_raw * 0.1,         # Convert 0.1mm to mm
        # Calculate full precision coordinates
        'lon_full': (lon_raw * 1e-7) + (lonHp_raw * 1e-9),
        'lat_full': (lat_raw * 1e-7) + (latHp_raw * 1e-9),
        'height_full': (height_raw / 1000.0) + (heightHp_raw * 0.0001),
    }]

def decode_pvt_message(msg):
    """Extract PVT message data (if present) for time reference"""
    return [{
        'message_type': 'PVT',
        'timestamp': datetime.now(timezone.utc).isoformat(),
        'iTOW': getattr(msg, 'iTOW', 0) / 1000.0,
        'year': getattr(msg, 'year', 0),
        'month': getattr(msg, 'month', 0),
        'day': getattr(msg, 'day', 0),
        'hour': getattr(msg, 'hour', 0),
        'min': getattr(msg, 'min', 0),
        'sec': getattr(msg, 'sec', 0),
        'valid': getattr(msg, 'valid', 0),
        'tAcc': getattr(msg, 'tAcc', 0),
        'nano': getattr(msg, 'nano', 0),
        'fixType': getattr(msg, 'fixType', 0),
        'flags': getattr(msg, 'flags', 0),
        'numSV': getattr(msg, 'numSV', 0),
        'lon': getattr(msg, 'lon', 0) * 1e-7,
        'lat': getattr(msg, 'lat', 0) * 1e-7,
        'height': getattr(msg, 'height', 0) / 1000.0,
        'hMSL': getattr(msg, 'hMSL', 0) / 1000.0,
        'hAcc': getattr(msg, 'hAcc', 0) / 1000.0,
        'vAcc': getattr(msg, 'vAcc', 0) / 1000.0,
    }]

def process_ubx_file(input_file, output_file=None, message_filter=None, separate_files=False):
    """
    Process UBX file and convert to CSV
    
    Args:
        input_file: Path to input UBX file
        output_file: Path to output CSV file (optional)
        message_filter: List of message types to include (optional)
        separate_files: If True, create separate CSV files for each message type
    """
    input_path = Path(input_file)
    
    if not input_path.exists():
        print(f"Error: Input file '{input_file}' not found")
        return False
    
    # Generate output filename if not provided
    if output_file is None:
        if separate_files:
            output_base = input_path.stem
        else:
            output_file = input_path.with_suffix('.csv')
    else:
        if separate_files:
            output_base = Path(output_file).stem
        else:
            output_file = Path(output_file)
    
    print(f"Processing UBX file: {input_path}")
    
    # Default message filter
    if message_filter is None:
        message_filter = ['RAWX', 'HPPOSLLH', 'PVT']
    
    # Separate data containers for each message type
    rawx_data = []
    hpposllh_data = []
    pvt_data = []
    
    message_counts = {}
    error_count = 0
    
    try:
        with open(input_path, 'rb') as file:
            ubx_reader = UBXReader(file, quitonerror=False)
            
            print("Decoding UBX messages...")
            
            for raw_data, parsed_msg in ubx_reader:
                try:
                    if parsed_msg is None:
                        continue
                    
                    msg_type = None
                    decoded_data = []
                    
                    # Identify message type and decode
                    if hasattr(parsed_msg, 'identity'):
                        msg_id = parsed_msg.identity
                        
                        if msg_id == 'RXM-RAWX' and 'RAWX' in message_filter:
                            msg_type = 'RAWX'
                            decoded_data = decode_rawx_message(parsed_msg)
                            rawx_data.extend(decoded_data)
                        elif msg_id == 'NAV-HPPOSLLH' and 'HPPOSLLH' in message_filter:
                            msg_type = 'HPPOSLLH'
                            # Add debug output for HPPOSLLH messages
                            print(f"\n--- HPPOSLLH Message Debug ---")
                            print(f"Message attributes:")
                            for attr in sorted(dir(parsed_msg)):
                                if not attr.startswith('_'):
                                    try:
                                        value = getattr(parsed_msg, attr)
                                        print(f"  {attr}: {type(value)} = {value}")
                                    except Exception as e:
                                        print(f"  {attr}: <error: {e}>")
                            print("--- End HPPOSLLH Debug ---\n")
                            
                            decoded_data = decode_hpposllh_message(parsed_msg)
                            hpposllh_data.extend(decoded_data)
                        elif msg_id == 'NAV-PVT' and 'PVT' in message_filter:
                            msg_type = 'PVT'
                            decoded_data = decode_pvt_message(parsed_msg)
                            pvt_data.extend(decoded_data)
                    
                    if msg_type and decoded_data:
                        message_counts[msg_type] = message_counts.get(msg_type, 0) + 1
                        
                        total_records = len(rawx_data) + len(hpposllh_data) + len(pvt_data)
                        if total_records % 1000 == 0 and total_records > 0:
                            print(f"Processed {total_records} records...")
                
                except Exception as e:
                    error_count += 1
                    if error_count <= 10:  # Only print first 10 errors
                        print(f"Error processing message: {e}")
    
    except Exception as e:
        print(f"Error reading UBX file: {e}")
        return False
    
    # Summary
    total_records = len(rawx_data) + len(hpposllh_data) + len(pvt_data)
    print(f"\nProcessing complete!")
    print(f"Total records extracted: {total_records}")
    print("Message counts:")
    for msg_type, count in message_counts.items():
        print(f"  {msg_type}: {count}")
    if error_count > 0:
        print(f"Errors encountered: {error_count}")
    
    # Debug: Check HP and Stdev values in the processed data
    print(f"\n=== DEBUG: High Precision and Standard Deviation Values ===")
    
    if hpposllh_data:
        print(f"HPPOSLLH High Precision Values (first 5 records):")
        for i, record in enumerate(hpposllh_data[:5]):
            print(f"  Record {i+1}: lonHp={record.get('lonHp', 'MISSING')}, latHp={record.get('latHp', 'MISSING')}, heightHp={record.get('heightHp', 'MISSING')}, hMSLHp={record.get('hMSLHp', 'MISSING')}")
    
    if rawx_data:
        print(f"RAWX Standard Deviation Values (first 5 records):")
        for i, record in enumerate(rawx_data[:5]):
            print(f"  Record {i+1}: prStdev={record.get('prStdev', 'MISSING')}, cpStdev={record.get('cpStdev', 'MISSING')}, doStdev={record.get('doStdev', 'MISSING')}")
    
    print(f"=== END DEBUG ===\n")
    
    # Write to CSV files
    success = False
    if separate_files:
        # Create separate files for each message type
        files_created = []
        
        if rawx_data:
            rawx_file = f"{output_base}_rawx.csv"
            try:
                df_rawx = pd.DataFrame(rawx_data)
                df_rawx.to_csv(rawx_file, index=False, float_format='%.9f')
                files_created.append(rawx_file)
                print(f"RAWX CSV file created: {rawx_file} ({len(rawx_data)} records)")
                print(f"RAWX columns: {list(df_rawx.columns)}")
                success = True
            except Exception as e:
                print(f"Error writing RAWX CSV file: {e}")
        
        if hpposllh_data:
            hpposllh_file = f"{output_base}_hpposllh.csv"
            try:
                df_hppos = pd.DataFrame(hpposllh_data)
                df_hppos.to_csv(hpposllh_file, index=False, float_format='%.9f')
                files_created.append(hpposllh_file)
                print(f"HPPOSLLH CSV file created: {hpposllh_file} ({len(hpposllh_data)} records)")
                print(f"HPPOSLLH columns: {list(df_hppos.columns)}")
                success = True
            except Exception as e:
                print(f"Error writing HPPOSLLH CSV file: {e}")
        
        if pvt_data:
            pvt_file = f"{output_base}_pvt.csv"
            try:
                df_pvt = pd.DataFrame(pvt_data)
                df_pvt.to_csv(pvt_file, index=False, float_format='%.9f')
                files_created.append(pvt_file)
                print(f"PVT CSV file created: {pvt_file} ({len(pvt_data)} records)")
                print(f"PVT columns: {list(df_pvt.columns)}")
                success = True
            except Exception as e:
                print(f"Error writing PVT CSV file: {e}")
        
        if not files_created:
            print("No data files created - no matching messages found")
            return False
            
    else:
        # Create single combined file
        all_data = rawx_data + hpposllh_data + pvt_data
        if all_data:
            try:
                df = pd.DataFrame(all_data)
                df.to_csv(output_file, index=False, float_format='%.9f')
                print(f"\nCombined CSV file created: {output_file}")
                print(f"CSV columns: {list(df.columns)}")
                success = True
            except Exception as e:
                print(f"Error writing combined CSV file: {e}")
                return False
        else:
            print("No data extracted from UBX file")
            return False
    
    return success

def main():
    parser = argparse.ArgumentParser(
        description="Convert UBX binary files to CSV format",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python ubx_decoder.py GNSS001.ubx
  python ubx_decoder.py GNSS001.ubx -o output.csv
  python ubx_decoder.py GNSS001.ubx -m RAWX HPPOSLLH
  python ubx_decoder.py GNSS001.ubx -m RAWX --output rawx_only.csv
  python ubx_decoder.py GNSS001.ubx --separate
  python ubx_decoder.py GNSS001.ubx --separate -o my_data
        """
    )
    
    parser.add_argument('input_file', help='Input UBX file path')
    parser.add_argument('-o', '--output', help='Output CSV file path (optional)')
    parser.add_argument('-m', '--messages', nargs='+', 
                       choices=['RAWX', 'HPPOSLLH', 'PVT'],
                       help='Message types to extract (default: all)')
    parser.add_argument('--separate', action='store_true',
                       help='Create separate CSV files for each message type')
    
    args = parser.parse_args()
    
    success = process_ubx_file(
        input_file=args.input_file,
        output_file=args.output,
        message_filter=args.messages,
        separate_files=args.separate
    )
    
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()