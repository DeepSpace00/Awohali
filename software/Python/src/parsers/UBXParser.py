"""
UBX File Parser and CSV Splitter - Manual Parser

This script manually decodes .ubx files and splits different UBX message types into separate CSV files.
Supports UBX-NAV-TIMEUTC, UBX-RXM-RAWX, UBX-RXM-SFRBX, UBX-NAV-CLOCK, 
UBX-NAV-HPPOSECEF, and UBX-NAV-HPPOSLLH messages.
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
            'MON-COMMS': [],
            'MON-HW3': [],
            'MON-RF': [],
            'MON-SPAN': [],
            'MON-SYS': [],
            'NAV-CLOCK': [],
            'NAV-DOP': [],
            'NAV-HPPOSECEF': [],
            'NAV-HPPOSLLH': [],
            'NAV-PVT': [],
            'NAV-SAT': [],
            'NAV-SIG': [],
            'NAV-STATUS': [],
            'NAV-TIMEGAL': [],
            'NAV-TIMEGPS': [],
            'NAV-TIMEUTC': [],
            'RXM-MEASX': [],
            'RXM-RAWX': [],
            'RXM-SFRBX': [],
        }
        
        # Current UTC time for timestamping
        self.current_utc_time = None
        
        # Exclusion functionality
        self.excluded_columns = {'msgVer', 'version', 'reserved0', 'reserved1', 'reserved2', 'reserved3',
                                 'reserved4'} #Global
        # exclusions
        self.message_type_exclusions = {
            'MON-COMMS': set(),
            'MON-HW3': set(),
            'MON-RF': set(),
            'MON-SPAN': set(),
            'MON-SYS': set(),
            'NAV-CLOCK': set(),
            'NAV-DOP': set(),
            'NAV-HPPOSECEF': set(),
            'NAV-HPPOSLLH': set(),
            'NAV-PVT': set(),
            'NAV-SAT': set(),
            'NAV-SIG': set(),
            'NAV-STATUS': set(),
            'NAV-TIMEGAL': set(),
            'NAV-TIMEGPS': set(),
            'NAV-TIMEUTC': set(),
            'RXM-MEASX': set(),
            'RXM-RAWX': set(),
            'RXM-SFRBX': set(),
        }
        
        # Column orders
        self.column_orders = self._get_default_column_orders()
        if column_orders:
            self.column_orders.update(column_orders)
        
        # UBX message class and ID mappings
        self.message_types = {
            (0x0a, 0x36): 'MON-COMMS',
            (0x0a, 0x37): 'MON-HW3',
            (0x0a, 0x38): 'MON-RF',
            (0x0a, 0x31): 'MON-SPAN',
            (0x0a, 0x39): 'MON-SYS',
            (0x01, 0x22): 'NAV-CLOCK',
            (0x01, 0x04): 'NAV-DOP',
            (0x01, 0x13): 'NAV-HPPOSECEF',
            (0x01, 0x14): 'NAV-HPPOSLLH',
            (0x01, 0x07): 'NAV-PVT',
            (0x01, 0x35): 'NAV-SAT',
            (0x01, 0x43): 'NAV-SIG',
            (0x01, 0x03): 'NAV-STATUS',
            (0x01, 0x25): 'NAV-TIMEGAL',
            (0x01, 0x20): 'NAV-TIMEGPS',
            (0x01, 0x21): 'NAV-TIMEUTC',
            (0x02, 0x14): 'RXM-MEASX',
            (0x02, 0x15): 'RXM-RAWX', 
            (0x02, 0x13): 'RXM-SFRBX'
        }
        
        print(f"Initialization complete.")

    def _get_default_column_orders(self):
        """Define default column orders for each message type."""
        return {
            'MON-COMMS': [
                'version', 'nPorts', 'mem', 'alloc', 'outputPort', 'reserved0',
                'protIds', 'portId', 'txPending', 'txBytes', 'txUsage',
                'txPeakUsage', 'rxPending', 'rxBytes', 'rxUsage', 'rxPeakUsage',
                'overrunErrs', 'msgs', 'reserved1', 'skipped'
            ],
            'MON-HW3': [
                'version', 'nPins', 'rtcCalib', 'safeBoot', 'xtalAbsent',
                'hwVersion', 'reserved0', 'reserved1', 'pinId', 'pinMask',
                'periphPIO', 'pinBank', 'direction', 'value', 'vpManager',
                'pioIrq', 'pioPullHigh', 'pioPullLow', 'VP', 'reserved2'
            ],
            'MON-RF': [
                'version', 'nBlocks', 'reserved0', 'blockId', 'jammingState',
                'antStatus', 'antPower', 'postStatus', 'reserved1', 'noisePerMS',
                'agcCnt', 'cwSuppression', 'ofsI', 'magI', 'ofsQ', 'magQ',
                'rfBlockGnssBand', 'reserved2'
            ],
            'MON-SPAN': [
                'version', 'numRfBlocks', 'reserved0', 'spectrum', 'span', 'res',
                'center', 'pga', 'reserved1'
            ],
            'MON-SYS': [
                'msgVer', 'bootType', 'cpuLoad', 'cpuLoadMax', 'memUsage',
                'memUsageMax', 'ioUsage', 'ioUsageMax', 'runTime', 'noticeCount',
                'warnCount', 'errorCount', 'tempValue', 'reserved0'
            ],
            'NAV-CLOCK': [
                'iTOW', 'clkB', 'clkD', 'tAcc', 'fAcc'
            ],
            'NAV-DOP': [
                'iTOW', 'gDOP', 'pDOP', 'tDOP', 'vDOP', 'hDOP', 'nDOP', 'eDOP'
            ],
            'NAV-HPPOSECEF': [
                'version', 'reserved0', 'iTOW', 'ecefX', 'ecefY',
                'ecefZ', 'ecefXHp', 'ecefYHp', 'ecefZHp', 'invalidEcef', 'pAcc',
            ],
            'NAV-HPPOSLLH': [
                'version', 'reserved0', 'invalidL1h', 'iTOW', 'lon', 'lat',
                'height', 'hMSL', 'lonHp', 'latHp', 'heightHp', 'hMSLHp',
                'hAcc', 'vAcc'
            ],
            'NAV-PVT': [
                'iTOW', 'year', 'month', 'day', 'hour', 'min', 'sec', 'validDate',
                'validTime', 'fullyResolved', 'validMsg', 'tAcc', 'nano', 'fixType',
                'gnssFixOK', 'diffSoln', 'psmState', 'headVehValid', 'carrSoln',
                'confirmedAvai', 'confirmedDate', 'confirmedTime', 'numSV', 'lon',
                'lat', 'height', 'hMSL', 'hAcc', 'vAcc', 'velN', 'venE', 'velD',
                'gSpeed', 'headMot', 'sAcc', 'headAcc', 'pDOP', 'invalidL1h',
                'lastCorrectionAge', 'authTime', 'nmaFixStatus', 'reserved0',
                'headVeh', 'magDec', 'magAcc'
            ],
            'NAV-SAT': [
                'iTOW', 'version', 'numSvs', 'reserved0', 'gnssId', 'svId', 'cno',
                'elev', 'azim', 'prRes', 'qualityInd', 'svUsed', 'health',
                'diffCorr', 'smoothed', 'orbitSource', 'ephAvail', 'almAvail',
                'anoAvail', 'aopAvail', 'sbasCorrUsed', 'rtcmCorrUsed',
                'slasCorrUsed', 'spartnCorrUsed', 'prCorrUsed', 'crCorrUsed',
                'doCorrUsed', 'clasCorrUsed', 'lppCorrUsed', 'hasCorrUsed'
            ],
            'NAV-SIG': [
                'iTOW', 'version', 'numSigs', 'reserved0', 'gnssId', 'svId',
                'freqId', 'prRes', 'cno', 'qualityInd', 'corrSource', 'ionoModel',
                'health', 'prSmoothed', 'prUsed', 'crUsed', 'doUsed', 'prCorrUsed',
                'crCorrUsed', 'doCorrUsed', 'authStatus', 'reserved1'
            ],
            'NAV-STATUS': [
                'iTOW', 'gpsFix', 'gpsFixOk', 'diffSoln', 'wknSet', 'towSet',
                'fixStat', 'diffCorr', 'carrSolnValid', 'mapMatching', 'psmState',
                'spoofDetState', 'carrSoln', 'ttff', 'msss'
            ],
            'NAV-TIMEGAL': [
                'iTOW', 'galTow', 'fGalTow', 'galWno', 'leapS', 'galTowValid',
                'galWnoValid', 'leapSValid', 'tAcc'
            ],
            'NAV-TIMEGPS': [
                'iTOW', 'fTOW', 'week', 'leapS', 'towValid', 'weekValid',
                'leapSValid', 'tAcc'
            ],
            'NAV-TIMEUTC': [
                'iTOW', 'tAcc', 'nano', 'year', 'month', 'day',
                'hour', 'min', 'sec', 'validTOW', 'validWKN', 'validUTC',
                'authStatus', 'utcStandard'
            ],
            'RXM-MEASX': [
                'version', 'reserved0', 'gpsTOW', 'gloTOW', 'bdsTOW', 'reserved1', 'qzssTOW',
                'gpsTOWacc', 'gloTOWacc', 'bdsTOWacc', 'reserved2', 'qzssTOWacc',
                'numSV', 'towSet', 'reserved3', 'gnssId', 'svId', 'cNo', 'mpathIndic',
                'dopplerMS', 'dopplerHz', 'wholeChips', 'fracChips', 'codePhase',
                'intCodePhase', 'pseuRangeRMSErr', 'reserved4'
            ],
            'RXM-RAWX': [
                'rcvTow', 'week', 'leapS', 'numMeas', 'leapSec', 'clkReset',
                'version', 'reserved0', 'prMes', 'cpMes', 'doMes', 'gnssId',
                'svId', 'sigId', 'freqId', 'locktime', 'cno', 'prStdev',
                'cpStdev', 'doStdev', 'trkStat', 'prValid', 'cpValid', 'halfCyc',
                'subHalfCyc', 'reserved1'
            ],
            'RXM-SFRBX': [
                'gnssId', 'svId', 'sigId', 'freqId', 'numWords'
                'chn', 'version', 'reserved0', 'dwrd'
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

    def parse_mon_comms(self, payload):
        """Parse UBX-MON-COMMS message."""
        if len(payload) < 8:
            return []

        version = payload[0]
        nPorts = payload[1]
        txErrors = payload[2]
        reserved0 = payload[3]
        protIds = struct.unpack('<I', payload[4:8])[0]

        mem = bool(txErrors & 0x01)
        alloc = bool(txErrors & 0x02)
        outputPort = (txErrors & 0x03) >> 2

        measurements = []

        for i in range(nPorts):
            meas_offset = 8 + (i * 40)
            if meas_offset + 40 > len(payload):
                break

            meas_data = payload[meas_offset:meas_offset + 40]

            data = {
                'version': version,
                'nPorts': nPorts,
                'mem': mem,
                'alloc': alloc,
                'outputPort': outputPort,
                'reserved0': reserved0,
                'protIds': protIds,
                'portId': struct.unpack('<H', meas_data[0:2])[0],
                'txPending': struct.unpack('<H', meas_data[2:4])[0],
                'txBytes': struct.unpack('<I', meas_data[4:8])[0],
                'txUsage': meas_data[8],
                'txPeakUsage': meas_data[9],
                'rxPending': struct.unpack('<H', meas_data[10:12])[0],
                'rxBytes': struct.unpack('<I', meas_data[12:16])[0],
                'rxUsage': meas_data[16],
                'rxPeakUsage': meas_data[17],
                'overrunErrs': struct.unpack('<H', meas_data[18:20])[0],
                'msgs': struct.unpack('<d', meas_data[20:28])[0],
                'reserved1': struct.unpack('<d', meas_data[28:36])[0],
                'skipped': struct.unpack('<I', meas_data[36:40])[0],
            }

            data = self.apply_exclusions(data, 'MON-COMMS')
            measurements.append(data)

        return measurements

    def parse_mon_hw3(self, payload):
        """ Parse UBX-MON-HW3 message."""
        if len(payload) < 22:
            return []

        version = payload[0]
        nPins = payload[1]
        flags = payload[2]
        hwVersion = struct.unpack('10B', payload[3:13])[0]
        reserved0 = payload[13]

        rtcCalib = bool(flags & 0x01)
        safeBoot = bool(flags & 0x02)
        xtalAbsent = bool(flags & 0x03)

        measurements = []

        for i in range(nPins):
            meas_offset = 22 + (i * 6)
            if meas_offset + 6 > len(payload):
                break

            meas_data = payload[meas_offset:meas_offset + 6]

            pinMask = struct.unpack('<H', meas_data[2:4])[0]
            periphPIO = bool(pinMask & 0x01)
            pinBank = (pinMask & 0x02) >> 3
            direction = bool(pinMask & 0x05)
            value = bool(pinMask & 0x06)
            vpManager = bool(pinMask & 0x07)
            pioIrq = bool(pinMask & 0x08)
            pioPullHigh = bool(pinMask & 0x09)
            pioPullLow = bool(pinMask & 0x0A)

            data = {
                'version': version,
                'nPins': nPins,
                'rtcCalib': rtcCalib,
                'safeBoot': safeBoot,
                'xtalAbsent': xtalAbsent,
                'hwVersion': hwVersion,
                'reserved0': reserved0,
                'reserved1': meas_data[0],
                'pinId': meas_data[1],
                'periphPIO': periphPIO,
                'pinBank': pinBank,
                'direction': direction,
                'value': value,
                'vpManager': vpManager,
                'pioIrq': pioIrq,
                'pioPullHigh': pioPullHigh,
                'pioPullLow': pioPullLow,
                'VP': meas_data[4],
                'reserved2': meas_data[5]
            }

            data = self.apply_exclusions(data, 'MON-HW3')
            measurements.append(data)

        return measurements

    def parse_mon_rf(self, payload):
        """Parse UBX-MON-RF message."""
        if len(payload) < 4:
            return []

        # Parse header
        version = payload[0]
        nBlocks = payload[1]
        reserved0 = struct.unpack('<H', payload[2:4])[0]

        measurements = []

        for i in range(nBlocks):
            meas_offset = 4 + (i * 24)
            if meas_offset + 24 > len(payload):
                break

            meas_data = payload[meas_offset:meas_offset + 24]

            flags = meas_data[1]
            jammingState = (flags & 0x01) >> 2

            data = {
                'version': version,
                'nBlocks': nBlocks,
                'reserved0': reserved0,
                'blockId': meas_data[0],
                'jammingState': jammingState,
                'antStatus': meas_data[2],
                'antPower': meas_data[3],
                'postStatus': struct.unpack('<I', payload[4:8])[0],
                'reserved1': struct.unpack('<I', payload[8:12])[0],
                'noisePerMS': struct.unpack('<H', payload[12:14])[0],
                'agcCnt': struct.unpack('<H', payload[14:16])[0],
                'cwSuppression': meas_data[16],
                'ofsI': meas_data[17],
                'magI': meas_data[18],
                'ofsQ': meas_data[19],
                'magQ': meas_data[20],
                'rfBlockGnssBand': meas_data[21],
                'reserved2': struct.unpack('<H', payload[20:22])[0],
            }

            data = self.apply_exclusions(data, 'MON-RF')
            measurements.append(data)

        return measurements

    def parse_mon_span(self, payload):
        """Parse UBX-MON-SPAN message."""
        if len(payload) < 4:
            return []

        version = payload[0]
        numRfBlocks = payload[1]
        reserved0 = struct.unpack('<H', payload[2:4])[0]

        measurements = []
        for i in range(numRfBlocks):
            meas_offset = 4 + (i * 272)
            if meas_offset + 272 > len(payload):
                break

            meas_data = payload[meas_offset:meas_offset + 272]

            data = {
                'version': version,
                'numRfBlocks': numRfBlocks,
                'reserved0': reserved0,
                'spectrum': struct.unpack('256B', meas_data[0:256])[0],
                'span': struct.unpack('<I', meas_data[256:260])[0],
                'res': struct.unpack('<I', meas_data[260:264])[0],
                'center': struct.unpack('<I', meas_data[264:268])[0],
                'pga': meas_data[268],
                'reserved1': struct.unpack('3B', meas_data[269:272])[0]
            }

            data = self.apply_exclusions(data, 'MON-SPAN')
            measurements.append(data)

        return measurements

    def parse_mon_sys(self, payload):
        """Parse UBX-MON-SYS message."""
        if len(payload) < 19:
            return None

        data = {
            'msgVer': payload[0],
            'bootType': payload[1],
            'cpuLoad': payload[2],
            'cpuLoadMax': payload[3],
            'memUsage': payload[4],
            'memUsageMax': payload[5],
            'ioUsage': payload[6],
            'ioUsageMax': payload[7],
            'runTime': struct.unpack('<I', payload[8:12])[0],
            'noticeCount': struct.unpack('<H', payload[12:14])[0],
            'warnCount': struct.unpack('<H', payload[14:16])[0],
            'errorCount': struct.unpack('<H', payload[16:18])[0],
            'tempValue': payload[18],
            'reserved0': struct.unpack('5B', payload[19:25])[0]
        }

        data = self.apply_exclusions(data, 'MON-SYS')
        return data

    def parse_nav_clock(self, payload):
        """Parse UBX-NAV-CLOCK message."""
        if len(payload) < 20:
            return None

        data = {
            'iTOW': struct.unpack('<I', payload[0:4])[0],
            'clkB': struct.unpack('<i', payload[4:8])[0],
            'clkD': struct.unpack('<i', payload[8:12])[0],
            'tAcc': struct.unpack('<I', payload[12:16])[0],
            'fAcc': struct.unpack('<I', payload[16:20])[0]
        }

        # Apply exclusions
        data = self.apply_exclusions(data, 'NAV-CLOCK')
        return data

    def parse_nav_dop(self, payload):
        """Parse UBX-NAV-DOP message."""
        if len(payload) < 18:
            return None

        data = {
            'iTOW': struct.unpack('<I', payload[0:4])[0],
            'gDOP': struct.unpack('<H', payload[4:6])[0],
            'pDOP': struct.unpack('<H', payload[6:8])[0],
            'tDOP': struct.unpack('<H', payload[8:10])[0],
            'vDOP': struct.unpack('<H', payload[10:12])[0],
            'hDOP': struct.unpack('<H', payload[12:14])[0],
            'nDOP': struct.unpack('<H', payload[14:16])[0],
            'eDOP': struct.unpack('<H', payload[16:18])[0],
        }

        data = self.apply_exclusions(data, 'NAV-DOP')
        return data

    def parse_nav_hpposecef(self, payload):
        """Parse UBX-NAV-HPPOSECEF message."""
        if len(payload) < 28:
            return None

        data = {
            'version': payload[0],
            'reserved1': struct.unpack('<I', payload[1:4] + b'\x00')[0],
            'iTOW': struct.unpack('<I', payload[4:8])[0],
            'ecefX': struct.unpack('<i', payload[8:12])[0],
            'ecefY': struct.unpack('<i', payload[12:16])[0],
            'ecefZ': struct.unpack('<i', payload[16:20])[0],
            'ecefXHp': struct.unpack('<b', payload[20:21])[0],
            'ecefYHp': struct.unpack('<b', payload[21:22])[0],
            'ecefZHp': struct.unpack('<b', payload[22:23])[0],
            'reserved2': payload[23],
            'pAcc': struct.unpack('<I', payload[24:28])[0]
        }

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

        data = {
            'version': payload[0],
            'reserved1': struct.unpack('<I', payload[1:4] + b'\x00')[0],
            'iTOW': struct.unpack('<I', payload[4:8])[0],
            'lon': struct.unpack('<i', payload[8:12])[0],
            'lat': struct.unpack('<i', payload[12:16])[0],
            'height': struct.unpack('<i', payload[16:20])[0],
            'hMSL': struct.unpack('<i', payload[20:24])[0],
            'lonHp': struct.unpack('<b', payload[24:25])[0],
            'latHp': struct.unpack('<b', payload[25:26])[0],
            'heightHp': struct.unpack('<b', payload[26:27])[0],
            'hMSLHp': struct.unpack('<b', payload[27:28])[0],
            'hAcc': struct.unpack('<I', payload[28:32])[0],
            'vAcc': struct.unpack('<I', payload[32:36])[0]
        }

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

    def parse_nav_pvt(self, payload):
        """Parse UBX-NAV-PVT message."""
        if len(payload) < 92:
            return None

        validByte = payload[11]
        validDate = bool(validByte & 0x01)
        validTime = bool(validByte & 0x02)
        fullyResolved = bool(validByte & 0x03)
        validMag = bool(validByte & 0x04)

        flagsByte = payload[21]
        gnssFixOk = bool(flagsByte & 0x01)
        diffSolnOk = bool(flagsByte & 0x02)
        psmState = (flagsByte & 0x03) >> 3
        headVehValid = bool(flagsByte & 0x06)
        carrSoln = (flagsByte & 0x07) >> 2

        flags2Byte = payload[22]
        confirmedAvai = bool(flags2Byte & 0x06)
        confirmedDate = bool(flags2Byte & 0x07)
        confirmedTime = bool(flags2Byte & 0x08)

        flags3Byte = struct.unpack('<H', payload[78:80])[0]
        invalidL1h = bool(flags3Byte & 0x01)
        lastCorrectionAge = (flags3Byte & 0x02) >> 3
        authTime = bool(flags3Byte & 0x0D)
        nmaFixStatus = bool(flags3Byte & 0x0E)

        data = {
            'iTOW': struct.unpack('<I', payload[0:4])[0],
            'year': struct.unpack('<H', payload[4:6])[0],
            'month': payload[6],
            'day': payload[7],
            'hour': payload[8],
            'min': payload[9],
            'sec': payload[10],
            'validDate': validDate,
            'validTime': validTime,
            'fullyResolved': fullyResolved,
            'validMag': validMag,
            'tAcc': struct.unpack('<I', payload[12:16])[0],
            'nano': struct.unpack('<i', payload[16:20])[0],
            'fixType': payload[20],
            'gnssFixOk': gnssFixOk,
            'diffSolnOk': diffSolnOk,
            'psmState': psmState,
            'headVehValid': headVehValid,
            'carrSoln': carrSoln,
            'confirmedAvai': confirmedAvai,
            'confirmedDate': confirmedDate,
            'confirmedTime': confirmedTime,
            'numSV': payload[23],
            'lon': struct.unpack('<i', payload[24:28])[0],
            'lat': struct.unpack('<i', payload[28:32])[0],
            'height': struct.unpack('<i', payload[32:36])[0],
            'hMSL': struct.unpack('<i', payload[36:40])[0],
            'hAcc': struct.unpack('<I', payload[40:44])[0],
            'vAcc': struct.unpack('<I', payload[44:48])[0],
            'velN': struct.unpack('<i', payload[48:52])[0],
            'velE': struct.unpack('<i', payload[52:56])[0],
            'velD': struct.unpack('<i', payload[56:60])[0],
            'gSpeed': struct.unpack('<i', payload[60:64])[0],
            'headMot': struct.unpack('<i', payload[64:68])[0],
            'sAcc': struct.unpack('<I', payload[68:72])[0],
            'headAcc': struct.unpack('<I', payload[72:76])[0],
            'pDOP': struct.unpack('<H', payload[76:78])[0],
            'invalidL1h': invalidL1h,
            'lastCorrectionAge': lastCorrectionAge,
            'authTime': authTime,
            'nmaFixStatus': nmaFixStatus,
            'reserved0': struct.unpack('<I', payload[80:84])[0],
            'headVeh': struct.unpack('<I', payload[84:88])[0],
            'magDec': struct.unpack('<h', payload[88:90])[0],
            'magAcc': struct.unpack('<H', payload[90:92])[0]
        }

        data = self.apply_exclusions(data, 'NAV-PVT')
        return data

    def parse_nav_sat(self, payload):
        """Parse UBX-NAV-SAT message."""
        if len(payload) < 8:
            return []

        # Parse header
        iTOW = struct.unpack('<I', payload[:4])[0]
        version = payload[4]
        numSvs = payload[5]
        reserved0 = struct.unpack('<H', payload[6:8])[0]

        measurements = []

        for i in range(numSvs):
            meas_offset = 8 + (i * 12)
            if meas_offset + 12 > len(payload):
                break

            meas_data = payload[meas_offset:meas_offset + 12]

            flags = meas_data[8]
            qualityInd = (flags & 0x01) >> 3
            svUsed = bool(flags & 0x04)
            health = (flags & 0x05) >> 2
            diffCorr = bool(flags & 0x06)
            smoothed = bool(flags & 0x07)
            orbitSource = (flags & 0x08) >> 3
            ephAvail = bool(flags & 0x0C)
            almAvail = bool(flags & 0x0D)
            anoAvail = bool(flags & 0x0E)
            aopAvail = bool(flags & 0x0F)
            sbasCorrUsed = bool(flags & 0x10)
            rtcmCorrUsed = bool(flags & 0x11)
            slasCorrUsed = bool(flags & 0x12)
            spartnCorrUsed = bool(flags & 0x13)
            prCorrUsed = bool(flags & 0x14)
            crCorrUsed = bool(flags & 0x15)
            doCorrUsed = bool(flags & 0x16)
            clasCorrUsed = bool(flags & 0x17)
            lppCorrUsed = bool(flags & 0x18)
            hasCorrUsed = bool(flags & 0x19)

            data = {
                'iTOW': iTOW,
                'version': version,
                'numSvs': numSvs,
                'reserved0': reserved0,
                'gnssId': meas_data[0],
                'svId': meas_data[1],
                'cno': meas_data[2],
                'elev': meas_data[3],
                'azim': struct.unpack('<h', payload[4:6])[0],
                'prRes': struct.unpack('<h', payload[6:8])[0],
                'qualityInd': qualityInd,
                'svUse': svUsed,
                'health': health,
                'diffCorr': diffCorr,
                'smoothed': smoothed,
                'orbitSource': orbitSource,
                'ephAvail': ephAvail,
                'almAvail': almAvail,
                'anoAvail': anoAvail,
                'aopAvail': aopAvail,
                'sbasCorrUsed': sbasCorrUsed,
                'rtcmCorrUsed': rtcmCorrUsed,
                'slasCorrUsed': slasCorrUsed,
                'spartnCorrUsed': spartnCorrUsed,
                'prCorrUsed': prCorrUsed,
                'crCorrUsed': crCorrUsed,
                'doCorrUsed': doCorrUsed,
                'clasCorrUsed': clasCorrUsed,
                'lppCorrUsed': lppCorrUsed,
                'hasCorrUsed': hasCorrUsed
            }

            data = self.apply_exclusions(data, 'NAV-SAT')
            measurements.append(data)

        return measurements

    def parse_nav_sig(self, payload):
        """Parse UBX-NAV-SAT message."""
        if len(payload) < 8:
            return []

        iTOW = struct.unpack('<I', payload[0:4])[0]
        version = payload[4]
        numSigs = payload[5]
        reserved0 = struct.unpack('<H', payload[6:8])[0]

        measurements = []
        for i in range(numSigs):
            meas_offset = 8 + (i * 16)
            if meas_offset + 16 > len(payload):
                break

            meas_data = payload[meas_offset:meas_offset + 16]

            flags = meas_data[10]
            health = (flags & 0x01) >> 2
            prSmoothed = bool(flags & 0x03)
            prUsed = bool(flags & 0x04)
            crUsed = bool(flags & 0x05)
            doUsed = bool(flags & 0x06)
            prCorrUsed = bool(flags & 0x07)
            crCorrUsed = bool(flags & 0x08)
            doCorrUsed = bool(flags & 0x09)
            authStatus = bool(flags & 0x0A)

            data = {
                'iTOW': iTOW,
                'version': version,
                'numSigs': numSigs,
                'reserved0': reserved0,
                'gnssId': meas_data[0],
                'svId': meas_data[1],
                'sigId': meas_data[2],
                'freqId': meas_data[3],
                'prRes': struct.unpack('<h', payload[4:6])[0],
                'cno': meas_data[6],
                'qualityInd': meas_data[7],
                'corrSource': meas_data[8],
                'ionoModel': meas_data[9],
                'health': health,
                'prSmoothed': prSmoothed,
                'prUsed': prUsed,
                'crUsed': crUsed,
                'doUsed': doUsed,
                'prCorrUsed': prCorrUsed,
                'crCorrUsed': crCorrUsed,
                'doCorrUsed': doCorrUsed,
                'authStatus': authStatus,
                'reserved1': struct.unpack('<I', payload[11:15])[0]
            }

            data = self.apply_exclusions(data, 'NAV-SIG')
            measurements.append(data)

        return measurements

    def parse_nav_status(self, payload):
        """Parse UBX-NAV-STATUS message."""
        if len(payload) < 16:
            return None

        flags = payload[5]
        gpsFixOk = bool(flags & 0x01)
        diffSoln = bool(flags & 0x02)
        wknSet = bool(flags & 0x03)
        towSet = bool(flags & 0x04)

        fixStat = payload[6]
        diffCorr = bool(fixStat & 0x01)
        carrSolnValid = bool(fixStat & 0x02)
        mapMatching = (fixStat & 0x03) >> 2

        flags2 = payload[7]
        psmState = (flags2 & 0x01) >> 2
        spoofDetState = (flags2 & 0x04) >> 2
        carrSoln = (flags2 & 0x07) >> 2

        data = {
            'iTOW': struct.unpack('<I', payload[0:4])[0],
            'gpsFix': payload[4],
            'gpsFixOk': gpsFixOk,
            'diffSoln': diffSoln,
            'wknSet': wknSet,
            'towSet': towSet,
            'diffCorr': diffCorr,
            'carrSolnValid': carrSolnValid,
            'mapMatching': mapMatching,
            'psmState': psmState,
            'spoofDetState': spoofDetState,
            'carrSoln': carrSoln,
            'ttff': struct.unpack('<I', payload[8:12])[0],
            'msss': struct.unpack('<I', payload[12:17])[0]
        }

        data = self.apply_exclusions(data, 'NAV-STATUS')
        return data

    def parse_nav_timegal(self, payload):
        """Parse UBX-NAV-TIMEGAL message."""
        if len(payload) < 20:
            return None

        validByte = payload[15]
        galTowValid = bool(validByte & 0x01)
        galWnoValid = bool(validByte & 0x02)
        leapSValid = bool(validByte & 0x03)

        data = {
            'iTOW': struct.unpack('<I', payload[0:4])[0],
            'galTow': struct.unpack('<I', payload[4:8])[0],
            'fGalTow': struct.unpack('<i', payload[8:12])[0],
            'galWno': struct.unpack('<h', payload[12:14])[0],
            'leapS': payload[14],
            'galTowValid': galTowValid,
            'galWnoValid': galWnoValid,
            'leapSValid': leapSValid,
            'tAcc': struct.unpack('<I', payload[16:21])[0],
        }

        data = self.apply_exclusions(data, 'NAV-TIMEGAL')
        return data

    def parse_nav_timegps(self, payload):
        """Parse UBX-NAV-TIMEGPS message."""
        if len(payload) < 16:
            return None

        validByte = payload[11]
        towValid = bool(validByte & 0x01)
        weekValid = bool(validByte & 0x02)
        leapSValid = bool(validByte & 0x03)

        data = {
            'iTOW': struct.unpack('<I', payload[0:4])[0],
            'fTOW': struct.unpack('<i', payload[4:8])[0],
            'week': struct.unpack('<h', payload[8:10])[0],
            'leapS': payload[10],
            'towValid': towValid,
            'weekValid': weekValid,
            'leapSValid': leapSValid,
            'tAcc': struct.unpack('<I', payload[12:17])[0]
        }

        data = self.apply_exclusions(data, 'NAV-TIMEGPS')
        return data

    def parse_nav_timeutc(self, payload):
        """Parse UBX-NAV-TIMEUTC message."""
        if len(payload) < 20:
            return None

        validByte = payload[19]
        validTOW = bool(validByte & 0x01)
        validWKN = bool(validByte & 0x02)
        validUTC = bool(validByte & 0x03)
        authStatus = bool(validByte & 0x04)
        utcStandard = (validByte & 0x05) >> 4
            
        data = {
            'iTOW': struct.unpack('<I', payload[0:4])[0],
            'tAcc': struct.unpack('<I', payload[4:8])[0],
            'nano': struct.unpack('<i', payload[8:12])[0],
            'year': struct.unpack('<H', payload[12:14])[0],
            'month': payload[14],
            'day': payload[15],
            'hour': payload[16],
            'min': payload[17],
            'sec': payload[18],
            'validTOW': validTOW,
            'validWKN': validWKN,
            'validUTC': validUTC,
            'authStatus': authStatus,
            'utcStandard': utcStandard
        }

        data = self.apply_exclusions(data, 'NAV-TIMEUTC')
        return data

    def parse_rxm_measx(self, payload):
        """Parse UBX-RXM-MEASX message."""
        if len(payload) < 44:
            return []

        # Parse header
        version = payload[0]
        reserved0 = struct.unpack('3B', payload[1:4])[0]
        gpsTOW = struct.unpack('<I', payload[4:8])[0]
        gloTOW = struct.unpack('<I', payload[8:12])[0]
        bdsTOW = struct.unpack('<I', payload[12:16])[0]
        reserved1 = struct.unpack('<I', payload[16:20])[0]
        qzssTOW = struct.unpack('<I', payload[20:24])[0]
        gpsTOWacc = struct.unpack('<H', payload[24:26])[0]
        gloTOWacc = struct.unpack('<H', payload[26:28])[0]
        bdsTOWacc = struct.unpack('<H', payload[28:30])[0]
        reserved2 = struct.unpack('<H', payload[30:32])[0]
        qzssTOWacc = struct.unpack('<H', payload[32:34])[0]
        numSV = payload[34]
        flags = payload[35]
        towSet = (flags & 0x01) >> 2

        reserved3 = struct.unpack('<Q', payload[36:44])[0]

        measurements = []

        for i in range(numSV):
            meas_offset = 44 + (i * 24)
            if meas_offset + 24 > len(payload):
                break

            meas_data = payload[meas_offset:meas_offset + 24]

            data = {
                'version': version,
                'reserved0': reserved0,
                'gpsTOW': gpsTOW,
                'gloTOW': gloTOW,
                'bdsTOW': bdsTOW,
                'reserved1': reserved1,
                'qzssTOW': qzssTOW,
                'gpsTOWacc': gpsTOWacc,
                'gloTOWacc': gloTOWacc,
                'bdsTOWacc': bdsTOWacc,
                'reserved2': reserved2,
                'qzssTOWacc': qzssTOWacc,
                'towSet': towSet,
                'reserved3': reserved3,
                'gnssId': meas_data[0],
                'svId': meas_data[1],
                'cNo': meas_data[2],
                'mpathIndic': meas_data[3],
                'dopplerMS': struct.unpack('<i', meas_data[4:8])[0],
                'dopplerHz': struct.unpack('<i', meas_data[8:12])[0],
                'wholeChips': struct.unpack('<H', meas_data[12:14])[0],
                'fracChips': struct.unpack('<H', meas_data[14:16])[0],
                'codePhase': struct.unpack('<I', meas_data[16:20])[0],
                'intCodePhase': meas_data[20],
                'pseuRangeRMS': meas_data[21],
                'reserved4': struct.unpack('<H', meas_data[21:23])[0]
            }

            data = self.apply_exclusions(data, 'NAV-MEASX')
            measurements.append(data)

        return measurements
    
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
        reserved0 = struct.unpack('<H', payload[14:16])[0]

        leapSec = bool(recStat & 0x01)
        clkReset = bool(recStat & 0x02)
        
        measurements = []
        
        # Parse each measurement (32 bytes each)
        for i in range(numMeas):
            meas_offset = 16 + (i * 32)
            if meas_offset + 32 > len(payload):
                break
                
            meas_data = payload[meas_offset:meas_offset + 32]

            trkStat = meas_data[29]
            prValid = bool(trkStat & 0x01)
            cpValid = bool(trkStat & 0x02)
            halfCyc = bool(trkStat & 0x03)
            subHalfCyc = bool(trkStat & 0x04)
            
            data = {
                'rcvTow': rcvTow,
                'week': week,
                'leapS': leapS,
                'numMeas': numMeas,
                'leapSec': leapSec,
                'clkReset': clkReset,
                'version': version,
                'reserved0': reserved0,
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
                'prValid': prValid,
                'cpValid': cpValid,
                'halfCyc': halfCyc,
                'subHalfCyc': subHalfCyc,
                'reserved1': struct.unpack('<H', meas_data[30:32])[0]
            }
            
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

                if msg_type == 'MON-COMMS':
                    comms = self.parse_mon_comms(msg['payload'])
                    if comms:
                        for comm in comms:
                            self.message_data['MON-COMMS'].append(comm)
                            supported_count += 1

                elif msg_type == 'MON-HW3':
                    pins = self.parse_mon_hw3(msg['payload'])
                    if pins:
                        for pin in pins:
                            self.message_data['MON-HW3'].append(pin)
                            supported_count += 1

                elif msg_type == 'MON-RF':
                    blocks = self.parse_mon_rf(msg['payload'])
                    if blocks:
                        for block in blocks:
                            self.message_data['MON-RF'].append(block)
                            supported_count += 1

                elif msg_type == 'MON-SPAN':
                    rfBlocks = self.parse_mon_span(msg['payload'])
                    if rfBlocks:
                        for rfBlock in rfBlocks:
                            self.message_data['MON-SPAN'].append(rfBlock)
                            supported_count += 1

                elif msg_type == 'MON-SYS':
                    result = self.parse_mon_sys(msg['payload'])
                    if result:
                        self.message_data['MON-SYS'].append(result)
                        supported_count += 1

                elif msg_type == 'NAV-CLOCK':
                    data = self.parse_nav_clock(msg['payload'])
                    if data:
                        self.message_data['NAV-CLOCK'].append(data)
                        supported_count += 1

                elif msg_type == 'NAV-DOP':
                    data = self.parse_nav_dop(msg['payload'])
                    if data:
                        self.message_data['NAV-DOP'].append(data)
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

                elif msg_type == 'NAV-PVT':
                    data = self.parse_nav_pvt(msg['payload'])
                    if data:
                        self.message_data['NAV-PVT'].append(data)
                        supported_count += 1

                elif msg_type == 'NAV-SAT':
                    sats = self.parse_nav_sat(msg['payload'])
                    if sats:
                        for sat in sats:
                            self.message_data['NAV-SAT'].append(sat)
                            supported_count += 1

                elif msg_type == 'NAV-SIG':
                    signals = self.parse_nav_sig(msg['payload'])
                    if signals:
                        for signal in signals:
                            self.message_data['NAV-SIG'].append(signal)
                            supported_count += 1

                elif msg_type == 'NAV-STATUS':
                    data = self.parse_nav_status(msg['payload'])
                    if data:
                        self.message_data['NAV-STATUS'].append(data)
                        supported_count += 1

                elif msg_type == 'NAV-TIMEGAL':
                    data = self.parse_nav_timegal(msg['payload'])
                    if data:
                        self.message_data['NAV-TIMEGAL'].append(data)
                        supported_count += 1

                elif msg_type == 'NAV-TIMEGPS':
                    data = self.parse_nav_timegps(msg['payload'])
                    if data:
                        self.message_data['NAV-TIMEGPS'].append(data)
                        supported_count += 1

                elif msg_type == 'NAV-TIMEUTC':
                    result = self.parse_nav_timeutc(msg['payload'])
                    if result:
                        self.message_data['NAV-TIMEUTC'].append(data)
                        supported_count += 1

                elif msg_type == 'RXM-MEASX':
                    measurements = self.parse_rxm_measx(msg['payload'])
                    if measurements:
                        for measurement in measurements:
                            self.message_data['RXM-MEASX'].append(measurement)
                            supported_count += 1

                elif msg_type == 'RXM-RAWX':
                    measurements = self.parse_rxm_rawx(msg['payload'])
                    if measurements:
                        for measurement in measurements:
                            self.message_data['RXM-RAWX'].append(measurement)
                            supported_count += 1
                
                elif msg_type == 'RXM-SFRBX':
                    result = self.parse_rxm_sfrbx(msg['payload'])
                    if data:
                        self.message_data['RXM-SFRBX'].append(result)
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
        print("Usage: python UBXParser.py <ubx_file_path> [output_directory]")
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