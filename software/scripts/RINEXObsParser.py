#!/usr/bin/env python3
"""
RINEX Observation File Parser (Version 2.11/3.05)
Supports GPS, Galileo, and GLONASS constellations
Outputs observation data to CSV format
Uses only standard Python libraries
"""

import sys
import re
from datetime import datetime, timedelta
from collections import defaultdict


class RinexObservationParser:
    """Parser for RINEX 2.11 and 3.05 observation files"""

    def __init__(self, filepath):
        self.filepath = filepath
        self.version = None
        self.header = {}
        self.observation_types = {}  # constellation -> list of obs types
        self.epoch_data = []
        self.constellation_map = {
            'G': 'GPS',
            'R': 'GLONASS',
            'E': 'Galileo',
            'C': 'BeiDou',
            'J': 'QZSS',
            'I': 'IRNSS',
            'S': 'SBAS'
        }

    def parse(self):
        """Main parsing function"""
        try:
            with open(self.filepath, 'r') as f:
                self._parse_header(f)
                self._parse_observations(f)
            return True
        except Exception as e:
            print(f"Error parsing RINEX file: {e}", file=sys.stderr)
            return False

    def _parse_header(self, f):
        """Parse RINEX header section"""
        for line in f:
            if 'END OF HEADER' in line:
                break

            # Extract version and file type
            if 'RINEX VERSION / TYPE' in line:
                self.version = float(line[0:9].strip())
                self.header['file_type'] = line[20:21].strip()

            # Extract marker name
            elif 'MARKER NAME' in line:
                self.header['marker_name'] = line[0:60].strip()

            # Extract receiver info
            elif 'REC # / TYPE / VERS' in line:
                self.header['receiver_number'] = line[0:20].strip()
                self.header['receiver_type'] = line[20:40].strip()
                self.header['receiver_version'] = line[40:60].strip()

            # Extract antenna info
            elif 'ANT # / TYPE' in line:
                self.header['antenna_number'] = line[0:20].strip()
                self.header['antenna_type'] = line[20:40].strip()

            # Extract approximate position
            elif 'APPROX POSITION XYZ' in line:
                try:
                    self.header['approx_position_x'] = float(line[0:14].strip())
                    self.header['approx_position_y'] = float(line[14:28].strip())
                    self.header['approx_position_z'] = float(line[28:42].strip())
                except ValueError:
                    pass

            # Extract observation interval
            elif 'INTERVAL' in line:
                try:
                    self.header['interval'] = float(line[0:10].strip())
                except ValueError:
                    pass

            # Extract time of first observation
            elif 'TIME OF FIRST OBS' in line:
                try:
                    year = int(line[0:6].strip())
                    month = int(line[6:12].strip())
                    day = int(line[12:18].strip())
                    hour = int(line[18:24].strip())
                    minute = int(line[24:30].strip())
                    second = float(line[30:43].strip())
                    self.header['first_obs'] = datetime(year, month, day, hour, minute, int(second))
                except (ValueError, TypeError):
                    pass

            # Parse observation types (RINEX 2.11)
            elif '# / TYPES OF OBSERV' in line and self.version < 3.0:
                try:
                    num_obs = int(line[0:6].strip())
                    obs_types = []
                    # First line can contain up to 9 observation types
                    for i in range(min(9, num_obs)):
                        obs_type = line[10 + i * 6:10 + (i + 1) * 6].strip()
                        if obs_type:
                            obs_types.append(obs_type)

                    # Read continuation lines if needed
                    remaining = num_obs - len(obs_types)
                    while remaining > 0:
                        line = next(f)
                        for i in range(min(9, remaining)):
                            obs_type = line[10 + i * 6:10 + (i + 1) * 6].strip()
                            if obs_type:
                                obs_types.append(obs_type)
                        remaining -= 9

                    # In RINEX 2.11, observation types apply to all constellations
                    self.observation_types['G'] = obs_types
                    self.observation_types['R'] = obs_types
                    self.observation_types['E'] = obs_types
                except (ValueError, StopIteration):
                    pass

            # Parse observation types (RINEX 3.0+)
            elif 'SYS / # / OBS TYPES' in line and self.version >= 3.0:
                try:
                    constellation = line[0:1].strip()
                    num_obs = int(line[3:6].strip())
                    obs_types = []

                    # First line contains up to 13 observation types
                    for i in range(min(13, num_obs)):
                        obs_type = line[7 + i * 4:7 + (i + 1) * 4].strip()
                        if obs_type:
                            obs_types.append(obs_type)

                    # Read continuation lines if needed
                    remaining = num_obs - len(obs_types)
                    while remaining > 0:
                        line = next(f)
                        for i in range(min(13, remaining)):
                            obs_type = line[7 + i * 4:7 + (i + 1) * 4].strip()
                            if obs_type:
                                obs_types.append(obs_type)
                        remaining -= 13

                    self.observation_types[constellation] = obs_types
                except (ValueError, StopIteration):
                    pass

    def _parse_observations(self, f):
        """Parse observation data section"""
        if self.version < 3.0:
            self._parse_rinex2_observations(f)
        else:
            self._parse_rinex3_observations(f)

    def _parse_rinex2_observations(self, f):
        """Parse RINEX 2.11 observation data"""
        for line in f:
            # Check for epoch header
            if len(line) < 29:
                continue

            # Epoch record starts with date/time
            try:
                # RINEX 2 epoch format: YY MM DD HH MM SS.SSSSSSS  0 NN
                if line[0:1] == ' ' and line[1:3].strip().isdigit():
                    # Parse epoch header
                    year = int(line[1:3].strip())
                    # Handle Y2K: assume 1980-2079
                    if year >= 80:
                        year += 1900
                    else:
                        year += 2000
                    month = int(line[4:6].strip())
                    day = int(line[7:9].strip())
                    hour = int(line[10:12].strip())
                    minute = int(line[13:15].strip())
                    second = float(line[16:26].strip())

                    epoch_flag = int(line[28:29].strip())
                    num_sats = int(line[29:32].strip())

                    # Skip special event records
                    if epoch_flag > 1:
                        # Read and skip event lines
                        for _ in range((num_sats + 11) // 12):
                            next(f, None)
                        continue

                    epoch_time = datetime(year, month, day, hour, minute, int(second))
                    microsecond = int((second - int(second)) * 1000000)
                    epoch_time = epoch_time.replace(microsecond=microsecond)

                    # Parse satellite list
                    satellites = []
                    sat_line = line
                    for i in range(num_sats):
                        if i > 0 and i % 12 == 0:
                            # Read continuation line
                            sat_line = next(f, '')
                        sat_idx = (i % 12) * 3 + 32
                        sat_id = sat_line[sat_idx:sat_idx + 3].strip()
                        if sat_id:
                            satellites.append(sat_id)

                    # Parse observations for each satellite
                    obs_types = self.observation_types.get('G', [])
                    num_obs_types = len(obs_types)

                    for sat in satellites:
                        # Determine constellation
                        if sat[0].isalpha():
                            constellation = sat[0]
                            prn = sat[1:]
                        else:
                            # Old format: assume GPS if no letter prefix
                            constellation = 'G'
                            prn = sat

                        # Read observation lines (max 5 observations per line)
                        observations = {}
                        obs_idx = 0

                        for line_num in range((num_obs_types + 4) // 5):
                            obs_line = next(f, '')
                            for i in range(min(5, num_obs_types - obs_idx)):
                                start_col = i * 16
                                obs_str = obs_line[start_col:start_col + 14].strip()
                                lli = obs_line[start_col + 14:start_col + 15].strip()
                                ssi = obs_line[start_col + 15:start_col + 16].strip()

                                obs_type = obs_types[obs_idx]

                                if obs_str:
                                    try:
                                        observations[obs_type] = float(obs_str)
                                        if lli:
                                            observations[f"{obs_type}_LLI"] = int(lli)
                                        if ssi:
                                            observations[f"{obs_type}_SSI"] = int(ssi)
                                    except ValueError:
                                        observations[obs_type] = None
                                else:
                                    observations[obs_type] = None

                                obs_idx += 1

                        # Store epoch data
                        epoch_record = {
                            'epoch': epoch_time,
                            'constellation': constellation,
                            'prn': prn,
                            'sat_id': sat
                        }
                        epoch_record.update(observations)
                        self.epoch_data.append(epoch_record)

            except (ValueError, IndexError, StopIteration) as e:
                # Skip corrupted lines
                continue

    def _parse_rinex3_observations(self, f):
        """Parse RINEX 3.0+ observation data"""
        for line in f:
            # Check for epoch header (starts with '>')
            if line.startswith('>'):
                try:
                    # Parse epoch header
                    # Format: > YYYY MM DD HH MM SS.SSSSSSS  0 NN
                    year = int(line[2:6].strip())
                    month = int(line[7:9].strip())
                    day = int(line[10:12].strip())
                    hour = int(line[13:15].strip())
                    minute = int(line[16:18].strip())
                    second = float(line[19:29].strip())

                    epoch_flag = int(line[31:32].strip())
                    num_sats = int(line[32:35].strip())

                    # Skip special event records
                    if epoch_flag > 1:
                        for _ in range(num_sats):
                            next(f, None)
                        continue

                    epoch_time = datetime(year, month, day, hour, minute, int(second))
                    microsecond = int((second - int(second)) * 1000000)
                    epoch_time = epoch_time.replace(microsecond=microsecond)

                    # Parse observations for each satellite
                    for _ in range(num_sats):
                        obs_line = next(f, '')
                        if len(obs_line) < 3:
                            continue

                        # Extract satellite ID
                        sat_id = obs_line[0:3].strip()
                        if not sat_id:
                            continue

                        constellation = sat_id[0]
                        prn = sat_id[1:]

                        # Get observation types for this constellation
                        obs_types = self.observation_types.get(constellation, [])

                        # Parse observations
                        observations = {}
                        for i, obs_type in enumerate(obs_types):
                            start_col = 3 + i * 16
                            if start_col + 14 > len(obs_line):
                                break

                            obs_str = obs_line[start_col:start_col + 14].strip()
                            lli = obs_line[start_col + 14:start_col + 15].strip() if start_col + 15 <= len(
                                obs_line) else ''
                            ssi = obs_line[start_col + 15:start_col + 16].strip() if start_col + 16 <= len(
                                obs_line) else ''

                            if obs_str:
                                try:
                                    observations[obs_type] = float(obs_str)
                                    if lli:
                                        observations[f"{obs_type}_LLI"] = int(lli)
                                    if ssi:
                                        observations[f"{obs_type}_SSI"] = int(ssi)
                                except ValueError:
                                    observations[obs_type] = None
                            else:
                                observations[obs_type] = None

                        # Store epoch data
                        epoch_record = {
                            'epoch': epoch_time,
                            'constellation': constellation,
                            'prn': prn,
                            'sat_id': sat_id
                        }
                        epoch_record.update(observations)
                        self.epoch_data.append(epoch_record)

                except (ValueError, IndexError, StopIteration) as e:
                    # Skip corrupted epochs
                    continue

    def to_csv(self, output_basename=None):
        """Export parsed data to CSV format, separated by constellation"""
        if not self.epoch_data:
            print("No observation data to export", file=sys.stderr)
            return

        # Group data by constellation
        constellation_data = defaultdict(list)
        for record in self.epoch_data:
            constellation_data[record['constellation']].append(record)

        # Process each constellation separately
        for constellation, records in sorted(constellation_data.items()):
            # Collect all unique observation types for this constellation
            obs_types = set()
            for record in records:
                for key in record.keys():
                    if key not in ['epoch', 'constellation', 'prn', 'sat_id']:
                        obs_types.add(key)

            # Sort observation types
            obs_types = sorted(list(obs_types))

            # Create CSV header
            header_fields = ['epoch', 'constellation', 'prn', 'sat_id'] + obs_types

            # Determine output file
            if output_basename:
                # Strip extension if present
                import os
                base_name = os.path.splitext(output_basename)[0]
                constellation_name = self.constellation_map.get(constellation, constellation)
                output_file = f"{base_name}_{constellation_name}.csv"
                f = open(output_file, 'w')
                print(f"Writing {constellation_name} data to {output_file}", file=sys.stderr)
            else:
                # Output to stdout with constellation header
                f = sys.stdout
                print(f"\n=== {self.constellation_map.get(constellation, constellation)} ({constellation}) ===",
                      file=sys.stderr)

            try:
                # Write header
                f.write(','.join(header_fields) + '\n')

                # Write data
                for record in records:
                    values = []
                    for field in header_fields:
                        if field == 'epoch':
                            values.append(record['epoch'].strftime('%Y-%m-%d %H:%M:%S.%f'))
                        elif field in record:
                            val = record[field]
                            if val is None:
                                values.append('')
                            elif isinstance(val, float):
                                values.append(f"{val:.3f}")
                            else:
                                values.append(str(val))
                        else:
                            values.append('')

                    f.write(','.join(values) + '\n')

            finally:
                if output_basename:
                    f.close()
                    print(f"  {len(records)} observations written", file=sys.stderr)

    def get_summary(self):
        """Generate summary statistics of parsed data"""
        if not self.epoch_data:
            return "No data parsed"

        # Count observations by constellation
        constellation_counts = defaultdict(int)
        satellite_set = set()
        epochs_set = set()

        for record in self.epoch_data:
            constellation_counts[record['constellation']] += 1
            satellite_set.add(record['sat_id'])
            epochs_set.add(record['epoch'])

        summary = []
        summary.append(f"RINEX Version: {self.version}")
        summary.append(f"Marker Name: {self.header.get('marker_name', 'N/A')}")
        summary.append(f"Total Epochs: {len(epochs_set)}")
        summary.append(f"Total Satellites: {len(satellite_set)}")
        summary.append(f"Total Observations: {len(self.epoch_data)}")
        summary.append("\nObservations by Constellation:")
        for const, count in sorted(constellation_counts.items()):
            const_name = self.constellation_map.get(const, const)
            summary.append(f"  {const_name} ({const}): {count}")

        return '\n'.join(summary)


def main():
    """Main entry point for command-line usage"""
    if len(sys.argv) < 2:
        print("Usage: python rinex_parser.py <rinex_file> [output_basename]", file=sys.stderr)
        print("\nThe output_basename should not include extension.", file=sys.stderr)
        print("Separate CSV files will be created for each constellation:", file=sys.stderr)
        print("  <basename>_GPS.csv, <basename>_Galileo.csv, <basename>_GLONASS.csv, etc.\n", file=sys.stderr)
        print("Examples:")
        print("  python rinex_parser.py observation.rnx output")
        print("    Creates: output_GPS.csv, output_Galileo.csv, output_GLONASS.csv")
        print("\n  python rinex_parser.py observation.rnx obs_data.csv")
        print("    Creates: obs_data_GPS.csv, obs_data_Galileo.csv, obs_data_GLONASS.csv")
        print("\n  python rinex_parser.py observation.rnx")
        print("    Output to stdout (all constellations)")
        sys.exit(1)

    rinex_file = sys.argv[1]
    output_basename = sys.argv[2] if len(sys.argv) > 2 else None

    # Parse RINEX file
    parser = RinexObservationParser(rinex_file)

    print(f"Parsing RINEX file: {rinex_file}", file=sys.stderr)

    if not parser.parse():
        print("Failed to parse RINEX file", file=sys.stderr)
        sys.exit(1)

    # Print summary
    print("\n" + parser.get_summary(), file=sys.stderr)

    if output_basename:
        print(f"\nExporting to constellation-specific CSV files...", file=sys.stderr)
    else:
        print(f"\nExporting to stdout...", file=sys.stderr)

    # Export to CSV
    parser.to_csv(output_basename)

    if output_basename:
        print(f"\nSuccessfully exported constellation data", file=sys.stderr)


if __name__ == "__main__":
    main()