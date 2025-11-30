#!/usr/bin/env python3
"""
RINEX Navigation File Parser
Parses RINEX 2.11/3.0+ navigation files to extract GPS, Galileo, and GLONASS ephemeris data
Outputs JSON format compatible with UBX SFRBX parser output
Uses only standard Python libraries
"""

import sys
import json
from datetime import datetime #, timedelta
from collections import defaultdict
# from typing import Dict, List, Any, Optional


class RinexNavParser:
    """Parser for RINEX navigation files"""

    def __init__(self, filepath):
        self.filepath = filepath
        self.version = None
        self.header = {}
        self.ephemerides = {
            'GPS': defaultdict(dict),
            'Galileo': defaultdict(dict),
            'GLONASS': defaultdict(dict)
        }
        self.ionospheric_params = {}
        self.utc_params = {}

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
                self._parse_ephemeris_data(f)
            return True
        except Exception as e:
            print(f"Error parsing RINEX navigation file: {e}", file=sys.stderr)
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

            # Extract ionospheric correction parameters (GPS Klobuchar model)
            elif 'ION ALPHA' in line or 'IONOSPHERIC CORR' in line:
                if 'ION ALPHA' in line:
                    # RINEX 2.x format
                    try:
                        alpha0 = self._parse_fortran_float(line[2:14])
                        alpha1 = self._parse_fortran_float(line[14:26])
                        alpha2 = self._parse_fortran_float(line[26:38])
                        alpha3 = self._parse_fortran_float(line[38:50])
                        self.ionospheric_params['alpha'] = [alpha0, alpha1, alpha2, alpha3]
                    except ValueError:
                        pass

            elif 'ION BETA' in line:
                # RINEX 2.x format
                try:
                    beta0 = self._parse_fortran_float(line[2:14])
                    beta1 = self._parse_fortran_float(line[14:26])
                    beta2 = self._parse_fortran_float(line[26:38])
                    beta3 = self._parse_fortran_float(line[38:50])
                    self.ionospheric_params['beta'] = [beta0, beta1, beta2, beta3]
                except ValueError:
                    pass

            # Extract UTC time correlation parameters
            elif 'DELTA-UTC' in line or 'TIME SYSTEM CORR' in line:
                if 'DELTA-UTC' in line:
                    # RINEX 2.x format
                    try:
                        A0 = self._parse_fortran_float(line[3:22])
                        A1 = self._parse_fortran_float(line[22:41])
                        T = int(line[41:50].strip())
                        W = int(line[50:59].strip())
                        self.utc_params = {
                            'A0': A0,
                            'A1': A1,
                            't_ot': T,
                            'WN_ot': W
                        }
                    except (ValueError, IndexError):
                        pass

            # Extract leap seconds
            elif 'LEAP SECONDS' in line:
                try:
                    leap_seconds = int(line[0:6].strip())
                    self.header['leap_seconds'] = leap_seconds
                except ValueError:
                    pass

    def _parse_fortran_float(self, s):
        """Parse Fortran-style floating point (e.g., 1.2345D-08)"""
        s = s.strip()
        if not s:
            return 0.0
        # Replace D with E for Python float parsing
        s = s.replace('D', 'E').replace('d', 'E')
        return float(s)

    def _parse_ephemeris_data(self, f):
        """Parse ephemeris data section"""
        if self.version < 3.0:
            self._parse_rinex2_ephemeris(f)
        else:
            self._parse_rinex3_ephemeris(f)

    def _parse_rinex2_ephemeris(self, f):
        """Parse RINEX 2.x ephemeris data (GPS-only typically)"""
        for line in f:
            # Check if this is a satellite epoch line
            # Format: PRN YY MM DD HH MM SS.S af0 af1 af2
            if len(line) < 22:
                continue

            try:
                # Parse PRN and epoch
                prn = int(line[0:2].strip())
                year = int(line[3:5].strip())
                month = int(line[6:8].strip())
                day = int(line[9:11].strip())
                hour = int(line[12:14].strip())
                minute = int(line[15:17].strip())
                second = float(line[18:22].strip())

                # Handle Y2K: assume 1980-2079
                if year >= 80:
                    year += 1900
                else:
                    year += 2000

                # Parse clock parameters from epoch line
                af0 = self._parse_fortran_float(line[22:41])
                af1 = self._parse_fortran_float(line[41:60])
                af2 = self._parse_fortran_float(line[60:79])

                # Read the next 7 lines of ephemeris data
                lines = []
                for i in range(7):
                    next_line = next(f, '')
                    if not next_line:
                        break
                    lines.append(next_line)

                if len(lines) < 7:
                    continue

                # Parse broadcast orbit parameters
                # Line 1 (index 0)
                IODE = self._parse_fortran_float(lines[0][3:22])
                Crs = self._parse_fortran_float(lines[0][22:41])
                delta_n = self._parse_fortran_float(lines[0][41:60])
                M0 = self._parse_fortran_float(lines[0][60:79])

                # Line 2
                Cuc = self._parse_fortran_float(lines[1][3:22])
                e = self._parse_fortran_float(lines[1][22:41])
                Cus = self._parse_fortran_float(lines[1][41:60])
                sqrt_A = self._parse_fortran_float(lines[1][60:79])

                # Line 3
                toe = self._parse_fortran_float(lines[2][3:22])
                Cic = self._parse_fortran_float(lines[2][22:41])
                Omega0 = self._parse_fortran_float(lines[2][41:60])
                Cis = self._parse_fortran_float(lines[2][60:79])

                # Line 4
                i0 = self._parse_fortran_float(lines[3][3:22])
                Crc = self._parse_fortran_float(lines[3][22:41])
                omega = self._parse_fortran_float(lines[3][41:60])
                Omega_dot = self._parse_fortran_float(lines[3][60:79])

                # Line 5
                IDOT = self._parse_fortran_float(lines[4][3:22])
                codes_on_L2 = self._parse_fortran_float(lines[4][22:41])
                GPS_week = self._parse_fortran_float(lines[4][41:60])
                L2_P_flag = self._parse_fortran_float(lines[4][60:79])

                # Line 6
                SV_accuracy = self._parse_fortran_float(lines[5][3:22])
                SV_health = self._parse_fortran_float(lines[5][22:41])
                TGD = self._parse_fortran_float(lines[5][41:60])
                IODC = self._parse_fortran_float(lines[5][60:79])

                # Line 7
                transmission_time = self._parse_fortran_float(lines[6][3:22])
                fit_interval = self._parse_fortran_float(lines[6][22:41]) if len(lines[6]) > 22 else 4.0

                # Calculate toc (time of clock) from epoch
                epoch_dt = datetime(year, month, day, hour, minute, int(second))
                # Convert to GPS time of week (approximate)
                # This is simplified - proper conversion needs GPS week start
                toc = toe  # Use toe as toc approximation

                # Create ephemeris dictionary matching sfrbx_parser format
                eph = {
                    'Cic': Cic,
                    'Cis': Cis,
                    'Crc': Crc,
                    'Crs': Crs,
                    'Cuc': Cuc,
                    'Cus': Cus,
                    'IDOT': IDOT,
                    'M0': M0,
                    'Omega0': Omega0,
                    'Omega_dot': Omega_dot,
                    'WN': int(GPS_week),
                    'af0': af0,
                    'af1': af1,
                    'af2': af2,
                    'delta_n0': delta_n,
                    'e': e,
                    'i0': i0,
                    'omega': omega,
                    'sqrt_A': sqrt_A,
                    'svID': prn,
                    'toc': toc,
                    'toe': toe,
                    'tow': transmission_time
                }

                # Store ephemeris
                sat_id = f"G{prn:02d}"
                toe_key = f"G{prn:02d}_{int(toe)}"

                if sat_id not in self.ephemerides['GPS']:
                    self.ephemerides['GPS'][sat_id] = {}

                self.ephemerides['GPS'][sat_id][toe_key] = eph

            except (ValueError, IndexError, StopIteration) as e:
                # Skip corrupted ephemeris records
                continue

    def _parse_rinex3_ephemeris(self, f):
        """Parse RINEX 3.x ephemeris data (multi-GNSS)"""
        for line in f:
            # Check if this is a satellite epoch line
            # Format: SYS PRN YYYY MM DD HH MM SS af0 af1 af2
            if len(line) < 23:
                continue

            try:
                # Parse satellite ID
                sat_sys = line[0:1]
                if sat_sys not in self.constellation_map:
                    continue

                constellation = self.constellation_map[sat_sys]
                prn = int(line[1:3].strip())

                # Parse epoch
                year = int(line[4:8].strip())
                month = int(line[9:11].strip())
                day = int(line[12:14].strip())
                hour = int(line[15:17].strip())
                minute = int(line[18:20].strip())
                second = float(line[21:23].strip())

                # Parse clock parameters from epoch line
                af0 = self._parse_fortran_float(line[23:42])
                af1 = self._parse_fortran_float(line[42:61])
                af2 = self._parse_fortran_float(line[61:80])

                # Determine number of data lines based on constellation
                if constellation == 'GLONASS':
                    num_lines = 3  # GLONASS has different format
                else:
                    num_lines = 7  # GPS, Galileo, etc.

                # Read ephemeris data lines
                lines = []
                for i in range(num_lines):
                    next_line = next(f, '')
                    if not next_line:
                        break
                    lines.append(next_line)

                if len(lines) < num_lines:
                    continue

                # Parse based on constellation
                if constellation == 'GPS':
                    eph = self._parse_gps_ephemeris(lines, prn, af0, af1, af2)
                    if eph:
                        sat_id = f"G{prn:02d}"
                        toe_key = f"G{prn:02d}_{int(eph['toe'])}"
                        if sat_id not in self.ephemerides['GPS']:
                            self.ephemerides['GPS'][sat_id] = {}
                        self.ephemerides['GPS'][sat_id][toe_key] = eph

                elif constellation == 'Galileo':
                    eph = self._parse_galileo_ephemeris(lines, prn, af0, af1, af2)
                    if eph:
                        sat_id = f"E{prn:02d}"
                        toe_key = f"E{prn:02d}_{int(eph['toe'])}"
                        if sat_id not in self.ephemerides['Galileo']:
                            self.ephemerides['Galileo'][sat_id] = {}
                        self.ephemerides['Galileo'][sat_id][toe_key] = eph

                elif constellation == 'GLONASS':
                    # GLONASS uses different ephemeris format (position/velocity)
                    # Not implementing full GLONASS parsing in this version
                    pass

            except (ValueError, IndexError, StopIteration) as e:
                # Skip corrupted ephemeris records
                continue

    def _parse_gps_ephemeris(self, lines, prn, af0, af1, af2):
        """Parse GPS ephemeris from data lines"""
        try:
            # Line 1
            IODE = self._parse_fortran_float(lines[0][4:23])
            Crs = self._parse_fortran_float(lines[0][23:42])
            delta_n = self._parse_fortran_float(lines[0][42:61])
            M0 = self._parse_fortran_float(lines[0][61:80])

            # Line 2
            Cuc = self._parse_fortran_float(lines[1][4:23])
            e = self._parse_fortran_float(lines[1][23:42])
            Cus = self._parse_fortran_float(lines[1][42:61])
            sqrt_A = self._parse_fortran_float(lines[1][61:80])

            # Line 3
            toe = self._parse_fortran_float(lines[2][4:23])
            Cic = self._parse_fortran_float(lines[2][23:42])
            Omega0 = self._parse_fortran_float(lines[2][42:61])
            Cis = self._parse_fortran_float(lines[2][61:80])

            # Line 4
            i0 = self._parse_fortran_float(lines[3][4:23])
            Crc = self._parse_fortran_float(lines[3][23:42])
            omega = self._parse_fortran_float(lines[3][42:61])
            Omega_dot = self._parse_fortran_float(lines[3][61:80])

            # Line 5
            IDOT = self._parse_fortran_float(lines[4][4:23])
            codes_on_L2 = self._parse_fortran_float(lines[4][23:42])
            GPS_week = self._parse_fortran_float(lines[4][42:61])
            L2_P_flag = self._parse_fortran_float(lines[4][61:80])

            # Line 6
            SV_accuracy = self._parse_fortran_float(lines[5][4:23])
            SV_health = self._parse_fortran_float(lines[5][23:42])
            TGD = self._parse_fortran_float(lines[5][42:61])
            IODC = self._parse_fortran_float(lines[5][61:80])

            # Line 7
            transmission_time = self._parse_fortran_float(lines[6][4:23])
            fit_interval = self._parse_fortran_float(lines[6][23:42]) if len(lines[6]) > 23 else 4.0

            return {
                'Cic': Cic,
                'Cis': Cis,
                'Crc': Crc,
                'Crs': Crs,
                'Cuc': Cuc,
                'Cus': Cus,
                'IDOT': IDOT,
                'M0': M0,
                'Omega0': Omega0,
                'Omega_dot': Omega_dot,
                'WN': int(GPS_week),
                'af0': af0,
                'af1': af1,
                'af2': af2,
                'delta_n0': delta_n,
                'e': e,
                'i0': i0,
                'omega': omega,
                'sqrt_A': sqrt_A,
                'svID': prn,
                'toc': toe,  # Simplified: using toe as toc
                'toe': toe,
                'tow': transmission_time
            }
        except (ValueError, IndexError):
            return None

    def _parse_galileo_ephemeris(self, lines, prn, af0, af1, af2):
        """Parse Galileo ephemeris from data lines"""
        try:
            # Line 1
            IODnav = self._parse_fortran_float(lines[0][4:23])
            Crs = self._parse_fortran_float(lines[0][23:42])
            delta_n = self._parse_fortran_float(lines[0][42:61])
            M0 = self._parse_fortran_float(lines[0][61:80])

            # Line 2
            Cuc = self._parse_fortran_float(lines[1][4:23])
            e = self._parse_fortran_float(lines[1][23:42])
            Cus = self._parse_fortran_float(lines[1][42:61])
            sqrt_A = self._parse_fortran_float(lines[1][61:80])

            # Line 3
            toe = self._parse_fortran_float(lines[2][4:23])
            Cic = self._parse_fortran_float(lines[2][23:42])
            Omega0 = self._parse_fortran_float(lines[2][42:61])
            Cis = self._parse_fortran_float(lines[2][61:80])

            # Line 4
            i0 = self._parse_fortran_float(lines[3][4:23])
            Crc = self._parse_fortran_float(lines[3][23:42])
            omega = self._parse_fortran_float(lines[3][42:61])
            Omega_dot = self._parse_fortran_float(lines[3][61:80])

            # Line 5
            IDOT = self._parse_fortran_float(lines[4][4:23])
            data_sources = self._parse_fortran_float(lines[4][23:42])
            GAL_week = self._parse_fortran_float(lines[4][42:61])
            spare = self._parse_fortran_float(lines[4][61:80])

            # Line 6
            SISA = self._parse_fortran_float(lines[5][4:23])
            SV_health = self._parse_fortran_float(lines[5][23:42])
            BGD_E5a_E1 = self._parse_fortran_float(lines[5][42:61])
            BGD_E5b_E1 = self._parse_fortran_float(lines[5][61:80])

            # Line 7
            transmission_time = self._parse_fortran_float(lines[6][4:23])
            spare2 = self._parse_fortran_float(lines[6][23:42]) if len(lines[6]) > 23 else 0.0

            return {
                'Cic': Cic,
                'Cis': Cis,
                'Crc': Crc,
                'Crs': Crs,
                'Cuc': Cuc,
                'Cus': Cus,
                'IDOT': IDOT,
                'M0': M0,
                'Omega0': Omega0,
                'Omega_dot': Omega_dot,
                'WN': int(GAL_week),
                'af0': af0,
                'af1': af1,
                'af2': af2,
                'delta_n0': delta_n,
                'e': e,
                'i0': i0,
                'omega': omega,
                'sqrt_A': sqrt_A,
                'svID': prn,
                'toc': toe,  # Simplified: using toe as toc
                'toe': toe,
                'tow': transmission_time
            }
        except (ValueError, IndexError):
            return None

    def to_json(self, output_file=None, indent=2):
        """Export ephemerides to JSON format matching sfrbx_parser output"""
        # Build result dictionary
        result = {}

        for constellation in ['GPS', 'Galileo', 'GLONASS']:
            if self.ephemerides[constellation]:
                result[constellation] = dict(self.ephemerides[constellation])

        # Add ionospheric and UTC parameters if available
        if self.ionospheric_params:
            result['ionospheric_params'] = self.ionospheric_params
        if self.utc_params:
            result['utc_params'] = self.utc_params

        # Write to file or stdout
        if output_file:
            with open(output_file, 'w') as f:
                json.dump(result, f, indent=indent)
            print(f"Ephemerides saved to {output_file}", file=sys.stderr)
        else:
            print(json.dumps(result, indent=indent))

        return result

    def get_summary(self):
        """Generate summary statistics"""
        summary = []
        summary.append(f"RINEX Version: {self.version}")
        summary.append(f"File Type: {self.header.get('file_type', 'N/A')}")

        for constellation in ['GPS', 'Galileo', 'GLONASS']:
            if self.ephemerides[constellation]:
                num_sats = len(self.ephemerides[constellation])
                num_eph = sum(len(ephs) for ephs in self.ephemerides[constellation].values())
                summary.append(f"{constellation}: {num_sats} satellites, {num_eph} ephemerides")

        if self.ionospheric_params:
            summary.append(f"Ionospheric parameters: {len(self.ionospheric_params)} sets")
        if self.utc_params:
            summary.append(f"UTC parameters: Available")

        return '\n'.join(summary)


def main():
    """Main entry point for command-line usage"""
    if len(sys.argv) < 2:
        print("Usage: python rinex_nav_parser.py <rinex_nav_file> [output.json]", file=sys.stderr)
        print("\nExamples:", file=sys.stderr)
        print("  python rinex_nav_parser.py brdc3290.25n ephemeris.json", file=sys.stderr)
        print("  python rinex_nav_parser.py brdc3290.25n  # Output to stdout", file=sys.stderr)
        sys.exit(1)

    rinex_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None

    # Parse RINEX navigation file
    parser = RinexNavParser(rinex_file)

    print(f"Parsing RINEX navigation file: {rinex_file}", file=sys.stderr)

    if not parser.parse():
        print("Failed to parse RINEX navigation file", file=sys.stderr)
        sys.exit(1)

    # Print summary
    print("\n" + parser.get_summary(), file=sys.stderr)

    if output_file:
        print(f"\nExporting to JSON: {output_file}", file=sys.stderr)
    else:
        print(f"\nExporting to stdout...", file=sys.stderr)

    # Export to JSON
    parser.to_json(output_file)


if __name__ == "__main__":
    main()