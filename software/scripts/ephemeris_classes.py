"""
Satellite ephemeris classes for GPS and Galileo
Contains ephemeris parameters and GNSS constants
"""

import json
from typing import Dict
from datetime import datetime, timezone


class GNSSConstants:
    """Physical and system constants for GNSS calculations"""
    # GPS Constants (IS-GPS-200)
    GPS_MU = 3.986005e14  # Earth's gravitational parameter (m^3/s^2)
    GPS_OMEGA_E = 7.2921151467e-5  # Earth's rotation rate (rad/s)
    GPS_C = 299792458.0  # Speed of light (m/s)
    GPS_F = -4.442807633e-10  # Relativistic correction term (s/m^(1/2))

    # Galileo Constants (OS-SIS-ICD)
    GAL_MU = 3.986004418e14  # Earth's gravitational parameter (m^3/s^2)
    GAL_OMEGA_E = 7.2921151467e-5  # Earth's rotation rate (rad/s)
    GAL_C = 299792458.0  # Speed of light (m/s)
    GAL_F = -4.442807309e-10  # Relativistic correction term (s/m^(1/2))

    # Time constants
    GPS_WEEK_SECONDS = 604800  # Seconds in a week
    GAL_WEEK_SECONDS = 604800  # Seconds in a week
    GPS_EPOCH = datetime(1980, 1, 6, 0, 0, 0, tzinfo=timezone.utc)
    GAL_EPOCH = datetime(1999, 8, 22, 0, 0, 0, tzinfo=timezone.utc)  # GST epoch


class SatelliteEphemeris:
    """Base class for satellite ephemeris calculations"""

    def __init__(self, sat_id: str, eph_data: Dict[str, float], constellation: str):
        self.sat_id = sat_id
        self.constellation = constellation
        self.eph_data = eph_data

        # Common ephemeris parameters
        self.sqrt_a = eph_data.get('sqrt_A')
        self.e = eph_data.get('e')
        self.i0 = eph_data.get('i0')
        self.omega0 = eph_data.get('Omega0')
        self.omega = eph_data.get('omega')
        self.M0 = eph_data.get('M0')
        self.delta_n = eph_data.get('delta_n0')
        self.omega_dot = eph_data.get('Omega_dot')
        self.idot = eph_data.get('IDOT')

        # Perturbation corrections
        self.cuc = eph_data.get('Cuc')
        self.cus = eph_data.get('Cus')
        self.crc = eph_data.get('Crc')
        self.crs = eph_data.get('Crs')
        self.cic = eph_data.get('Cic')
        self.cis = eph_data.get('Cis')

        # Time parameters
        self.toe = eph_data.get('toe')  # Reference time of ephemeris
        self.toc = eph_data.get('toc', self.toe)  # Clock correction reference time

        # Clock correction parameters
        self.af0 = eph_data.get('af0')
        self.af1 = eph_data.get('af1')
        self.af2 = eph_data.get('af2', 0.0)

        # Week number (handle both 'week' and 'WN' keys)
        self.week = eph_data.get('week', eph_data.get('WN'))

    def validate(self) -> bool:
        """Validate that all required parameters are present"""
        required = [
            self.sqrt_a, self.e, self.i0, self.omega0, self.omega, self.M0,
            self.delta_n, self.omega_dot, self.idot, self.cuc, self.cus,
            self.crc, self.crs, self.cic, self.cis, self.toe, self.af0, self.af1
        ]
        return all(x is not None for x in required)


class GPSSatellite(SatelliteEphemeris):
    """GPS satellite ephemeris and position calculations"""

    def __init__(self, sat_id: str, eph_data: Dict[str, float]):
        super().__init__(sat_id, eph_data, "GPS")
        self.mu = GNSSConstants.GPS_MU
        self.omega_e = GNSSConstants.GPS_OMEGA_E
        self.F = GNSSConstants.GPS_F


class GalileoSatellite(SatelliteEphemeris):
    """Galileo ephemeris and position calculations"""

    def __init__(self, sat_id: str, eph_data: Dict[str, float]):
        super().__init__(sat_id, eph_data, "Galileo")
        self.mu = GNSSConstants.GAL_MU
        self.omega_e = GNSSConstants.GAL_OMEGA_E
        self.F = GNSSConstants.GAL_F


def create_satellite(sat_id: str, eph_data: Dict[str, float]):
    """Create satellite object"""

    if sat_id.startswith('G'):
        return GPSSatellite(sat_id, eph_data)
    elif sat_id.startswith('E'):
        return GalileoSatellite(sat_id, eph_data)
    else:
        raise ValueError(f"Invalid satellite ID: {sat_id}. Must start with 'G' or 'E'")

def load_ephemeris(json_file: str, sat_id: str, gps_tow: float):
    with open(json_file, 'r') as f:
        data = json.load(f)

    # Determine constellation
    if sat_id.startswith('G'):
        constellation = 'GPS'
    elif sat_id.startswith('E'):
        constellation = 'Galileo'
    else:
        raise ValueError(f"Invalid satellite ID: {sat_id}. Must start with 'G' or 'E'")

    sat_ephemeris_dict = data[constellation][sat_id]

    available_tows = []
    for tow_key in sat_ephemeris_dict.keys():
        tow_str = tow_key.split('_')[1]
        tow_value = float(tow_str)

        time_diff = gps_tow - tow_value

        # Handle week wraparound
        if time_diff < -302400:
            time_diff += 604800
        elif time_diff > 302400:
            time_diff -= 604800

        # If Ephemeris is not within 2 hours of current TOW
        if abs(time_diff) <= 7200:
            available_tows.append((tow_value, tow_key, time_diff))

    if available_tows:
        # Find the closest ephemeris to gps_tow
        best_tow, best_key, best_diff = min(available_tows, key=lambda x: abs(x[2]))

        eph_data = sat_ephemeris_dict[best_key]

        # print(f"Selected ephemeris for {sat_id}: toe={best_tow}, age={smallest_positive_diff:.1f}s")

        return create_satellite(sat_id, eph_data)
    else:
        raise ValueError(f"No valid ephemeris found for {sat_id} near TOW {gps_tow}")

def get_available_satellites(json_file: str):
    with open(json_file, 'r') as f:
        data = json.load(f)

    results = {
        'GPS': sorted(data.get('GPS', {}).keys()),
        'Galileo': sorted(data.get('Galileo', {}).keys())
    }

    return results