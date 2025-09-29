#!./.venv/bin/python Python 3.13.5
"""
Satellite Geometry Calculator
Parses UBX-NAV-HPPOSECEF from .ubx file and reads ephemeris.json
to calculate geometric ranges, elevations, azimuths, and other geometric parameters.
"""

import json
import math
import struct
from datetime import datetime, timezone, timedelta


class SatelliteGeometryCalculator:
    """Calculate satellite geometry from stored ephemeris data and UBX position"""
    
    # WGS84 constants
    GM = 3.986005e14  # Earth's gravitational constant (m^3/s^2)
    OMEGA_E = 7.2921151467e-5  # Earth's rotation rate (rad/s)
    C = 299792458.0  # Speed of light (m/s)
    
    # WGS84 ellipsoid parameters
    A = 6378137.0  # Semi-major axis (m)
    F = 1.0 / 298.257223563  # Flattening
    E2 = F * (2 - F)  # First eccentricity squared
    
    # UBX message constants
    UBX_SYNC1 = 0xB5
    UBX_SYNC2 = 0x62
    UBX_CLASS_NAV = 0x01
    UBX_ID_HPPOSECEF = 0x14
    
    # GPS epoch
    GPS_EPOCH = datetime(1980, 1, 6, 0, 0, 0, tzinfo=timezone.utc)
    
    def __init__(self, ephemeris_file, ubx_file=None):
        """
        Load ephemeris data from JSON file and optionally parse UBX file
        
        Args:
            ephemeris_file: Path to ephemeris JSON file
            ubx_file: Optional path to UBX file (overrides position from JSON)
        """
        # Load ephemeris JSON
        with open(ephemeris_file, 'r') as f:
            self.data = json.load(f)
        
        self.timestamp = self.data.get('timestamp')
        self.ephemeris = self.data.get('ephemeris', {})
        self.stored_distances = self.data.get('satellite_distances', {})
        
        # Get receiver position from JSON or UBX file
        if ubx_file:
            print(f"Parsing UBX file for receiver position: {ubx_file}")
            self.receiver_ecef = self.parse_hpposecef_from_ubx(ubx_file)
        else:
            self.receiver_ecef = self.data.get('receiver_position_ecef')
        
        if not self.receiver_ecef:
            raise ValueError("No receiver position available (not in JSON and no UBX file provided)")
        
        # Convert receiver ECEF to LLA for elevation/azimuth calculations
        self.receiver_lla = self.ecef_to_lla(
            self.receiver_ecef['x'],
            self.receiver_ecef['y'],
            self.receiver_ecef['z']
        )
    
    @staticmethod
    def utc_to_gps_tow(utc_time_str):
        """
        Convert UTC time string to GPS Time of Week (seconds)
        
        Args:
            utc_time_str: UTC time string in ISO format (e.g., "2025-09-29T01:41:12.000185")
            
        Returns:
            GPS time of week in seconds
        """
        # Parse the UTC time string
        # Handle fractional seconds
        if '.' in utc_time_str:
            # Remove 'Z' if present and parse
            utc_time_str = utc_time_str.rstrip('Z')
            utc_time = datetime.fromisoformat(utc_time_str)
        else:
            utc_time = datetime.fromisoformat(utc_time_str.rstrip('Z'))
        
        # Ensure timezone aware
        if utc_time.tzinfo is None:
            utc_time = utc_time.replace(tzinfo=timezone.utc)
        
        # Calculate time since GPS epoch
        time_since_epoch = utc_time - SatelliteGeometryCalculator.GPS_EPOCH
        
        # GPS time in seconds
        gps_seconds = time_since_epoch.total_seconds()
        
        # Subtract leap seconds (GPS time doesn't account for leap seconds)
        # As of 2017, there are 18 leap seconds difference
        # This is approximate - for precise work, use a leap second table
        leap_seconds = 18
        gps_seconds += leap_seconds
        
        # Calculate GPS week and time of week
        gps_week = int(gps_seconds / 604800)
        gps_tow = gps_seconds % 604800
        
        print(f"UTC time: {utc_time.isoformat()}")
        print(f"GPS Week: {gps_week}, GPS TOW: {gps_tow:.6f} seconds")
        
        return gps_tow
    
    def parse_hpposecef_from_ubx(self, ubx_file):
        """
        Parse UBX file to extract HPPOSECEF message
        
        Args:
            ubx_file: Path to .ubx file
            
        Returns:
            dict with receiver ECEF position
        """
        with open(ubx_file, 'rb') as f:
            data = f.read()
        
        receiver_ecef = None
        idx = 0
        
        while idx < len(data) - 8:
            # Look for UBX header
            if data[idx] == self.UBX_SYNC1 and data[idx+1] == self.UBX_SYNC2:
                msg_class = data[idx+2]
                msg_id = data[idx+3]
                length = struct.unpack('<H', data[idx+4:idx+6])[0]
                
                if idx + 8 + length <= len(data):
                    if msg_class == self.UBX_CLASS_NAV and msg_id == self.UBX_ID_HPPOSECEF:
                        payload = data[idx+6:idx+6+length]
                        receiver_ecef = self.parse_hpposecef(payload)
                        # Keep the last valid one (or could return first, or collect all)
                
                idx += 8 + length
            else:
                idx += 1
        
        if not receiver_ecef:
            raise ValueError("No UBX-NAV-HPPOSECEF message found in file")
        
        print(f"Found receiver position: X={receiver_ecef['x']:.3f}m, "
              f"Y={receiver_ecef['y']:.3f}m, Z={receiver_ecef['z']:.3f}m")
        print(f"Position accuracy: {receiver_ecef['accuracy']:.4f}m")
        
        return receiver_ecef
    
    def parse_hpposecef(self, payload):
        """Parse UBX-NAV-HPPOSECEF message for high-precision ECEF position"""
        if len(payload) < 28:
            return None
        
        # Parse according to UBX protocol specification
        version = payload[0]
        iTOW = struct.unpack('<I', payload[4:8])[0]  # GPS time of week (ms)
        
        # Position in cm (standard precision)
        ecefX = struct.unpack('<i', payload[8:12])[0]   # cm
        ecefY = struct.unpack('<i', payload[12:16])[0]  # cm
        ecefZ = struct.unpack('<i', payload[16:20])[0]  # cm
        
        # High-precision component in 0.1mm
        ecefXHp = struct.unpack('b', payload[20:21])[0]  # 0.1mm
        ecefYHp = struct.unpack('b', payload[21:22])[0]  # 0.1mm
        ecefZHp = struct.unpack('b', payload[22:23])[0]  # 0.1mm
        
        pAcc = struct.unpack('<I', payload[24:28])[0]  # Position accuracy estimate (0.1mm)
        
        # Combine standard and high-precision components
        # Convert to meters: cm + (0.1mm * 0.0001) = cm * 0.01 + hp * 0.0001
        x = ecefX * 0.01 + ecefXHp * 0.0001  # meters
        y = ecefY * 0.01 + ecefYHp * 0.0001  # meters
        z = ecefZ * 0.01 + ecefZHp * 0.0001  # meters
        
        accuracy = pAcc * 0.0001  # meters
        
        return {
            'x': x,
            'y': y,
            'z': z,
            'accuracy': accuracy,
            'iTOW': iTOW
        }
    
    def ecef_to_lla(self, x, y, z):
        """
        Convert ECEF coordinates to Latitude, Longitude, Altitude
        
        Args:
            x, y, z: ECEF coordinates in meters
            
        Returns:
            (lat, lon, alt) in degrees and meters
        """
        # Longitude
        lon = math.atan2(y, x)
        
        # Iterative calculation for latitude and altitude
        p = math.sqrt(x**2 + y**2)
        lat = math.atan2(z, p * (1 - self.E2))
        
        for _ in range(5):
            N = self.A / math.sqrt(1 - self.E2 * math.sin(lat)**2)
            alt = p / math.cos(lat) - N
            lat = math.atan2(z, p * (1 - self.E2 * N / (N + alt)))
        
        N = self.A / math.sqrt(1 - self.E2 * math.sin(lat)**2)
        alt = p / math.cos(lat) - N
        
        return (math.degrees(lat), math.degrees(lon), alt)
    
    def calculate_satellite_position(self, ephemeris, gps_time):
        """
        Calculate satellite ECEF position at given GPS time
        
        Args:
            ephemeris: dict with orbital parameters
            gps_time: GPS time in seconds of week
            
        Returns:
            (x, y, z) in ECEF coordinates (meters)
        """
        # Extract ephemeris parameters
        a = ephemeris['semi_major_axis']
        e = ephemeris['eccentricity']
        i0 = ephemeris['inclination']
        omega0 = ephemeris['right_ascension']
        omega = ephemeris['argument_of_perigee']
        M0 = ephemeris['mean_anomaly']
        toe = ephemeris['toe']
        
        # Time from ephemeris reference epoch
        tk = gps_time - toe
        
        # Handle week crossovers
        if tk > 302400:
            tk -= 604800
        elif tk < -302400:
            tk += 604800
        
        # Mean motion
        n0 = math.sqrt(self.GM / (a**3))
        n = n0
        
        # Mean anomaly
        Mk = M0 + n * tk
        
        # Solve Kepler's equation for eccentric anomaly
        Ek = Mk
        for _ in range(10):
            Ek = Mk + e * math.sin(Ek)
        
        # True anomaly
        nu = math.atan2(math.sqrt(1 - e**2) * math.sin(Ek), 
                        math.cos(Ek) - e)
        
        # Argument of latitude
        phi = nu + omega
        
        # Radius
        r = a * (1 - e * math.cos(Ek))
        
        # Positions in orbital plane
        x_orb = r * math.cos(phi)
        y_orb = r * math.sin(phi)
        
        # Corrected longitude of ascending node
        omega_k = omega0 - self.OMEGA_E * tk
        
        # ECEF coordinates
        x = x_orb * math.cos(omega_k) - y_orb * math.cos(i0) * math.sin(omega_k)
        y = x_orb * math.sin(omega_k) + y_orb * math.cos(i0) * math.cos(omega_k)
        z = y_orb * math.sin(i0)
        
        return (x, y, z)
    
    def calculate_geometric_range(self, sat_ecef, receiver_ecef):
        """
        Calculate geometric range (straight-line distance) between satellite and receiver
        
        Args:
            sat_ecef: (x, y, z) satellite position
            receiver_ecef: dict or tuple with x, y, z
            
        Returns:
            range in meters
        """
        if isinstance(receiver_ecef, dict):
            rx, ry, rz = receiver_ecef['x'], receiver_ecef['y'], receiver_ecef['z']
        else:
            rx, ry, rz = receiver_ecef
        
        dx = sat_ecef[0] - rx
        dy = sat_ecef[1] - ry
        dz = sat_ecef[2] - rz
        
        return math.sqrt(dx**2 + dy**2 + dz**2)
    
    def calculate_elevation_azimuth(self, sat_ecef, receiver_ecef, receiver_lla):
        """
        Calculate elevation and azimuth angles
        
        Args:
            sat_ecef: (x, y, z) satellite position in ECEF
            receiver_ecef: dict or tuple with receiver ECEF position
            receiver_lla: (lat, lon, alt) in degrees and meters
            
        Returns:
            (elevation, azimuth) in degrees
        """
        if isinstance(receiver_ecef, dict):
            rx, ry, rz = receiver_ecef['x'], receiver_ecef['y'], receiver_ecef['z']
        else:
            rx, ry, rz = receiver_ecef
        
        lat, lon, alt = receiver_lla
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        
        # Vector from receiver to satellite in ECEF
        dx = sat_ecef[0] - rx
        dy = sat_ecef[1] - ry
        dz = sat_ecef[2] - rz
        
        # Convert to East-North-Up (ENU) coordinates
        e = -math.sin(lon_rad) * dx + math.cos(lon_rad) * dy
        n = (-math.sin(lat_rad) * math.cos(lon_rad) * dx 
             - math.sin(lat_rad) * math.sin(lon_rad) * dy 
             + math.cos(lat_rad) * dz)
        u = (math.cos(lat_rad) * math.cos(lon_rad) * dx 
             + math.cos(lat_rad) * math.sin(lon_rad) * dy 
             + math.sin(lat_rad) * dz)
        
        # Calculate elevation and azimuth
        horizontal_dist = math.sqrt(e**2 + n**2)
        elevation = math.degrees(math.atan2(u, horizontal_dist))
        azimuth = math.degrees(math.atan2(e, n))
        
        if azimuth < 0:
            azimuth += 360
        
        return (elevation, azimuth)
    
    def calculate_satellite_velocity(self, ephemeris, gps_time):
        """
        Calculate satellite velocity in ECEF
        
        Args:
            ephemeris: dict with orbital parameters
            gps_time: GPS time in seconds of week
            
        Returns:
            (vx, vy, vz) velocity in m/s
        """
        a = ephemeris['semi_major_axis']
        e = ephemeris['eccentricity']
        i0 = ephemeris['inclination']
        omega0 = ephemeris['right_ascension']
        omega = ephemeris['argument_of_perigee']
        M0 = ephemeris['mean_anomaly']
        toe = ephemeris['toe']
        
        tk = gps_time - toe
        if tk > 302400:
            tk -= 604800
        elif tk < -302400:
            tk += 604800
        
        n0 = math.sqrt(self.GM / (a**3))
        Mk = M0 + n0 * tk
        
        # Solve for Ek
        Ek = Mk
        for _ in range(10):
            Ek = Mk + e * math.sin(Ek)
        
        # Eccentric anomaly rate
        Ek_dot = n0 / (1 - e * math.cos(Ek))
        
        # True anomaly
        nu = math.atan2(math.sqrt(1 - e**2) * math.sin(Ek), math.cos(Ek) - e)
        
        # Argument of latitude and its rate
        phi = nu + omega
        phi_dot = Ek_dot * math.sqrt(1 - e**2) / (1 - e * math.cos(Ek))
        
        # Radius and radius rate
        r = a * (1 - e * math.cos(Ek))
        r_dot = a * e * math.sin(Ek) * Ek_dot
        
        # Velocities in orbital plane
        vx_orb = r_dot * math.cos(phi) - r * phi_dot * math.sin(phi)
        vy_orb = r_dot * math.sin(phi) + r * phi_dot * math.cos(phi)
        
        # Longitude of ascending node
        omega_k = omega0 - self.OMEGA_E * tk
        
        # ECEF velocities
        vx = (vx_orb * math.cos(omega_k) - vy_orb * math.cos(i0) * math.sin(omega_k) 
              + self.OMEGA_E * (vy_orb * math.cos(i0) * math.cos(omega_k) + vx_orb * math.sin(omega_k)))
        vy = (vx_orb * math.sin(omega_k) + vy_orb * math.cos(i0) * math.cos(omega_k)
              - self.OMEGA_E * (vx_orb * math.cos(omega_k) - vy_orb * math.cos(i0) * math.sin(omega_k)))
        vz = vy_orb * math.sin(i0)
        
        return (vx, vy, vz)
    
    def calculate_range_rate(self, sat_ecef, sat_vel, receiver_ecef):
        """
        Calculate range rate (relative velocity along line of sight)
        
        Args:
            sat_ecef: (x, y, z) satellite position
            sat_vel: (vx, vy, vz) satellite velocity
            receiver_ecef: dict or tuple with receiver position
            
        Returns:
            range rate in m/s (positive = moving away)
        """
        if isinstance(receiver_ecef, dict):
            rx, ry, rz = receiver_ecef['x'], receiver_ecef['y'], receiver_ecef['z']
        else:
            rx, ry, rz = receiver_ecef
        
        # Unit vector from receiver to satellite
        dx = sat_ecef[0] - rx
        dy = sat_ecef[1] - ry
        dz = sat_ecef[2] - rz
        
        range_dist = math.sqrt(dx**2 + dy**2 + dz**2)
        
        unit_x = dx / range_dist
        unit_y = dy / range_dist
        unit_z = dz / range_dist
        
        # Project satellite velocity onto line of sight
        # (Assuming receiver velocity is negligible or subtract it if known)
        range_rate = sat_vel[0] * unit_x + sat_vel[1] * unit_y + sat_vel[2] * unit_z
        
        return range_rate
    
    def calculate_all_geometry(self, gps_time=None, utc_time_str=None):
        """
        Calculate all geometric parameters for all satellites
        
        Args:
            gps_time: GPS time in seconds of week (uses iTOW from file if None)
            utc_time_str: UTC time string (alternative to gps_time)
            
        Returns:
            dict with geometry for each satellite
        """
        if not self.receiver_ecef:
            raise ValueError("No receiver position available")
        
        # Determine which time to use
        if utc_time_str:
            gps_time = self.utc_to_gps_tow(utc_time_str)
        elif gps_time is None:
            gps_time = self.receiver_ecef['iTOW'] / 1000.0
        
        geometry = {}
        
        for sat_id, eph in self.ephemeris.items():
            try:
                # Calculate satellite position and velocity
                sat_ecef = self.calculate_satellite_position(eph, gps_time)
                sat_vel = self.calculate_satellite_velocity(eph, gps_time)
                
                # Calculate geometric range
                geometric_range = self.calculate_geometric_range(sat_ecef, self.receiver_ecef)
                
                # Calculate elevation and azimuth
                elevation, azimuth = self.calculate_elevation_azimuth(
                    sat_ecef, self.receiver_ecef, self.receiver_lla
                )
                
                # Calculate range rate
                range_rate = self.calculate_range_rate(sat_ecef, sat_vel, self.receiver_ecef)
                
                geometry[sat_id] = {
                    'position_ecef': {
                        'x': sat_ecef[0],
                        'y': sat_ecef[1],
                        'z': sat_ecef[2]
                    },
                    'velocity_ecef': {
                        'vx': sat_vel[0],
                        'vy': sat_vel[1],
                        'vz': sat_vel[2],
                        'speed': math.sqrt(sat_vel[0]**2 + sat_vel[1]**2 + sat_vel[2]**2)
                    },
                    'geometric_range_m': geometric_range,
                    'geometric_range_km': geometric_range / 1000.0,
                    'range_rate_ms': range_rate,
                    'elevation_deg': elevation,
                    'azimuth_deg': azimuth,
                    'visible': elevation > 0  # Satellite is above horizon
                }
                
            except Exception as e:
                print(f"Error calculating geometry for {sat_id}: {e}")
        
        return geometry
    
    def export_geometry(self, output_file='satellite_geometry.json', gps_time=None, utc_time_str=None):
        """Export calculated geometry to JSON file"""
        geometry = self.calculate_all_geometry(gps_time, utc_time_str)
        
        # Determine final GPS time used
        if utc_time_str:
            final_gps_time = self.utc_to_gps_tow(utc_time_str)
        elif gps_time:
            final_gps_time = gps_time
        else:
            final_gps_time = self.receiver_ecef['iTOW'] / 1000.0
        
        # Count visible satellites
        visible_sats = sum(1 for g in geometry.values() if g['visible'])
        
        output = {
            'timestamp': self.timestamp,
            'computation_time_utc': utc_time_str if utc_time_str else None,
            'receiver_position_ecef': self.receiver_ecef,
            'receiver_position_lla': {
                'latitude_deg': self.receiver_lla[0],
                'longitude_deg': self.receiver_lla[1],
                'altitude_m': self.receiver_lla[2]
            },
            'gps_time_sow': final_gps_time,
            'satellite_geometry': geometry,
            'summary': {
                'total_satellites': len(geometry),
                'visible_satellites': visible_sats
            }
        }
        
        with open(output_file, 'w') as f:
            json.dump(output, f, indent=2)
        
        print(f"\nExported geometry for {len(geometry)} satellites to {output_file}")
        print(f"Visible satellites (elevation > 0°): {visible_sats}/{len(geometry)}")
        
        # Print summary
        if geometry:
            print("\nSatellite Geometry Summary:")
            print(f"{'Sat ID':<8} {'Range (km)':<12} {'Elevation':<12} {'Azimuth':<12} {'Range Rate (m/s)':<18} {'Visible'}")
            print("-" * 90)
            for sat_id, geom in sorted(geometry.items()):
                visible = "✓" if geom['visible'] else "✗"
                print(f"{sat_id:<8} {geom['geometric_range_km']:<12.3f} "
                      f"{geom['elevation_deg']:<12.1f} {geom['azimuth_deg']:<12.1f} "
                      f"{geom['range_rate_ms']:<18.3f} {visible}")


if __name__ == '__main__':
    import argparse
    import sys
    import os
    
    parser = argparse.ArgumentParser(
        description='Calculate satellite geometry from ephemeris JSON and UBX file'
    )
    parser.add_argument('ephemeris_file', help='Input ephemeris JSON file')
    parser.add_argument('ubx_file', help='Input UBX file for receiver position')
    parser.add_argument('geometry_file', nargs='?', default='satellite_geometry.json',
                       help='Output geometry JSON file (default: satellite_geometry.json)')
    
    time_group = parser.add_mutually_exclusive_group()
    time_group.add_argument('--gps-time', type=float, default=None,
                           help='GPS time of week in seconds (e.g., 290000.0)')
    time_group.add_argument('--utc-time', type=str, default=None,
                           help='UTC time in ISO format (e.g., "2025-09-29T01:41:12.000185" or "2025-09-29T01:41:12")')
    
    args = parser.parse_args()
    
    # Check if input files exist
    if not os.path.exists(args.ephemeris_file):
        print(f"Error: Ephemeris file '{args.ephemeris_file}' not found", file=sys.stderr)
        sys.exit(1)
    
    if not os.path.exists(args.ubx_file):
        print(f"Error: UBX file '{args.ubx_file}' not found", file=sys.stderr)
        sys.exit(1)
    
    try:
        calc = SatelliteGeometryCalculator(args.ephemeris_file, args.ubx_file)
        calc.export_geometry(args.geometry_file, args.gps_time, args.utc_time)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)