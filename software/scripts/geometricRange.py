#!./.venv/bin/python Python 3.13.5
"""
Calculate satellite ECEF position from broadcast ephemeris and compute slant range
to a ground point using WGS-84 parameters
"""

import json
import math
from datetime import datetime, timedelta
import sys


class SatellitePositionCalculator:
    """Calculate satellite position from GPS/Galileo ephemeris"""
    
    # WGS-84 Constants
    WGS84_A = 6378137.0  # Semi-major axis (m)
    WGS84_E = 0.0818191908426  # First eccentricity
    WGS84_E2 = WGS84_E ** 2  # e^2
    
    # Earth rotation rate (rad/s)
    OMEGA_E_GPS = 7.2921151467e-5  # GPS (WGS-84)
    OMEGA_E_GAL = 7.2921151467e-5  # Galileo (same as GPS)
    
    # Gravitational constant * Earth mass (m^3/s^2)
    GM_GPS = 3.986005e14  # GPS
    GM_GAL = 3.986004418e14  # Galileo
    
    # Speed of light (m/s)
    C = 299792458.0
    
    def __init__(self, ephemeris: dict):
        """Initialize with ephemeris dictionary"""
        self.eph = ephemeris
        
        # Determine constellation
        if 'sqrt_A' in ephemeris:
            self.constellation = 'GPS' if 'IODC' in ephemeris else 'Galileo'
        else:
            raise ValueError("Invalid ephemeris format")
        
        # Set constellation-specific constants
        if self.constellation == 'GPS':
            self.omega_e = self.OMEGA_E_GPS
            self.gm = self.GM_GPS
        else:
            self.omega_e = self.OMEGA_E_GAL
            self.gm = self.GM_GAL
    
    def calculate_satellite_position(self, target_time: datetime) -> tuple:
        """
        Calculate satellite ECEF position at given UTC time
        Returns: (x, y, z) in meters, or None if ephemeris is not valid
        """
        # Parse Toe_UTC
        toe_dt = datetime.strptime(self.eph['Toe_UTC'], '%Y-%m-%d %H:%M:%S')
        
        # Calculate time from ephemeris reference epoch
        dt = (target_time - toe_dt).total_seconds()
        
        # Check if ephemeris is valid (typically ±2 hours from Toe)
        if abs(dt) > 7200:
            print(f"Warning: Time difference from Toe is {dt/3600:.2f} hours")
            print(f"Ephemeris may not be valid (typically valid for ±2 hours)")
        
        # Extract ephemeris parameters
        sqrt_a = self.eph['sqrt_A']
        e = self.eph['e']
        m0 = self.eph['M0']
        delta_n = self.eph['Delta_n']
        omega = self.eph['Omega']
        omega0 = self.eph['Omega0']
        omega_dot = self.eph['Omega_dot']
        i0 = self.eph['i0']
        idot = self.eph['IDOT']
        cuc = self.eph['Cuc']
        cus = self.eph['Cus']
        crc = self.eph['Crc']
        crs = self.eph['Crs']
        cic = self.eph['Cic']
        cis = self.eph['Cis']
        
        # Semi-major axis
        a = sqrt_a ** 2
        
        # Computed mean motion (rad/s)
        n0 = math.sqrt(self.gm / (a ** 3))
        
        # Corrected mean motion
        n = n0 + delta_n
        
        # Mean anomaly
        mk = m0 + n * dt
        
        # Solve Kepler's equation for eccentric anomaly (iterative)
        ek = mk  # Initial guess
        for _ in range(10):
            ek_new = mk + e * math.sin(ek)
            if abs(ek_new - ek) < 1e-12:
                break
            ek = ek_new
        
        # True anomaly
        sin_vk = (math.sqrt(1 - e**2) * math.sin(ek)) / (1 - e * math.cos(ek))
        cos_vk = (math.cos(ek) - e) / (1 - e * math.cos(ek))
        vk = math.atan2(sin_vk, cos_vk)
        
        # Argument of latitude
        phik = vk + omega
        
        # Second harmonic perturbations
        delta_uk = cus * math.sin(2 * phik) + cuc * math.cos(2 * phik)
        delta_rk = crs * math.sin(2 * phik) + crc * math.cos(2 * phik)
        delta_ik = cis * math.sin(2 * phik) + cic * math.cos(2 * phik)
        
        # Corrected argument of latitude
        uk = phik + delta_uk
        
        # Corrected radius
        rk = a * (1 - e * math.cos(ek)) + delta_rk
        
        # Corrected inclination
        ik = i0 + idot * dt + delta_ik
        
        # Positions in orbital plane
        xk_prime = rk * math.cos(uk)
        yk_prime = rk * math.sin(uk)
        
        # Corrected longitude of ascending node
        omegak = omega0 + (omega_dot - self.omega_e) * dt - self.omega_e * self.eph['Toe']
        
        # Earth-fixed coordinates
        xk = xk_prime * math.cos(omegak) - yk_prime * math.cos(ik) * math.sin(omegak)
        yk = xk_prime * math.sin(omegak) + yk_prime * math.cos(ik) * math.cos(omegak)
        zk = yk_prime * math.sin(ik)
        
        return (xk, yk, zk)
    
    @staticmethod
    def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> tuple:
        """
        Convert geodetic coordinates (WGS-84) to ECEF
        
        Args:
            lat_deg: Latitude in degrees
            lon_deg: Longitude in degrees
            alt_m: Altitude above ellipsoid in meters
        
        Returns:
            (x, y, z) in meters
        """
        lat = math.radians(lat_deg)
        lon = math.radians(lon_deg)
        
        # Radius of curvature in prime vertical
        N = SatellitePositionCalculator.WGS84_A / math.sqrt(
            1 - SatellitePositionCalculator.WGS84_E2 * math.sin(lat)**2
        )
        
        x = (N + alt_m) * math.cos(lat) * math.cos(lon)
        y = (N + alt_m) * math.cos(lat) * math.sin(lon)
        z = (N * (1 - SatellitePositionCalculator.WGS84_E2) + alt_m) * math.sin(lat)
        
        return (x, y, z)
    
    @staticmethod
    def calculate_slant_range(sat_pos: tuple, ground_pos: tuple) -> float:
        """
        Calculate slant range between satellite and ground point
        
        Args:
            sat_pos: Satellite ECEF position (x, y, z) in meters
            ground_pos: Ground ECEF position (x, y, z) in meters
        
        Returns:
            Slant range in kilometers
        """
        dx = sat_pos[0] - ground_pos[0]
        dy = sat_pos[1] - ground_pos[1]
        dz = sat_pos[2] - ground_pos[2]
        
        range_m = math.sqrt(dx**2 + dy**2 + dz**2)
        return range_m / 1000.0  # Convert to km
    
    @staticmethod
    def ecef_to_geodetic(x: float, y: float, z: float) -> tuple:
        """
        Convert ECEF coordinates to geodetic (WGS-84)
        
        Returns:
            (latitude_deg, longitude_deg, altitude_m)
        """
        # Longitude
        lon = math.atan2(y, x)
        
        # Iterative solution for latitude and altitude
        p = math.sqrt(x**2 + y**2)
        lat = math.atan2(z, p * (1 - SatellitePositionCalculator.WGS84_E2))
        
        for _ in range(10):
            N = SatellitePositionCalculator.WGS84_A / math.sqrt(
                1 - SatellitePositionCalculator.WGS84_E2 * math.sin(lat)**2
            )
            alt = p / math.cos(lat) - N
            lat_new = math.atan2(z, p * (1 - SatellitePositionCalculator.WGS84_E2 * N / (N + alt)))
            
            if abs(lat_new - lat) < 1e-12:
                break
            lat = lat_new
        
        return (math.degrees(lat), math.degrees(lon), alt)


def main():
    # ===== CONFIGURATION =====
    # Hard-coded parameters - edit these values
    GROUND_ECEF = (863791.147, -5503184.660, 3095966.747)  # X, Y, Z in meters
    SATELLITE_ID = "E19"  # Satellite ID with prefix (e.g., "G18" for GPS, "E24" for Galileo)
    TARGET_TIME = datetime(2025, 9, 30, 17, 8, 44)  # UTC time
    EPHEMERIS_FILE = 'ephemeris_EVK.json'  # Path to JSON file
    # =========================
    
    if len(sys.argv) > 1:
        print("Note: Command-line arguments are ignored. Edit hard-coded values in script.")
    
    # Load ephemeris JSON
    print(f"Loading ephemeris from: {EPHEMERIS_FILE}")
    try:
        with open(EPHEMERIS_FILE, 'r') as f:
            ephemeris_data = json.load(f)
    except FileNotFoundError:
        print(f"Error: File '{EPHEMERIS_FILE}' not found")
        sys.exit(1)
    
    # Parse satellite ID
    if not SATELLITE_ID or len(SATELLITE_ID) < 2:
        print("Error: Invalid SATELLITE_ID format. Use 'G##' for GPS or 'E##' for Galileo")
        sys.exit(1)
    
    sat_prefix = SATELLITE_ID[0].upper()
    try:
        sat_num = int(SATELLITE_ID[1:])
    except ValueError:
        print("Error: Invalid SATELLITE_ID format. Use 'G##' for GPS or 'E##' for Galileo")
        sys.exit(1)
    
    # Determine constellation
    if sat_prefix == 'G':
        constellation = 'GPS'
    elif sat_prefix == 'E':
        constellation = 'Galileo'
    else:
        print(f"Error: Unknown satellite prefix '{sat_prefix}'. Use 'G' for GPS or 'E' for Galileo")
        sys.exit(1)
    
    # Find all ephemeris entries for this satellite
    sat_prefix_str = f"{sat_prefix}{sat_num:02d}_"
    available_eph = {k: v for k, v in ephemeris_data[constellation].items() 
                     if k.startswith(sat_prefix_str)}
    
    if not available_eph:
        print(f"Error: No ephemeris found for {constellation} satellite {SATELLITE_ID}")
        available_sats = list(set([k.split('_')[0] for k in ephemeris_data[constellation].keys()]))
        print(f"Available {constellation} satellites: {sorted(available_sats)}")
        sys.exit(1)
    
    # Select ephemeris closest to target time
    best_eph_key = None
    min_time_diff = float('inf')
    
    print(f"\nSearching for best ephemeris for {SATELLITE_ID}...")
    print(f"Found {len(available_eph)} ephemeris entries")
    
    for eph_key, eph in available_eph.items():
        toe_dt = datetime.strptime(eph['Toe_UTC'], '%Y-%m-%d %H:%M:%S')
        time_diff = abs((TARGET_TIME - toe_dt).total_seconds())
        
        if time_diff < min_time_diff:
            min_time_diff = time_diff
            best_eph_key = eph_key
    
    eph = available_eph[best_eph_key]
    
    # Calculate satellite position
    print(f"\n{'='*60}")
    print(f"Satellite Position and Slant Range Calculation")
    print(f"{'='*60}")
    print(f"Satellite: {best_eph_key} ({constellation})")
    print(f"Target Time: {TARGET_TIME} UTC")
    print(f"Ephemeris Toe: {eph['Toe_UTC']} UTC")
    print(f"Time from Toe: {min_time_diff/3600:.2f} hours")
    
    if min_time_diff > 7200:
        print(f"WARNING: Target time is {min_time_diff/3600:.2f} hours from Toe")
        print(f"         Ephemeris may not be valid (typically valid for ±2 hours)")
    
    calc = SatellitePositionCalculator(eph)
    sat_pos = calc.calculate_satellite_position(TARGET_TIME)
    
    print(f"\nSatellite ECEF Position:")
    print(f"  X: {sat_pos[0]:14.3f} m")
    print(f"  Y: {sat_pos[1]:14.3f} m")
    print(f"  Z: {sat_pos[2]:14.3f} m")
    
    # Convert to geodetic for reference
    sat_lat, sat_lon, sat_alt = calc.ecef_to_geodetic(*sat_pos)
    print(f"\nSatellite Geodetic Position:")
    print(f"  Latitude:  {sat_lat:10.6f}°")
    print(f"  Longitude: {sat_lon:10.6f}°")
    print(f"  Altitude:  {sat_alt/1000:10.3f} km")
    
    print(f"\nGround ECEF Position:")
    print(f"  X: {GROUND_ECEF[0]:14.3f} m")
    print(f"  Y: {GROUND_ECEF[1]:14.3f} m")
    print(f"  Z: {GROUND_ECEF[2]:14.3f} m")
    
    # Convert ground position to geodetic for reference
    ground_lat, ground_lon, ground_alt = calc.ecef_to_geodetic(*GROUND_ECEF)
    print(f"\nGround Geodetic Position:")
    print(f"  Latitude:  {ground_lat:10.6f}°")
    print(f"  Longitude: {ground_lon:10.6f}°")
    print(f"  Altitude:  {ground_alt:10.3f} m")
    
    # Calculate slant range
    slant_range = calc.calculate_slant_range(sat_pos, GROUND_ECEF)
    
    print(f"\n{'='*60}")
    print(f"SLANT RANGE: {slant_range:,.3f} km")
    print(f"{'='*60}\n")


if __name__ == '__main__':
    main()