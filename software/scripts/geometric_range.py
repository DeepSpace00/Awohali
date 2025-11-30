"""
Calculate satellite ECEF position from broadcast ephemeris and compute slant range
to a ground point using WGS-84 parameters
"""

import math
from datetime import datetime, timezone
from typing import Tuple, Dict, Any
from software.scripts.ephemeris_classes import SatelliteEphemeris, GNSSConstants, load_ephemeris

class PositionCalculator:
    """Calculate satellite positions from ephemeris"""

    @staticmethod
    def calculate_eccentric_anomaly(M: float, e: float, tol: float = 1e-12, max_iter: int = 15) -> float:
        """
        Calculate eccentric anomaly using iterative method

        Args:
            @type M: float
            @param M: Mean anomaly (rad)

            @type e: float
            @param e: Eccentricity

            @type tol: float
            @param tol: Convergence tolerance

            @type max_iter: int
            @param max_iter: Maximum number of iterations

        Returns:
            @rtype: float
            @return: Eccentric anomaly (rad)
        """

        E = M # Initial guess
        for _ in range(max_iter):
            E_new = M + e * math.sin(E)
            if abs(E_new - E) < tol:
                return E_new
            E = E_new

        return E

    @staticmethod
    def calculate_satellite_position(sat: SatelliteEphemeris, t_tx: float) -> Tuple[float, float, float]:
        """
        Calculate satellite ECEF coordinates at transmission time

        Args:
            @type sat: SatelliteEphemeris
            @param sat: Satellite ephemeris object

            @type t_tx: float
            @param t_tx: Transmission time in system time (seconds)

        Returns:
            @rtype: Tuple[float, float, float]
            @return: Tuple of (x, y, z) ECEF coordinates in meters
        """

        # Semi-major axis
        a = sat.sqrt_a ** 2

        # Time from ephemeris reference time
        tk = t_tx - sat.toe

        # Handle week crossovers
        if tk > 302400:
            tk -= 604800
        elif tk < -302400:
            tk += 604800

        # Corrected mean motion
        n0 = math.sqrt(sat.mu / (a ** 3))
        n = n0 + sat.delta_n

        # Mean anomaly
        M = sat.M0 + n * tk

        # Eccentric anomaly (iterative solution)
        E = PositionCalculator.calculate_eccentric_anomaly(M, sat.e)

        # True anomaly
        sin_nu = (math.sqrt(1 - sat.e ** 2) * math.sin(E)) / (1 - sat.e * math.cos(E))
        cos_nu = (math.cos(E) - sat.e) / (1 - sat.e * math.cos(E))
        nu = math.atan2(sin_nu, cos_nu)

        # Argument of latitude
        phi = nu + sat.omega

        # Argument of latitude corrections
        du = sat.cus * math.sin(2 * phi) + sat.cuc * math.cos(2 * phi)

        # Radius correction
        dr = sat.crs * math.sin(2 * phi) + sat.crc * math.cos(2 * phi)

        # Inclination Correction
        di = sat.cis * math.sin(2 * phi) + sat.cic * math.cos(2 * phi)

        # Corrected argument of latitude
        u = phi + du

        # Corrected radius
        r = a * (1 - sat.e * math.cos(E)) + dr

        # Corrected inclination
        i = sat.i0 + di + sat.idot * tk

        # Positions in orbital plane
        x_prime = r * math.cos(u)
        y_prime = r * math.sin(u)

        # Corrected longitude of ascending node
        omega_k = sat.omega0 + (sat.omega_dot - sat.omega_e) * tk - sat.omega_e * sat.toe

        # Earth-fixed coordinates
        x = x_prime * math.cos(omega_k) - y_prime * math.cos(i) * math.sin(omega_k)
        y = x_prime * math.sin(omega_k) + y_prime * math.cos(i) * math.cos(omega_k)
        z = y_prime * math.sin(i)

        return x, y, z

class TimeConverter:
    """Convert between different time systems"""

    @staticmethod
    def utc_to_gps_time(utc_time: datetime) -> Tuple[int, float]:
        """
        Convert UTC to GPS time
        Note: Assumes 18 leap seconds

        Args:
            @type utc_time: datetime
            @param utc_time: UTC datetime object

        Returns:
            @rtype: Tuple[int, float]
            @return: Tuple of (GPS week number, seconds into week)
        """

        # Leap seconds from GPS epoch to current (approximate)
        leap_seconds = 18

        delta = utc_time - GNSSConstants.GPS_EPOCH
        gps_seconds = delta.total_seconds() + leap_seconds

        week = int(gps_seconds / GNSSConstants.GPS_WEEK_SECONDS)
        tow = gps_seconds % GNSSConstants.GPS_WEEK_SECONDS

        return week, tow

    @staticmethod
    def utc_to_galileo_time(utc_time: datetime) -> Tuple[int, float]:
        """
        Convert UTC to Galileo time
        Note: Assumes 18 leap seconds

        Args:
            @type utc_time: datetime
            @param utc_time: UTC datetime object

        Returns:
            @rtype: Tuple[int, float]
            @return: Tuple of (Galileo week number, seconds into week)
        """

        # Leap seconds from GAL epoch to current (approximate)
        leap_seconds = 18

        delta = utc_time - GNSSConstants.GAL_EPOCH
        gst_seconds = delta.total_seconds() + leap_seconds

        week = int(gst_seconds / GNSSConstants.GAL_WEEK_SECONDS)
        tow = gst_seconds % GNSSConstants.GAL_WEEK_SECONDS

        return week, tow

class GeometricRangeCalculator:
    """Calculate geometric range between receiver and satellite"""

    @staticmethod
    def rotate_satellite_position(sat_pos: Tuple[float, float, float], omega_e: float,
                                  tau: float)-> Tuple[float, float, float]:
        """
        Rotate satellite position to account for Earth rotation during signal transit

        The satellite position is computed at transmission time in the ECEF frame at that instant. Durring signal
        propagation, Earth rotates, so we need to rotate the satellite position to the ECEF frame at reception time.

        Args:
            @type sat_pos: Tuple[float, float, float]
            @param sat_pos: Satellite ECEF position at transmission time (x, y, z) in meters

            @type omega_e: float
            @param omega_e: Earth rotation rate (rad/s)

            @type tau: float
            @param tau: Signal travel time (seconds)

        Returns:
            @rtype: Tuple[float, float, float]
            @return: Rotated satellite ECEF position at reception time (x, y, z) in meters
        """

        # Rotation angle durring signal propagation
        theta = omega_e * tau

        # Rotation matrix
        x_rot = math.cos(theta) * sat_pos[0] + math.sin(theta) * sat_pos[1]
        y_rot = -math.sin(theta) * sat_pos[0] + math.cos(theta) * sat_pos[1]
        z_rot = sat_pos[2]

        return x_rot, y_rot, z_rot

    @staticmethod
    def calculate_geometric_range(sat_pos: Tuple[float, float, float], rcv_pos: Tuple[float, float, float]) -> float:
        """
        Calculate geometric range from receiver to satellite

        Args:
            @type sat_pos: Tuple[float, float, float]
            @param sat_pos: Satellite ECEF coordinates (x, y, z) in meters

            @type rcv_pos: Tuple[float, float, float]
            @param rcv_pos: Receiver ECEF coordinates (x, y, z) in meters

        Returns:
            @rtype: float
            @return: Geometric range in meters
        """

        dx = sat_pos[0] - rcv_pos[0]
        dy = sat_pos[1] - rcv_pos[1]
        dz = sat_pos[2] - rcv_pos[2]

        return math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

def calculate_satellite_position_and_range(json_file: str, sat_id: str, rcv_pos: Tuple[float, float, float],
                                           utc_time: datetime = None, gps_week: int = None,
                                           gps_tow: float = None) -> Dict[str, Any]:
    """
    Function to calculate satellite position and geometric range

    Args:
        @type json_file: str
        @param json_file: Path to JSON file containing ephemeris data
        @type sat_id: str
        @param sat_id: Satellite ID (e.g., 'G01' for GPS PRN 1, 'E01' for Galileo SVID 1)
        @type rcv_pos: Tuple[float, float, float]
        @param rcv_pos: Receiver ECEF coordinates (x, y, z) in meters
        @type utc_time: datetime
        @param utc_time: Time in system time (seconds)
        @type gps_week: int
        @param gps_week: GPS week number (optional if utc_time is provided) - use WN from RXM-RAWX
        @type gps_tow: float
        @param gps_tow: GPS time of week in seconds (optional if utc_time is provided) - use rcvTow from RXM-RAWX

    Returns:
        @rtype: Dict[str, Any]
        @return: Dictionary with satellite position, geometric range, and related data
    """

    sat = load_ephemeris(json_file, sat_id, gps_tow)

    if sat_id.startswith('G'):
        if gps_week is not None and gps_tow is not None:
            week = gps_week
            tow = gps_tow
        elif utc_time is not None:
            week, tow = TimeConverter.utc_to_week(utc_time)
        else:
            raise ValueError("Must provide either utc_time or (gps_week, gps_tow)")

    elif sat_id.startswith('E'):
        if gps_week is not None and gps_tow is not None:
            week = gps_week
            tow = gps_tow
        elif utc_time is not None:
            week, tow = TimeConverter.utc_to_galileo_time(utc_time)
        else:
            raise ValueError("Must provide either utc_time or (gps_week, gps_tow)")

    else:
        raise ValueError(f"Invalid satellite ID: {sat_id}. Must start with 'G' or 'E'")

    # Calculate satellite position
    # Account for signal travel time (iterative) and Earth rotation)

    c = GNSSConstants.GPS_C
    tau = 0.075 # Initial guess: ~75ms | signal travel time
    for _ in range(3):
        t_tx = tow - tau

        # Calculate satellite position at transmission time
        sat_pos_tx = PositionCalculator.calculate_satellite_position(sat, t_tx)

        # Rotate satellite position to reception time to account for Earth rotation
        sat_pos = GeometricRangeCalculator.rotate_satellite_position(sat_pos_tx, sat.omega_e, tau)

        # Calculate geometric range with rotated position
        geometric_range = GeometricRangeCalculator.calculate_geometric_range(sat_pos, rcv_pos)

        tau = geometric_range / c # signal travel time

    # Final calculation with corrected transmission time
    t_tx = tow - tau
    sat_pos_tx = PositionCalculator.calculate_satellite_position(sat, t_tx)
    sat_pos = GeometricRangeCalculator.rotate_satellite_position(sat_pos_tx, sat.omega_e, tau)

    # Calculate range with corrections
    geometric_range = GeometricRangeCalculator.calculate_geometric_range(sat_pos, rcv_pos)

    results = {
        'satellite_id': sat_id,
        'constellation': sat.constellation,
        'time_source': 'gps_week_tow' if (gps_week is not None and gps_tow is not None) else 'utc_converted',
        'system_time': {
            'week': week,
            'tow': tow
        },
        'transmission_time': t_tx,
        'satellite_ecef_m': {
            'x': sat_pos[0],
            'y': sat_pos[1],
            'z': sat_pos[2]
        },
        'receiver_ecef_m': {
            'x': rcv_pos[0],
            'y': rcv_pos[1],
            'z': rcv_pos[2]
        },
        'geometric_range_m': geometric_range
    }

    return results

def main():
    """Example usage"""

    json_file = "ephemeris.json"
    utc_time = datetime(2025, 9, 30, 18, 45, 30, tzinfo=timezone.utc)
    receiver_ecef = (867089.785, -5504797.877, 3092195.664) # Example receiver position
    gps_tow = 240595.004
    gps_week = 2386
    sat_id = 'G04'

    print("\n" + "="*80)
    print("GNSS Geometric Range Calculation")
    print("="*80)

    results = calculate_satellite_position_and_range(json_file, sat_id, receiver_ecef, utc_time, gps_week, gps_tow)

    print(f"\n{results['satellite_id']} ({results['constellation']}):")
    print(f"  Geometric Range:          {results['geometric_range_m']:>15,.2f} m")
    print(f"  Sat Clock Correction:     {results['sat_clock_correction_m']:>15,.2f} m")
    print(f"  Corrected Range:          {results['corrected_range_m']:>15,.2f} m")

if __name__ == "__main__":
    main()