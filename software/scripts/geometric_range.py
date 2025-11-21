"""
Calculate satellite ECEF position from broadcast ephemeris and compute slant range
to a ground point using WGS-84 parameters
"""

import json
import math
from datetime import datetime, timezone
from typing import Tuple, Dict, Any

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
    GPS_EPOCH = datetime(1980, 1, 6, 0, 0, 0, tzinfo=timezone.utc)
    GAL_EPOCH = datetime(1999, 8, 22, 0, 0, 0, tzinfo=timezone.utc)  # GST epoch

class SatelliteEphemeris:
    """Base class for satellite ephemeris data"""

    def __init__(self, sat_id: str, eph_data: Dict[str, float], constellation: str):
        self.sat_id = sat_id
        self.constellation = constellation
        self.eph_data = eph_data

        # Common ephemeris parameters
        self.sqrt_a = eph_data.get('sqrtA')
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
    """GPS satellite ephemeris and position calculation"""

    def __init__(self, sat_id: str, eph_data: Dict[str, float]):
        super().__init__(sat_id, eph_data, "GPS")
        self.mu = GNSSConstants.GPS_MU
        self.omega_e = GNSSConstants.GPS_OMEGA_E
        self.F = GNSSConstants.GPS_F

class GalileoSatellite(SatelliteEphemeris):
    """Galileo satellite ephemeris and position calculation"""

    def __init__(self, sat_id: str, eph_data: Dict[str, float]):
        super().__init__(sat_id, eph_data, "Galileo")
        self.mu = GNSSConstants.GAL_MU
        self.omega_e = GNSSConstants.GAL_OMEGA_E
        self.F = GNSSConstants.GAL_F

class PositionCalculator:
    """Calculate satellite positions from ephemeris"""

    @staticmethod
    def calculate_eccentric_anomaly(M: float, e: float, tol: float = 1e-12, max_iter: int = 15) -> float:
        """
        Calculate eccentric anomaly using iterative method

        Args:
            M: Mean anomaly (rad)
            e: Eccentricity
            tol: Convergence tolerance
            max_iter: Maximum iterations

        Returns:
            Eccentric anomaly (rad)
        """
        E = M  # Initial guess
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
            sat: SatelliteEphemeris object
            t_tx: Transmission time in system time (seconds)

        Returns:
            Tuple of (x, y, z) ECEF coordinates in meters
        """
        # Semi-major axis
        a = sat.sqrt_a ** 2

        # Time from ephemeris reference epoch
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

        # Second harmonic perturbations
        sin_2phi = math.sin(2 * phi)
        cos_2phi = math.cos(2 * phi)

        # Argument of latitude correction
        du = sat.cuc * cos_2phi + sat.cus * sin_2phi
        # Radius correction
        dr = sat.crc * cos_2phi + sat.crs * sin_2phi
        # Inclination correction
        di = sat.cic * cos_2phi + sat.cis * sin_2phi

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

        return (x, y, z)

    @staticmethod
    def calculate_clock_correction(sat: SatelliteEphemeris, t: float) -> float:
        """
        Calculate satellite clock correction

        Args:
            sat: SatelliteEphemeris object
            t: Time in system time (seconds)

        Returns:
            Clock correction in seconds
        """
        # Time from clock reference epoch
        dt = t - sat.toc

        # Handle week crossovers
        if dt > 302400:
            dt -= 604800
        elif dt < -302400:
            dt += 604800

        # Clock correction polynomial
        dt_sv = sat.af0 + sat.af1 * dt + sat.af2 * dt ** 2

        # Relativistic correction
        a = sat.sqrt_a ** 2
        E = PositionCalculator.calculate_eccentric_anomaly(
            sat.M0 + math.sqrt(sat.mu / (a ** 3)) * dt, sat.e
        )
        dt_r = sat.F * sat.sqrt_a * sat.e * math.sin(E)

        return dt_sv + dt_r

class TimeConverter:
    """Convert between different time systems"""

    @staticmethod
    def utc_to_gps_time(utc_time: datetime) -> Tuple[int, float]:
        """
        Convert UTC to GPS time (week number and time of week)
        Note: Does not account for leap seconds - assumes 18 leap seconds

        Args:
            utc_time: UTC datetime object

        Returns:
            Tuple of (GPS week number, seconds into week)
        """
        # Leap seconds from GPS epoch to current (approximate)
        leap_seconds = 18  # As of 2024, should be updated

        delta = utc_time - GNSSConstants.GPS_EPOCH
        gps_seconds = delta.total_seconds() + leap_seconds

        week = int(gps_seconds / GNSSConstants.GPS_WEEK_SECONDS)
        tow = gps_seconds % GNSSConstants.GPS_WEEK_SECONDS

        return week, tow

    @staticmethod
    def utc_to_galileo_time(utc_time: datetime) -> Tuple[int, float]:
        """
        Convert UTC to Galileo System Time (week number and time of week)
        Note: Does not account for leap seconds

        Args:
            utc_time: UTC datetime object

        Returns:
            Tuple of (GST week number, seconds into week)
        """
        # Leap seconds from GAL epoch to current (approximate)
        leap_seconds = 18  # Should be updated

        delta = utc_time - GNSSConstants.GAL_EPOCH
        gst_seconds = delta.total_seconds() + leap_seconds

        week = int(gst_seconds / GNSSConstants.GPS_WEEK_SECONDS)
        tow = gst_seconds % GNSSConstants.GPS_WEEK_SECONDS

        return week, tow

class RangeCalculator:
    """Calculate geometric range between receiver and satellite"""

    @staticmethod
    def rotate_satellite_position(sat_pos: Tuple[float, float, float],
                                  omega_e: float,
                                  tau: float) -> Tuple[float, float, float]:
        """
        Rotate satellite position to account for Earth rotation during signal transit

        The satellite position is computed at transmission time in the ECEF frame
        at that instant. During signal propagation, Earth rotates, so we need to
        rotate the satellite position to the ECEF frame at reception time.

        Args:
            sat_pos: Satellite ECEF position at transmission time (x, y, z) in meters
            omega_e: Earth rotation rate (rad/s)
            tau: Signal travel time (seconds)

        Returns:
            Rotated satellite ECEF position at reception time (x, y, z) in meters
        """
        # Rotation angle during signal propagation (Earth rotates eastward)
        theta = omega_e * tau

        # Rotation matrix around Z-axis (right-hand rule, positive = counterclockwise)
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        # Apply rotation: R_z(theta) * [x; y; z]
        x_rot = cos_theta * sat_pos[0] + sin_theta * sat_pos[1]
        y_rot = -sin_theta * sat_pos[0] + cos_theta * sat_pos[1]
        z_rot = sat_pos[2]  # Z component unchanged

        return (x_rot, y_rot, z_rot)

    @staticmethod
    def calculate_geometric_range(sat_pos: Tuple[float, float, float],
                                  rcv_pos: Tuple[float, float, float]) -> float:
        """
        Calculate geometric range from receiver to satellite

        Args:
            sat_pos: Satellite ECEF coordinates (x, y, z) in meters
            rcv_pos: Receiver ECEF coordinates (x, y, z) in meters

        Returns:
            Geometric range in meters
        """
        dx = sat_pos[0] - rcv_pos[0]
        dy = sat_pos[1] - rcv_pos[1]
        dz = sat_pos[2] - rcv_pos[2]

        return math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

    @staticmethod
    def calculate_range_with_correction(sat_pos: Tuple[float, float, float],
                                        rcv_pos: Tuple[float, float, float],
                                        sat: SatelliteEphemeris,
                                        t: float) -> Dict[str, float]:
        """
        Calculate range with clock corrections

        Args:
            sat_pos: Satellite ECEF coordinates
            rcv_pos: Receiver ECEF coordinates
            sat: SatelliteEphemeris object
            t: Time in system time (seconds)

        Returns:
            Dictionary with geometric range, clock correction, and corrected range
        """
        geom_range = RangeCalculator.calculate_geometric_range(sat_pos, rcv_pos)
        clock_corr = PositionCalculator.calculate_clock_correction(sat, t)

        return {
            'geometric_range_m': geom_range,
            'clock_correction_s': clock_corr,
            'clock_correction_m': clock_corr * GNSSConstants.GPS_C,
            'corrected_range_m': geom_range - clock_corr * GNSSConstants.GPS_C
        }

def load_ephemeris_data(json_file: str) -> Dict[str, Dict[str, Dict[str, float]]]:
    """
    Load ephemeris data from JSON file

    Args:
        json_file: Path to JSON file containing ephemeris data

    Returns:
        Dictionary with GPS and Galileo ephemeris data
    """
    with open(json_file, 'r') as f:
        return json.load(f)

def calculate_satellite_position_and_range(json_file: str,
                                           sat_id: str,
                                           receiver_ecef: Tuple[float, float, float],
                                           utc_time: datetime = None,
                                           gps_week: int = None,
                                           gps_tow: float = None,
                                           receiver_clock_bias: float = None) -> Dict[str, Any]:
    """
    Main function to calculate satellite position and range

    Args:
        json_file: Path to JSON file with ephemeris data
        sat_id: Satellite ID (e.g., 'G01' for GPS PRN 1, 'E01' for Galileo SVID 1)
        receiver_ecef: Receiver ECEF coordinates (x, y, z) in meters
        utc_time: UTC time for position calculation (optional if gps_week and gps_tow provided)
        gps_week: GPS week number (optional if utc_time provided) - use from RXM-RAWX
        gps_tow: GPS time of week in seconds (optional if utc_time provided) - use rcvTow from RXM-RAWX
        receiver_clock_bias: Optional receiver clock bias in seconds from NAV-CLOCK message.
                           If provided, the comparison will account for receiver clock bias.

    Returns:
        Dictionary containing satellite position, range, and related data
    """
    # Load ephemeris data
    eph_data = load_ephemeris_data(json_file)

    # Determine constellation
    if sat_id.startswith('G'):
        constellation = 'GPS'
        sat_data = eph_data['GPS'][sat_id]
        sat = GPSSatellite(sat_id, sat_data)

        # Get time parameters
        if gps_week is not None and gps_tow is not None:
            week = gps_week
            tow = gps_tow
        elif utc_time is not None:
            week, tow = TimeConverter.utc_to_gps_time(utc_time)
        else:
            raise ValueError("Must provide either utc_time or (gps_week, gps_tow)")

    elif sat_id.startswith('E'):
        constellation = 'Galileo'
        sat_data = eph_data['Galileo'][sat_id]
        sat = GalileoSatellite(sat_id, sat_data)

        # Get time parameters
        if gps_week is not None and gps_tow is not None:
            # Galileo uses GST which is aligned with GPS time for practical purposes
            # Both use same week numbering and TOW (with small constant offset)
            week = gps_week
            tow = gps_tow
        elif utc_time is not None:
            week, tow = TimeConverter.utc_to_galileo_time(utc_time)
        else:
            raise ValueError("Must provide either utc_time or (gps_week, gps_tow)")
    else:
        raise ValueError(f"Invalid satellite ID: {sat_id}. Must start with 'G' or 'E'")

    # Validate ephemeris
    if not sat.validate():
        raise ValueError(f"Incomplete ephemeris data for {sat_id}")

    # Calculate satellite position
    # Account for signal travel time (iterative) and Earth rotation
    c = GNSSConstants.GPS_C
    tau = 0.075  # Initial guess: ~75ms
    for _ in range(3):  # Few iterations sufficient
        t_tx = tow - tau
        # Calculate satellite position at transmission time
        sat_pos_tx = PositionCalculator.calculate_satellite_position(sat, t_tx)
        # Rotate satellite position to reception time to account for Earth rotation
        sat_pos = RangeCalculator.rotate_satellite_position(sat_pos_tx, sat.omega_e, tau)
        # Calculate geometric range with rotated position
        geom_range = RangeCalculator.calculate_geometric_range(sat_pos, receiver_ecef)
        tau = geom_range / c

    # Final calculation with corrected transmission time
    t_tx = tow - tau
    sat_pos_tx = PositionCalculator.calculate_satellite_position(sat, t_tx)
    sat_pos = RangeCalculator.rotate_satellite_position(sat_pos_tx, sat.omega_e, tau)

    # Calculate range with corrections
    range_data = RangeCalculator.calculate_range_with_correction(
        sat_pos, receiver_ecef, sat, t_tx
    )

    result = {
        'satellite_id': sat_id,
        'constellation': constellation,
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
            'x': receiver_ecef[0],
            'y': receiver_ecef[1],
            'z': receiver_ecef[2]
        },
        **range_data
    }

    # If receiver clock bias is provided, add expected pseudorange
    if receiver_clock_bias is not None:
        # Expected pseudorange = corrected_range + c * receiver_clock_bias
        expected_pseudorange = range_data['corrected_range_m'] + c * receiver_clock_bias
        result['receiver_clock_bias_s'] = receiver_clock_bias
        result['receiver_clock_bias_m'] = receiver_clock_bias * c
        result['expected_pseudorange_m'] = expected_pseudorange

    return result

# Example usage
if __name__ == "__main__":
    # Example: Calculate position for both GPS and Galileo satellites
    json_file = "ephemeris_sample.json"
    utc_time = datetime(2025, 9, 30, 18, 45, 34, tzinfo=timezone.utc)
    receiver_ecef = (863794.228, -5503182.971, 3095968.875)  # Example receiver position
    gps_tow = 240595.004
    gps_week = 2386

    # Example receiver clock bias from NAV-CLOCK message
    # u-blox NAV-CLOCK provides clkBias in nanoseconds
    # Convert to seconds: receiver_clock_bias_s = clkBias_ns / 1e9
    # This is a hypothetical value for demonstration - use your actual NAV-CLOCK data
    receiver_clock_bias_from_nav_clock = 3.8279e-3  # seconds (3.7 ms example)

    # Test both satellites
    satellites = [
        ("G01", 25049745),  # GPS with measured pseudorange
        ("E10", 27538215)   # Galileo with measured pseudorange
    ]

    print("\n" + "="*80)
    print("GNSS Range Calculation - With and Without Receiver Clock Bias")
    print("="*80)

    print("\n" + "-"*80)
    print("PART 1: Without receiver clock bias (showing implied bias)")
    print("-"*80)

    implied_biases = []
    for sat_id, measured_pseudorange in satellites:
        try:
            result = calculate_satellite_position_and_range(
                json_file, sat_id, receiver_ecef, utc_time
            )

            # Calculate implied receiver clock bias
            range_diff = measured_pseudorange - result['corrected_range_m']
            implied_rcv_clock_bias = range_diff / GNSSConstants.GPS_C
            implied_biases.append(implied_rcv_clock_bias)

            print(f"\n{result['satellite_id']} ({result['constellation']}):")
            print(f"  Corrected Range:     {result['corrected_range_m']:>15,.2f} m")
            print(f"  Measured Pseudorange:{measured_pseudorange:>15,.2f} m")
            print(f"  Difference:          {range_diff:>15,.2f} m")
            print(f"  → Implied receiver clock bias: {implied_rcv_clock_bias*1e3:>8.4f} ms")

        except Exception as e:
            print(f"Error processing {sat_id}: {e}")

    avg_bias = sum(implied_biases) / len(implied_biases) if implied_biases else 0
    print(f"\n  Average implied bias: {avg_bias*1e3:.4f} ms")

    print("\n" + "-"*80)
    print("PART 2: With receiver clock bias from NAV-CLOCK")
    print(f"Using receiver_clock_bias = {receiver_clock_bias_from_nav_clock*1e3:.4f} ms")
    print("-"*80)

    for sat_id, measured_pseudorange in satellites:
        try:
            result = calculate_satellite_position_and_range(
                json_file, sat_id, receiver_ecef, utc_time, gps_week, gps_tow,
                receiver_clock_bias=receiver_clock_bias_from_nav_clock
            )

            residual = measured_pseudorange - result['expected_pseudorange_m']

            print(f"\n{result['satellite_id']} ({result['constellation']}):")
            print(f"  Geometric Range:        {result['geometric_range_m']:>15,.2f} m")
            print(f"  Sat Clock Correction:   {result['clock_correction_m']:>15,.2f} m")
            print(f"  Corrected Range:        {result['corrected_range_m']:>15,.2f} m")
            print(f"  Receiver Clock Bias:    {result['receiver_clock_bias_m']:>15,.2f} m")
            print(f"  Expected Pseudorange:   {result['expected_pseudorange_m']:>15,.2f} m")
            print(f"  Measured Pseudorange:   {measured_pseudorange:>15,.2f} m")
            print(f"  Residual:               {residual:>15,.2f} m")
            print(f"  → Residual is ionosphere + troposphere + multipath + noise")

        except Exception as e:
            print(f"Error processing {sat_id}: {e}")

    print("\n" + "="*80)
    print("NOTES:")
    print("  • NAV-CLOCK contains the receiver's estimated clock bias (clkBias field)")
    print("  • Use: receiver_clock_bias_s = clkBias_ns / 1e9")
    print("  • Expected residual: 5-30m (ionosphere/troposphere + 1-10m multipath/noise)")
    print("  • If residuals are larger, check ephemeris quality and signal conditions")
    print("="*80)