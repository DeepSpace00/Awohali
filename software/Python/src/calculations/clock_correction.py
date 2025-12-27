"""
Calculate receiver-satellite clock offset errors
"""

import math
from calculations.geometric_range import PositionCalculator
from ephemerisdes.ephemeris import SatelliteEphemeris


def calculate_accumulated_clock_bias(clock_df):
    """Track when clkBias resets and calculate accumulated offset"""
    accumulated_offset = 0
    prev_bias = clock_df.iloc[0]['clkB']
    accumulated_biases = []

    for i, row in clock_df.iterrows():
        current_bias = row['clkB']

        # Detect reset (large negative jump)
        if current_bias < prev_bias - 1e9:  # Reset detected (>1 second jump back)
            accumulated_offset += prev_bias  # Add previous value to offset

        accumulated_biases.append(accumulated_offset + current_bias)
        prev_bias = current_bias

    clock_df['accumulated_clkB'] = accumulated_biases
    return clock_df

def calculate_satellite_clock_offset(sat: SatelliteEphemeris, t: float) -> float:
    """
    Calculate satellite clock correction

    Args:
        @type sat: SatelliteEphemeris
        @param sat: Satellite ephemeris object

        @type t: float
        @param t: Time in system time (seconds)

    Returns:
        @rtype: float
        @return: Satellite clock offset in seconds
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

    return dt_sv


def calculate_relativistic_clock_correction(sat: SatelliteEphemeris, t: float) -> float:
    """
    Calculate relativistic clock correction

    Args:
        @type sat: SatelliteEphemeris
        @param sat: Satellite ephemeris object
        @type t: float
        @param t: Time in system time (seconds)

    Returns:
        @rtype: float
        @return: Relativistic clock correction in seconds
    """

    # Time from clock reference epoch
    dt = t - sat.toc

    # Handle week crossovers
    if dt > 302400:
        dt -= 604800
    elif dt < -302400:
        dt += 604800

    # Relativistic correction
    a = sat.sqrt_a ** 2
    M = sat.M0 + math.sqrt(sat.mu / (a ** 3)) * dt
    E = PositionCalculator.calculate_eccentric_anomaly(M, sat.e)
    dt_rel = sat.F * sat.sqrt_a * sat.e * math.sin(E)

    return dt_rel