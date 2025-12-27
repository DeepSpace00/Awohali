from datetime import datetime, timezone, timedelta

GPS_LEAP_SECONDS = 18

def datetime_to_gps_tow(dt):
    """Convert datetime to GPS Time of Week (TOW) in seconds"""
    # GPS epoch: January 6, 1980 00:00:00 UTC
    gps_epoch = datetime(1980, 1, 6, 0, 0, 0, tzinfo=timezone.utc)

    # Ensure input datetime is timezone-aware (assume UTC if naive)
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)

    # Calculate seconds since GPS epoch
    delta = dt - gps_epoch
    gps_seconds = delta.total_seconds()

    # GPS week number and time of week
    gps_week = int(gps_seconds / 604800)  # 604800 seconds per week
    tow = gps_seconds % 604800

    return tow, gps_week


def datetime_to_gps_tow_precise(dt):
    gps_epoch = datetime(1980, 1, 6, 0, 0, 0, tzinfo=timezone.utc)
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)

    delta = dt - gps_epoch
    gps_seconds = delta.total_seconds() + GPS_LEAP_SECONDS

    gps_week = int(gps_seconds / 604800)
    tow = gps_seconds % 604800

    return tow, gps_week

def gps_tow_to_datetime(tow, gps_week, include_leap_seconds=True):
    """
    Convert GPS Time of Week to datetime

    Parameters:
    -----------
    tow : float
        Time of week in seconds (0-604800)
    gps_week : int
        GPS week number since GPS epoch
    include_leap_seconds : bool
        If True, converts to UTC by subtracting leap seconds
        If False, returns GPS time (which doesn't account for leap seconds)

    Returns:
    --------
    datetime : timezone-aware datetime in UTC
    """
    # GPS epoch: January 6, 1980 00:00:00 UTC
    gps_epoch = datetime(1980, 1, 6, 0, 0, 0, tzinfo=timezone.utc)

    # Calculate total seconds since GPS epoch
    total_seconds = gps_week * 604800 + tow

    # Convert to datetime (this is in GPS time)
    gps_time = gps_epoch + timedelta(seconds=total_seconds)

    # Convert to UTC if requested
    if include_leap_seconds:
        utc_time = gps_time - timedelta(seconds=GPS_LEAP_SECONDS)
        return utc_time

    return gps_time