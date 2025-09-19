#!/usr/bin/env python3
"""
UBX File Decoder to CSV - rewritten

Behavior change: timestamps for RAWX and HPPOSLLH are synchronized to the most
recent valid NAV-PVT message (year, month, day, hour, min, sec). If no valid
PVT has been seen yet the 'timestamp' field is set to None.  No fallback to
current wall time is used.

Dependencies:
    pip install pyubx2 pandas
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import sys
import struct
import pandas as pd
from pyubx2 import UBXReader

# Global state: most recent valid PVT datetime (timezone-aware UTC) or None
last_pvt_dt = None


def build_pvt_datetime_from_msg(msg):
    try:
        year = int(getattr(msg, "year", 0))
        month = int(getattr(msg, "month", 1))
        day = int(getattr(msg, "day", 1))
        hour = int(getattr(msg, "hour", 0))
        minute = int(getattr(msg, "min", 0))
        sec = int(getattr(msg, "sec", 0))
        nano = int(getattr(msg, "nano", 0))

        # If sec == 0 but iTOW is available, recompute sec from iTOW
        iTOW = getattr(msg, "iTOW", None)
        if iTOW and sec == 0:
            # iTOW is in ms; convert to seconds of day
            tow_seconds = (iTOW / 1000.0) % (24 * 3600)
            hour = int(tow_seconds // 3600)
            minute = int((tow_seconds % 3600) // 60)
            sec = int(tow_seconds % 60)
            nano = int((tow_seconds - int(tow_seconds)) * 1e9)

        microsecond = max(0, min(999999, nano // 1000))
        return datetime(year, month, day, hour, minute, sec, microsecond, tzinfo=timezone.utc)
    except Exception:
        return None



def decode_pvt_message(msg):
    """
    Decode NAV-PVT into a dict and update global last_pvt_dt if valid.
    Returns a list with a single dict (to match other decoders that return lists).
    """
    global last_pvt_dt
    dt = build_pvt_datetime_from_msg(msg)
    if dt is not None:
        last_pvt_dt = dt

    pvt = {
        "message_type": "PVT",
        # timestamp is the datetime derived from msg fields, or None if invalid
        "timestamp": dt.isoformat() if dt else None,
        "iTOW": getattr(msg, "iTOW", 0) / 1000.0,
        "year": getattr(msg, "year", 0),
        "month": getattr(msg, "month", 0),
        "day": getattr(msg, "day", 0),
        "hour": getattr(msg, "hour", 0),
        "min": getattr(msg, "min", 0),
        "sec": getattr(msg, "sec", 0),
        "nano": getattr(msg, "nano", 0),
        "valid": getattr(msg, "valid", 0),
        "tAcc": getattr(msg, "tAcc", 0),
        "fixType": getattr(msg, "fixType", 0),
        "flags": getattr(msg, "flags", 0),
        "numSV": getattr(msg, "numSV", 0),
        "lon": getattr(msg, "lon", 0) * 1e-7,
        "lat": getattr(msg, "lat", 0) * 1e-7,
        "height": getattr(msg, "height", 0) / 1000.0,
        "hMSL": getattr(msg, "hMSL", 0) / 1000.0,
        "hAcc": getattr(msg, "hAcc", 0) / 1000.0,
        "vAcc": getattr(msg, "vAcc", 0) / 1000.0,
    }
    return [pvt]


def decode_rawx_message(msg):
    """
    Extract RAWX message data into list of dicts (one per measurement).
    Each returned dict will include 'timestamp' synchronized to last valid PVT (or None).
    """
    global last_pvt_dt

    base = {
        "message_type": "RAWX",
        "timestamp": last_pvt_dt.isoformat() if last_pvt_dt else None,
        "iTOW": getattr(msg, "iTOW", 0) / 1000.0,
        "week": getattr(msg, "week", 0),
        "leapS": getattr(msg, "leapS", 0),
        "numMeas": getattr(msg, "numMeas", 0),
        "recStat": getattr(msg, "recStat", 0),
        "version": getattr(msg, "version", 0),
    }

    payload = getattr(msg, "payload", b"")
    num_meas = int(getattr(msg, "numMeas", 0))
    records = []

    # RAWX header is 16 bytes (rcvTow double, week uint16, leapS uint8, numMeas uint8, recStat uint8, version uint8, reserved uint16)
    if len(payload) >= 16 and num_meas > 0:
        try:
            header = struct.unpack("<dHBBBBH", payload[:16])
        except struct.error:
            header = None

        measurement_size = 32  # UBX RAWX measurement block is 32 bytes
        expected = 16 + (num_meas * measurement_size)
        if len(payload) < expected:
            # payload may be truncated; still attempt to parse what we can
            num_meas = max(0, (len(payload) - 16) // measurement_size)

        for i in range(num_meas):
            offset = 16 + (i * measurement_size)
            meas_payload = payload[offset : offset + measurement_size]
            if len(meas_payload) < measurement_size:
                continue
            try:
                # prMes(8d) cpMes(8d) doMes(4f) gnssId(1B) svId(1B) sigId(1B) freqId(1B)
                # locktime(2H) cno(1B) prStdev(1B) cpStdev(1B) doStdev(1B) trkStat(1B) reserved(1B)
                unpacked = struct.unpack("<ddfBBBBHBBBBBB", meas_payload)
                rec = {
                    **base,
                    "meas_index": i,
                    "prMes": unpacked[0],
                    "cpMes": unpacked[1],
                    "doMes": unpacked[2],
                    "gnssId": unpacked[3],
                    "svId": unpacked[4],
                    "sigId": unpacked[5],
                    "freqId": unpacked[6],
                    "locktime": unpacked[7],
                    "cno": unpacked[8],
                    "prStdev": unpacked[9],
                    "cpStdev": unpacked[10],
                    "doStdev": unpacked[11],
                    "trkStat": unpacked[12],
                }
                records.append(rec)
            except struct.error:
                # If unpack fails, skip measurement but keep going
                continue

    # If no per-measurement records were created, still return the base record (user may want the header info)
    if not records:
        return [base]
    return records


def decode_hpposllh_message(msg):
    """
    Extract HPPOSLLH message into a dict. Timestamp is synchronized to last valid PVT (or None).
    Attempts to parse high-precision fields from payload; falls back to pyubx2 fields if parsing fails.
    """
    global last_pvt_dt

    payload = getattr(msg, "payload", b"")
    ts = last_pvt_dt.isoformat() if last_pvt_dt else None

    # Preferred unpack formats:
    # 1) payload size >= 36: [version(1), reserved(1), reserved(1), invalidLlh(1), iTOW(U4),
    #    lon(i4), lat(i4), height(i4), hMSL(i4), lonHp(i1), latHp(i1), heightHp(i1), hMSLHp(i1), hAcc(U4), vAcc(U4)]
    #    struct format: '<BBBBIiiii4bII' -> totals 36 bytes
    # 2) payload size >= 32: [iTOW(U4), lon(i4), lat(i4), height(i4), hMSL(i4), lonHp(i1), latHp(i1), heightHp(i1), hMSLHp(i1), hAcc(U4), vAcc(U4)]
    #    struct format: '<Iiiii4bII' -> totals 32 bytes
    try:
        if len(payload) >= 36:
            unpacked = struct.unpack("<BBBBIiiii4bII", payload[:36])
            version = unpacked[0]
            invalidLlh = unpacked[3]
            iTOW = unpacked[4]
            lon_raw = unpacked[5]
            lat_raw = unpacked[6]
            height_raw = unpacked[7]
            hMSL_raw = unpacked[8]
            lonHp_raw = unpacked[9]
            latHp_raw = unpacked[10]
            heightHp_raw = unpacked[11]
            hMSLHp_raw = unpacked[12]
            hAcc_raw = unpacked[13]
            vAcc_raw = unpacked[14]
        elif len(payload) >= 32:
            unpacked = struct.unpack("<Iiiii4bII", payload[:32])
            iTOW = unpacked[0]
            lon_raw = unpacked[1]
            lat_raw = unpacked[2]
            height_raw = unpacked[3]
            hMSL_raw = unpacked[4]
            lonHp_raw = unpacked[5]
            latHp_raw = unpacked[6]
            heightHp_raw = unpacked[7]
            hMSLHp_raw = unpacked[8]
            hAcc_raw = unpacked[9]
            vAcc_raw = unpacked[10]
            version = getattr(msg, "version", 0)
            invalidLlh = getattr(msg, "invalidLlh", 0)
        else:
            # Not enough payload to parse HP fields; fall back to parsed attributes
            raise struct.error("HPPOSLLH payload too short for manual unpack")
    except struct.error:
        # Fallback: use pyubx2-provided attributes (no HP fields)
        return [{
            "message_type": "HPPOSLLH",
            "timestamp": ts,
            "version": getattr(msg, "version", 0),
            "invalidLlh": getattr(msg, "invalidLlh", 0),
            "iTOW": getattr(msg, "iTOW", 0) / 1000.0,
            "lon": getattr(msg, "lon", 0.0),
            "lat": getattr(msg, "lat", 0.0),
            "height": getattr(msg, "height", 0.0) / 1000.0,
            "hMSL": getattr(msg, "hMSL", 0.0) / 1000.0,
            "lonHp": None,
            "latHp": None,
            "heightHp": None,
            "hMSLHp": None,
            "hAcc": getattr(msg, "hAcc", 0.0) / 1000.0,
            "vAcc": getattr(msg, "vAcc", 0.0) / 1000.0,
            "lon_full": getattr(msg, "lon", 0.0),
            "lat_full": getattr(msg, "lat", 0.0),
            "height_full": getattr(msg, "height", 0.0) / 1000.0,
        }]

    # Convert unpacked raw fields to human units
    lon = lon_raw * 1e-7
    lat = lat_raw * 1e-7
    height_m = height_raw / 1000.0
    hMSL_m = hMSL_raw / 1000.0

    lon_hp = lonHp_raw * 1e-9
    lat_hp = latHp_raw * 1e-9
    height_hp_mm = heightHp_raw * 0.1  # 0.1 mm resolution
    hMSLHp_mm = hMSLHp_raw * 0.1

    hAcc_mm = hAcc_raw * 0.1
    vAcc_mm = vAcc_raw * 0.1

    rec = {
        "message_type": "HPPOSLLH",
        "timestamp": ts,
        "version": version,
        "invalidLlh": invalidLlh,
        "iTOW": iTOW / 1000.0,
        "lon": lon,
        "lat": lat,
        "height": height_m,
        "hMSL": hMSL_m,
        "lonHp": lon_hp,
        "latHp": lat_hp,
        "heightHp": height_hp_mm,
        "hMSLHp": hMSLHp_mm,
        "hAcc": hAcc_mm,
        "vAcc": vAcc_mm,
        "lon_full": lon + lon_hp,
        "lat_full": lat + lat_hp,
        "height_full": height_m + (height_hp_mm / 1000.0),
    }
    return [rec]


def process_ubx_file(input_file, output_file=None, message_filter=None, separate_files=False):
    """
    Process UBX file and convert to CSV.
    Timestamps for RAWX/HPPOSLLH come from the most recent valid PVT message (see last_pvt_dt).
    """
    input_path = Path(input_file)
    if not input_path.exists():
        print(f"Error: Input file '{input_file}' not found")
        return False

    if message_filter is None:
        message_filter = ["RAWX", "HPPOSLLH", "PVT"]

    rawx_data = []
    hppos_data = []
    pvt_data = []
    message_counts = {}
    error_count = 0

    try:
        with open(input_path, "rb") as fh:
            ubx_reader = UBXReader(fh, quitonerror=False)
            for raw, parsed in ubx_reader:
                try:
                    if parsed is None:
                        continue
                    if not hasattr(parsed, "identity"):
                        continue
                    mid = parsed.identity

                    if mid == "NAV-PVT" and "PVT" in message_filter:
                        records = decode_pvt_message(parsed)
                        pvt_data.extend(records)
                        message_counts["PVT"] = message_counts.get("PVT", 0) + 1

                    elif mid == "RXM-RAWX" and "RAWX" in message_filter:
                        records = decode_rawx_message(parsed)
                        rawx_data.extend(records)
                        message_counts["RAWX"] = message_counts.get("RAWX", 0) + 1

                    elif mid == "NAV-HPPOSLLH" and "HPPOSLLH" in message_filter:
                        records = decode_hpposllh_message(parsed)
                        hppos_data.extend(records)
                        message_counts["HPPOSLLH"] = message_counts.get("HPPOSLLH", 0) + 1

                except Exception as e:
                    error_count += 1
                    if error_count <= 10:
                        print(f"Error processing message: {e}")

    except Exception as e:
        print(f"Error reading UBX file: {e}")
        return False

    total = len(rawx_data) + len(hppos_data) + len(pvt_data)
    print(f"Processing complete. Total records: {total}")
    print("Message counts:")
    for k, v in message_counts.items():
        print(f"  {k}: {v}")
    if error_count:
        print(f"Errors encountered: {error_count}")

    # Write CSVs
    success = False
    if separate_files:
        base = input_path.stem
        created = []
        if rawx_data:
            fname = f"{base}_rawx.csv"
            pd.DataFrame(rawx_data).to_csv(fname, index=False, float_format="%.9f")
            created.append(fname)
            print(f"Created {fname} ({len(rawx_data)} rows)")
            success = True
        if hppos_data:
            fname = f"{base}_hpposllh.csv"
            pd.DataFrame(hppos_data).to_csv(fname, index=False, float_format="%.9f")
            created.append(fname)
            print(f"Created {fname} ({len(hppos_data)} rows)")
            success = True
        if pvt_data:
            fname = f"{base}_pvt.csv"
            pd.DataFrame(pvt_data).to_csv(fname, index=False, float_format="%.9f")
            created.append(fname)
            print(f"Created {fname} ({len(pvt_data)} rows)")
            success = True

        if not created:
            print("No files created (no matching messages).")
            return False

    else:
        all_records = rawx_data + hppos_data + pvt_data
        if not all_records:
            print("No data extracted.")
            return False
        out_path = Path(output_file) if output_file else input_path.with_suffix(".csv")
        pd.DataFrame(all_records).to_csv(out_path, index=False, float_format="%.9f")
        print(f"Created combined CSV: {out_path} ({len(all_records)} rows)")
        success = True

    return success


def main():
    parser = argparse.ArgumentParser(description="Convert UBX binary file to CSV (timestamps from NAV-PVT)")
    parser.add_argument("input_file", help="Input UBX file")
    parser.add_argument("-o", "--output", help="Output CSV (when not --separate)")
    parser.add_argument(
        "-m", "--messages", nargs="+", choices=["RAWX", "HPPOSLLH", "PVT"],
        help="Message types to extract (default: all)"
    )
    parser.add_argument("--separate", action="store_true", help="Write separate CSVs per message type")
    args = parser.parse_args()

    ok = process_ubx_file(args.input_file, output_file=args.output, message_filter=args.messages, separate_files=args.separate)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
