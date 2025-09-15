"""
ubxrawx2csv.py

Usage:

python3 ubxrawx2csv.py filename=pygpsdata.log outputfile=rawx.csv

Parses RXM-RAWX messages from a UBX binary logfile and outputs
them into a usable CSV format.

Created on 11 Sep 2025
"""

import csv
from sys import argv
from pyubx2.ubxreader import (
    ERR_LOG,
    GET,
    UBX_PROTOCOL,
    VALCKSUM,
    UBXReader,
)

ubx_file = "software/ubx_data/GNSS011.ubx"
output_file = "software/ubx_data/GNSS011.csv"

def errhandler(err):
    """Handles errors output by iterator."""
    print(f"\nERROR: {err}\n")


def main(**kwargs):
    """Main Routine."""

    filename = kwargs.get("filename", ubx_file)
    outputfile = kwargs.get("outputfile", output_file)

    print(f"Opening file {filename}...")

    # CSV headers
    headers = [
        "rcvTow",   # Receiver time of week (s)
        "week",     # GPS week
        "leapS",    # Leap seconds
        "numMeas",  # Number of measurements in this block
        "prMes",    # Pseudorange (m)
        "cpMes",    # Carrier phase (cycles)
        "doMes",    # Doppler (Hz)
        "gnssId",   # GNSS ID
        "svId",     # Satellite ID
        "freqId",   # Frequency ID
        "locktime", # Lock time indicator
        "cno",      # Carrier-to-noise ratio (dB-Hz)
        "prStd",    # Pseudorange std dev
        "cpStd",    # Carrier phase std dev
        "doStd"     # Doppler std dev
    ]

    with open(filename, "rb") as stream, open(outputfile, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(headers)

        count = 0

        ubr = UBXReader(
            stream,
            protfilter=UBX_PROTOCOL,  # UBX only
            quitonerror=ERR_LOG,
            validate=VALCKSUM,
            msgmode=GET,
            parsebitfield=True,
            errorhandler=errhandler,
        )

        for _, parsed_data in ubr:
            if parsed_data.identity == "RXM-RAWX":
                # Iterate over each measurement
                for i in range(1, parsed_data.numMeas + 1):
                    row = [
                        parsed_data.rcvTow,
                        parsed_data.week,
                        parsed_data.leapS,
                        parsed_data.numMeas,
                        getattr(parsed_data, f"prMes_{i}", None),
                        getattr(parsed_data, f"cpMes_{i}", None),
                        getattr(parsed_data, f"doMes_{i}", None),
                        getattr(parsed_data, f"gnssId_{i}", None),
                        getattr(parsed_data, f"svId_{i}", None),
                        getattr(parsed_data, f"freqId_{i}", None),
                        getattr(parsed_data, f"locktime_{i}", None),
                        getattr(parsed_data, f"cno_{i}", None),
                        getattr(parsed_data, f"prStdev_{i}", None),
                        getattr(parsed_data, f"cpStdev_{i}", None),
                        getattr(parsed_data, f"doStdev_{i}", None),
                    ]
                    writer.writerow(row)
                    count += 1

    print(f"\n{count} RXM-RAWX measurements written to {outputfile}\n")
    print("Conversion Complete")


if __name__ == "__main__":
    main(**dict(arg.split("=") for arg in argv[1:]))
