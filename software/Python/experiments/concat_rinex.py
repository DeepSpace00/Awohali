#!/usr/bin/env python3
"""
concat_rinex_obs.py

Concatenate multiple RINEX observation CSV files into a single file.
Writes the header once, then appends all data rows in chronological order.

Usage:
    python3 concat_rinex_obs.py -o combined_obs.csv obs_day1.csv obs_day2.csv obs_day3.csv

    # Or use glob patterns:
    python3 concat_rinex_obs.py -o combined_obs.csv obs_*.csv

    # Skip epoch sorting (preserve file order as-is):
    python3 concat_rinex_obs.py --no-sort -o combined_obs.csv obs_*.csv
"""

import argparse
import csv
import sys
from pathlib import Path


EPOCH_COL = "epoch"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Concatenate RINEX observation CSV files."
    )
    parser.add_argument(
        "files",
        nargs="+",
        type=Path,
        help="Input TSV files (in the order you want them concatenated).",
    )
    parser.add_argument(
        "-o", "--output",
        type=Path,
        required=True,
        help="Output TSV file path.",
    )
    parser.add_argument(
        "--no-sort",
        action="store_true",
        help="Skip sorting by epoch; preserve file order instead.",
    )
    parser.add_argument(
        "--encoding",
        default="utf-8",
        help="File encoding (default: utf-8).",
    )
    return parser.parse_args()


def read_tsv(path: Path, encoding: str) -> tuple[list[str], list[dict]]:
    """Return (fieldnames, rows) from a TSV file."""
    with open(path, newline="", encoding=encoding) as f:
        reader = csv.DictReader(f, delimiter=",")
        fieldnames = reader.fieldnames
        if fieldnames is None:
            raise ValueError(f"No header found in {path}")
        rows = list(reader)
    return list(fieldnames), rows


def main():
    args = parse_args()

    # Validate inputs
    for p in args.files:
        if not p.exists():
            sys.exit(f"Error: input file not found: {p}")

    # Read all files, verify header consistency
    reference_fields = None
    all_rows = []

    for path in args.files:
        fields, rows = read_tsv(path, args.encoding)

        if reference_fields is None:
            reference_fields = fields
        elif fields != reference_fields:
            # Warn but don't abort — columns may be in a different order
            extra = set(fields) - set(reference_fields)
            missing = set(reference_fields) - set(fields)
            if extra or missing:
                print(
                    f"Warning: {path.name} has mismatched columns.\n"
                    f"  Extra:   {extra or 'none'}\n"
                    f"  Missing: {missing or 'none'}",
                    file=sys.stderr,
                )

        all_rows.extend(rows)
        print(f"  Read {len(rows):>7,} rows from {path.name}")

    print(f"\nTotal rows before dedup check: {len(all_rows):,}")

    # Optional sort by epoch
    if not args.no_sort:
        if EPOCH_COL not in reference_fields:
            print(
                f"Warning: '{EPOCH_COL}' column not found; skipping sort.",
                file=sys.stderr,
            )
        else:
            all_rows.sort(key=lambda r: r[EPOCH_COL])
            print("Rows sorted by epoch.")

    # Write output
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output, "w", newline="", encoding=args.encoding) as f:
        writer = csv.DictWriter(
            f,
            fieldnames=reference_fields,
            delimiter=",",
            extrasaction="ignore",   # drop columns not in reference header
        )
        writer.writeheader()
        writer.writerows(all_rows)

    print(f"Wrote {len(all_rows):,} rows → {args.output}")


if __name__ == "__main__":
    main()