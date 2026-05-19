#!/usr/bin/env python3
"""
Delete a range of rows from data_raw.csv.

Usage examples:
  # Delete by row index (1-based, excluding header):
  python trim_csv.py data_raw.csv --row-start 100 --row-end 200

  # Delete by timestamp range (nanoseconds):
  python trim_csv.py data_raw.csv --ts-start 1778180400000000000 --ts-end 1778180500000000000

  # Dry-run (preview without writing):
  python trim_csv.py data_raw.csv --row-start 100 --row-end 200 --dry-run

  # Write to a different output file (default: overwrites input):
  python trim_csv.py data_raw.csv --row-start 100 --row-end 200 -o output.csv
"""

import argparse
import csv
import sys
from pathlib import Path


def load_csv(path: Path):
    with open(path, newline="") as f:
        raw = f.read()

    comment = ""
    lines = raw.splitlines(keepends=True)
    data_lines = []
    for line in lines:
        if line.startswith("#"):
            comment += line
        else:
            data_lines.append(line)

    reader = csv.DictReader(data_lines)
    rows = list(reader)
    fieldnames = reader.fieldnames
    return comment, fieldnames, rows


def trim_by_row(rows, row_start, row_end):
    """Remove rows in [row_start, row_end] (1-based, inclusive)."""
    if row_start < 1 or row_end < row_start:
        sys.exit(f"Invalid row range: {row_start}–{row_end}")
    before = rows[: row_start - 1]
    after = rows[row_end:]
    removed = rows[row_start - 1 : row_end]
    return before + after, removed


def trim_by_timestamp(rows, ts_start, ts_end):
    """Remove rows where timestamp is in [ts_start, ts_end] (inclusive)."""
    ts_col = rows[0] and list(rows[0].keys())[0]
    kept, removed = [], []
    for row in rows:
        ts = int(float(row[ts_col]))
        if ts_start <= ts <= ts_end:
            removed.append(row)
        else:
            kept.append(row)
    return kept, removed


def write_csv(path: Path, comment: str, fieldnames, rows):
    with open(path, "w", newline="") as f:
        if comment:
            f.write(comment)
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description="Delete a range of rows from a CSV file.")
    parser.add_argument("input", help="Path to data_raw.csv")
    parser.add_argument("-o", "--output", help="Output path (default: overwrite input)")
    parser.add_argument("--row-start", type=int, help="First row to delete (1-based, data rows only)")
    parser.add_argument("--row-end", type=int, help="Last row to delete (1-based, inclusive)")
    parser.add_argument("--ts-start", type=int, help="Timestamp range start (ns, inclusive)")
    parser.add_argument("--ts-end", type=int, help="Timestamp range end (ns, inclusive)")
    parser.add_argument("--dry-run", action="store_true", help="Preview only, do not write")
    args = parser.parse_args()

    use_row = args.row_start is not None or args.row_end is not None
    use_ts = args.ts_start is not None or args.ts_end is not None

    if use_row and use_ts:
        sys.exit("Specify either --row-start/--row-end or --ts-start/--ts-end, not both.")
    if not use_row and not use_ts:
        sys.exit("Specify a deletion range via --row-start/--row-end or --ts-start/--ts-end.")

    src = Path(args.input)
    if not src.exists():
        sys.exit(f"File not found: {src}")

    comment, fieldnames, rows = load_csv(src)
    total = len(rows)

    if use_row:
        row_start = args.row_start
        row_end = args.row_end if args.row_end is not None else row_start
        kept, removed = trim_by_row(rows, row_start, row_end)
        print(f"Row range: {row_start}–{row_end}")
    else:
        ts_start = args.ts_start
        ts_end = args.ts_end if args.ts_end is not None else ts_start
        kept, removed = trim_by_timestamp(rows, ts_start, ts_end)
        print(f"Timestamp range: {ts_start} – {ts_end}")

    print(f"Total rows: {total}  |  Removed: {len(removed)}  |  Remaining: {len(kept)}")

    if args.dry_run:
        print("Dry-run mode — no files written.")
        if removed:
            ts_col = list(removed[0].keys())[0]
            print(f"First removed timestamp: {removed[0][ts_col]}")
            print(f"Last  removed timestamp: {removed[-1][ts_col]}")
        return

    dst = Path(args.output) if args.output else src
    write_csv(dst, comment, fieldnames, kept)
    print(f"Written to: {dst}")


if __name__ == "__main__":
    main()
