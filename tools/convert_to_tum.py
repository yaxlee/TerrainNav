#!/usr/bin/env python3
"""Convert a DGVI-SLAM/OKVIS trajectory CSV to TUM format (standard library only)."""

import argparse
import csv
from decimal import Decimal, InvalidOperation
from pathlib import Path


def convert(source: Path, destination: Path) -> None:
    if source.resolve() == destination.resolve():
        raise ValueError("Input and output paths must differ")
    # Validate before creating the output to avoid leaving partial results.
    rows = []
    with source.open(newline="") as stream:
        reader = csv.reader(stream)
        next(reader, None)
        for line_number, row in enumerate(reader, start=2):
            if not row or not any(value.strip() for value in row):
                continue
            if len(row) < 8:
                raise ValueError(f"Line {line_number}: expected at least 8 columns")
            try:
                values = [Decimal(value.strip()) for value in row[:8]]
            except InvalidOperation as error:
                raise ValueError(f"Line {line_number}: invalid numeric value") from error
            if not all(value.is_finite() for value in values):
                raise ValueError(f"Line {line_number}: non-finite value")
            timestamp = values[0] / Decimal(1_000_000_000)
            pose = " ".join(value.strip() for value in row[1:8])
            rows.append(f"{timestamp:.9f} {pose}\n")
    with destination.open("x") as stream:
        stream.write("# timestamp_s tx ty tz qx qy qz qw\n")
        stream.writelines(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="Trajectory CSV with nanosecond timestamps")
    parser.add_argument("-o", "--output", type=Path, help="Default: INPUT_STEM_tum.txt")
    args = parser.parse_args()
    output = args.output or args.input.with_name(args.input.stem + "_tum.txt")
    try:
        convert(args.input, output)
    except (OSError, ValueError) as error:
        parser.exit(1, f"Error: {error}\n")
    print(output)


if __name__ == "__main__":
    main()
