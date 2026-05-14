#!/usr/bin/env python3
"""
Halve the camera frame rate of an OKVIS-format dataset by keeping every other
entry in each cam*/data.csv file. Image files are NOT deleted.
A backup of each original CSV is saved as data.csv.bak.

Usage:
    python3 halve_framerate.py <dataset_mav0_path>
Example:
    python3 halve_framerate.py /home/aelx/OKVIS2-X/Dataset/pa_1/mav0
"""

import sys
import os
import glob
import shutil

def halve_csv(csv_path):
    with open(csv_path, 'r') as f:
        lines = f.readlines()

    if not lines:
        print(f"  [skip] empty: {csv_path}")
        return

    # Separate header (lines starting with '#') from data lines
    header = [l for l in lines if l.startswith('#')]
    data   = [l for l in lines if not l.startswith('#')]

    kept = data[::2]  # keep every other frame (0, 2, 4, ...)

    # Backup original
    bak_path = csv_path + '.bak'
    if not os.path.exists(bak_path):
        shutil.copy2(csv_path, bak_path)
        print(f"  backup -> {bak_path}")
    else:
        print(f"  backup already exists, overwriting csv only")

    with open(csv_path, 'w') as f:
        f.writelines(header)
        f.writelines(kept)

    print(f"  {len(data)} -> {len(kept)} frames  ({csv_path})")

def restore_csv(csv_path):
    bak_path = csv_path + '.bak'
    if os.path.exists(bak_path):
        shutil.copy2(bak_path, csv_path)
        print(f"  restored: {csv_path}")
    else:
        print(f"  no backup found: {bak_path}")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    mav0_path = sys.argv[1].rstrip('/')
    restore_mode = '--restore' in sys.argv

    csv_files = sorted(glob.glob(os.path.join(mav0_path, 'cam*/data.csv')))
    if not csv_files:
        print(f"No cam*/data.csv found under {mav0_path}")
        sys.exit(1)

    for csv_path in csv_files:
        if restore_mode:
            restore_csv(csv_path)
        else:
            halve_csv(csv_path)

    if restore_mode:
        print("Restored original frame rates.")
    else:
        print("Done. To restore originals: python3 halve_framerate.py <path> --restore")
