#!/usr/bin/env python3
"""Extract bitrobot's raw IMU/magnetometer .db files (imu_left, imu_right,
mag_middle) from one session into plain CSVs. Values are raw ADC counts,
not m/s^2 or rad/s.

Usage:
    python extract_bitrobot_imu.py <session_dir> <output_dir>
"""
import sys
import os
import glob
import csv
import sqlite3
import math


def find_one(session_dir, suffix):
    matches = sorted(glob.glob(os.path.join(session_dir, f"*{suffix}")))
    if not matches:
        raise FileNotFoundError(f"no file matching *{suffix} in {session_dir}")
    if len(matches) > 1:
        print(f"  warning: multiple matches for *{suffix}, using the first: {matches}")
    return matches[0]


def dump_table(db_path, table, cols, out_csv):
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    cur.execute(f"SELECT {','.join(cols)} FROM {table} ORDER BY timestamp")
    rows = cur.fetchall()
    conn.close()
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        w.writerows(rows)
    return rows


def sanity_check(name, rows, is_accel=False):
    if not rows:
        print(f"  {name}: EMPTY")
        return
    ts = [r[0] for r in rows]
    dur_s = (ts[-1] - ts[0]) / 1e9
    rate_hz = (len(ts) - 1) / dur_s if dur_s > 0 else float("nan")
    print(f"  {name}: n={len(ts)} duration_s={dur_s:.2f} implied_rate_hz={rate_hz:.2f}")
    if is_accel:
        mags = [math.sqrt(r[1] ** 2 + r[2] ** 2 + r[3] ** 2) for r in rows]
        mean_mag = sum(mags) / len(mags)
        print(f"    |accel| (raw counts) mean={mean_mag:.1f} min={min(mags):.1f} max={max(mags):.1f}")
        print(f"    /8192 (ICM-42688-P +/-4g sensitivity, 32768/4) = {mean_mag/8192:.3f} "
              f"-- should read ~1.0 if the device is on +/-4g range")


def main(session_dir, output_dir):
    os.makedirs(output_dir, exist_ok=True)

    imu_left_db = find_one(session_dir, "imu_left.db")
    imu_right_db = find_one(session_dir, "imu_right.db")
    mag_db = find_one(session_dir, "mag_middle.db")
    print(f"imu_left:  {imu_left_db}")
    print(f"imu_right: {imu_right_db}")
    print(f"mag:       {mag_db}\n")

    print("=== extracting ===")
    for side, db_path in (("left", imu_left_db), ("right", imu_right_db)):
        acc = dump_table(db_path, "acc_data", ["timestamp", "x", "y", "z"],
                          os.path.join(output_dir, f"{side}_acc.csv"))
        gyro = dump_table(db_path, "gyro_data", ["timestamp", "x", "y", "z"],
                           os.path.join(output_dir, f"{side}_gyro.csv"))
        print(f"\n{side} IMU ({os.path.basename(db_path)}):")
        sanity_check(f"{side}_acc.csv", acc, is_accel=True)
        sanity_check(f"{side}_gyro.csv", gyro)

    mag = dump_table(mag_db, "mag_data", ["timestamp", "mag_x", "mag_y", "mag_z"],
                      os.path.join(output_dir, "mag.csv"))
    print(f"\nmagnetometer ({os.path.basename(mag_db)}) -- one physical sensor, one file:")
    sanity_check("mag.csv", mag)

    print(f"\n=== wrote 5 files to {output_dir} ===")
    print("(left_acc.csv, left_gyro.csv, right_acc.csv, right_gyro.csv, mag.csv)")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("usage: python extract_bitrobot_imu.py <session_dir> <output_dir>")
        sys.exit(1)
    main(sys.argv[1], sys.argv[2])
