#!/usr/bin/env python3
"""Extract bitrobot's IMU/magnetometer SQLite .db files to CSV, and
investigate whether their timestamps can be trusted as synced to the video
for this segment (rather than assuming so)."""
import sys
import sqlite3
import csv


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


def main(imu_db, mag_db, out_prefix):
    acc = dump_table(imu_db, "acc_data", ["timestamp", "x", "y", "z"], f"{out_prefix}_acc.csv")
    gyro = dump_table(imu_db, "gyro_data", ["timestamp", "x", "y", "z"], f"{out_prefix}_gyro.csv")
    mag = dump_table(mag_db, "mag_data", ["timestamp", "mag_x", "mag_y", "mag_z"], f"{out_prefix}_mag.csv")

    for name, rows in (("acc", acc), ("gyro", gyro), ("mag", mag)):
        if not rows:
            print(f"{name}: EMPTY")
            continue
        ts = [r[0] for r in rows]
        dur_s = (ts[-1] - ts[0]) / 1e9
        rate = (len(ts) - 1) / dur_s if dur_s > 0 else float("nan")
        print(f"{name}: n={len(ts)} t0={ts[0]} t_last={ts[-1]} duration_s={dur_s:.3f} "
              f"implied_rate_hz={rate:.2f}")

        # gravity-magnitude sanity check for accel specifically
        if name == "acc":
            import math
            mags = [math.sqrt(r[1] ** 2 + r[2] ** 2 + r[3] ** 2) for r in rows]
            print(f"  |a| mean={sum(mags)/len(mags):.4f} min={min(mags):.4f} max={max(mags):.4f}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3])
