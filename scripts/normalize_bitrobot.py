#!/usr/bin/env python3
"""bitrobot normalizer -- produces the Standard Package v2 format (see
01-normalized-layer-spec.md) from one raw bitrobot chunk directory.

Input:  a chunk directory, e.g.
        raw/bitrobot/<site>/<date>/<unit>/<session>/<chunk>/
        containing calibration.json (the eye-pair Kalibr calibration,
        trigger file), imu_*.db / mag_middle_*.db, and per-camera .mp4s.
Output: left_raw.mp4, right_raw.mp4, sbs_raw.mp4, left_rectified.mp4,
        right_rectified.mp4, sbs_rectified.mp4, calibration.json, meta.json,
        imu.csv, frame_timestamps.csv, status.json.

Usage:
    python normalize_bitrobot.py <chunk_dir> <output_dir>
"""
import sys
import os
import re
import json
import glob
import shutil
import sqlite3
import subprocess
import time
import math
import resource
import traceback
from datetime import datetime, timezone

import numpy as np
import cv2

G = 9.80665


def find_camera_file(chunk_dir, recorder_camera):
    """Resolve which raw video belongs to a calibrated camera. Matched by
    exact prefix + numeric chunk suffix (e.g. "left_000.mp4") -- NOT a loose
    glob, since "left_far_000.mp4"/"left_front_000.mp4" also start with
    "left_" and would falsely match a naive left_*.mp4 pattern."""
    pattern = re.compile(rf"^{re.escape(recorder_camera)}_\d+\.mp4$")
    matches = sorted(f for f in os.listdir(chunk_dir) if pattern.match(f))
    if not matches:
        raise FileNotFoundError(f"no file matching '{recorder_camera}_<N>.mp4' in {chunk_dir}")
    if len(matches) > 1:
        raise RuntimeError(f"multiple files match '{recorder_camera}_<N>.mp4': {matches} "
                            f"-- multi-part chunk not supported")
    return os.path.join(chunk_dir, matches[0])


def find_one(chunk_dir, suffix):
    matches = sorted(glob.glob(os.path.join(chunk_dir, f"*{suffix}")))
    if not matches:
        raise FileNotFoundError(f"no file matching *{suffix} in {chunk_dir}")
    return matches[0]


def fov_fisheye(K, D, w, h):
    """FOV for an equidistant/Kannala-Brandt fisheye camera. The pinhole
    formula (2*atan(w/2fx)) is wrong for this lens model -- unproject the
    real edge/corner pixels through the fisheye model and read the ray
    angle instead."""
    cx, cy = float(K[0, 2]), float(K[1, 2])
    pts = np.array([
        [w - 1.0, cy],
        [cx, h - 1.0],
        [w - 1.0, h - 1.0],
    ], dtype=np.float64).reshape(-1, 1, 2)
    norm = cv2.fisheye.undistortPoints(pts, K.astype(np.float64), D.reshape(4, 1).astype(np.float64)).reshape(-1, 2)
    h_fov = 2 * np.degrees(np.arctan(abs(norm[0, 0])))
    v_fov = 2 * np.degrees(np.arctan(abs(norm[1, 1])))
    d_fov = 2 * np.degrees(np.arctan(np.hypot(norm[2, 0], norm[2, 1])))
    return float(h_fov), float(v_fov), float(d_fov)


def fov_pinhole(fx, fy, w, h):
    h_fov = 2 * np.degrees(np.arctan(w / (2 * fx)))
    v_fov = 2 * np.degrees(np.arctan(h / (2 * fy)))
    d_fov = 2 * np.degrees(np.arctan(np.hypot(w / 2, h / 2) / ((fx + fy) / 2)))
    return float(h_fov), float(v_fov), float(d_fov)


def load_calibration(chunk_dir):
    with open(os.path.join(chunk_dir, "calibration.json")) as f:
        cal = json.load(f)
    cam0, cam1 = cal["cameras"]["cam0"], cal["cameras"]["cam1"]
    # cam0/cam1 must be left/right in that order -- verify, don't assume.
    if cam0["recorder_camera"] != "left" or cam1["recorder_camera"] != "right":
        raise RuntimeError(f"unexpected camera order: cam0={cam0['recorder_camera']!r} "
                            f"cam1={cam1['recorder_camera']!r} (expected left/right)")
    return cal, cam0, cam1


def build_rectify_maps(cam0, cam1, T_cam1_cam0):
    w, h = cam0["resolution_px"]
    assert [w, h] == cam1["resolution_px"], "cam0/cam1 resolution mismatch"
    K1 = np.array([[cam0["intrinsics_px"][0], 0, cam0["intrinsics_px"][2]],
                   [0, cam0["intrinsics_px"][1], cam0["intrinsics_px"][3]],
                   [0, 0, 1]], dtype=np.float64)
    K2 = np.array([[cam1["intrinsics_px"][0], 0, cam1["intrinsics_px"][2]],
                   [0, cam1["intrinsics_px"][1], cam1["intrinsics_px"][3]],
                   [0, 0, 1]], dtype=np.float64)
    D1 = np.array(cam0["distortion_coefficients"], dtype=np.float64).reshape(4, 1)
    D2 = np.array(cam1["distortion_coefficients"], dtype=np.float64).reshape(4, 1)
    R = np.ascontiguousarray(np.array(T_cam1_cam0)[:3, :3])
    T = np.ascontiguousarray(np.array(T_cam1_cam0)[:3, 3]).reshape(3, 1)

    R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
        K1=K1, D1=D1, K2=K2, D2=D2, imageSize=(w, h), R=R, tvec=T,
        flags=cv2.CALIB_ZERO_DISPARITY, balance=0.0, fov_scale=1.0)
    map1_l, map2_l = cv2.fisheye.initUndistortRectifyMap(K1, D1, R1, P1, (w, h), cv2.CV_32FC1)
    map1_r, map2_r = cv2.fisheye.initUndistortRectifyMap(K2, D2, R2, P2, (w, h), cv2.CV_32FC1)
    return {"K1": K1, "D1": D1, "K2": K2, "D2": D2, "w": w, "h": h,
            "map1_l": map1_l, "map2_l": map2_l, "map1_r": map1_r, "map2_r": map2_r,
            "P1": P1, "P2": P2}


def remux_copy(src, dst):
    subprocess.run(["ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
                    "-i", src, "-c:v", "copy", "-an", dst], check=True)


def rectify_and_encode(left_src, right_src, maps, fps, left_dst, right_dst):
    cap_l = cv2.VideoCapture(left_src)
    cap_r = cv2.VideoCapture(right_src)
    w, h = maps["w"], maps["h"]
    proc_l = subprocess.Popen(
        ["ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
         "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{w}x{h}", "-r", str(fps), "-i", "-",
         "-c:v", "libx264", "-crf", "18", "-preset", "medium", "-pix_fmt", "yuv420p", left_dst],
        stdin=subprocess.PIPE)
    proc_r = subprocess.Popen(
        ["ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
         "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{w}x{h}", "-r", str(fps), "-i", "-",
         "-c:v", "libx264", "-crf", "18", "-preset", "medium", "-pix_fmt", "yuv420p", right_dst],
        stdin=subprocess.PIPE)
    n = 0
    while True:
        okl, fl = cap_l.read()
        okr, fr = cap_r.read()
        if not okl or not okr:
            break
        rl = cv2.remap(fl, maps["map1_l"], maps["map2_l"], cv2.INTER_LINEAR, borderValue=(0, 0, 0))
        rr = cv2.remap(fr, maps["map1_r"], maps["map2_r"], cv2.INTER_LINEAR, borderValue=(0, 0, 0))
        proc_l.stdin.write(rl.tobytes())
        proc_r.stdin.write(rr.tobytes())
        n += 1
    cap_l.release(); cap_r.release()
    proc_l.stdin.close(); proc_r.stdin.close()
    proc_l.wait(); proc_r.wait()
    if proc_l.returncode != 0 or proc_r.returncode != 0:
        raise RuntimeError("ffmpeg rectified-video encode failed")
    return n


def hstack(a, b, out):
    subprocess.run(["ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
                     "-i", a, "-i", b, "-filter_complex", "hstack=inputs=2",
                     "-c:v", "libx264", "-crf", "18", "-preset", "medium", "-pix_fmt", "yuv420p", out],
                    check=True)


def read_imu_db(db_path):
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    cur.execute("SELECT timestamp, x, y, z FROM acc_data ORDER BY timestamp")
    acc = cur.fetchall()
    cur.execute("SELECT timestamp, x, y, z FROM gyro_data ORDER BY timestamp")
    gyro = cur.fetchall()
    conn.close()
    return acc, gyro


def read_db_metadata(db_path):
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    cur.execute("SELECT key, value FROM metadata")
    meta = dict(cur.fetchall())
    conn.close()
    return meta


def main(chunk_dir, output_dir):
    t_start = time.time()
    chunk_dir = os.path.abspath(chunk_dir)
    run_id = "-".join(chunk_dir.strip("/").split("/")[-3:])  # <session>-<chunk>-ish, no batch orchestrator here
    os.makedirs(output_dir, exist_ok=True)

    try:
        print("=== 1. loading calibration ===")
        cal, cam0, cam1 = load_calibration(chunk_dir)
        print(f"  device_id={cal['device_id']} camera_pair={cal.get('_provenance', {}).get('camera_pair')}")

        print("\n=== 2. resolving raw video files (from calibration's recorder_camera, not hardcoded) ===")
        left_src = find_camera_file(chunk_dir, cam0["recorder_camera"])
        right_src = find_camera_file(chunk_dir, cam1["recorder_camera"])
        print(f"  left  ({cam0['recorder_camera']}): {os.path.basename(left_src)}")
        print(f"  right ({cam1['recorder_camera']}): {os.path.basename(right_src)}")

        print("\n=== 3. building rectification maps (cv2.fisheye, equidistant model) ===")
        T_cam1_cam0 = cal["extrinsics"]["T_cam1_cam0"]
        maps = build_rectify_maps(cam0, cam1, T_cam1_cam0)
        baseline_m = float(cal["extrinsics"]["stereo_baseline_m"])
        print(f"  baseline={baseline_m*1000:.2f}mm resolution={maps['w']}x{maps['h']}")

        print("\n=== 4. raw videos (stream-copy, source already HEVC) ===")
        left_raw = os.path.join(output_dir, "left_raw.mp4")
        right_raw = os.path.join(output_dir, "right_raw.mp4")
        remux_copy(left_src, left_raw)
        remux_copy(right_src, right_raw)

        probe = cv2.VideoCapture(left_raw)
        fps = probe.get(cv2.CAP_PROP_FPS) or 30.0
        probe.release()
        print(f"  wrote {left_raw}, {right_raw} (fps={fps:.3f})")

        print("\n=== 5. rectified videos (cv2.remap + libx264) ===")
        left_rect = os.path.join(output_dir, "left_rectified.mp4")
        right_rect = os.path.join(output_dir, "right_rectified.mp4")
        n_frames = rectify_and_encode(left_raw, right_raw, maps, fps, left_rect, right_rect)
        print(f"  rectified {n_frames} frame pairs")

        print("\n=== 6. side-by-side videos ===")
        sbs_raw = os.path.join(output_dir, "sbs_raw.mp4")
        sbs_rect = os.path.join(output_dir, "sbs_rectified.mp4")
        hstack(left_raw, right_raw, sbs_raw)
        hstack(left_rect, right_rect, sbs_rect)
        print(f"  wrote {sbs_raw}, {sbs_rect}")

        print("\n=== 7. IMU (imu_right_000-style file -- the one calibration.json's imu block describes) ===")
        imu_db = find_one(chunk_dir, "imu_right_*.db")
        mag_db_check = find_one(chunk_dir, "mag_middle_*.db")  # exists, not consumed (schema has no mag slot)
        acc, gyro = read_imu_db(imu_db)
        scales = cal["_provenance"]["raw_sensor_scales"]
        accel_lsb_per_g = scales["accel_lsb_per_g"]
        gyro_lsb_per_dps = scales["gyro_lsb_per_dps"]
        n_imu = min(len(acc), len(gyro))
        t0 = acc[0][0]
        imu_csv_path = os.path.join(output_dir, "imu.csv")
        with open(imu_csv_path, "w") as f:
            f.write("timestamp_s,ax,ay,az,gx,gy,gz\n")
            for i in range(n_imu):
                ta, ax, ay, az = acc[i]
                tg, gx, gy, gz = gyro[i]
                ts = (ta - t0) / 1e9
                f.write(f"{ts:.6f},"
                        f"{ax*G/accel_lsb_per_g:.6f},{ay*G/accel_lsb_per_g:.6f},{az*G/accel_lsb_per_g:.6f},"
                        f"{math.radians(gx/gyro_lsb_per_dps):.6f},{math.radians(gy/gyro_lsb_per_dps):.6f},"
                        f"{math.radians(gz/gyro_lsb_per_dps):.6f}\n")
        print(f"  wrote {n_imu} rows to {imu_csv_path} (from {os.path.basename(imu_db)})")

        print("\n=== 8. frame_timestamps.csv (nominal rate -- no per-frame timing source exists for bitrobot) ===")
        ft_path = os.path.join(output_dir, "frame_timestamps.csv")
        with open(ft_path, "w") as f:
            f.write("frame_index,timestamp_s\n")
            for i in range(n_frames):
                f.write(f"{i},{i/fps:.6f}\n")
        print(f"  wrote {n_frames} rows to {ft_path}")

        print("\n=== 9. calibration.json ===")
        h_fov0, v_fov0, d_fov0 = fov_fisheye(maps["K1"], maps["D1"], maps["w"], maps["h"])
        h_fov1, v_fov1, d_fov1 = fov_fisheye(maps["K2"], maps["D2"], maps["w"], maps["h"])
        P1, P2 = maps["P1"], maps["P2"]
        fx_r = float(P2[0, 0])
        rect_baseline_m = abs(float(P2[0, 3]) / fx_r) if fx_r != 0 else 0.0

        def raw_camera(cam, K, h_fov, v_fov, d_fov):
            return {
                "fx": K[0, 0], "fy": K[1, 1], "cx": K[0, 2], "cy": K[1, 2],
                "h_fov_degrees": round(h_fov, 2), "v_fov_degrees": round(v_fov, 2),
                "d_fov_degrees": round(d_fov, 2),
                "distortion": cam["distortion_coefficients"],
                "distortion_model": "equidistant",
            }

        def rect_camera(P):
            fx, fy, cx, cy = float(P[0, 0]), float(P[1, 1]), float(P[0, 2]), float(P[1, 2])
            h_fov, v_fov, d_fov = fov_pinhole(fx, fy, maps["w"], maps["h"])
            return {"fx": fx, "fy": fy, "cx": cx, "cy": cy,
                    "h_fov_degrees": round(h_fov, 2), "v_fov_degrees": round(v_fov, 2),
                    "d_fov_degrees": round(d_fov, 2),
                    "distortion": [0.0, 0.0, 0.0, 0.0], "distortion_model": "radtan"}

        calibration_out = {
            "schema_version": 2,
            "source": "bitrobot_calibration",
            "raw": {
                "left": raw_camera(cam0, maps["K1"], h_fov0, v_fov0, d_fov0),
                "right": raw_camera(cam1, maps["K2"], h_fov1, v_fov1, d_fov1),
                "stereo": {"baseline_mm": round(baseline_m * 1000, 2), "baseline_meters": round(baseline_m, 6)},
                "rotation": np.array(T_cam1_cam0)[:3, :3].tolist(),
                "translation": np.array(T_cam1_cam0)[:3, 3].tolist(),
            },
            "rectified": {
                "left": rect_camera(P1),
                "right": rect_camera(P2),
                "stereo": {"baseline_mm": round(rect_baseline_m * 1000, 2), "baseline_meters": round(rect_baseline_m, 6)},
                "rotation": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                "translation": [-rect_baseline_m, 0.0, 0.0],
            },
            "imu": {
                "T_cam0_imu": cal["extrinsics"]["T_cam0_imu"],
                "T_cam1_imu": cal["extrinsics"]["T_cam1_imu"],
                "accelerometer_noise_density": cal["imu"]["accelerometer_noise_density"],
                "accelerometer_random_walk": cal["imu"]["accelerometer_random_walk"],
                "gyroscope_noise_density": cal["imu"]["gyroscope_noise_density"],
                "gyroscope_random_walk": cal["imu"]["gyroscope_random_walk"],
                "cam_imu_time_offset_s": cal["temporal"]["cam_imu_time_offset_s"],
            },
        }
        with open(os.path.join(output_dir, "calibration.json"), "w") as f:
            json.dump(calibration_out, f, indent=2)
        print(f"  wrote calibration.json")

        print("\n=== 10. meta.json ===")
        db_meta = read_db_metadata(imu_db)
        total_bytes = os.path.getsize(left_src) + os.path.getsize(right_src)
        proc_duration_s = time.time() - t_start
        meta_out = {
            "schema_version": 2,
            "device": {
                "device_type_from_folder_path": "bitrobot",
                "model_from_device_metadata": f"{db_meta.get('product', 'robocap')} ({db_meta.get('camera', 'unknown')})",
                "device_id": cal.get("device_id"),
                "firmware_version": db_meta.get("version"),
            },
            "recording": {
                "width": maps["w"], "height": maps["h"], "fps": fps,
                "duration_seconds": n_frames / fps if fps else None,
                "frame_count": n_frames,
                "is_stereo": True,
                "codec": "hevc",
            },
            "source": {
                "original_format": "mp4",
                "original_files": [os.path.basename(left_src), os.path.basename(right_src)],
                "total_bytes": total_bytes,
                "recorded_utc": None,
                "normalized_utc": datetime.now(timezone.utc).isoformat(),
            },
            "encoding": {"codec": "libx264", "crf": 18, "preset": "medium", "lossless": False},
            "processing_metrics": {
                "duration_seconds": round(proc_duration_s, 2),
                "compute_type": "cpu",
                "memory_used_mb": round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024, 1),
            },
        }
        with open(os.path.join(output_dir, "meta.json"), "w") as f:
            json.dump(meta_out, f, indent=2)
        print(f"  wrote meta.json")

        write_status(output_dir, run_id, "ok", None)
        print(f"\n=== done in {proc_duration_s:.1f}s -> {output_dir} ===")
        return output_dir

    except Exception as exc:
        traceback.print_exc()
        write_status(output_dir, run_id, "failed", f"{type(exc).__name__}: {exc}")
        raise


def write_status(output_dir, run_id, status, error):
    with open(os.path.join(output_dir, "status.json"), "w") as f:
        json.dump({
            "stage": "normalization",
            "run_id": run_id,
            "status": status,
            "error": error,
            "generated_at": datetime.now(timezone.utc).isoformat(),
        }, f, indent=2)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("usage: python normalize_bitrobot.py <chunk_dir> <output_dir>")
        sys.exit(1)
    main(sys.argv[1], sys.argv[2])
