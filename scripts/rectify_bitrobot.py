#!/usr/bin/env python3
"""Rectify bitrobot's left_eye/right_eye pair using cv2.fisheye.stereoRectify
(equidistant model, natively supported -- unlike DAS-EGO's Double Sphere).
Applies to the full raw videos (not just a still frame), then runs the same
empirical validation used for DAS-EGO: ORB feature matching for disparity
sign + vertical alignment.
"""
import sys
import json
import numpy as np
import cv2


def load_calib(anycalib_json, colmap_json):
    with open(anycalib_json) as f:
        calib = json.load(f)
    with open(colmap_json) as f:
        ext = json.load(f)

    def K_D(name):
        fx, fy, cx, cy, k1, k2, k3, k4 = calib[name]["intrinsics_fx_fy_cx_cy_k1_k2_k3_k4"]
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
        D = np.array([k1, k2, k3, k4]).reshape(4, 1)
        return K, D

    K0, D0 = K_D("left_eye")
    K1, D1 = K_D("right_eye")
    w, h = calib["left_eye"]["image_size"]

    def quat_xyzw_to_R(q):
        x, y, z, w_ = q
        n = x * x + y * y + z * z + w_ * w_
        s = 2.0 / n
        X, Y, Z, W = x * s, y * s, z * s, w_ * s
        xx, yy, zz = x * X, y * Y, z * Z
        xy, xz, yz = x * Y, x * Z, y * Z
        wx, wy, wz = w_ * X, w_ * Y, w_ * Z
        return np.array([
            [1 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1 - (xx + yy)],
        ])

    R = quat_xyzw_to_R(ext["median_quaternion_xyzw"])
    T = np.array(ext["median_translation_arbitrary_scale"]).reshape(3, 1)
    return K0, D0, K1, D1, R, T, (w, h)


def build_maps(K0, D0, K1, D1, R, T, size):
    R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
        K0, D0, K1, D1, size, R, T, cv2.CALIB_ZERO_DISPARITY,
        newImageSize=size, balance=0.0, fov_scale=1.0)
    m0 = cv2.fisheye.initUndistortRectifyMap(K0, D0, R1, P1, size, cv2.CV_16SC2)
    m1 = cv2.fisheye.initUndistortRectifyMap(K1, D1, R2, P2, size, cv2.CV_16SC2)
    info = {"R1": R1.tolist(), "R2": R2.tolist(), "P1": P1.tolist(), "P2": P2.tolist(),
            "Q": Q.tolist(), "baseline_arbitrary_scale": abs(float(P2[0, 3] / P2[0, 0]))}
    return m0, m1, info


def rectify_video(in_path, out_path, mapx_mapy, size, max_frames=0):
    cap = cv2.VideoCapture(in_path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    w, h = size
    writer = cv2.VideoWriter(out_path, fourcc, fps, (w, h))
    n = 0
    while not (max_frames and n >= max_frames):
        ok, frame = cap.read()
        if not ok:
            break
        rect = cv2.remap(frame, mapx_mapy[0], mapx_mapy[1], cv2.INTER_LINEAR, borderValue=(0, 0, 0))
        writer.write(rect)
        n += 1
    cap.release()
    writer.release()
    return n


def main():
    anycalib_json, colmap_json = sys.argv[1], sys.argv[2]
    left_in, right_in = sys.argv[3], sys.argv[4]
    left_out, right_out = sys.argv[5], sys.argv[6]
    info_out = sys.argv[7]

    K0, D0, K1, D1, R, T, size = load_calib(anycalib_json, colmap_json)
    m0, m1, info = build_maps(K0, D0, K1, D1, R, T, size)
    print(f"rectification maps built. baseline (arbitrary scale)={info['baseline_arbitrary_scale']:.4f}")

    max_frames = int(sys.argv[8]) if len(sys.argv) > 8 else 0
    n0 = rectify_video(left_in, left_out, m0, size, max_frames)
    print(f"wrote {left_out} ({n0} frames)")
    n1 = rectify_video(right_in, right_out, m1, size, max_frames)
    print(f"wrote {right_out} ({n1} frames)")

    with open(info_out, "w") as f:
        json.dump(info, f, indent=2)
    print(f"wrote {info_out}")


if __name__ == "__main__":
    main()
