#!/usr/bin/env python3
"""Assemble bitrobot's self-calibrated intrinsics (AnyCalib) + extrinsics
(COLMAP) into a Kalibr-style camchain-imucam.yaml -- the same format
robocap-slam/simplecv expects, and the same format the org already uses for
akai/oak-d. Also writes a calibration_provenance.json spelling out exactly
how this was derived and its accuracy caveats, since this is NOT a vendor/
checkerboard calibration.
"""
import sys
import json
from datetime import datetime, timezone

import numpy as np


def quat_xyzw_to_R(q):
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n
    X, Y, Z, W = x * s, y * s, z * s, w * s
    xx, yy, zz = x * X, y * Y, z * Z
    xy, xz, yz = x * Y, x * Z, y * Z
    wx, wy, wz = w * X, w * Y, w * Z
    return np.array([
        [1 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1 - (xx + yy)],
    ])


def cam_block(name, calib):
    c = calib[name]
    fx, fy, cx, cy, k1, k2, k3, k4 = c["intrinsics_fx_fy_cx_cy_k1_k2_k3_k4"]
    w, h = c["image_size"]
    return {
        "camera_model": "pinhole",
        "intrinsics": [round(fx, 4), round(fy, 4), round(cx, 4), round(cy, 4)],
        "distortion_model": "equidistant",
        "distortion_coeffs": [round(k1, 6), round(k2, 6), round(k3, 6), round(k4, 6)],
        "resolution": [w, h],
    }


def main(anycalib_json, colmap_json, out_yaml, out_provenance):
    with open(anycalib_json) as f:
        calib = json.load(f)
    with open(colmap_json) as f:
        ext = json.load(f)

    R = quat_xyzw_to_R(ext["median_quaternion_xyzw"])
    t_arbitrary = np.array(ext["median_translation_arbitrary_scale"])
    T_cn_cnm1 = np.eye(4)
    T_cn_cnm1[:3, :3] = R
    T_cn_cnm1[:3, 3] = t_arbitrary  # NOTE: arbitrary SfM scale, not meters -- see provenance

    lines = []
    lines.append("%YAML:1.0")
    lines.append("# Self-calibrated bitrobot (RoboCap) camchain-imucam.")
    lines.append("# WARNING: extrinsic translation scale is ARBITRARY (SfM scale-ambiguous),")
    lines.append("# NOT metric meters. See calibration_provenance.json before using for depth.")
    lines.append("cam0:")
    b0 = cam_block("left_eye", calib)
    for k, v in b0.items():
        lines.append(f"  {k}: {v}")
    lines.append("  rostopic: /bitrobot/left_eye")
    lines.append("cam1:")
    b1 = cam_block("right_eye", calib)
    for k, v in b1.items():
        lines.append(f"  {k}: {v}")
    lines.append("  rostopic: /bitrobot/right_eye")
    lines.append("  T_cn_cnm1:")
    for row in T_cn_cnm1.tolist():
        lines.append(f"    - [{row[0]:.8f}, {row[1]:.8f}, {row[2]:.8f}, {row[3]:.8f}]")
    lines.append("")
    lines.append("# Non-stereo cameras (no matching overlapping pair confirmed) -- intrinsics")
    lines.append("# only, informational, not used for rectification:")
    for name in ["left", "right", "left_front", "right_front"]:
        lines.append(f"{name}:")
        for k, v in cam_block(name, calib).items():
            lines.append(f"  {k}: {v}")

    with open(out_yaml, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"wrote {out_yaml}")

    provenance = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "device": "bitrobot (RoboCap, FrodoBots)",
        "device_id": "46bc75e615605ccd",
        "source_footage": "raw/bitrobot/tester-bitrobot/2026-08-13/worker_42/46bc75e615605ccd/20260813_111149_session2/segment6 (smallest segment, ~308s)",
        "method": "self-calibration -- NOT a vendor/factory or Kalibr-checkerboard calibration",
        "intrinsics": {
            "tool": "AnyCalib (ICCV 2025, github.com/javrtg/AnyCalib)",
            "model_id": "anycalib_gen",
            "camera_model_requested": "kb:4 (Kannala-Brandt / equidistant, 4 coefficients)",
            "input": "1 representative frame per camera, extracted at t=5s",
            "known_quality_flag": "AnyCalib's own optimizer reported 'Worse cost after "
                                   "optimization' for the 'right' camera specifically -- "
                                   "treat its intrinsics as lower-confidence. left_eye/"
                                   "right_eye (the cameras actually used for rectification) "
                                   "had no such warning.",
        },
        "extrinsics": {
            "tool": "COLMAP via pycolmap (SfM / incremental bundle adjustment)",
            "method_detail": "Pooled 38 left_eye + 38 right_eye frames (synchronized, "
                              "sampled every 8s across segment6) into one incremental "
                              "reconstruction seeded with AnyCalib intrinsics. For every "
                              "synchronized frame pair both registered, computed the "
                              "relative pose directly (not via pycolmap's Rig API). "
                              "Dropped 6/36 pairs whose baseline deviated >50% from the "
                              "median (traced to weakly-constrained late-registered frames "
                              "with low matched-point ratios, not genuine baseline "
                              "variation -- this is a fixed rig). Took the median rotation "
                              "and translation direction across the remaining 30 pairs.",
            "reconstruction_quality": "73/76 frames registered, 11620 3D points",
            "baseline_relative_spread_unfiltered": ext["baseline_relative_spread_unfiltered"],
            "baseline_relative_spread_after_filtering": ext["baseline_relative_spread_filtered"],
            "n_pairs_used": ext["n_pairs_used_after_robust_filter"],
            "CRITICAL_CAVEAT_SCALE": (
                "SfM reconstructions are scale-ambiguous by construction -- rotation and "
                "translation DIRECTION are meaningful, but the numeric translation "
                "magnitude in this file is in COLMAP's own arbitrary reconstruction "
                "scale, NOT metric meters. This does not affect stereo rectification "
                "(row-alignment only depends on rotation + translation direction, "
                "verified empirically -- see rectification validation), but any "
                "downstream disparity-to-real-world-depth conversion using this file's "
                "T_cn_cnm1 translation will be wrong until independently re-scaled "
                "against a known real-world reference (e.g. a real Kalibr calibration, "
                "or a measured physical baseline)."
            ),
        },
        "accuracy_expectation": (
            "This is an interim, self-calibrated result explicitly chosen because no "
            "vendor/factory or checkerboard calibration exists for this bitrobot unit. "
            "Expect meaningfully lower precision than a dedicated Kalibr/AprilGrid "
            "capture (industry precedent: ~0.1-0.2px reprojection error for Kalibr vs. "
            "multi-pixel error common in natural-scene SfM self-calibration). Replace "
            "with a real Kalibr capture if/when the team sources one."
        ),
        "tool_versions": {
            "anycalib": "1.0 (installed from github.com/javrtg/AnyCalib, editable install)",
            "pycolmap": "4.1.1",
            "torch": "2.13.0+cpu",
        },
    }
    with open(out_provenance, "w") as f:
        json.dump(provenance, f, indent=2)
    print(f"wrote {out_provenance}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
