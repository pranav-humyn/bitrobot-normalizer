#!/usr/bin/env python3
"""Self-calibrate the left_eye/right_eye stereo extrinsic (baseline + rotation)
from ordinary pilot footage via COLMAP SfM/bundle adjustment, seeded with
AnyCalib's per-camera intrinsics.

Deliberately NOT using pycolmap's newer rig API (Rig/Frame data model) --
that API is less familiar and harder to verify correctness of in one pass.
Instead: pool all left_eye+right_eye frames into one ordinary incremental
reconstruction (letting real visual overlap between the two cameras tie them
into one coordinate frame), then, for every synchronized (left_eye_i,
right_eye_i) frame pair that both got registered, compute the relative pose
directly and look at the spread across pairs -- consistent agreement is the
real evidence of a good result, not a single trusted number.
"""
import sys
import json
import shutil
from pathlib import Path

import numpy as np
import pycolmap


def main(images_dir, anycalib_json, work_dir):
    images_dir = Path(images_dir)
    work_dir = Path(work_dir)
    if work_dir.exists():
        shutil.rmtree(work_dir)
    work_dir.mkdir(parents=True)
    db_path = work_dir / "database.db"

    with open(anycalib_json) as f:
        calib = json.load(f)

    def cam_params(name):
        c = calib[name]["intrinsics_fx_fy_cx_cy_k1_k2_k3_k4"]
        return c  # [fx, fy, cx, cy, k1, k2, k3, k4] -- matches COLMAP's OPENCV_FISHEYE order

    db = pycolmap.Database.open(str(db_path))

    def make_camera(camera_id, name):
        p = cam_params(name)
        w, h = calib[name]["image_size"]
        cam = pycolmap.Camera.create_from_model_id(
            camera_id=camera_id, model=pycolmap.CameraModelId.OPENCV_FISHEYE,
            focal_length=(p[0] + p[1]) / 2, width=w, height=h,
        )
        cam.params = p
        return cam

    cam_left = make_camera(1, "left_eye")
    cam_right = make_camera(2, "right_eye")
    db.write_camera(cam_left, use_camera_id=True)
    db.write_camera(cam_right, use_camera_id=True)
    db.close()

    left_images = sorted(images_dir.glob("left_eye_*.jpg"))
    right_images = sorted(images_dir.glob("right_eye_*.jpg"))
    print(f"{len(left_images)} left_eye frames, {len(right_images)} right_eye frames")

    pycolmap.extract_features(
        database_path=str(db_path), image_path=str(images_dir),
        image_names=[p.name for p in left_images],
        camera_mode=pycolmap.CameraMode.SINGLE,
        reader_options=pycolmap.ImageReaderOptions(existing_camera_id=1),
    )
    pycolmap.extract_features(
        database_path=str(db_path), image_path=str(images_dir),
        image_names=[p.name for p in right_images],
        camera_mode=pycolmap.CameraMode.SINGLE,
        reader_options=pycolmap.ImageReaderOptions(existing_camera_id=2),
    )
    print("feature extraction done")

    pycolmap.match_exhaustive(database_path=str(db_path))
    print("matching done")

    maps = pycolmap.incremental_mapping(
        database_path=str(db_path), image_path=str(images_dir),
        output_path=str(work_dir / "sparse"),
    )
    if not maps:
        print("RECONSTRUCTION FAILED -- no model produced. Likely insufficient "
              "motion/parallax in this footage segment for self-calibration.")
        return

    # use the largest reconstructed model
    rec = max(maps.values(), key=lambda r: r.num_reg_images())
    print(f"reconstruction: {rec.num_reg_images()} images registered, "
          f"{rec.num_points3D()} 3D points")

    # collect per-frame-index pose for each camera. Use everything after the
    # "left_eye_"/"right_eye_" prefix (e.g. "s1_000") as the pairing key --
    # NOT just the last underscore token, which would collide across segments
    # (segment1's sample 000 and segment2's sample 000 are NOT the same
    # synchronized instant).
    def frame_idx(name, prefix):
        return name[len(prefix):].rsplit(".", 1)[0]

    left_poses, right_poses = {}, {}
    for image_id, image in rec.images.items():
        name = image.name
        cam_from_world = image.cam_from_world()  # Rigid3d: world -> camera
        if name.startswith("left_eye_"):
            left_poses[frame_idx(name, "left_eye_")] = cam_from_world
        elif name.startswith("right_eye_"):
            right_poses[frame_idx(name, "right_eye_")] = cam_from_world

    print(f"registered: {len(left_poses)} left_eye, {len(right_poses)} right_eye")

    rel_rotations, rel_translations, used_idx = [], [], []
    for idx in sorted(set(left_poses) & set(right_poses)):
        L = left_poses[idx]  # world -> left_eye_cam
        R = right_poses[idx]  # world -> right_eye_cam
        # relative: right_eye_cam <- left_eye_cam = R_world_to_right * inv(R_world_to_left)
        rel = R * L.inverse()
        rel_rotations.append(rel.rotation.quat)  # (x,y,z,w)
        rel_translations.append(rel.translation)
        used_idx.append(idx)
        print(f"  pair {idx}: t={rel.translation}")

    if not rel_rotations:
        print("No synchronized pairs both got registered -- cannot derive extrinsics "
              "from this reconstruction.")
        return

    rel_translations = np.array(rel_translations)
    rel_rotations = np.array(rel_rotations)
    baseline_norms = np.linalg.norm(rel_translations, axis=1)
    rel_std = baseline_norms.std() / baseline_norms.mean() if baseline_norms.mean() > 0 else float("nan")
    print(f"\n=== relative pose across {len(rel_translations)} synchronized pairs (unfiltered) ===")
    print(f"baseline (arbitrary SfM scale): mean={baseline_norms.mean():.4f} "
          f"median={np.median(baseline_norms):.4f} std={baseline_norms.std():.4f} "
          f"(relative spread {rel_std:.1%}) min={baseline_norms.min():.4f} max={baseline_norms.max():.4f}")

    # robust filter: drop pairs whose baseline deviates >50% from the median
    # (fixed rig -> real variation here means noisy pose estimates, not a
    # genuinely different baseline)
    med_baseline = np.median(baseline_norms)
    keep = np.abs(baseline_norms - med_baseline) / med_baseline < 0.5
    n_dropped = int((~keep).sum())
    print(f"\nrobust filter: dropping {n_dropped}/{len(keep)} pairs >50% off the median, "
          f"keeping {int(keep.sum())}")

    ft, fr = rel_translations[keep], rel_rotations[keep]
    fb = np.linalg.norm(ft, axis=1)
    fb_spread = fb.std() / fb.mean() if fb.mean() > 0 else float("nan")
    print(f"filtered baseline (arbitrary SfM scale): mean={fb.mean():.4f} median={np.median(fb):.4f} "
          f"std={fb.std():.4f} (relative spread {fb_spread:.1%})")
    print("(NOTE: SfM reconstructions are scale-ambiguous without a known metric "
          "reference -- this baseline is in COLMAP's arbitrary reconstruction scale, "
          "not meters, unless independently scaled. Flagging honestly rather than "
          "presenting a fabricated metric unit.)")

    med_t = np.median(ft, axis=0)
    ref = fr[0]
    fr = np.where((fr @ ref)[:, None] < 0, -fr, fr)
    med_q = np.median(fr, axis=0)
    med_q = med_q / np.linalg.norm(med_q)
    result = {
        "median_translation_arbitrary_scale": med_t.tolist(),
        "median_quaternion_xyzw": med_q.tolist(),
        "baseline_relative_spread_unfiltered": float(rel_std),
        "baseline_relative_spread_filtered": float(fb_spread),
        "n_pairs_unfiltered": len(rel_translations),
        "n_pairs_used_after_robust_filter": int(keep.sum()),
        "n_pairs_dropped_as_outliers": n_dropped,
        "n_left_eye_registered": len(left_poses),
        "n_right_eye_registered": len(right_poses),
        "n_left_eye_total": len(left_images),
        "n_right_eye_total": len(right_images),
    }
    with open(work_dir / "extrinsics_result.json", "w") as f:
        json.dump(result, f, indent=2)
    print(f"\nwrote {work_dir / 'extrinsics_result.json'}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3])
