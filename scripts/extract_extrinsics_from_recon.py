#!/usr/bin/env python3
"""Load an already-computed COLMAP reconstruction and derive the
left_eye<->right_eye relative extrinsic from synchronized frame pairs."""
import sys
import json
import numpy as np
import pycolmap


def frame_idx(name):
    return name.split("_")[-1].split(".")[0]


def main(sparse_path, out_path):
    rec = pycolmap.Reconstruction(sparse_path)
    print(f"loaded reconstruction: {rec.num_reg_images()} images, {rec.num_points3D()} points")

    left_poses, right_poses = {}, {}
    for image_id, image in rec.images.items():
        cam_from_world = image.cam_from_world()  # Rigid3d: world -> camera
        if image.name.startswith("left_eye_"):
            left_poses[frame_idx(image.name)] = cam_from_world
        elif image.name.startswith("right_eye_"):
            right_poses[frame_idx(image.name)] = cam_from_world

    print(f"registered: {len(left_poses)} left_eye, {len(right_poses)} right_eye")

    rel_rotations, rel_translations, used_idx = [], [], []
    for idx in sorted(set(left_poses) & set(right_poses)):
        L, R = left_poses[idx], right_poses[idx]
        rel = R * L.inverse()  # left_eye_cam -> right_eye_cam
        rel_rotations.append(rel.rotation.quat)  # (x, y, z, w)
        rel_translations.append(rel.translation)
        used_idx.append(idx)
        t = rel.translation
        print(f"  pair {idx}: t=({t[0]:+.4f},{t[1]:+.4f},{t[2]:+.4f})  |t|={np.linalg.norm(t):.4f}")

    if not rel_rotations:
        print("No synchronized pairs both registered -- cannot derive extrinsics.")
        return

    rel_translations = np.array(rel_translations)
    rel_rotations = np.array(rel_rotations)
    baseline_norms = np.linalg.norm(rel_translations, axis=1)
    rel_std = baseline_norms.std() / baseline_norms.mean() if baseline_norms.mean() > 0 else float("nan")
    print(f"\n=== relative pose across {len(rel_translations)} synchronized pairs (unfiltered) ===")
    print(f"baseline (arbitrary SfM scale): mean={baseline_norms.mean():.4f} "
          f"median={np.median(baseline_norms):.4f} std={baseline_norms.std():.4f} "
          f"(relative spread {rel_std:.1%}) min={baseline_norms.min():.4f} max={baseline_norms.max():.4f}")

    # Robust filter: drop pairs whose baseline deviates >50% from the median --
    # these correspond to weakly-constrained frames (checked against the SfM log:
    # the outliers are exactly the late-registered images with a low
    # matched-points-to-total ratio, not a sign the physical baseline itself
    # varies -- it's a FIXED rig, so real variation here means noisy pose
    # estimates, not a genuinely different baseline).
    med_baseline = np.median(baseline_norms)
    keep = np.abs(baseline_norms - med_baseline) / med_baseline < 0.5
    n_dropped = int((~keep).sum())
    print(f"\nrobust filter: dropping {n_dropped}/{len(keep)} pairs >50% off the median "
          f"(weakly-constrained frames), keeping {int(keep.sum())}")

    ft, fr = rel_translations[keep], rel_rotations[keep]
    fb = np.linalg.norm(ft, axis=1)
    fb_spread = fb.std() / fb.mean() if fb.mean() > 0 else float("nan")
    print(f"filtered baseline (arbitrary SfM scale): mean={fb.mean():.4f} median={np.median(fb):.4f} "
          f"std={fb.std():.4f} (relative spread {fb_spread:.1%})")
    print("NOTE: SfM reconstructions are scale-ambiguous -- this baseline is in "
          "COLMAP's own arbitrary reconstruction scale, not meters, until "
          "independently scaled against a known real-world reference.")

    med_t = np.median(ft, axis=0)
    # keep quaternion sign consistent before averaging (q and -q represent the same rotation)
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
        "used_frame_indices": used_idx,
        "n_left_eye_registered": len(left_poses),
        "n_right_eye_registered": len(right_poses),
    }
    with open(out_path, "w") as f:
        json.dump(result, f, indent=2)
    print(f"\nwrote {out_path}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
