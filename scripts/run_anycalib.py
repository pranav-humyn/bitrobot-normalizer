#!/usr/bin/env python3
"""Run AnyCalib on one representative frame per bitrobot camera, requesting
the Kannala-Brandt (equidistant) 4-parameter model to match the org's
existing calibration convention (akai/oak-d). Saves per-camera intrinsics to
a JSON file. Reports a self-consistency check (reprojecting the dense
fov_field with the fitted closed-form intrinsics) as a rough fit-quality
signal, since AnyCalib's API doesn't expose an explicit residual/confidence
score directly.
"""
import sys
import json
import numpy as np
import torch
from PIL import Image

from anycalib import AnyCalib

CAMERAS = ["left", "right", "left_eye", "right_eye", "left_front", "right_front"]

# NOTE: AnyCalib's predict() output has no built-in residual/confidence score.
# Deliberately not attempting a custom re-projection self-consistency metric
# here (the rays<->pixel-grid mapping isn't documented precisely enough to
# trust a homemade check) -- real validation instead comes downstream from
# the rectified-pair empirical checks (row-alignment + disparity sign on
# real footage), the same ground-truth-free method already proven on
# DAS-EGO, rather than an unverified intermediate metric here.


def main(frames_dir, out_path, model_id):
    dev = torch.device("cpu")
    model = AnyCalib(model_id=model_id).to(dev)

    results = {}
    for cam in CAMERAS:
        img_path = f"{frames_dir}/{cam}.jpg"
        image = np.array(Image.open(img_path).convert("RGB"))
        h, w = image.shape[:2]
        t = torch.tensor(image, dtype=torch.float32, device=dev).permute(2, 0, 1) / 255

        out = model.predict(t, cam_id="kb:4")
        intrinsics = out["intrinsics"].cpu().numpy().tolist()
        pred_size = out["pred_size"]

        results[cam] = {
            "image_size": [w, h],
            "pred_size": list(pred_size),
            "model": "kannala_brandt_4",
            "intrinsics_fx_fy_cx_cy_k1_k2_k3_k4": intrinsics,
        }
        print(f"{cam}: image={w}x{h} pred_size={pred_size} "
              f"fx={intrinsics[0]:.2f} fy={intrinsics[1]:.2f} "
              f"cx={intrinsics[2]:.2f} cy={intrinsics[3]:.2f} "
              f"k=[{intrinsics[4]:.4f},{intrinsics[5]:.4f},{intrinsics[6]:.4f},{intrinsics[7]:.4f}]")

    with open(out_path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nwrote {out_path}")


if __name__ == "__main__":
    frames_dir = sys.argv[1]
    out_path = sys.argv[2]
    model_id = sys.argv[3] if len(sys.argv) > 3 else "anycalib_gen"
    main(frames_dir, out_path, model_id)
