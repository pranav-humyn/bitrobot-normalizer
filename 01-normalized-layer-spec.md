# Normalized Layer Specification — Standard Package v2

**The source of truth for the normalization contract.** Adapters ingest device-native
raw dumps and produce exactly this package — identical shape for every device.
Downstream stages read only this package and never branch on device names. The schema
module (`normalization/schema.py`) and the three adapters implement this document, and
the post-normalization validation gate enforces it at runtime.

- Layer: `normalized/<device>/<session…>/`
- Devices: `zed` · `akai` · `oakd`
- Schema version: **2**

---

## 1. Raw dump — what each device uploads

Every session lands under `raw/<device>/<session…>/`. The device folder name is
authoritative for device type (case-insensitive — `ZED` and `zed` are the same device).
The session path may nest several levels (date / unit / recording / chunk folder); the
pipeline treats everything between the device folder and the files as the session
identity, and **one run is triggered per chunk folder**. Each device has one **trigger
file** — the upload that starts the pipeline; all other files must be uploaded before
(or with) it.

Real session paths, per fleet:

```
raw/oak-d/2026-07-31/WRK-203BLR/rec_20260730_200133/00000/…
raw/akai/2026-08-07/akai-ego-001_2026-08-07_08-35-25/000/…
raw/ZED/first_test_run/<name>.svo2
```

### zed — single-container upload

| File | Content |
|---|---|
| `<name>.svo2` **(trigger)** | ZED SVO2 container: stereo frames, per-frame hardware timestamps, IMU stream, factory calibration, serial number, firmware — all read via the ZED SDK. |

### akai — segmented multi-file upload

| File | Content |
|---|---|
| `left_XXX.mjpeg` / `right_XXX.mjpeg` | Raw MJPEG byte streams, one pair per segment (XXX = 001, 002, … contiguous). |
| `imu_XXX.jsonl` | One JSON object per line: `t_us, ax, ay, az, gx, gy, gz, fsync_flag, fsync_delay_us`. Values are raw ICM-20948 ADC counts on a per-segment microsecond clock. `fsync_flag: 1` rows mark the exact moment a video frame was captured. |
| `calibration.json` **(trigger)** | Kalibr-derived rig calibration (schema v2 or v3): per-camera intrinsics + distortion (equidistant or radtan), `calibration_camera_mapping` (recorder stream ↔ cam0/cam1), `physical_eye`, extrinsics `T_cam1_cam0`, `T_cam0_imu`, `T_cam1_imu`, an `imu` block with four noise parameters, a `device_id` field (e.g. `"akai-ego-001"`), and `temporal.cam_imu_time_offset_s` (Kalibr-calibrated cam-IMU time offset). |
| Session sidecars *(not consumed)* | `left_XXX.pts` / `right_XXX.pts`, `trig_XXX.jsonl`, `meta.json`, `left.meta.json` / `right.meta.json` (~21 MB per-frame recorder metadata), cumulative `left.pts` / `right.pts`, `run_state.json`, `exposure_gate.log`, `system_metrics.csv` — recorder diagnostics present in every session; ignored by normalization. |

### oakd — segmented MP4 upload

| File | Content |
|---|---|
| `left_seg_*.mp4` / `right_seg_*.mp4` | Already-encoded H.264 video segments per eye. |
| `left_seg_*_ts.csv` / `right_seg_*_ts.csv` **(required)** | Per-frame timing + exposure metadata, one row per frame: `seq, t_device_s, t_mid_exposure_s, exposure_us, iso, color_temp_k` — on the **same device clock as imu.csv**. This is oakd's frame-timing signal (akai's equivalent is fsync markers; zed's is SDK stamps). |
| `imu.csv` **(required)** | Exact schema: `t_accel_device_s, t_gyro_device_s, ax, ay, az, gx, gy, gz, qi, qj, qk, qr` — **already SI** (m/s², rad/s), accel and gyro carrying separate per-sensor device-clock timestamps, quaternion columns typically empty. A missing file fails normalization as an incomplete upload. |
| `calibration.json` **(trigger)** | DepthAI device dump, two sections: `summary` (left/right/rgb intrinsics + 14-coefficient distortion, `stereo_baseline_cm`) and `eeprom` (per-socket `cameraData` extrinsics chain, `imuExtrinsics`, factory `stereoRectificationData`, `productName`). This is the fleet-standard format; legacy parsers for two older shapes are retained for tolerance. |
| Not consumed | `rgb_seg_*.mp4` + `rgb_seg_*_ts.csv`, `manifest.csv`, `system.csv`, `unit.json` — RGB stream and recorder diagnostics; ignored by normalization. |

---

## 2. Normalized package — layout & rules

```
normalized/<device>/<session>/
├── left_raw.mp4            REQUIRED
├── right_raw.mp4           REQUIRED
├── sbs_raw.mp4             REQUIRED
├── left_rectified.mp4      REQUIRED
├── right_rectified.mp4     REQUIRED
├── sbs_rectified.mp4       REQUIRED
├── meta.json               REQUIRED
├── calibration.json        REQUIRED
├── imu.csv                 REQUIRED
├── frame_timestamps.csv    REQUIRED
└── status.json             REQUIRED
```

**Every file, every device, every session.** All three devices produce all eleven files
— verified against real fleet uploads. There is no optional file and no per-device
variation in package shape. If a future device cannot produce one of these files,
onboarding it requires a spec revision first — the package shape never varies silently.

### Rules

1. **One package shape.** All eleven files are required for every device and every
   session. Downstream stages read any file unconditionally — no existence checks, no
   device-name branching, no fallbacks.
2. **The package is validated before it is published.** After the adapter finishes, the
   normalization entrypoint verifies: all eleven files present, `calibration.json`
   parses against the schema, `imu.csv` and `frame_timestamps.csv` are non-degenerate
   (rows > 0, non-zero variance, monotonic timestamps). Any violation fails the stage —
   a contract-violating package never reaches the normalized layer as a success.
3. **One writer.** Adapters produce values; the schema module's dataclasses serialize
   `meta.json` and `calibration.json`. No adapter hand-builds output JSON.
4. **Frame ordering guarantee.** Frame `i` of `left_raw.mp4`, frame `i` of
   `right_raw.mp4`, and row `i` of `frame_timestamps.csv` all refer to the same
   captured stereo pair.
5. **Shared clock.** `imu.csv` and `frame_timestamps.csv` share one clock and one
   rebase — this is what makes frame↔IMU alignment possible for visual-inertial
   consumers without touching raw data.
6. **Canonical device strings.** Exactly one spelling per device everywhere: `zed`,
   `akai`, `oakd`. Raw folder names map to these at intake.
7. **Missing means null, never fabricated.** An attribute whose source doesn't exist
   is `null` (`device_id`, `recorded_utc`) — never a placeholder string, never a
   zero-filled row, never the current time standing in for a recording time. This
   applies to attributes only; files are never optional.

---

## 3. Video files

### left_raw.mp4 · right_raw.mp4 · sbs_raw.mp4 (required)

Un-rectified per-eye videos plus a side-by-side composite (left | right hstack, human
review & QA). Constant frame rate; real capture timing lives in
`frame_timestamps.csv`.

| Device | Production |
|---|---|
| zed | Frames decoded from the SVO2 via the SDK (`VIEW.LEFT_UNRECTIFIED` / `VIEW.RIGHT_UNRECTIFIED`), encoded H.264 (libx264, CRF from `stages.normalization.video` config). |
| akai | MJPEG segments byte-concatenated in segment order, encoded HEVC via NVENC (GPU). Frame order is preserved 1:1 from the MJPEG stream. |
| oakd | Source MP4 segments concatenated by ffmpeg stream-copy (no re-encode) per eye; the sbs composite is re-encoded (libx264). |

### left_rectified.mp4 · right_rectified.mp4 · sbs_rectified.mp4 (required)

Stereo-rectified per-eye videos: epipolar lines horizontal, distortion removed,
described by the `rectified` section of `calibration.json`. Consumed by droid (stereo
SLAM), vipe (monocular SLAM), and the vlm_judgment stages (preferred over raw).

| Device | Production |
|---|---|
| zed | SDK-native rectification (`VIEW.LEFT` / `VIEW.RIGHT`) using factory calibration; encoded same as raw. |
| akai | Computed rectification: `cv2.fisheye.stereoRectify` (equidistant model) or `cv2.stereoRectify` (radtan), per-frame `cv2.remap`, NVENC encode. Uses the camera-mapping-corrected intrinsics/extrinsics from the raw calibration. |
| oakd | Computed rectification via `cv2.stereoRectify` (radtan, 14-coefficient distortion) from the parsed device calibration, per-frame remap, libx264 re-encode. The EEPROM's factory `stereoRectificationData` rotations serve as a cross-check. |

---

## 4. meta.json

```jsonc
{
  "schema_version": 2,
  "device": {
    "device_type_from_folder_path": "zed | akai | oakd",  // routing identity — raw/<device>/…, never file content
    "model_from_device_metadata": "<hardware model>",     // read from the device's own data (SDK / calibration / EEPROM)
    "device_id":   "<unit serial>" | null,
    "firmware_version": "<string>" | null
  },
  "recording": {
    "width": <int>, "height": <int>, "fps": <float>,
    "duration_seconds": <float>, "frame_count": <int>,
    "is_stereo": true,
    "codec": "h264 | hevc"
  },
  "source": {
    "original_format": "svo2 | mjpeg | mp4",
    "original_files": ["<filename>", ...],
    "total_bytes": <int>,
    "recorded_utc": "<ISO-8601>" | null,
    "normalized_utc": "<ISO-8601>"
  },
  "encoding":  { "codec": "<encoder>", "crf": <int>|null,
                 "preset": "<string>", "lossless": <bool> },
  "processing_metrics": { "duration_seconds": <float>, "compute_type": "cpu|gpu", ... }
}
```

### Attribute derivation

**device**

| Attribute | zed | akai | oakd |
|---|---|---|---|
| device_type_from_folder_path | *(all)* From the raw folder path (`raw/<device>/…`), mapped to the canonical string at intake. Never from file content — the name states the derivation. | | |
| model_from_device_metadata | SDK camera model | `"Akai-Pi"` | EEPROM `productName` (e.g. `"OAK-D-PRO-W"`) — read from the device's own data; the name states the derivation. |
| device_id | SDK `camera_information.serial_number` | Calibration's `device_id` field (e.g. `"akai-ego-001"`) | EEPROM `deviceName` when non-empty; else `null` |
| firmware_version | SDK `camera_configuration.firmware_version` | `null` | `null` |

**recording**

| Attribute | zed | akai | oakd |
|---|---|---|---|
| width, height | SDK `camera_configuration.resolution` | Resolution field of the raw calibration | Probed from the concatenated output video (cv2); must match calibration dims or the run fails |
| fps | SDK `camera_configuration.fps` | Probed from encoded output; device nominal (30) as fallback | Probed from output video |
| frame_count | SDK `get_svo_number_of_frames()` | Frame count of encoded output | Frame count of concatenated output |
| duration_seconds | *(all)* frame_count / fps | | |
| is_stereo | *(all)* `true` — all three are stereo rigs | | |
| codec | h264 (libx264) | hevc (NVENC) | h264 (source passthrough) |

**source**

| Attribute | zed | akai | oakd |
|---|---|---|---|
| original_format | `"svo2"` | `"mjpeg"` | `"mp4"` |
| original_files | The .svo2 filename | All segment filenames (left + right) | All segment filenames (left + right) |
| total_bytes | *(all)* Sum of the listed source files' sizes | | |
| recorded_utc | First frame's hardware timestamp from the SVO2 (epoch-based, pre-rebase) | `null` — the recorder logs no wall-clock time | `null` — no wall-clock source |
| normalized_utc | *(all)* UTC now, at package write time | | |

---

## 5. calibration.json

One shape for every device. `raw` describes the un-rectified cameras as physically
calibrated; `rectified` describes the virtual cameras of the rectified videos; `imu`
relates the IMU to the cameras. **Left/right always means the video streams** — any
recorder-vs-physical-eye mapping is resolved by the adapter before writing.

```jsonc
{
  "schema_version": 2,
  "source": "zed_sdk | akai_calibration | oak_d_calibration",
  "raw": {
    "left": {
      "fx": <float>, "fy": <float>, "cx": <float>, "cy": <float>,
      "h_fov_degrees": <float>, "v_fov_degrees": <float>, "d_fov_degrees": <float>,
      "distortion": [<float>, ...],
      "distortion_model": "radtan | equidistant"
    },
    "right": { ...same shape... },
    "stereo": { "baseline_mm": <float>, "baseline_meters": <float> },
    "rotation": [[3x3]],          // left-cam -> right-cam
    "translation": [x, y, z]      // meters
  },
  "rectified": { ...same shape as raw... },
  "imu": {
    "T_cam0_imu": [[4x4]],        // IMU -> left camera frame (p_cam = T * p_imu)
    "T_cam1_imu": [[4x4]],        // IMU -> right camera frame
    "accelerometer_noise_density": <float>,   // m/s^2 / sqrt(Hz)
    "accelerometer_random_walk":  <float>,
    "gyroscope_noise_density":    <float>,    // rad/s / sqrt(Hz)
    "gyroscope_random_walk":      <float>,
    "cam_imu_time_offset_s":      <float>     // t_imu = t_cam + offset
  }
}
```

### Attribute derivation

**raw.left / raw.right — intrinsics & distortion**

| Attribute | zed | akai | oakd |
|---|---|---|---|
| fx, fy, cx, cy | SDK `calibration_parameters_raw.left_cam / right_cam` | `cameras.cam0/cam1` intrinsics (`intrinsics` in schema v2, `intrinsics_px` in v3), assigned to left/right via `calibration_camera_mapping` | `summary.left / summary.right` intrinsic matrices (equivalently EEPROM `cameraData` sockets 1/2) |
| distortion | SDK `disto` coefficient array | `distortion_coefficients` (4 coeffs for equidistant; full set for radtan) | `summary.left/right.distortion` (14-coefficient DepthAI model; consumers use the leading radtan coefficients) |
| distortion_model | `"radtan"` (Brown–Conrady) | From the calibration's model field: `"equidistant"` (fisheye) or `"radtan"` | `"radtan"` |
| h/v/d_fov_degrees | SDK per-camera FOV; diagonal computed | Computed from intrinsics — pinhole formula (radtan) or ray-angle via `cv2.fisheye.undistortPoints` (equidistant — the pinhole formula would report a fisheye far too narrow) | Computed from intrinsics (pinhole formula) |

**raw.stereo / rotation / translation — stereo extrinsics**

| Attribute | zed | akai | oakd |
|---|---|---|---|
| rotation, translation | SDK `stereo_transform` | `extrinsics.T_cam1_cam0`, inverted when `calibration_camera_mapping` reverses the streams so the stored transform is always left→right of the *videos* | EEPROM socket chain: general composition `inv(T_root_right) @ T_root_left` (collapses to the direct socket transform on the real fleet layout 1 → 2 → 0), cm→m; baseline cross-checked against `summary.stereo_baseline_cm` |
| baseline | *(all)* Norm of the translation; mm = meters × 1000 | | |

**rectified — post-rectification virtual cameras**

| zed | akai | oakd |
|---|---|---|
| SDK `calibration_parameters` (factory rectified model: zero distortion, common fx/fy) | From the same `stereoRectify` output that built the rectified videos: fx/fy/cx/cy off `P1`/`P2`, distortion = zeros, rotation = identity, translation = pure-x baseline `P2[0,3]/fx` | Same mechanism as akai, from its `cv2.stereoRectify` output |

**imu — IMU-camera extrinsics & noise model**

| Attribute | zed | akai | oakd |
|---|---|---|---|
| T_cam0_imu | SDK `sensors_configuration.camera_imu_transform` (factory IMU↔left-camera transform) | Passthrough: `extrinsics.T_cam0_imu` from the raw calibration | EEPROM `imuExtrinsics` (IMU→socket 0) composed through the socket chain to the left camera (cm→m). **Known factory quirk:** the stored IMU rotation is a 180° flip that measures wrong on real units (true rotation ≈ identity) — the adapter detects exactly that matrix and corrects it to identity, so downstream inherits the corrected transform |
| T_cam1_imu | Composed: stereo extrinsic ∘ camera_imu_transform | Passthrough: `extrinsics.T_cam1_imu` | Composed through the socket chain to the right camera |
| noise densities & random walks (4) | SDK `accelerometer_parameters / gyroscope_parameters` — `noise_density`, `random_walk`; gyro values converted deg→rad | Passthrough: the `imu` block of the raw calibration | Adapter-baked constants (not present in the EEPROM). `accelerometer_noise_density` is **deliberately inflated** (0.08): the BMI accel reports gravity 5–8% off and OKVIS has no accel-scale state, so the estimator must distrust it — field-tuned value |
| cam_imu_time_offset_s | `0.0` — the SDK stamps frames and IMU on one clock | Passthrough: `temporal.cam_imu_time_offset_s` (Kalibr-calibrated, ~-30 ms observed; convention `t_imu = t_cam + offset`) | `0.0` — frame ts.csv and imu.csv share the device clock |

---

## 6. imu.csv

```
timestamp_s, ax, ay, az, gx, gy, gz
```

| Column | Unit | Meaning |
|---|---|---|
| timestamp_s | seconds | Monotonic, starts near 0 at recording start. Shares its clock with `frame_timestamps.csv`. |
| ax, ay, az | m/s² | Linear acceleration (specific force), device IMU frame. |
| gx, gy, gz | rad/s | Angular velocity, device IMU frame. |

### Derivation

| Device | Source & conversion |
|---|---|
| zed | SDK sensor polling during frame iteration (`get_sensors_data`, deduplicated by hardware timestamp). Accel arrives in m/s²; gyro converted deg/s → rad/s. Rebased by the shared t0. |
| akai | Parsed from `imu_XXX.jsonl`; the seven flat keys are required per row — rows missing any key are counted and skipped, never zero-filled (the old parser's `.get(key, 0)` fallback silently produced an all-zero imu.csv). Raw ADC counts → SI via config divisors (`stages.normalization.devices.akai`): accel = counts × 9.80665 / `imu_accelerometer_counts_per_g`; gyro = counts / `imu_gyroscope_counts_per_deg_per_sec` × π/180. Per-segment clocks rebased onto one continuous monotonic timeline. A gravity-magnitude sanity check (median \|accel\| ≈ 9.8) is logged on every run. |
| oakd | Parsed against the exact fleet schema (`t_accel_device_s, t_gyro_device_s, ax…gz, qi…qr`) — values are already SI, no conversion. Row timestamp = `t_gyro_device_s` (gyro timing governs orientation integration; the accel timestamp differs by ≤ ~2 ms and is absorbed by downstream tolerance). Quaternion columns ignored. An unrecognized timestamp column **raises** instead of silently writing 0.0 on every row (the old parser's bug); rows with missing values are skipped, never zero-filled; same gravity sanity check. |

A session whose IMU yields zero usable samples **fails normalization** — every
published package carries a real, non-degenerate `imu.csv`. An all-zero or fabricated
file is equally a validation failure, never a fallback.

---

## 7. frame_timestamps.csv

```
frame_index, timestamp_s
```

| Column | Meaning |
|---|---|
| frame_index | 0-based frame number in `left_raw.mp4` / `right_raw.mp4` (same index = same stereo pair). |
| timestamp_s | Capture time of that frame, on the **same clock and same rebase** as `imu.csv`. This is what makes frame↔IMU alignment possible for visual-inertial consumers (okvis) without touching raw data. |

### Derivation

| Device | Source |
|---|---|
| zed | Per-frame hardware timestamp from the SDK (`get_timestamp(TIME_REFERENCE.IMAGE)`) captured during frame iteration — same clock domain as the SDK sensor timestamps feeding `imu.csv`. |
| akai | Derived from the fsync markers in `imu_XXX.jsonl`: the i-th row with `fsync_flag: 1` timestamps the i-th video frame, at `t_us − fsync_delay_us`. Rebased identically to `imu.csv` (per-segment base + running offset), so relative frame↔IMU alignment is exact. |
| oakd | Concatenation of the per-segment `left_seg_*_ts.csv` files in segment order, using the `t_mid_exposure_s` column (the frame's effective capture instant — with ~30 ms exposures it differs from shutter-end `t_device_s` by ~15 ms, half a frame at 30 fps). Row i timestamps frame i of the concatenated video. Rebased with the same t0 as `imu.csv`. |

---

## 8. status.json

```json
{
  "stage": "normalization",
  "run_id": "<run id>",
  "status": "ok | failed",
  "error": "<message>" | null,
  "generated_at": "<ISO-8601 UTC>"
}
```

Written on every exit path — success and failure alike — by the shared
`write_status()` helper. The state machine's post-normalization gate reads it;
finalize aggregates it into the run summary. `status: "ok"` is written **only after
the package passes contract validation**.

---

## 9. Enforcement gate

`normalization/schema.py::validate_package()` — deliberately stdlib-only so it runs in
every stage image — is invoked by `normalization/__main__.py` after every adapter run,
before `status.json` is written:

- all ten local package files present and non-empty
- `meta.json` parses and carries `schema_version: 2`
- `calibration.json` parses against the full v2 schema (both sections + imu block)
- `imu.csv`: rows > 0, monotonic timeline, sensor values not all zero
- `frame_timestamps.csv`: rows > 0, monotonic timeline

Any violation raises `ContractViolation` (a permanent failure) → the stage fails →
the execution goes red via the `execution_status` machinery. A contract-violating
package never publishes as a success.

---

## 10. Downstream consumption map

What each stage reads from the package — the demand side that justifies every field:

| Stage | Reads |
|---|---|
| validation_metadata_quality | Per-eye videos (probe + pixel checks), `meta.json` device block, `calibration.json` FOV + baseline, `imu.csv` sample rate + gyro motion stats |
| validation_quality_ml | Whichever per-eye videos exist (hand detection @1fps) |
| vlm_judgment (task_description, video_qc) | Configured eye's video — rectified preferred, raw fallback; `meta.json` recording.fps; optional `task.json` |
| usable_clip_extraction | Nothing from the package directly (joins the two validation per-frame parquets + video_qc.json); `meta.json` optional context |
| chunking | The complete package — cuts all six videos frame-exactly, slices `imu.csv` + `frame_timestamps.csv`, copies `calibration.json`, rewrites `meta.json`; each chunk must pass `validate_package` |
| droid | Left+right videos (rectified per config), `calibration.json` baseline_meters |
| vipe | Configured eye's rectified video, `calibration.json` raw intrinsics + `imu.T_cam0_imu` + `cam_imu_time_offset_s` (gravity alignment), `imu.csv`, `meta.json` (dims) |
| okvis | Left+right raw videos, `frame_timestamps.csv`, `imu.csv`, `calibration.json` intrinsics + distortion_model + full `imu` block, `meta.json` (dims + device for IMU saturation limits) |
| state machine / finalize | `status.json` |

**Every stage runs on every device.** The package carries everything every stage
needs, identically for all three devices — no stage has a data precondition that any
device fails. Whether a stage is *enabled* per device remains an ops choice in
`pipeline.yaml` (`stages.<name>.devices` — e.g. okvis on oakd's lower-grade IMU is a
quality-tuning question, not a data question).

---

## 11. Implementation map

Where each piece of this contract lives in code:

| Responsibility | File |
|---|---|
| Schema dataclasses, writers (`meta.json`, `calibration.json`, `imu.csv`, `frame_timestamps.csv`), `validate_package()` gate | `normalization/schema.py` |
| Gate invocation (`ContractViolation` → failed stage) | `normalization/__main__.py` |
| akai adapter: MJPEG→HEVC, fisheye rectification, jsonl→SI IMU + fsync→frame timestamps, calibration passthrough (imu block, device_id, time offset) | `normalization/adapters/akai.py` |
| oakd adapter: stream-copy concat, computed rectification (14-coeff radtan), exact-schema IMU parser, ts.csv→frame timestamps, EEPROM socket-chain imu block (+ factory-rotation correction) | `normalization/adapters/oak_d.py` |
| zed adapter: SDK decode/rectify, per-frame timestamps, SDK imu block (`camera_imu_transform` + noise params), real `recorded_utc` | `normalization/adapters/zed.py` |
| okvis normalized-input consumption: generic EuRoC builder (`build_euroc_normalized`), v2 config generation (`gen_config` hw=normalized, left-first by contract), device-aware IMU saturation limits | `okvis/build_clip.py`, `okvis/__main__.py` |
| vipe normalized-input consumption: package-only fetches, v2-schema gravity alignment (fisheye + radtan), calibrated time offset | `vipe/entrypoint.sh`, `vipe/gravity_align.py`, `vipe/batch_main.py` |
| Batch env wiring: okvis + vipe receive `NORMALIZED_PREFIX` (no stage receives `RAW_PREFIX`) | `infra/terraform/modules/state_machine/templates/pipeline.asl.json` |
| Device-gate ops config (quality choices, not data gates) | `config/pipeline.yaml` |

Notable correctness details encoded in the implementation:

- **okvis camera order**: the legacy raw path hardcoded "cam1-first" (proven on an old
  unit whose recorder mapping was inverted). The normalized path is left-first by
  contract — the adapter resolves any mapping before writing calibration.json — so the
  per-unit ordering landmine is gone.
- **oakd IMU rotation**: the EEPROM's 180°-flip `imuExtrinsics` rotation is corrected
  once in the adapter (okvis previously worked around it with an env override).
- **Shared-rebase invariant**: each adapter chooses one t0 for both `imu.csv` and
  `frame_timestamps.csv`, preserving relative frame↔IMU offsets exactly.

## 12. Verification & first-run checklist

Static verification performed (all passing):

- 48-check suite against **real fleet data**: real akai v3 calibration (baseline
  matches file to 1e-6; imu-block/time-offset passthrough exact; identity-mapping
  branch correct), real oakd EEPROM (socket-chain baseline ≈ summary 7.5 cm; rotation
  corrected; live 14-coeff `cv2.stereoRectify`), real oakd imu.csv (timestamps parse
  correctly — the old parser produced 0.0 on every row), real ts.csv
  (`t_mid_exposure_s` selected), akai fsync consolidation math (counts→SI, per-segment
  rebase, marker-delay application), zed calibration builder round-trip, and the gate's
  positive + negative cases (missing file / all-zero IMU / unparseable calibration).
- `py_compile` on all touched Python, `bash -n` on entrypoint.sh, YAML parse,
  ASL graph walk (113 states, no orphans/dangling refs, zero `RAW_PREFIX`),
  `terraform validate`.

To verify on the first real runs (agreed accepted-risk items):

1. **okvis on normalized mp4** — eyeball the trajectory against a known-good raw-path
   result before trusting at scale (the mp4 is a second lossy encode over the source).
2. **zed `camera_imu_transform` direction** — an inversion would show up immediately
   as gravity misalignment in vipe/okvis output.
3. **zed rolling shutter with okvis** — OKVIS2 has no rolling-shutter compensation;
   check quality before enabling zed in `stages.okvis.devices`.
