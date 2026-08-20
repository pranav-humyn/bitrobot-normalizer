# bitrobot (RoboCap) Normalization

Takes one raw bitrobot chunk folder (calibration + IMU/mag `.db` files +
per-camera `.mp4` clips) and produces the Standard Package v2 normalized
output — the same format already used in production for akai/oak-d.

## Input

One raw bitrobot **chunk directory** — a session's raw upload is split into
many chunk folders (`000`, `001`, ... one per ~10-minute recording segment),
each self-contained:

```
<chunk_dir>/
├── calibration.json         # real Kalibr calibration for the left_eye/right_eye pair
├── calibration_front.json   # calibration for a different pair (not used here)
├── imu_<N>.db                # second IMU (not covered by calibration.json)
├── imu_right_<N>.db          # the IMU calibration.json actually describes
├── mag_middle_<N>.db         # magnetometer (not part of the output schema)
├── left_<N>.mp4, right_<N>.mp4            # the calibrated eye pair (used)
├── left_front_<N>.mp4, right_front_<N>.mp4  # different pair (not used)
└── left_far_<N>.mp4, right_far_<N>.mp4      # different pair, no calibration (not used)
```

Which raw `.mp4` is "left"/"right" is resolved from `calibration.json`'s own
`recorder_camera` field on every run — never hardcoded by filename, since
this can vary in principle by unit.

## Output

| File | What it is |
|---|---|
| `left_raw.mp4`, `right_raw.mp4` | The calibrated eye pair, stream-copied (source is already H.265/HEVC, no re-encode) |
| `left_rectified.mp4`, `right_rectified.mp4` | The stereo pair, undistorted + row-aligned (`cv2.fisheye`, equidistant/Kannala-Brandt model), real H.264 encode |
| `sbs_raw.mp4`, `sbs_rectified.mp4` | Left+right stacked side by side, for quick visual review |
| `calibration.json` | Per-camera intrinsics (raw + rectified), stereo baseline/rotation/translation, and the real camera-IMU extrinsics + noise model from Kalibr |
| `meta.json` | Device identity, recording dimensions/fps/duration, source file info, encoding settings |
| `imu.csv` | `timestamp_s,ax,ay,az,gx,gy,gz` — accel in m/s², gyro in rad/s |
| `frame_timestamps.csv` | `frame_index,timestamp_s` — see note below |
| `status.json` | `stage/run_id/status/error/generated_at` — written on both success and failure |

## Running it

Locally:
```
pip install -r requirements.txt
python scripts/normalize_bitrobot.py <chunk_dir> <output_dir>
```

With Docker:
```
docker build -t bitrobot-normalizer .
docker run --rm \
  -v /path/to/chunk_dir:/input:ro \
  -v /path/to/output_dir:/output \
  bitrobot-normalizer /input /output
```

## Important notes

- **A real vendor calibration now exists** (Kalibr `calibrate_cameras` ->
  `calibrate_imu_camera`) for this rig, replacing the earlier self-calibrated
  version used before it was uploaded — real metric baseline, real
  reprojection-error stats, real camera-IMU extrinsics.
- **Which physical `.db` file the calibration describes was verified, not
  assumed.** `calibration.json`'s IMU block states `update_rate_hz: 202.3`;
  the actual measured sample rate of `imu_right_<N>.db` (202.2Hz) matches
  that almost exactly, versus `imu_<N>.db`'s 199.5Hz — confirming
  `imu_right_<N>.db` is the one described. The device carries a second IMU
  (`imu_<N>.db`) this schema has no slot for; it's read but not used.
- **Raw ADC counts are converted using real, vendor-provided constants** from
  `calibration.json`'s own `_provenance.raw_sensor_scales` (`accel_lsb_per_g:
  8192`, `gyro_lsb_per_dps: 65.5`) — not an inferred/guessed value.
- **No per-frame capture timestamp exists for bitrobot** (unlike akai's fsync
  markers or oak-d's `_ts.csv`). `frame_timestamps.csv` is derived from the
  nominal frame rate (frame *i* at *i*/fps), not a measured per-frame
  timestamp — a real limitation of this device's raw data, not a bug.
- **Strict 11-file output** — only the calibrated eye pair is normalized;
  `left_front`/`right_front` and `left_far`/`right_far` are left out, matching
  the spec's "no per-device variation in package shape" rule (verified
  directly against live akai/oak-d packages in production).
- **Validated so far**: one short chunk end-to-end. Not yet run across a full
  session (22 chunks) or a different unit/site.
