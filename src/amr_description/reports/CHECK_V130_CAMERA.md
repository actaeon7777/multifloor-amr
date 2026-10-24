# AMR v130 camera and IMU check — 2026-10-24

## Input

- `AMR_MASTER_Linear v130.step`, 218 leaf occurrences (v119: 215). SHA-256 in `input_sha256.json`;
  the STEP is kept outside the workspace (`~/Documents/AMR_CAD/` on the Jetson).
- Only the camera and IMU placement were checked. Meshes, collision boxes and mass were not
  regenerated from v130, and the other parts were not compared with v119.

## Gemini 2 placement

- `Gemini 2 v2:1` is at base_link (311, 0, 59) mm with the optical axis along
  (0.978148, 0, 0.207912) and vendor X along base +Y, the same as v119. `camera_link`
  stays at `xyz = 0.320488 0.025 0.061017`, `rpy = 0 −0.20944 0`.
- Factory extrinsics (OrbbecSDK v2.10.6, FW 1.4.92): depth and IR share intrinsics and an
  identity extrinsic, so the depth frame is the IR camera. The color origin is +13.9 mm along
  optical X from depth, matching the CAD IR (Y +25) to color (Y +11) spacing. The depth origin
  is therefore the left (+Y) IR lens, and `camera_y = +0.025` is correct. This settles the open
  assumption in `UPDATE_V119.md`.

## Real vehicle on the floor

Floor-plane fit to the median of 15 depth frames, 0.34–1.82 m ahead, 226,893 points, 2.6 mm rms:

| | Measured | URDF |
|---|---|---|
| Pitch up | 11.84° | 12° |
| Roll | 0.31° | 0° |
| Lens height above floor | 0.1476 m | 0.147 m |

Fits over 0.3–0.6, 0.6–1.0 and 1.0–2.0 m give 11.6–12.0° and 0.146–0.152 m. The URDF is unchanged.

## Camera IMU

- At rest on the same spot the Gemini 2 accelerometer reads 13.5° pitch and −0.9° roll; the
  STM32 accelerometer reads 0.29° base tilt. The camera IMU is about 1.5° (pitch) and 1° (roll)
  off the optical axes, while the SDK reports an identity depth→IMU extrinsic.
- The stack does not use the camera IMU. Using it later needs this rotation calibrated first.

## STM32 IMU

`imu_link` at `0.097 0 0.021` matches the CAD measurement (97.153, 0, 21.039) mm within 0.2 mm.
