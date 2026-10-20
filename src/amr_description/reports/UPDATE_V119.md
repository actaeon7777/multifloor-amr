# AMR v119 update — 2026-10-20

## Input

- `AMR_MASTER_Linear v119.step`, full assembly, 215 leaf occurrences. SHA-256 in
  `input_sha256.json`; the STEP itself is not stored in the package.
- Same origin as v80 (drive-axle midpoint), so CAD millimetres equal `base_link` coordinates.

## Changes from v80

Every leaf was compared with v80 by bounding box and volume.

| Item | v80 | v119 |
|---|---|---|
| Housing | none | four solid panels (`좌상단`, `우상단`, `좌하단`, `우하단`), X ±328, Y ±278, Z −55 to 80 mm; fenders over the drive wheels to Z 100 mm |
| Depth camera | none | Orbbec `Gemini 2 v2` (vendor model, 13 parts) in the housing nose |
| Battery connector | none | `Amass XT90E-M`, rear, X −326 to −303 mm |
| Duplicate `2020-200 (1)` | present (coincident with `Chassis v5/2020-200:4`) | removed |
| Rear casters `caster v6:2/3` | — | swivel frame and wheel rotated about the swivel axis; mounts unchanged |

Frame, uprights, mast, Z10T, suspension, LiDAR mount and body, drive wheels and caster
mounts are unchanged. Re-exporting the 31 frame parts from v119 gives the same 287,840
triangles as the v80 `chassis.stl`.

The housing panels are solid bodies, not thin shells (3.0–3.4 L each, 12.8 L total).
Between Z 0 and 80 mm the side bands over the suspension are solid.

## Gemini 2

- Vendor model frame: X across the baseline, Y up, Z out of the lens. Its occurrence
  places the origin at base_link (311, 0, 59) mm with the optical axes along
  (cos 12°, 0, sin 12°): **the camera is pitched 12° up**. The rotation matrix is exact to 1e-9.
- Lens axes (cylindrical faces): IR at Y ±25 mm (50 mm baseline), the smaller lens at
  Y +11 mm, emitter at Y 0. All on one plane through the vendor origin.
- `camera_link` = left (+Y) IR lens axis at the lens front face (vendor Z 9.7 mm, 1.3 mm
  behind the cover glass):
  `xyz = 0.320488 0.025 0.061017`, `rpy = 0 −0.20944 0` (−12° pitch = nose up).
- Assumption: OrbbecSDK_ROS2 uses the depth stream as its base, so its `camera_link`
  coincides with the depth frame, which for active stereo is the left IR camera. The URDF
  therefore defines only `base_link → camera_link`. The driver publishes the depth, color,
  IR and optical frames under it. Check on the real camera with
  `ros2 run tf2_ros tf2_echo camera_link camera_right_ir_frame`. If the assumption holds,
  it reads about (0, −0.050, 0). If it reads (0, +0.050, 0), change `camera_y` to −0.025.
- Mesh `gemini2.stl` is in the `camera_link` frame (98,290 triangles). Placed through the
  joint, it matches the base_link export within 2e-5 mm.

## Checks

- Field of view against the housing and frame meshes: 1,564 rays per lens over the depth
  FOV (91° × 66°) from both IR lenses and over 86° × 55° from the +11 mm lens. **No ray is
  blocked.**
- Floor coverage: the lens is 0.147 m above the floor and the lower FOV edge is −21°. The
  floor enters the depth image 0.38 m ahead of the lens (base_link X 0.70 m). At distance d
  ahead of the lens, only objects taller than 0.147 − 0.384·d m are seen: 0.11 m at 0.1 m and
  0.07 m at 0.2 m.
- LiDAR scan plane (Z 108.605 mm): the housing top is 80 mm and the fenders 99.9 mm, so the
  housing stays below the plane, with 8.7 mm clearance over the fenders at 0.18–0.30 m range.
  A LiDAR tilt above about 1.6° toward the sides would graze the fenders.
  The camera (Z ≤ 72 mm) stays below the plane. `scan_body_filter` needs no change.

## Deliverables

- `meshes/visual/housing.stl`: 4 panels, base_link frame, mm, 8,546 triangles.
- `meshes/visual/gemini2.stl`: camera_link frame, mm.
- `meshes/visual/chassis.stl`: 31 frame parts plus the XT90 connector, 293,092 triangles.
- `meshes/reference/amr_master.stl`: whole v119 assembly, 909,891 triangles. As in v80, 17 faces
  of `lidar2 v1 v4` have no triangulation.
- URDF: housing and camera visuals on `base_link`, empty `camera_link` frame.
  Collision: the lower-frame and suspension-bracket boxes are replaced by a housing box
  (0.656 × 0.556 × 0.135 m, centre Z 0.0125) and two fender boxes (0.228 × 0.098 × 0.020 m at
  (0, ±0.229, 0.090)). The box fills the front nose radius and the lower shoulder beside the
  top deck (Z 60–80 mm at |Y| > 215 mm in parts).
- Meshing: OCCT 7.8 (cadquery-ocp 7.8.1.1.post1), 0.2 mm linear, 0.5 rad angular deflection.

## Not changed / open

- Mass: total set to 18.0 kg (v28 measured 15.0 kg). The 3.0 kg increase is all on `base_link`
  (9.8 → 12.8 kg); drive wheels and casters keep their measured values. The `base_link` centre of
  mass and inertia box are still the v28 approximation, now with the larger mass.
- Footprint: the housing makes the body 0.656 m long (X ±0.328). The Nav2 local costmap footprint,
  `rl/amr_rl/scan.py` and the docs now use X ±0.328. The width is unchanged (±0.287 m at the
  drive-wheel outer faces). The half-diagonal grows from 0.430 to 0.436 m, still inside the global
  costmap `robot_radius` 0.44 m.
- Isaac Sim: no depth-camera sensor prim was added.

## Isaac Sim check (2026-10-20)

`./build_scene.sh` re-imported the physics URDF (`sim/assets/capstone_amr_physics_1`). Runtime mass
18.0 kg; 29 colliders including the housing and fender boxes; `camera_link` present.

- `./run_amr.sh --test`, `validate_passive.py`, `validate_stability.py` (floor4) and
  `probe_floor_contact.py --assert-stable` passed. Against v80 (15 kg): forward 0.408 m vs 0.409 m,
  in-place turn 0.845 rad vs 0.852 rad, ramped stop travel 0.021 m vs 0.020 m, floor4 max tilt 0.079° vs 0.080°.
- Full stack (DWB with the `nav2-shim-tuning` RotationShim settings, as in the v80 run compared), six scenarios × seed 0, 150 s limit
  (`reports/trials/v119_model_20261020`): all reached the goal, no contacts and no command-limit violations.
  The minimum footprint clearance in `s_curve` is 0.076 m (one recovery, 23 s). The v80 run with the same
  navigation settings had 0.133 m clearance and hit the 100 s limit. The other scenarios are within
  ±0.02 m clearance and ±2.4 s of v80. These are single runs.
- The Orbbec ROS 2 driver is not in the workspace.
