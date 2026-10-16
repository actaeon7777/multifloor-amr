# AMR v80 update — 2026-10-16

## Input

- `AMR_MASTER_Linear v80.step` (Autodesk export 2026-10-16T01:55+09:00), full assembly.
  SHA-256 is recorded in `input_sha256.json`. The STEP itself is not stored in the package.
- 218 assembly occurrences. The CAD origin is the drive-axle midpoint, so CAD coordinates
  (mm) equal `base_link` coordinates.

## Changes from v28

| Item | v28 | v80 |
|---|---|---|
| Drive track (wheel-centre spacing) | 0.499 m | **0.529 m** (each wheel 15 mm outward) |
| Drive wheel outer face | Y ±271.9 mm | Y ±286.8 mm |
| Lower frame | 640 × 520 mm chassis | 640 × 520 mm profile frame, Z −54 to 1 mm |
| Upper structure | none | four 2020 uprights to Z 166 mm, side rails, front cross member |
| Mast / top device | none | 2020 mast to Z 656 mm, Z10T body Z 631–754 mm |
| Drive-wheel mounting | fixed | MGN12 linear guides, motor bracket (`sus` assemblies) |
| Mounting plate | 560 × 360 × 5 mm | removed |

Unchanged (position and volume checked against the v28 meshes): LiDAR Mount v3,
LiDAR body frame (`lidar_link` Z 108.605 mm), four casters at (±190, ±220) mm,
WA171 drive wheel mesh, wheel radius 85.65 mm and axle height.

The 529 mm track keeps the v28 convention: the joint sits 29.6 mm inside the wheel
mesh from the hub face. In v80 the hub face is at Y ±235.0 mm, giving ±264.5 mm.

## Deliverables

- `urdf/capstone_amr.urdf.xacro` and the expanded `capstone_amr.urdf`.
- `meshes/visual/chassis.stl` (31 frame parts, 287,840 triangles),
  `drive_suspension.stl` (both sides, 78,724 triangles; balls and cages omitted),
  `z10t.stl` (Z10T body and mount). All three are in the `base_link` frame, mm.
- `meshes/visual/mounting_plate.stl` removed.
- `meshes/reference/amr_master.stl` regenerated from the whole v80 assembly.
- Collision: boxes from part bounds (see `README_ISAAC_SIM.md`).
- Meshing: OCCT 7.8 (cadquery-ocp 7.8.1.1.post1), 0.2 mm linear and 0.5 rad angular deflection.

## CAD notes

- Each `sus` assembly contains `NURIROBOT_WA171_3D v7` twice at the same placement, and
  `2020-200 (1)` at the top level coincides with `Chassis v5/2020-200:4`. The duplicates
  were exported once.
- `lidar2 v1 v4` fails the OCCT BREP check (17 faces untriangulated), so the existing
  `lidar_body.stl` is kept; its placement matches v80.
- `Z10T v1` was treated as a fixed part of the chassis.

## Not changed / open

- Mass: the measured v28 budget (15.0 kg; `base_link` 9.8 kg) and the `base_link`
  inertia box are retained. v80 adds the upper frame, mast, Z10T and suspension
  hardware and removes the plate, so the real mass and centre of mass need reweighing.
- The linear-guide suspension is modelled as rigid. Spring rate and travel are not in the CAD.
- The four 2020 uprights cross the LiDAR scan plane. From `lidar_link` they occupy about
  ±21.9–30.4° and ±148.6–157.4° at 0.27–0.28 m. The real scan therefore sees the robot body;
  `amr_stack` `scan_body_filter` turns those returns into NaN on the real robot (`/scan_raw` → `/scan`).
  The simulated LiDAR ignores the robot's own colliders and does not reproduce this.
- Isaac Sim runtime import and physics were not executed as part of this description update.
