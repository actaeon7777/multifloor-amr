# AMR v28 update — 2026-09-28

## Inputs and geometry

- `AMR_MASTER v28.step`: authoritative assembly placement.
- `LiDAR Mount v3.step` and `Mounting_Plate v3.step`: replacement part geometry.
- Both individual part solids exactly match the assembly's local solids in serialized
  BREP geometry and topology, excluding tessellation. SHA-256 signatures are recorded
  in `standalone_assembly_agreement.json`.
- 52 assembly records compared with v23: component inventory and transforms unchanged.
  Leaf volume/bounds changes occur only in the two replacement parts.

| Part | Local bounds, mm | Old volume, mm³ | New volume, mm³ |
|---|---|---:|---:|
| LiDAR mount | X/Y ±40; Z −5 to 110 | 223293.141653 | 291293.141653 |
| Mounting plate | X ±280, Y ±180; Z −34 to −29 | 1008000 | 962000 |

The mount origin remains at base_link Z = −29 mm. Its assembled lower face is now
Z = −34 mm, matching the bottom of the plate's new opening. The existing sensor
frame remains at base_link Z = 108.605 mm (194.605 mm above the ground).
Existing tiny CAD-to-simulation alignment corrections are preserved.

## Deliverables

- Updated `src/amr_description/urdf/capstone_amr.urdf.xacro` and its generated URDF.
- Updated `meshes/visual/lidar_mount.stl` and `mounting_plate.stl`.
- Mount collision: five boxes following the side walls and top; small screw holes omitted.
- Existing coarse chassis collision still covers the plate opening.
- Updated full-assembly reference STL; not used by the URDF. The vendor assembly
  has BREP validity issues; unchanged vendor parts were not repaired. Both replacement
  part BREPs are valid. The full reference export reported 24 faces skipped for null
  triangulation; this does not affect the two replacement meshes or the URDF, which
  never uses that reference file.
- Updated browser preview Xacro generation and revision annotations.
- Previous assets retained in `backups/before_v28_update.zip` outside the delivery ZIP.

## Validation and reproduction

From the project root, `python scripts/verify_v28.py` validates Xacro/URDF consistency,
the robot tree, preserved joints/dynamics, positive inertias, relative mesh paths,
updated mesh closure/winding/volume/bounds, collision coverage, and CAD placements.
The report is `validation_v28.json`. JavaScript syntax was also checked with `node --check`.

Conversion scripts use workspace-local `.tools/python` packages: cadquery-ocp
7.8.1.1.post1, cadquery-vtk 9.3.1, xacro 2.1.1, and PyYAML 6.0.3. These are not needed
to import the generated URDF. Meshing uses 0.05 mm linear and 0.15 rad angular deflection.

Exact BREP serialization is used for same-part agreement because OCCT 7.8's boolean
operation produced invalid nonempty differences even between two independent imports
of the identical supplied LiDAR STEP. No source geometry was modified to address that
kernel behavior.

Isaac Sim runtime import/physics was not executed. The previous 15.0 kg total and
approximated center of mass/inertias remain; the revised robot has not been reweighed.
