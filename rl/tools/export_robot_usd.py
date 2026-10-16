"""Export the AMR with the scene's physics overrides as a standalone USD for Isaac Lab.

`sim/scripts/build_scene.py` authors drive gains, joint resistance, wheel materials and
contact compliance as overrides on `/World/AMR` in `sim/amr_flat_test.usda`, with the
materials under `/World/Materials`. Isaac Lab references one default prim per robot, so
move both under a single `/AMR` prim. The robot asset is still referenced, not copied.
"""
import argparse
import os
from pathlib import Path

from pxr import Sdf

ROOT = Path(__file__).resolve().parents[2]
SCENE = ROOT / "sim/amr_flat_test.usda"
OUTPUT = ROOT / "rl/assets/amr_rl.usda"
MOVES = {"/World/AMR": "/AMR", "/World/Materials": "/AMR/Materials"}


def remap(path):
    for old, new in MOVES.items():
        if path == old or path.startswith(old + "/"):
            return new + path[len(old):]
    raise ValueError(f"Relationship target outside the exported prims: {path}")


def export(scene, output):
    source = Sdf.Layer.FindOrOpen(str(scene))
    assert source, scene
    layer = Sdf.Layer.CreateAnonymous(".usda")
    for key in ("metersPerUnit", "upAxis", "kilogramsPerUnit"):
        if source.pseudoRoot.HasInfo(key):
            layer.pseudoRoot.SetInfo(key, source.pseudoRoot.GetInfo(key))
    # Dict order matters: /AMR must exist before /AMR/Materials is copied under it.
    for old, new in MOVES.items():
        assert Sdf.CopySpec(source, old, layer, new), old
    layer.defaultPrim = "AMR"

    robot = layer.GetPrimAtPath("/AMR")
    # Isaac Lab owns the spawn pose; drop the scene's 2 mm lift.
    for name in list(robot.properties.keys()):
        if name.startswith("xformOp"):
            robot.RemoveProperty(robot.properties[name])
    references = robot.referenceList.prependedItems
    assert len(references) == 1, references
    asset = (scene.parent / references[0].assetPath).resolve()
    assert asset.is_file(), asset
    robot.referenceList.prependedItems = [Sdf.Reference("./" + os.path.relpath(asset, output.parent))]

    retargeted = 0

    def visit(path):
        nonlocal retargeted
        if not path.IsPropertyPath():
            return
        spec = layer.GetRelationshipAtPath(path)
        if not spec:
            return
        for attr in ("explicitItems", "prependedItems", "appendedItems", "addedItems"):
            items = list(getattr(spec.targetPathList, attr))
            if items:
                setattr(spec.targetPathList, attr, [Sdf.Path(remap(str(item))) for item in items])
                retargeted += len(items)

    layer.Traverse("/AMR", visit)
    output.parent.mkdir(parents=True, exist_ok=True)
    layer.Export(str(output))
    return retargeted


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--scene", type=Path, default=SCENE)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    count = export(args.scene.resolve(), args.output.resolve())
    print(f"AMR_RL_ASSET {args.output} retargeted_relationships={count}")
