"""Open the AMR scene or verify real PhysX wheel-drive motion without teleporting."""
import argparse
import asyncio
import json
from physics_parameters import load_config, configure_runtime
from pathlib import Path
import traceback
import time

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser()
parser.add_argument("--test", action="store_true", help="Headless joint-resistance and forward/reverse/turn/stop checks")
parser.add_argument("--preview", action="store_true", help="Save a rendered PNG and exit")
parser.add_argument("--world", choices=("flat", "floor4"), help="Default: detailed fourth floor; --test defaults to flat")
args = parser.parse_args()
args.world = args.world or ('flat' if args.test else 'floor4')
if args.test and args.world != "flat":
    parser.error("--test validates the flat scene; use --world flat")

from isaacsim import SimulationApp

app = SimulationApp({"headless": args.test or args.preview, "width": 1280, "height": 720, "multi_gpu": False})

import carb
import omni.usd
from isaacsim.core.simulation_manager import SimulationManager
from pxr import UsdPhysics


def main():
    scene_name = "capstone_amr_4_floor.usda" if args.world == "floor4" else "amr_flat_test.usda"
    scene = ROOT / "sim" / scene_name
    if not scene.is_file():
        raise FileNotFoundError("Run ./build_scene.sh first")
    build = json.loads((ROOT / "reports/scene_build.json").read_text())
    config = load_config(ROOT)
    configure_runtime()
    carb.settings.get_settings().set("/persistent/app/usd/muteUsdDiagnostics", False)
    assert omni.usd.get_context().open_stage(str(scene))
    for _ in range(30):
        app.update()
    stage = omni.usd.get_context().get_stage()
    assert stage.GetPrimAtPath(build["articulation_root"]).HasAPI(UsdPhysics.ArticulationRootAPI)

    from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
    viewport = get_active_viewport()
    if viewport:
        viewport.camera_path = "/World/Floor4Camera" if args.world == "floor4" else "/World/Camera"
        viewport.set_texture_resolution((1280, 720))
    if args.preview:
        assert viewport, "No renderer viewport available"
        for _ in range(60):
            app.update()
        filename = ROOT / "reports" / ("floor4_preview.png" if args.world == "floor4" else "amr_preview.png")
        temporary = filename.with_name(f".amr_preview_{time.time_ns()}.png")
        capture = capture_viewport_to_file(viewport, str(temporary))
        future = asyncio.ensure_future(capture.wait_for_result())
        while not future.done():
            app.update()
        future.result()
        # The Kit future confirms the capture request, while PNG encoding can
        # finish later on a worker. Wait for the complete PNG end marker.
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            if temporary.is_file() and temporary.stat().st_size > 1000:
                with temporary.open("rb") as stream:
                    stream.seek(-12, 2)
                    if stream.read() == bytes.fromhex("0000000049454e44ae426082"):
                        break
            app.update()
            time.sleep(0.01)
        else:
            raise TimeoutError("PNG encoding did not finish within 30 seconds")
        temporary.replace(filename)
        print(f"AMR_PREVIEW_SAVED {filename}", flush=True)
        return
    from drive_controller import AMRDriveController
    from physics_parameters import validate_config
    from isaacsim.core.experimental.utils import app as app_utils

    validate_config(config)
    SimulationManager.setup_simulation(dt=1 / config["physics_hz"], device="cpu")
    controller = AMRDriveController(build["articulation_root"], config)
    door_controller = None
    try:
        if not args.test:
            if args.world == 'floor4':
                import runpy
                door_controller = runpy.run_path(str(ROOT/'sim/tools/map_door_controller.py'))['MAP_DOOR_CONTROLLER']
            print("AMR ready. Press Play to simulate. Bounded wheel command controller active. Scene: " + str(scene), flush=True)
            while app.is_running():
                app.update()
            return
        report = ROOT / "reports/runtime_validation.json"
        report.write_text(json.dumps({"passed": False, "status": "running"}) + "\n")
        app_utils.play()
        app.update()
        from validate_motion import validate_motion
        result = validate_motion(stage, build, config, controller)
        report.write_text(json.dumps(result, indent=2) + "\n")
        print("AMR_RUNTIME_PASS " + json.dumps(result), flush=True)
        app_utils.stop()
    finally:
        if door_controller:
            door_controller.shutdown()
        controller.close()



try:
    main()
except BaseException:
    traceback.print_exc()
    app.close(exit_code=1)
    raise
else:
    app.close()
