"""Bounded GUI smoke trial with Kit CPU tracing; never changes the installed SDK."""
import argparse
import json
import os
from pathlib import Path
import runpy
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--settings', type=json.loads, default={})
parser.add_argument('--trace', action='store_true')
parser.add_argument('--render-loading', choices=['nonblocking', 'synchronous'], default='synchronous')
parser.add_argument('--exercise-doors', action='store_true')
args = parser.parse_args()
output = args.output.resolve();output.mkdir(parents=True, exist_ok=False)
os.environ['AMR_SIM_PROFILE'] = str(output/'sim_profile.json')
os.environ['ROS_DOMAIN_ID'] = '107'
# Probe explicitly supplied settings rather than the production override.
os.environ['AMR_RENDER_LOADING'] = args.render_loading

from isaacsim import SimulationApp
from runtime_profile import RuntimeProfile

original_init, original_update = SimulationApp.__init__, SimulationApp.update
original_record = RuntimeProfile.record
active = False
finished = False
samples = []
profiler = None
doors_exercised = False


def initialize(self, configuration=None, *positional, **keywords):
    configuration = dict(configuration or {})
    configuration['extra_args'] = list(configuration.get('extra_args', [])) + [
        f'--{key}={json.dumps(value)}' for key, value in args.settings.items()]
    original_init(self, configuration, *positional, **keywords)
    import carb
    settings = carb.settings.get_settings()
    for key, value in args.settings.items():
        settings.set(key, value)


def update(self):
    wall, cpu = time.perf_counter(), time.thread_time()
    original_update(self)
    if active:
        samples.append({'wall_time': wall, 'wall_ms': 1000*(time.perf_counter()-wall),
                        'main_thread_cpu_ms': 1000*(time.thread_time()-cpu)})


def record(self, name, start, sim_time):
    global active, finished, profiler, doors_exercised
    original_record(self, name, start, sim_time)
    if not active and not finished and sim_time >= 1.:
        import carb
        import carb.profiler
        settings = carb.settings.get_settings()
        keys = ['/app/runLoops/main/rateLimitEnabled', '/app/runLoops/main/rateLimitFrequency',
                '/app/asyncRendering', '/app/asyncRenderingLowLatency', '/app/hydraEngine/waitIdle',
                '/app/gatherRenderResults', '/rtx/ecoMode/enabled',
                '/rtx-transient/hydra/geometrystreaming/active',
                '/rtx-transient/hydra/geometrystreaming/syncLoad',
                '/rtx-transient/hydra/geometrystreaming/streamingBudgetMB',
                '/rtx/hydra/materialSyncLoads', '/rtx/materialDb/syncLoads',
                '/rtx/hydra/geometrystreaming/gpuBudgetMB',
                '/UJITSO/geometry', '/persistent/UJITSO/geometry',
                '/exts/omni.kit.renderer.core/threads/syncMainToPresent',
                '/exts/omni.kit.renderer.core/present/enabled',
                '/exts/omni.kit.renderer.core/present/displays/limit']
        (output/'settings.json').write_text(json.dumps({k: settings.get(k) for k in keys}, indent=2)+'\n')
        if args.trace:
            profiler = carb.profiler.acquire_profiler_interface(plugin_name='carb.profiler-cpu.plugin')
            profiler.set_capture_mask(0x0FFFFFFFFFFFFFFFF)
            settings.set('/plugins/carb.profiler-cpu.plugin/filePath', str(output/'kit_trace.json'))
            settings.set('/plugins/carb.profiler-cpu.plugin/compressProfile', False)
            settings.set('/plugins/carb.profiler-cpu.plugin/saveProfile', True)
        active = True
        if args.exercise_doors:
            from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
            capture_viewport_to_file(get_active_viewport(), str(output/'doors_closed.png'))
        print('GUI_PROBE_CAPTURE_START', flush=True)
    elif active and sim_time >= 5.5:
        active = False;finished = True
        if profiler:
            import carb
            carb.settings.get_settings().set('/plugins/carb.profiler-cpu.plugin/saveProfile', False)
            profiler.set_capture_mask(0)
        (output/'frames.json').write_text(json.dumps(samples, indent=2)+'\n')
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
        capture_viewport_to_file(get_active_viewport(), str(output/'viewport.png'))
        print('GUI_PROBE_CAPTURE_END', len(samples), flush=True)
    if args.exercise_doors and not doors_exercised and sim_time >= 2.:
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        changed = []
        for prim in stage.Traverse():
            kind = prim.GetAttribute('door:type').Get()
            if kind == 'steel':
                prim.GetAttribute('xformOp:rotateZ:doorMotion').Set(prim.GetAttribute('door:openAngle').Get())
            elif kind == 'elevator':
                prim.GetAttribute('xformOp:translate').Set(prim.GetAttribute('door:openPosition').Get())
            else:
                continue
            prim.GetAttribute('door:isOpen').Set(True)
            changed.append({'path':str(prim.GetPath()), 'type':kind, 'is_open':True})
        assert any(p['type']=='steel' for p in changed) and any(p['type']=='elevator' for p in changed)
        (output/'door_probe.json').write_text(json.dumps(changed, indent=2)+'\n')
        doors_exercised = True


SimulationApp.__init__, SimulationApp.update = initialize, update
RuntimeProfile.record = record
sys.argv = [str(ROOT/'sim/scripts/run_ros_sim.py'), '--headless', 'false', '--smoke-test']
runpy.run_path(sys.argv[0], run_name='__main__')
