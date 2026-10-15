"""Version-checked adapter for the native PhysX ray batch implementation."""
import importlib.util
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
SUPPORTED_PHYSX_VERSION = '110.3.2'


def load_native():
    import omni.kit.app
    manager = omni.kit.app.get_app().get_extension_manager()
    extension_id = manager.get_enabled_extension_id('omni.physx')
    package = manager.get_extension_dict(extension_id)['package']
    if package['version'] != SUPPORTED_PHYSX_VERSION:
        raise RuntimeError(f"Native LiDAR supports omni.physx {SUPPORTED_PHYSX_VERSION}; got {package['version']}. "
                           'Revalidate the SDK ABI before enabling another version.')
    if '_amr_lidar_native' not in sys.modules:
        spec = importlib.util.spec_from_file_location('_amr_build_native', ROOT/'scripts/build_native_lidar.py')
        builder = importlib.util.module_from_spec(spec);spec.loader.exec_module(builder)
        path = builder.build()
        spec = importlib.util.spec_from_file_location('_amr_lidar_native', path)
        module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        sys.modules[spec.name] = module
    return sys.modules['_amr_lidar_native']


class NativeLidarRaycaster:
    """Keep queries on the simulation thread and cache robot collider identities."""
    backend = 'physx_scene_query_cpp_v1'

    def __init__(self, query, stage, robot_path='/World/AMR'):
        from pxr import Sdf, Tf, Usd
        self.stage = stage
        self.robot_path = Sdf.Path(robot_path)
        self.dirty = True
        self.native = load_native().BatchRaycaster(query, [])
        self.notice = Tf.Notice.Register(Usd.Notice.ObjectsChanged, self._changed, stage)
        self._refresh()

    def _changed(self, notice, sender):
        # Pose updates do not change collider identity. Add/remove/resync does.
        if any(path.HasPrefix(self.robot_path) or self.robot_path.HasPrefix(path)
               for path in notice.GetResyncedPaths()):
            self.dirty = True

    def _refresh(self):
        from pxr import PhysicsSchemaTools, Usd
        prim = self.stage.GetPrimAtPath(self.robot_path)
        paths = [PhysicsSchemaTools.sdfPathToInt(p.GetPath()) for p in Usd.PrimRange(prim)] if prim else []
        self.native.set_ignored(paths)
        self.dirty = False

    def cast_batch(self, origin, directions, max_distance):
        if self.dirty:
            self._refresh()
        return self.native.cast(origin, directions, max_distance)

    def close(self):
        self.notice.Revoke()
        self.native.close()
