#!/usr/bin/env python3
"""Build the small native LiDAR module using the workspace's Isaac SDK headers.

No installation or network access. Vendored dependencies carry pinned versions
and licenses. The generated binary stays under build/ and rebuilds after moves.
"""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'sim/native'
BUILD = ROOT/'build/amr_lidar_native'


def build():
    runtime = Path(os.environ.get('ISAACSIM_PATH', ROOT/'runtime/isaac-sim-6.1.0')).resolve()
    includes = [SOURCE/'third_party/pybind11/include', SOURCE/'third_party/physx/include',
                runtime/'kit/dev/include', runtime/'kit/python/include/python3.12']
    required = [includes[0]/'pybind11/pybind11.h', includes[1]/'omni/physx/IPhysxSceneQuery.h',
                includes[2]/'carb/Types.h', includes[3]/'Python.h']
    for path in required:
        if not path.is_file():
            raise RuntimeError(f'Missing native LiDAR build dependency: {path}')
    compiler = shutil.which(os.environ.get('CXX', 'g++'))
    if not compiler:
        raise RuntimeError('Native LiDAR needs GCC with C++17 support (g++). Install the system build toolchain first.')
    environment = os.environ.copy()
    for key in ('LD_LIBRARY_PATH', 'LD_PRELOAD', 'PYTHONHOME', 'PYTHONPATH'):
        environment.pop(key, None)
    # Match omni.physx's public pybind11 type registry (internals v5 / GCC ABI
    # 1016), using GCC's actual ABI compatibility mode instead of pointer hacks.
    flags = ['-std=c++17', '-fabi-version=16', '-O3', '-DNDEBUG', '-shared', '-fPIC',
             '-fvisibility=hidden', '-pthread', '-Wall', '-Wextra']
    paths = sorted(p for p in SOURCE.rglob('*') if p.is_file() and p.suffix in ('.h', '.hpp', '.cpp'))
    paths += [Path(__file__).resolve(), includes[2]/'carb/Defines.h', includes[2]/'carb/Types.h',
              includes[3]/'Python.h', includes[3]/'patchlevel.h', includes[3]/'pyconfig.h']
    digest = hashlib.sha256()
    for path in paths:
        digest.update(str(path).encode());digest.update(path.read_bytes())
    version = subprocess.check_output([compiler, '--version'], text=True, env=environment).splitlines()[0]
    digest.update(version.encode());digest.update(repr(flags).encode())
    fingerprint = digest.hexdigest()
    BUILD.mkdir(parents=True, exist_ok=True)
    output = BUILD/'_amr_lidar_native.so'
    manifest = BUILD/'build_manifest.json'
    with (BUILD/'.build.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        previous = json.loads(manifest.read_text()) if manifest.exists() else {}
        if output.exists() and previous.get('fingerprint') == fingerprint:
            return output
        temporary = BUILD/'_amr_lidar_native.tmp.so'
        command = [compiler, *flags, *(f'-I{p}' for p in includes), str(SOURCE/'amr_lidar_native.cpp'), '-o', str(temporary)]
        print('Building native PhysX LiDAR module...', file=sys.stderr, flush=True)
        try:
            subprocess.run(command, check=True, env=environment)
            temporary.replace(output)
        finally:
            temporary.unlink(missing_ok=True)
        record = {'fingerprint': fingerprint, 'compiler': version, 'flags': flags,
                  'runtime': str(runtime), 'module': str(output),
                  'module_sha256': hashlib.sha256(output.read_bytes()).hexdigest()}
        manifest.write_text(json.dumps(record, indent=2)+'\n')
    return output


if __name__ == '__main__':
    print(build())
