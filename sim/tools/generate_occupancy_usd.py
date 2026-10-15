#!/usr/bin/env python3
"""Convert a ROS PGM/YAML occupancy map into an Isaac Sim USD environment.

When the annotation PNG contains blue paint, blue is treated as the clean wall
source instead of the noisy occupied pixels in the PGM. Red, green, and pink
paint define steel doors, elevator doors, and locker banks. The tool deliberately
has no dependency on ROS, Pillow, PyYAML, or Pixar USD.
"""

from __future__ import annotations

import argparse
import ast
import math
import os
import struct
import zlib
from pathlib import Path
from typing import Callable, Iterable, Sequence


PROJECT_ROOT = Path(__file__).resolve().parents[2]
MAP_DIRECTORY = PROJECT_ROOT / "src/amr_system/amr_stack/maps/G_4_floor"
DEFAULT_MAP = MAP_DIRECTORY / "4_floor_0522.yaml"
DEFAULT_OUTPUT = PROJECT_ROOT / "sim" / "layers" / "4_floor_0522_map.usda"
DEFAULT_ANNOTATIONS = MAP_DIRECTORY / "4_floor_0522(영역지정).png"


def parse_map_yaml(path: Path) -> dict[str, object]:
    """Read the small, flat YAML subset used by ROS occupancy maps."""
    values: dict[str, str] = {}
    for line_number, original_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = original_line.split("#", 1)[0].strip()
        if not line:
            continue
        if ":" not in line:
            raise ValueError(f"{path}:{line_number}: expected 'key: value'")
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip()

    required = {
        "image",
        "resolution",
        "origin",
        "negate",
        "occupied_thresh",
        "free_thresh",
    }
    missing = sorted(required - values.keys())
    if missing:
        raise ValueError(f"{path}: missing keys: {', '.join(missing)}")

    image_value = values["image"]
    if image_value[:1] in {'"', "'"}:
        image_value = ast.literal_eval(image_value)
    image_path = Path(image_value)
    if not image_path.is_absolute():
        image_path = path.parent / image_path

    origin = ast.literal_eval(values["origin"])
    if not isinstance(origin, (list, tuple)) or len(origin) != 3:
        raise ValueError(f"{path}: origin must be [x, y, yaw]")

    mode = values.get("mode", "trinary").lower()
    if mode != "trinary":
        raise ValueError(
            f"{path}: only ROS trinary maps are supported (got mode={mode!r})"
        )

    resolution = float(values["resolution"])
    occupied_thresh = float(values["occupied_thresh"])
    free_thresh = float(values["free_thresh"])
    negate = int(values["negate"])
    if resolution <= 0:
        raise ValueError(f"{path}: resolution must be positive")
    if negate not in (0, 1):
        raise ValueError(f"{path}: negate must be 0 or 1")
    if not 0 <= free_thresh < occupied_thresh <= 1:
        raise ValueError(
            f"{path}: expected 0 <= free_thresh < occupied_thresh <= 1"
        )

    return {
        "image": image_path.resolve(),
        "resolution": resolution,
        "origin": tuple(float(item) for item in origin),
        "negate": negate,
        "occupied_thresh": occupied_thresh,
        "free_thresh": free_thresh,
        "mode": mode,
    }


def read_pgm(path: Path) -> tuple[int, int, bytes]:
    """Read an 8-bit binary PGM (P5) without image-library dependencies."""
    data = path.read_bytes()
    cursor = 0

    def next_token() -> bytes:
        nonlocal cursor
        while True:
            while cursor < len(data) and data[cursor] in b" \t\r\n\v\f":
                cursor += 1
            if cursor < len(data) and data[cursor] == ord("#"):
                newline = data.find(b"\n", cursor)
                if newline < 0:
                    raise ValueError(f"{path}: unterminated PGM comment")
                cursor = newline + 1
                continue
            break
        start = cursor
        while cursor < len(data) and data[cursor] not in b" \t\r\n\v\f":
            cursor += 1
        if start == cursor:
            raise ValueError(f"{path}: truncated PGM header")
        return data[start:cursor]

    magic = next_token()
    width = int(next_token())
    height = int(next_token())
    max_value = int(next_token())
    if magic != b"P5":
        raise ValueError(f"{path}: expected binary PGM magic P5, got {magic!r}")
    if width <= 0 or height <= 0:
        raise ValueError(f"{path}: PGM dimensions must be positive")
    if max_value != 255:
        raise ValueError(f"{path}: only 8-bit PGM max value 255 is supported")

    # P5 requires one whitespace separator after max_value. Treat CRLF as one
    # logical separator without consuming a whitespace-valued first pixel.
    if cursor >= len(data) or data[cursor] not in b" \t\r\n\v\f":
        raise ValueError(f"{path}: missing separator after PGM max value")
    if data[cursor : cursor + 2] == b"\r\n":
        cursor += 2
    else:
        cursor += 1

    expected_size = width * height
    pixels = data[cursor : cursor + expected_size]
    if len(pixels) != expected_size:
        raise ValueError(
            f"{path}: expected {expected_size} pixels, found {len(pixels)}"
        )
    return width, height, pixels


def read_png_rgb(path: Path) -> tuple[int, int, bytes]:
    """Read an 8-bit, non-interlaced RGB/RGBA PNG as packed RGB bytes."""
    data = path.read_bytes()
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError(f"{path}: not a PNG file")

    cursor = 8
    width = height = bit_depth = color_type = interlace = None
    compressed = bytearray()
    while cursor + 12 <= len(data):
        length = struct.unpack(">I", data[cursor : cursor + 4])[0]
        chunk_type = data[cursor + 4 : cursor + 8]
        chunk_data = data[cursor + 8 : cursor + 8 + length]
        cursor += 12 + length
        if chunk_type == b"IHDR":
            width, height, bit_depth, color_type, _, _, interlace = struct.unpack(
                ">IIBBBBB", chunk_data
            )
        elif chunk_type == b"IDAT":
            compressed.extend(chunk_data)
        elif chunk_type == b"IEND":
            break

    if None in (width, height, bit_depth, color_type, interlace):
        raise ValueError(f"{path}: PNG has no valid IHDR chunk")
    if bit_depth != 8 or color_type not in (2, 6) or interlace != 0:
        raise ValueError(
            f"{path}: expected 8-bit non-interlaced RGB/RGBA PNG "
            f"(got depth={bit_depth}, color_type={color_type}, interlace={interlace})"
        )

    raw = zlib.decompress(bytes(compressed))
    bytes_per_pixel = 3 if color_type == 2 else 4
    stride = width * bytes_per_pixel
    expected_size = height * (stride + 1)
    if len(raw) != expected_size:
        raise ValueError(
            f"{path}: expected {expected_size} decompressed bytes, found {len(raw)}"
        )

    def paeth(left: int, up: int, upper_left: int) -> int:
        estimate = left + up - upper_left
        left_distance = abs(estimate - left)
        up_distance = abs(estimate - up)
        upper_left_distance = abs(estimate - upper_left)
        if left_distance <= up_distance and left_distance <= upper_left_distance:
            return left
        if up_distance <= upper_left_distance:
            return up
        return upper_left

    decoded = bytearray()
    previous = bytearray(stride)
    offset = 0
    for _ in range(height):
        filter_type = raw[offset]
        scanline = raw[offset + 1 : offset + 1 + stride]
        offset += stride + 1
        reconstructed = bytearray(stride)
        for index, value in enumerate(scanline):
            left = reconstructed[index - bytes_per_pixel] if index >= bytes_per_pixel else 0
            up = previous[index]
            upper_left = previous[index - bytes_per_pixel] if index >= bytes_per_pixel else 0
            if filter_type == 0:
                predictor = 0
            elif filter_type == 1:
                predictor = left
            elif filter_type == 2:
                predictor = up
            elif filter_type == 3:
                predictor = (left + up) // 2
            elif filter_type == 4:
                predictor = paeth(left, up, upper_left)
            else:
                raise ValueError(f"{path}: unsupported PNG filter {filter_type}")
            reconstructed[index] = (value + predictor) & 0xFF
        if bytes_per_pixel == 3:
            decoded.extend(reconstructed)
        else:
            # Ignore fully transparent paint and discard alpha for the existing
            # annotation classifier. This also lets editors save RGBA PNGs.
            for index in range(0, len(reconstructed), 4):
                red, green, blue, alpha = reconstructed[index : index + 4]
                if alpha == 0:
                    decoded.extend((255, 255, 255))
                else:
                    decoded.extend((red, green, blue))
        previous = reconstructed
    return width, height, bytes(decoded)


def find_annotation_regions(
    path: Path, expected_width: int, expected_height: int
) -> dict[str, list[tuple[int, int, int, int, int]]]:
    """Return 4-connected colored regions as x0, y0, x1, y1, pixel_count."""
    width, height, rgb = read_png_rgb(path)
    if (width, height) != (expected_width, expected_height):
        raise ValueError(
            f"{path}: annotation size {width}x{height} does not match "
            f"map size {expected_width}x{expected_height}"
        )

    def annotation_type(index: int) -> str | None:
        red, green, blue = rgb[index * 3 : index * 3 + 3]
        return _annotation_type(red, green, blue)

    kinds = [annotation_type(index) for index in range(width * height)]
    regions: dict[str, list[tuple[int, int, int, int, int]]] = {
        "locker": [],
        "steel": [],
        "elevator": [],
        "wall": [],
    }
    visited = bytearray(width * height)
    for seed, kind in enumerate(kinds):
        if kind is None or visited[seed]:
            continue
        visited[seed] = 1
        stack = [seed]
        count = 0
        min_x = max_x = seed % width
        min_y = max_y = seed // width
        while stack:
            index = stack.pop()
            x = index % width
            y = index // width
            count += 1
            min_x = min(min_x, x)
            max_x = max(max_x, x)
            min_y = min(min_y, y)
            max_y = max(max_y, y)
            for neighbor in (
                index - 1 if x else -1,
                index + 1 if x + 1 < width else -1,
                index - width if y else -1,
                index + width if y + 1 < height else -1,
            ):
                if neighbor >= 0 and not visited[neighbor] and kinds[neighbor] == kind:
                    visited[neighbor] = 1
                    stack.append(neighbor)
        if count >= 5:
            regions[kind].append((min_x, min_y, max_x + 1, max_y + 1, count))

    for values in regions.values():
        values.sort(key=lambda region: (region[1], region[0]))
    return regions


def _annotation_type(red: int, green: int, blue: int) -> str | None:
    """Classify one annotation pixel using editor-friendly color tolerances."""
    if red > 200 and green < 60 and blue < 60:
        return "steel"
    if red > 180 and green < 60 and blue > 180:
        return "locker"
    if red < 60 and green > 180 and blue < 80:
        return "elevator"
    if red < 60 and green < 60 and blue > 180:
        return "wall"
    return None


def merge_cell_rectangles(
    width: int,
    height: int,
    pixels: bytes,
    predicate: Callable[[int], bool],
) -> list[tuple[int, int, int, int]]:
    """Merge equal horizontal runs across rows into non-overlapping boxes."""
    active: dict[tuple[int, int], tuple[int, int, int, int]] = {}
    finished: list[tuple[int, int, int, int]] = []

    for row in range(height):
        runs: list[tuple[int, int]] = []
        column = 0
        row_offset = row * width
        while column < width:
            if not predicate(pixels[row_offset + column]):
                column += 1
                continue
            start = column
            column += 1
            while column < width and predicate(pixels[row_offset + column]):
                column += 1
            runs.append((start, column))

        next_active: dict[tuple[int, int], tuple[int, int, int, int]] = {}
        for run in runs:
            if run in active:
                x0, x1, y0, _ = active[run]
                next_active[run] = (x0, x1, y0, row + 1)
            else:
                next_active[run] = (run[0], run[1], row, row + 1)
        finished.extend(rect for run, rect in active.items() if run not in next_active)
        active = next_active

    finished.extend(active.values())
    return finished


def _format_number(value: float) -> str:
    if abs(value) < 5e-12:
        value = 0.0
    return format(value, ".9g")


def _transform_xy(
    x: float, y: float, origin: Sequence[float]
) -> tuple[float, float]:
    origin_x, origin_y, yaw = origin
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        origin_x + cosine * x - sine * y,
        origin_y + sine * x + cosine * y,
    )


def _region_local_bounds(
    region: Sequence[int], height: int, resolution: float
) -> tuple[float, float, float, float]:
    x0, image_y0, x1, image_y1 = region[:4]
    return (
        x0 * resolution,
        x1 * resolution,
        (height - image_y1) * resolution,
        (height - image_y0) * resolution,
    )


def _choose_door_swing(
    region: Sequence[int],
    width: int,
    height: int,
    pixels: bytes,
    resolution: float,
    occupied_threshold: float,
    negate: int,
) -> tuple[tuple[float, float], float, float, float]:
    """Choose a hinge and +/-90 degree swing that avoids mapped obstacles."""
    x0, x1, y0, y1 = _region_local_bounds(region, height, resolution)
    if (x1 - x0) >= (y1 - y0):
        endpoints = ((x0, (y0 + y1) / 2), (x1, (y0 + y1) / 2))
    else:
        endpoints = (((x0 + x1) / 2, y0), ((x0 + x1) / 2, y1))
    length = math.dist(*endpoints)

    def score_point(x: float, y: float) -> float:
        column = math.floor(x / resolution)
        row_from_bottom = math.floor(y / resolution)
        row = height - 1 - row_from_bottom
        if not 0 <= column < width or not 0 <= row < height:
            return -20.0
        pixel = pixels[row * width + column]
        occupancy = pixel / 255.0 if negate else (255 - pixel) / 255.0
        if occupancy > occupied_threshold:
            return -12.0
        if pixel >= 250:
            return 4.0
        return 1.0

    candidates: list[tuple[float, tuple[float, float], float, float]] = []
    for hinge, other in (endpoints, endpoints[::-1]):
        base_angle = math.atan2(other[1] - hinge[1], other[0] - hinge[0])
        for open_angle in (math.pi / 2, -math.pi / 2):
            direction_angle = base_angle + open_angle
            dx = math.cos(direction_angle)
            dy = math.sin(direction_angle)
            normal_x = -dy
            normal_y = dx
            score = 0.0
            for step in range(1, 13):
                distance = length * step / 12
                for lateral_offset in (-0.08, 0.0, 0.08):
                    score += score_point(
                        hinge[0] + dx * distance + normal_x * lateral_offset,
                        hinge[1] + dy * distance + normal_y * lateral_offset,
                    )
            candidates.append((score, hinge, base_angle, open_angle))

    _, hinge, base_angle, open_angle = max(candidates, key=lambda item: item[0])
    return hinge, math.degrees(base_angle), math.degrees(open_angle), length


def _locker_text(
    name: str,
    center: tuple[float, float],
    dimensions: tuple[float, float],
    yaw_degrees: float,
) -> str:
    return f'''            def Cube "{name}" (
                prepend apiSchemas = ["MaterialBindingAPI", "PhysicsCollisionAPI"]
            )
            {{
                bool doubleSided = 1
                float3[] extent = [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)]
                color3f[] primvars:displayColor = [(0.32, 0.40, 0.48)] (
                    interpolation = "constant"
                )
                bool physics:collisionEnabled = 1
                rel material:binding:physics = </World/GMap/Looks/LockerMaterial>
                double size = 1
                double xformOp:rotateZ = {_format_number(yaw_degrees)}
                float3 xformOp:scale = ({_format_number(dimensions[0])}, {_format_number(dimensions[1])}, 1.9)
                double3 xformOp:translate = ({_format_number(center[0])}, {_format_number(center[1])}, 0.95)
                uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateZ", "xformOp:scale"]
            }}
'''


def _steel_door_text(
    name: str,
    hinge: tuple[float, float],
    base_angle: float,
    open_angle: float,
    length: float,
    door_height: float,
) -> str:
    return f'''            def Xform "{name}" (
                prepend apiSchemas = ["PhysicsRigidBodyAPI"]
            )
            {{
                custom token door:type = "steel"
                custom bool door:isOpen = 0
                custom double door:closedAngle = 0
                custom double door:openAngle = {_format_number(open_angle)}
                bool physics:kinematicEnabled = 1
                bool physics:rigidBodyEnabled = 1
                double xformOp:rotateZ = {_format_number(base_angle)}
                double xformOp:rotateZ:doorMotion = 0
                double3 xformOp:translate = ({_format_number(hinge[0])}, {_format_number(hinge[1])}, 0)
                uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateZ", "xformOp:rotateZ:doorMotion"]

                def Cube "Panel" (
                    prepend apiSchemas = ["MaterialBindingAPI", "PhysicsCollisionAPI"]
                )
                {{
                    bool doubleSided = 1
                    float3[] extent = [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)]
                    color3f[] primvars:displayColor = [(0.34, 0.36, 0.38)] (
                        interpolation = "constant"
                    )
                    bool physics:collisionEnabled = 1
                    rel material:binding = </World/GMap/Looks/SteelDoorMaterial>
                    rel material:binding:physics = </World/GMap/Looks/DoorMaterial>
                    double size = 1
                    float3 xformOp:scale = ({_format_number(length)}, 0.05, {_format_number(door_height)})
                    double3 xformOp:translate = ({_format_number(length / 2)}, 0, {_format_number(door_height / 2)})
                    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
                }}

                def Cube "KickPlate" (
                    prepend apiSchemas = ["MaterialBindingAPI"]
                )
                {{
                    rel material:binding = </World/GMap/Looks/DoorDetailMaterial>
                    double size = 1
                    float3 xformOp:scale = ({_format_number(max(0.2, length - 0.12))}, 0.012, 0.28)
                    double3 xformOp:translate = ({_format_number(length / 2)}, -0.032, 0.22)
                    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
                }}

                def Cube "Handle" (
                    prepend apiSchemas = ["MaterialBindingAPI"]
                )
                {{
                    rel material:binding = </World/GMap/Looks/DoorDetailMaterial>
                    double size = 1
                    float3 xformOp:scale = (0.16, 0.035, 0.035)
                    double3 xformOp:translate = ({_format_number(max(0.18, length - 0.22))}, -0.065, 1.0)
                    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
                }}
            }}
'''


def _steel_door_frame_text(
    name: str,
    hinge: tuple[float, float],
    base_angle: float,
    length: float,
    door_height: float,
) -> str:
    return f'''            def Xform "{name}"
            {{
                double xformOp:rotateZ = {_format_number(base_angle)}
                double3 xformOp:translate = ({_format_number(hinge[0])}, {_format_number(hinge[1])}, 0)
                uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateZ"]

                def Cube "HingeJamb" (prepend apiSchemas = ["MaterialBindingAPI"])
                {{
                    rel material:binding = </World/GMap/Looks/DoorFrameMaterial>
                    double size = 1
                    float3 xformOp:scale = (0.09, 0.13, {_format_number(door_height + 0.1)})
                    double3 xformOp:translate = (0, 0, {_format_number((door_height + 0.1) / 2)})
                    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
                }}

                def Cube "LatchJamb" (prepend apiSchemas = ["MaterialBindingAPI"])
                {{
                    rel material:binding = </World/GMap/Looks/DoorFrameMaterial>
                    double size = 1
                    float3 xformOp:scale = (0.09, 0.13, {_format_number(door_height + 0.1)})
                    double3 xformOp:translate = ({_format_number(length)}, 0, {_format_number((door_height + 0.1) / 2)})
                    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
                }}

                def Cube "Header" (prepend apiSchemas = ["MaterialBindingAPI"])
                {{
                    rel material:binding = </World/GMap/Looks/DoorFrameMaterial>
                    double size = 1
                    float3 xformOp:scale = ({_format_number(length + 0.18)}, 0.13, 0.12)
                    double3 xformOp:translate = ({_format_number(length / 2)}, 0, {_format_number(door_height + 0.06)})
                    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
                }}
            }}
'''


def _elevator_panel_text(
    name: str,
    closed_position: tuple[float, float],
    open_position: tuple[float, float],
    yaw_degrees: float,
    panel_length: float,
    door_height: float,
) -> str:
    return f'''                def Xform "{name}" (
                    prepend apiSchemas = ["PhysicsRigidBodyAPI"]
                )
                {{
                    custom token door:type = "elevator"
                    custom bool door:isOpen = 0
                    custom double3 door:closedPosition = ({_format_number(closed_position[0])}, {_format_number(closed_position[1])}, 0)
                    custom double3 door:openPosition = ({_format_number(open_position[0])}, {_format_number(open_position[1])}, 0)
                    bool physics:kinematicEnabled = 1
                    bool physics:rigidBodyEnabled = 1
                    double xformOp:rotateZ = {_format_number(yaw_degrees)}
                    double3 xformOp:translate = ({_format_number(closed_position[0])}, {_format_number(closed_position[1])}, 0)
                    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateZ"]

                    def Cube "Panel" (
                        prepend apiSchemas = ["MaterialBindingAPI", "PhysicsCollisionAPI"]
                    )
                    {{
                        bool doubleSided = 1
                        float3[] extent = [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)]
                        color3f[] primvars:displayColor = [(0.62, 0.66, 0.70)] (
                            interpolation = "constant"
                        )
                        bool physics:collisionEnabled = 1
                        rel material:binding = </World/GMap/Looks/ElevatorPanelMaterial>
                        rel material:binding:physics = </World/GMap/Looks/DoorMaterial>
                        double size = 1
                        float3 xformOp:scale = ({_format_number(panel_length)}, 0.06, {_format_number(door_height)})
                        double3 xformOp:translate = (0, 0, {_format_number(door_height / 2)})
                        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
                    }}
                }}
'''


def _elevator_surround_text(
    center: tuple[float, float],
    yaw_degrees: float,
    length: float,
    door_height: float,
) -> str:
    return f'''                def Xform "Surround"
                {{
                    double xformOp:rotateZ = {_format_number(yaw_degrees)}
                    double3 xformOp:translate = ({_format_number(center[0])}, {_format_number(center[1])}, 0)
                    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateZ"]

                    def Cube "LeftJamb" (prepend apiSchemas = ["MaterialBindingAPI"])
                    {{
                        rel material:binding = </World/GMap/Looks/ElevatorFrameMaterial>
                        double size = 1
                        float3 xformOp:scale = (0.14, 0.14, {_format_number(door_height + 0.18)})
                        double3 xformOp:translate = ({_format_number(-(length / 2 + 0.07))}, 0, {_format_number((door_height + 0.18) / 2)})
                        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
                    }}

                    def Cube "RightJamb" (prepend apiSchemas = ["MaterialBindingAPI"])
                    {{
                        rel material:binding = </World/GMap/Looks/ElevatorFrameMaterial>
                        double size = 1
                        float3 xformOp:scale = (0.14, 0.14, {_format_number(door_height + 0.18)})
                        double3 xformOp:translate = ({_format_number(length / 2 + 0.07)}, 0, {_format_number((door_height + 0.18) / 2)})
                        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
                    }}

                    def Cube "Header" (prepend apiSchemas = ["MaterialBindingAPI"])
                    {{
                        rel material:binding = </World/GMap/Looks/ElevatorFrameMaterial>
                        double size = 1
                        float3 xformOp:scale = ({_format_number(length + 0.28)}, 0.14, 0.18)
                        double3 xformOp:translate = (0, 0, {_format_number(door_height + 0.09)})
                        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
                    }}

                    def Cube "FloorIndicator" (prepend apiSchemas = ["MaterialBindingAPI"])
                    {{
                        rel material:binding = </World/GMap/Looks/IndicatorMaterial>
                        double size = 1
                        float3 xformOp:scale = (0.32, 0.025, 0.10)
                        double3 xformOp:translate = (0, -0.085, {_format_number(door_height + 0.1)})
                        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
                    }}

                    def Cube "CallPanel" (prepend apiSchemas = ["MaterialBindingAPI"])
                    {{
                        rel material:binding = </World/GMap/Looks/IndicatorMaterial>
                        double size = 1
                        float3 xformOp:scale = (0.10, 0.025, 0.28)
                        double3 xformOp:translate = ({_format_number(length / 2 + 0.22)}, -0.085, 1.05)
                        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
                    }}
                }}
'''


def _append_box(
    points: list[tuple[float, float, float]],
    indices: list[int],
    x0: float,
    x1: float,
    y0: float,
    y1: float,
    z0: float,
    z1: float,
    origin: Sequence[float],
) -> None:
    base = len(points)
    xy = [
        _transform_xy(x0, y0, origin),
        _transform_xy(x1, y0, origin),
        _transform_xy(x1, y1, origin),
        _transform_xy(x0, y1, origin),
    ]
    points.extend((x, y, z0) for x, y in xy)
    points.extend((x, y, z1) for x, y in xy)
    for face in (
        (0, 3, 2, 1),
        (4, 5, 6, 7),
        (0, 1, 5, 4),
        (1, 2, 6, 5),
        (2, 3, 7, 6),
        (3, 0, 4, 7),
    ):
        indices.extend(base + item for item in face)


def _append_quad(
    points: list[tuple[float, float, float]],
    indices: list[int],
    x0: float,
    x1: float,
    y0: float,
    y1: float,
    z: float,
    origin: Sequence[float],
) -> None:
    """Append one upward-facing map-local quad transformed into world space."""
    base = len(points)
    points.extend(
        (*_transform_xy(x, y, origin), z)
        for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))
    )
    indices.extend((base, base + 1, base + 2, base + 3))


def _array_lines(items: Iterable[str], indent: str, per_line: int = 8) -> str:
    values = list(items)
    if not values:
        return ""
    lines = []
    for index in range(0, len(values), per_line):
        lines.append(indent + ", ".join(values[index : index + per_line]))
    return ",\n".join(lines)


def _mesh_text(
    name: str,
    points: Sequence[tuple[float, float, float]],
    indices: Sequence[int],
    color: tuple[float, float, float],
    physics_material: str,
    hidden: bool = False,
) -> str:
    if len(indices) % 4:
        raise ValueError(f"{name}: quad index count is not divisible by four")
    minimum = tuple(min(point[axis] for point in points) for axis in range(3))
    maximum = tuple(max(point[axis] for point in points) for axis in range(3))
    point_values = (
        f"({_format_number(x)}, {_format_number(y)}, {_format_number(z)})"
        for x, y, z in points
    )
    index_values = (str(index) for index in indices)
    count_values = ("4" for _ in range(len(indices) // 4))
    min_text = ", ".join(_format_number(value) for value in minimum)
    max_text = ", ".join(_format_number(value) for value in maximum)
    color_text = ", ".join(_format_number(value) for value in color)
    visibility_text = '            token visibility = "invisible"\n' if hidden else ""
    # Keep the slab's surface triangles. The very wide, thin bounding box loses
    # stable contact with the rounded wheel cylinders near the floor centre.
    # A straight-drive/turn reproduction is in sim/scripts/probe_floor_contact.py.
    approximation = 'none'

    return f'''        def Mesh "{name}" (
            prepend apiSchemas = ["MaterialBindingAPI", "PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"]
        )
        {{
            bool doubleSided = 1
            float3[] extent = [({min_text}), ({max_text})]
            int[] faceVertexCounts = [
{_array_lines(count_values, "                ", per_line=24)}
            ]
            int[] faceVertexIndices = [
{_array_lines(index_values, "                ", per_line=16)}
            ]
            point3f[] points = [
{_array_lines(point_values, "                ", per_line=4)}
            ]
            color3f[] primvars:displayColor = [({color_text})] (
                interpolation = "constant"
            )
{visibility_text}            rel material:binding = </World/GMap/Looks/{physics_material}>
            bool physics:collisionEnabled = 1
            uniform token physics:approximation = "{approximation}"
            rel material:binding:physics = </World/GMap/Looks/{physics_material}>
            uniform token subdivisionScheme = "none"
        }}
'''


def _visual_mesh_text(
    name: str,
    points: Sequence[tuple[float, float, float]],
    indices: Sequence[int],
    colors: Sequence[tuple[float, float, float]],
    material: str | None = None,
) -> str:
    """Create a render-only quad mesh with constant or per-face colors."""
    if len(indices) % 4:
        raise ValueError(f"{name}: quad index count is not divisible by four")
    face_count = len(indices) // 4
    if len(colors) not in (1, face_count):
        raise ValueError(f"{name}: expected one or {face_count} colors")
    minimum = tuple(min(point[axis] for point in points) for axis in range(3))
    maximum = tuple(max(point[axis] for point in points) for axis in range(3))
    point_values = (
        f"({_format_number(x)}, {_format_number(y)}, {_format_number(z)})"
        for x, y, z in points
    )
    index_values = (str(index) for index in indices)
    count_values = ("4" for _ in range(face_count))
    color_values = (
        f"({_format_number(red)}, {_format_number(green)}, {_format_number(blue)})"
        for red, green, blue in colors
    )
    interpolation = "constant" if len(colors) == 1 else "uniform"
    min_text = ", ".join(_format_number(value) for value in minimum)
    max_text = ", ".join(_format_number(value) for value in maximum)
    material_text = (
        f"            rel material:binding = </World/GMap/Looks/{material}>\n"
        if material
        else ""
    )
    schema_text = (
        ' (\n            prepend apiSchemas = ["MaterialBindingAPI"]\n        )'
        if material
        else ""
    )
    return f'''        def Mesh "{name}"{schema_text}
        {{
            bool doubleSided = 1
            float3[] extent = [({min_text}), ({max_text})]
            int[] faceVertexCounts = [
{_array_lines(count_values, "                ", per_line=24)}
            ]
            int[] faceVertexIndices = [
{_array_lines(index_values, "                ", per_line=16)}
            ]
            point3f[] points = [
{_array_lines(point_values, "                ", per_line=4)}
            ]
            color3f[] primvars:displayColor = [
{_array_lines(color_values, "                ", per_line=4)}
            ] (
                interpolation = "{interpolation}"
            )
{material_text}            uniform token subdivisionScheme = "none"
        }}
'''


def _finished_wall_bounds(
    rectangle: Sequence[int],
    height: int,
    resolution: float,
    finish_thickness: float,
    snap: float,
) -> tuple[float, float, float, float]:
    """Regularize thin raster runs into clean, slightly overlapping wall strips."""
    column0, column1, row0, row1 = rectangle
    x0 = column0 * resolution
    x1 = column1 * resolution
    y0 = (height - row1) * resolution
    y1 = (height - row0) * resolution
    width = x1 - x0
    depth = y1 - y0
    overlap = finish_thickness * 0.5

    if min(width, depth) <= 3 * resolution and max(width, depth) >= 2 * min(width, depth):
        if width >= depth:
            center = round(((y0 + y1) * 0.5) / snap) * snap
            y0, y1 = center - finish_thickness * 0.5, center + finish_thickness * 0.5
            x0, x1 = x0 - overlap, x1 + overlap
        else:
            center = round(((x0 + x1) * 0.5) / snap) * snap
            x0, x1 = center - finish_thickness * 0.5, center + finish_thickness * 0.5
            y0, y1 = y0 - overlap, y1 + overlap
    elif width <= 3 * resolution and depth <= 3 * resolution:
        center_x = round(((x0 + x1) * 0.5) / snap) * snap
        center_y = round(((y0 + y1) * 0.5) / snap) * snap
        x0, x1 = center_x - finish_thickness * 0.5, center_x + finish_thickness * 0.5
        y0, y1 = center_y - finish_thickness * 0.5, center_y + finish_thickness * 0.5
    return x0, x1, y0, y1


def generate_usda(
    yaml_path: Path,
    output_path: Path,
    wall_height: float,
    floor_thickness: float,
    annotation_path: Path | None = None,
    floor_tile_size: float = 1.0,
    wall_finish_thickness: float = 0.12,
    door_height: float = 2.0,
) -> dict[str, int]:
    if wall_height <= 0:
        raise ValueError("wall height must be positive")
    if floor_thickness <= 0:
        raise ValueError("floor thickness must be positive")
    if floor_tile_size <= 0:
        raise ValueError("floor tile size must be positive")
    if wall_finish_thickness <= 0:
        raise ValueError("wall finish thickness must be positive")
    if door_height <= 0 or door_height >= wall_height:
        raise ValueError("door height must be positive and lower than wall height")

    config = parse_map_yaml(yaml_path)
    width, height, pixels = read_pgm(config["image"])
    resolution = config["resolution"]
    origin = config["origin"]
    occupied_threshold = config["occupied_thresh"]
    negate = config["negate"]

    annotation_regions: dict[str, list[tuple[int, int, int, int, int]]] = {
        "locker": [],
        "steel": [],
        "elevator": [],
        "wall": [],
    }
    annotation_rgb: bytes | None = None
    if annotation_path is not None:
        annotation_regions = find_annotation_regions(
            annotation_path, width, height
        )
        _, _, annotation_rgb = read_png_rgb(annotation_path)

    def is_occupied(pixel: int) -> bool:
        occupancy = pixel / 255.0 if negate else (255 - pixel) / 255.0
        return occupancy > occupied_threshold

    source_occupied_cells = sum(is_occupied(pixel) for pixel in pixels)
    use_annotated_walls = bool(annotation_regions["wall"] and annotation_rgb)
    wall_source = "blue_annotations" if use_annotated_walls else "occupancy_pgm"
    door_cut_cell_count = 0
    if use_annotated_walls:
        wall_pixels = bytes(
            1
            if _annotation_type(*annotation_rgb[index : index + 3]) == "wall"
            else 0
            for index in range(0, len(annotation_rgb), 3)
        )
        rectangles = merge_cell_rectangles(
            width, height, wall_pixels, predicate=bool
        )
    else:
        wall_pixels = bytearray(pixels)
        free_pixel = 0 if negate else 255
        for region in annotation_regions["steel"] + annotation_regions["elevator"]:
            column0, row0, column1, row1 = region[:4]
            for row in range(row0, row1):
                start = row * width + column0
                wall_pixels[start : start + column1 - column0] = bytes(
                    [free_pixel]
                ) * (column1 - column0)
        rectangles = merge_cell_rectangles(
            width, height, bytes(wall_pixels), predicate=is_occupied
        )
        door_cut_cell_count = source_occupied_cells - sum(
            (x1 - x0) * (row1 - row0) for x0, x1, row0, row1 in rectangles
        )
    wall_occupied_cells = sum(
        (x1 - x0) * (row1 - row0) for x0, x1, row0, row1 in rectangles
    )
    if not rectangles:
        raise ValueError(f"{config['image']}: map contains no occupied cells")

    floor_points: list[tuple[float, float, float]] = []
    floor_indices: list[int] = []
    _append_box(
        floor_points,
        floor_indices,
        0.0,
        width * resolution,
        0.0,
        height * resolution,
        -floor_thickness,
        0.0,
        origin,
    )

    wall_points: list[tuple[float, float, float]] = []
    wall_indices: list[int] = []
    wall_snap = max(resolution * 2, 0.1)
    for column0, column1, row0, row1 in rectangles:
        # PGM row zero is the top of the image; ROS map Y grows upward from
        # the bottom row.
        if use_annotated_walls:
            x0, x1, y0, y1 = _finished_wall_bounds(
                (column0, column1, row0, row1),
                height,
                resolution,
                wall_finish_thickness,
                wall_snap,
            )
        else:
            x0 = column0 * resolution
            x1 = column1 * resolution
            y0 = (height - row1) * resolution
            y1 = (height - row0) * resolution
        _append_box(
            wall_points,
            wall_indices,
            x0,
            x1,
            y0,
            y1,
            0.0,
            wall_height,
            origin,
        )

    # The original occupancy geometry stays as the exact invisible collider.
    # A second, render-only shell standardizes noisy 5 cm scan runs into a
    # consistent wall thickness and adds a low baseboard for an indoor finish.
    finish_points: list[tuple[float, float, float]] = []
    finish_indices: list[int] = []
    baseboard_points: list[tuple[float, float, float]] = []
    baseboard_indices: list[int] = []
    for rectangle in rectangles:
        x0, x1, y0, y1 = _finished_wall_bounds(
            rectangle,
            height,
            resolution,
            wall_finish_thickness,
            wall_snap,
        )
        _append_box(
            finish_points, finish_indices, x0, x1, y0, y1, 0.0, wall_height, origin
        )
        baseboard_margin = 0.012
        _append_box(
            baseboard_points,
            baseboard_indices,
            x0 - baseboard_margin,
            x1 + baseboard_margin,
            y0 - baseboard_margin,
            y1 + baseboard_margin,
            0.01,
            0.12,
            origin,
        )

    checker_points: list[tuple[float, float, float]] = []
    checker_indices: list[int] = []
    checker_colors: list[tuple[float, float, float]] = []
    columns = math.ceil(width * resolution / floor_tile_size)
    rows = math.ceil(height * resolution / floor_tile_size)
    checker_palette = ((0.36, 0.37, 0.40), (0.58, 0.59, 0.62))
    for row in range(rows):
        for column in range(columns):
            x0 = column * floor_tile_size
            x1 = min((column + 1) * floor_tile_size, width * resolution)
            y0 = row * floor_tile_size
            y1 = min((row + 1) * floor_tile_size, height * resolution)
            _append_quad(
                checker_points,
                checker_indices,
                x0,
                x1,
                y0,
                y1,
                0.002,
                origin,
            )
            checker_colors.append(checker_palette[(row + column) % 2])

    map_yaw_degrees = math.degrees(origin[2])
    locker_parts: list[str] = []
    for index, region in enumerate(annotation_regions["locker"], start=1):
        x0, x1, y0, y1 = _region_local_bounds(region, height, resolution)
        center = _transform_xy((x0 + x1) / 2, (y0 + y1) / 2, origin)
        locker_parts.append(
            _locker_text(
                f"LockerBank_{index:02d}",
                center,
                (x1 - x0, y1 - y0),
                map_yaw_degrees,
            )
        )

    steel_door_parts: list[str] = []
    steel_frame_parts: list[str] = []
    for index, region in enumerate(annotation_regions["steel"], start=1):
        hinge_local, base_angle, open_angle, length = _choose_door_swing(
            region,
            width,
            height,
            pixels,
            resolution,
            occupied_threshold,
            negate,
        )
        hinge_world = _transform_xy(*hinge_local, origin)
        steel_door_parts.append(
            _steel_door_text(
                f"SteelDoor_{index:02d}",
                hinge_world,
                base_angle + map_yaw_degrees,
                open_angle,
                length,
                door_height,
            )
        )
        steel_frame_parts.append(
            _steel_door_frame_text(
                f"SteelDoorFrame_{index:02d}",
                hinge_world,
                base_angle + map_yaw_degrees,
                length,
                door_height,
            )
        )

    elevator_parts: list[str] = []
    for index, region in enumerate(annotation_regions["elevator"], start=1):
        x0, x1, y0, y1 = _region_local_bounds(region, height, resolution)
        if (x1 - x0) >= (y1 - y0):
            direction = (1.0, 0.0)
            length = x1 - x0
            direction_angle = 0.0
        else:
            direction = (0.0, 1.0)
            length = y1 - y0
            direction_angle = 90.0
        center = ((x0 + x1) / 2, (y0 + y1) / 2)
        panel_length = length / 2
        panel_texts: list[str] = []
        for panel_name, sign in (("NegativePanel", -1.0), ("PositivePanel", 1.0)):
            closed_local = (
                center[0] + direction[0] * sign * length / 4,
                center[1] + direction[1] * sign * length / 4,
            )
            open_local = (
                center[0] + direction[0] * sign * 3 * length / 4,
                center[1] + direction[1] * sign * 3 * length / 4,
            )
            panel_texts.append(
                _elevator_panel_text(
                    panel_name,
                    _transform_xy(*closed_local, origin),
                    _transform_xy(*open_local, origin),
                    direction_angle + map_yaw_degrees,
                    panel_length,
                    door_height,
                )
            )
        center_world = _transform_xy(*center, origin)
        elevator_parts.append(
            f'''            def Xform "ElevatorDoor_{index:02d}"
            {{
{''.join(panel_texts)}{_elevator_surround_text(center_world, direction_angle + map_yaw_degrees, length, door_height)}            }}
'''
        )

    try:
        source_yaml = os.path.relpath(yaml_path.resolve(), output_path.parent.resolve())
        source_image = os.path.relpath(config["image"], output_path.parent.resolve())
        source_annotations = (
            os.path.relpath(annotation_path.resolve(), output_path.parent.resolve())
            if annotation_path is not None
            else ""
        )
    except ValueError:
        source_yaml = yaml_path.resolve().as_posix()
        source_image = config["image"].as_posix()
        source_annotations = (
            annotation_path.resolve().as_posix() if annotation_path is not None else ""
        )
    source_yaml = source_yaml.replace(os.sep, "/")
    source_image = source_image.replace(os.sep, "/")
    source_annotations = source_annotations.replace(os.sep, "/")

    annotation_asset = (
        f"        custom asset sourceAnnotations = @{source_annotations}@\n"
        if annotation_path is not None
        else ""
    )
    objects_text = f'''        def Scope "Objects"
        {{
            def Scope "Lockers"
            {{
{''.join(locker_parts)}            }}

            def Scope "SteelDoors"
            {{
{''.join(steel_door_parts)}            }}

            def Scope "DoorFrames"
            {{
{''.join(steel_frame_parts)}            }}

            def Scope "ElevatorDoors"
            {{
{''.join(elevator_parts)}            }}
        }}
'''

    floor_mesh = _mesh_text(
        "Floor", floor_points, floor_indices, (0.62, 0.65, 0.68), "FloorMaterial"
    )
    wall_mesh = _mesh_text(
        "Walls", wall_points, wall_indices, (0.82, 0.83, 0.82), "WallMaterial", hidden=True
    )
    checker_mesh = _visual_mesh_text(
        "CheckerFloor", checker_points, checker_indices, checker_colors
    )
    finish_mesh = _visual_mesh_text(
        "WallFinish", finish_points, finish_indices, [(0.82, 0.83, 0.82)], "WallFinishMaterial"
    )
    baseboard_mesh = _visual_mesh_text(
        "WallBaseboard", baseboard_points, baseboard_indices, [(0.18, 0.20, 0.22)], "BaseboardMaterial"
    )
    origin_text = ", ".join(_format_number(value) for value in origin)
    text = f'''#usda 1.0
(
    defaultPrim = "World"
    doc = "Generated from {yaml_path.name}; run sim/tools/generate_occupancy_usd.py to rebuild."
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "World"
{{
    def Xform "GMap"
    {{
        custom asset sourceMapYaml = @{source_yaml}@
        custom asset sourceMapImage = @{source_image}@
{annotation_asset}        custom int mapWidth = {width}
        custom int mapHeight = {height}
        custom double mapResolution = {_format_number(resolution)}
        custom double3 mapOrigin = ({origin_text})
        custom token wallSource = "{wall_source}"
        custom int sourceOccupiedCellCount = {source_occupied_cells}
        custom int occupiedCellCount = {wall_occupied_cells}
        custom int doorCutCellCount = {door_cut_cell_count}
        custom int wallRectangleCount = {len(rectangles)}
        custom int lockerCount = {len(annotation_regions['locker'])}
        custom int steelDoorCount = {len(annotation_regions['steel'])}
        custom int elevatorDoorCount = {len(annotation_regions['elevator'])}
        custom int wallAnnotationRegionCount = {len(annotation_regions['wall'])}
        custom double wallHeight = {_format_number(wall_height)}
        custom double doorHeight = {_format_number(door_height)}
        custom double wallFinishThickness = {_format_number(wall_finish_thickness)}
        custom double floorTileSize = {_format_number(floor_tile_size)}

        def Scope "Looks"
        {{
            def Material "FloorMaterial" (
                prepend apiSchemas = ["PhysicsMaterialAPI"]
            )
            {{
                float physics:dynamicFriction = 0.7
                float physics:restitution = 0
                float physics:staticFriction = 0.8
            }}

            def Material "WallMaterial" (
                prepend apiSchemas = ["PhysicsMaterialAPI"]
            )
            {{
                float physics:dynamicFriction = 0.5
                float physics:restitution = 0
                float physics:staticFriction = 0.7
            }}

            def Material "WallFinishMaterial"
            {{
                token outputs:surface.connect = </World/GMap/Looks/WallFinishMaterial/PreviewSurface.outputs:surface>

                def Shader "PreviewSurface"
                {{
                    uniform token info:id = "UsdPreviewSurface"
                    color3f inputs:diffuseColor = (0.82, 0.83, 0.82)
                    float inputs:metallic = 0
                    float inputs:roughness = 0.72
                    token outputs:surface
                }}
            }}

            def Material "BaseboardMaterial"
            {{
                token outputs:surface.connect = </World/GMap/Looks/BaseboardMaterial/PreviewSurface.outputs:surface>

                def Shader "PreviewSurface"
                {{
                    uniform token info:id = "UsdPreviewSurface"
                    color3f inputs:diffuseColor = (0.18, 0.20, 0.22)
                    float inputs:metallic = 0
                    float inputs:roughness = 0.58
                    token outputs:surface
                }}
            }}

            def Material "LockerMaterial" (
                prepend apiSchemas = ["PhysicsMaterialAPI"]
            )
            {{
                float physics:dynamicFriction = 0.5
                float physics:restitution = 0
                float physics:staticFriction = 0.6
            }}

            def Material "DoorMaterial" (
                prepend apiSchemas = ["PhysicsMaterialAPI"]
            )
            {{
                float physics:dynamicFriction = 0.4
                float physics:restitution = 0
                float physics:staticFriction = 0.5
            }}

            def Material "SteelDoorMaterial"
            {{
                token outputs:surface.connect = </World/GMap/Looks/SteelDoorMaterial/PreviewSurface.outputs:surface>

                def Shader "PreviewSurface"
                {{
                    uniform token info:id = "UsdPreviewSurface"
                    color3f inputs:diffuseColor = (0.16, 0.19, 0.22)
                    float inputs:metallic = 0.72
                    float inputs:roughness = 0.32
                    token outputs:surface
                }}
            }}

            def Material "DoorFrameMaterial"
            {{
                token outputs:surface.connect = </World/GMap/Looks/DoorFrameMaterial/PreviewSurface.outputs:surface>

                def Shader "PreviewSurface"
                {{
                    uniform token info:id = "UsdPreviewSurface"
                    color3f inputs:diffuseColor = (0.08, 0.10, 0.12)
                    float inputs:metallic = 0.8
                    float inputs:roughness = 0.25
                    token outputs:surface
                }}
            }}

            def Material "DoorDetailMaterial"
            {{
                token outputs:surface.connect = </World/GMap/Looks/DoorDetailMaterial/PreviewSurface.outputs:surface>

                def Shader "PreviewSurface"
                {{
                    uniform token info:id = "UsdPreviewSurface"
                    color3f inputs:diffuseColor = (0.52, 0.56, 0.60)
                    float inputs:metallic = 0.92
                    float inputs:roughness = 0.18
                    token outputs:surface
                }}
            }}

            def Material "ElevatorPanelMaterial"
            {{
                token outputs:surface.connect = </World/GMap/Looks/ElevatorPanelMaterial/PreviewSurface.outputs:surface>

                def Shader "PreviewSurface"
                {{
                    uniform token info:id = "UsdPreviewSurface"
                    color3f inputs:diffuseColor = (0.50, 0.54, 0.58)
                    float inputs:metallic = 0.94
                    float inputs:roughness = 0.2
                    token outputs:surface
                }}
            }}

            def Material "ElevatorFrameMaterial"
            {{
                token outputs:surface.connect = </World/GMap/Looks/ElevatorFrameMaterial/PreviewSurface.outputs:surface>

                def Shader "PreviewSurface"
                {{
                    uniform token info:id = "UsdPreviewSurface"
                    color3f inputs:diffuseColor = (0.24, 0.27, 0.30)
                    float inputs:metallic = 0.9
                    float inputs:roughness = 0.22
                    token outputs:surface
                }}
            }}

            def Material "IndicatorMaterial"
            {{
                token outputs:surface.connect = </World/GMap/Looks/IndicatorMaterial/PreviewSurface.outputs:surface>

                def Shader "PreviewSurface"
                {{
                    uniform token info:id = "UsdPreviewSurface"
                    color3f inputs:diffuseColor = (0.01, 0.02, 0.015)
                    color3f inputs:emissiveColor = (0.02, 0.32, 0.08)
                    float inputs:metallic = 0.1
                    float inputs:roughness = 0.35
                    token outputs:surface
                }}
            }}
        }}

{floor_mesh}
{checker_mesh}
{wall_mesh}
{finish_mesh}
{baseboard_mesh}
{objects_text}
    }}
}}
'''

    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = output_path.with_suffix(output_path.suffix + ".tmp")
    temporary_path.write_text(text, encoding="utf-8")
    temporary_path.replace(output_path)
    return {
        "source_occupied_cells": source_occupied_cells,
        "wall_occupied_cells": wall_occupied_cells,
        "rectangles": len(rectangles),
        "wall_vertices": len(wall_points),
        "floor_tiles": len(checker_colors),
        "wall_finish_rectangles": len(rectangles),
        "lockers": len(annotation_regions["locker"]),
        "steel_doors": len(annotation_regions["steel"]),
        "elevator_doors": len(annotation_regions["elevator"]),
        "wall_annotation_regions": len(annotation_regions["wall"]),
    }


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "map_yaml",
        nargs="?",
        type=Path,
        default=DEFAULT_MAP,
        help=f"ROS map YAML (default: {DEFAULT_MAP})",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"generated USDA layer (default: {DEFAULT_OUTPUT})",
    )
    annotation_group = parser.add_mutually_exclusive_group()
    annotation_group.add_argument(
        "--annotations",
        type=Path,
        help="RGB/RGBA PNG: blue walls, red steel doors, green elevator doors, pink lockers",
    )
    annotation_group.add_argument(
        "--without-annotations",
        action="store_true",
        help="ignore the default fourth-floor annotation PNG",
    )
    parser.add_argument(
        "--wall-height",
        type=float,
        default=3.0,
        help="wall height in metres (default: 3.0)",
    )
    parser.add_argument(
        "--door-height",
        type=float,
        default=2.0,
        help="steel and elevator door height in metres (default: 2.0)",
    )
    parser.add_argument(
        "--floor-thickness",
        type=float,
        default=0.1,
        help="floor slab thickness in metres (default: 0.1)",
    )
    parser.add_argument(
        "--floor-tile-size",
        type=float,
        default=1.0,
        help="checker floor tile size in metres (default: 1.0)",
    )
    parser.add_argument(
        "--wall-finish-thickness",
        type=float,
        default=0.12,
        help="regularized visible wall thickness in metres (default: 0.12)",
    )
    return parser


def main() -> None:
    arguments = build_argument_parser().parse_args()
    map_yaml = arguments.map_yaml.resolve()
    output = arguments.output.resolve()
    annotations = arguments.annotations.resolve() if arguments.annotations else None
    if (
        annotations is None
        and not arguments.without_annotations
        and map_yaml == DEFAULT_MAP.resolve()
        and DEFAULT_ANNOTATIONS.exists()
    ):
        annotations = DEFAULT_ANNOTATIONS.resolve()
    stats = generate_usda(
        map_yaml,
        output,
        arguments.wall_height,
        arguments.floor_thickness,
        annotations,
        arguments.floor_tile_size,
        arguments.wall_finish_thickness,
        arguments.door_height,
    )
    print(f"generated: {output}")
    print(f"source occupied cells: {stats['source_occupied_cells']}")
    print(f"wall cells after door openings: {stats['wall_occupied_cells']}")
    print(f"merged wall rectangles: {stats['rectangles']}")
    print(f"wall vertices: {stats['wall_vertices']}")
    print(f"checker floor tiles: {stats['floor_tiles']}")
    print(f"finished wall rectangles: {stats['wall_finish_rectangles']}")
    print(
        "annotated objects: "
        f"{stats['lockers']} lockers, "
        f"{stats['steel_doors']} steel doors, "
        f"{stats['elevator_doors']} elevator doors"
    )
    if stats["wall_annotation_regions"]:
        print(f"blue wall regions: {stats['wall_annotation_regions']}")


if __name__ == "__main__":
    main()
