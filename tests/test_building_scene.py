"""Indoor lighting of the generated stacked building (sim/tools/build_building_scene.py)."""
from pathlib import Path
import re
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'sim/tools'))
import build_building_scene as scene  # noqa: E402


def test_lights_centred_along_corridors_and_spaced():
    interior = np.zeros((200, 200), bool)
    interior[10:50, 5:195] = True     # corridor along x: y 0.5-2.5 m (centre 1.5)
    interior[50:56, 120:125] = True   # door recess in its north wall
    interior[50:190, 60:100] = True   # corridor along y: x 3-5 m (centre 4)
    lights = scene.ceiling_light_positions(interior, .05, (0., 0.), clearance=.8, spacing=3.)
    along_x = [(x, y) for x, y, along_y in lights if not along_y]
    along_y = [(x, y) for x, y, along_y in lights if along_y]
    assert len(along_x) >= 2 and len(along_y) >= 2
    assert all(abs(y - 1.5) < .03 for _, y in along_x)  # centred, recess ignored
    assert all(abs(x - 4.) < .03 for x, _ in along_y)
    for x, y, _ in lights:
        assert interior[int(y / .05), int(x / .05)]
    for i, a in enumerate(lights):
        for b in lights[i + 1:]:
            assert np.hypot(a[0] - b[0], a[1] - b[1]) >= 3. - 1e-9


def test_generated_building_is_lit_indoors():
    output = ROOT / 'sim/_test_building.usda'  # build() reports the path relative to the workspace
    world = (ROOT / 'sim/config/building.json').read_text()
    try:
        result = scene.build(ROOT / 'src/amr_system/amr_stack/config/floors.yaml', output)
        text = output.read_text()
    finally:
        (ROOT / 'sim/config/building.json').write_text(world)
        output.unlink(missing_ok=True)
    for light in ('Sun', 'Fill'):
        assert re.search(rf'over "{light}" \(\s*active = false', text)
    assert all(count > 5 for count in result['ceiling_lights'].values())
    assert text.count('def RectLight "Light_') == sum(result['ceiling_lights'].values())
    assert 'def RectLight "CarLight"' in text
    # Ceiling and light panels are visual only: LiDAR raycasts and contacts must not see them.
    for name in ('Ceiling', 'LightPanels'):
        for match in re.finditer(rf'def Mesh "{name}" \(\n\s*prepend apiSchemas = \[([^\]]*)\]', text):
            assert 'Collision' not in match.group(1)
