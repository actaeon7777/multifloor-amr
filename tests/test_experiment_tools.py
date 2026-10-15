import importlib.util
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from build_sim_map import boxes_from_grid
from compare_trials import compare
from evaluate_trial import segment_distance, clearance, polygon_clearance


def test_map_extrusion_preserves_occupied_cells():
    occupied=np.array([[0,1,1,0],[1,1,1,0],[1,1,0,1]],dtype=bool)
    boxes=boxes_from_grid(occupied,.05,[-3.,-2.,0.])
    restored=np.zeros_like(occupied)
    for row in range(3):
        for col in range(4):
            x=-3+(col+.5)*.05;y=-2+(3-row-.5)*.05
            restored[row,col]=any(abs(x-b['center'][0])<b['size'][0]/2+1e-9 and abs(y-b['center'][1])<b['size'][1]/2+1e-9 for b in boxes)
    np.testing.assert_array_equal(restored,occupied)


def test_missing_contact_evidence_does_not_pass_gate():
    baseline=[{'world':'arena','seed':i,'success':True,'valid_for_comparison':True} for i in range(5)]
    assert not compare(baseline,baseline)['candidate_gate_passed']


def test_segment_projection_and_endpoint_errors():
    assert segment_distance([.5,.3],[0,0],[1,0])==.3
    assert segment_distance([2,0],[0,0],[1,0])==1.


def test_clearance_uses_oriented_footprint_and_detects_overlap():
    import math
    import pytest
    box={'center':[1.,0.,.5],'size':[.2,2.,1.]}
    assert clearance([0,0,0],[box])==pytest.approx(.58)
    assert clearance([0,0,math.pi/2],[box])==pytest.approx(.628)
    assert clearance([1.,0.,0],[box])==0.


def test_detailed_map_clearance_retains_rotated_obstacle_shape():
    import math
    import pytest
    diamond = [(.3,.7),(.7,.3),(1.1,.7),(.7,1.1)]
    # Perpendicular distance from the footprint corner to x+y=1.
    assert polygon_clearance([0.,0.,0.], [diamond]) == pytest.approx((1.-.320-.272)/math.sqrt(2))
    assert polygon_clearance([.7,.7,0.], [diamond]) == 0.
