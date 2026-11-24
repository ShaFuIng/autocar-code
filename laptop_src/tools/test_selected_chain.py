"""Regression tests for diagnostic joins outside the selected control chain."""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'laptop_workbench/runtimes/opencv-4.11'),str(ROOT/'web'),str(ROOT/'web/k_lane_runtime')]
from junction_segment_candidate import selected_guard

def sample(x,y,source):return dict(x=x,y=y,source=source)
def data(samples, selected):
    return dict(path=[[p['x']*2,p['y']*2] for p in selected],path_samples=samples,observed_edges=[],valid=True)

# Historical defect: the diagnostic join is below every selected point.
good=[sample(320,y,'double') for y in range(230,243,4)]
bad=[sample(375,246,'single'),sample(375,294,'single'),sample(320,298,'double')]
r=selected_guard(data(good+bad,good))
assert r['path']==data(good,good)['path'] and not r.get('geometry_guard')
# An actual selected-chain discontinuity must still be rejected.
near=[sample(375,y,'single') for y in range(246,263,4)]
r=selected_guard(data(good+near,good+near))
assert r['geometry_guard']['cutoff_y']==246 and len(r['path'])==len(near)
# A continuous source change should not create a cut.
near=[sample(322,y,'single') for y in range(246,263,4)]
r=selected_guard(data(good+near,good+near))
assert not r.get('geometry_guard') and len(r['path'])==len(good+near)
print('3 selected-chain regression cases passed')
