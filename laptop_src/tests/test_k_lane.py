import sys,unittest
from pathlib import Path
W=Path(__file__).resolve().parents[1]/'laptop_workbench';sys.path[:0]=[str(W/'runtimes/opencv-4.11'),str(W)]
import numpy as np
from junction_k_lane import LaneGate,lane_evidence

def d(opening=False,near=False,compatible=True,supported=True):
 return dict(opening=opening,near_opening=near,reference_compatible=compatible,a_supported=supported)
class GateTests(unittest.TestCase):
 def test_single_not_junction(self):
  g=LaneGate()
  for _ in range(30):x=g.step(d(supported=False),True,False,True)
  self.assertEqual(x['state'],'A')
 def test_wrong_K_blocked(self):
  g=LaneGate()
  for _ in range(20):x=g.step(d(True,True,False,False),True,False,True)
  self.assertNotEqual(x['state'],'CROSS')
 def test_one_frame_opening_blocked(self):
  g=LaneGate()
  for _ in range(10):g.step(d(True),True,True,True)
  x=g.step(d(True,True),True,True,True)
  self.assertNotEqual(x['state'],'CROSS')
 def test_confirm_then_no_premature_exit(self):
  g=LaneGate()
  for _ in range(8):x=g.step(d(True,True),True,True,True)
  self.assertEqual(x['state'],'CROSS')
  for _ in range(8):x=g.step(d(),True,True,True)
  self.assertEqual(x['state'],'CROSS')
  for _ in range(3):g.step(d(supported=False),True,False,True)
  for _ in range(10):x=g.step(d(),True,True,True)
  self.assertEqual(x['state'],'A')
 def test_mismatched_road_can_cancel(self):
  g=LaneGate()
  for _ in range(8):g.step(d(True,True),True,True,True)
  for _ in range(12):x=g.step(d(compatible=False),True,False,True)
  self.assertEqual(x['state'],'A')
 def test_only_entrance_connected_components_count(self):
  m=np.zeros((360,640),bool)
  for x in [100,300,500]:m[150:355,x:x+5]=True
  a=dict(path=[[400,y*2] for y in range(150,355,4)],observed_edges=[[[104,y] for y in range(150,355,4)],[[300,y] for y in range(150,355,4)]],path_samples=[])
  x=lane_evidence(m,a,dict(fused=[[0,100],[0,300]]))
  self.assertTrue(any(x['raw_zone_counts']))
  self.assertEqual(sum(z['branch_rows'] for z in x['zones']),0)
if __name__=='__main__':unittest.main()
