import sys,unittest
from pathlib import Path
from unittest.mock import patch
W=Path(__file__).resolve().parents[1]/'laptop_workbench';sys.path[:0]=[str(W/'runtimes/opencv-4.11'),str(W)]
import numpy as np
import road_segments_v32 as mod

def trace(a,b,x):
 return np.array([[y,x-3,x+3] for y in range(a,b+1,4)],float)
class EntranceTests(unittest.TestCase):
 def test_far_pair_rejected(self):
  r=mod.RoadPath('road-entrance-v3.2');r.width=np.array([[150,180],[350,240]]);r.width_at=0
  with patch.object(mod,'traces',return_value=[trace(150,210,250),trace(150,210,400),trace(210,354,180)]):
   x=r.step(np.zeros((360,640,3),np.uint8),np.zeros((360,640),bool),0)
  self.assertTrue(any(z['reason']=='FAR_PAIR_NOT_CONNECTED_TO_ENTRANCE' for z in x['rejections']))
  self.assertEqual(x['coverage']['double_rows'],0)
  self.assertTrue(all(e[-1][1]>=270 for e in x['observed_edges']))
 def test_failed_alignment_does_not_overwrite_anchor(self):
  r=mod.RoadPath('road-entrance-v3.2');r.t=0;r.memory_gray=np.zeros((360,640),np.uint8)
  r.memory=[dict(x=320.,y=250.,source='double',observed_at=0.,base=1.,alignment_error=0.)]
  with patch.object(mod,'estimate_motion',return_value=(None,dict(valid=False))),patch.object(mod,'traces',return_value=[trace(230,354,200),trace(230,354,450)]):
   x=r.step(np.zeros((360,640,3),np.uint8),np.zeros((360,640),bool),.1)
  self.assertEqual(r.memory[0]['x'],320.)
  self.assertEqual(r.memory[0]['observed_at'],0.)
 def test_near_pair_can_rebuild_after_confirmation(self):
  r=mod.RoadPath('road-entrance-v3.2');r.t=0;r.memory_gray=np.zeros((360,640),np.uint8)
  r.memory=[dict(x=320.,y=250.,source='double',observed_at=0.,base=1.,alignment_error=0.)]
  with patch.object(mod,'estimate_motion',return_value=(None,dict(valid=False))),patch.object(mod,'traces',return_value=[trace(230,354,200),trace(230,354,450)]):
   for t in [.1,.2,.3]:x=r.step(np.zeros((360,640,3),np.uint8),np.zeros((360,640),bool),t)
  self.assertTrue(any(z['reason']=='CONFIRMED_ENTRANCE_ANCHOR_REBUILT' for z in x['rejections']))
if __name__=='__main__':unittest.main()
