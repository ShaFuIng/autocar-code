"""Prepared candidate adapter; no hardware, PID, network or service calls."""
import sys
from pathlib import Path
import cv2
sys.path.insert(0,str(Path(__file__).resolve().parent/'k_lane_runtime'))
from k_parallel_path import ParallelKLane
class KLaneLivePath:
 algorithm='junction-k-lane-v3'
 def __init__(self,row=.6):
  self.row=row;self.tracker=ParallelKLane(row)
 def step(self,image,mask,captured,on_a=None):
  r=self.tracker.step(image,mask,captured,on_a=(lambda a:on_a(self.format(a,captured))) if on_a else None)
  return self.format(r,captured)
 def set_row(self,row):
  self.row=row;self.tracker.set_row(row)
 def close(self):self.tracker.close()
 def format(self,r,captured):
  diag={k:r[k] for k in ('mode','reason','k','road_segments','history_segments','path_semantics','observed_edges','coverage','motion','rejections','parallel','geometry_guard') if k in r}
  diag.update(path_revision='k-branch-geometry-live-r1',opencv=cv2.__version__,capture_s=captured,control_connected=False)
  return dict(path=r['path'],quality=float(r['quality']),valid=bool(r['valid']),path_diagnostic=diag)
