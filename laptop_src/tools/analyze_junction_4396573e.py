"""Read-only local replay of frozen masks; no hardware or model loading."""
import sys,json,base64,shutil
from pathlib import Path
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'laptop_workbench/data/experiments/junction-4396573e'
OUT.mkdir(parents=True,exist_ok=True)
sys.path.insert(0,str(ROOT/'web'));sys.path.insert(0,str(ROOT/'web/k_lane_runtime'))
from road_segments_v32 import RoadPath
from temporal_lane import estimate_motion
RUN=ROOT.parent/'firmware_builds/first_fault_v1/run-4396573e'
def load_tunnel():
 d=json.loads((RUN/'evidence.json').read_text(encoding='utf8'))
 logs=[json.loads(l) for k,v in d['files'].items() if k.endswith('/pid.jsonl') for l in v.splitlines()]
 frames=[json.loads(l) for k,v in d['files'].items() if 'frames_' in k for l in v.splitlines()]
 cap=cv2.VideoCapture(str(RUN/'video_004.avi'));items=[]
 for row in logs:
  v=row.get('vision',{})
  if not v.get('mask_png'):continue
  f=min(frames,key=lambda f:abs(f['capture_monotonic']-v['capture_at']))
  cap.set(1,f['frame']);ok,im=cap.read();assert ok
  mask=cv2.imdecode(np.frombuffer(base64.b64decode(v['mask_png']),np.uint8),0)>0
  items.append((v['seq'],v['capture_at'],cv2.resize(im,(640,360)),mask,v,f))
 cap.release();return items,logs[0]['parameters']
if __name__=='__main__':
 tracker=RoadPath('road-entrance-v3.2');rows=[]
 for seq,t,im,mask,v,f in load_tunnel()[0]:
  anchor=tracker.memory_gray
  count=None
  if anchor is not None:
   roi=np.zeros_like(anchor);roi[172:338,20:-20]=255
   pts=cv2.goodFeaturesToTrack(anchor,250,.02,9,mask=roi);count=0 if pts is None else len(pts)
  r=tracker.step(im,mask,t)
  rows.append(dict(seq=seq,features_on_anchor=count,motion=r['motion'],path_points=len(r['path']),rejections=r['rejections'],anchor_age=None if anchor is None else float(t-anchor_t)))
  if tracker.memory_gray is not anchor:anchor_t=t
 (OUT/'diagnostic.json').write_text(json.dumps(rows,indent=2),encoding='utf8')
 print(json.dumps(rows[-12:],indent=2))
