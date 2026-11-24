import json,base64,subprocess
from pathlib import Path
OUT=Path(__file__).resolve().parents[1]/'laptop_workbench/data/experiments/k-latency/run-3b428'
SCRIPT=r"""
import sys,json,time,gc,contextlib,cProfile,pstats
from pathlib import Path
sys.path.insert(0,'/web')
import cv2,numpy as np
from live_model_worker import ADAPTERS
from k_lane_live_path import KLaneLivePath
import junction_k as k,temporal_lane as temporal,road_segments_v32 as road,junction_k_lane as lane
cv2.setNumThreads(2)
spec=json.loads(Path('/web/pid_models.json').read_text())['v4-default']
with contextlib.redirect_stdout(sys.stderr):model=ADAPTERS[spec['adapter']](Path(spec['weights']))
meta=[json.loads(s) for s in Path('/run/frames_001.jsonl').read_text().splitlines()]
cap=cv2.VideoCapture('/run/video_001.avi');frames=[]
for row in meta:
 ok,im=cap.read()
 if not ok:break
 im=cv2.resize(im,(640,360))
 with contextlib.redirect_stdout(sys.stderr):mask=model.predict(im)
 frames.append((im,mask,row['capture_monotonic']))
cap.release();del model;gc.collect();gc.freeze()
measure={};counts={};in_detect=False
orig_hough=cv2.HoughLinesP
def hough(*a,**kw):
 out=orig_hough(*a,**kw)
 if in_detect:counts['hough_segments']=0 if out is None else len(out)
 return out
cv2.HoughLinesP=hough
orig_detect=k.detect
def detect(*a,**kw):
 global in_detect
 in_detect=True
 try:
  out=orig_detect(*a,**kw);counts['left_candidates']=len(out[1][0]);counts['right_candidates']=len(out[1][1]);return out
 finally:in_detect=False
k.detect=detect
origfit=np.polyfit
def fit(*a,**kw):
 counts['polyfit_calls']=counts.get('polyfit_calls',0)+1
 return origfit(*a,**kw)
np.polyfit=fit
def wrap(obj,name,label):
 orig=getattr(obj,name)
 def wrapped(*a,**kw):
  t=time.perf_counter()
  try:
   result=orig(*a,**kw)
   if label=='k_motion_total':counts['k_motion_diagnostic']=result[1]
   return result
  finally:measure[label]=measure.get(label,0)+(time.perf_counter()-t)*1000
 setattr(obj,name,wrapped)
for obj,name,label in [(k,'detect','white_detect'),(temporal,'_estimate_motion','optical_flow_uncached'),(k,'motion','k_motion_total'),(road.RoadPath,'step','A_total'),(k.VisibleTracker,'step','K_visible_total'),(lane,'lane_evidence','lane_pairing'),(lane,'evidence','handoff_evidence'),(lane.KLanePath,'combine','combine_total')]:wrap(obj,name,label)
rows=[]
for repeat in range(1):
 tracker=KLaneLivePath(.6)
 for i,(im,m,timestamp) in enumerate(frames):
  measure={};counts={};tick=time.perf_counter();result=tracker.step(im,m,timestamp);total=(time.perf_counter()-tick)*1000
  rows.append(dict(repeat=repeat,record_frame=i,capture_at=timestamp,total_ms=total,stages=measure,counts=counts,cache_hits=temporal._motion_cache_hits,mode=result['path_diagnostic']['mode'],valid=result['valid']))
print(json.dumps(dict(rows=rows,frame_count=len(frames),temporal_source=temporal.__file__,limitation='Recorded AVI at lower sample rate, JPEG/codec differs and pre-run tracker history is unavailable; not exact original inference frame replay.')))
"""
REMOTE=r"""
import json,sys,pathlib,subprocess,socket,urllib.request,base64,os
p=json.load(sys.stdin);root=pathlib.Path('/home/jetson/orin_interface_v1');secret=(pathlib.Path.home()/'.config/orin_lab/token').read_text().strip();headers={'Authorization':'Basic '+base64.b64encode(('jetson:'+secret).encode()).decode()}
for h in ['127.0.0.1']+subprocess.check_output(['hostname','-I'],text=True).split():
 if ':' in h:continue
 try:
  req=urllib.request.Request('http://'+h+':8000/api/pid/status',headers=headers)
  with urllib.request.urlopen(req,timeout=2) as r:status=json.load(r)
  break
 except OSError:pass
assert not status.get('run') or status['run']['state']!='running','PID running; no replay started'
spec=json.loads((root/'web/pid_models.json').read_text())['v4-default'];weightdir=str(pathlib.Path(spec['weights']).parent)
cmd=['docker','run','--rm','-i','--network','none','--runtime','nvidia','--user',str(os.getuid())+':'+str(os.getgid()),'--cap-drop','ALL','--security-opt','no-new-privileges:true','--memory','4g','-e','OPENBLAS_NUM_THREADS=1','-e','OMP_NUM_THREADS=1','-e','USER=jetson','-e','LOGNAME=jetson','-e','TORCHINDUCTOR_CACHE_DIR=/tmp/torch_cache','-e','HOME=/tmp','-e','YOLO_CONFIG_DIR=/tmp/yolo','-v',str(root/'web')+':/web:ro','-v',weightdir+':'+weightdir+':ro','-v',str(root/'logs/manual_driving/3b428ba5375b4984a4f36c70a11230ae')+':/run:ro','orin-notebook:v1','python3','-']
r=subprocess.run(cmd,input=p['script'],capture_output=True,text=True,timeout=180);print(r.stderr,file=sys.stderr);assert r.returncode==0;rj=json.loads(r.stdout);rj['preview_state_at_start']=status['state'];print(json.dumps(rj))
"""
args=['ssh','-i','C:/Users/User/.ssh/codex_orin_ed25519','-o','BatchMode=yes','-o','ConnectTimeout=6','-o','StrictHostKeyChecking=yes','-o','HostKeyAlias=orinnano-super.local','jetson@192.168.0.244',"python3 -c \"import base64;exec(base64.b64decode('"+base64.b64encode(REMOTE.encode()).decode()+"'))\""]
r=subprocess.run(args,input=json.dumps({'script':SCRIPT}),capture_output=True,text=True,encoding='utf-8',timeout=210);print(r.stderr);assert r.returncode==0,r.stdout
(OUT/'replay-motion-detail.json').write_text(r.stdout,encoding='utf-8');d=json.loads(r.stdout);print(json.dumps({k:v for k,v in d.items() if k!='rows'}));print(json.dumps([r for r in d['rows'] if r['record_frame']>=39]))
