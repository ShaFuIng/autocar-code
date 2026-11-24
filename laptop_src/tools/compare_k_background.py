import json,subprocess,base64
from pathlib import Path
OUT=Path(__file__).resolve().parents[1]/'laptop_workbench/data/experiments/k-background'
SCRIPT=r"""
import sys,json,time,gc,contextlib
from pathlib import Path
sys.path.insert(0,'/web')
import cv2,numpy as np
from k_background_candidate import AsyncKLane
from junction_k_lane import KLanePath
from live_model_worker import ADAPTERS
cv2.setNumThreads(2)
frames=json.loads(Path('/input/frames.json').read_text());cases={}
for name,lo,hi in [('intersection1',640,820),('intersection2',1010,1200)]:
 cases[name]=[(f['index'],f['capture_s'],cv2.imread('/input/raw/%06d.jpg'%f['index']),cv2.imread('/input/mask/%06d.png'%f['index'],0)>0) for f in frames if lo<=f['index']<=hi]
spec=json.loads(Path('/web/pid_models.json').read_text())['v4-default']
with contextlib.redirect_stdout(sys.stderr):model=ADAPTERS[spec['adapter']](Path(spec['weights']))
cap=cv2.VideoCapture('/run/video_001.avi');metadata=[json.loads(s) for s in Path('/run/frames_001.jsonl').read_text().splitlines()];run=[]
for f in metadata:
 ok,im=cap.read()
 if not ok:break
 im=cv2.resize(im,(640,360))
 with contextlib.redirect_stdout(sys.stderr):mask=model.predict(im)
 run.append((f['frame'],f['capture_monotonic'],im,mask))
cap.release();del model;cases['stopped_run']=run;gc.collect();gc.freeze()
outputs={}
for name,items in cases.items():
 outputs[name]={}
 for variant in ['baseline','background']:
  tracker=KLanePath(.6) if variant=='baseline' else AsyncKLane(.6)
  rows=[];begin=time.perf_counter();cpu=time.process_time();first=items[0][1]
  for i,captured,im,mask in items:
   remaining=begin+(captured-first)-time.perf_counter()
   if remaining>0:time.sleep(remaining)
   start=time.perf_counter();r=tracker.step(im,mask,captured);elapsed=(time.perf_counter()-start)*1000
   rows.append(dict(index=i,captured=captured,ms=elapsed,mode=r['mode'],source=r['k']['source'],path=r['path'],valid=r['valid'],reason=r['reason'],background=r.get('background')))
  if variant=='background':tracker.close()
  cpu_ms=(time.process_time()-cpu)*1000
  outputs[name][variant]=dict(rows=rows,cpu_ms=cpu_ms,foreground_ms=sum(x['ms'] for x in rows),background_ms=tracker.background.ms if variant=='background' else 0)
  print(name,variant,len(rows),round(cpu_ms),file=sys.stderr,flush=True)
Path('/output/comparison-full.json').write_text(json.dumps(outputs))
summary={}
def transitions(rows):
 last=None;out=[]
 for r in rows:
  if r['mode']!=last:out.append([r['index'],r['mode']]);last=r['mode']
 return out
for name,pair in outputs.items():
 b=pair['baseline'];c=pair['background'];summary[name]=dict(frames=len(b['rows']),baseline_mean_ms=np.mean([r['ms'] for r in b['rows']]),background_mean_ms=np.mean([r['ms'] for r in c['rows']]),baseline_cpu_ms=b['cpu_ms'],background_cpu_ms=c['cpu_ms'],baseline_modes=transitions(b['rows']),background_modes=transitions(c['rows']),baseline_empty=sum(not r['valid'] for r in b['rows']),background_empty=sum(not r['valid'] for r in c['rows']),background_errors=list(set(r['background']['error'] for r in c['rows'] if r['background']['error'])))
print(json.dumps(summary))
"""
REMOTE=r"""
import sys,json,pathlib,subprocess,base64,os,shutil,socket,urllib.request
p=json.load(sys.stdin);root=pathlib.Path('/home/jetson/orin_interface_v1');secret=(pathlib.Path.home()/'.config/orin_lab/token').read_text().strip();headers={'Authorization':'Basic '+base64.b64encode(('jetson:'+secret).encode()).decode()}
for host in ['127.0.0.1']+subprocess.check_output(['hostname','-I'],text=True).split():
 if ':' in host:continue
 try:
  with urllib.request.urlopen(urllib.request.Request('http://'+host+':8000/api/pid/status',headers=headers),timeout=2) as r:status=json.load(r)
  break
 except OSError:pass
assert not status.get('run') or status['run']['state']!='running','PID active; no replay started'
work=root/'offline_experiments/k_background_20261024';work.mkdir(exist_ok=True,parents=True)
shutil.copytree(root/'web',work/'web',dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
(work/'web/k_background_candidate.py').write_text(p['candidate']);(work/'compare.py').write_text(p['script'])
spec=json.loads((root/'web/pid_models.json').read_text())['v4-default'];weights=str(pathlib.Path(spec['weights']).parent)
cmd=['docker','run','--rm','--network','none','--runtime','nvidia','--user',str(os.getuid())+':'+str(os.getgid()),'--cap-drop','ALL','--security-opt','no-new-privileges:true','--memory','4g','-e','USER=jetson','-e','LOGNAME=jetson','-e','HOME=/tmp','-e','TORCHINDUCTOR_CACHE_DIR=/tmp/torch','-e','OPENBLAS_NUM_THREADS=1','-e','OMP_NUM_THREADS=1','-v',str(work/'web')+':/web:ro','-v',str(work)+':/output','-v',weights+':'+weights+':ro','-v',str(root/'releases/k-shared-lane-test-r1/input')+':/input:ro','-v',str(root/'logs/manual_driving/3b428ba5375b4984a4f36c70a11230ae')+':/run:ro','orin-notebook:v1','python3','/output/compare.py']
r=subprocess.run(cmd,capture_output=True,text=True,timeout=240);print(r.stderr,file=sys.stderr);assert r.returncode==0,r.stdout
print(r.stdout)
"""
args=['ssh','-i','C:/Users/User/.ssh/codex_orin_ed25519','-o','BatchMode=yes','-o','ConnectTimeout=6','-o','StrictHostKeyChecking=yes','-o','HostKeyAlias=orinnano-super.local','jetson@192.168.0.244',"python3 -c \"import base64;exec(base64.b64decode('"+base64.b64encode(REMOTE.encode()).decode()+"'))\""]
r=subprocess.run(args,input=json.dumps(dict(script=SCRIPT,candidate=(OUT/'payload/web/k_background_candidate.py').read_text())),capture_output=True,text=True,encoding='utf-8',timeout=260);(OUT/'execution.log').write_text(r.stderr,encoding='utf-8');print(r.stderr);print(r.stdout);assert r.returncode==0
(OUT/'comparison.json').write_text(r.stdout,encoding='utf-8')
