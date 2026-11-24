import json,base64,subprocess
from pathlib import Path
OUT=Path(__file__).resolve().parents[1]/'laptop_workbench/data/experiments/k-latency'
SCRIPT=r"""
import sys,json,time,gc
from pathlib import Path
sys.path.insert(0,'/web')
import cv2,numpy as np
from k_lane_live_path import KLaneLivePath
cv2.setNumThreads(2);gc.collect();gc.freeze();rows=[];tracker=None;last=None
for f in json.loads(Path('/input/frames.json').read_text()):
 i=f['index']
 if not (640<=i<=820 or 1010<=i<=1200):continue
 if last is None or i!=last+1:tracker=KLaneLivePath(.6)
 last=i;im=cv2.imread('/input/raw/%06d.jpg'%i);m=cv2.imread('/input/mask/%06d.png'%i,0)>0
 t=time.perf_counter();r=tracker.step(im,m,f['capture_s']);elapsed=(time.perf_counter()-t)*1000
 import temporal_lane
 rows.append(dict(index=i,path=r['path'],valid=r['valid'],quality=r['quality'],diagnostic=r['path_diagnostic'],ms=elapsed,cache_hits=getattr(temporal_lane,'_motion_cache_hits',0)))
Path('/output/'+sys.argv[1]+'.json').write_text(json.dumps(rows));print(sys.argv[1],len(rows),flush=True)
"""
REMOTE=r"""
import json,sys,base64,pathlib,shutil,subprocess,os
p=json.load(sys.stdin);root=pathlib.Path('/home/jetson/orin_interface_v1');work=root/'offline_experiments/k_latency_20261024';work.mkdir(exist_ok=True,parents=True)
for variant in ['before','candidate']:
 target=work/variant
 shutil.copytree(root/'web',target,dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
 if variant=='candidate':
  for name,data in p['files'].items():
   dest=target/name;dest.parent.mkdir(exist_ok=True,parents=True);dest.write_bytes(base64.b64decode(data))
(work/'compare.py').write_text(p['script'])
for variant in ['before','candidate']:
 cmd=['docker','run','--user',str(os.getuid())+':'+str(os.getgid()),'--rm','--network','none','--cap-drop','ALL','-e','OPENBLAS_NUM_THREADS=1','-e','OMP_NUM_THREADS=1','-v',str(work/variant)+':/web:ro','-v',str(root/'releases/k-shared-lane-test-r1/input')+':/input:ro','-v',str(work)+':/output','orin-notebook:v1','python3','/output/compare.py',variant]
 subprocess.run(cmd,check=True,stdout=sys.stderr)
a=json.loads((work/'before.json').read_text());b=json.loads((work/'candidate.json').read_text());diff=[x['index'] for x,y in zip(a,b) if any(x[k]!=y[k] for k in ['path','valid','quality','diagnostic'])]
import statistics
print(json.dumps(dict(frames=len(a),different=diff,before_ms=statistics.mean(x['ms'] for x in a),candidate_ms=statistics.mean(x['ms'] for x in b),cache_hits=sum(x['cache_hits'] for x in b),remote=str(work))))
"""
files={str(p.relative_to(OUT/'payload/web')).replace('\\','/'):base64.b64encode(p.read_bytes()).decode() for p in (OUT/'payload/web').rglob('*.py')}
args=['ssh','-i','C:/Users/User/.ssh/codex_orin_ed25519','-o','BatchMode=yes','-o','ConnectTimeout=6','-o','StrictHostKeyChecking=yes','-o','HostKeyAlias=orinnano-super.local','jetson@192.168.0.244',"python3 -c \"import base64;exec(base64.b64decode('"+base64.b64encode(REMOTE.encode()).decode()+"'))\""]
r=subprocess.run(args,input=json.dumps(dict(files=files,script=SCRIPT)),capture_output=True,text=True,encoding='utf-8',timeout=300);print(r.stderr);print(r.stdout);assert r.returncode==0
(OUT/'comparison.json').write_text(r.stdout,encoding='utf-8')
