"""Exercise real worker stdio loop with saved masks, never a model/device."""
from pathlib import Path
import sys,json,io,contextlib,base64,time
ROOT=Path(__file__).resolve().parents[1];W=ROOT/'laptop_workbench';OUT=W/'data/experiments/k-live-prep'
sys.path[:0]=[str(W/'runtimes/opencv-4.11'),str(ROOT/'web'),str(W)]
import cv2,numpy as np
import live_model_worker as worker
from pid_paths import selected,PATHS,K_FILES
old={};exec((OUT/'baseline/pid_paths.py').read_text(),old)
assert {k:v for k,v in PATHS.items() if k!='junction-k-v1'}==old['PATHS']
assert selected('junction-k-v1')['preview']
try:selected('junction-k-v1',control=True)
except ValueError:pass
else:raise AssertionError('K must remain disconnected from real PID')
source=W/'data/experiments/orin-k/input';manifest=json.loads((source/'manifest.json').read_text());checks=[]
config=OUT/'fake-model-config.json';config.write_text(json.dumps({'offline-mask':{'weights':str(OUT/'baseline/pid_paths.py'),'adapter':'saved-mask'}}))
for case in manifest['cases']:
 class SavedMasks:
  def __init__(self,weights):self.i=-1
  def predict(self,image):
   self.i+=1
   if self.i==0:return np.zeros(image.shape[:2],bool)
   return cv2.imdecode(np.fromfile(str(source/case['frames'][self.i-1]['mask']),np.uint8),0)>0
 worker.ADAPTERS['saved-mask']=SavedMasks
 messages=[dict(seq=i,jpeg=base64.b64encode((source/f['image']).read_bytes()).decode(),row_ratio=.6,path_algorithm='junction-k-v1',captured=f['capture_s']) for i,f in enumerate(case['frames'])]
 stream=io.StringIO();previous=sys.stdin;argv=sys.argv;sys.stdin=io.StringIO(''.join(json.dumps(m)+'\n' for m in messages));sys.argv=['worker',str(config),'offline-mask']
 try:
  with contextlib.redirect_stdout(stream):worker.main()
 finally:sys.stdin=previous;sys.argv=argv
 lines=[json.loads(x) for x in stream.getvalue().splitlines()];assert lines[0]['ready'] and lines[0]['opencv']=='4.11.0'
 expected=json.loads((W/'data/experiments/orin-k-stable/candidate/laptop'/f"{case['id']}.json").read_text())['rows']
 assert len(lines)==len(expected)+1
 for i,(r,b) in enumerate(zip(lines[1:],expected)):
  assert r['seq']==i and r['path_algorithm']=='junction-k-v1'
  assert r['path']==b['path'] and r['path_diagnostic']['k']['state']==b['k']['state']
  assert r['path_diagnostic']['capture_s']==messages[i]['captured']
  assert r['mask_png'] and r['preview'] and not r['path_diagnostic']['control_connected']
 checks.append(dict(case=case['id'],frames=len(expected),protocol_parity=True,mode_parity=True,path_parity=True,mean_worker_ms=sum(r['inference_ms'] for r in lines[1:])/len(expected)))
 print('PASS',checks[-1],flush=True)
report=dict(state='prepared_not_deployed',checks=checks,original_registry_preserved=True,real_pid_connected=False,model_inference=False,orin_connected=False,service_restarted=False,hardware_output=False,opencv=cv2.__version__,limitations=['Saved masks replace V4 only for offline protocol verification','Laptop worker time excludes neural inference and is not Orin control latency','K counters were tested at 10fps; real capture cadence and closed-loop timing require checking on Orin'])
(OUT/'entry-validation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
