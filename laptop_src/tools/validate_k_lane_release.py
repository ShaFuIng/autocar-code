"""Validate the frozen deployment payload via real worker stdio, saved masks only."""
from pathlib import Path
import sys,json,contextlib,base64,hashlib
ROOT=Path(__file__).resolve().parents[1];W=ROOT/'laptop_workbench';OUT=W/'data/experiments/k-lane-release';STAGE=OUT/'payload'
sys.path[:0]=[str(W/'runtimes/opencv-4.11'),str(STAGE/'web'),str(STAGE/'laptop_workbench')]
import cv2,numpy as np
import live_model_worker as worker
from pid_paths import PATHS,selected,K_LANE_FILES
old={};exec((OUT/'laptop-baseline/web/pid_paths.py').read_text(encoding='utf-8'),old)
assert {k:v for k,v in PATHS.items() if k!='junction-k-lane-v3'}==old['PATHS']
assert selected('junction-k-lane-v3')['preview']
try:selected('junction-k-lane-v3',control=True)
except ValueError:pass
else:raise AssertionError('Candidate must not connect real PID')
assert all((STAGE/'laptop_workbench'/n).exists() for n in K_LANE_FILES)
for p in STAGE.rglob('*.py'):compile(p.read_text(encoding='utf-8'),str(p),'exec')
job=W/'data/analysis/f7e170a0869d4fdd8632575997f39a4a';r=json.loads((job/'result.json').read_text());expected=r['models']['C'];frames=r['frames']
class SavedMasks:
 def __init__(self,weights):self.i=-1
 def predict(self,image):
  self.i+=1
  return np.zeros(image.shape[:2],bool) if self.i==0 else cv2.imdecode(np.fromfile(str(job/'C'/f'{self.i-1:06d}.png'),np.uint8),0)>0
worker.ADAPTERS['saved-mask']=SavedMasks
config=OUT/'validation-model.json';config.write_text(json.dumps({'offline-mask':{'weights':str(STAGE/'web/pid_paths.py'),'adapter':'saved-mask'}}))
class Inputs:
 def __iter__(self):
  for i,f in enumerate(frames):yield json.dumps(dict(seq=i,jpeg=base64.b64encode((job/'raw'/f'{i:06d}.jpg').read_bytes()).decode(),row_ratio=.6,path_algorithm='junction-k-lane-v3',captured=f['capture_s']))+'\n'
class Replies:
 def __init__(self):self.buffer='';self.i=-1;self.ms=[]
 def write(self,text):
  self.buffer+=text
  while '\n' in self.buffer:
   line,self.buffer=self.buffer.split('\n',1)
   if not line:continue
   x=json.loads(line)
   if self.i==-1:assert x['ready'] and x['opencv']=='4.11.0' and x['k_lane_revision']=='k-shared-lane-test-r1'
   else:
    b=expected[self.i]['A'];assert x['seq']==self.i and x['path']==b['path'] and x['path_diagnostic']['k']['state']==b['k']['state'],self.i
    assert x['path_diagnostic']['capture_s']==frames[self.i]['capture_s'] and not x['path_diagnostic']['control_connected']
    assert x['mask_png'] and x['preview'];self.ms.append(x['inference_ms'])
    if self.i%400==0:print('verified',self.i,file=sys.__stdout__,flush=True)
   self.i+=1
  return len(text)
 def flush(self):pass
sink=Replies();before=sys.stdin;argv=sys.argv;sys.stdin=Inputs();sys.argv=['worker',str(config),'offline-mask']
try:
 with contextlib.redirect_stdout(sink):worker.main()
finally:sys.stdin=before;sys.argv=argv
assert sink.i==len(frames)
report=dict(release='k-shared-lane-test-r1',state='prepared_not_deployed',frames=len(frames),worker_protocol_parity=True,path_and_mode_parity=True,original_registry_preserved=True,preview_only=True,real_pid_connected=False,orin_connected=False,hardware_output=False,model_inference=False,opencv=cv2.__version__,numpy=np.__version__,mean_saved_mask_worker_ms=sum(sink.ms)/len(sink.ms))
(OUT/'validation.json').write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps(report),flush=True)
