import sys,json,time,copy,base64,gzip,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'laptop_workbench/runtimes/opencv-4.11'));sys.path.insert(0,str(ROOT/'web'));sys.path.insert(0,str(ROOT/'web/k_lane_runtime'))
import cv2,numpy as np
from junction_k_lane import KLanePath,lane_evidence
from junction_approach_candidate import ApproachGate
from junction_branch_candidate import TemporalBranchGate,augment,guard_single
OUT=ROOT/'laptop_workbench/data/experiments/k-branch-temporal';OUT.mkdir(exist_ok=True)
cv2.setNumThreads(2)
RUN=ROOT.parent/'firmware_builds/i2c_retry_v3/run-0d77c3ca/0d77c3ca05284af9b8e5ad4d0ef81d5c'
def latest():
 logs=[json.loads(l) for l in (RUN/'pid.jsonl').read_text().splitlines()];frames=[json.loads(l) for l in (RUN/'frames_001.jsonl').read_text().splitlines()];cap=cv2.VideoCapture(str(RUN/'video_001.avi'))
 for x in logs:
  v=x.get('vision',{})
  if not v.get('mask_png'):continue
  f=min(frames,key=lambda f:abs(f['capture_monotonic']-v['capture_at']));cap.set(1,f['frame']);ok,im=cap.read();assert ok
  yield v['seq'],v['capture_at'],cv2.resize(im,(640,360)),cv2.imdecode(np.frombuffer(base64.b64decode(v['mask_png']),np.uint8),0)>0
job=ROOT/'laptop_workbench/data/analysis/f7e170a0869d4fdd8632575997f39a4a'
def fullclip():
 saved=json.loads((job/'result.json').read_text(encoding='utf8'))
 for i,f in enumerate(saved['frames']):
  im=cv2.imdecode(np.fromfile(str(job/'raw'/f'{i:06d}.jpg'),np.uint8),1);mask=cv2.imdecode(np.fromfile(str(job/'C'/f'{i:06d}.png'),np.uint8),0)>0
  yield i,f['capture_s'],cv2.resize(im,(640,360)),mask
for name,loader in [('latest',latest),('fullclip',fullclip)]:
 cache=OUT/(name+'-geometry.jsonl.gz');rows=[];tr=KLanePath();baseline=KLanePath();baseline.gate=ApproachGate();candidate=KLanePath();candidate.gate=TemporalBranchGate()
 if cache.exists():
  def items():
   for line in gzip.open(cache,'rt',encoding='utf8'):
    c=json.loads(line);yield c['seq'],c['t'],None,None,c
  iterable=items();writer=None
 else:iterable=((seq,t,im,mask,None) for seq,t,im,mask in loader());writer=gzip.open(cache,'wt',encoding='utf8')
 for seq,t,im,mask,c in iterable:
  if c is None:
   a=tr.base.step(im,mask,t);v=tr.visible.step(im);d=lane_evidence(mask,a,v);d['capture_s']=t;d=augment(d,mask,a)
   c=dict(seq=seq,t=t,a=a,v=v,d=d)
   writer.write(json.dumps(c,default=lambda o:o.tolist() if hasattr(o,'tolist') else o.item())+'\n')
  else:a,v,d=c['a'],c['v'],c['d']
  b=baseline.combine(a,v,mask,copy.deepcopy(d));ga=guard_single(a,candidate.gate);r=candidate.combine(ga,v,mask,copy.deepcopy(d))
  rows.append(dict(seq=seq,t=t,baseline={k:b[k] for k in ['mode','reason','path','valid','k']},candidate={k:r[k] for k in ['mode','reason','path','valid','k']},guard=ga.get('single_guard')))
  if im is not None and ((name=='latest' and seq in [440,460,470,480,500,520]) or (name=='fullclip' and seq in [234,973,1420,746,1092])):
   cv2.imencode('.jpg',im)[1].tofile(str(OUT/f'{name}-{seq}.jpg'))
  if seq%200==0:print(name,seq,flush=True)
 if writer:writer.close()
 (OUT/(name+'.json')).write_text(json.dumps(rows),encoding='utf8')
 def summary(rs):
  out={'frames':len(rs)}
  for key in ['baseline','candidate']:
   transitions=[];last=None
   for x in rs:
    if x[key]['mode']!=last:transitions.append([x['seq'],x[key]['mode'],x[key]['reason']]);last=x[key]['mode']
   out[key]={'transitions':transitions,'empty':sum(not x[key]['valid'] for x in rs),'cross_frames':sum(x[key]['mode']=='KL_CROSS' for x in rs)}
  return out
 sections={'all':summary(rows)}
 for label,lo,hi in ([('second_entry',410,540)] if name=='latest' else [('intersection1',640,820),('intersection2',1010,1200),('curve23',210,260),('adjacent97',940,1000),('curve142',1390,1450)]):sections[label]=summary([x for x in rows if lo<=x['seq']<=hi])
 (OUT/(name+'-summary.json')).write_text(json.dumps(sections,indent=2));print(name,json.dumps(sections),flush=True)
