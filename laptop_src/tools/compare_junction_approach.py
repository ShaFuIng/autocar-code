import sys,json,time,copy,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'laptop_workbench/runtimes/opencv-4.11'))
from analyze_junction_4396573e import load_tunnel,cv2,np
sys.path.insert(0,str(ROOT/'web/k_lane_runtime'))
from junction_k_lane import KLanePath,lane_evidence
from junction_approach_candidate import ApproachGate
from k_adaptive_pid import KAdaptivePID
OUT=ROOT/'laptop_workbench/data/experiments/k-approach';OUT.mkdir(parents=True,exist_ok=True)
cv2.setNumThreads(2)
job=ROOT/'laptop_workbench/data/analysis/f7e170a0869d4fdd8632575997f39a4a'
saved=json.loads((job/'result.json').read_text(encoding='utf8'))
live,params=load_tunnel();assert params['max_gap']==.4
def fullclip():
 for i,f in enumerate(saved['frames']):
  im=cv2.imdecode(np.fromfile(str(job/'raw'/f'{i:06d}.jpg'),np.uint8),1)
  mask=cv2.imdecode(np.fromfile(str(job/'C'/f'{i:06d}.png'),np.uint8),0)>0
  yield i,f['capture_s'],cv2.resize(im,(640,360)),mask,None,None
results={};tiles=[]
for name,items in [('latest',live),('fullclip',fullclip())]:
 a=KLanePath(.6);b=KLanePath(.6);b.gate=ApproachGate();pids=[KAdaptivePID(params),KAdaptivePID(params)];rows=[]
 for seq,t,im,mask,_,meta in items:
  # Geometry is gate-independent. Sharing the same computations removes replay
  # noise and ensures this experiment changes only switching, not the paths.
  base=a.base.step(im,mask,t);v=a.visible.step(im);d=lane_evidence(mask,base,v);d['capture_s']=t
  early=b.gate.can_publish_early();pair=[]
  for tr,pid in zip([a,b],pids):
   start=time.perf_counter();r=tr.combine(base,v,mask,copy.deepcopy(d));ms=(time.perf_counter()-start)*1000
   p=pid.step_path(t,r['path'] if r['valid'] else [])
   pair.append(dict(mode=r['mode'],source=r['k']['source'],reason=r['reason'],gate=r['k']['mask_gate'],path=r['path'],valid=r['valid'],pid=p,gate_ms=ms))
  if early:assert pair[1]['source']=='A',(name,seq,'early A would differ')
  rows.append(dict(seq=seq,captured=t,baseline=pair[0],candidate=pair[1],candidate_early_allowed=early))
  if name=='latest' and seq in [70,75,85,100,120,139]:
   images=[]
   for label,r in zip(['BASELINE','APPROACH'],pair):
    vis=im.copy();vis[mask]=(vis[mask]*.65+np.array([255,0,255])*.35).astype('uint8')
    if r['path']:cv2.polylines(vis,[np.int32(np.array(r['path'])/2)],False,(255,200,0),2)
    cv2.putText(vis,f'{label} seq{seq} {r["mode"]}',(10,22),0,.55,(0,255,255),2);images.append(vis)
   tiles.append(np.hstack(images))
  if seq%200==0:print(name,seq,flush=True)
 results[name]=rows
 (OUT/(name+'.json')).write_text(json.dumps(rows),encoding='utf8')
def summarize(rows):
 def transitions(key):
  last=None;out=[]
  for x in rows:
   state=x[key]['mode']
   if state!=last:out.append([x['seq'],state,x[key]['reason']]);last=state
  return out
 return dict(frames=len(rows),baseline_modes=transitions('baseline'),candidate_modes=transitions('candidate'),baseline_empty=sum(not x['baseline']['valid'] for x in rows),candidate_empty=sum(not x['candidate']['valid'] for x in rows),path_changes=sum(x['baseline']['path']!=x['candidate']['path'] for x in rows))
summary={'latest':summarize(results['latest'])}
for name,lo,hi in [('intersection1',640,820),('intersection2',1010,1200),('curve23',210,260),('adjacent97',940,1000),('curve142',1390,1450)]:
 summary[name]=summarize([x for x in results['fullclip'] if lo<=x['seq']<=hi])
(OUT/'report.json').write_text(json.dumps(dict(status='offline_only_not_deployed',summary=summary,max_gap=.4,opencv=cv2.__version__,notes=['Latest replay uses original masks and nearest AVI frames; pre-run preview state is not recorded','Full original clip replay preserves preceding state for target intervals','No tunnel candidate included; only gate logic changes']),indent=2),encoding='utf8')
cv2.imencode('.jpg',np.vstack(tiles))[1].tofile(str(OUT/'comparison.jpg'))
before=OUT/'baseline';before.mkdir(exist_ok=True)
for f in ['junction_k_lane.py','road_segments_v32.py']:
 if not (before/f).exists():shutil.copy2(ROOT/'web/k_lane_runtime'/f,before/f)
shutil.copy2(ROOT/'tools/junction_approach_candidate.py',OUT/'candidate.py')
print(json.dumps(summary),flush=True)
