"""Same masks/images, full K gate and adaptive PID, laptop only."""
import sys,json,time,shutil,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'laptop_workbench/runtimes/opencv-4.11'))
from analyze_tunnel_reference import *
from junction_k_lane import KLanePath
from tunnel_reference_candidate import RoadPath as Candidate
from k_adaptive_pid import KAdaptivePID
cv2.setNumThreads(2)
job=ROOT/'laptop_workbench/data/analysis/f7e170a0869d4fdd8632575997f39a4a'
saved=json.loads((job/'result.json').read_text(encoding='utf8'))
tunnel,params=load_tunnel();assert params['max_gap']==.4
def intersection(lo,hi):
 for i in range(lo,hi+1):
  im=cv2.imdecode(np.fromfile(str(job/'raw'/f'{i:06d}.jpg'),np.uint8),1)
  mask=cv2.imdecode(np.fromfile(str(job/'C'/f'{i:06d}.png'),np.uint8),0)>0
  yield i,saved['frames'][i]['capture_s'],cv2.resize(im,(640,360)),mask,None,None
summary={};outputs={};tiles=[]
for name,items in [('tunnel',tunnel),('intersection1',intersection(640,820)),('intersection2',intersection(1010,1200))]:
 a=KLanePath(.6);b=KLanePath(.6);b.base=Candidate('road-entrance-v3.2',None,.6)
 pa=KAdaptivePID(params);pb=KAdaptivePID(params);rows=[]
 for seq,t,im,mask,live,meta in items:
  outputs_pair=[]
  for tracker,pid in [(a,pa),(b,pb)]:
   start=time.perf_counter();r=tracker.step(im,mask,t);ms=(time.perf_counter()-start)*1000
   p=pid.step_path(t,r['path'] if r['valid'] else [])
   outputs_pair.append(dict(path=r['path'],valid=r['valid'],mode=r['mode'],pid=p,motion=r['motion'],recovery=r.get('far_recovery'),ms=ms))
  row=dict(seq=seq,captured=t,baseline=outputs_pair[0],candidate=outputs_pair[1]);rows.append(row)
  if name=='tunnel' and seq in [420,426,428,429,430]:
   pair=[]
   for label,r in zip(['BASELINE','CANDIDATE'],outputs_pair):
    vis=im.copy();vis[mask]=(vis[mask]*.65+np.array([255,0,255])*.35).astype('uint8')
    rec=r.get('recovery') or {}
    for edge in rec.get('pixel_edges',[]):
     if len(edge)>1:cv2.polylines(vis,[np.int32(edge)],False,(0,165,255),2)
    if r['path']:cv2.polylines(vis,[np.int32(np.array(r['path'])/2)],False,(255,200,0),2)
    cv2.putText(vis,f'{label} seq {seq} valid={r["pid"]["valid"]}',(10,20),0,.55,(0,255,255),2)
    pair.append(vis)
   tiles.append(np.hstack(pair))
 changes=[x for x in rows if x['baseline']['path']!=x['candidate']['path']]
 summary[name]=dict(frames=len(rows),path_changes=len(changes),mode_changes=sum(x['baseline']['mode']!=x['candidate']['mode'] for x in rows),
  accepted=[x['seq'] for x in rows if (x['candidate']['recovery'] or {}).get('accepted')],
  baseline_empty=sum(not x['baseline']['valid'] for x in rows),candidate_empty=sum(not x['candidate']['valid'] for x in rows),
  baseline_pid_invalid=sum(not x['baseline']['pid']['valid'] for x in rows),candidate_pid_invalid=sum(not x['candidate']['pid']['valid'] for x in rows),
  mean_ms={k:float(np.mean([x[k]['ms'] for x in rows])) for k in ['baseline','candidate']})
 outputs[name]=rows;print(name,summary[name],flush=True)
(OUT/'comparison.json').write_text(json.dumps(outputs,indent=2),encoding='utf8')
(OUT/'report.json').write_text(json.dumps(dict(status='offline_candidate_not_deployed',opencv=cv2.__version__,summary=summary,limits=['Frozen live masks with nearest AVI frames (not bit-identical camera frames)','No model retraining or mask modification','No Orin deployment or hardware actions','Image contrast evidence is not human ground truth'],max_gap=.4),indent=2),encoding='utf8')
cv2.imencode('.jpg',np.vstack(tiles))[1].tofile(str(OUT/'comparison.jpg'))
before=OUT/'baseline';before.mkdir(exist_ok=True)
for f in ['road_segments_v32.py','temporal_lane.py','junction_k_lane.py']:
 target=before/f
 if not target.exists():shutil.copy2(ROOT/'web/k_lane_runtime'/f,target)
shutil.copy2(ROOT/'tools/tunnel_reference_candidate.py',OUT/'candidate.py')
