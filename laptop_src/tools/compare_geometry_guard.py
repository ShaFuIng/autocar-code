import sys,json,gzip,base64,copy
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'laptop_workbench/runtimes/opencv-4.11'),str(ROOT/'web'),str(ROOT/'web/k_lane_runtime')]
import cv2,numpy as np
from junction_k_lane import KLanePath,lane_evidence
from junction_branch_candidate import TemporalBranchGate,augment,guard_single
from junction_geometry_guard import geometry_guard
from k_adaptive_pid import KAdaptivePID
SRC=ROOT/'laptop_workbench/data/experiments/k-branch-temporal';OUT=ROOT/'laptop_workbench/data/experiments/k-geometry-guard';OUT.mkdir(exist_ok=True)
RUN=ROOT.parent/'firmware_builds/i2c_retry_v3/run-0d77c3ca/0d77c3ca05284af9b8e5ad4d0ef81d5c';logs=[json.loads(l) for l in (RUN/'pid.jsonl').read_text().splitlines()];live={x['vision']['seq']:x for x in logs if x.get('vision',{}).get('mask_png')};params=logs[0]['parameters'];assert params['max_gap']==.4
frames=[json.loads(l) for l in (RUN/'frames_001.jsonl').read_text().splitlines()];cap=cv2.VideoCapture(str(RUN/'video_001.avi'));summary={};cards=[]
for name in ['latest','fullclip']:
 baseline=KLanePath();baseline.gate=TemporalBranchGate();candidate=KLanePath();candidate.gate=TemporalBranchGate();pids=[KAdaptivePID(params),KAdaptivePID(params)];rows=[]
 for line in gzip.open(SRC/(name+'-geometry.jsonl.gz'),'rt',encoding='utf8'):
  c=json.loads(line);seq=c['seq'];a=c['a'];v=c['v'];t=c['t']
  data=np.frombuffer(base64.b64decode(live[seq]['vision']['mask_png']),np.uint8) if name=='latest' else np.fromfile(str(ROOT/f'laptop_workbench/data/analysis/f7e170a0869d4fdd8632575997f39a4a/C/{seq:06d}.png'),np.uint8)
  mask=cv2.imdecode(data,0)>0;ga=geometry_guard(a);d=copy.deepcopy(c['d']) # Rejection cannot create positive branch/entrance evidence.
  b=baseline.combine(guard_single(a,baseline.gate),v,mask,copy.deepcopy(c['d']));r=candidate.combine(guard_single(a,candidate.gate),v,mask,d)
  if ga.get('geometry_guard') and r['k']['source'] in ('A','blend'):
   cutoff=ga['geometry_guard']['cutoff_y']*2;r['path']=[p for p in r['path'] if p[1]>=cutoff];r['valid']=len(r['path'])>=2
  row=dict(seq=seq,t=t,guard=ga.get('geometry_guard'),baseline={k:b[k] for k in ['mode','reason','path','valid']},candidate={k:r[k] for k in ['mode','reason','path','valid']})
  for key,pid in zip(['baseline','candidate'],pids):row[key]['pid']=pid.step_path(t,row[key]['path'] if row[key]['valid'] else [])
  rows.append(row)
  if name=='latest' and seq in [438,439,440,441,442,460,470,480,500,520]:
   f=min(frames,key=lambda f:abs(f['capture_monotonic']-t));cap.set(1,f['frame']);ok,im=cap.read();assert ok;im=cv2.resize(im,(640,360));panels=[]
   for label,key,edges in [('BEFORE','baseline',a.get('observed_edges',[])),('GUARDED','candidate',ga.get('observed_edges',[]))]:
    vis=im.copy()
    for edge in edges:
     if len(edge)>1:cv2.polylines(vis,[np.int32(edge)],False,(0,210,0),1)
    path=row[key]['path']
    if path:cv2.polylines(vis,[np.int32(np.array(path)/2)],False,(255,120,0),2)
    cv2.putText(vis,f'{label} seq{seq} {row[key]["mode"]}',(8,22),0,.6,(0,255,255),2);panels.append(vis)
   cv2.imencode('.jpg',np.hstack(panels))[1].tofile(str(OUT/f'compare-{seq}.jpg'));cards.append(dict(seq=seq,image=f'compare-{seq}.jpg',seconds=round(live[seq].get('wall',logs[0]['started'])-logs[0]['started'],2),guard=row['guard']))
 (OUT/(name+'.json')).write_text(json.dumps(rows),encoding='utf8')
 def summarize(rs):
  result={'frames':len(rs),'guard_frames':sum(x['guard'] is not None for x in rs),'path_changes':sum(x['baseline']['path']!=x['candidate']['path'] for x in rs)}
  for key in ['baseline','candidate']:
   transitions=[];last=None
   for x in rs:
    if x[key]['mode']!=last:transitions.append([x['seq'],x[key]['mode']]);last=x[key]['mode']
   result[key]={'transitions':transitions,'empty':sum(not x[key]['valid'] for x in rs),'invalid_pid':sum(not x[key]['pid']['valid'] for x in rs)}
  return result
 summary[name]={'all':summarize(rows)}
 sections=[('focus',438,442),('second_entry',410,557)] if name=='latest' else [('intersection1',640,820),('intersection2',1010,1200),('curve23',210,260),('adjacent97',940,1000),('curve142',1390,1450)]
 for label,lo,hi in sections:summary[name][label]=summarize([x for x in rows if lo<=x['seq']<=hi])
 print(name,json.dumps(summary[name]),flush=True)
(OUT/'report.json').write_text(json.dumps(dict(status='offline_only_not_deployed',version='geometry-guard-r1',summary=summary,cards=cards,max_gap=.4),ensure_ascii=False,indent=2),encoding='utf8')
