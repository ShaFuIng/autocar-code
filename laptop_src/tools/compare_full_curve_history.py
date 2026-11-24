"""Read-only source replay: continuous history vs reset at curve entry. No vehicle I/O."""
import sys,json,gzip,copy
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];WB=ROOT/'laptop_workbench';OUT=WB/'data/experiments/full-curve-history-20261028';OUT.mkdir(exist_ok=True)
sys.path[:0]=[str(WB/'runtimes/opencv-4.11'),str(ROOT/'web'),str(ROOT/'web/k_lane_runtime')]
import cv2,numpy as np
from junction_segment_candidate import SegmentCandidate
from junction_k_lane import lane_evidence
cv2.setNumThreads(2)
full=SegmentCandidate();cold=None;rows=[];windows=[('curve23',210,260),('adjacent97',940,1000),('curve142',1390,1450)];report={'status':'running','windows':[],'method':'Identical archived raw V4 masks and visible tracker evidence; A recomputed sequentially. No color intervention; isolates history initialization.'}
for line in gzip.open(WB/'data/experiments/k-branch-temporal/fullclip-geometry.jsonl.gz','rt',encoding='utf8'):
 c=json.loads(line);i=c['seq'];t=c['t'];im=cv2.imdecode(np.fromfile(str(ROOT/f'shadow_workbench/runs/k-video003-000-256/{i:06d}.jpg'),np.uint8),1);im=cv2.resize(im,(640,360));mask=cv2.imdecode(np.fromfile(str(WB/f'data/analysis/f7e170a0869d4fdd8632575997f39a4a/C/{i:06d}.png'),np.uint8),0)>0
 a=full.base.step(im,mask,t);d=lane_evidence(mask,a,c['v']);d['capture_s']=t;r=full.combine(a,c['v'],mask,d)
 row=dict(index=i,t=t,player_s=i/10,full=r)
 win=next((w for w in windows if w[1]<=i<=w[2]),None)
 if win:
  if i==win[1]:cold=SegmentCandidate()
  ac=cold.base.step(im,mask,t);dc=lane_evidence(mask,ac,c['v']);dc['capture_s']=t;rc=cold.combine(ac,c['v'],mask,dc);row['cold']=rc;row['window']=win[0]
  panels=[]
  for label,result in [('CONTINUOUS',r),('RESET AT WINDOW START',rc)]:
   vis=im.copy();display_mask=cv2.resize(mask.astype('uint8'),(640,360),interpolation=cv2.INTER_NEAREST)>0;vis[display_mask]=(vis[display_mask]*.65+np.array([220,40,220])*.35).astype('uint8')
   if result['path']:cv2.polylines(vis,[np.int32(np.array(result['path'])/2)],False,(255,130,0),2)
   cv2.putText(vis,label,(8,20),0,.5,(0,255,255),1);panels.append(vis)
  cv2.imencode('.jpg',np.hstack(panels),[cv2.IMWRITE_JPEG_QUALITY,85])[1].tofile(str(OUT/f'{i:06d}.jpg'))
 rows.append(row)
 if i%300==0:print(i,flush=True)
(OUT/'rows.json').write_text(json.dumps(rows),encoding='utf8')
for name,lo,hi in windows:
 rs=[x for x in rows if lo<=x['index']<=hi];summary=dict(name=name,start=lo,end=hi,frames=len(rs))
 for key in ['full','cold']:
  summary[key]=dict(empty=sum(not x[key]['valid'] for x in rs),far_pair_rejected=sum(any(z['reason']=='FAR_PAIR_NOT_CONNECTED_TO_ENTRANCE' for z in x[key].get('rejections',[])) for x in rs),with_memory=sum(x[key].get('memory_state',{}).get('stored_points',0)>0 for x in rs),modes=sorted(set(x[key]['mode'] for x in rs)))
 summary['different_paths']=sum(x['full']['path']!=x['cold']['path'] for x in rs);report['windows'].append(summary)
report['frames']=len(rows);report['status']='complete';(OUT/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8');print(json.dumps(report),flush=True)
