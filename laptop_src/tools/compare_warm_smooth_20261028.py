"""Offline color experiments; no live registration or hardware operations."""
import sys,os,json,copy,hashlib,importlib.util
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];WB=ROOT/'laptop_workbench';OUT=WB/'data/experiments/warm-smooth-20261028';OUT.mkdir(exist_ok=True)
PREV=WB/'data/experiments/new-junctions-20261028';BASE=WB/'data/experiments/selected-chain-20261028'
sys.path[:0]=[str(WB/'runtimes/opencv-4.11'),str(ROOT/'web'),str(ROOT/'web/k_lane_runtime'),str(ROOT.parent/'yolo26_sem_v5_focus70/.deps')]
os.environ['YOLO_CONFIG_DIR']=str(OUT/'yolo-config')
import cv2,numpy as np
from ultralytics import YOLO
from junction_segment_candidate import SegmentCandidate
from k_adaptive_pid import KAdaptivePID
from warm_color_candidate import WarmColor
cv2.setNumThreads(2)
params=json.loads((BASE/'parameters.json').read_text());p=params['data']['parameters'];assert params['rev']==79 and p['visual_center_offset_px_640']==18 and p['max_gap']==.4
(OUT/'parameters.json').write_text(json.dumps(params,indent=2))
model=YOLO(str(ROOT.parent/'yolo26_sem_v4_trial/runs/trial01/weights/best.pt'))
report=dict(status='running',parameter_revision=79,visual_center_offset_px_640=18,max_gap=.4,clips=[],methods=['raw V4 + selected-chain K','bounded ground-highlight color correction + V4','temporally smoothed bounded color + V4'])
for c in json.loads((PREV/'report.json').read_text())['clips']:
 name=c['name'];folder=OUT/name;folder.mkdir(exist_ok=True);old=json.loads((BASE/name/'rows.json').read_text());previous=json.loads((WB/'data/experiments/warm-no-label-20261028'/name/'rows.json').read_text());stamps=json.loads((Path(c['source_folder'])/'preview/frames.json').read_text());caps={};rows=[]
 smooth=WarmColor();algs=[SegmentCandidate()];pids=[KAdaptivePID(p)]
 for i,st in enumerate(stamps):
  if st['source'] not in caps:caps[st['source']]=cv2.VideoCapture(str(Path(c['source_folder'])/st['source']))
  ok,im=caps[st['source']].read();assert ok;im=cv2.resize(im,(640,360));t=st['capture_s'];raw=cv2.imdecode(np.fromfile(str(PREV/name/'mask'/f'{i:06d}.png'),np.uint8),0)>0
  corrected,diag=smooth.step(im,t);gains=np.array(diag['gains']);active=diag['active']
  if active:
   sem=model.predict(corrected,imgsz=640,device=0,verbose=False)[0].semantic_mask.data.cpu().numpy()==1
   bmask=cv2.resize(sem.astype('uint8'),(640,360),interpolation=cv2.INTER_NEAREST)>0
  else:bmask=raw.copy()
  row=dict(index=i,t=t,active=active,gains=gains.tolist(),color_state=diag,added_pixels=0,baseline=old[i]['candidate'])
  alg=algs[0];a=alg.base.step(corrected,bmask,t);v=alg.visible.step(corrected)
  from junction_k_lane import lane_evidence
  d=lane_evidence(bmask,a,v);d['capture_s']=t;r=alg.combine(a,v,bmask,d);pr=pids[0].step_path(t,r['path'] if r['valid'] else [])
  row['warm']={k:r[k] for k in ['path','valid','mode','reason','k']};row['warm']['pid']=pr;row['warm']['motion']=a.get('motion');row['warm']['memory']=a.get('memory_state')
  row['color']=previous[i]['color'];cmask=bmask
  panels=[]
  for key,mask in zip(['baseline','color','warm'],[raw,bmask,cmask]):
   vis=im.copy();vis[mask]=(vis[mask]*.65+np.array([220,40,220])*.35).astype('uint8')
   path=row[key]['path']
   if path:cv2.polylines(vis,[np.int32(np.array(path)/2)],False,(255,130,0),2)
   cv2.putText(vis,f"{key} {i} {t:.2f}s {row[key]['mode']}",(6,20),0,.5,(0,255,255),1);panels.append(vis if key!='color' else cv2.imdecode(np.fromfile(str(WB/'data/experiments/warm-no-label-20261028'/name/f'{i:06d}.jpg'),np.uint8),1)[:,640:1280])
  cv2.imencode('.jpg',np.hstack(panels),[cv2.IMWRITE_JPEG_QUALITY,85])[1].tofile(str(folder/f'{i:06d}.jpg'))
  row['mask_pixels']=[int(raw.sum()),previous[i]['mask_pixels'][1],int(bmask.sum())];rows.append(row)
  if i%100==0:print(name,i,flush=True)
 for cap in caps.values():cap.release()
 (folder/'rows.json').write_text(json.dumps(rows),encoding='utf8')
 summary=dict(name=name,title=c['title'],frames=len(rows),duration=rows[-1]['t'],groups={},active_frames=sum(r['active'] for r in rows))
 for key in ['baseline','color','warm']:
  changes=[];last=None
  for r in rows:
   if r[key]['mode']!=last:changes.append(dict(index=r['index'],t=r['t'],mode=r[key]['mode'],reason=r[key]['reason']));last=r[key]['mode']
  summary['groups'][key]=dict(empty=sum(not r[key]['valid'] for r in rows),pid_invalid=sum(not r[key]['pid']['valid'] for r in rows),transitions=changes)
 report['clips'].append(summary);(OUT/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8');print(json.dumps(summary,ensure_ascii=False),flush=True)
report['status']='complete';(OUT/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
