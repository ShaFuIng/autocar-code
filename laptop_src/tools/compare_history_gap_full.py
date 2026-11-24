import sys,json,time,os
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];WB=ROOT/'laptop_workbench';BASE=WB/'data/experiments/warm-smooth-20261028';SRC=WB/'data/experiments/new-junctions-20261028';OUT=WB/'data/experiments/history-gap-20261028';OUT.mkdir(exist_ok=True)
sys.path[:0]=[str(WB/'runtimes/opencv-4.11'),str(ROOT/'web'),str(ROOT/'web/k_lane_runtime'),str(ROOT.parent/'yolo26_sem_v5_focus70/.deps')]
import cv2,numpy as np
from history_gap_candidate import HistoryGap
from warm_color_candidate import WarmColor
from junction_segment_candidate import SegmentCandidate
from junction_k_lane import lane_evidence
os.environ.setdefault('YOLO_CONFIG_DIR',str(OUT/'yolo-config'))
from ultralytics import YOLO
model=YOLO(str(ROOT.parent/"yolo26_sem_v4_trial/runs/trial01/weights/best.pt"))
from k_adaptive_pid import KAdaptivePID
cv2.setNumThreads(2)
p=json.loads((BASE/'parameters.json').read_text())['data']['parameters'];report=dict(status='running',clips=[],parameters='v79/18px/400ms',evidence='Full smooth-color K output including geometry guard; active color frames rerun V4; independent raw mask for history checks')
for c in json.loads((SRC/'report.json').read_text(encoding='utf8'))['clips']:
 name=c['name'];folder=OUT/name;folder.mkdir(exist_ok=True);old=json.loads((BASE/name/'rows.json').read_text());stamps=json.loads((Path(c['source_folder'])/'preview/frames.json').read_text());caps={};rows=[];tracker=HistoryGap();pid=KAdaptivePID(p);color=WarmColor();alg=SegmentCandidate();refpid=KAdaptivePID(p)
 for i,st in enumerate(stamps):
  if st['source'] not in caps:caps[st['source']]=cv2.VideoCapture(str(Path(c['source_folder'])/st['source']))
  ok,image=caps[st['source']].read();assert ok;image=cv2.resize(image,(640,360));mask=cv2.imdecode(np.fromfile(str(SRC/name/'mask'/f'{i:06d}.png'),np.uint8),0)>0
  corrected,cd=color.step(image,st['capture_s']);cmask=mask
  if cd['active']:
   sem=model.predict(corrected,imgsz=640,device=0,verbose=False)[0].semantic_mask.data.cpu().numpy()==1;cmask=cv2.resize(sem.astype('uint8'),(640,360),interpolation=cv2.INTER_NEAREST)>0
  a=alg.base.step(corrected,cmask,st['capture_s']);v=alg.visible.step(corrected);d=lane_evidence(cmask,a,v);d['capture_s']=st['capture_s'];before=alg.combine(a,v,cmask,d);before['pid']=refpid.step_path(st['capture_s'],before['path'] if before['valid'] else [])
  assert before['path']==old[i]['warm']['path'] and before['mode']==old[i]['warm']['mode'], (name,i,'baseline changed')
  start=time.perf_counter();r=tracker.step(image,mask,st['capture_s'],before);ms=(time.perf_counter()-start)*1000;r['pid']=pid.step_path(st['capture_s'],r['path'] if r['valid'] else [])
  row=dict(index=i,t=st['capture_s'],baseline=before,candidate=r,elapsed_ms=ms);rows.append(row)
  panels=[]
  for item in [before,r]:
   im=image.copy();im[mask]=(im[mask]*.65+np.array([220,40,220])*.35).astype('uint8')
   if item['path']:cv2.polylines(im,[np.int32(np.array(item['path'])/2)],False,(255,130,0),2)
   if item.get('gap_history_path'):cv2.polylines(im,[np.int32(np.array(item['gap_history_path'])/2)],False,(0,170,255),3)
   panels.append(im)
  cv2.imencode('.jpg',np.hstack(panels),[cv2.IMWRITE_JPEG_QUALITY,85])[1].tofile(str(folder/f'{i:06d}.jpg'))
 for cap in caps.values():cap.release()
 (folder/'rows.json').write_text(json.dumps(rows),encoding='utf8')
 from collections import Counter
 summary=dict(name=name,title=c['title'],frames=len(rows),before_empty=sum(not r['baseline']['valid'] for r in rows),after_empty=sum(not r['candidate']['valid'] for r in rows),used=[r['index'] for r in rows if r['candidate']['gap_repair']['used']],reasons=dict(Counter(r['candidate']['gap_repair']['reason'] for r in rows)),p95_ms=float(np.percentile([r['elapsed_ms'] for r in rows],95)),mode_changes=sum(r['baseline']['mode']!=r['candidate']['mode'] for r in rows))
 report['clips'].append(summary);(OUT/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8');print(summary,flush=True)
report['status']='complete';(OUT/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
