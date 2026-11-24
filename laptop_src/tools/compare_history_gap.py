import sys,json,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];WB=ROOT/'laptop_workbench';BASE=WB/'data/experiments/warm-smooth-20261028';SRC=WB/'data/experiments/new-junctions-20261028';OUT=WB/'data/experiments/history-gap-20261028';OUT.mkdir(exist_ok=True)
sys.path[:0]=[str(WB/'runtimes/opencv-4.11'),str(ROOT/'web'),str(ROOT/'web/k_lane_runtime')]
import cv2,numpy as np
from history_gap_candidate import HistoryGap
from k_adaptive_pid import KAdaptivePID
cv2.setNumThreads(2)
p=json.loads((BASE/'parameters.json').read_text())['data']['parameters'];report=dict(status='running',clips=[],parameters='v79/18px/400ms',evidence='Frozen smooth-color paths; independent raw V4 mask checks; no model rerun')
for c in json.loads((SRC/'report.json').read_text(encoding='utf8'))['clips']:
 name=c['name'];folder=OUT/name;folder.mkdir(exist_ok=True);old=json.loads((BASE/name/'rows.json').read_text());stamps=json.loads((Path(c['source_folder'])/'preview/frames.json').read_text());caps={};rows=[];tracker=HistoryGap();pid=KAdaptivePID(p)
 for i,st in enumerate(stamps):
  if st['source'] not in caps:caps[st['source']]=cv2.VideoCapture(str(Path(c['source_folder'])/st['source']))
  ok,image=caps[st['source']].read();assert ok;image=cv2.resize(image,(640,360));mask=cv2.imdecode(np.fromfile(str(SRC/name/'mask'/f'{i:06d}.png'),np.uint8),0)>0
  before=old[i]['warm'];start=time.perf_counter();r=tracker.step(image,mask,st['capture_s'],before);ms=(time.perf_counter()-start)*1000;r['pid']=pid.step_path(st['capture_s'],r['path'] if r['valid'] else [])
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
