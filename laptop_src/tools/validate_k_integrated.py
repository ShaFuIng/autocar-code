import sys,os,json,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];WB=ROOT/'laptop_workbench';OUT=WB/'data/experiments/k-integrated-r2';SRC=WB/'data/experiments/new-junctions-20261028';OLD=WB/'data/experiments/history-gap-20261028'
sys.path[:0]=[str(WB/'runtimes/opencv-4.11'),str(ROOT/'web'),str(ROOT/'web/k_lane_runtime'),str(ROOT.parent/'yolo26_sem_v5_focus70/.deps')]
os.environ['YOLO_CONFIG_DIR']=str(OUT/'yolo-config')
import cv2,numpy as np
from k_integrated_live import IntegratedK
from k_adaptive_pid import KAdaptivePID
from ultralytics import YOLO

def main():
 cv2.setNumThreads(2);model=YOLO(str(ROOT.parent/'yolo26_sem_v4_trial/runs/trial01/weights/best.pt'));p=json.loads((WB/'data/experiments/warm-smooth-20261028/parameters.json').read_text())['data']['parameters'];report=dict(status='running',algorithm=IntegratedK.algorithm,clips=[],parameters='v79/18px/400ms',hardware_output=False)
 for c in json.loads((SRC/'report.json').read_text(encoding='utf8'))['clips']:
  name=c['name'];old=json.loads((OLD/name/'rows.json').read_text());stamps=json.loads((Path(c['source_folder'])/'preview/frames.json').read_text());caps={};tracker=IntegratedK();pid=KAdaptivePID(p);rows=[]
  try:
   for i,st in enumerate(stamps):
    if st['source'] not in caps:caps[st['source']]=cv2.VideoCapture(str(Path(c['source_folder'])/st['source']))
    ok,im=caps[st['source']].read();assert ok;t=st['capture_s'];processed=tracker.preprocess(im,t)
    if tracker.prepared[3]['active']:
     sem=model.predict(processed,imgsz=640,device=0,verbose=False)[0].semantic_mask.data.cpu().numpy()==1;mask=cv2.resize(sem.astype('uint8'),(640,360),interpolation=cv2.INTER_NEAREST)>0
    else:mask=cv2.imdecode(np.fromfile(str(SRC/name/'mask'/f'{i:06d}.png'),np.uint8),0)>0
    early=[];start=time.perf_counter();r=tracker.step(processed,mask,t,on_a=lambda x:early.append(x));ms=(time.perf_counter()-start)*1000
    assert len(early)<=1
    if early:assert early[0]['path']==r['path'] and early[0]['valid']==r['valid'] and early[0]['path_diagnostic']['mode']==r['path_diagnostic']['mode'],(name,i,'early differs')
    d=r['path_diagnostic'];pr=pid.step_path(t,r['path'] if r['valid'] else [])
    rows.append(dict(index=i,t=t,valid=r['valid'],path=r['path'],pid=pr,mode=d['mode'],diagnostic=d,ms=ms,early=bool(early),baseline_equal=r['path']==old[i]['candidate']['path'],mode_equal=d['mode']==old[i]['candidate']['mode']))
    if i%150==0:print(name,i,flush=True)
  finally:
   tracker.close()
   for cap in caps.values():cap.release()
  (OUT/(name+'.json')).write_text(json.dumps(rows),encoding='utf8')
  summary=dict(name=name,frames=len(rows),empty=sum(not x['valid'] for x in rows),pid_invalid=sum(not x['pid']['valid'] for x in rows),path_differences=[x['index'] for x in rows if not x['baseline_equal']],mode_differences=[x['index'] for x in rows if not x['mode_equal']],early_frames=sum(x['early'] for x in rows),path_p95_ms=float(np.percentile([x['ms'] for x in rows],95)),color_p95_ms=float(np.percentile([x['diagnostic']['color_correction']['elapsed_ms'] for x in rows],95)))
  report['clips'].append(summary);(OUT/'report.json').write_text(json.dumps(report,indent=2),encoding='utf8');print(summary,flush=True)
 report['status']='complete';(OUT/'report.json').write_text(json.dumps(report,indent=2),encoding='utf8')
if __name__=='__main__':main()
