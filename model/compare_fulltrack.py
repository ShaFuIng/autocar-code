from pathlib import Path
import os,json,time,hashlib
os.environ.setdefault('USER','jetson')
import cv2,numpy as np,torch
from ultralytics import YOLO
torch.set_num_threads(2);cv2.setNumThreads(2)
out=Path('/out');models=[YOLO('/v3/best.pt'),YOLO('/v4/best.pt')]
segments=[Path('/input/video_003.avi'),Path('/input/video_004.avi')]
report={'input':[],'inference':'imgsz640, FP32, no temporal smoothing, all frames','note':'Source nominal FPS playback. Capture elapsed comes from recording timestamps. This video includes training frames; visual demonstration, not independent validation.','weights_sha256':[hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in ('/v3/best.pt','/v4/best.pt')]}
from datetime import datetime
first=None;count=0;writer=None;times=[[],[]]
for source in segments:
 cap=cv2.VideoCapture(str(source));fps=cap.get(cv2.CAP_PROP_FPS);expected=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));timestamps=[json.loads(l)['time'] for l in source.with_name(source.name.replace('video_','frames_').replace('.avi','.jsonl')).read_text().splitlines()]
 assert len(timestamps)==expected
 if writer is None:
  output_fps=fps;writer=cv2.VideoWriter(str(out/'comparison_raw.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),fps,(1920,408));assert writer.isOpened()
 assert fps==output_fps
 local=0
 while True:
  ok,im=cap.read()
  if not ok:break
  stamp=datetime.fromisoformat(timestamps[local]);first=first or stamp;elapsed=(stamp-first).total_seconds()
  panel=np.zeros((408,1920,3),np.uint8);views=[im]
  for j,model in enumerate(models):
   start=time.perf_counter();pred=model.predict(im,imgsz=640,device=0,half=False,verbose=False)[0].semantic_mask.data.cpu().numpy()==1;times[j].append((time.perf_counter()-start)*1000)
   assert pred.shape==im.shape[:2]
   view=im.copy();view[pred]=(view[pred]*.55+np.array([255,0,255])*.45).astype('uint8');views.append(view)
  for j,(label,view) in enumerate(zip(['Original','Previous v3','New v4'],views)):
   cv2.putText(panel,label,(j*640+12,28),cv2.FONT_HERSHEY_SIMPLEX,.8,(255,255,255),2)
   panel[48:408,j*640:(j+1)*640]=cv2.resize(view,(640,360),interpolation=cv2.INTER_AREA)
  cv2.putText(panel,f'Capture {elapsed:6.1f}s | Frame {count:04d}',(210,28),0,.52,(180,180,180),1)
  writer.write(panel)
  if count in (0,450,900,1200,1500,1740,1770,1950,2200,2400):cv2.imwrite(str(out/f'frame_{count:04d}.jpg'),panel)
  count+=1;local+=1
  if count%200==0:print('FRAMES',count,flush=True)
 cap.release();assert local==expected
 report['input'].append({'name':source.name,'frames':local,'fps':fps})
writer.release();cap=cv2.VideoCapture(str(out/'comparison_raw.mp4'));decoded=0
while cap.read()[0]:decoded+=1
cap.release();assert decoded==count
report.update(frames=count,decoded_output=decoded,output_fps=output_fps,playback_seconds=count/output_fps,capture_elapsed_seconds=elapsed,predict_median_ms=[float(np.median(t[5:])) for t in times])
(out/'video_report.json').write_text(json.dumps(report,indent=2));print('COMPLETE',json.dumps(report),flush=True)
