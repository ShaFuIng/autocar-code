import os,json,time
from pathlib import Path
os.environ.setdefault('USER','jetson')
import torch,cv2,numpy as np
from ultralytics import YOLO
torch.set_num_threads(2);cv2.setNumThreads(2)
out=Path('/work/comparison');out.mkdir(exist_ok=True)
def metrics(c):
 tp,fp,fn,tn=map(int,c);return dict(tp=tp,fp=fp,fn=fn,tn=tn,boundary_iou=tp/max(1,tp+fp+fn),precision=tp/max(1,tp+fp),recall=tp/max(1,tp+fn),dice=2*tp/max(1,2*tp+fp+fn))
report={'note':'Matched original-resolution evaluation; same-route development validation, not independent test. One unidentified partially labeled image was retained at user request.','sets':{}}
for setname,base in [('new_v4',Path('/work/dataset')),('old_v3',Path('/previous/dataset'))]:
 report['sets'][setname]={}
 for version,weight in [('v3','/previous/runs/trial01/weights/best.pt'),('v4','/work/runs/trial01/weights/best.pt')]:
  model=YOLO(weight);total=np.zeros(4,dtype=np.int64);rows=[]
  for p in sorted((base/'images/val').glob('*.jpg')):
   im=cv2.imread(str(p));gt=cv2.imread(str(base/'masks/val'/(p.stem+'.png')),0)==1
   pred=model.predict(im,imgsz=640,device=0,half=False,verbose=False)[0].semantic_mask.data.cpu().numpy()==1
   assert pred.shape==gt.shape
   c=np.array([(pred&gt).sum(),(pred&~gt).sum(),(~pred&gt).sum(),(~pred&~gt).sum()]);total+=c;rows.append({'name':p.name,**metrics(c)})
   overlay=im.copy();overlay[pred]=(overlay[pred]*.5+np.array([255,0,255])*.5).astype('uint8')
   cv2.imwrite(str(out/(setname+'_'+version+'_'+p.stem+'.jpg')),overlay)
  report['sets'][setname][version]={'count':len(rows),'aggregate':metrics(total),'per_image':rows}
  del model;torch.cuda.empty_cache()
 a=report['sets'][setname];a['delta_v4_minus_v3']={k:a['v4']['aggregate'][k]-a['v3']['aggregate'][k] for k in ('boundary_iou','precision','recall','dice')}
 (out/'metrics.json').write_text(json.dumps(report,indent=2))
 print(setname,json.dumps(a['delta_v4_minus_v3']),flush=True)
print('COMPARISON_COMPLETE',flush=True)
