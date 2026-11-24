import sys,json,copy
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];WB=ROOT/'laptop_workbench';BASE=WB/'data/experiments/warm-smooth-20261028';SRC=WB/'data/experiments/new-junctions-20261028';OUT=WB/'data/experiments/history-gap-20261028'
sys.path[:0]=[str(WB/'runtimes/opencv-4.11'),str(ROOT/'web'),str(ROOT/'web/k_lane_runtime')]
import cv2,numpy as np
from history_gap_candidate import HistoryGap
cv2.setNumThreads(2)
results=[]
for c in json.loads((SRC/'report.json').read_text(encoding='utf8'))['clips']:
 if c['name']=='slope':continue
 name=c['name'];rows=json.loads((OUT/name/'rows.json').read_text());stamps=json.loads((Path(c['source_folder'])/'preview/frames.json').read_text())
 for target in [30,60,90,151]:
  f=HistoryGap();got=[]
  for i in range(target-1,target+2):
   st=stamps[i];cap=cv2.VideoCapture(str(Path(c['source_folder'])/st['source']));cap.set(cv2.CAP_PROP_POS_FRAMES,st['source_frame']);ok,im=cap.read();cap.release();assert ok;im=cv2.resize(im,(640,360));mask=cv2.imdecode(np.fromfile(str(SRC/name/'mask'/f'{i:06d}.png'),np.uint8),0)>0
   r=copy.deepcopy(rows[i]['baseline'])
   if i==target:r['path']=[];r['valid']=False
   a=f.step(im,mask,st['capture_s'],r);got.append(a)
  a=got[1];expected=np.array(rows[target]['baseline']['path'])/2;actual=np.array(a['path'])/2;error=None
  if len(expected)>1 and len(actual)>1:
   expected=expected[expected[:,1].argsort()];actual=actual[actual[:,1].argsort()];ys=np.arange(max(expected[0,1],actual[0,1]),min(expected[-1,1],actual[-1,1]),4)
   if len(ys):error=float(np.max(abs(np.interp(ys,expected[:,1],expected[:,0])-np.interp(ys,actual[:,1],actual[:,0]))))
  results.append(dict(clip=name,index=target,t=stamps[target]['capture_s'],injected=True,restored=a['valid'],diagnostic=a['gap_repair'],difference_from_original_px=error,return_exact=got[2]['path']==rows[target+1]['baseline']['path']))
(OUT/'injected-dropouts.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf8');print(json.dumps(results,ensure_ascii=False))
