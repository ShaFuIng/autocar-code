import sys,json
from pathlib import Path
W=Path(__file__).resolve().parents[1]/'laptop_workbench';sys.path[:0]=[str(W/'runtimes/opencv-4.11'),str(W)]
import cv2
cv2.setNumThreads(2)
from shadow_worker import path_rows
from analysis_jobs import read,write
uid=read(W/'data/experiments/entrance-v32/job.json')['job'];folder=W/'data/analysis'/uid;r=read(folder/'result.json')
inputs=[dict(image=str(folder/'raw'/f'{i:06d}.jpg'),capture_s=f['capture_s'],frame=f['frame']) for i,f in enumerate(r['frames'])];errors=[]
for i,x in enumerate(path_rows(r['spec']['groups'][2],inputs,folder/'C',None)):
 expected=r['models']['C'][i]['A']
 if x['path']!=expected['path'] or x['mode']!=expected['mode']:errors.append(i)
 if i%400==0:print(i,flush=True)
report=dict(job=uid,frames=len(inputs),worker_mismatch_frames=errors,opencv=cv2.__version__,hardware_output=False,unit_tests=3)
write(W/'ENTRANCE_V32_ACCEPTANCE.json',report);print(json.dumps(report),flush=True)
