"""Only file inference. No network, camera, serial or deployment operations."""
import sys,json,time,traceback,hashlib,shutil
from pathlib import Path
from analysis_jobs import CONFIGS,read,write,ROOT
folder=Path(sys.argv[1]);job=read(folder/'job.json')
try:
 import overnight_runner as lib
 import numpy as np
 lib.torch.set_num_threads(4);lib.cv2.setNumThreads(2)
 inputs=read(folder/'input.json');results={};digests={};done=0
 geom=read(ROOT.parent/'shadow_workbench/geometry.json')
 (folder/'raw').mkdir(exist_ok=True)
 for i,r in enumerate(inputs):
  dest=folder/'raw'/f'{i:06d}.jpg';shutil.copyfile(r['image'],dest);r['image']=str(dest)
 sources=[dict(frame=r['frame'],capture_s=r['capture_s'],sha256=lib.sha(Path(r['image']))) for r in inputs]
 for name in job['models']:
  _,weight,size=next(c for c in CONFIGS if c[0]==name);weight=ROOT.parents[1]/weight if weight else None
  if name=='opencv-local':predict=lib.opencv
  elif name in ('tiny-unet','fast-scnn-inspired'):
   model=(lib.TinyUNet() if name=='tiny-unet' else lib.FastInspired()).cuda();model.load_state_dict(lib.torch.load(weight,weights_only=True)['model']);predict=lib.SmallPredictor(model)
  else:predict=lib.YoloPredictor(weight,size)
  digests[name]=lib.sha(weight) if weight else None
  trackers=[lib.RouteTracker(),lib.MotionTracker(np.array(geom['H'])),lib.UnifiedTracker()];rows=[]
  (folder/name).mkdir(exist_ok=True)
  for i,r in enumerate(inputs):
   start=time.perf_counter();im=lib.read(Path(r['image']));mask=predict(im);t=r['capture_s']
   paths=dict(A=trackers[0].update(mask,t),B=trackers[1].update(im,mask,t),C=trackers[2].update(im,mask,t))
   lib.cv2.imencode('.png',mask.astype('uint8')*255)[1].tofile(str(folder/name/f'{i:06d}.png'))
   rows.append(dict(frame=i,capture_s=t,analysis_ms=round((time.perf_counter()-start)*1000,2),**paths));done+=1
   write(folder/'status.json',dict(state='running',completed=done,total=job['total'],current=name))
  results[name]=rows
  del predict
  lib.torch.cuda.empty_cache()
 write(folder/'result.json',dict(id=job['id'],source=job['source'],models=results,frames=sources,weights_sha256=digests,geometry=geom,execution=job['execution'],limitations=['輸入為已同步原影片解碼的 JPEG 影格，與直接 AVI 解碼可能有壓縮差異。','A 是白線幾何追蹤；B 結合地面映射與影像運動；C 統合單／雙側邊界推算路徑。','沒有人工真值，不計辨識準確率；路徑有輸出不代表正確或可安全行駛。','評語保存在筆電，不寫回原始 Orin 分析檔。']))
 write(folder/'status.json',dict(state='completed',completed=done,total=job['total'],finished=time.time()))
except Exception:
 write(folder/'status.json',dict(state='failed',error=traceback.format_exc()));raise
