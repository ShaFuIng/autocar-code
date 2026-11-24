from pathlib import Path
import sys,json,time,copy,uuid,os,shutil
W=Path(__file__).resolve().parents[1]/'laptop_workbench';sys.path[:0]=[str(W/'runtimes/opencv-4.11'),str(W)]
import cv2,numpy as np
from road_segments_v32 import RoadPath
from junction_k_mask import KMaskPath
from shadow_worker import pid_rows
from analysis_jobs import read,write
cv2.setNumThreads(2)
old=W/'data/analysis/2ccdaa21fd0b464e8e5e38a854b0770f';out=W/'data/experiments/entrance-v32';out.mkdir(exist_ok=True)
r=read(old/'result.json');j=read(old/'job.json');uid=uuid.uuid4().hex;dest=W/'data/analysis'/uid;dest.mkdir();write(out/'job.json',{'job':uid})
j.update(id=uid,title='近處入口約束 A v3.2／遮罩K｜整段離線比較',created=time.time(),state='running',pid=os.getpid());j.pop('validation_summary',None)
groups=[copy.deepcopy(j['spec']['groups'][0]) for _ in range(3)]
for g,k,algo in zip(groups,'ABC',['road-memory-v3.1','road-entrance-v3.2','junction-k-entrance-v2']):g.update(id=k,algorithm=algo,label=algo)
j['spec']['groups']=groups;write(dest/'job.json',j);write(dest/'input.json',read(old/'input.json'))
for name in ['raw','A','B','C']:
 (dest/name).mkdir()
 src=old/('raw' if name=='raw' else 'A')
 for f in src.glob('*'):
  if f.is_file():
   try:os.link(f,dest/name/f.name)
   except OSError:shutil.copy2(f,dest/name/f.name)
base=RoadPath('road-entrance-v3.2');k=KMaskPath();cache=read(W/'data/experiments/k-mask-switch/visible-cache.json');rows=[];combined=[]
for i,f in enumerate(r['frames']):
 im=cv2.imdecode(np.fromfile(str(old/'raw'/f'{i:06d}.jpg'),np.uint8),1);mask=cv2.imdecode(np.fromfile(str(old/'A'/f'{i:06d}.png'),np.uint8),0)>0
 tick=time.perf_counter();a=base.step(im,mask,f['capture_s']);a.update(path_ms=(time.perf_counter()-tick)*1000,capture_s=f['capture_s'],frame=f['frame']);rows.append(a)
 tick=time.perf_counter();c=k.combine(a,cache[i],mask);c.update(path_ms=(time.perf_counter()-tick)*1000,timing_scope='cached_reference_gate_only',capture_s=f['capture_s'],frame=f['frame']);combined.append(c)
 if i%200==0:print(i,flush=True);write(dest/'status.json',dict(state='running',completed=i,total=1760))
r['models']['B'],r['summaries']['B']=pid_rows(rows,groups[1]['parameters'],False);r['models']['C'],r['summaries']['C']=pid_rows(combined,groups[2]['parameters'],False)
r.update(id=uid,spec=j['spec'],baseline_job=old.name,entrance_candidate=True,mask_switch_candidate=True,partial=False)
r['limitations']=['A為原v3.1；B為近處入口約束v3.2；C為新版A接遮罩切換K。原遮罩及原結果保留。C使用可見參考快取，計時不代表完整運算。無真值，不以有路徑判定正確。']
write(dest/'result.json',r);write(dest/'status.json',dict(state='completed',completed=1760,total=1760));write(dest/'exit.json',dict(returncode=0))
report=dict(job=uid,empty_A=sum(not x['path'] for x in rows),empty_C=sum(not x['path'] for x in combined),events=[dict(t=i/10,state=x['k']['state']) for i,x in enumerate(combined) if i==0 or x['mode']!=combined[i-1]['mode']])
write(out/'report.json',report);print(json.dumps(report),flush=True)
