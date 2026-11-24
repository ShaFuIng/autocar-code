from pathlib import Path
import sys,json,time,copy,uuid,os,shutil,hashlib
W=Path(__file__).resolve().parents[1]/'laptop_workbench';sys.path[:0]=[str(W/'runtimes/opencv-4.11'),str(W)]
import cv2,numpy as np
from junction_k_lane import KLanePath
from shadow_worker import pid_rows
from analysis_jobs import read,write
cv2.setNumThreads(2)
old=W/'data/analysis/8e65b7bc19a3403cafccaad50b770645';out=W/'data/experiments/k-lane-switch';out.mkdir(exist_ok=True)
original_sha=hashlib.sha256((old/'result.json').read_bytes()).hexdigest();r=read(old/'result.json');j=read(old/'job.json');uid=uuid.uuid4().hex;dest=W/'data/analysis'/uid;dest.mkdir();write(out/'job.json',{'job':uid})
j.update(id=uid,title='共用車道約束K｜整段離線比較',created=time.time(),state='running',pid=os.getpid());j.pop('validation_summary',None)
groups=[copy.deepcopy(j['spec']['groups'][1]),copy.deepcopy(j['spec']['groups'][2]),copy.deepcopy(j['spec']['groups'][2])]
for g,k in zip(groups,'ABC'):g['id']=k
groups[2].update(algorithm='junction-k-lane-v3',label='共用車道約束K');j['spec']['groups']=groups
write(dest/'job.json',j);write(dest/'input.json',read(old/'input.json'))
for name in ['raw','A','B','C']:
 (dest/name).mkdir();src=old/('raw' if name=='raw' else 'A')
 for f in src.glob('*'):
  if f.is_file():
   try:os.link(f,dest/name/f.name)
   except OSError:shutil.copy2(f,dest/name/f.name)
r['models']['A']=copy.deepcopy(r['models']['B']);r['models']['B']=copy.deepcopy(r['models']['C']);r['summaries']['A']=copy.deepcopy(r['summaries']['B']);r['summaries']['B']=copy.deepcopy(r['summaries']['C'])
k=KLanePath();cache=read(W/'data/experiments/k-mask-switch/visible-cache.json');rows=[]
for i,f in enumerate(r['frames']):
 mask=cv2.imdecode(np.fromfile(str(old/'A'/f'{i:06d}.png'),np.uint8),0)>0
 tick=time.perf_counter();c=k.combine(r['models']['A'][i]['A'],cache[i],mask);c.update(path_ms=(time.perf_counter()-tick)*1000,timing_scope='cached_reference_gate_only',capture_s=f['capture_s'],frame=f['frame']);rows.append(c)
 if i%400==0:print(i,flush=True);write(dest/'status.json',dict(state='running',completed=i,total=1760))
r['models']['C'],r['summaries']['C']=pid_rows(rows,groups[2]['parameters'],False)
r.update(id=uid,spec=j['spec'],baseline_job=old.name,lane_candidate=True,entrance_candidate=True,mask_switch_candidate=True,partial=False)
r['limitations']=['A為入口約束A v3.2；B為前版遮罩K；C為共用車道約束K。沿用同一遮罩、A與可見參考快取；計時僅切換計算，不代表完整效能。原版保留，不以有路徑當正確率。']
write(dest/'result.json',r);write(dest/'status.json',dict(state='completed',completed=1760,total=1760));write(dest/'exit.json',dict(returncode=0))
assert hashlib.sha256((old/'result.json').read_bytes()).hexdigest()==original_sha
report=dict(job=uid,baseline_sha256=original_sha,empty=sum(not x['path'] for x in rows),events=[dict(t=i/10,state=x['k']['state'],reason=x['reason']) for i,x in enumerate(rows) if i==0 or x['mode']!=rows[i-1]['mode']])
write(out/'report.json',report);print(json.dumps(report),flush=True)
