from pathlib import Path
import sys,json,copy,time,uuid,os,hashlib,shutil
ROOT=Path(__file__).resolve().parents[1];W=ROOT/'laptop_workbench';sys.path[:0]=[str(W/'runtimes/opencv-4.11'),str(W)]
import cv2,numpy as np
from junction_k_mask import KMaskPath
from junction_k import VisibleTracker
from shadow_worker import pid_rows
from analysis_jobs import write,read
cv2.setNumThreads(2)
old=W/'data/analysis/ee3d85e48a8a47b592fb4997ea645d67';out=W/'data/experiments/k-mask-switch';out.mkdir(exist_ok=True)
original_sha=hashlib.sha256((old/'result.json').read_bytes()).hexdigest();result=read(old/'result.json');job=read(old/'job.json');uid=uuid.uuid4().hex;dest=W/'data/analysis'/uid;dest.mkdir()
job.update(id=uid,title='遮罩切換新版｜0:00～2:56｜A／原K／遮罩K｜PID關閉',created=time.time(),state='running',pid=os.getpid(),total=1760);job.pop('validation_summary',None)
g=copy.deepcopy(job['spec']['groups'][1]);g.update(id='C',label='遮罩切換K',algorithm='junction-k-mask-v1');job['spec']['groups'].append(g)
write(dest/'job.json',job);write(dest/'input.json',read(old/'input.json'));write(out/'job.json',{'job':uid,'baseline':old.name})
for name in ['raw','A','B']:
 (dest/name).mkdir()
 for f in (old/name).glob('*'):
  if f.is_file():
   try:os.link(f,dest/name/f.name)
   except OSError:shutil.copy2(f,dest/name/f.name)
(dest/'C').mkdir()
for f in (old/'A').glob('*.png'):
 try:os.link(f,dest/'C'/f.name)
 except OSError:shutil.copy2(f,dest/'C'/f.name)
tracker=KMaskPath();visible=VisibleTracker();cachefile=out/'visible-cache.json';cache=read(cachefile) if cachefile.exists() else [];rows=[]
for i,frame in enumerate(result['frames']):
 mask=cv2.imdecode(np.fromfile(str(old/'A'/f'{i:06d}.png'),np.uint8),0)>0
 if len(cache)>i:v=cache[i]
 else:
  im=cv2.imdecode(np.fromfile(str(old/'raw'/f'{i:06d}.jpg'),np.uint8),1);v=visible.step(im);cache.append(v)
 tick=time.perf_counter();r=tracker.combine(result['models']['A'][i]['A'],v,mask);r.update(timing_scope='cached_reference_gate_only',path_ms=(time.perf_counter()-tick)*1000,capture_s=frame['capture_s'],frame=frame['frame']);rows.append(r)
 if i%50==0:write(dest/'status.json',dict(state='running',completed=i,total=1760,current='遮罩切換K；沿用原V4遮罩'));print(i,flush=True)
write(cachefile,cache)
result['models']['C'],result['summaries']['C']=pid_rows(rows,g['parameters'],False)
result.update(id=uid,spec=job['spec'],baseline_job=old.name,partial=False,mask_switch_candidate=True)
result['limitations'].append('C為遮罩切換K離線候選；A/B沿用原結果，白線遮罩相同，C不重新辨識模型。')
write(dest/'result.json',result);write(dest/'status.json',dict(state='completed',completed=1760,total=1760,finished=time.time()));write(dest/'exit.json',{'returncode':0})
assert hashlib.sha256((old/'result.json').read_bytes()).hexdigest()==original_sha
metrics=dict(job=uid,baseline=old.name,baseline_sha256=original_sha,empty=sum(not x['path'] for x in rows),k_frames=sum(x['k']['source']=='K' for x in rows),events=[dict(player_s=i/10,state=x['k']['state'],reason=x['reason']) for i,x in enumerate(rows) if i==0 or x['mode']!=rows[i-1]['mode']],focus={str(t):rows[round(t*10)]['k'] for t in [23.4,65,70,75,79,102,108,112,117.7]},hardware_output=False,opencv=cv2.__version__)
write(out/'report.json',metrics);print(json.dumps({k:v for k,v in metrics.items() if k!='focus'},ensure_ascii=False),flush=True)
