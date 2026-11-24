from pathlib import Path
import sys,json,copy,time,uuid,os,hashlib,shutil
W=Path(__file__).resolve().parents[1]/'laptop_workbench'
sys.path[:0]=[str(W/'runtimes/opencv-4.11'),str(W)]
import numpy as np
from shadow_worker import pid_rows
def read(p):return json.loads(p.read_text(encoding='utf-8'))
def write(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2),encoding='utf-8')
out=W/'data/experiments/k-lane-deployed'; remote=read(out/'result.json')
src=W/'data/analysis/f7e170a0869d4fdd8632575997f39a4a';r=read(src/'result.json');j=read(src/'job.json')
original_sha=hashlib.sha256((src/'result.json').read_bytes()).hexdigest()
uid='2c5a095faec248b8b849978b726e2b58';dest=W/'data/analysis'/uid;dest.mkdir(exist_ok=True)
groups=copy.deepcopy(j['spec']['groups']);groups[1]=copy.deepcopy(groups[2]);groups[1]['id']='B'
groups[0]['label']='入口約束 A v3.2';groups[1]['label']='筆電凍結 K r1';groups[2]['label']='Orin 離線 K r1'
j.update(id=uid,title='Orin部署驗證｜筆電與Orin完整K比較',created=time.time(),state='completed');j['spec']['groups']=groups;j.pop('validation_summary',None)
write(dest/'job.json',j);write(dest/'input.json',read(src/'input.json'))
for name in ('raw','A','B','C'):
 (dest/name).mkdir(exist_ok=True)
 for f in (src/name).iterdir():
  if f.is_file() and not (dest/name/f.name).exists():
   try:os.link(f,dest/name/f.name)
   except OSError:shutil.copy2(f,dest/name/f.name)
local=[x['A'] for x in r['models']['C']];rows=[];errors=[];mode_diff=[];source_diff=[]
for i,x in enumerate(remote['rows']):
 d=x['path_diagnostic'];v=dict(d,path=x['path'],valid=x['valid'],quality=x['quality'],path_ms=x['path_ms'],execution_host='orin',capture_s=r['frames'][i]['capture_s'],frame=r['frames'][i]['frame'])
 rows.append(v)
 a=local[i]
 if a['k']['state']!=v['k']['state']:mode_diff.append(i)
 if a['k'].get('source')!=v['k'].get('source'):source_diff.append(i)
 p=np.array(a['path']);q=np.array(v['path'])
 if len(p)>1 and len(q)>1:
  p=p[np.argsort(p[:,1])];q=q[np.argsort(q[:,1])];lo=max(p[0,1],q[0,1]);hi=min(p[-1,1],q[-1,1])
  if hi>=lo:
   ys=np.linspace(lo,hi,25);e=np.abs(np.interp(ys,p[:,1],p[:,0])-np.interp(ys,q[:,1],q[:,0]));errors.append(dict(index=i,mean=float(e.mean()),max=float(e.max())))
def events(rs):return [dict(player_s=i/10,state=x['k']['state'],source=x['k'].get('source')) for i,x in enumerate(rs) if i==0 or x['k']['state']!=rs[i-1]['k']['state']]
report=dict(release=remote['release'],job=uid,frames=len(rows),opencv=remote['opencv'],numpy=remote['numpy'],platform=remote['platform'],mode_differences=mode_diff,source_differences=source_diff,empty=sum(not x['path'] for x in rows),max_common_row_difference_px=max(x['max'] for x in errors),mean_frame_difference_px=float(np.mean([x['mean'] for x in errors])),largest_differences=sorted(errors,key=lambda x:x['max'],reverse=True)[:10],laptop_events=events(local),orin_events=events(rows),focus={str(t):dict(laptop=local[round(t*10)]['k'],orin=rows[round(t*10)]['k']) for t in (23.4,71.4,79,97.3,108.9,117.7,142)},path_ms_median=float(np.median([x['path_ms'] for x in rows])),baseline_sha256=original_sha,model_inference=False,pid_connected=False,hardware_output=False)
r['models']['B']=r['models']['C'];r['summaries']['B']=r['summaries']['C'];r['models']['C'],r['summaries']['C']=pid_rows(rows,groups[2]['parameters'],False)
r.update(id=uid,spec=j['spec'],baseline_job=src.name,orin_lane_candidate=True,partial=False)
r['limitations']=['相同保存原圖與遮罩，在Orin重新計算完整路徑與光流；不含模型推論，不是控制延遲或實車驗收。','部署至獨立版本目錄，原車控未替換或重啟；PID關閉。']
write(dest/'result.json',r);write(dest/'status.json',dict(state='completed',completed=len(rows),total=len(rows)));write(dest/'exit.json',dict(returncode=0))
assert hashlib.sha256((src/'result.json').read_bytes()).hexdigest()==original_sha
write(out/'report.json',report);print(json.dumps({k:v for k,v in report.items() if k!='focus'},ensure_ascii=False))
