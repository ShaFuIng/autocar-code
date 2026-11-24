"""Local offline job registry. Never imports hardware or changes deployed weights."""
import json, re, subprocess, threading, time, uuid, os
from pathlib import Path
ROOT=Path(__file__).resolve().parent
PY=Path.home()/'AppData/Local/Programs/Python/Python310/python.exe'
CONFIGS=[('v4-640','yolo26_sem_v4_trial/runs/trial01/weights/best.pt',640),('v4-960','yolo26_sem_v4_trial/runs/trial01/weights/best.pt',960),('v3-640','yolo26_sem_v3_trial/runs/trial01/weights/best.pt',640),('v5-focus02','yolo26_sem_v5_focus70/runs/focus02/weights/best.pt',640),('v5-focus03','yolo26_sem_v5_focus70/runs/focus03/weights/best.pt',960),('opencv-local',None,0),('tiny-unet','Orin_Interface_v1/laptop_workbench/data/experiments/night-20261018/tiny-unet.pt',640),('fast-scnn-inspired','Orin_Interface_v1/laptop_workbench/data/experiments/night-20261018/fast-scnn-inspired.pt',640)]
def alive(pid):
 if not pid:return False
 if os.name=='nt':
  import ctypes
  from ctypes import wintypes
  k=ctypes.WinDLL('kernel32',use_last_error=True)
  k.OpenProcess.argtypes=[wintypes.DWORD,wintypes.BOOL,wintypes.DWORD];k.OpenProcess.restype=wintypes.HANDLE
  k.GetExitCodeProcess.argtypes=[wintypes.HANDLE,ctypes.POINTER(wintypes.DWORD)]
  k.CloseHandle.argtypes=[wintypes.HANDLE]
  h=k.OpenProcess(0x1000,False,pid)
  if not h:return False
  code=wintypes.DWORD();ok=k.GetExitCodeProcess(h,ctypes.byref(code));k.CloseHandle(h);return bool(ok and code.value==259)
 return Path('/proc') .joinpath(str(pid)).exists()
def read(p):return json.loads(p.read_text(encoding='utf-8'))
def write(p,d):
 t=p.with_name(p.name+'.'+uuid.uuid4().hex+'.tmp');t.write_text(json.dumps(d,ensure_ascii=False),encoding='utf-8')
 for attempt in range(30):
  try:os.replace(t,p);return
  except PermissionError:
   if attempt==29:raise
   time.sleep(.1)
class Analysis:
 def __init__(self,app):
  self.app=app;self.root=app.store.root/'analysis';self.root.mkdir(exist_ok=True);self.lock=threading.Lock();self.process=None
 def sources(self):
  self.app.index_synced();return [dict(id=k,title=v['title'],frames=len(v['rows']) if 'rows' in v else v.get('frames_hint')) for k,v in self.app.replays.items() if v.get('has_images') or v['kind']=='shadow']
 def models(self):return [dict(id=n,available=p is None or (ROOT.parents[1]/p).is_file(),kind='傳統影像處理' if p is None else '白線分割模型') for n,p,s in CONFIGS]
 def jobs(self):
  rows=[]
  for p in self.root.glob('*/job.json'):
   d=read(p);status=p.parent/'status.json'
   if status.exists():d.update(read(status))
   if d.get('state') in ('queued','running') and ((p.parent/'exit.json').exists() or not alive(d.get('pid'))):d.update(state='failed',error='分析程序提前結束，請查看工作日誌並重新建立任務')
   rows.append(d)
  return sorted(rows,key=lambda d:d['created'],reverse=True)
 def start(self,p):
  with self.lock:
   if (self.process and self.process.poll() is None) or any(j['state'] in ('queued','running') for j in self.jobs()):raise ValueError('已有離線分析執行中，請等候完成')
   source=p.get('source');models=p.get('models');catalog={m['id'] for m in self.models() if m['available']}
   if not isinstance(models,list) or not 1<=len(models)<=8 or len(set(models))!=len(models) or any(m not in catalog for m in models):raise ValueError('請選擇有效且不重複的模型')
   if source not in {s['id'] for s in self.sources()}:raise ValueError('請選擇有影像的已同步影片')
   self.app.ensure_replay(source);r=self.app.replays[source];frames=[]
   for i,row in enumerate(r['rows']):
    f=self.app.assets.get(source+'/'+str(row.get('frame',i)).zfill(6)+'.jpg')
    if not f or not f.is_file():raise ValueError('來源影格不完整，請重新同步')
    entry=dict(image=str(f.resolve()),capture_s=row.get('capture_s',0),frame=i)
    if row.get('exact_evidence'):
     v=row['vision'];entry['live_evidence']=dict(mask=str((r['folder']/v['mask_file']).resolve()),mask_sha256=v['mask_sha256'],raw_sha256=v['raw_sha256'],weights_sha256=v['weights_sha256'],recognition=v.get('recognition',{}),origin='live_pid_original')
    frames.append(entry)
   spec=None
   if p.get('kind')=='shadow':
    from shadow_offline import specification
    spec=specification(self.app,p,len(frames))
    models=list(dict.fromkeys(g['model'] for g in spec['groups']))
    if any(m not in catalog for m in models):raise ValueError('所選V4權重不存在')
    frames=frames[spec['start']:spec['end']+1:spec['stride']]
   uid=uuid.uuid4().hex;folder=self.root/uid;folder.mkdir();job=dict(id=uid,source=source,title=r['title'],models=models,created=time.time(),state='queued',execution='筆電 GPU／純離線；不更動 Orin',total=len(frames)*len(models))
   if spec:job.update(kind='shadow',spec=spec,total=len(frames)*(len(models)+len(spec['groups'])))
   write(folder/'job.json',job);write(folder/'input.json',frames)
   log=(folder/'worker.log').open('wb')
   self.process=subprocess.Popen([str(PY),str(ROOT/('shadow_worker.py' if spec else 'analysis_worker.py')),str(folder)],stdout=log,stderr=subprocess.STDOUT,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0));log.close()
   job['pid']=self.process.pid;write(folder/'job.json',job)
   proc=self.process
   def wait():write(folder/'exit.json',dict(returncode=proc.wait()))
   threading.Thread(target=wait,daemon=True).start();return job
 def result(self,uid):
  if not re.fullmatch('[0-9a-f]{32}',uid):raise ValueError('Invalid job')
  return read(self.root/uid/'result.json')
 def cancel(self,p):
  uid=p.get('job','')
  if not re.fullmatch('[0-9a-f]{32}',uid):raise ValueError('Invalid job')
  folder=self.root/uid
  if read(folder/'job.json').get('kind')!='shadow':raise ValueError('此入口僅取消新離線影子工作')
  write(folder/'cancel.json',dict(requested=time.time()))
  return dict(requested=True,job=uid)
 def reviews(self,uid):
  self.result(uid)
  with self.app.store.lock:
   return [json.loads(r[0]) for r in self.app.store.db.execute('SELECT body FROM documents WHERE key LIKE ? ORDER BY rowid DESC',('analysis-review-'+uid+'-%',))]
 def review(self,p):
  result=self.result(p['job']);a=p.get('start');b=p.get('end')
  if type(a)!=int or type(b)!=int or not 0<=a<=b<len(result['frames']):raise ValueError('評語區間超出影片範圍')
  if p.get('model') not in result['models'] or p.get('target') not in ['mask','A','B','C']:raise ValueError('評語對象無效')
  if p.get('verdict') not in ['未確認','正確','錯誤','不確定'] or not isinstance(p.get('notes'),str) or not 1<=len(p['notes'].strip())<=5000:raise ValueError('請填寫評語（最多5000字）')
  item={k:p[k] for k in ['job','start','end','model','target','verdict','notes']};item.update(id=uuid.uuid4().hex,created=time.time(),source=result['source'])
  self.app.store.save('analysis-review-'+p['job']+'-'+item['id'],0,item);return item
