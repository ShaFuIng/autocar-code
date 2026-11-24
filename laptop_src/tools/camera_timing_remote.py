"""Explicit road-v3 deployment and stationary preview; never a driving endpoint."""
import base64,json,subprocess,sys,zipfile,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'laptop_workbench/data/experiments/control-camera-split'
REMOTE=r'''
import base64,json,sys,pathlib,subprocess,socket,urllib.request,time,hashlib,os,datetime
p=json.load(sys.stdin);root=pathlib.Path('/home/jetson/orin_interface_v1')
secret=(pathlib.Path.home()/'.config/orin_lab/token').read_text().strip()
host=None
for h in ['127.0.0.1']+subprocess.check_output(['hostname','-I'],text=True).split():
 if ':' in h:continue
 try:
  with socket.create_connection((h,8000),timeout=1):host=h;break
 except OSError:pass
base='http://'+host+':8000'
headers={'Authorization':'Basic '+base64.b64encode(('jetson:'+secret).encode()).decode(),'Origin':base,'Content-Type':'application/json','X-Orin-Control':'1'}
def api(path,data=None):
 assert data is None or path in ('/api/pid/prepare','/api/pid/preview-stop')
 req=urllib.request.Request(base+path,headers=headers,data=json.dumps(data).encode() if data is not None else None)
 with urllib.request.urlopen(req,timeout=8) as r:
  raw=r.read();return raw if path.endswith('/frame') else json.loads(raw)
def state():
 s=api('/api/status');v=s.get('vehicle') or {};pid=api('/api/pid/status')
 manual=api('/api/manual/status?compact=1').get('current');record=api('/api/record/status').get('current')
 return dict(vehicle=v,uart_age_s=s.get('uart_age_s'),synced=s.get('servo_settings_synced'),motor_busy=s.get('motor_busy'),manual=manual,record=record,pid={k:pid.get(k) for k in ('state','error','run','path_algorithm','path_algorithms','road_pid_version','ready_frames')})
def idle(s):
 assert s['vehicle']['motor_a']['pwm_command']==s['vehicle']['motor_b']['pwm_command']==0,'Wheels nonzero'
 assert not s['motor_busy'],'Motor test busy'
 assert not s['pid']['run'] or s['pid']['run']['state']!='running','PID active'
 assert not s['manual'] or s['manual'].get('status') not in ('running','starting'),'Manual active'
 assert not s['record'] or s['record'].get('status') in ('stopped','completed','failed'),'Recording active'
if p['action']=='deploy':
 before=state();idle(before)
 if before['pid']['state']!='idle':
  api('/api/pid/preview-stop',{})
  for _ in range(40):
   if api('/api/pid/status')['state'] in ('idle','failed'):break
   time.sleep(.1)
  assert api('/api/pid/status')['state'] in ('idle','failed'),'Preview has not stopped'
 stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S');backup=root/'backups'/('control_camera_split_'+stamp);backup.mkdir()
 decoded={};old={};missing=[]
 for n,h in p['manifest']['files'].items():
  assert not pathlib.PurePosixPath(n).is_absolute() and '..' not in pathlib.PurePosixPath(n).parts
  data=base64.b64decode(p['files'][n]);assert hashlib.sha256(data).hexdigest()==h
  if n.endswith('.py'):compile(data,n,'exec')
  if n in p.get('expected_before',{}):assert hashlib.sha256((root/n).read_bytes()).hexdigest()==p['expected_before'][n], 'Remote source changed: '+n
  target=root/n;decoded[n]=data
  if target.exists():
   raw=target.read_bytes();dest=backup/n;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(raw);old[n]=hashlib.sha256(raw).hexdigest()
  else:missing.append(n)
 (backup/'backup-manifest.json').write_text(json.dumps(dict(original_sha256=old,originally_missing=missing,new_sha256=p['manifest']['files']),indent=2))
 for n,data in decoded.items():
  target=root/n;target.parent.mkdir(parents=True,exist_ok=True);tmp=target.with_name(target.name+'.road-stage');tmp.write_bytes(data);os.replace(tmp,target)
 assert all(hashlib.sha256((root/n).read_bytes()).hexdigest()==h for n,h in p['manifest']['files'].items())
 subprocess.run(['systemctl','--user','restart','orin-dashboard.service'],check=True,timeout=30)
 print(json.dumps(dict(backup=str(backup),files=p['manifest']['files'],service=subprocess.check_output(['systemctl','--user','show','orin-dashboard.service','-p','ActiveState','-p','MainPID'],text=True),before=before),ensure_ascii=False))
elif p['action']=='status':print(json.dumps(state(),ensure_ascii=False))
elif p['action']=='preview':
 s=state();idle(s);assert s['pid']['state']=='idle','Preview already present'
 algorithm=p['algorithm'];assert algorithm=='junction-k-lane-v3'
 api('/api/pid/prepare',dict(model_id='v4-default',path_algorithm=algorithm,row_ratio=.6))
 seen=[];last=None;beg=time.monotonic()
 try:
  while time.monotonic()-beg<85:
   r=api('/api/pid/status');assert not r.get('run'),'Unexpected driving run'
   if r['state']=='failed':raise RuntimeError(r.get('error'))
   latest=r.get('latest')
   if latest and latest.get('seq')!=last:
    last=latest['seq'];seen.append({k:latest.get(k) for k in ('seq','frame_seq','valid','capture_at','consumed_at','control_age_s','age_s','inference_ms','sample_row_ratio','path','path_diagnostic','weights_sha256','path_algorithm')})
    if len(seen)>=10:break
   time.sleep(1)
  frame=base64.b64encode(api('/api/pid/frame')).decode() if seen else None
  print(json.dumps(dict(algorithm=algorithm,state=r['state'],error=r.get('error'),samples=seen,preview=frame,run=r.get('run')),ensure_ascii=False))
 finally:
  api('/api/pid/preview-stop',{})
  for _ in range(15):
   if api('/api/pid/status')['state'] in ('idle','failed'):break
   time.sleep(.3)
'''
action=sys.argv[1];payload=dict(action=action)
if action=='deploy':
 manifest=json.loads((OUT/'timing-manifest.json').read_text());payload.update(expected_before={},manifest=manifest,files={n:base64.b64encode((OUT/'payload'/n).read_bytes()).decode() for n in manifest['files']})
elif action=='preview':payload['algorithm']=sys.argv[2]
args=['ssh','-i','C:/Users/User/.ssh/codex_orin_ed25519','-o','BatchMode=yes','-o','ConnectTimeout=5','-o','StrictHostKeyChecking=yes','-o','HostKeyAlias=orinnano-super.local','jetson@192.168.0.244',"python3 -c \"import base64;exec(base64.b64decode('"+base64.b64encode(REMOTE.encode()).decode()+"'))\""]
r=subprocess.run(args,input=json.dumps(payload),capture_output=True,text=True,encoding='utf-8',timeout=120)
if r.returncode:print(r.stderr);raise SystemExit(r.returncode)
data=json.loads(r.stdout);name='timing-'+action+('-'+payload['algorithm'] if action=='preview' else '')
if data.get('preview'):(OUT/(name+'.jpg')).write_bytes(base64.b64decode(data.pop('preview')))
(OUT/(name+'.json')).write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
if action=='preview':print(json.dumps(dict(algorithm=data['algorithm'],state=data['state'],samples=[{k:x.get(k) for k in ('seq','valid','control_age_s','inference_ms','sample_row_ratio')} for x in data['samples']],run=data['run']),ensure_ascii=False))
else:print(json.dumps(data,ensure_ascii=False))
