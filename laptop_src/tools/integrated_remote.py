import ast,sys,base64,json,subprocess,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'laptop_workbench/data/experiments/k-integrated-r2'
action=sys.argv[1];payload=dict(action=action)
source0=ast.parse((ROOT/'tools/k_parallel_remote.py').read_text(encoding='utf8'));remote=next(ast.literal_eval(n.value) for n in source0.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='REMOTE' for t in n.targets));remote=remote[:remote.index("if p['action']=='deploy':")]
remote+=r'''
stage=root/'offline_experiments/k_integrated_r2_20261028'
if p['action']=='status':
 print(json.dumps(state()))
elif p['action']=='stage':
 stage.mkdir(parents=True,exist_ok=True)
 for name,raw in p['files'].items():
  assert '..' not in pathlib.PurePosixPath(name).parts and not name.startswith('/')
  dest=stage/name;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(base64.b64decode(raw))
 print(json.dumps(dict(staged=str(stage),files=list(p['files']),service=subprocess.check_output(['systemctl','--user','show','orin-dashboard.service','-p','MainPID'],text=True))))
elif p['action']=='probe':
 cmd=['docker','run','--rm','--network','none','--read-only','--tmpfs','/tmp','-e','PYTHONDONTWRITEBYTECODE=1','-v',str(stage)+':'+str(stage)+':ro','-v',str(root/'web')+':'+str(root/'web')+':ro','orin-notebook:v1','python3',str(stage/'probe.py')]
 run=subprocess.run(cmd,capture_output=True,text=True,timeout=150)
 assert run.returncode==0,run.stderr
 print(run.stdout)
elif p['action']=='restart':
 s=state();idle(s);assert s['pid']['state'] in ('idle','failed')
 before=subprocess.check_output(['systemctl','--user','show','orin-dashboard.service','-p','MainPID'],text=True)
 subprocess.run(['systemctl','--user','restart','orin-dashboard.service'],check=True)
 for _ in range(15):
  time.sleep(1)
  try:
   after=state()
   if any(a['id']=='junction-k-integrated-r2' for a in after['pid']['path_algorithms']):break
  except Exception:continue
 else:raise RuntimeError('r2 service not ready')
 idle(after)
 print(json.dumps(dict(before=before,service=subprocess.check_output(['systemctl','--user','show','orin-dashboard.service','-p','MainPID','-p','ActiveState'],text=True),state=after)))
elif p['action']=='preview':
 s=state();idle(s);assert s['pid']['state']=='idle'
 api('/api/pid/prepare',dict(model_id='v4-default',path_algorithm='junction-k-integrated-r2',row_ratio=.6,curve_outward_px_640=0))
 seen=[];last=None;beg=time.monotonic()
 try:
  while time.monotonic()-beg<100:
   r=api('/api/pid/status');assert not r.get('run') or r['run']['state']!='running'
   if r['state']=='failed':raise RuntimeError(r.get('error'))
   latest=r.get('latest')
   if latest and latest.get('seq')!=last:
    last=latest['seq'];seen.append({k:latest.get(k) for k in ('seq','valid','capture_at','consumed_at','control_age_s','inference_ms','path_algorithm','path_diagnostic','path')})
    if len(seen)>=12:break
   time.sleep(.5)
  assert seen,'No preview frames'
  frame=base64.b64encode(api('/api/pid/frame')).decode()
  print(json.dumps(dict(state=r['state'],samples=seen,preview=frame,run=r.get('run')),ensure_ascii=False))
 finally:api('/api/pid/preview-stop',{})
elif p['action']=='deploy':
 s=state();idle(s);assert s['pid']['state'] in ('idle','failed'),'Existing preview retained; do not interrupt'
 for n,h in p['expected'].items():
  target=root/n
  if h is None:assert not target.exists(),'New file already exists: '+n
  else:assert hashlib.sha256(target.read_bytes()).hexdigest()==h,'Source changed: '+n
 backup=root/'backups'/('k_integrated_r2_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'));backup.mkdir()
 for n,h in p['manifest'].items():
  data=(stage/'payload'/n).read_bytes();assert hashlib.sha256(data).hexdigest()==h;compile(data,n,'exec');target=root/n
  if target.exists():
   old=backup/n;old.parent.mkdir(parents=True,exist_ok=True);old.write_bytes(target.read_bytes())
  target.parent.mkdir(parents=True,exist_ok=True);tmp=target.with_name(target.name+'.r2-stage');tmp.write_bytes(data);os.replace(tmp,target)
 (backup/'manifest.json').write_text(json.dumps(dict(before=p['expected'],after=p['manifest']),indent=2))
 print(json.dumps(dict(backup=str(backup),files=p['manifest'],restart_performed=False,state=state(),service=subprocess.check_output(['systemctl','--user','show','orin-dashboard.service','-p','MainPID'],text=True))))
'''
if action=='stage':
 files=list((OUT/'payload').rglob('*.py'))+[OUT/'probe.py',OUT/'fixtures.npz'];payload['files']={str(f.relative_to(OUT)).replace('\\','/'):base64.b64encode(f.read_bytes()).decode() for f in files}
if action=='deploy':
 report=json.loads((OUT/'report.json').read_text());assert report['status']=='complete' and all(not x['path_differences'] and not x['mode_differences'] for x in report['clips'])
 probe=json.loads((OUT/'remote-probe.json').read_text());assert probe['passed'];payload['manifest']=json.loads((OUT/'manifest.json').read_text());before=json.loads((OUT/'remote-before.json').read_text());payload['expected']={k:before['manifest'][k] for k in payload['manifest']}
args=['ssh','-i','C:/Users/User/.ssh/codex_orin_ed25519','-o','BatchMode=yes','-o','ConnectTimeout=5','-o','StrictHostKeyChecking=yes','-o','HostKeyAlias=orinnano-super.local','jetson@192.168.0.244',"python3 -c \"import base64;exec(base64.b64decode('"+base64.b64encode(remote.encode()).decode()+"'))\""]
r=subprocess.run(args,input=json.dumps(payload),capture_output=True,text=True,encoding='utf8',timeout=180);assert r.returncode==0,r.stderr;d=json.loads(r.stdout)
if d.get('preview'):(OUT/'stationary-preview.jpg').write_bytes(base64.b64decode(d.pop('preview')))
(OUT/('remote-'+action+'.json')).write_text(json.dumps(d,ensure_ascii=False,indent=2),encoding='utf8')
if action=='status':print(json.dumps(dict(pid=d['pid']['state'],run=d['pid']['run'],pwm=[d['vehicle'][k]['pwm_command'] for k in ['motor_a','motor_b']],vehicle_state=d['vehicle'].get('state'),i2c=d['vehicle'].get('i2c_errors'))))
elif action=='preview':print(json.dumps(dict(state=d['state'],samples=[dict(seq=x['seq'],valid=x['valid'],age=x['control_age_s']) for x in d['samples']])))
elif action=='restart':print(json.dumps(dict(before=d['before'],service=d['service'],pid=d['state']['pid']['state'],pwm=[d['state']['vehicle'][k]['pwm_command'] for k in ['motor_a','motor_b']])) )
elif action=='probe':print(json.dumps({k:v for k,v in d.items() if k!='rows'}))
elif action=='deploy':print(json.dumps(dict(backup=d['backup'],service=d['service'],restart=False,pid=d['state']['pid']['state'])))
else:print(json.dumps(d))
