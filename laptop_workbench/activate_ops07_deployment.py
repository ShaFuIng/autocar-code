"""Activate the reviewed release only after explicit service-restart authorization.

No motor/servo command or model start. Checks stopped state, exact source hashes,
and backup before stopping the service. A changed source aborts the deployment.
"""
import argparse
import json
from stage_ops07_deployment import remote, OUT

SCRIPT=r'''
import base64,hashlib,json,os,shutil,socket,subprocess,sys,time,urllib.request
from pathlib import Path
request=json.load(sys.stdin)
web=Path.home()/'orin_interface_v1/web'
release=web.parent/'ops07/releases/20261109-r5/candidates/best-today-46db'
backup=Path(request['backup'])
assert backup.resolve().is_relative_to((web.parent/'backups').resolve())
expected=request['manifest']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
current={p.name:sha(p) for p in web.iterdir() if p.is_file() and p.suffix in ('.py','.json')}
if current!=expected:raise ValueError('Remote source changed after backup; inspect before activation')
for name,digest in expected.items():
 assert sha(backup/'web'/name)==digest,'Backup mismatch: '+name
manifest=json.loads((release/'manifest.json').read_text())
for name,row in manifest['files'].items():
 assert Path(name).name==name
 assert sha(release/'web'/name)==row['after'],'Release changed: '+name
secret=(Path.home()/'.config/orin_lab/token').read_text().strip()
auth='Basic '+base64.b64encode(('jetson:'+secret).encode()).decode()
def host():
 for h in ['127.0.0.1']+subprocess.check_output(['hostname','-I'],text=True).split():
  if ':' in h:continue
  try:
   with socket.create_connection((h,8000),timeout=.5):return h
  except OSError:pass
 raise ValueError('Service unavailable')
def get(path):
 req=urllib.request.Request('http://'+host()+':8000'+path,headers={'Authorization':auth})
 with urllib.request.urlopen(req,timeout=4) as response:return json.load(response)
pid=get('/api/pid/status');manual=get('/api/manual/status?compact=1')
envelope=manual['vehicle'];vehicle=envelope['vehicle']
assert (pid.get('run') or {}).get('state')!='running','A PID run is active'
assert not envelope.get('uart_error') and envelope.get('uart_age_s',99)<=1,'Telemetry is stale'
assert vehicle.get('state') in ('LOCKED','READY','MANUAL'),'Unexpected vehicle state for activation'
current_manual=manual.get('current') or {}
if current_manual.get('status')=='running':
 assert current_manual.get('controller')=='manual','Another autonomous controller is active'
 assert all(current_manual.get('command',{}).get(k)==0 for k in ('right','left')),'Manual motion command is active'
assert all(vehicle.get(k,{}).get('pwm_command')==0 and vehicle.get(k,{}).get('target')==0 for k in ('motor_a','motor_b')),'Nonzero motor output'
before_pid=subprocess.check_output(['systemctl','--user','show','orin-dashboard.service','-p','MainPID','--value'],text=True).strip()
changed=[];new=[]
try:
 subprocess.run(['systemctl','--user','stop','orin-dashboard.service'],check=True,timeout=20)
 # Recheck after stop: a second writer must not be silently overwritten.
 assert {p.name:sha(p) for p in web.iterdir() if p.is_file() and p.suffix in ('.py','.json')}==expected
 for name,row in manifest['files'].items():
  target=web/name
  if target.exists() and sha(target)==row['after']:continue
  if not target.exists():new.append(name)
  temp=target.with_name(target.name+'.ops07-new')
  shutil.copy2(release/'web'/name,temp);os.replace(temp,target);changed.append(name)
 subprocess.run(['systemctl','--user','start','orin-dashboard.service'],check=True,timeout=20)
 state=None
 for _ in range(15):
  try:
   state=get('/api/ops07/status')
   if state.get('revision')=='ops07-orin-r3':break
  except Exception:pass
  time.sleep(.5)
 assert state and state.get('revision')=='ops07-orin-r3','New service API did not become ready'
 assert state['package_identity']==manifest['identity'],'Active identity mismatch'
 assert state['state'] in ('idle','paused') and state['hold'] is True,'Unexpected activation state'
 after={name:sha(web/name) for name in manifest['files']}
 assert all(after[name]==row['after'] for name,row in manifest['files'].items())
 result=dict(activated=True,package=manifest['package'],revision=state['revision'],backup=str(backup),
             changed=changed,new_files=new,before_pid=before_pid,
             service=subprocess.check_output(['systemctl','--user','show','orin-dashboard.service','-p','MainPID','-p','ActiveState'],text=True),
             session=state,manual=get('/api/manual/status?compact=1'),previous_manual=current_manual,motor_servo_commands=False,
             model_loaded=False,physical_test=False)
except Exception as error:
 # Restore only owned replacements; never delete another writer's data.
 subprocess.run(['systemctl','--user','stop','orin-dashboard.service'],timeout=20)
 for name in changed:
  target=web/name
  if sha(target)!=manifest['files'][name]['after']:continue
  if name in new:target.unlink()
  else:shutil.copy2(backup/'web'/name,target)
 subprocess.run(['systemctl','--user','start','orin-dashboard.service'],check=True,timeout=20)
 result=dict(activated=False,rollback=True,error=str(error),backup=str(backup),changed=changed)
print(json.dumps(result))
'''


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--restart-authorized',action='store_true')
    args=parser.parse_args()
    if not args.restart_authorized:raise SystemExit('Explicit current user restart authorization required')
    backup=json.loads((OUT/'backup.json').read_text(encoding='utf8'))
    result=remote(SCRIPT,json.dumps(backup).encode())
    (OUT/'activation-result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('manual','session')},ensure_ascii=False))
    if not result.get('activated'):raise SystemExit(1)


if __name__=='__main__':main()
