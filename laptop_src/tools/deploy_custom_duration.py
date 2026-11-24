import base64,json,subprocess
from pathlib import Path
remote = r"""
import pathlib,datetime,json,hashlib,subprocess
root=pathlib.Path('/home/jetson/orin_interface_v1')
changes={
 'web/live_pid.py':[('not 1<=seconds<=20','seconds<1'),('Duration 1..20 seconds','Duration must be finite and at least 1 second'),('max_duration_s=20','max_duration_s=None,custom_duration_version=1')],
 'web/manual_drive.py':[('not 1<=seconds<=20','seconds<1'),('PID duration must be 1..20 seconds','PID duration must be finite and at least 1 second')]
}
prepared={}
for name,replacements in changes.items():
 old=(root/name).read_text()
 new=old
 for a,b in replacements:
  if new.count(a)!=1:raise RuntimeError('Unexpected source: '+name+' '+a)
  new=new.replace(a,b)
 compile(new,name,'exec');prepared[name]=(old,new)
backup=root/'backups'/('custom_duration_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
backup.mkdir(parents=True)
for name,(old,new) in prepared.items():
 dest=backup/name;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_text(old)
 (root/name).write_text(new)
subprocess.run(['systemctl','--user','restart','orin-dashboard.service'],check=True,timeout=30)
print(json.dumps({'backup':str(backup),'sha256':{name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in prepared},'service':subprocess.check_output(['systemctl','--user','show','orin-dashboard.service','-p','ActiveState','-p','MainPID'],text=True)}))
"""
ssh=['ssh','-i','C:/Users/User/.ssh/codex_orin_ed25519','-o','BatchMode=yes','-o','ConnectTimeout=5','-o','StrictHostKeyChecking=yes','-o','HostKeyAlias=orinnano-super.local','jetson@172.17.37.68',"python3 -c \"import base64;exec(base64.b64decode('"+base64.b64encode(remote.encode()).decode()+"'))\""]
r=subprocess.run(ssh,capture_output=True,text=True,timeout=50)
if r.returncode:raise SystemExit(r.stderr)
result=json.loads(r.stdout)
Path('Orin_Interface_v1/laptop_workbench/data/experiments/pid-custom-duration/deployment.json').write_text(json.dumps(result,indent=2),encoding='utf8')
print(r.stdout)
