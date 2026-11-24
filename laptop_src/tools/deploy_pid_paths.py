"""Explicitly invoked deployment: back up five sources, compile, restart dashboard."""
import base64
import hashlib
import json
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[1]
names = ['web/pid_paths.py', 'web/live_pid.py', 'web/live_model_worker.py',
         'laptop_workbench/temporal_lane.py', 'laptop_workbench/curve_replay.py']
files = {name: base64.b64encode((root/name).read_bytes()).decode() for name in names}
remote = r'''
import base64,datetime,hashlib,json,os,pathlib,subprocess,sys
files=json.load(sys.stdin)
root=pathlib.Path('/home/jetson/orin_interface_v1')
stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
backup=root/'backups'/('pid_paths_'+stamp)
backup.mkdir(parents=True)
decoded={}
for name,encoded in files.items():
    data=base64.b64decode(encoded)
    compile(data,name,'exec')
    decoded[name]=data
    target=root/name
    if target.exists():
        saved=backup/name
        saved.parent.mkdir(parents=True,exist_ok=True)
        saved.write_bytes(target.read_bytes())
manifest={}
for name,data in decoded.items():
    target=root/name
    target.parent.mkdir(parents=True,exist_ok=True)
    staged=target.with_name(target.name+'.pid-path-stage')
    staged.write_bytes(data)
    os.replace(staged,target)
    manifest[name]=hashlib.sha256(target.read_bytes()).hexdigest()
(backup/'manifest.json').write_text(json.dumps(manifest,indent=2))
subprocess.run(['systemctl','--user','restart','orin-dashboard.service'],check=True,timeout=30)
print(json.dumps(dict(backup=str(backup),sha256=manifest)))
'''
encoded = base64.b64encode(remote.encode()).decode()
ssh = ['ssh','-i',str(Path.home()/'.ssh/codex_orin_ed25519'),
       '-o','BatchMode=yes','-o','ConnectTimeout=5',
       '-o','StrictHostKeyChecking=yes','-o','HostKeyAlias=orinnano-super.local',
       'jetson@172.17.37.68',
       f"python3 -c \"import base64;exec(base64.b64decode('{encoded}'))\""]
r = subprocess.run(ssh,input=json.dumps(files),text=True,capture_output=True,timeout=60)
if r.returncode:
    raise SystemExit(r.stderr)
result=json.loads(r.stdout)
assert result['sha256']=={name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in names}
(root/'laptop_workbench/data/PID_PATH_DEPLOYMENT.json').write_text(
    json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result,indent=2))
