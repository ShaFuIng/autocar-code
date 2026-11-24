from pathlib import Path
import base64,json,subprocess
remote='''import json,pathlib,hashlib
root=pathlib.Path('/home/jetson/orin_interface_v1');events=[]
for p in (root/'logs').glob('web_uart.jsonl*'):
 for line in p.read_text(errors='replace').splitlines():
  try:r=json.loads(line)
  except Exception:continue
  if 1792833270<=r.get('time',0)<=1792833275:events.append(r)
files={}
for n in ['web/server.py','web/manual_drive.py','Drive.cpp','Safety.cpp','ManualDrive.cpp']:
 p=root/n
 if p.exists():files[n]={'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'text':p.read_text()}
print(json.dumps({'events':sorted(events,key=lambda r:r['time']),'files':files}))
'''
args=['ssh','-i','C:/Users/User/.ssh/codex_orin_ed25519','-o','BatchMode=yes','-o','ConnectTimeout=6','-o','StrictHostKeyChecking=yes','-o','HostKeyAlias=orinnano-super.local','jetson@192.168.0.244','python3 -c "import base64;exec(base64.b64decode(\''+base64.b64encode(remote.encode()).decode()+'\'))"']
r=subprocess.run(args,capture_output=True,text=True,encoding='utf-8',timeout=20);assert r.returncode==0,r.stderr
d=json.loads(r.stdout);out=Path('Orin_Interface_v1/laptop_workbench/data/experiments/k-parallel/run-6bbe2c');(out/'uart-evidence.json').write_text(json.dumps(d,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(d['events'],ensure_ascii=False,indent=2));print('source files',list(d['files']))
