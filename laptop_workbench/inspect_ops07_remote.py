"""Read-only deployment inspection. Credentials remain in the remote process."""
import json
import subprocess
from pathlib import Path

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'data/experiments/ops07-deployment-20261109'
SSH=['ssh','-i',str(Path.home()/'.ssh/codex_orin_ed25519'),'-o','BatchMode=yes',
     '-o','ConnectTimeout=5','-o','StrictHostKeyChecking=yes','-o','HostKeyAlias=orinnano-super.local']
REMOTE=r'''
import base64,hashlib,json,socket,subprocess,urllib.request,shutil
from pathlib import Path
root=Path.home()/'orin_interface_v1/web'
manifest={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in root.iterdir() if p.is_file() and p.suffix in ('.py','.json')}
service=subprocess.check_output(['systemctl','--user','show','orin-dashboard.service','-p','MainPID','-p','ActiveState','-p','SubState','-p','ExecMainStartTimestamp'],text=True)
secret=(Path.home()/'.config/orin_lab/token').read_text().strip()
auth='Basic '+base64.b64encode(('jetson:'+secret).encode()).decode()
host=None
for h in ['127.0.0.1']+subprocess.check_output(['hostname','-I'],text=True).split():
 if ':' in h:continue
 try:
  with socket.create_connection((h,8000),timeout=1):pass
  host=h;break
 except OSError:pass
def get(path):
 req=urllib.request.Request('http://'+host+':8000'+path,headers={'Authorization':auth})
 with urllib.request.urlopen(req,timeout=5) as response:return json.load(response)
pid=get('/api/pid/status');manual=get('/api/manual/status?compact=1')
envelope=manual.get('vehicle') or {};vehicle=envelope.get('vehicle',envelope)
out=dict(manifest=manifest,service=service,disk_free=shutil.disk_usage(root).free,
 pid={k:pid.get(k) for k in ('state','model_id','run','profile','path_algorithm','ready_frames')},
 manual={k:manual.get(k) for k in ('active','run','current')},
 vehicle={k:vehicle.get(k) for k in ('state','reason','generation','ms','pca_ok','i2c_errors','motor_a','motor_b','steer_us_command','steer_output_enabled','servo_profile','fw')},
 dist_ff=json.loads((root/'dist_ff.json').read_text()) if (root/'dist_ff.json').exists() else None)
print(json.dumps(out))
'''

def inspect():
    OUT.mkdir(parents=True,exist_ok=True)
    preferred=json.loads((ROOT/'data/orin/connection.json').read_text())['host']
    for host in dict.fromkeys([preferred,'orinnano-super.local','192.168.55.1']):
        run=subprocess.run(SSH+['jetson@'+host,'python3 -'],input=REMOTE,text=True,capture_output=True,timeout=25,encoding='utf8')
        if run.returncode:
            print(host,run.stderr.strip()[-250:]);continue
        data=json.loads(run.stdout);data['ssh_host']=host
        (OUT/'before.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf8')
        print(json.dumps(dict(host=host,service=data['service'],files=len(data['manifest']),pid_state=data['pid']['state'],
                             run_state=(data['pid']['run'] or {}).get('state'),vehicle=data['vehicle']),ensure_ascii=False))
        return data
    raise RuntimeError('Known SSH endpoints unavailable')

if __name__=='__main__':inspect()
