import ast,base64,json,subprocess,hashlib
from pathlib import Path
root=Path(__file__).resolve().parents[1];out=root/'laptop_workbench/data/experiments/k-adaptive'
tree=ast.parse((root/'tools/k_parallel_remote.py').read_text());remote=next(ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='REMOTE' for t in n.targets))
remote=remote.split("elif p['action']=='baseline':")[0]
remote=remote.replace("('k_parallel_'+stamp)","('k_adaptive_'+stamp)")
remote=remote.replace(" # Worker code is reloaded by the next preview; the vehicle service stays running."," subprocess.run(['systemctl','--user','restart','orin-dashboard.service'],check=True)")
names=['web/live_pid.py','web/k_adaptive_pid.py'];files={n:(out/'payload'/n).read_bytes() for n in names}
p=dict(action='deploy',expected_before={'web/live_pid.py':hashlib.sha256((out/'before/live_pid.py').read_bytes()).hexdigest()},manifest={'files':{n:hashlib.sha256(v).hexdigest() for n,v in files.items()}},files={n:base64.b64encode(v).decode() for n,v in files.items()})
args=['ssh','-i','C:/Users/User/.ssh/codex_orin_ed25519','-o','BatchMode=yes','-o','ConnectTimeout=6','-o','StrictHostKeyChecking=yes','-o','HostKeyAlias=orinnano-super.local','jetson@192.168.0.244',"python3 -c \"import base64;exec(base64.b64decode('"+base64.b64encode(remote.encode()).decode()+"'))\""]
r=subprocess.run(args,input=json.dumps(p),capture_output=True,text=True,encoding='utf-8',timeout=90);assert r.returncode==0,r.stderr
d=json.loads(r.stdout);(out/'deployment.json').write_text(json.dumps(d,ensure_ascii=False,indent=2),encoding='utf-8');print(d['backup']);print(d['service'])
