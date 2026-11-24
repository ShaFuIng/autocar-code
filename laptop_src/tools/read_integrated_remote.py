import ast,json,subprocess,base64,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'laptop_workbench/data/experiments/k-integrated-r2'
FILES=['web/pid_paths.py','web/live_pid.py','web/live_model_worker.py','web/k_integrated_live.py','web/k_integrated_parallel.py','web/warm_color_candidate.py','web/history_gap_candidate.py','web/junction_segment_candidate.py','web/junction_branch_candidate.py','web/junction_geometry_guard.py']
tree=ast.parse((ROOT/'tools/k_parallel_remote.py').read_text(encoding='utf8'));source=next(ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='REMOTE' for t in n.targets))
source=source[:source.index("if p['action']=='deploy':")]+"\nnames="+repr(FILES)+"\nprint(json.dumps(dict(files={n:base64.b64encode((root/n).read_bytes()).decode() if (root/n).exists() else None for n in names},state=state(),service=subprocess.check_output(['systemctl','--user','show','orin-dashboard.service','-p','MainPID'],text=True))))\n"
args=['ssh','-i','C:/Users/User/.ssh/codex_orin_ed25519','-o','BatchMode=yes','-o','ConnectTimeout=5','-o','StrictHostKeyChecking=yes','-o','HostKeyAlias=orinnano-super.local','jetson@192.168.0.244',"python3 -c \"import base64;exec(base64.b64decode('"+base64.b64encode(source.encode()).decode()+"'))\""]
r=subprocess.run(args,input='{}',capture_output=True,text=True,encoding='utf8',timeout=30);assert r.returncode==0,r.stderr;d=json.loads(r.stdout);manifest={}
for name,encoded in d.pop('files').items():
 manifest[name]=None
 if encoded:
  raw=base64.b64decode(encoded);target=OUT/'remote-before'/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(raw);manifest[name]=hashlib.sha256(raw).hexdigest()
(OUT/'remote-before.json').write_text(json.dumps(dict(manifest=manifest,**d),ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps(dict(service=d['service'],pid=d['state']['pid']['state'],run=d['state']['pid']['run'],pwm=[d['state']['vehicle'][k]['pwm_command'] for k in ['motor_a','motor_b']],files=manifest)))
