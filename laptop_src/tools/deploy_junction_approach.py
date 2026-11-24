"""Deploy only reviewed worker files, then stationary preview. No PID start API."""
import ast,base64,json,subprocess,hashlib,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'laptop_workbench/data/experiments/k-approach'
tree=ast.parse((ROOT/'tools/k_parallel_remote.py').read_text(encoding='utf8'))
remote=next(ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='REMOTE' for t in n.targets))
remote=remote.replace("('k_parallel_'+stamp)","('k_approach_'+stamp)")
remote=remote.replace("assert s['pid']['state']=='idle','Preview already present'", "assert s['pid']['state'] in ('idle','failed'),'Preview already present'")
remote=remote.replace("assert not r.get('run'),'Unexpected driving run'", "assert not r.get('run') or r['run']['state']!='running','Unexpected driving run'")
action=sys.argv[1];payload=dict(action=action)
if action=='deploy':
 files={'web/'+name:(OUT/'candidate-runtime'/name).read_bytes() for name in ['k_parallel_path.py','junction_approach_candidate.py','k_lane_live_path.py']}
 payload.update(expected_before={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in ['web/k_parallel_path.py','web/k_lane_live_path.py']},manifest={'files':{n:hashlib.sha256(b).hexdigest() for n,b in files.items()}},files={n:base64.b64encode(b).decode() for n,b in files.items()})
elif action=='preview':payload['algorithm']='junction-k-lane-v3'
elif action!='status':raise ValueError(action)
args=['ssh','-i','C:/Users/User/.ssh/codex_orin_ed25519','-o','BatchMode=yes','-o','ConnectTimeout=6','-o','StrictHostKeyChecking=yes','-o','HostKeyAlias=orinnano-super.local','jetson@192.168.0.244',"python3 -c \"import base64;exec(base64.b64decode('"+base64.b64encode(remote.encode()).decode()+"'))\""]
r=subprocess.run(args,input=json.dumps(payload),capture_output=True,text=True,encoding='utf8',timeout=120)
assert r.returncode==0,r.stderr
d=json.loads(r.stdout)
if d.get('preview'):(OUT/'stationary-preview.jpg').write_bytes(base64.b64decode(d.pop('preview')))
(OUT/(action+'-orin.json')).write_text(json.dumps(d,ensure_ascii=False,indent=2),encoding='utf8')
if action=='deploy':print(json.dumps({k:d[k] for k in ['backup','service','files']}))
elif action=='preview':
 print(json.dumps(dict(state=d['state'],run=d['run'],samples=[dict(seq=x['seq'],valid=x['valid'],revision=x['path_diagnostic'].get('path_revision'),k_revision=x['path_diagnostic']['k'].get('revision'),age=x['control_age_s']) for x in d['samples']])))
else:
 v=d['vehicle'];print(json.dumps(dict(pid=d['pid']['state'],run=d['pid']['run'],state=v['state'],pwm=[v['motor_a']['pwm_command'],v['motor_b']['pwm_command']],i2c=v['i2c_errors'],pca=v['pca_ok'])))
