import ast,base64,json,subprocess,hashlib,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'laptop_workbench/data/experiments/k-geometry-deployment'
FILES=['web/k_parallel_path.py','web/k_lane_live_path.py','web/junction_reviewed_path.py','web/junction_branch_candidate.py','web/junction_geometry_guard.py']
tree=ast.parse((ROOT/'tools/k_parallel_remote.py').read_text(encoding='utf8'));remote=next(ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='REMOTE' for t in n.targets))
remote=remote.replace("('k_parallel_'+stamp)","('k_branch_geometry_'+stamp)")
remote=remote.replace("assert s['pid']['state']=='idle','Preview already present'","assert s['pid']['state'] in ('idle','failed'),'Preview already present'")
remote=remote.replace("assert not r.get('run'),'Unexpected driving run'","assert not r.get('run') or r['run']['state']!='running','Unexpected driving run'")
remote=remote.replace("names=['web/live_model_worker.py','web/k_lane_live_path.py','web/k_lane_runtime/junction_k_lane.py']","names="+repr(FILES+['web/junction_approach_candidate.py','web/k_lane_runtime/junction_k_lane.py']))
remote=remote.replace("for n in names}","for n in names if (root/n).exists()}")
action=sys.argv[1];payload=dict(action=action)
if action=='deploy':
 parity=json.loads((OUT/'runtime-parity.json').read_text());assert all(not x['mismatches'] for x in parity.values())
 files={n:(ROOT/n).read_bytes() for n in FILES};before=json.loads((OUT/'baseline.json').read_text())
 payload.update(expected_before={n:hashlib.sha256(base64.b64decode(v)).hexdigest() for n,v in before['files'].items()},manifest={'files':{n:hashlib.sha256(v).hexdigest() for n,v in files.items()}},files={n:base64.b64encode(v).decode() for n,v in files.items()})
elif action=='preview':payload['algorithm']='junction-k-lane-v3'
args=['ssh','-i','C:/Users/User/.ssh/codex_orin_ed25519','-o','BatchMode=yes','-o','ConnectTimeout=6','-o','StrictHostKeyChecking=yes','-o','HostKeyAlias=orinnano-super.local','jetson@192.168.0.244',"python3 -c \"import base64;exec(base64.b64decode('"+base64.b64encode(remote.encode()).decode()+"'))\""]
r=subprocess.run(args,input=json.dumps(payload),capture_output=True,text=True,encoding='utf8',timeout=120)
assert r.returncode==0,r.stderr
d=json.loads(r.stdout)
if d.get('preview'):(OUT/'preview.jpg').write_bytes(base64.b64decode(d.pop('preview')))
(OUT/(action+'.json')).write_text(json.dumps(d,ensure_ascii=False,indent=2),encoding='utf8')
if action=='baseline':print(json.dumps(dict(service=d['service'],files=list(d['files']),state=d['state']['pid']['state'],run=d['state']['pid']['run'],pwm=[d['state']['vehicle'][k]['pwm_command'] for k in ['motor_a','motor_b']])))
elif action=='deploy':print(json.dumps({k:d[k] for k in ['backup','files','service']}))
elif action=='preview':print(json.dumps(dict(state=d['state'],samples=[dict(seq=x['seq'],valid=x['valid'],age=x['control_age_s'],revision=x['path_diagnostic'].get('path_revision'),parallel=x['path_diagnostic'].get('parallel')) for x in d['samples']])))
else:print(json.dumps(dict(state=d['vehicle']['state'],pwm=[d['vehicle'][k]['pwm_command'] for k in ['motor_a','motor_b']],pid=d['pid']['state'],i2c=d['vehicle'].get('i2c_errors'))))
