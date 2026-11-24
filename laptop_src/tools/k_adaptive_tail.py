import base64,json,subprocess
from pathlib import Path
root=Path(__file__).resolve().parents[1];out=root/'laptop_workbench/data/experiments/k-adaptive'
code='''import sys,json,base64,contextlib
sys.path.insert(0,'/web')
import cv2
from live_model_worker import V4
from pathlib import Path
spec=json.loads(Path('/web/pid_models.json').read_text())['v4-default']
with contextlib.redirect_stdout(sys.stderr):model=V4(Path(spec['weights']))
cap=cv2.VideoCapture('/run/video_002.avi');rows=[]
for i in range(283,287):
 cap.set(cv2.CAP_PROP_POS_FRAMES,i);ok,im=cap.read()
 if not ok:continue
 im=cv2.resize(im,(640,360))
 with contextlib.redirect_stdout(sys.stderr):mask=model.predict(im)
 rows.append(dict(frame=i,mask=base64.b64encode(cv2.imencode('.png',mask.astype('uint8')*255)[1]).decode()))
print(json.dumps(rows))
'''
remote='''import subprocess,json,pathlib,sys
root=pathlib.Path('/home/jetson/orin_interface_v1');spec=json.loads((root/'web/pid_models.json').read_text())['v4-default'];weights=spec['weights']
cmd=['docker','run','--rm','--network','none','--runtime=nvidia','--cap-drop','ALL','--security-opt','no-new-privileges:true','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-v',str(root/'web')+':/web:ro','-v',weights+':'+weights+':ro','-v',str(root/'logs/manual_driving/db7089a3c4a246c1a7c8595779681d00')+':/run:ro','orin-notebook:v1','python3','-c',CODE]
r=subprocess.run(cmd,capture_output=True,text=True,timeout=100);print(r.stderr,file=sys.stderr);assert r.returncode==0,r.stdout;print(r.stdout)
'''.replace('CODE',repr(code))
args=['ssh','-i','C:/Users/User/.ssh/codex_orin_ed25519','-o','BatchMode=yes','-o','ConnectTimeout=6','-o','StrictHostKeyChecking=yes','-o','HostKeyAlias=orinnano-super.local','jetson@192.168.0.244',"python3 -c \"import base64;exec(base64.b64decode('"+base64.b64encode(remote.encode()).decode()+"'))\""]
r=subprocess.run(args,capture_output=True,text=True,encoding='utf-8',timeout=120);assert r.returncode==0,r.stderr
rows=json.loads(r.stdout);(out/'tail-masks.json').write_text(json.dumps(rows));print('Offline masks',len(rows))

