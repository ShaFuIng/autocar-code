"""Run only after explicit restart authorization. No firmware/model replacement."""
import base64,hashlib,json,os,py_compile,shutil,subprocess,time,urllib.request
from pathlib import Path
stage=Path(__file__).resolve().parent
web=Path.home()/'orin_interface_v1/web'
expected={'manual_drive.py':'7f6070282c1aafde9a5e66844b4376e051105099fe6681f852a8ca3e0ebc0fe2',
          'server.py':'cd6e45e4511890e735e3e380a007bd3619b35e63354c4614c99077871775eed9'}
files=['visual_pid.py','live_model_worker.py','pid_models.json','live_pid.py','manual_drive.py','server.py']
for name,digest in expected.items():
    if hashlib.sha256((web/name).read_bytes()).hexdigest()!=digest:raise RuntimeError('Upstream changed: '+name)
for name in files:
    if name.endswith('.py'):py_compile.compile(str(stage/name),doraise=True)
secret=(Path.home()/'.config/orin_lab/token').read_text().strip()
import socket,subprocess
base=None
for host in ['127.0.0.1']+subprocess.check_output(['hostname','-I'],text=True).split():
    if ':' in host:continue
    try:
        with socket.create_connection((host,8000),timeout=.5):pass
        base='http://'+host+':8000';break
    except OSError:pass
if base is None:raise RuntimeError('Dashboard unavailable')
headers={'Authorization':'Basic '+base64.b64encode(('jetson:'+secret).encode()).decode()}
with urllib.request.urlopen(urllib.request.Request(base+'/api/status',headers=headers),timeout=3) as r:status=json.load(r)
v=status['vehicle']
if status['uart_age_s']>1 or v['state']!='LOCKED' or any(v[k]['pwm_command'] for k in ('motor_a','motor_b')):raise RuntimeError('Require fresh LOCKED and zero motors')
backup=web.parent/('backup_pid_'+time.strftime('%Y%m%d_%H%M%S'));backup.mkdir()
for name in files:
    if (web/name).exists():shutil.copy2(web/name,backup/name)
(backup/'before.json').write_text(json.dumps({'state':v['state'],'files':files,'created':time.time()}))
for name in files:
    tmp=web/(name+'.pid-new');shutil.copy2(stage/name,tmp);os.replace(tmp,web/name)
subprocess.run(['systemctl','--user','restart','orin-dashboard.service'],check=True)
print(json.dumps({'backup':str(backup),'restarted':'orin-dashboard.service','model_replaced':False,'firmware_flashed':False}),flush=True)
