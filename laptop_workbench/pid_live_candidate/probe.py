"""Read-only camera/model probe. Fake manual controller makes actuation impossible."""
import base64,json,time,threading,urllib.request
from pathlib import Path
from live_pid import LivePID
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
def read(path):
    with urllib.request.urlopen(urllib.request.Request(base+path,headers=headers),timeout=3) as r:return r.read()
def camera():
    status=json.loads(read('/api/status'))
    return read('/camera.jpg'),time.monotonic()-status['camera_age_s'],status.get('camera_seq',0)
class NoActuators:
    root=Path(__file__).resolve().parent/'probe_logs'
    root.mkdir(exist_ok=True)
    def profile(self):return {'center_us':1300,'min_us':500,'max_us':2500}
    def busy(self):return False
    def start(self,*args,**kwargs):raise AssertionError('No actuator adapter in probe')
    def request_stop(self,*args):raise AssertionError('No actuator adapter in probe')
live=LivePID(NoActuators(),camera,threading.RLock(),lambda:False)
live.prepare({'model_id':'v4-default','row_ratio':.75})
rows=[];seen=None
for _ in range(120):
    time.sleep(1);s=live.status();r=s.get('latest')
    if r and r['seq']!=seen:
        seen=r['seq'];rows.append(r);print(json.dumps({'state':s['state'],'seq':seen,'age_s':r['age_s'],'ms':r['inference_ms'],'quality':r['quality'],'error':r['error']}),flush=True)
    if s['state']=='failed' or len(rows)>=12:break
live.stop_preview()
if live.worker:live.worker.join(15)
result={'state':s['state'],'error':s['error'],'frames':rows,'actuator_output':False}
(Path(__file__).parent/'probe_result.json').write_text(json.dumps(result,indent=2))
print(json.dumps({'state':s['state'],'error':s['error'],'frames':len(rows),'actuator_output':False}),flush=True)
