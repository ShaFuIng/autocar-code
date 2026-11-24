"""Explicitly authorized single 10s 15-percent PID test; never retries a start."""
import json,time,uuid,threading,urllib.request
from pathlib import Path
base='http://127.0.0.1:8093'
def api(path,data=None):
    req=urllib.request.Request(base+path,data=None if data is None else json.dumps(data).encode(),headers={'Content-Type':'application/json','X-Workbench':'1'})
    with urllib.request.urlopen(req,timeout=3) as r:return json.load(r)
root=Path(__file__).resolve().parents[1]/'data'
s=api('/api/orin/status')
assert s['connected'] and s['vehicle']['state']=='LOCKED' and all(s['vehicle'][k]['pwm_command']==0 for k in ('motor_a','motor_b'))
live=api('/api/pid/live/status')
if live['state']=='idle':api('/api/pid/live/prepare',{'model_id':'v4-default','row_ratio':.75})
deadline=time.monotonic()+90
while True:
    live=api('/api/pid/live/status')
    if live['state']=='ready' and live['ready_frames']>=3 and live['latest']['age_s']<.4:break
    if live['state']=='failed' or time.monotonic()>deadline:raise RuntimeError('No fresh reliable path; no start sent')
    time.sleep(.5)
doc=api('/api/document/visual-pid')
(root/'pid-before-forward-test.json').write_text(json.dumps(doc,indent=2))
zero=json.loads(json.dumps(doc['data']));zero['parameters']['pwm']=15
saved=api('/api/document',dict(key='visual-pid',rev=doc['rev'],data=zero))
owner=uuid.uuid4().hex;rows=[];errors=[];done=threading.Event();run=None
try:
    run=api('/api/pid/live/start',dict(owner=owner,seconds=10,area_clear=True,parameter_revision=saved['rev']))
    print(json.dumps({'started':run,'pwm':15,'seconds':10}),flush=True)
    def heartbeat():
        seq=0
        while not done.wait(.15):
            seq+=1
            try:api('/api/pid/live/heartbeat',dict(owner=owner,run_id=run['run_id'],seq=seq))
            except Exception as e:errors.append(str(e));return
    threading.Thread(target=heartbeat,daemon=True).start()
    until=time.monotonic()+11
    while time.monotonic()<until:
        live=api('/api/pid/live/status');s=api('/vehicle/api/status')
        rows.append({'wall':time.time(),'pid':live,'vehicle':s})
        if any(not 0<=s['vehicle'][k]['pwm_command']<=39 for k in ('motor_a','motor_b')):raise RuntimeError('PWM exceeds authorized 15 percent')
        if live.get('run',{}).get('state')=='stopped':break
        time.sleep(.15)
finally:
    done.set();stop=api('/api/pid/live/stop',{})
    time.sleep(.5);final=api('/api/orin/status')
    result={'run':run,'saved_revision':saved['rev'],'samples':rows,'heartbeat_errors':errors,'stop':stop,'final':final}
    (root/'pid-forward-15pct-10s-test.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'run':run,'samples':len(rows),'reason':(stop.get('run') or {}).get('reason'),'final_state':final['vehicle']['state'],'pwm':[final['vehicle'][k]['pwm_command'] for k in ('motor_a','motor_b')],'heartbeat_errors':errors}),flush=True)
