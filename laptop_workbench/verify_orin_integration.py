"""End-to-end acceptance through laptop API; only camera-only recording is mutated."""
import json,time,urllib.request,urllib.error,uuid
from pathlib import Path
ROOT=Path(__file__).resolve().parent;BASE='http://127.0.0.1:8093'
def request(path,data=None):
    headers={'X-Workbench':'1','Origin':BASE,'Content-Type':'application/json'}
    req=urllib.request.Request(BASE+path,data=json.dumps(data).encode() if data is not None else None,headers=headers)
    with urllib.request.urlopen(req,timeout=90) as r:
        body=r.read();return body if path.endswith('.jpg') else json.loads(body)
def check(s):
    assert s['connected'],s
    v=s['vehicle'];assert v['state']=='LOCKED',v
    for k in ('motor_a','motor_b'):assert v[k]['pwm_command']==v[k]['target']==0,v
before=request('/api/orin/status');check(before)
result={'before':before,'tests':[]}
def passed(name):result['tests'].append(name);print(name,flush=True)
photo=request('/orin-camera.jpg');assert photo[:2]==b'\xff\xd8';passed('real camera JPEG')
for path,data in [('/api/orin/record/start',{'job':uuid.uuid4().hex,'duration_s':0}),('/api/orin/record/stop',{'job':'../escape'}),('/api/orin/motor/start',{})]:
    try:request(path,data);raise AssertionError('unsafe request accepted')
    except urllib.error.HTTPError as e:assert e.code==400
passed('duration, ownership ID and unsupported motion API rejected')
existing=(before.get('recording') or {}).get('current') or {}
if existing.get('status')=='recording':
    result['recording_test']='skipped: existing recording left untouched'
else:
    job=uuid.uuid4().hex;row=request('/api/orin/record/start',dict(job=job,duration_s=10));assert row['job']==job
    # Idempotent retry with the exact same request key must not create a second recording.
    repeat=request('/api/orin/record/start',dict(job=job,duration_s=10));assert repeat['job']==job
    deadline=time.time()+60
    while time.time()<deadline:
        time.sleep(3);s=request('/api/orin/status');check(s);row=next((j for j in s['jobs'] if j['job']==job),{})
        if row.get('state') in ('failed','ownership_changed'):raise AssertionError(row)
        if row.get('state')=='completed':break
    assert row.get('state')=='completed' and row['recording']['frames']>0,row
    result['recording_job']=row;passed('10-second Orin recording self-terminated with frames, idempotent job ID')
catalogue=request('/api/orin/discover',{});result['remote_record_count']=len(catalogue['records']);passed('remote record catalogue')
request('/api/orin/sync',{'all':True});deadline=time.time()+1200;previous=None
while time.time()<deadline:
    data=request('/api/orin/records');sync=data['sync']
    marker=(sync.get('current'),sync.get('completed'),sync['state'])
    if marker!=previous:print(marker,flush=True);previous=marker
    if sync['state'] not in ('running','idle'):break
    time.sleep(3)
result['sync']=sync;result['local_records']=data['local'];assert sync['state']=='completed',sync
records=request('/api/records');synced=[x for x in records if x['id'].startswith('orin-')];assert synced
for r in synced:
    replay=request('/api/replay/'+r['id']);assert replay['rows']
    name=str(replay['rows'][0].get('frame',0)).zfill(6)+'.jpg'
    with urllib.request.urlopen(BASE+'/asset/'+r['id']+'/'+name,timeout=10) as f:assert f.read(2)==b'\xff\xd8'
result['replay_records']=len(synced);passed('synced records indexed and replay images readable')
after=request('/api/orin/status');check(after);assert before['services']==after['services'],'Original service lifecycle changed'
result['after']=after;passed('vehicle remains LOCKED zero PWM; existing service uptime/restarts unchanged')
result['completed_at']=time.time();(ROOT/'ORIN_INTEGRATION_ACCEPTANCE.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print('ACCEPTANCE COMPLETE',flush=True)
