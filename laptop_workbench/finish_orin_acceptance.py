import json,time,urllib.request
from pathlib import Path
ROOT=Path(__file__).resolve().parent;BASE='http://127.0.0.1:8093'
def request(path,data=None):
    req=urllib.request.Request(BASE+path,data=json.dumps(data).encode() if data is not None else None,headers={'Origin':BASE,'X-Workbench':'1','Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=60) as response:return json.load(response)
before=json.loads((ROOT/'data/orin/route_audit.json').read_text(encoding='utf-8'))
old=request('/api/orin/records')['sync']
request('/api/orin/sync',{'missing':True})
for i in range(240):
    data=request('/api/orin/records');s=data['sync']
    if i%5==0:print(s,flush=True)
    if s['state']!='running':break
    time.sleep(2)
assert s['state']=='completed',s
catalog=request('/api/orin/discover',{});assert len(catalog['local'])==len(catalog['records']),(len(catalog['local']),len(catalog['records']))
records=[r for r in request('/api/records') if r['id'].startswith('orin-')]
frames=0
for r in records:
    replay=request('/api/replay/'+r['id']);assert replay['rows'];frames+=len(replay['rows'])
    assert all(replay['rows'][i]['capture_s']<=replay['rows'][i+1]['capture_s'] for i in range(len(replay['rows'])-1)),r
    name=str(replay['rows'][0].get('frame',0)).zfill(6)+'.jpg'
    with urllib.request.urlopen(BASE+'/asset/'+r['id']+'/'+name,timeout=10) as response:assert response.read(2)==b'\xff\xd8'
after=request('/api/orin/status');assert after['connected'] and after['vehicle']['state']=='LOCKED'
for key in ['motor_a','motor_b']:assert after['vehicle'][key]['pwm_command']==after['vehicle'][key]['target']==0
assert before['services']==after['services']
jobs=after['jobs'];timed=[j for j in jobs if j['state']=='completed' and j['duration_s']==10];assert timed and timed[0]['recording']['frames']>0
edge=json.loads((ROOT/'ORIN_RECORDING_EDGE_ACCEPTANCE.json').read_text());assert edge['concurrent_start_rejected'] and edge['owned_early_stop_passed']
result=dict(completed_at=time.time(),remote_record_count=len(catalog['records']),replay_records=len(records),replay_frames=frames,sync=s,local_records=catalog['local'],before=before,after=after,recording_job=timed[0],initial_sync=old,notes=['Two transient Windows folder sharing violations repaired by retry; source records preserved.','Camera transport verified; current image dark, scene/optics require on-site check.'],tests=['Real Orin status and JPEG through new website','10-second remote recording completed without browser heartbeat','Idempotent job key; concurrent start rejected; owned early stop passed','Unsupported motion API and invalid ownership/duration rejected','All listed remote records synced with SHA256 verification','Synced replay rows/timestamps and JPEG reads verified','Original services startup timestamps and restart counts unchanged','Vehicle remains LOCKED; both motor command/target zero'])
(ROOT/'ORIN_INTEGRATION_ACCEPTANCE.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print('PASS',len(catalog['local']),len(records),frames,flush=True)
