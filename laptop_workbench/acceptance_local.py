"""Non-destructive HTTP acceptance on the laptop simulator. Refuses other backends."""
import base64,hashlib,json,time,uuid,urllib.request,urllib.error
from pathlib import Path

BASE='http://127.0.0.1:8093'
def api(path,data=None):
    req=urllib.request.Request(BASE+path,data=json.dumps(data).encode() if data is not None else None,headers={'Content-Type':'application/json','X-Workbench':'1'})
    with urllib.request.urlopen(req,timeout=8) as r:return json.load(r)
report={'checks':[],'started':time.time()}
def check(name,condition):
    assert condition,name
    report['checks'].append(name)
s=api('/api/status')
check('backend is simulation and vehicle disconnected',s['backend']=='simulation' and s['vehicle_connected'] is False)
if s['active']:raise RuntimeError('Existing task active; refusing to interrupt')
uid='qa-'+uuid.uuid4().hex
key=uid+'-draft'
data={'name':'自動驗收專用','wheelbase':None,'source':'simulation','notes':'保留原配置；此文件不是實測'}
d=api('/api/document',{'key':key,'rev':0,'data':data})
check('draft round trip including unknown null',api('/api/document/'+key)['data']==data)
try:api('/api/document',{'key':key,'rev':0,'data':data});raise AssertionError('conflict accepted')
except urllib.error.HTTPError as e:check('stale revision rejected',e.code==409)
api('/api/document',{'key':key,'rev':1,'data':data|{'notes':'第二版驗收'}})
check('immutable version history retained',len(api('/api/history/'+key))==2)
payload=b'laptop-only attachment acceptance\n'
a=api('/api/attachment',{'name':'acceptance.txt','record':key,'offset_s':1.25,'content':base64.b64encode(payload).decode()})
check('attachment hash and manual offset',a['sha256']==hashlib.sha256(payload).hexdigest() and a['offset_s']==1.25)
with urllib.request.urlopen(BASE+'/attachment/'+a['id']) as r:check('attachment bytes readable',r.read()==payload)
authorized=s['authorized']
api('/api/configure',{'action':'authorize','enabled':True})
try:
    p={'id':uid,'owner':'qa-owner-'+uuid.uuid4().hex,'mode':'fixed','duration_s':10,'pwm':0}
    t=api('/api/start',p)
    check('idempotent start',api('/api/start',p)['id']==t['id'])
    # No heartbeat, input, or polling during this interval.
    time.sleep(10.4)
    t=next(t for t in api('/api/tasks') if t['id']==uid)
    check('10s no-client-input task finished',t['status']=='stopped' and t['reason']=='DURATION_COMPLETE' and 10<=t['elapsed_s']<10.3)
    check('repeat after completion does not restart',api('/api/start',p)['status']=='stopped')
    r=api('/api/replay/'+uid)
    check('replay contains timestamped samples',len(r['rows'])>50 and all(x['source']=='simulation' for x in r['rows']))
    review={'start':0,'end':1,'target':'path','verdict':'未確認','notes':'自動驗收，不當實車真值'}
    api('/api/document',{'key':'review-'+uid,'rev':0,'data':review})
    check('review persists',api('/api/document/review-'+uid)['data']==review)
    check('no duplicate task rows',sum(t['id']==uid for t in api('/api/tasks'))==1)
    report['task']={k:t[k] for k in ('id','status','reason','elapsed_s')}
finally:
    current=api('/api/status')
    if not current['active']:api('/api/configure',{'action':'authorize','enabled':authorized})
report['ended']=time.time()
Path(__file__).with_name('ACCEPTANCE_LOCAL.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps(report,ensure_ascii=True,indent=2))
