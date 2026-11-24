"""One explicitly authorized raised-wheel test, no retries or next round."""
import json,time,uuid,threading,urllib.request,sys
from pathlib import Path
pwm=int(sys.argv[1]);assert pwm in (10,15,20)
def api(path,p=None):
    req=urllib.request.Request('http://127.0.0.1:8093/vehicle/api/'+path,data=None if p is None else json.dumps(p).encode(),headers={'Content-Type':'application/json','X-Workbench':'1','X-Orin-Control':'1','Origin':'http://127.0.0.1:8093'})
    with urllib.request.urlopen(req,timeout=3) as f:return json.load(f)
s=api('status');v=s['vehicle']
assert s['uart_age_s']<1 and v['state'] in ('LOCKED','READY') and all(v[k]['pwm_command']==0 for k in ('motor_a','motor_b'))
assert all(v[k]['delta']==0 for k in ('enc_a','enc_b'))
owner=uuid.uuid4().hex;done=threading.Event();errors=[];run=None
try:
    run=api('motor/start',dict(owner=owner,surface='bench',wheels_raised=True,wheel='both',direction='forward',right_pct=pwm,left_pct=pwm,seconds=5,warmup_s=1,label='按壓左輪後架空驗證，各5秒；排除起步1秒'))
    assert run.get('ok'),run
    print(json.dumps(run),flush=True)
    def beat():
        while not done.wait(.15):
            try:api('motor/heartbeat',dict(owner=owner,run_id=run['run_id']))
            except Exception as e:errors.append(str(e));return
    threading.Thread(target=beat,daemon=True).start()
    deadline=time.monotonic()+8
    while time.monotonic()<deadline:
        r=api('motor/status')['current']
        if r['id']!=run['run_id']:raise RuntimeError('Control ownership changed')
        if r['status'] not in ('preparing','running','stopping'):break
        rows=[x for x in r['rows'] if x['phase']=='running' and x['elapsed_s']>=1.5]
        if len(rows)>=3 and all(max(abs(x['right_cps']),abs(x['left_cps']))>0 and abs(abs(x['right_cps'])-abs(x['left_cps']))/max(abs(x['right_cps']),abs(x['left_cps']))>.3 for x in rows[-3:]):
            api('motor/stop',{});raise RuntimeError('Persistent wheel difference >30%; stopped')
        time.sleep(.2)
finally:
    done.set();api('motor/stop',{});time.sleep(.6)
    final=api('status')
    result=api('motor/result/'+run['run_id']) if run else {}
    root=Path(__file__).resolve().parents[1]/'data'
    (root/('bench-recheck-'+str(pwm)+'.json')).write_text(json.dumps({'result':result,'final':final,'heartbeat_errors':errors},ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps({'pwm':pwm,'status':result.get('status'),'reason':result.get('reason'),'summary':result.get('summary'),'state':final['vehicle']['state'],'motors':[final['vehicle'][k]['pwm_command'] for k in ('motor_a','motor_b')],'delta':[final['vehicle'][k]['delta'] for k in ('enc_a','enc_b')],'errors':errors}),flush=True)
