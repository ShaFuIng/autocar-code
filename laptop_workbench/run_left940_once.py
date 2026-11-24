import json,time,uuid,urllib.request
from pathlib import Path
base='http://127.0.0.1:8093/vehicle/api/'
dest=Path(__file__).parent/'data/ground-left940-trial1-20pct-10s.json'
assert not dest.exists(), 'Existing trial: do not repeat'
def req(path,p=None):
    r=urllib.request.Request(base+path,data=None if p is None else json.dumps(p).encode(),headers={'Origin':'http://127.0.0.1:8093','X-Orin-Control':'1','Content-Type':'application/json'})
    return json.load(urllib.request.urlopen(r,timeout=3))
def check(s):
    v=s['vehicle']
    assert not s['uart_error'] and s['uart_age_s']<1 and not s['camera_error'] and s['camera_age_s']<1
    assert s['servo_settings_synced'] and v['servo_profile']['center_us']==1300
    assert v['state'] in ('READY','LOCKED') and v['pins']['D11']==1 and not s['motor_busy']
    assert all(v[k]['pwm_command']==0 and v[k]['target']==0 for k in ('motor_a','motor_b'))
    assert all(v[k]['delta']==0 for k in ('enc_a','enc_b'))
    return v
out={'automatic_next_trial':False,'provisional_endpoints_us':{'left':940,'right':1900},'saved_limits_changed':False}
out['before']=req('status');v=check(out['before'])
dest.write_text(json.dumps(out,indent=2),encoding='utf-8')
try:
    if v['state']=='LOCKED':
        req('servo',{'action':'enable','generation':v['generation']})
        time.sleep(.4)
    s=req('status');v=check(s);assert v['state']=='READY'
    assert not v['steer_output_enabled'] or abs(v['steer_us_command']-940)<=100
    req('servo',{'generation':v['generation'],'us':940})
    time.sleep(.6)
    out['positioned']=req('status');v=check(out['positioned'])
    assert v['steer_output_enabled'] and v['steer_us_command']==940 and not v['servo_returning']
    owner=uuid.uuid4().hex
    run=req('motor/start',dict(owner=owner,surface='ground',area_clear=True,wheel='both',direction='forward',right_pct=20,left_pct=20,seconds=10,warmup_s=0,label='Left 940us: trial1 20pct 10s single run'))
    out['run_id']=run['run_id']
    for i in range(160):
        req('motor/heartbeat',{'owner':owner,'run_id':run['run_id']})
        m=req('motor/status')['current'];assert m['id']==run['run_id']
        if m['status'] not in ('preparing','running','stopping'):
            out['result']=m;break
        time.sleep(.1)
    else: raise RuntimeError('Test did not finish')
except Exception as e:
    out['error']=str(e)
    try: out['stop_response']=req('servo',{'action':'stop'})
    except Exception as stop_error: out['stop_error']=str(stop_error)
finally:
    out['after_samples']=[]
    for i in range(8):
        try: out['after_samples'].append(req('status'))
        except Exception as e: out['read_error']=str(e)
        time.sleep(.2)
    dest.write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps({'error':out.get('error'),'run_id':out.get('run_id'),'status':out.get('result',{}).get('status'),'reason':out.get('result',{}).get('reason'),'summary':out.get('result',{}).get('summary'),'after':[{'state':s['vehicle']['state'],'pwm':[s['vehicle'][k]['pwm_command'] for k in ('motor_a','motor_b')],'delta':[s['vehicle'][k]['delta'] for k in ('enc_a','enc_b')],'steer':s['vehicle']['steer_us_command']} for s in out['after_samples']]}))
