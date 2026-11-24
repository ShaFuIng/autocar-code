"""Single-owner, finite-duration wheel tests. No automatic restart or hardware I/O here."""
import csv
import json
import math
import re
import statistics
import threading
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path

ACTIVE = {'preparing', 'running', 'stopping'}
FIELDS = ['host_time','sample_ms','elapsed_s','interval_s','phase','right_pwm','left_pwm','right_target','left_target',
          'right_raw_net','left_raw_net','right_delta_raw','left_delta_raw','right_cps','left_cps',
          'right_invalid','left_invalid','rx_bad','rx_rejected','tx_dropped','state','reason','generation']


try:import throttle_cap   # throttle-cap-r1 (speed-next-20261117): web/throttle_cap.json max_pct (absent = 50 = old fixed limits)
except Exception:throttle_cap=None
def _tc_max():   # throttle-cap-r1: allowed |PWM| percent; any failure -> 50 (the old literal)
    try:return int(throttle_cap.max_pct()) if throttle_cap is not None else 50
    except Exception:return 50


def test_config(payload):
    surface=payload.get('surface','bench')
    if surface not in ('bench','ground'):raise ValueError('Invalid test surface')
    if surface=='bench' and payload.get('wheels_raised') is not True:raise ValueError('Confirm drive wheels are raised before running')
    if surface=='ground' and payload.get('area_clear') is not True:raise ValueError('Confirm clear ground test area')
    wheel=payload.get('wheel','both')
    direction=payload.get('direction','forward')
    if wheel not in ('both','right','left') or direction not in ('forward','reverse'):
        raise ValueError('Invalid wheel selection or direction')
    right,left=payload.get('right_pct'),payload.get('left_pct')
    lim=_tc_max()   # throttle-cap-r1: bench limit = throttle_cap.json max_pct (absent = 50); ground pulses stay <= 20 below
    if any(type(x) is not int or not -lim<=x<=lim for x in (right,left)):   # motor-test-signed-r1 (user 2026-11-06): negative = that wheel reverses
        raise ValueError('PWM must be an integer from -%d to %d percent (negative = reverse)'%(lim,lim))
    duration=payload.get('seconds');warmup=payload.get('warmup_s',0 if surface=='ground' else 1)
    if type(duration) not in (int,float) or not math.isfinite(duration) or not .5<=duration<=30:
        raise ValueError('Duration must be 0.5 to 30 seconds')
    if type(warmup) not in (int,float) or not math.isfinite(warmup) or not 0<=warmup<duration:
        raise ValueError('Warmup must be at least zero and shorter than duration')
    if surface=='ground' and (wheel!='both' or direction!='forward' or right!=left or not 0<=right<=20 or duration>10):raise ValueError('Ground pulse requires equal forward wheels, at most 20 percent and 10 seconds')
    owner=payload.get('owner','')
    if not isinstance(owner,str) or not re.fullmatch(r'[0-9a-f]{32}',owner):
        raise ValueError('Missing page ownership token')
    if wheel=='right':left=0
    if wheel=='left':right=0
    label=payload.get('label','')
    if not isinstance(label,str) or len(label)>80:raise ValueError('Label must be at most 80 characters')
    sign=1 if direction=='forward' else -1
    return dict(surface=surface,wheel=wheel,direction=direction,right_pct=right,left_pct=left,seconds=duration,
                duration_ms=round(duration*1000),warmup_s=warmup,label=label,
                right_command=sign*right,left_command=sign*left)


def statistics_for(rows,config):
    steady=[r for r in rows if r['phase']=='running' and r['right_cps'] is not None and r.get('interval_s') and r['elapsed_s']-r['interval_s']+1e-9>=config['warmup_s'] and r['elapsed_s']<=config['seconds']+1e-9]
    result={'steady_samples':len(steady),'mean_right_cps':None,'mean_left_cps':None,
            'right_left_ratio':None,'speed_gap_pct':None,'first_right_pulse_s':None,'first_left_pulse_s':None,
            'right_invalid_added':0,'left_invalid_added':0}
    if len(steady)>=3:
        total_time=sum(x['interval_s'] for x in steady)
        r=sum(abs(x['right_delta_raw']) for x in steady)/total_time
        l=sum(abs(x['left_delta_raw']) for x in steady)/total_time
        result.update(mean_right_cps=r,mean_left_cps=l,right_left_ratio=r/l if l else None,
                      speed_gap_pct=100*abs(r-l)/max(r,l) if max(r,l) else 0)
    if config.get('wheel')!='both': result['right_left_ratio']=result['speed_gap_pct']=None
    for side in ('right','left'):
        hits=[r['elapsed_s'] for r in rows if r['phase']=='running' and r[side+'_delta_raw'] not in (None,0)]
        result['first_'+side+'_pulse_s']=min(hits) if hits else None
        if rows:result[side+'_invalid_added']=(rows[-1][side+'_invalid']-rows[0][side+'_invalid']) & 0xffffffff
    return result


class MotorTestManager:
    def __init__(self,root,state,send,clock=time.monotonic,sleep=time.sleep):
        self.root=Path(root);self.state=state;self.send=send;self.clock=clock;self.sleep=sleep
        self.lock=threading.RLock();self.current=None;self.owner=None;self.last_seen=0
        self.cancel=threading.Event();self.cancel_reason='MANUAL_STOP';self.thread=None

    def busy(self):
        with self.lock:return bool(self.current and self.current['status'] in ACTIVE)

    def _atomic(self,run):
        if not re.fullmatch(r'[0-9a-f]{32}',run['id']):raise ValueError('Invalid run ID')
        self.root.mkdir(parents=True,exist_ok=True)
        path=self.root/(run['id']+'.json');tmp=path.with_suffix('.tmp')
        with tmp.open('w',encoding='utf-8') as f:
            json.dump({k:v for k,v in run.items() if k!='rows'},f,ensure_ascii=False,indent=2)
            f.flush()
        tmp.replace(path)

    def recover(self):
        self.root.mkdir(parents=True,exist_ok=True)
        for path in self.root.glob('*.json'):
            try:
                run=json.loads(path.read_text(encoding='utf-8'))
                if run.get('status') in ACTIVE:
                    run.update(status='interrupted',reason='SERVER_RESTARTED_NO_RESUME')
                    self._atomic(run)
            except (OSError,ValueError,KeyError):continue

    def _ready(self,config=None):
        d=self.state();v=d.get('vehicle') or {}
        if d.get('uart_error') or d.get('uart_age_s',99)>1:raise ValueError('UART unavailable')
        if v.get('fw') not in ('orin-v1.3.0-bench','orin-v1.4.1-manual') or not d.get('servo_settings_synced'):
            raise ValueError('Bench firmware and synchronized steering settings required')
        web_ready=v.get('web_timed_test_start') is True and v.get('state')=='LOCKED'
        if (v.get('state')!='READY' and not web_ready) or v.get('pins',{}).get('D11')!=1:
            raise ValueError('Check ESTOP, state and web timed-test firmware capability')
        if config and config.get('surface')=='ground':
            if v.get('web_timed_test_start') is not True:raise ValueError('Timed web-test firmware capability required')
            if d.get('camera_error') or d.get('camera_age_s',99)>1:raise ValueError('Fresh camera required')
        if v.get('servo_returning'):raise ValueError('Wait for steering to finish')
        if any(v.get(k,{}).get('delta',0)!=0 for k in ('enc_a','enc_b')):raise ValueError('Wait until both wheels have stopped before starting')
        return v

    def start(self,payload):
        config=test_config(payload)
        with self.lock:
            if self.busy():raise ValueError('Another test is active')
            v=self._ready(config)
            run=dict(id=uuid.uuid4().hex,created_at=datetime.now(timezone.utc).isoformat(),
                     status='preparing',reason='',config=config,generation=v['generation'],
                     rows=[],elapsed_s=0,summary=None)
            self._atomic(run)
            self.current=run;self.owner=payload['owner'];self.last_seen=self.clock()
            self.cancel.clear();self.cancel_reason='MANUAL_STOP'
            self.thread=threading.Thread(target=self._run,args=(run,),daemon=True)
            self.thread.start()
            return {'ok':True,'run_id':run['id']}

    def heartbeat(self,payload):
        with self.lock:
            if not self.current or payload.get('run_id')!=self.current['id'] or payload.get('owner')!=self.owner:
                raise ValueError('This page does not own the active test')
            # A heartbeat already in flight may arrive after normal completion.
            # Acknowledge the end without extending any lease or restarting a test.
            if not self.busy():return {'ok':True,'active':False,'status':self.current['status'],'reason':self.current['reason']}
            if self.clock()-self.last_seen>1:
                raise ValueError('BROWSER_HEARTBEAT_LOST')
            self.last_seen=self.clock()
        return {'ok':True,'active':True}

    def request_stop(self,reason='MANUAL_STOP'):
        with self.lock:
            if not self.busy():return {'ok':True,'active':False}
            self.cancel_reason=reason;self.cancel.set();self.current['status']='stopping'
        return {'ok':True,'active':True}

    def _write_sample(self,run,v,previous,writer,raw_file,coast=False):
        sample=v.get('enc_sample_ms')
        if type(sample) is not int:return previous
        if previous and sample==previous['enc_sample_ms']:return previous
        if previous and ((sample-previous['enc_sample_ms']) & 0xffffffff)>0x7fffffff:
            raise ValueError('Firmware encoder clock restarted')
        m=v.get('motor_test') or {};start=m.get('start_ms',0)
        if v.get('state')=='MOTOR_TEST' and not m.get('started'):
            return json.loads(json.dumps(v))
        elapsed=((sample-start)&0xffffffff)/1000 if start else 0
        if elapsed>1000:elapsed=0
        row=dict(host_time=datetime.now(timezone.utc).isoformat(),sample_ms=sample,elapsed_s=elapsed,interval_s=None,
                 phase='coast' if coast or v.get('state')!='MOTOR_TEST' else 'running',
                 right_pwm=v['motor_a']['pwm_command'],left_pwm=v['motor_b']['pwm_command'],
                 right_target=v['motor_a']['target'],left_target=v['motor_b']['target'],
                 right_cps=None,left_cps=None,right_delta_raw=None,left_delta_raw=None,
                 rx_bad=v.get('rx_bad'),rx_rejected=v.get('rx_rejected'),tx_dropped=v.get('tx_dropped'),
                 state=v.get('state'),reason=v.get('reason'),generation=v.get('generation'))
        for side,key,sign in (('right','enc_a',-1),('left','enc_b',1)):
            e=v[key];row[side+'_raw_net']=e['forward']-e['reverse'];row[side+'_invalid']=e['invalid']
            if previous:
                pe=previous[key]
                delta=((e['forward']-pe['forward'])&0xffffffff)-((e['reverse']-pe['reverse'])&0xffffffff)
                dt=((sample-previous['enc_sample_ms'])&0xffffffff)/1000
                row['interval_s']=dt
                row[side+'_delta_raw']=delta
                row[side+'_cps']=sign*delta/dt if dt else None
        writer.writerow(row);raw_file.write(json.dumps({'type':'sample',**row},ensure_ascii=False)+'\n');raw_file.flush()
        with self.lock:
            run['rows'].append(row);run['elapsed_s']=min(elapsed,run['config']['seconds'])
        return json.loads(json.dumps(v))

    def _checked_send(self,payload):
        if self.cancel.is_set():raise ValueError(self.cancel_reason)
        with self.lock:last_seen=self.last_seen
        if self.clock()-last_seen>1:raise ValueError('BROWSER_HEARTBEAT_LOST')
        return self.send(payload)

    def _firmware_completed(self,generation,error):
        # The firmware ends a timed test on its own clock and bumps generation;
        # a move/end packet racing that end is rejected as STALE_GENERATION or
        # TEST_NOT_ARMED. Those rejections mean the test already left MOTOR_TEST,
        # so wait for status telemetry (200 ms cadence) instead of one 0.25 s look.
        # Any other error keeps the original single short check before STOP.
        wait=1.0 if error in ('STALE_GENERATION','TEST_NOT_ARMED') else .25
        deadline=self.clock()+wait
        while True:
            self.sleep(.05 if wait>.25 else .25)
            latest=(self.state().get('vehicle') or {})
            if latest.get('state')=='READY' and latest.get('reason')=='MOTOR_TEST_COMPLETE' and latest.get('generation')==((generation+1)&0xffffffff):return True
            if self.clock()>=deadline:return False

    def _run(self,run):
        generation=run['generation'];config=run['config'];armed=False;finished=False;status='failed';reason='UNKNOWN'
        previous=None;begin=self.clock()
        try:
            with (self.root/(run['id']+'.csv')).open('w',encoding='utf-8',newline='') as csv_file, (self.root/(run['id']+'.jsonl')).open('w',encoding='utf-8') as raw:
                writer=csv.DictWriter(csv_file,fieldnames=FIELDS);writer.writeheader()
                raw.write(json.dumps({'type':'test','id':run['id'],'config':config},ensure_ascii=False)+'\n')
                self._checked_send(dict(motor_action='arm',generation=generation,duration_ms=config['duration_ms']))
                armed=True
                self._checked_send(dict(motor_action='move',generation=generation,right=0,left=0))
                self._checked_send(dict(motor_action='move',generation=generation,right=config['right_command'],left=config['left_command']))
                begin=self.clock();last_send=begin
                with self.lock:run['status']='running'
                while True:
                    now=self.clock()
                    if self.cancel.is_set():raise ValueError(self.cancel_reason)
                    with self.lock:last_seen=self.last_seen
                    if now-last_seen>1:raise ValueError('BROWSER_HEARTBEAT_LOST')
                    d=self.state();v=d.get('vehicle') or {}
                    if d.get('uart_error') or d.get('uart_age_s',99)>1:raise ValueError('UART_LOST')
                    previous=self._write_sample(run,v,previous,writer,raw)
                    csv_file.flush()
                    complete=v.get('state')=='READY' and v.get('reason')=='MOTOR_TEST_COMPLETE' and v.get('generation')==((generation+1)&0xffffffff)
                    if complete:finished=True;break
                    if v.get('state')!='MOTOR_TEST' or v.get('generation')!=generation:raise ValueError('AUTHORIZATION_CHANGED: '+str(v.get('reason')))
                    if now-begin>=config['seconds']:
                        try:self._checked_send(dict(motor_action='end',generation=generation))
                        except ValueError as exc:
                            if not self._firmware_completed(generation,str(exc)):raise
                        finished=True;break
                    if now-last_send>=.15:
                        self._checked_send(dict(motor_action='move',generation=generation,right=config['right_command'],left=config['left_command']))
                        last_send=self.clock()
                    self.sleep(.04)
                # Capture stopped/coasting observations, never send motion here.
                stop_at=self.clock()
                while self.clock()-stop_at<.5:
                    if self.cancel.is_set():raise ValueError(self.cancel_reason)
                    d=self.state();v=d.get('vehicle') or {}
                    if v and not d.get('uart_error') and d.get('uart_age_s',99)<1:
                        previous=self._write_sample(run,v,previous,writer,raw,coast=True)
                    self.sleep(.05)
                status='completed';reason='DURATION_COMPLETE'
        except Exception as exc:
            reason=str(exc)
            status='stopped' if reason=='MANUAL_STOP' else 'failed'
            if armed and reason not in ('MANUAL_STOP','BROWSER_HEARTBEAT_LOST','UART_LOST') and not self.cancel.is_set():
                if self._firmware_completed(generation,reason):
                    finished=True;status='completed';reason='DURATION_COMPLETE'
        finally:
            if not finished or self.cancel.is_set():
                try:self.send({'action':'stop'})
                except Exception:pass # Firmware lease stops outputs even if UART is gone.
            with self.lock:
                run.update(status=status,reason=reason,elapsed_s=min(self.clock()-begin,config['seconds']),
                           ended_at=datetime.now(timezone.utc).isoformat(),summary=statistics_for(run['rows'],config))
                try:self._atomic(run)
                except OSError as exc:run['reason']+='; SAVE_ERROR: '+str(exc)

    def snapshot(self):
        with self.lock:
            current=json.loads(json.dumps(self.current)) if self.current else None
        history=[]
        if self.root.exists():
            for path in self.root.glob('*.json'):
                try:
                    run=json.loads(path.read_text(encoding='utf-8'))
                    if re.fullmatch(r'[0-9a-f]{32}',run.get('id','')):history.append(run)
                except (OSError,ValueError):continue
        history.sort(key=lambda x:x.get('created_at',''),reverse=True)
        return {'current':current,'history':history[:30],'limit_pct':_tc_max(),'sample_hz':5}

    def result(self,run_id):
        if not re.fullmatch(r'[0-9a-f]{32}',run_id):raise ValueError('Invalid run ID')
        with self.lock:
            if self.current and self.current['id']==run_id:return json.loads(json.dumps(self.current))
        path=self.root/(run_id+'.json')
        if not path.exists():raise ValueError('Test not found')
        run=json.loads(path.read_text(encoding='utf-8'));rows=[]
        raw=self.root/(run_id+'.jsonl')
        if raw.exists():
            for line in raw.read_text(encoding='utf-8').splitlines():
                obj=json.loads(line)
                if obj.get('type')=='sample':rows.append({k:v for k,v in obj.items() if k!='type'})
        run['rows']=rows;return run

    def download(self,run_id,extension):
        if not re.fullmatch(r'[0-9a-f]{32}',run_id) or extension not in ('csv','jsonl'):raise ValueError('Invalid download')
        path=self.root/(run_id+'.'+extension)
        if not path.exists():raise ValueError('File not found')
        return path.read_bytes()
