from ops07_runtime import deadline_expired
def _pid_deadline_expired(run,now):   # pause-resume-r1 (pause-resume-20261120): a PID run without deadline (orin_timed seconds null, or suspended while paused) never expires here
    if run.get('execution_mode')!='orin_continuous' and run.get('deadline') is None:return False
    return deadline_expired(run,now)
"""Single-owner manual driving, bounded intent lease, persistent trims and session logs."""
import math
import json
import os
import re
import threading
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from feedback_observation import observation,bounded_context

FIRMWARE = 'orin-v1.4.1-manual'
NONBLOCK_PATH=Path(__file__).with_name('uart_nonblock.json')
NONBLOCK_OFF={'enabled':False,'max_outstanding':1,'ack_timeout_s':1.0,'read_timeout_s':.05,'min_period_s':.1,'max_period_s':.1}
SUPERSEDE_LINK_LOSS_S=.4   # uart-ack-supersede-r1: default bound for 'no ACK at all'; must stay below the firmware's 0.5 s COMMAND_TIMEOUT


def _supersede_config(c):   # uart-ack-supersede-r1: ON only when ack_supersede is true, max_outstanding>=2 and the link-loss bound is valid; anything else -> {} (old behaviour)
    link=c.get('ack_link_loss_s',SUPERSEDE_LINK_LOSS_S)
    if c.get('ack_supersede') is True and c['max_outstanding']>=2 and type(link) in (int,float) and math.isfinite(link) and .2<=link<=.45 and link<c['ack_timeout_s']:
        return {'ack_supersede':True,'ack_link_loss_s':float(link)}
    return {}


def load_nonblock(path=None):   # uart-nonblock-r1: missing/invalid/disabled file -> blocking PID dispatch exactly as before
    try:
        c=json.loads(Path(path or NONBLOCK_PATH).read_text(encoding='utf-8'))
        num=lambda k,lo,hi:type(c.get(k)) in (int,float) and math.isfinite(c[k]) and lo<=c[k]<=hi
        if c.get('enabled') is True and type(c.get('max_outstanding')) is int and 1<=c['max_outstanding']<=4 and num('ack_timeout_s',.1,2.5) and num('read_timeout_s',.001,.05) and num('max_period_s',.02,.2) and num('min_period_s',.01,c['max_period_s']):
            out={k:c[k] for k in NONBLOCK_OFF}
            if 'ack_supersede' in c:out.update(_supersede_config(c))   # uart-ack-supersede-r1: key absent -> exactly the old dict
            if 'pace_after_timeout_s' in c and num('pace_after_timeout_s',.005,c['min_period_s']):out['pace_after_timeout_s']=float(c['pace_after_timeout_s'])   # uart-pace-r1: key absent / invalid -> old pacing
            return out
    except (OSError,ValueError,TypeError,AttributeError,KeyError):pass
    return dict(NONBLOCK_OFF)


def atomic_json(path, value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_suffix('.tmp')
    with temp.open('w',encoding='utf-8') as f:
        json.dump(value,f,ensure_ascii=False,indent=2);f.flush();os.fsync(f.fileno())
    temp.replace(path)


try:import throttle_cap   # throttle-cap-r1 (speed-next-20261117): web/throttle_cap.json max_pct (absent = 50 = old fixed limits)
except Exception:throttle_cap=None
def _tc_max(v):   # throttle-cap-r1: allowed |PWM| percent; any failure -> 50 (the old literal)
    try:return int(throttle_cap.max_pct(v)) if throttle_cap is not None else 50
    except Exception:return 50


class ManualDriveManager:
    def __init__(self,root,state,send,profile,save_center,defaults_path,recorder=None,clock=time.monotonic,send_async=None):
        self.root=Path(root);self.state=state;self.send=send;self.profile=profile
        self.send_async=send_async;self.kick=threading.Event();self.inflight=[];self.nonblock=load_nonblock()   # uart-nonblock-r1
        self.save_center=save_center;self.defaults_path=Path(defaults_path);self.recorder=recorder;self.clock=clock
        self.lock=threading.RLock();self.current=None;self.owner=None;self.last_input=0;self.input_seq=-1
        self.intent=(0,0);self.hard_zero=False;self.centering=False;self.cancel=threading.Event();self.reason='MANUAL_STOP';self.thread=None
        self.defaults={'right_pct':10,'left_pct':10,'steer_rate_us_s':300,'accel_seconds':2.0,'decel_seconds':1.0}
        self.defaults_error=None
        if self.defaults_path.exists():
            try:
                p=json.loads(self.defaults_path.read_text())
                validated=self._validate(p,self.profile());self.defaults={k:validated[k] for k in self.defaults}
            except (ValueError,OSError,TypeError) as exc:self.defaults_error='Manual defaults invalid: '+str(exc)
        self.root.mkdir(parents=True,exist_ok=True)
        for folder in self.root.iterdir():
            path=folder/'summary.json'
            if folder.is_dir() and path.exists():
                try:
                    p=json.loads(path.read_text())
                    if p.get('status') in ('preparing','running','stopping'):
                        p.update(status='interrupted',reason='SERVER_RESTARTED_NO_RESUME');atomic_json(path,p)
                except (ValueError,OSError):pass

    def busy(self):
        with self.lock:return bool(self.current and self.current['status'] in ('preparing','running','stopping'))

    def _validate(self,p,profile,lim=50):   # throttle-cap-r1: lim = allowed |PWM| (50 except a PID start, see start())
        if not isinstance(p,dict):raise ValueError('Expected manual settings object')
        for key in ('right_pct','left_pct'):
            if type(p.get(key)) is not int or not -lim<=p[key]<=lim:raise ValueError('PWM must be an integer -%d..%d (negative = reverse)'%(lim,lim))   # wheel-pwm-signed-r1
        if (p['right_pct']>0 and p['left_pct']<0) or (p['right_pct']<0 and p['left_pct']>0):   # wheel-pwm-signed-r2: allowed when the firmware reports mixed_direction_version >= 1
            try:mv=int(((self.state() or {}).get('vehicle') or {}).get('mixed_direction_version') or 0)
            except (TypeError,ValueError):mv=0
            if mv<1:raise ValueError('MIXED_DIRECTION: one wheel forward and the other reverse is not allowed by this firmware')
        center=p.get('center_us',profile['center_us'])
        if type(center) is not int or not profile['min_us']<center<profile['max_us']:raise ValueError('Center outside steering limits')
        rate=p.get('steer_rate_us_s',self.defaults['steer_rate_us_s'])
        if type(rate) is not int or not 50<=rate<=1500:raise ValueError('Steering rate must be an integer 50..1500 us/s')   # steer-rate-r1 (2026-11-03, live a591a0ec): 800 us/s ramp added ~0.26 s to every steering reversal
        times={}
        for key in ('accel_seconds','decel_seconds'):
            value=p.get(key,self.defaults[key])
            if type(value) not in (int,float) or not math.isfinite(value) or not 0<=value<=5:raise ValueError('Acceleration/deceleration time must be 0..5 seconds')
            times[key]=float(value)
        return {'right_pct':p['right_pct'],'left_pct':p['left_pct'],'center_us':center,'steer_rate_us_s':rate,**times}

    def _owned(self,p):
        if not self.busy() or p.get('run_id')!=self.current['id'] or p.get('owner')!=self.owner:raise ValueError('This page does not own manual driving')

    def _ready(self):
        if self.defaults_error:raise ValueError(self.defaults_error)
        d=self.state();v=d.get('vehicle') or {}
        if self.recorder and getattr(self.recorder,'error',None):raise ValueError('RECORDING_ERROR: '+self.recorder.error)
        if d.get('uart_error') or d.get('uart_age_s',99)>1:raise ValueError('UART unavailable')
        if v.get('fw')!=FIRMWARE:raise ValueError('Flash '+FIRMWARE+' first')
        if not d.get('servo_settings_synced') or (v.get('state')!='READY' and not (v.get('web_manual_start') is True and v.get('state')=='LOCKED')) or v.get('pins',{}).get('D11')!=1:raise ValueError('Check synchronized settings, ESTOP and web-start authorization')
        if v.get('servo_returning'):raise ValueError('Steering is returning to center')
        if d.get('camera_error') or d.get('camera_age_s',99)>1:raise ValueError('Live camera required before driving')
        if any(v.get(k,{}).get('delta',0) for k in ('enc_a','enc_b')):raise ValueError('Wait for both wheels to stop')
        return v

    def start(self,p,_pid=False):
        if p.get('control_version')!=2:raise ValueError('Please refresh the manual page (Ctrl+F5)')
        if not isinstance(p.get('owner'),str) or not re.fullmatch('[0-9a-f]{32}',p['owner']):raise ValueError('Invalid page owner')
        if p.get('area_clear') is not True:raise ValueError('Confirm clear operating area')
        if type(p.get('record')) is not bool:raise ValueError('Choose recording state')
        with self.lock:
            if self.busy():raise ValueError('Manual driving already active')
            v=self._ready();profile=self.profile();config=self._validate(p,profile,_tc_max(v) if _pid else 50)   # throttle-cap-r1: only the PID start may exceed 50
            # Initialize using the MCU's synchronized center; trims are applied afterward.
            run={'control_version':2,'id':uuid.uuid4().hex,'status':'preparing','reason':'','created_at':datetime.now(timezone.utc).isoformat(),
                 'generation':v['generation'],'config':config,'limits':profile,'recording':p['record'],
                 'command':{'right':0,'left':0,'steer_us':profile['center_us']},'last_input_seq':-1,'input_diagnostics':{'accepted':0,'max_gap_ms':0,'last_gap_ms':0}}
            run['controller']='visual_pid' if _pid else 'manual'
            if _pid:
                seconds=p.get('seconds')
                mode=p.get('execution_mode','browser_lease')
                if mode not in ('browser_lease','orin_timed','orin_continuous'):raise ValueError('Unknown PID execution mode')
                if mode!='orin_continuous' and not (mode=='orin_timed' and seconds is None) and (type(seconds) not in (int,float) or not math.isfinite(seconds) or seconds<1):raise ValueError('PID duration must be finite and at least 1 second')
                run['execution_mode']=mode;run['ops07_hold']=mode=='orin_continuous'
                run['deadline']=None if mode=='orin_continuous' or seconds is None else self.clock()+seconds;run['pid_target_us']=profile['center_us']
            self.current=run;self.owner=p['owner'];self.last_input=self.clock();self.input_seq=-1;self.intent=(0,0);self.hard_zero=False;self.centering=False
            self.cancel.clear();self.reason='MANUAL_STOP';folder=self.root/run['id'];folder.mkdir()
            atomic_json(folder/'summary.json',run)
            self.thread=threading.Thread(target=self._run,args=(run,),daemon=True);self.thread.start()
            return {'ok':True,'run_id':run['id']}

    def input(self,p):
        with self.lock:
            self._owned(p)
            if self.current.get('controller')=='visual_pid':raise ValueError('PID owns control; stop before manual input')
            if self.clock()-self.last_input>1:raise ValueError('BROWSER_INPUT_LOST')
            seq=p.get('seq');direction=p.get('direction');steer=p.get('steer')
            if type(seq) is not int or seq<=self.input_seq:raise ValueError('Stale input sequence')
            if type(direction) is not int or direction not in (-1,0,1) or type(steer) is not int or steer not in (-1,0,1):raise ValueError('Invalid driving intent')
            hard_zero=p.get('hard_zero',False);center=p.get('center',False)
            if type(hard_zero) is not bool or type(center) is not bool:raise ValueError('Invalid control flags')
            if hard_zero and direction:raise ValueError('Hard zero requires neutral throttle')
            event=p.get('event','HEARTBEAT')
            if event not in ('HEARTBEAT','KEY_DOWN','KEY_UP','POINTER','CONTROL_AREA_ENTER','CONTROL_AREA_EXIT','SPACE_ZERO','INPUT_FIELD','CENTER'):raise ValueError('Invalid input event')
            if event!='HEARTBEAT':self.current['last_input_event']={'event':event,'seq':seq,'time':datetime.now(timezone.utc).isoformat()}
            now=self.clock();gap=round((now-self.last_input)*1000,1)
            diag=self.current['input_diagnostics'];diag.update(accepted=diag['accepted']+1,last_gap_ms=gap,max_gap_ms=max(diag['max_gap_ms'],gap),last_received_at=datetime.now(timezone.utc).isoformat())
            self.hard_zero=hard_zero or event in ('SPACE_ZERO','CONTROL_AREA_EXIT','INPUT_FIELD');self.centering=center and not steer
            self.intent=(direction,steer);self.input_seq=seq;self.last_input=now
            self.current['last_input_seq']=seq
            return {'ok':True}

    def pid_input(self,p):
        with self.lock:
            self._owned(p)
            if self.current.get('controller')!='visual_pid':raise ValueError('Not a PID session')
            if self.clock()-self.last_input>1:raise ValueError('PID_INPUT_LOST')
            seq=p.get('seq');target=p.get('target_us');power=p.get('pwm')
            if type(seq) is not int or seq<=self.input_seq:raise ValueError('Stale PID sequence')
            if type(target) not in (int,float) or not math.isfinite(target) or not self.current['limits']['min_us']<=target<=self.current['limits']['max_us']:raise ValueError('Invalid PID target')
            if type(power) not in (int,float) or not math.isfinite(power) or not 0<=power<=min(self.current['config']['right_pct'],self.current['config']['left_pct']):raise ValueError('Invalid PID PWM')
            rl=None   # diff-assist-r1: optional per-wheel PWM (right = motor A / right wheel, left = motor B / left wheel)
            if 'pwm_right' in p or 'pwm_left' in p:
                rl=(p.get('pwm_right'),p.get('pwm_left'));lim=min(self.current['config']['right_pct'],self.current['config']['left_pct'])
                if any(type(x) not in (int,float) or not math.isfinite(x) or not 0<=x<=lim for x in rl):raise ValueError('Invalid PID wheel PWM')
                if not power or abs(rl[0]+rl[1]-2*power)>1e-6 or max(abs(rl[0]-power),abs(rl[1]-power))>10:raise ValueError('Invalid PID wheel PWM')
            self.current['pid_target_us']=target;self.current['pid_pwm']=power;self.current['pid_pwm_rl']=rl
            self.current['pid_observation_context']=bounded_context(p.get('observation_context'))
            self.intent=(1 if power else 0,0);self.hard_zero=not power;self.input_seq=seq;self.last_input=self.clock()
            self.kick.set()   # uart-nonblock-r1: wake a nonblocking PID loop on the new target
            return {'ok':True}

    def pid_pause_hold(self,run_id,hold):   # pause-resume-r1 (pause-resume-20261120): output latch (centre, PWM 0) while the PID run is paused
        with self.lock:
            run=self.current
            if not run or run.get('id')!=run_id or run.get('controller')!='visual_pid':raise ValueError('Not this PID session')
            run['pid_pause_hold']=bool(hold)
            if hold:run['pid_target_us']=run['limits']['center_us'];run['pid_pwm']=0;run['pid_pwm_rl']=None;self.intent=(0,0);self.hard_zero=True
            self.kick.set()
            return {'ok':True}
    def pause_deadline(self,run_id):   # pause-resume-r1 (pause-resume-20261120): suspend this run's deadline; returns the remaining seconds (None = no deadline)
        with self.lock:
            run=self.current
            if not run or run.get('id')!=run_id or run.get('deadline') is None:return None
            rest=max(0.,run['deadline']-self.clock());run['deadline']=None;run['deadline_suspended_s']=rest;return rest
    def resume_deadline(self,run_id,rest):   # pause-resume-r1 (pause-resume-20261120): restore the suspended deadline (paused time not counted)
        with self.lock:
            run=self.current
            if not run or run.get('id')!=run_id or rest is None:return None
            run['deadline']=self.clock()+float(rest);run.pop('deadline_suspended_s',None);return run['deadline']

    def settings(self,p):
        with self.lock:
            if self.busy():
                self._owned(p)
                if self.current.get('controller')=='visual_pid':raise ValueError('PID parameters frozen until stopped')
                profile=self.current['limits']
            else:profile=self.profile()
            config=self._validate(p,profile)
            moving=self.busy() and (self.intent[0] or self.current['command']['right'] or self.current['command']['left'])
            if moving and abs(config['center_us']-self.current['config']['center_us'])>20:raise ValueError('While driving adjust center by at most 20 us')
            sg=lambda x:(x>0)-(x<0)
            if moving and any(sg(config[k]) and sg(self.current['config'][k]) and sg(config[k])!=sg(self.current['config'][k]) for k in ('right_pct','left_pct')):raise ValueError('Stop before changing a wheel direction')   # wheel-pwm-signed-r1
            if p.get('save') is True:
                before=dict(self.defaults,center_us=self.profile()['center_us'])
                self.save_center(config['center_us'])
                atomic_json(self.defaults_path,{k:config[k] for k in self.defaults})
                self.defaults={k:config[k] for k in self.defaults}
                self.defaults_error=None
                entry={'id':uuid.uuid4().hex,'saved_at':datetime.now(timezone.utc).isoformat(),
                       'before':before,'config':config,'profile':self.profile(),'run_id':self.current['id'] if self.busy() else None}
                atomic_json(self.root/'settings_history'/(entry['id']+'.json'),entry)
            if self.busy():self.current['config']=config
            return {'ok':True,'config':config,'saved':p.get('save') is True}

    def request_stop(self,reason='MANUAL_STOP'):
        with self.lock:
            if not self.busy():return {'ok':True,'active':False}
            self.reason=reason;self.intent=(0,0);self.cancel.set();self.current['status']='stopping'
            return {'ok':True,'active':True}

    def stop_for_page(self,p,reason):
        with self.lock:
            if not self.current or p.get('run_id')!=self.current['id'] or p.get('owner')!=self.owner:
                return {'ok':True,'active':False}
            return self.request_stop(reason)

    def recording(self,p):
        with self.lock:
            self._owned(p)
            if self.current['status']!='running':raise ValueError('Wait until manual driving is ready')
            if type(p.get('record')) is not bool:raise ValueError('Invalid recording state')
            if self.recorder:
                if p['record'] and not self.current['recording']:self.recorder.start(self.current['id'])
                if not p['record']:self.recorder.stop()
            self.current['recording']=p['record'];return {'ok':True,'recording':p['record']}

    def _check(self):
        if self.cancel.is_set():raise ValueError(self.reason)
        with self.lock:age=self.clock()-self.last_input
        if age>1:raise ValueError('BROWSER_INPUT_LOST')
        if self.current and self.current.get('controller')=='visual_pid' and _pid_deadline_expired(self.current,self.clock()):raise ValueError('PID_DURATION_COMPLETE')
        d=self.state()
        if d.get('uart_error') or d.get('uart_age_s',99)>1:raise ValueError('UART_LOST')
        if d.get('camera_error') or d.get('camera_age_s',99)>1:raise ValueError('CAMERA_LOST')
        return d

    def _send(self,p):
        self._check();return self.send(p)

    def _send_async(self,p):   # uart-nonblock-r1
        self._check();job=self.send_async(p);job['sent_at']=self.clock();self.inflight.append(job);return job

    def _resolve(self):   # uart-nonblock-r1: drop finished jobs; failures end the run like the blocking _send would
        if self.nonblock.get('ack_supersede'):return self._resolve_superseding()   # uart-ack-supersede-r1
        dcw=False;ack_ms=None
        for job in list(self.inflight):
            if job['event'].is_set():
                self.inflight.remove(job);result=job['result']
                if job.get('acked_at') is not None:ack_ms=(job['acked_at']-job['sent_at'])*1000
                if not result or not result.get('ok'):
                    reason=(result or {}).get('reason','UART disconnected')
                    if 'DIRECTION_CHANGE_WAIT' not in reason:raise ValueError(reason)
                    dcw=True
            elif self.clock()-job['sent_at']>self.nonblock['ack_timeout_s']:raise ValueError('No firmware acknowledgement; movement not confirmed')
        return dcw,ack_ms

    def _settle(self,prev_ack_ms):   # uart-nonblock-r1: at max_outstanding wait for the oldest ACK instead of silently skipping the frame
        if self.nonblock.get('ack_supersede'):return self._settle_superseding(prev_ack_ms)   # uart-ack-supersede-r1
        dcw,ms=self._resolve();waited=0.
        if len(self.inflight)>=self.nonblock['max_outstanding']:
            start=self.clock();job=self.inflight[0]
            job['event'].wait(max(0.,self.nonblock['ack_timeout_s']-(start-job['sent_at'])))
            waited=(self.clock()-start)*1000;late,later=self._resolve();dcw=dcw or late;ms=ms if later is None else later
            if len(self.inflight)>=self.nonblock['max_outstanding']:raise ValueError('No firmware acknowledgement; movement not confirmed')
        return dcw,waited,prev_ack_ms if ms is None else ms

    def _link_check(self):   # uart-ack-supersede-r1: 'no ACK at all' bound; same reason text as the old ACK timeout
        if not self.inflight:return
        age=self.clock()-max(self.last_ack_at,self.inflight[0]['sent_at'])
        if age>self.nonblock['ack_link_loss_s']:
            self.link_loss={'unconfirmed_ms':round(age*1000,1),'limit_ms':round(self.nonblock['ack_link_loss_s']*1000,1),'pending':len(self.inflight)}
            raise ValueError('No firmware acknowledgement; movement not confirmed')

    def _resolve_superseding(self):   # uart-ack-supersede-r1: like _resolve, but a job the server marked superseded (a NEWER seq was ACKed ok) is counted, not an error
        dcw=False;ack_ms=None
        for job in list(self.inflight):
            if job['event'].is_set():
                self.inflight.remove(job);result=job['result']
                if job.get('acked_at') is not None:
                    ack_ms=(job['acked_at']-job['sent_at'])*1000;self.last_ack_at=max(self.last_ack_at,job['acked_at'])
                if result and result.get('ok') is True and result.get('type')=='superseded':
                    self.sup_total+=1
                    event={'seq':result.get('seq'),'by':result.get('superseded_by'),'sent_at':job['sent_at'],'resolved_at':self.clock(),'command':{k:(job.get('payload') or {}).get(k) for k in ('right','left','us')}}
                    self.sup_new.append(event)
                    if len(self.sup_events)<200:self.sup_events.append(event)
                    continue
                if not result or not result.get('ok'):
                    reason=(result or {}).get('reason','UART disconnected')
                    if 'DIRECTION_CHANGE_WAIT' not in reason:raise ValueError(reason)
                    dcw=True
            elif self.clock()-job['sent_at']>self.nonblock['ack_timeout_s']:raise ValueError('No firmware acknowledgement; movement not confirmed')
        self._link_check()
        return dcw,ack_ms

    def _settle_superseding(self,prev_ack_ms):   # uart-ack-supersede-r1: at max_outstanding wait in 10 ms slices; ends by an ACK (direct or superseding) or by the link-loss bound
        dcw,ms=self._resolve_superseding();waited=0.
        if len(self.inflight)>=self.nonblock['max_outstanding']:
            start=self.clock()
            while len(self.inflight)>=self.nonblock['max_outstanding']:
                if self.cancel.is_set():raise ValueError(self.reason)
                self.inflight[0]['event'].wait(.01)
                late,later=self._resolve_superseding();dcw=dcw or late;ms=ms if later is None else later
            waited=(self.clock()-start)*1000
        return dcw,waited,prev_ack_ms if ms is None else ms

    def _sup_timing(self):   # uart-ack-supersede-r1: extra dispatch_timing fields (ON only); superseded events are logged once
        new,self.sup_new=self.sup_new,[]
        age=round((self.clock()-max(self.last_ack_at,self.inflight[0]['sent_at']))*1000,1) if self.inflight else None
        return {'ack_supersede':True,'ack_superseded':new,'ack_superseded_total':self.sup_total,'ack_unconfirmed_ms':age,'ack_link_loss_ms':round(self.nonblock['ack_link_loss_s']*1000,1)}

    def _pace(self,now,prev_kicked=None):   # uart-nonblock-r1: next iteration on a new PID target (kick), never before min_period, at the latest max_period
        c=self.nonblock;start=self.clock();kicked=False
        while not self.cancel.is_set():
            rest=now+c['max_period_s']-self.clock()
            if rest<=0:break
            # Short slices keep stop latency near today's cancel.wait.
            if self.kick.wait(min(rest,.01)):kicked=True;break
        mp=c['min_period_s']
        if kicked and prev_kicked is False and c.get('pace_after_timeout_s') is not None:mp=c['pace_after_timeout_s'];self.pace_short=getattr(self,'pace_short',0)+1   # uart-pace-r1: the iteration at `now` was a max_period re-send, so a NEW target does not wait min_period after it
        waited=(self.clock()-start)*1000;rest=now+mp-self.clock()
        if rest>0:self.cancel.wait(rest)
        return kicked,waited

    def _run(self,run):
        gen=run['generation'];right=left=offset=0;last_dir=0;zero_at=self.clock();last_sample=None;last_config=None;last_steer_at=self.clock();power_config=None;power_reference=(0,0)
        folder=self.root/run['id'];reason='UNKNOWN'
        nb=run.get('controller')=='visual_pid' and self.nonblock['enabled'] and self.send_async is not None;self.inflight=[];prev_ack_ms=kick_wait_ms=None;kicked=False   # uart-nonblock-r1: computed once per run; False keeps blocking dispatch and cadence
        sup=nb and bool(self.nonblock.get('ack_supersede'))   # uart-ack-supersede-r1
        if sup:self.last_ack_at=self.clock();self.sup_total=0;self.sup_new=[];self.sup_events=[];self.link_loss=None
        try:
            with (folder/'controls.jsonl').open('a',encoding='utf-8') as log:
                self._send({'manual_action':'arm','generation':gen})
                self._send({'manual_action':'move','generation':gen,'right':0,'left':0,'us':run['limits']['center_us']})
                if run['recording'] and self.recorder:self.recorder.start(run['id'])
                with self.lock:run['status']='running'
                while True:
                    d=self._check();feedback_read_at=self.clock();v=d.get('vehicle') or {}
                    if v.get('generation')!=gen or v.get('state')!='MANUAL':raise ValueError('AUTHORIZATION_CHANGED: '+str(v.get('reason')))
                    if nb:dcw,ack_wait_ms,prev_ack_ms=self._settle(prev_ack_ms)   # uart-nonblock-r1
                    with self.lock:direction,steer=self.intent;config=dict(run['config']);hard_zero=self.hard_zero;centering=self.centering
                    now=self.clock()
                    elapsed=min(.2,max(0,now-last_steer_at));last_steer_at=now
                    previous_right,previous_left=right,left
                    reversing=bool(direction and last_dir and direction!=last_dir)
                    target_dir=0 if reversing else direction
                    def approach(value,target,amount):return value+max(-amount,min(amount,target-value))
                    if config!=power_config:
                        power_reference=(max(abs(right),abs(config['right_pct']),1),max(abs(left),abs(config['left_pct']),1));power_config=dict(config)   # wheel-pwm-signed-r1: magnitude
                    def power(value,limit,reference):
                        target=target_dir*limit
                        duration=config['accel_seconds'] if target and abs(target)>abs(value) else config['decel_seconds']
                        # Scaling by the configured maximum makes time independent
                        # of loop/ACK rate; never inject a minimum starting boost.
                        return target if duration==0 else approach(value,target,reference*elapsed/duration)
                    if hard_zero:right=left=0
                    else:right=power(right,config['right_pct'],power_reference[0]);left=power(left,config['left_pct'],power_reference[1])
                    if previous_right or previous_left:
                        if not right and not left:zero_at=now
                    if reversing:
                        if not right and not left and now-zero_at>=.7 and not any(v.get(k,{}).get('delta',0) for k in ('enc_a','enc_b')):
                            last_dir=direction
                    elif direction and not hard_zero:last_dir=direction
                    step=config['steer_rate_us_s']*elapsed
                    if run.get('controller')=='visual_pid':
                        with self.lock:
                            target=run['pid_target_us'];power=run.get('pid_pwm',0);feedback_context=run.get('pid_observation_context');rl=run.get('pid_pwm_rl')
                            if run.get('ops07_hold') or run.get('pid_pause_hold'):   # pause-resume-r1 (pause-resume-20261120): paused PID run = the same centre / PWM 0 latch
                                target=run['limits']['center_us'];power=0;rl=None;hard_zero=True
                            if nb:self.kick.clear()   # uart-nonblock-r1: cleared with the target read, so a later pid_input always re-kicks
                        if hard_zero:right=left=0
                        elif rl:right,left=rl   # diff-assist-r1
                        else:right=left=power
                        offset=approach(offset,target-config['center_us'],step)
                    elif steer:offset+=steer*step
                    elif centering:offset=approach(offset,0,step)
                    # Otherwise hold the steering command when A/D are released.
                    minimum,maximum=run['limits']['min_us'],run['limits']['max_us']
                    offset=max(minimum-config['center_us'],min(maximum-config['center_us'],offset))
                    pulse=round(config['center_us']+offset)
                    send=self._send_async if nb else self._send   # uart-nonblock-r1: nonblocking queues without waiting for the ACK
                    if nb and dcw:right=left=0;zero_at=now
                    try:send({'manual_action':'move','generation':gen,'right':round(right),'left':round(left),'us':pulse})
                    except ValueError as e:
                        if 'DIRECTION_CHANGE_WAIT' not in str(e):raise
                        right=left=0;zero_at=now
                        send({'manual_action':'move','generation':gen,'right':0,'left':0,'us':pulse})
                    with self.lock:run['command']={'right':round(right),'left':round(left),'steer_us':pulse}
                    ack_at=None if nb else self.clock()   # uart-nonblock-r1: the ACK is unknown at log time; never fake it
                    nb_timing={'mode':'nonblock','sent_at':self.inflight[-1]['sent_at'],'dispatch_sent_monotonic_s':self.inflight[-1]['sent_at'],'ack_pending':len(self.inflight),'prev_ack_ms':prev_ack_ms,'ack_wait_ms':ack_wait_ms,'kick_wait_ms':kick_wait_ms,'kicked':kicked,'direction_change_wait':dcw,**(self._sup_timing() if sup else {}),**({'pace_short':getattr(self,'pace_short',0)} if 'pace_after_timeout_s' in self.nonblock else {})} if nb else {}   # uart-ack-supersede-r1: extra fields only when ON
                    sample=v.get('enc_sample_ms')
                    if sample!=last_sample or config!=last_config or run.get('controller')=='visual_pid':
                        log.write(json.dumps({'time':datetime.now(timezone.utc).isoformat(),'config':config,'intent':{'direction':direction,'steer':steer,'hard_zero':hard_zero,'center':centering},'input_event':run.get('last_input_event'),'input_diagnostics':dict(run['input_diagnostics']),'input_age_ms':round((self.clock()-self.last_input)*1000,1),'command':dict(run['command']),'vehicle':v,'feedback_observation':observation(d,feedback_read_at,feedback_context if run.get('controller')=='visual_pid' else None,run['id'],gen,run['command'],ack_at),'dispatch_timing':{'iteration_at':now,'ack_at':ack_at,'iteration_to_ack_ms':None if nb else (ack_at-now)*1000,'pid_target_us':target if run.get('controller')=='visual_pid' else None,'period_includes_ack':run.get('controller')=='visual_pid' and not nb,**nb_timing}},ensure_ascii=False)+'\n');log.flush()
                        last_sample=sample;last_config=config
                    # PID cadence includes UART ACK time; do not add 100 ms after ACK.
                    if nb:kicked,kick_wait_ms=self._pace(now,kicked)   # uart-nonblock-r1; uart-pace-r1: kicked = how THIS iteration was triggered
                    elif run.get('controller')=='visual_pid':
                        self.cancel.wait(max(0., .1-(self.clock()-now)))
                    else:
                        time.sleep(.1)
        except Exception as e:reason=str(e)
        finally:
            for job in self.inflight:job['cancelled']=True   # uart-nonblock-r1: queued-but-unsent moves must not follow the STOP
            try:self.send({'action':'stop'})
            except Exception:pass
            if self.recorder:self.recorder.stop()
            with self.lock:
                run['input_diagnostics']['age_at_stop_ms']=round((self.clock()-self.last_input)*1000,1)
                run.update(status='stopped' if self.cancel.is_set() else 'failed',reason=reason,recording=False,ended_at=datetime.now(timezone.utc).isoformat())
                if sup:run['uart_ack_supersede']={'superseded':self.sup_total,'events':list(self.sup_events),'link_loss':self.link_loss,'link_loss_limit_s':self.nonblock['ack_link_loss_s']}   # uart-ack-supersede-r1
                atomic_json(folder/'summary.json',run)

    def snapshot(self, include_history=True):
        with self.lock:
            result={'current':json.loads(json.dumps(self.current)),'defaults':dict(self.defaults,center_us=self.profile()['center_us']),
                    'control_version':2,'firmware_required':FIRMWARE,'input_lease_ms':1000,'defaults_error':self.defaults_error}
        if not include_history:return result
        entries=[]
        for path in (self.root/'settings_history').glob('*.json'):
            try:entries.append(json.loads(path.read_text(encoding='utf-8')))
            except (ValueError,OSError):pass
        result['settings_history']=sorted(entries,key=lambda x:x['saved_at'],reverse=True)
        history=[]
        for path in self.root.glob('*/summary.json'):
            try:history.append(json.loads(path.read_text()))
            except (ValueError,OSError):pass
        result['history']=sorted(history,key=lambda x:x.get('created_at',''),reverse=True)[:30]
        return result


class SessionRecorder:
    def __init__(self,root):
        self.root=Path(root);self.lock=threading.RLock();self.run_id=None;self.writer=None;self.frames=None;self.index=0;self.segment=0;self.error=None

    def start(self,run_id):
        with self.lock:
            self.stop();self.run_id=run_id;self.segment+=1;self.index=0;self.error=None

    def write(self,frame,expected_id=None,capture_at=None):
        with self.lock:
            if not self.run_id or (expected_id is not None and self.run_id!=expected_id):return False
            import cv2
            folder=self.root/self.run_id;folder.mkdir(parents=True,exist_ok=True)
            if self.writer is None:
                self.writer=cv2.VideoWriter(str(folder/f'video_{self.segment:03d}.avi'),cv2.VideoWriter_fourcc(*'MJPG'),10,(frame.shape[1],frame.shape[0]))
                if not self.writer.isOpened():raise RuntimeError('Video writer unavailable')
                self.frames=(folder/f'frames_{self.segment:03d}.jsonl').open('w')
            self.writer.write(frame);self.frames.write(json.dumps({'frame':self.index,'time':datetime.now(timezone.utc).isoformat(),'capture_monotonic':capture_at})+'\n');self.frames.flush();self.index+=1
            return True

    def stop(self):
        with self.lock:
            self.run_id=None
            if self.writer:self.writer.release()
            if self.frames:self.frames.close()
            self.writer=self.frames=None
