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

FIRMWARE = 'orin-v1.4.1-manual'


def atomic_json(path, value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_suffix('.tmp')
    with temp.open('w',encoding='utf-8') as f:
        json.dump(value,f,ensure_ascii=False,indent=2);f.flush();os.fsync(f.fileno())
    temp.replace(path)


class ManualDriveManager:
    def __init__(self,root,state,send,profile,save_center,defaults_path,recorder=None,clock=time.monotonic):
        self.root=Path(root);self.state=state;self.send=send;self.profile=profile
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

    def _validate(self,p,profile):
        if not isinstance(p,dict):raise ValueError('Expected manual settings object')
        for key in ('right_pct','left_pct'):
            if type(p.get(key)) is not int or not 0<=p[key]<=25:raise ValueError('PWM must be an integer 0..25')
        center=p.get('center_us',profile['center_us'])
        if type(center) is not int or not profile['min_us']<center<profile['max_us']:raise ValueError('Center outside steering limits')
        rate=p.get('steer_rate_us_s',self.defaults['steer_rate_us_s'])
        if type(rate) is not int or not 50<=rate<=800:raise ValueError('Steering rate must be an integer 50..800 us/s')
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
        if d.get('uart_error') or d.get('uart_age_s',99)>1:raise ValueError('UART unavailable')
        if v.get('fw')!=FIRMWARE:raise ValueError('Flash '+FIRMWARE+' first')
        if not d.get('servo_settings_synced') or (v.get('state')!='READY' and not (v.get('web_manual_start') is True and v.get('state')=='LOCKED')) or v.get('pins',{}).get('D11')!=1:raise ValueError('Check synchronized settings, ESTOP and web-start authorization')
        if v.get('servo_returning'):raise ValueError('Steering is returning to center')
        if d.get('camera_error') or d.get('camera_age_s',99)>1:raise ValueError('Live camera required before driving')
        if any(v.get(k,{}).get('delta',0) for k in ('enc_a','enc_b')):raise ValueError('Wait for both wheels to stop')
        return v

    def start(self,p):
        if p.get('control_version')!=2:raise ValueError('Please refresh the manual page (Ctrl+F5)')
        if not isinstance(p.get('owner'),str) or not re.fullmatch('[0-9a-f]{32}',p['owner']):raise ValueError('Invalid page owner')
        if p.get('area_clear') is not True:raise ValueError('Confirm clear operating area')
        if type(p.get('record')) is not bool:raise ValueError('Choose recording state')
        with self.lock:
            if self.busy():raise ValueError('Manual driving already active')
            v=self._ready();profile=self.profile();config=self._validate(p,profile)
            # Initialize using the MCU's synchronized center; trims are applied afterward.
            run={'control_version':2,'id':uuid.uuid4().hex,'status':'preparing','reason':'','created_at':datetime.now(timezone.utc).isoformat(),
                 'generation':v['generation'],'config':config,'limits':profile,'recording':p['record'],
                 'command':{'right':0,'left':0,'steer_us':profile['center_us']},'last_input_seq':-1,'input_diagnostics':{'accepted':0,'max_gap_ms':0,'last_gap_ms':0}}
            self.current=run;self.owner=p['owner'];self.last_input=self.clock();self.input_seq=-1;self.intent=(0,0);self.hard_zero=False;self.centering=False
            self.cancel.clear();self.reason='MANUAL_STOP';folder=self.root/run['id'];folder.mkdir()
            atomic_json(folder/'summary.json',run)
            self.thread=threading.Thread(target=self._run,args=(run,),daemon=True);self.thread.start()
            return {'ok':True,'run_id':run['id']}

    def input(self,p):
        with self.lock:
            self._owned(p)
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

    def settings(self,p):
        with self.lock:
            if self.busy():self._owned(p);profile=self.current['limits']
            else:profile=self.profile()
            config=self._validate(p,profile)
            moving=self.busy() and (self.intent[0] or self.current['command']['right'] or self.current['command']['left'])
            if moving and abs(config['center_us']-self.current['config']['center_us'])>20:raise ValueError('While driving adjust center by at most 20 us')
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
        d=self.state()
        if d.get('uart_error') or d.get('uart_age_s',99)>1:raise ValueError('UART_LOST')
        if d.get('camera_error') or d.get('camera_age_s',99)>1:raise ValueError('CAMERA_LOST')
        return d

    def _send(self,p):
        self._check();return self.send(p)

    def _run(self,run):
        gen=run['generation'];right=left=offset=0;last_dir=0;zero_at=self.clock();last_sample=None;last_config=None;last_steer_at=self.clock();power_config=None;power_reference=(0,0)
        folder=self.root/run['id'];reason='UNKNOWN'
        try:
            with (folder/'controls.jsonl').open('a',encoding='utf-8') as log:
                self._send({'manual_action':'arm','generation':gen})
                self._send({'manual_action':'move','generation':gen,'right':0,'left':0,'us':run['limits']['center_us']})
                if run['recording'] and self.recorder:self.recorder.start(run['id'])
                with self.lock:run['status']='running'
                while True:
                    d=self._check();v=d.get('vehicle') or {}
                    if v.get('generation')!=gen or v.get('state')!='MANUAL':raise ValueError('AUTHORIZATION_CHANGED: '+str(v.get('reason')))
                    with self.lock:direction,steer=self.intent;config=dict(run['config']);hard_zero=self.hard_zero;centering=self.centering
                    now=self.clock()
                    elapsed=min(.2,max(0,now-last_steer_at));last_steer_at=now
                    previous_right,previous_left=right,left
                    reversing=bool(direction and last_dir and direction!=last_dir)
                    target_dir=0 if reversing else direction
                    def approach(value,target,amount):return value+max(-amount,min(amount,target-value))
                    if config!=power_config:
                        power_reference=(max(abs(right),config['right_pct'],1),max(abs(left),config['left_pct'],1));power_config=dict(config)
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
                    if steer:offset+=steer*step
                    elif centering:offset=approach(offset,0,step)
                    # Otherwise hold the steering command when A/D are released.
                    minimum,maximum=run['limits']['min_us'],run['limits']['max_us']
                    offset=max(minimum-config['center_us'],min(maximum-config['center_us'],offset))
                    pulse=round(config['center_us']+offset)
                    try:self._send({'manual_action':'move','generation':gen,'right':round(right),'left':round(left),'us':pulse})
                    except ValueError as e:
                        if 'DIRECTION_CHANGE_WAIT' not in str(e):raise
                        right=left=0;zero_at=now
                        self._send({'manual_action':'move','generation':gen,'right':0,'left':0,'us':pulse})
                    with self.lock:run['command']={'right':round(right),'left':round(left),'steer_us':pulse}
                    sample=v.get('enc_sample_ms')
                    if sample!=last_sample or config!=last_config:
                        log.write(json.dumps({'time':datetime.now(timezone.utc).isoformat(),'config':config,'intent':{'direction':direction,'steer':steer,'hard_zero':hard_zero,'center':centering},'input_event':run.get('last_input_event'),'input_diagnostics':dict(run['input_diagnostics']),'input_age_ms':round((self.clock()-self.last_input)*1000,1),'command':dict(run['command']),'vehicle':v},ensure_ascii=False)+'\n');log.flush()
                        last_sample=sample;last_config=config
                    time.sleep(.1)
        except Exception as e:reason=str(e)
        finally:
            try:self.send({'action':'stop'})
            except Exception:pass
            if self.recorder:self.recorder.stop()
            with self.lock:
                run['input_diagnostics']['age_at_stop_ms']=round((self.clock()-self.last_input)*1000,1)
                run.update(status='stopped' if self.cancel.is_set() else 'failed',reason=reason,recording=False,ended_at=datetime.now(timezone.utc).isoformat())
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
        self.root=Path(root);self.lock=threading.RLock();self.run_id=None;self.writer=None;self.frames=None;self.index=0;self.segment=0

    def start(self,run_id):
        with self.lock:
            self.stop();self.run_id=run_id;self.segment+=1;self.index=0

    def write(self,frame):
        with self.lock:
            if not self.run_id:return
            import cv2
            folder=self.root/self.run_id;folder.mkdir(parents=True,exist_ok=True)
            if self.writer is None:
                self.writer=cv2.VideoWriter(str(folder/f'video_{self.segment:03d}.avi'),cv2.VideoWriter_fourcc(*'MJPG'),10,(frame.shape[1],frame.shape[0]))
                if not self.writer.isOpened():raise RuntimeError('Video writer unavailable')
                self.frames=(folder/f'frames_{self.segment:03d}.jsonl').open('w')
            self.writer.write(frame);self.frames.write(json.dumps({'frame':self.index,'time':datetime.now(timezone.utc).isoformat()})+'\n');self.frames.flush();self.index+=1

    def stop(self):
        with self.lock:
            self.run_id=None
            if self.writer:self.writer.release()
            if self.frames:self.frames.close()
            self.writer=self.frames=None
