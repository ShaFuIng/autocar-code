"""Orin-side 07 candidate. Importing this module never opens a device or socket.

Only the generated package's explicitly called API constructs Runtime. The laptop
uses Candidate/FakeExecutor, not this actuator adapter. No deployment in here.
"""
import copy
import json
import math
import os
from pathlib import Path
import threading
import time
import uuid
from urllib.request import urlopen
from ops07_route import entry_plan, seed_curve_map, seed_dash, seed_track, LapClock

REVISION='ops07-orin-r3'


def atomic_json(path, value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_name(path.name+'.tmp')
    with temp.open('w',encoding='utf8') as f:
        json.dump(value,f,ensure_ascii=False,allow_nan=False);f.flush();os.fsync(f.fileno())
    os.replace(temp,path)


def read_go(url):
    try:
        with urlopen(url,timeout=.2) as r:
            return {b'0':0,b'1':1}.get(r.read(9).strip()) if r.status==200 else None
    except (OSError,ValueError):return None


def continuous(run):
    return bool(run and run.get('execution_mode')=='orin_continuous')


def deadline_expired(run, now):
    if continuous(run):
        if run.get('deadline') is not None:raise ValueError('07 deadline must be null')
        return False
    return now>=run['deadline']


def manual_hold(manual, hold):
    """Latch at the output producer as well as the perception consumer."""
    with manual.lock:
        run=manual.current
        if not run or not continuous(run):raise ValueError('Not a 07 manual session')
        run['ops07_hold']=hold
        if hold:
            run['pid_target_us']=run['limits']['center_us'];run['pid_pwm']=0
            run['pid_pwm_rl']=None;manual.intent=(0,0);manual.hard_zero=True
        manual.kick.set()


# Clock values audited against the packaged classes. Configuration and sensor
# capture timestamps are never shifted. Short-lived control filters are rebuilt.
def rebase_route(live, dt):
    clocks={
        'cmap':('t','quiet','odo_t'),
        'dash':('climb_since','level_since','opened','last_seen','t_prev','follow_since','delay_since','delay_last','rearm_since'),
        'tpos':('last_t',), 'heading':('ref_t',), 'upath':('t',),
        'speed':('t','e_t','since'), 'entry':('blind_t0','slope_t0','up_t0'),
        'bottom':('t0','pe_t','hold_t0'), 'bcommit':('t0','prev_t'),
        'lane':('t','t_line','p_t'), 'yin':('t','steep_t','crest_t','climb_t0','ucam_t'),
        'cff':('t',), 'htm':('t','gz_t'), 'dipg':('wait_t0',), 'sslew':('last_t',),
        'crest':('last_pitched','since'), 'solid':('t0',), 'ulat':('t',),
        'diff':('t',), 'gdamp':('t',),
    }
    for name, fields in clocks.items():
        obj=getattr(live,name,None)
        if obj is None:continue
        for key in fields:
            val=getattr(obj,key,None)
            if type(val) in (int,float):setattr(obj,key,val+dt)
    if live.cmap.cur:
        live.cmap.cur['t0']+=dt
    for key in ('down_until','_af_t0','_xp_last','_xc_t'):
        val=getattr(live,key,None)
        if type(val) in (int,float):setattr(live,key,val+dt)
    for name,key,fields in (('speed','dh',('t0','last','steep_t')),
                            ('mid','t',('L','R')),('cff','last_opp',(-1,1))):
        d=getattr(getattr(live,name,None),key,None)
        if isinstance(d,dict):
            for field in fields:
                if type(d.get(field)) in (int,float):d[field]+=dt
    for key,fields in (('_slg',('t','t0')),('jg',('prev_t','since','sat_t'))):
        d=getattr(live,key,None)
        if isinstance(d,dict):
            for field in fields:
                if type(d.get(field)) in (int,float):d[field]+=dt
    for key in ('_last_post_turn','_cs','_ar_v','_sw_m','_jf_prev'):
        val=getattr(live,key,None)
        if isinstance(val,tuple) and type(val[0]) in (int,float):setattr(live,key,(val[0]+dt,*val[1:]))
    up=getattr(live,'upath',None)
    if up is not None and getattr(up,'po',None) is not None:up.po=(up.po[0]+dt,*up.po[1:])
    for name,key in (('entry','lane'),('dspeed','hist'),('wspeed','hist')):
        obj=getattr(live,name,None);rows=getattr(obj,key,None)
        if isinstance(rows,list):setattr(obj,key,[(row[0]+dt,*row[1:]) for row in rows])
    # Actual center commands were sent during the pause: don't replay old
    # steering-hold or estimator samples as if the car had kept turning.
    if getattr(live,'hold',None) is not None:live.hold.__init__()
    if getattr(live,'centre_est',None) is not None:
        live.centre_est.cmd=[];live.centre_est.gz=[]
    # Rebase history timestamps, not values or integrated yaw.
    h=getattr(live,'heading',None)
    if h:
        for key in ('hist','dd_hist'):
            rows=getattr(h,key,None)
            if isinstance(rows,list):
                setattr(h,key,[(r[0]+dt,*r[1:]) for r in rows])
        if h.track is not None:h.track=(h.track[0]+dt,*h.track[1:])


STALE=('_xp_ends','_xp_end','_xp_out','_xp_last','_xp_rate','_xc_t','_af_cur','_af_ok','_af_t0','_af_why',
       '_ar_ok','_ar_prev','_ar_v','_ar_why','_sw_m','_slg','_slg_arc','_slg_map','_jf_n','_jf_prev',
       '_last_post_turn','_cs','_cm_speed','_xp_c','_af_c','_ar_c','_sw_c','_sw_c2','_slg_c','_dos_c','_mt_c','upath','tpos')


def apply_seed(live, plan):
    from track_pos import TrackPos
    for key in STALE:
        if hasattr(live,key):delattr(live,key)
    seed_curve_map(live.cmap,plan);seed_dash(live.dash,plan)
    live.route_lap=plan['route_lap'];live.tpos=TrackPos();seed_track(live.tpos,plan)
    if plan['package']=='latest-2lap-97ed':live._xp_ends=dict(plan['crossing'])
    elif plan['package']=='best-today-46db':live._xp_end=plan['crossing'].get(0)
    if plan['swing_end'] is not None:live._sw_c2=plan['swing_end']


class Runtime:
    def __init__(self, live, package, root, clock=time.monotonic, flag_reader=read_go):
        self.live,self.package,self.clock=live,copy.deepcopy(package),clock
        self.path=Path(root)/'ops07-session.json';self.flag_reader=flag_reader
        self.token=None;self.saw_zero=False;self.flag=None;self.flag_url=None
        self.holding=True;self.last=self.clock();self.last_save=self.last;self.paused_at=self.last
        self.clockwork=LapClock();self.ticket=object();self.closed=False
        self.last_mcu_ms=None
        self.session=dict(id=None,state='idle',revision=REVISION,package_identity=package['identity'],
                          origin=None,entry=None,run_ids=[],events=[],center_status='unconfirmed')
        if self.path.exists():
            self.session=json.loads(self.path.read_text(encoding='utf8'))
            # Never carry tokens, monotonic times or automatic release across boot.
            self.session.update(revision=REVISION,state='paused' if self.session.get('id') else 'idle',needs_location=bool(self.session.get('id')),reason='PROCESS_RESTART',center_status='unconfirmed')
            saved=self.session.get('clock',{})
            for k in ('active_s','wall_s','lap_active','lap_wall','laps'):
                if k in saved:setattr(self.clockwork,k,copy.deepcopy(saved[k]))
            self.clockwork.relocate()
        self.persist()

    def event(self,kind,**kw):
        event=dict(kind=kind,wall=time.time(),**kw)
        self.session['events'].append(event)
        self.session['events']=self.session['events'][-100:]
        if self.session['id']:
            path=self.path.parent/('ops07-'+self.session['id']+'.jsonl')
            with path.open('a',encoding='utf8') as f:
                f.write(json.dumps(event,ensure_ascii=False)+'\n');f.flush()

    def persist(self):
        try:
            self.session['clock']=copy.deepcopy(vars(self.clockwork))
            atomic_json(self.path,self.session);self.last_save=self.clock()
        except Exception:
            self.holding=True;self.token=None
            self.live.manual.request_stop('OPS07_RECORDING_FAILED')
            self.session.update(state='fault',reason='OPS07_RECORDING_FAILED')
            raise

    def snapshot(self):
        with self.live.lock:
            out=copy.deepcopy(self.session)
            out.update(token=self.token,hold=self.holding,flag=self.flag,saw_zero=self.saw_zero,
                       clock=copy.deepcopy(vars(self.clockwork)),deadline=None)
            return out

    def validate_token(self,p):
        if not self.token or p.get('token')!=self.token or p.get('session_id')!=self.session['id']:
            raise ValueError('這是舊的操作要求，請重新讀取本場狀態。')

    def center_ready(self):
        m=self.live.manual;d=m.state();v=d.get('vehicle') or {};run=m.current or {}
        # These are MCU command acknowledgements, not measured wheel angle.
        ok=(not d.get('uart_error') and d.get('uart_age_s',99)<=1 and
            v.get('generation')==run.get('generation') and v.get('state')=='MANUAL' and
            v.get('pins',{}).get('D11')==1 and not v.get('servo_returning') and
            v.get('pca_ok') is True and v.get('steer_output_enabled') is True and
            v.get('steer_us_command')==run.get('limits',{}).get('center_us') and
            all(v.get(k,{}).get('pwm_command')==0 and v.get(k,{}).get('target')==0 for k in ('motor_a','motor_b')) and
            run.get('command',{}).get('steer_us')==run.get('limits',{}).get('center_us') and
            run.get('command',{}).get('right')==0 and run.get('command',{}).get('left')==0 and
            all(v.get(k,{}).get('delta')==0 for k in ('enc_a','enc_b')))
        self.session['center_status']='command_confirmed' if ok else 'unconfirmed'
        return ok

    def command(self, action, p):
        l=self.live
        if action=='load':
            with l.operation_lock:
                pars=self.package['parameters']
                with l.lock:
                    if l.manual.busy() or l.motor_busy():raise ValueError('請先停止車控，才能載入模型。')
                    worker=l.worker
                    matches=(getattr(l,'model_id',None)==self.package['recognition']['model_id'] and
                        getattr(l,'path_alias',None)==self.package['path_algorithm'] and
                        getattr(l,'row_ratio',None)==pars['row_ratio'] and
                        getattr(l,'connector_center_offset',0)==pars.get('visual_center_offset_px_640',0) and
                        getattr(l,'curve_outward_px_640',0)==pars.get('curve_outward_px_640',0))
                    if worker and worker.is_alive() and matches:return dict(ok=True,hardware_output=False,reused=True)
                    if worker and worker.is_alive():l.stop_preview()
                # The worker needs live.lock to exit; never join while holding it.
                if worker and worker.is_alive():
                    worker.join(8)
                    if worker.is_alive():raise ValueError('舊預覽尚未結束，請稍後重新載入。')
                return l.prepare(dict(model_id=self.package['recognition']['model_id'],
                    path_algorithm=self.package['path_algorithm'],row_ratio=pars['row_ratio'],
                    visual_center_offset_px_640=pars.get('visual_center_offset_px_640',0),
                    curve_outward_px_640=pars.get('curve_outward_px_640',0)))
        with l.operation_lock,l.lock:
            if action=='arm-hold':
                if self.session['id']:raise ValueError('本場仍在，請接續或重置。')
                if type(p.get('traffic_light')) is not bool:raise ValueError('請確認紅綠燈選項。')
                if p.get('front_car'):raise ValueError('前車辨識／距離來源尚未提供，不能啟用。')
                plan=entry_plan(self.package['id'],p.get('location'),p.get('phase','first'),x_m=p.get('x_m'))
                if p['traffic_light'] and not self.flag_url:raise ValueError('尚未設定場地起跑訊號來源。')
                self.session.update(id=uuid.uuid4().hex,origin=plan['location'],entry=plan,
                                    traffic_light=p['traffic_light'],needs_location=False)
                self.clockwork=LapClock(plan['location'])
                self.arm(p,plan)
                if p.get('start_when_ready') is True and not p['traffic_light']:
                    self.session['resume_release']=True;self.persist()
            elif action=='release':
                self.validate_token(p)
                if self.session.get('traffic_light') and self.session['state']=='waiting':
                    raise ValueError('正在等待紅綠燈，由 Orin 偵測放行。')
                self.release(p)
            elif action=='pause':
                self.validate_token(p)
                if not continuous(l.run) or l.run['state']!='running':raise ValueError('車控已停止，請確認故障。')
                self.account();self.holding=True;self.saw_zero=False;self.paused_at=self.clock()
                manual_hold(l.manual,True)
                self.session.update(state='paused',center_status='returning')
                self.event('pause');self.checkpoint();self.persist()
            elif action=='resume':
                if p.get('session_id')!=self.session['id'] or self.session['state'] not in ('paused','fault'):
                    raise ValueError('接續場次不符。')
                if self.session['package_identity']!=self.package['identity']:raise ValueError('請載入本場原方案。')
                if type(p.get('moved')) is not bool:raise ValueError('請確認是否搬車。')
                alive=continuous(l.run) and l.run['state']=='running' and l.manual.busy()
                if p['moved'] or self.session.get('needs_location') or not alive:
                    if p.get('location_confirmed') is not True:raise ValueError('故障或搬車後請確認現在的入口位置。')
                    plan=entry_plan(self.package['id'],p.get('location'),p.get('phase','first'),x_m=p.get('x_m'),
                                    route_lap=p.get('route_lap',0))
                    if alive:l.end('OPS07_RELOCATE')
                    if l.manual.busy():raise ValueError('車控正在結束；狀態停止後再接續。')
                    self.clockwork.relocate();self.arm(p,plan)
                else:
                    if p.get('location',self.session['entry']['location'])!=self.session['entry']['location']:
                        raise ValueError('改位置請勾選搬車。')
                    self.token=uuid.uuid4().hex;self.session['state']='centering'
                    self.event('resume_requested');self.persist()
                # Remain centered until fresh telemetry and explicit release;
                # tick handles this user's resume request, without browser lease.
                self.session['resume_release']=True;self.persist()
            elif action=='stop':
                self.validate_token(p);l.end('OPS07_USER_STOP')
            elif action=='reset':
                if l.manual.busy():raise ValueError('請先停止車控，再重置。')
                if self.session['id']:atomic_json(self.path.with_name('ops07-archive-'+self.session['id']+'.json'),self.snapshot())
                self.token=None;self.session.update(id=None,state='idle',origin=None,entry=None,run_ids=[],events=[],needs_location=False)
                self.clockwork=LapClock();self.persist()
            else:raise ValueError('Unknown 07 action')
            return self.snapshot()

    def arm(self,p,plan):
        l=self.live;self.holding=True;self.saw_zero=False;self.token=uuid.uuid4().hex
        self.last_mcu_ms=None
        self.paused_at=self.clock();self.session.update(state='centering',entry=plan,center_status='returning',resume_release=False)
        self.persist()
        request=dict(owner=p.get('owner'),area_clear=p.get('area_clear'),parameters=self.package['parameters'],
                     parameter_revision=self.package['parameter_revision'],path_algorithm=self.package['path_algorithm'],
                     execution_mode='orin_continuous',seconds=None,_ops07_ticket=self.ticket)
        try:
            result=l.start(request)
            apply_seed(l,plan)
            self.session['run_ids'].append(result['run_id'])
            self.session['needs_location']=False
            self.session['generation']=l.manual.current['generation']
            self.event('arm_hold',run_id=result['run_id'],entry=plan);self.persist()
        except Exception:
            l.end('OPS07_ARM_FAILED');self.token=None;self.session['state']='fault';self.persist();raise

    def release(self,p):
        self.validate_token(p);l=self.live
        if self.session['state']=='running':return  # same-token acknowledgement only
        if self.session['state'] not in ('ready','waiting','centering'):raise ValueError('尚未待發。')
        if not continuous(l.run) or l.run['state']!='running' or not l.manual.busy():raise ValueError('車控已停止。')
        if not self.center_ready():raise ValueError('等待雙輪停止及保存中心命令確認。')
        if not l.latest or not l.latest.get('valid') or l.consecutive<3 or self.clock()-l.latest['capture_at']>l.run['parameters']['max_gap']:
            raise ValueError('等待新的有效路徑；不會使用暫停前的影格放行。')
        if (l.manual.current or {}).get('generation')!=self.session['generation']:raise ValueError('控制板已重啟，請重新確認位置。')
        rebase_route(l,self.clock()-self.paused_at)
        l.controller=type(l.controller)(l.run['parameters'])
        l.start_t=self.clock();l.start_centred=False
        self.account();self.session.update(state='running',resume_release=False)
        self.event('release',token=self.token);self.persist()
        manual_hold(l.manual,False);self.holding=False;self.saw_zero=False

    def account(self):
        now=self.clock();dt=max(0.,now-self.last);self.last=now
        if self.session['id']:self.clockwork.tick(dt,self.session['state']=='running')

    def checkpoint(self):
        l=self.live
        if getattr(l,'cmap',None):
            self.session['route_checkpoint']=dict(seg=l.cmap.seg,idx=l.cmap.idx,dist=l.cmap.dist,
                cur=copy.deepcopy(l.cmap.cur),dash=l.dash.state,route_lap=l.dash.lap)

    def ended(self,reason):
        self.account();self.holding=True;self.token=None;self.saw_zero=False
        self.session.update(state='fault' if reason not in ('OPS07_USER_STOP','OPS07_RELOCATE') else 'paused',
                            reason=reason,needs_location=True,center_status='unconfirmed',resume_release=False)
        self.checkpoint();self.event('stop',reason=reason);self.persist()

    def tick(self):
        # Called by the Orin watcher; status reads have no side effects.
        with self.live.lock:
            self.account()
            if continuous(self.live.run) and self.live.run['state']=='running':
                d=self.live.manual.state();v=d.get('vehicle') or {};ms=v.get('ms')
                if d.get('uart_age_s',99)<=1 and type(ms) is int:
                    if self.last_mcu_ms is not None and ms<self.last_mcu_ms:
                        self.live.end('OPS07_MCU_CLOCK_REGRESSED');return
                    self.last_mcu_ms=ms
                if self.session['state']=='paused':self.center_ready()
            if self.session['state']=='centering' and self.center_ready():
                self.session['state']='waiting' if self.session.get('traffic_light') and not self.session.get('resume_release') else 'ready'
                self.saw_zero=False;self.persist()
            if self.session.get('resume_release') and self.session['state']=='ready':
                try:self.release(dict(token=self.token,session_id=self.session['id']))
                except ValueError:pass
            if self.clock()-self.last_save>=1 and self.session['id']:
                self.checkpoint();self.persist()

    def poll_flag(self):
        # Network outside live.lock. Recheck token/state after a slow response.
        with self.live.lock:
            if self.session['state']!='waiting':return
            token=self.token;url=self.flag_url
        value=self.flag_reader(url)
        with self.live.lock:
            if token!=self.token or self.session['state']!='waiting':return
            self.flag=value
            if value is None:self.saw_zero=False
            elif value==0:self.saw_zero=True
            elif self.saw_zero:
                try:self.release(dict(token=token,session_id=self.session['id']))
                except ValueError:self.saw_zero=False

    def observe(self, proposal):
        l=self.live;cm=proposal.get('curve_map') or {}
        pos=l.tpos.step(self.clock(),cm.get('dist_m'),l.dash.state,cm)
        if pos:
            proposal['track_pos']=pos;self.session['position']=pos
            crossing=self.clockwork.observe(pos)
            if crossing:self.event('estimated_flag',**crossing);self.persist()


def get_runtime(live):
    if getattr(live,'ops07',None) is None:
        package=json.loads(Path(__file__).with_name('ops07-package.json').read_text(encoding='utf8'))
        runtime=Runtime(live,package,live.manual.root)
        cfg=Path(__file__).with_name('ops07-site.json')
        if cfg.exists():runtime.flag_url=json.loads(cfg.read_text(encoding='utf8')).get('flag_url')
        live.ops07=runtime
        def poll():
            while not runtime.closed:
                started=time.monotonic()
                try:runtime.poll_flag()
                except Exception:
                    with live.lock:live.end('OPS07_FLAG_WORKER_ERROR')
                time.sleep(max(.001,.05-(time.monotonic()-started)))
        threading.Thread(target=poll,daemon=True,name='ops07-flag').start()
    return live.ops07


def api(live,action,p):
    with live.operation_lock:return get_runtime(live).command(action,p)


def holding_frame(live,result,captured,frame,seq,preview):
    r=getattr(live,'ops07',None)
    if not r or not continuous(live.run) or live.run['state']!='running' or not r.holding:return False
    now=r.clock()
    valid=result.get('valid') is True and bool(result.get('path')) and now-captured<=live.run['parameters']['max_gap']
    live.latest=dict(result,capture_at=captured,frame_seq=frame,valid=valid,consumed_at=now,control_age_s=now-captured)
    live.consecutive=live.consecutive+1 if valid else 0;live.state='ready' if valid else 'no_reliable_path'
    if preview is not None:live.preview=preview;live.preview_seq=seq
    # No DashRoute, CurveMap, speed or PID update occurs while waiting/paused.
    live._pid_send(dict(owner=live.owner,run_id=live.run['id'],seq=seq,
                        target_us=live.run['parameters']['center_us'],pwm=0))
    live.log.write(json.dumps(dict(kind='ops07_hold',seq=seq,capture_at=captured,valid=valid,state=r.session['state']))+'\n')
    return True
