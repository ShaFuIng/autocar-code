"""Finite observation sessions and offline PP. Never emits vehicle commands.

Only injected read-only status/camera callbacks are available to this module.
Image timestamps are bounded by request/receive times, not claimed exposure times.
"""
import base64
import hashlib
import json
import math
import re
import threading
import time
import uuid
from pathlib import Path


def finite(n, lo, hi):
    if type(n) not in (int, float) or not math.isfinite(n) or not lo <= n <= hi:
        raise ValueError(f'數值須介於 {lo}～{hi}')
    return n


def pure_pursuit(points, lookahead, speed, rate, dt, previous_us=1300):
    """Metric rear-axle path -> curvature, then provisional outer-front geometry map.

    Geometry here is an explicit simulation assumption, NOT a validated calibration.
    Radius observation assumed outer-front tire edge; track 26.5, width 4.5,
    wheelbase 23 cm. No extrapolation of the unmeasured center region.
    """
    finite(lookahead, .05, 5);finite(speed, 0, 2);finite(rate, 1, 800);finite(dt, .001, 1)
    clean=[]
    for x,y in points:
        finite(x,-10,10);finite(y,-10,10)
        if x>0:clean.append((x,y))
    target=next(((x,y) for x,y in clean if math.hypot(x,y)>=lookahead),None)
    if target is None:return dict(valid=False,reason='NO_VISIBLE_LOOKAHEAD',pwm_scale=0,hardware_output=False)
    x,y=target;k=2*y/(x*x+y*y)
    # Positive y means left. Outer tire radius cannot be inverted directly.
    left=[(1210,2.15),(1120,1.5),(1030,1.065),(940,.825)]
    right=[(1600,1.95),(1750,1.065),(1900,.525)]
    table=left if k>0 else right
    mapped=[(1/(math.sqrt(r*r-.23**2)-(.265+.045)/2),us) for us,r in table]
    magnitude=abs(k);reason='INTERPOLATED_ASSUMED_GEOMETRY';valid=True
    if magnitude<1e-9:desired=1300;reason='STRAIGHT'
    elif magnitude<mapped[0][0]:desired=1300;reason='CENTER_MAPPING_UNMEASURED';valid=False
    elif magnitude>mapped[-1][0]:desired=mapped[-1][1];reason='CURVATURE_OUTSIDE_MEASURED_RANGE';valid=False
    else:
        desired=mapped[-1][1]
        for (a,u),(b,v) in zip(mapped,mapped[1:]):
            if a<=magnitude<=b:desired=u+(v-u)*(magnitude-a)/(b-a);break
    limited=previous_us+max(-rate*dt,min(rate*dt,desired-previous_us))
    return dict(valid=valid,reason=reason,target=[x,y],curvature_1pm=k,desired_us=desired,
                proposal_us=limited,rate_limited=abs(limited-desired)>.01,
                pwm_scale=1/(1+abs(k)*.35) if valid else 0,hardware_output=False,
                calibration='SIMULATION_ASSUMPTION_OUTER_FRONT_WHEEL_NOT_VALIDATED')


class Trials:
    def __init__(self,store,read_status=None,read_camera=None):
        self.store=store;self.root=store.root/'field_trials';self.root.mkdir(exist_ok=True)
        self.read_status=read_status;self.read_camera=read_camera
        self.lock=threading.RLock();self.active=None;self.cancel=threading.Event()
        for p in self.root.glob('*/meta.json'):
            d=json.loads(p.read_text(encoding='utf-8'))
            if d['status']=='recording':
                d.update(status='interrupted',reason='WORKBENCH_RESTART_NO_RESUME',ended=time.time());self.write(p,d)

    @staticmethod
    def write(path,value):
        tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(value,ensure_ascii=False,allow_nan=False,indent=2),encoding='utf-8')
        for attempt in range(20):
            try:tmp.replace(path);return
            except PermissionError:
                if attempt==19:raise
                time.sleep(.01)

    def folder(self,id):
        if not re.fullmatch('[0-9a-f]{32}',id):raise ValueError('無效試跑ID')
        p=self.root/id
        if not p.is_dir():raise ValueError('找不到試跑')
        return p

    def list(self):
        rows=[]
        for p in self.root.glob('*/meta.json'):
            d=json.loads(p.read_text(encoding='utf-8'))
            d['parameter_revision']=d['parameter_snapshot']['rev']
            d.pop('parameter_snapshot');d.pop('geometry_snapshot');rows.append(d)
        return sorted(rows,key=lambda d:d['created'],reverse=True)

    def detail(self,id):
        p=self.folder(id);d=json.loads((p/'meta.json').read_text(encoding='utf-8'))
        rows=[]
        for line in (p/'events.jsonl').read_text(encoding='utf-8').splitlines():
            try:rows.append(json.loads(line))
            except json.JSONDecodeError:pass # retain crash-partial tail on disk
        d['events']=rows
        reviews=p/'reviews.jsonl';d['reviews']=[json.loads(l) for l in reviews.read_text(encoding='utf-8').splitlines()] if reviews.exists() else []
        return d

    def start(self,p):
        mode=p.get('mode');duration=finite(p.get('seconds'),1,60)
        if mode not in ('synthetic','observe'):raise ValueError('僅支援合成影子／唯讀紀錄；實車控制執行尚未開放')
        segment=p.get('segment','').strip()
        if not segment or len(segment)>100:raise ValueError('請填路段名稱（100字以內）')
        scenario=p.get('scenario','straight')
        if scenario not in ('straight','left','right','dropout'):raise ValueError('無效影子情境')
        doc=self.store.get('tracking-proposal')
        if not doc['rev']:raise ValueError('請先到06保存參數提案')
        if p.get('revision')!=doc['rev']:raise ValueError('提案已更新，請重新載入後選擇版本')
        params=doc['data'].get('parameters',{})
        for key,lo,hi in [('pwm',0,25),('steer_rate',50,800),('lookahead_min',.1,5),('lookahead_max',.1,5),('lookahead_time',0,5),('curve_scale',0,1)]:finite(params.get(key),lo,hi)
        if params['lookahead_min']>params['lookahead_max']:raise ValueError('前視上下限不合法')
        if mode=='observe' and p.get('record_only_confirmed') is not True:raise ValueError('須確認限時只結束紀錄、不控制車輛')
        with self.lock:
            request_id=p.get('request_id','')
            if not re.fullmatch('[0-9a-f]{32}',request_id):raise ValueError('缺少單輪請求識別碼')
            for prior in self.list():
                if prior.get('request_id')==request_id:return prior
            if self.active:raise ValueError('已有單輪紀錄進行中，不重複啟動')
            id=uuid.uuid4().hex;folder=self.root/id;folder.mkdir()
            meta=dict(id=id,request_id=request_id,created=time.time(),status='recording',mode=mode,segment=segment,seconds=duration,
                scenario=scenario,hardware_output=False,parameter_snapshot=doc,
                geometry_snapshot=self.store.get('vehicle-measurements'),
                capture_mode='synthetic' if mode=='synthetic' else 'camera.jpg sampled snapshots / read-only telemetry',
                requested_hz=5 if mode=='synthetic' else 2,reason=None)
            meta['parameter_sha256']=hashlib.sha256(json.dumps(doc,sort_keys=True).encode()).hexdigest()
            self.write(folder/'meta.json',meta);(folder/'events.jsonl').touch();self.active=id;self.cancel.clear()
            threading.Thread(target=self.run,args=(meta,),daemon=True).start()
            return meta

    def stop(self,p):
        with self.lock:
            if p.get('id')!=self.active:raise ValueError('不是目前紀錄；不停止其他工作')
            self.cancel.set()
        return {'ok':True,'hardware_output':False,'message':'僅結束記錄；沒有送車輛停止命令'}

    def run(self,meta):
        folder=self.folder(meta['id']);begin=time.monotonic();last_us=1300;seq=0
        try:
            with (folder/'events.jsonl').open('a',encoding='utf-8') as f:
                while not self.cancel.is_set() and time.monotonic()-begin<meta['seconds']:
                    t=time.monotonic()-begin;row=dict(seq=seq,t=t,laptop_wall=time.time())
                    if meta['mode']=='synthetic':
                        sign={'left':1,'right':-1}.get(meta['scenario'],0)
                        # Synthetic path is visible from rear x=0.72 m, same local frame each sample.
                        pts=[[.72+i*.04,sign*.5*(.72+i*.04)**2] for i in range(25)]
                        if meta['scenario']=='dropout' and t>meta['seconds']/2:pts=[]
                        p=meta['parameter_snapshot']['data']['parameters'];v=.213 # labeled scenario assumption
                        look=max(p['lookahead_min'],min(p['lookahead_max'],v*p['lookahead_time']))
                        result=pure_pursuit(pts,look,v,p['steer_rate'],.2,last_us)
                        last_us=result.get('proposal_us',last_us)
                        scale=min(result['pwm_scale'],p['curve_scale']) if sign else result['pwm_scale']
                        row.update(path=pts,proposal=result,proposal_pwm=p['pwm']*scale,
                                   synthetic_speed_mps=v,telemetry=None,image=None)
                    else:
                        start=time.monotonic();state=self.read_status();end=time.monotonic()
                        if state.get('uart_age_s',99)>1 or state.get('camera_age_s',99)>1:raise ValueError('資料過期，結束唯讀記錄；未送車控命令')
                        row.update(telemetry=state,status_request_s=start-begin,status_received_s=end-begin)
                        start=time.monotonic();raw=self.read_camera();end=time.monotonic()
                        if not raw.startswith(b'\xff\xd8') or len(raw)>8*1024*1024:raise ValueError('相機影格格式無效')
                        name=f'{seq:06d}.jpg';(folder/name).write_bytes(raw)
                        row.update(image=name,image_request_s=start-begin,image_received_s=end-begin,
                                   exposure_time_unknown=True,alignment_uncertainty_s=end-start+state.get('camera_age_s',0),
                                   image_sha256=hashlib.sha256(raw).hexdigest(),proposal=None)
                    f.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n');f.flush();seq+=1
                    self.cancel.wait(.2 if meta['mode']=='synthetic' else .5)
            meta.update(status='completed',reason='USER_ENDED_RECORDING' if self.cancel.is_set() else 'RECORDING_DURATION_COMPLETE')
        except Exception as e:meta.update(status='failed',reason=str(e))
        finally:
            meta.update(ended=time.time(),samples=seq,elapsed_s=time.monotonic()-begin)
            try:self.write(folder/'meta.json',meta)
            finally:
                with self.lock:self.active=None

    def review(self,p):
        folder=self.folder(p.get('id',''));text=p.get('note','')
        if not isinstance(text,str) or not text.strip() or len(text)>2000:raise ValueError('評語須1～2000字')
        t=finite(p.get('t'),0,60)
        meta=json.loads((folder/'meta.json').read_text(encoding='utf-8'))
        if t>meta.get('elapsed_s',meta['seconds']):raise ValueError('評語時間超出本輪紀錄')
        d=dict(id=uuid.uuid4().hex,t=t,note=text,created=time.time(),source='human_observation')
        with self.lock:
            with (folder/'reviews.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(d,ensure_ascii=False)+'\n')
        return d
