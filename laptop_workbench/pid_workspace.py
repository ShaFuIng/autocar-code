"""Local PID field records. No hardware access; raw source files stay read-only."""
import json, math, re, time, uuid, hashlib, threading
from pathlib import Path

def summarize(events, controls):
    start=next((e for e in events if e.get('kind')=='start'),{})
    samples=[e for e in events if e.get('kind')=='sample']
    errors=[e.get('proposal',{}).get('error') for e in samples]
    errors=[e for e in errors if type(e) in (int,float) and math.isfinite(e)]
    stop=next((e for e in reversed(events) if e.get('kind')=='stop'),{})
    cmds=[e.get('command',{}).get('steer_us') for e in controls]
    cmds=[v for v in cmds if type(v) in (int,float)]
    return dict(id=start.get('id'),created=start.get('started'),parameters=start.get('parameters',{}),
        revision=start.get('parameter_revision'),model=start.get('model_id'),weights=start.get('weights_sha256'),
        seconds=start.get('seconds'),actual_seconds=max(0,stop.get('at',start.get('started',0))-start.get('started',0)),
        reason=stop.get('reason','紀錄未完整'),samples=len(samples),
        mean_abs_error=sum(map(abs,errors))/len(errors) if errors else None,
        first_error=errors[0] if errors else None,last_error=errors[-1] if errors else None,
        command_range=[min(cmds),max(cmds)] if cmds else None,
        line_contact='unknown',line_contact_source='需現場評語；未做車身投影標定')

# summarize() of each synced run persists to disk: reading every pid.jsonl/controls.jsonl (1.4 GB) on each request hung page 05.
_CODE=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()[:16];_LOCK=threading.Lock();_CACHE=None
def _cached(store_root,key,compute):
    global _CACHE
    disk=store_root/'pid_workspace_summary_cache.json'
    with _LOCK:
        if _CACHE is None:
            try:d=json.loads(disk.read_text(encoding='utf-8'));_CACHE=d['summaries'] if d.get('code')==_CODE else {}
            except (OSError,ValueError,KeyError):_CACHE={}
        if key not in _CACHE:_CACHE[key]=compute();_CACHE['.dirty']=1
        return json.loads(json.dumps(_CACHE[key]))
def _flush(store_root):
    # written once per request; OneDrive may briefly lock the file, and a failed write only means the next restart re-parses
    with _LOCK:
        if not _CACHE or not _CACHE.pop('.dirty',None):return
        disk=store_root/'pid_workspace_summary_cache.json';tmp=disk.with_suffix('.tmp')
        for _ in range(10):
            try:tmp.write_text(json.dumps(dict(code=_CODE,summaries=_CACHE),ensure_ascii=False),encoding='utf-8');tmp.replace(disk);return
            except OSError:time.sleep(.3)
        _CACHE['.dirty']=1

class Workspace:
    def __init__(self,app):self.app=app
    def records(self):
        result=[]
        for item in self.app.orin.local_records():
            if item['kind']!='manual':continue
            folder=self.app.orin.local_folder('manual',item['id'])
            if not (folder/'pid.jsonl').exists():continue
            row=_cached(self.app.store.root,item['id']+'/'+item['version'],lambda:summarize(self.read_lines(folder/'pid.jsonl'),self.read_lines(folder/'controls.jsonl')))
            row.update(id=item['id'],replay_id='orin-manual-'+item['id'],source_version=item['version'])
            exclusions=self.app.store.root/'pid-control-exclusions.json'
            exclusion=json.loads(exclusions.read_text(encoding='utf-8')).get(item['id']) if exclusions.exists() else None
            row['control_exclusion']=exclusion
            row['control_valid']=False if exclusion else None
            if exclusion:
                row['diagnostic_mean_abs_error']=row['mean_abs_error'];row['mean_abs_error']=None;row['first_error']=None;row['last_error']=None
            row['context']=self.app.store.get('pid-session-'+item['id'])['data']
            result.append(row)
        _flush(self.app.store.root)
        return sorted(result,key=lambda r:r.get('created') or 0,reverse=True)
    @staticmethod
    def read_lines(path):
        rows=[]
        if path.exists():
            for line in path.read_text(encoding='utf8').splitlines():
                try:rows.append(json.loads(line))
                except ValueError:pass
        return rows
    def reviews(self):
        with self.app.store.lock:
            return [json.loads(r[0]) for r in self.app.store.db.execute("SELECT body FROM documents WHERE key LIKE 'pid-field-review-%' ORDER BY rowid DESC")]
    def add_review(self,p):
        run=p.get('run','')
        if not re.fullmatch('[0-9a-f]{32}',run) or run not in {x['id'] for x in self.records()}:raise ValueError('請先同步並選擇實際PID紀錄')
        if p.get('tracking') not in ('未確認','直行','偏左','偏右','左右擺動'):raise ValueError('軌跡選項無效')
        if p.get('contact') not in ('未確認','未見壓線','壓左線','壓右線','兩側壓線'):raise ValueError('壓線選項無效')
        seconds=p.get('at_s')
        duration=next(r['actual_seconds'] for r in self.records() if r['id']==run)
        if seconds is not None and (type(seconds) not in (int,float) or not math.isfinite(seconds) or not 0<=seconds<=duration+.5):raise ValueError('評語時間超出本輪紀錄')
        notes=p.get('notes','')
        if not isinstance(notes,str) or len(notes)>3000:raise ValueError('評語最多3000字')
        row=dict(id=uuid.uuid4().hex,run=run,created=time.time(),tracking=p['tracking'],contact=p['contact'],at_s=seconds,notes=notes,source='使用者現場判讀，非自動偵測')
        self.app.store.save('pid-field-review-'+row['id'],0,row)
        return row
    def geometry(self):
        doc=self.app.store.get('vehicle-measurements');items=doc['data'].get('items',{})
        def val(k):
            v=items.get(k,{}).get('value')
            return v if type(v) in (int,float) and math.isfinite(v) else None
        near,forward=val('camera_near_ground'),val('camera_forward')
        return dict(document=doc,front_axle_blind_cm=near+forward if near is not None and forward is not None else None,
            near_camera_cm=near,camera_forward_cm=forward,envelope_width_cm=val('envelope_width'),
            envelope_length_cm=val('envelope_length'),automatic_line_contact=False,
            limitation='換胎後需重測；前軸距離不是輪胎最前緣距離。未完成相機地面標定，不能從影像推算實際壓線。')

    def preview(self,p):
        from visual_pid import PID,validate,BACKLASH_DEFAULTS
        run=p.get('run','')
        if run not in {x['id'] for x in self.records()}:raise ValueError('請選擇已同步PID紀錄')
        params=validate(p.get('parameters'))
        controller=PID(params);baseline=PID(dict(params,backlash_enabled=0,backlash_initial_enabled=0));single=PID(dict(params,direction_enabled=0));rows=[]
        folder=self.app.orin.local_folder('manual',run)
        for event in self.read_lines(folder/'pid.jsonl'):
            if event.get('kind')!='sample':continue
            vision=event['vision'];t=vision['capture_at'];error=vision.get('error') if vision.get('valid') else None
            path=vision.get('path',[]) if vision.get('valid') else []
            r=controller.step_path(t,path);r['baseline_us']=baseline.step_path(t,path).get('steer_us');r['single_point_us']=single.step_path(t,path).get('steer_us');rows.append(r)
        result=dict(run=run,parameters=params,rows=rows,hardware_output=False,
                    interpretation='沿用原試跑的偏差時間線重算命令；不是閉環模擬，不能推論補償後車身位置。')
        key='pid-backlash-preview-'+uuid.uuid4().hex
        self.app.store.save(key,0,result)
        return dict(id=key,**result)
