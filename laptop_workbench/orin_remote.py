"""Restricted Orin adapter. Never imports control modules or opens devices."""
import base64,fcntl,hashlib,io,json,os,re,shutil,subprocess,sys,tarfile,time,urllib.request,urllib.error,uuid
from pathlib import Path
HOME=Path.home();ROOT=HOME/'orin_interface_v1';STATE=HOME/'.local/share/laptop_workbench';STATE.mkdir(parents=True,exist_ok=True)
ROOTS={'record':ROOT/'logs/record_only','shadow':ROOT/'shadow_workbench/runs','manual':ROOT/'logs/manual_driving'}
GETS={'/api/status','/api/record/status','/camera.jpg'}
POSTS={'/api/record/start','/api/record/stop'}
def save(p,d):
    tmp=p.with_suffix('.tmp');tmp.write_text(json.dumps(d));os.replace(tmp,p)
def request(path,post=False):
    if path not in (POSTS if post else GETS):raise ValueError('Endpoint not allowed')
    credential=base64.b64encode(('jetson:'+(HOME/'.config/orin_lab/token').read_text().strip()).encode()).decode()
    last=None
    for host in ['127.0.0.1']+subprocess.check_output(['hostname','-I'],text=True).split():
        if ':' in host:continue
        base='http://'+host+':8000';headers={'Authorization':'Basic '+credential}
        if post:headers.update(Origin=base,**{'X-Orin-Control':'1','Content-Type':'application/json'})
        try:
            with urllib.request.urlopen(urllib.request.Request(base+path,data=b'{}' if post else None,headers=headers),timeout=4) as r:return r.read()
        except urllib.error.HTTPError:raise RuntimeError('Orin API rejected request')
        except OSError as exc:last=exc
    raise RuntimeError('Orin API unreachable: '+type(last).__name__)
def get(path):return json.loads(request(path))
def jid(s):
    if not re.fullmatch('[0-9a-f]{32}',str(s)):raise ValueError('Invalid job ID')
    return STATE/(s+'.json')
def services():
    return subprocess.check_output(['systemctl','--user','show','orin-dashboard.service','orin-shadow.service','-p','Id','-p','ActiveState','-p','ActiveEnterTimestamp','-p','NRestarts'],text=True)
def status():
    s=get('/api/status');v=s.get('vehicle') or {}
    jobs=[]
    for p in sorted(STATE.glob('*.json'),key=lambda p:p.stat().st_mtime,reverse=True)[:10]:
        try:jobs.append(json.loads(p.read_text()))
        except ValueError:pass
    return dict(at=time.time(),vehicle={k:v.get(k) for k in ['fw','state','reason','generation','voltage_v','current_a','speed_mps','motor_a','motor_b','pca_ok','i2c_errors']},telemetry={k:s.get(k) for k in ['uart_age_s','uart_error','camera_age_s','camera_error','motor_busy','servo_settings_synced']},recording=get('/api/record/status'),services=services(),disk_free_bytes=shutil.disk_usage(ROOT).free,jobs=jobs)
def current():return get('/api/record/status').get('current') or {}
def start(payload):
    seconds=payload.get('duration_s');job=payload.get('job');p=jid(job)
    if type(seconds) not in (int,float) or not 1<=seconds<=120:raise ValueError('Duration must be 1..120s')
    with (STATE/'record.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        if p.exists():return json.loads(p.read_text())
        if current().get('status')=='recording':raise ValueError('Existing recording left untouched')
        for other in STATE.glob('*.json'):
            row=json.loads(other.read_text())
            if row.get('state') in ('queued','recording','stop_pending'):raise ValueError('Owned recording job already active')
        row=dict(job=job,state='queued',duration_s=seconds,created=time.time());save(p,row)
        with (STATE/(job+'.log')).open('wb') as log:
            subprocess.Popen([sys.executable,__file__,'worker',base64.b64encode(json.dumps(payload).encode()).decode()],stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True,close_fds=True)
        return row
def worker(payload):
    p=jid(payload['job']);row=json.loads(p.read_text());owned=None
    try:
        with (STATE/'record.lock').open('w') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX)
            before=current()
            if before.get('status')=='recording':raise ValueError('Existing recording left untouched')
            s=get('/api/status');v=s.get('vehicle') or {}
            if v.get('state')!='LOCKED' or s.get('motor_busy'):raise ValueError('Recording requires observed LOCKED idle vehicle')
            if any((v.get(k) or {}).get('pwm_command',0)!=0 or (v.get(k) or {}).get('target',0)!=0 for k in ('motor_a','motor_b')):raise ValueError('Nonzero motor telemetry')
            # Remember prior ID: upstream start is idempotent, never claim an existing take.
            after=json.loads(request('/api/record/start',True)).get('current') or {}
            if not after.get('id') or after.get('id')==before.get('id'):raise ValueError('Could not establish new recording ownership')
            owned=after['id'];row.update(state='recording',record_id=owned,started=time.time());save(p,row)
        deadline=time.monotonic()+payload['duration_s']
        while time.monotonic()<deadline:
            if (STATE/(payload['job']+'.stop')).exists():break
            time.sleep(.2)
        row.update(state='stop_pending');save(p,row)
        for attempt in range(10):
            try:
                cur=current()
                if cur.get('id')!=owned:row.update(state='ownership_changed',error='Current take changed; no stop sent');break
                if cur.get('status')=='recording':cur=json.loads(request('/api/record/stop',True)).get('current') or {}
                row.update(state='completed',recording=cur,finished=time.time());break
            except Exception:
                if attempt==9:raise RuntimeError('Could not confirm stop; inspect recording state')
                time.sleep(1)
    except Exception as exc:row.update(state='failed',error=str(exc),finished=time.time())
    save(p,row)
def folder(payload):
    kind=payload.get('kind');name=payload.get('id','')
    if kind not in ROOTS or not re.fullmatch(r'[A-Za-z0-9_-]{1,80}',name):raise ValueError('Invalid record')
    base=ROOTS[kind].resolve();p=(base/name).resolve()
    if p.parent!=base or not p.is_dir():raise ValueError('Record not found')
    return p
def files(p):
    allowed=re.compile(r'vision-\d{6}\.(jpg|png|json)|vision-status\.json|(summary|status|manifest|frames|controls|reviews|pid)\.(json|jsonl)|frames_\d+\.jsonl|video_\d+\.avi|\d{6}\.(jpg|png)')
    return [f for f in sorted(p.iterdir()) if f.is_file() and not f.is_symlink() and allowed.fullmatch(f.name)]
def catalogue():
    out=[]
    for kind,base in ROOTS.items():
        if not base.exists():continue
        for p in base.iterdir():
            if not p.is_dir() or p.is_symlink() or not re.fullmatch(r'[A-Za-z0-9_-]{1,80}',p.name):continue
            metadata={}
            for name in ('summary.json','status.json','manifest.json'):
                if (p/name).exists():
                    try:metadata[name]=json.loads((p/name).read_text())
                    except ValueError:pass
            fs=files(p)
            if fs:out.append(dict(kind=kind,id=p.name,bytes=sum(f.stat().st_size for f in fs),files=len(fs),metadata=metadata,modified=p.stat().st_mtime))
    return sorted(out,key=lambda r:r['modified'],reverse=True)
def audit():
    credential=base64.b64encode(('jetson:'+(HOME/'.config/orin_lab/token').read_text().strip()).encode()).decode()
    out=[]
    for port,paths in [(8000,['/','/manual','/api/status','/api/manual/status','/api/record/status','/camera.jpg']),(8091,['/','/api/runs','/api/record','/preview.jpg']),(8092,['/','/api/status'])]:
        base=None
        for host in ['127.0.0.1']+subprocess.check_output(['hostname','-I'],text=True).split():
            if ':' in host:continue
            url=f'http://{host}:{port}'
            try:
                with urllib.request.urlopen(urllib.request.Request(url+'/',headers={'Authorization':'Basic '+credential}),timeout=2):base=url;break
            except OSError:continue
        if not base:out.append(dict(port=port,state='unreachable',note='Read-only probe; service not started or restarted'));continue
        try:
            with urllib.request.urlopen(base+'/',timeout=3):protected=False
        except urllib.error.HTTPError as e:protected=e.code==401
        for path in paths:
            try:
                with urllib.request.urlopen(urllib.request.Request(base+path,headers={'Authorization':'Basic '+credential}),timeout=4) as response:
                    out.append(dict(port=port,path=path,status=response.status,bytes=len(response.read()),login_protected=protected))
            except OSError:out.append(dict(port=port,path=path,state='failed',login_protected=protected))
    return dict(at=time.time(),checks=out,services=services())
def archive(payload):
    p=folder(payload)
    if payload['kind']=='record' and current().get('id')==p.name and current().get('status')=='recording':raise ValueError('Cannot sync active recording')
    if (p/'summary.json').exists() and json.loads((p/'summary.json').read_text()).get('status')=='recording':raise ValueError('Recording not finalized')
    fs=files(p)
    # Freeze per-file size/hash and verify again after writing; caller verifies all content.
    manifest=[dict(name=f.name,bytes=f.stat().st_size,sha256=hashlib.sha256(f.read_bytes()).hexdigest()) for f in fs]
    with tarfile.open(fileobj=sys.stdout.buffer,mode='w|') as tar:
        data=json.dumps(manifest).encode();info=tarfile.TarInfo('sync_manifest.json');info.size=len(data);tar.addfile(info,io.BytesIO(data))
        for f in fs:tar.add(f,arcname=f.name,recursive=False)
def main():
    op=sys.argv[1];payload=json.loads(base64.b64decode(sys.argv[2]))
    if op=='worker':worker(payload);return
    if op=='status':out=status()
    elif op=='audit':out=audit()
    elif op=='camera':sys.stdout.buffer.write(request('/camera.jpg'));return
    elif op=='catalogue':out=catalogue()
    elif op=='archive':archive(payload);return
    elif op=='start':out=start(payload)
    elif op=='stop':
        row=json.loads(jid(payload.get('job')).read_text())
        if row['state'] not in ('queued','recording','stop_pending'):raise ValueError('No active owned job')
        (STATE/(payload['job']+'.stop')).touch();out={'requested':True,'job':payload['job']}
    else:raise ValueError('Operation not allowed')
    print(json.dumps(out))
if __name__=='__main__':
    try:main()
    except Exception as exc:print(json.dumps({'error':str(exc)}));sys.exit(1)
