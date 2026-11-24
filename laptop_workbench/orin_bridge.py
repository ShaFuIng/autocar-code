"""SSH transport with fixed operations and no vehicle command proxy."""
import base64,hashlib,json,os,re,subprocess,tarfile,threading,time,uuid
from pathlib import Path
ROOT=Path(__file__).resolve().parent
OPS={'status','camera','catalogue','archive','start','stop','audit'}
class Bridge:
    def __init__(self,root):
        self.root=Path(root)/'orin';self.root.mkdir(parents=True,exist_ok=True)
        self.lock=threading.Lock();self.sync_lock=threading.Lock();self.host=None;self.installed=set();self.cached=None;self.cached_at=0;self.pinned_host=None
        connection=self.root/'connection.json'
        if connection.exists():
            saved=json.loads(connection.read_text(encoding='utf-8'));host=saved.get('host','')
            if re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9.:-]*',host):
                self.host=host
                if saved.get('pin_host') is True:self.pinned_host=host
        self.progress={'state':'idle'};self.catalogue=[]
        previous=self.root/'sync_status.json'
        if previous.exists():
            self.progress=json.loads(previous.read_text(encoding='utf-8'))
            if self.progress['state']=='running':self.progress['state']='interrupted'
        p=self.root/'catalogue.json'
        if p.exists():self.catalogue=json.loads(p.read_text(encoding='utf-8'))
    def ssh(self,host):
        return ['ssh','-i',str(Path.home()/'.ssh/codex_orin_ed25519'),'-o','BatchMode=yes','-o','ConnectTimeout=5','-o','StrictHostKeyChecking=yes','-o','HostKeyAlias=orinnano-super.local','jetson@'+host]
    def call(self,op,payload=None,destination=None):
        if op not in OPS:raise ValueError('Operation not allowed')
        source=(ROOT/'orin_remote.py').read_bytes();digest=hashlib.sha256(source).hexdigest()[:16];remote='/home/jetson/.local/share/laptop_workbench/adapter-'+digest+'.py'
        encoded=base64.b64encode(json.dumps(payload or {}).encode()).decode()
        hosts=[self.pinned_host] if self.pinned_host else list(dict.fromkeys([self.host,'orinnano-super.local','192.168.50.1','10.149.72.240','172.17.37.68']))
        last=None
        for host in filter(None,hosts):
            try:
                if (host,digest) not in self.installed:
                    installer="import sys,pathlib;p=pathlib.Path('"+remote+"');p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(sys.stdin.buffer.read())"
                    r=subprocess.run(self.ssh(host)+['python3 -c "'+installer+'"'],input=source,capture_output=True,timeout=15)
                    if r.returncode:raise OSError('SSH installation unavailable')
                    self.installed.add((host,digest))
                cmd=self.ssh(host)+['python3 '+remote+' '+op+' '+encoded]
                if destination:
                    with Path(destination).open('wb') as f:r=subprocess.run(cmd,stdout=f,stderr=subprocess.PIPE,timeout=300)
                    if r.returncode:raise ValueError('Remote archive unavailable or still recording')
                    data=None
                else:
                    r=subprocess.run(cmd,capture_output=True,timeout=35)
                    if r.returncode:
                        try:message=json.loads(r.stdout).get('error','Remote operation rejected')
                        except ValueError:raise OSError('SSH unavailable')
                        raise ValueError(message)
                    data=r.stdout
                self.host=host
                return data if op=='camera' else (json.loads(data) if data else None)
            except (OSError,subprocess.TimeoutExpired) as exc:
                last=exc
                # Mutation ACK may have been lost: never retry start/stop blindly on another host.
                if op in ('start','stop'):raise ValueError('Orin acknowledgment unknown; refresh status before another action')
        raise ValueError('Orin SSH unavailable (known host/address retries exhausted)') from last
    def status(self):
        with self.lock:
            if time.time()-self.cached_at<3 and self.cached:return self.cached
            try:
                data=self.call('status');self.cached=dict(connected=True,host=self.host,**data);self.cached_at=time.time()
                self.write('last_status.json',self.cached)
                return self.cached
            except ValueError as e:return dict(connected=False,error=str(e),last_known=self.cached,at=time.time())
    def write(self,name,data):
        p=self.root/name;tmp=p.with_suffix('.tmp');tmp.write_text(json.dumps(data,ensure_ascii=False),encoding='utf-8');os.replace(tmp,p)
    def records(self,refresh=False):
        if refresh:self.catalogue=self.call('catalogue');self.write('catalogue.json',self.catalogue)
        return dict(records=self.catalogue,sync=self.progress,local=self.local_records())
    def local_records(self):
        out=[]
        for p in (self.root/'records').glob('*/*/current.json'):
            item=json.loads(p.read_text());out.append(item)
        return out
    def start_sync(self,payload):
        if not self.sync_lock.acquire(blocking=False):raise ValueError('Sync already running')
        self.progress=dict(state='running',started=time.time(),completed=0,errors=[])
        def work():
            try:
                records=self.call('catalogue');self.catalogue=records;self.write('catalogue.json',records)
                selected=records if payload.get('all') else [r for r in records if r['id']==payload.get('id') and r['kind']==payload.get('kind')]
                if payload.get('missing'):
                    local={(r['kind'],r['id']) for r in self.local_records()}
                    selected=[r for r in records if (r['kind'],r['id']) not in local]
                if not selected:raise ValueError('No matching records')
                self.progress['total']=len(selected)
                for record in selected:
                    self.progress['current']=record['kind']+'/'+record['id']
                    try:self.sync_one(record);self.progress['completed']+=1
                    except Exception as e:self.progress['errors'].append(dict(id=record['id'],error=str(e)))
                self.progress['state']='completed_with_errors' if self.progress['errors'] else 'completed'
            except Exception as e:self.progress.update(state='failed',error=str(e))
            finally:self.progress['finished']=time.time();self.write('sync_status.json',self.progress);self.sync_lock.release()
        threading.Thread(target=work,daemon=True).start();return self.progress.copy()
    def sync_one(self,record):
        kind,name=record['kind'],record['id']
        if kind not in ('record','shadow','manual') or not re.fullmatch('[A-Za-z0-9_-]{1,80}',name):raise ValueError('Invalid record')
        base=self.root/'records'/kind/name;base.mkdir(parents=True,exist_ok=True)
        temp=base/(uuid.uuid4().hex+'.tar');stage=base/('staging-'+uuid.uuid4().hex);stage.mkdir()
        try:
            self.call('archive',dict(kind=kind,id=name),temp)
            with tarfile.open(temp) as tar:
                members=tar.getmembers()
                if any(not m.isfile() or Path(m.name).name!=m.name or '\\' in m.name or m.size>2*1024**3 for m in members):raise ValueError('Unsafe archive entry')
                manifest=json.load(tar.extractfile('sync_manifest.json'))
                expected={r['name']:r for r in manifest}
                if {m.name for m in members}!=set(expected)|{'sync_manifest.json'}:raise ValueError('Manifest mismatch')
                for member in members:
                    target=stage/member.name;h=hashlib.sha256();size=0
                    with tar.extractfile(member) as src,target.open('wb') as dst:
                        for b in iter(lambda:src.read(1048576),b''):dst.write(b);h.update(b);size+=len(b)
                    if member.name in expected and (size!=expected[member.name]['bytes'] or h.hexdigest()!=expected[member.name]['sha256']):raise ValueError('Hash mismatch; source changed during synchronization')
            version=hashlib.sha256((stage/'sync_manifest.json').read_bytes()).hexdigest()[:20];dest=base/version
            if not dest.exists():
                # OneDrive/AV may briefly hold a new folder after file writes.
                for attempt in range(12):
                    try:stage.rename(dest);break
                    except PermissionError:
                        if attempt==11:raise
                        time.sleep(.5)
            else:
                for f in stage.iterdir():f.unlink()
                stage.rmdir()
            if list(dest.glob('video_*.avi')) and not (dest/'preview/frames.json').exists():
                py=Path.home()/'AppData/Local/Programs/Python/Python310/python.exe'
                r=subprocess.run([str(py),str(ROOT/'prepare_orin_replay.py'),str(dest)],capture_output=True,timeout=300)
                if r.returncode:raise ValueError('Originals verified but replay preparation failed')
            item=dict(kind=kind,id=name,version=version,files=len(manifest),bytes=sum(x['bytes'] for x in manifest),synced_at=time.time(),verified=True)
            self.write_relative(base/'current.json',item)
        finally:
            if temp.exists():temp.unlink()
    def write_relative(self,p,data):
        tmp=p.with_suffix('.tmp');tmp.write_text(json.dumps(data),encoding='utf-8');os.replace(tmp,p)
    def local_folder(self,kind,name):
        if kind not in ('record','shadow','manual') or not re.fullmatch('[A-Za-z0-9_-]{1,80}',name):raise ValueError('Invalid record')
        base=self.root/'records'/kind/name;item=json.loads((base/'current.json').read_text());return base/item['version']
