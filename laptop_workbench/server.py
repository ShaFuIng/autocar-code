"""Laptop-only workbench. Standard library; no vehicle/network/serial adapters."""
import argparse
import base64
import hashlib
import json
import math
import mimetypes
import re
import sqlite3
import socket
import threading
import time
import uuid
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse
from orin_bridge import Bridge
from analysis_jobs import Analysis
from integration import Integration
from ops07 import Operations
from field_trials import Trials
import visual_pid
# Optional two-point preview parameters (car-side visual_pid, not deployed yet);
# the laptop visual_pid.py copy does not define them, so keep the defaults here.
PREVIEW_DEFAULTS = dict(preview_far_weight=.35, preview_far_row=.45)
from pid_workspace import Workspace
from camera_measure import CameraMeasure
from straight_test import StraightTest
import junction_cases

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent
BUILD = 'workbench-ops07-deployment-20261109.610'


def encode(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False)


def number(value, low, high):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'數值須介於 {low}～{high}')
    return value


class Conflict(Exception):
    pass


class Store:
    def __init__(self, root):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(self.root / 'workbench.sqlite3', check_same_thread=False)
        self.lock = threading.RLock()
        self.db.executescript('''
        PRAGMA journal_mode=WAL;
        CREATE TABLE IF NOT EXISTS documents(key TEXT PRIMARY KEY, rev INTEGER, body TEXT);
        CREATE TABLE IF NOT EXISTS history(id INTEGER PRIMARY KEY, key TEXT, rev INTEGER, body TEXT, created REAL);
        CREATE TABLE IF NOT EXISTS tasks(id TEXT PRIMARY KEY, body TEXT);
        CREATE TABLE IF NOT EXISTS events(id INTEGER PRIMARY KEY, task TEXT, body TEXT);
        CREATE TABLE IF NOT EXISTS attachments(id TEXT PRIMARY KEY, body TEXT);
        ''')
        self.db.commit()

    def get(self, key):
        with self.lock:
            row = self.db.execute('SELECT rev,body FROM documents WHERE key=?', (key,)).fetchone()
            return {'key': key, 'rev': row[0], 'data': json.loads(row[1])} if row else {'key': key, 'rev': 0, 'data': {}}

    def save(self, key, rev, data):
        if not re.fullmatch(r'[a-zA-Z0-9_.-]{1,100}', key) or not isinstance(data, dict):
            raise ValueError('無效文件')
        with self.lock:
            current = self.get(key)
            if current['rev'] != rev:
                raise Conflict('另一頁已更新此資料；本機草稿保留。請讀取保存值後比較，不能直接覆蓋。')
            body = encode(data)
            self.db.execute('INSERT OR REPLACE INTO documents VALUES(?,?,?)', (key, rev + 1, body))
            self.db.execute('INSERT INTO history(key,rev,body,created) VALUES(?,?,?,?)', (key, rev + 1, body, time.time()))
            self.db.commit()
            return self.get(key)

    def tasks(self):
        with self.lock:
            return [json.loads(r[0]) for r in self.db.execute('SELECT body FROM tasks ORDER BY rowid DESC')]

    def task(self, task):
        with self.lock:
            self.db.execute('INSERT OR REPLACE INTO tasks VALUES(?,?)', (task['id'], encode(task)))
            self.db.commit()

    def event(self, task, data):
        with self.lock:
            self.db.execute('INSERT INTO events(task,body) VALUES(?,?)', (task, encode(data)))
            self.db.commit()

    def events(self, task):
        with self.lock:
            return [json.loads(r[0]) for r in self.db.execute('SELECT body FROM events WHERE task=? ORDER BY id', (task,))]


class Simulator:
    """Server-owned monotonic clock; intentionally cannot connect to real hardware."""
    def __init__(self, store, clock=time.monotonic):
        self.store, self.clock = store, clock
        self.lock = threading.RLock()
        self.active = None
        self.authorized = False
        self.fault = None
        self.applied = {'pwm_limit': 25, 'config_rev': 0}
        self.last_reason = 'SIMULATOR_BOOT'
        self.boot = uuid.uuid4().hex
        for task in store.tasks():
            if task['status'] == 'running':
                task.update(status='stopped', reason='EXECUTOR_RESTART', ended=time.time())
                store.task(task)
                store.event(task['id'], {'kind': 'stop', 'reason': 'EXECUTOR_RESTART', 'time': time.time()})

    def status(self):
        with self.lock:
            return dict(build=BUILD, backend='simulation', vehicle_connected=False, boot=self.boot,
                        authorized=self.authorized, fault=self.fault, applied=self.applied,
                        active=self.active, last_reason=self.last_reason,
                        elapsed_s=round(self.clock()-self.started, 3) if self.active else 0)

    def configure(self, p):
        with self.lock:
            if self.active:
                raise ValueError('請先停止任務再切換授權或套用上限')
            if p['action'] == 'authorize':
                if self.fault:
                    raise ValueError('請先解除模擬故障；解除不會自動開始')
                self.authorized = p.get('enabled') is True
            elif p['action'] == 'apply':
                limit = number(p.get('pwm_limit'), 0, 100)
                self.applied = {'pwm_limit': limit, 'config_rev': p.get('config_rev', 0)}
            else:
                raise ValueError('未知設定')
            return self.status()

    def start(self, p):
        with self.lock:
            uid, owner = p.get('id'), p.get('owner')
            if not isinstance(uid, str) or not re.fullmatch(r'[a-zA-Z0-9-]{8,80}', uid):
                raise ValueError('需要穩定任務ID')
            if not isinstance(owner, str) or not re.fullmatch(r'[a-zA-Z0-9-]{8,80}', owner):
                raise ValueError('需要控制者ID')
            existing = next((t for t in self.store.tasks() if t['id'] == uid), None)
            if existing:
                if existing['request'] != p:
                    raise Conflict('相同任務ID的參數不同，拒絕重用')
                return existing
            duration = number(p.get('duration_s'), .1, 120)
            pwm = number(p.get('pwm'), 0, self.applied['pwm_limit'])
            mode = p.get('mode')
            if mode not in ('fixed', 'vision', 'remote', 'record'):
                raise ValueError('無效模式')
            if mode == 'record' and pwm != 0:
                raise ValueError('純錄影必須零模擬動力')
            if self.active:
                raise Conflict('已有控制者／任務執行中；可停止後明確開始新任務')
            if self.fault or (not self.authorized and mode != 'record'):
                raise ValueError('需要模擬網頁授權且無故障')
            self.started = self.last_input = self.clock()
            self.remote_received = False
            self.last_sample = -1
            task = dict(id=uid, owner=owner, request=p, config_snapshot=dict(self.applied),
                        status='running', created=time.time(), duration_s=duration, mode=mode,
                        reason=None, source='simulation', boot=self.boot)
            self.store.task(task)
            self.active = task
            self.store.event(uid, {'kind': 'accepted', 't': 0, 'pwm': 0, 'source': 'simulation'})
            return task

    def stop(self, reason='USER_STOP'):
        with self.lock:
            if self.active:
                t = self.active
                elapsed = self.clock()-self.started
                t.update(status='stopped', ended=time.time(), elapsed_s=elapsed, reason=reason)
                self.store.event(t['id'], {'kind': 'stop', 't': elapsed, 'pwm': 0, 'reason': reason, 'source': 'simulation'})
                self.store.task(t)
                self.active = None
            self.last_reason = reason
            return self.status()

    def tick(self):
        with self.lock:
            t = self.active
            if not t:
                return
            elapsed = self.clock()-self.started
            if elapsed >= t['duration_s']:
                self.stop('DURATION_COMPLETE')
                return
            if t['mode'] == 'remote' and self.clock()-self.last_input > .75:
                self.stop('REMOTE_INPUT_TIMEOUT')
                return
            if elapsed - self.last_sample >= .1 - 1e-9:
                self.last_sample = elapsed
                self.store.event(t['id'], {'kind': 'sample', 't': round(elapsed, 4),
                    'pwm': (t['request']['pwm'] if t['mode']!='remote' or self.remote_received else 0), 'command_us': 1360,
                    'encoder_a': round(elapsed*70), 'encoder_b': round(-elapsed*70),
                    'frame_id': round(elapsed*10), 'path': 'SIMULATED' if t['mode']=='vision' else None,
                    'source': 'simulation'})

    def action(self, p):
        with self.lock:
            action = p.get('action')
            if action == 'input':
                if not self.active or self.active['id'] != p.get('id') or self.active['owner'] != p.get('owner'):
                    raise ValueError('不是本次任務控制者')
                self.last_input = self.clock()
                self.remote_received = True
            elif action == 'stop':
                return self.stop()
            elif action in ('estop', 'fault', 'camera', 'path'):
                if action in ('camera', 'path'):
                    if self.active and self.active['mode'] == 'vision':
                        self.stop('VISION_' + action.upper() + '_LOST')
                else:
                    self.fault = action
                    self.stop('ESTOP' if action == 'estop' else 'CONTROL_FAULT')
            elif action == 'clear':
                self.fault = None
            elif action == 'restart':
                self.stop('EXECUTOR_RESTART')
                self.authorized = False
                self.fault = None
                self.applied = {'pwm_limit': 25, 'config_rev': 0}
                self.boot = uuid.uuid4().hex
            else:
                raise ValueError('未知操作')
            return self.status()


class App:
    def __init__(self, data):
        self.store = Store(data)
        self.sim = Simulator(self.store)
        self.orin = Bridge(data)
        self.assets = {}
        self.replays = {}
        self.index_lock = threading.RLock()
        self.inventory = self.index()
        self.analysis = Analysis(self)
        self.integration = Integration(self)
        self.ops07 = Operations(self)
        self.camera_measure = CameraMeasure(self)
        self.straight = StraightTest(self)
        self.trials = Trials(self.store,
            read_status=lambda:self.integration.request('/api/status'),
            read_camera=lambda:base64.b64decode(self.integration.raw('/camera.jpg')['body']))

    def index(self):
        paths = [PROJECT/'Config.h', PROJECT/'Orin_Interface_v1.ino', PROJECT/'ManualDrive.cpp',
                 PROJECT/'Telemetry.cpp', PROJECT/'web/server.py', PROJECT/'web/manual_drive.py',
                 PROJECT/'field_test/engine.py', PROJECT/'shadow_workbench/geometry.json',
                 PROJECT/'shadow_workbench/route_unified.py']
        for folder in ('field_test/review_latest', 'shadow_workbench/runs'):
            base = PROJECT/folder
            if base.exists():
                paths.extend(p for p in base.rglob('*') if p.is_file() and p.suffix in ('.json', '.jsonl'))
        records = []
        for p in sorted(set(paths)):
            if not p.exists():
                continue
            records.append({'path': str(p.relative_to(PROJECT)).replace('\\', '/'), 'bytes': p.stat().st_size,
                            'sha256': hashlib.sha256(p.read_bytes()).hexdigest(), 'source': '本機唯讀盤點'})
        # Copy only historical annotations, never credentials or operational configuration.
        backup = self.store.root/'annotation_backups'
        for p in (PROJECT/'shadow_workbench').rglob('reviews.jsonl'):
            digest = hashlib.sha256(p.read_bytes()).hexdigest()
            backup.mkdir(exist_ok=True)
            target = backup/(digest+'.jsonl')
            if not target.exists():
                target.write_bytes(p.read_bytes())
        for p in (PROJECT/'shadow_workbench/runs').glob('*/frames.json'):
            uid = 'shadow-'+p.parent.name
            rows = json.loads(p.read_text(encoding='utf8'))
            if not rows:
                continue
            self.replays[uid] = {'id': uid, 'title': '既有影子資料 · '+p.parent.name,
                                'kind': 'shadow', 'rows': rows, 'folder': p.parent}
            for f in p.parent.iterdir():
                if f.suffix in ('.jpg', '.png'):
                    self.assets[uid+'/'+f.name] = f
        for p in (PROJECT/'field_test/review_latest').glob('*/frames_*.jsonl'):
            rows = [json.loads(x) for x in p.read_text().splitlines() if x]
            if rows:
                origin = datetime.fromisoformat(rows[0]['time']).timestamp()
                for row in rows:
                    row['capture_s'] = datetime.fromisoformat(row['time']).timestamp()-origin
                uid = 'field-'+p.parent.name
                controls = p.parent/'controls.jsonl'
                events = [json.loads(x) for x in controls.read_text().splitlines() if x] if controls.exists() else []
                for ev in events:
                    ev['t'] = datetime.fromisoformat(ev['time']).timestamp()-origin
                self.replays[uid] = {'id': uid, 'title': '既有直線紀錄 · '+p.parent.name[:8], 'kind': 'field',
                    'rows': rows, 'events': events, 'folder': p.parent}
                cache = self.store.root/'field_frames'/p.parent.name
                files = list(cache.glob('*.jpg'))
                if len(files) == len(rows):
                    self.replays[uid]['has_images'] = True
                    for f in files:
                        self.assets[uid+'/'+f.name] = f
        manifest = {'build': BUILD, 'created': time.time(), 'items': records,
                    'deployed_firmware': '交接記載 orin-v1.4.1-manual；本次未連實機確認',
                    'model': 'v4，交接SHA256 8b802fa03d34fa73049b92ac5c3a6df121f94d3e9f7875103ceaf50de3424e54'}
        (self.store.root/'inventory.json').write_text(encode(manifest), encoding='utf8')
        return manifest

    def replay(self, uid):
        self.index_synced()
        self.ensure_replay(uid)
        if uid in self.replays:
            r = self.replays[uid]
            out = {k:v for k,v in r.items() if k != 'folder'}
            reviews = r['folder']/'reviews.jsonl'
            out['legacy_reviews'] = [json.loads(x) for x in reviews.read_text(encoding='utf8').splitlines() if x] if reviews.exists() else []
            return out
        task = next((t for t in self.store.tasks() if t['id']==uid), None)
        if not task:
            raise ValueError('找不到紀錄')
        events = self.store.events(uid)
        return dict(id=uid, title=uid, kind='simulation', task=task, events=events,
                    rows=[dict(e, capture_s=e.get('t', 0)) for e in events if e['kind']=='sample'])

    def get(self, path):
        if path == '/api/ops07/lab-status':return self.ops07.lab.snapshot()
        if path == '/api/ops07/catalog':return self.ops07.catalog()
        if path == '/api/ops07/status':return self.ops07.status()
        if path == '/api/junction/cases':return junction_cases.catalog(self)
        if path.startswith('/api/junction/reviews/'):return junction_cases.reviews(self,path.split('/')[-1])
        if path == '/api/pid/connection-log':
            f=self.store.root/'pid-connection.jsonl'
            if not f.exists():return []
            with f.open('rb') as stream:
                stream.seek(0,2);size=stream.tell();stream.seek(max(0,size-256000))
                if size>256000:stream.readline()
                rows=[]
                for line in stream:
                    try:rows.append(json.loads(line))
                    except ValueError:pass
            return rows[-200:][::-1]
        if path == '/api/pid/backlash-previews':
            with self.store.lock:
                return [dict(id=r[0],**json.loads(r[1])) for r in self.store.db.execute("SELECT key,body FROM documents WHERE key LIKE 'pid-backlash-preview-%' ORDER BY rowid DESC")]
        if path == '/api/pid/workspace/light':
            workspace=Workspace(self)
            return dict(runs=[],runs_loaded=False,reviews=workspace.reviews(),geometry=workspace.geometry())
        if path == '/api/pid/workspace':
            workspace=Workspace(self)
            return dict(runs=workspace.records(),reviews=workspace.reviews(),geometry=workspace.geometry())
        if path == '/api/pid/live/status':return self.integration.request('/api/pid/status')
        if path == '/api/pid/runs':
            with self.store.lock:
                rows=[json.loads(r[0]) for r in self.store.db.execute("SELECT body FROM documents WHERE key LIKE 'pid-run-%' ORDER BY rowid DESC")]
            return [{k:v for k,v in r.items() if k!='rows'} for r in rows]
        if path == '/api/straight/runs':return self.straight.runs()
        if path == '/api/straight/presets/history':return self.straight.history()
        if path.startswith('/api/straight/run/'):return self.straight.detail(path.split('/')[-1])
        if path == '/api/pid/catalog':
            return dict(defaults=visual_pid.DEFAULTS, jobs=[j for j in self.analysis.jobs() if j['state']=='completed'], hardware_output=False)
        if path == '/api/field-trials':return self.trials.list()
        if path.startswith('/api/field-trial/'):return self.trials.detail(path.split('/')[-1])
        if path == "/api/integration/runs":return self.integration.request("/shadow/api/runs")
        if path == "/api/integration/status":return self.integration.request("/api/manual/status")
        if path == "/api/analysis":return dict(sources=self.analysis.sources(),models=self.analysis.models(),jobs=self.analysis.jobs())
        if path.startswith("/api/analysis/result/"):return self.analysis.result(path.split("/")[-1])
        if path.startswith("/api/analysis/reviews/"):return self.analysis.reviews(path.split("/")[-1])
        if path == '/api/orin/status':
            return self.orin.status()
        if path == '/api/orin/records':
            return self.orin.records()
        if path == '/api/orin/acceptance':
            file=ROOT/'ORIN_INTEGRATION_ACCEPTANCE.json'
            if not file.exists():return {'state':'pending'}
            data=json.loads(file.read_text(encoding='utf-8'))
            return dict(state='completed',completed_at=data['completed_at'],tests=data['tests'],remote_record_count=data['remote_record_count'],replay_records=data['replay_records'],sync=data['sync'],replay_breakdown=data.get('replay_breakdown'),historical_data_warnings=data.get('historical_data_warnings',[]))
        if path == '/api/orin/audit':
            file=self.orin.root/'route_audit.json'
            return json.loads(file.read_text(encoding='utf-8')) if file.exists() else {'checks':[]}
        if path == '/api/status':
            return self.sim.status()
        if path == '/api/inventory':
            return self.inventory
        if path == '/api/tasks':
            return self.store.tasks()
        if path == '/api/records':
            self.index_synced()
            return [dict(id=r['id'], title=r['title'], kind=r['kind']) for r in self.replays.values()] + [dict(id=t['id'],title='模擬 · '+t['mode']+' · '+t['id'][:8],kind='simulation') for t in self.store.tasks()]
        if path.startswith('/api/replay/'):
            return self.replay(path.split('/')[-1])
        if path.startswith('/api/document/'):
            return self.store.get(path.split('/')[-1])
        if path.startswith('/api/history/'):
            with self.store.lock:
                return [dict(rev=r[0], data=json.loads(r[1]), created=r[2]) for r in self.store.db.execute('SELECT rev,body,created FROM history WHERE key=? ORDER BY rev DESC', (path.split('/')[-1],))]
        if path == '/api/attachments':
            with self.store.lock:
                return [json.loads(r[0]) for r in self.store.db.execute('SELECT body FROM attachments')]
        raise ValueError('未知API')

    def post(self, path, p):
        if path.startswith('/api/ops07/'):return self.ops07.post(path.rsplit('/',1)[-1],p)
        if path == '/api/junction/review':return junction_cases.save(self,p)
        if path == '/api/camera-measure/capture':return self.camera_measure.start()
        if path == '/api/pid/connection-event':
            if p.get('event') not in ('blur','hidden','pagehide','route_change','heartbeat_ack','heartbeat_error','recover','recovered','recovery_failed','start_ack','user_stop'):
                raise ValueError('Unknown connection event')
            row=dict(kind='browser',event=p['event'])
            for key in ('run_id','detail'):
                if key in p:
                    if not isinstance(p[key],str) or len(p[key])>300:raise ValueError('Invalid connection text')
                    row[key]=p[key]
            for key in ('client_time','seq','rtt_ms','send_gap_ms'):
                if key in p:
                    if type(p[key]) not in (int,float) or not math.isfinite(p[key]) or p[key]<0:raise ValueError('Invalid connection timing')
                    row[key]=p[key]
            self.integration.diagnostic(row)
            return {'ok':True}
        if path == '/api/pid/heartbeat-probe':
            started=time.monotonic()
            self.integration.request('/api/pid/status',channel='heartbeat')
            warmup_ms=round((time.monotonic()-started)*1000,1)
            started=time.monotonic()
            s=self.integration.request('/api/pid/status',channel='heartbeat')
            rtt_ms=round((time.monotonic()-started)*1000,1)
            return dict(ok=True,ready=rtt_ms<500,rtt_ms=rtt_ms,warmup_ms=warmup_ms,state=s.get('state'),run_state=(s.get('run') or {}).get('state'))
        if path == '/api/pid/backlash-preview':return Workspace(self).preview(p)
        if path == '/api/pid/field-review':return Workspace(self).add_review(p)
        if path.startswith('/api/pid/live/'):
            action=path.rsplit('/',1)[1]
            if action not in ('preview-config','prepare','start','heartbeat','stop','preview-stop'):raise ValueError('Unknown PID action')
            if action=='prepare':
                amount=self.store.get('visual-pid')['data']['parameters'].get('curve_outward_px_640',0.)
                p=dict(p,curve_outward_px_640=amount)
                if amount:
                    remote=self.integration.request('/api/pid/status')
                    if remote.get('curve_outward_version',0)<1:raise ValueError('彎道外移尚未部署Orin；保存值已保留，請完成部署後載入')
                    if p.get('path_algorithm') not in ('junction-k-lane-v3','junction-k-integrated-r2'):raise ValueError('彎道外移目前支援K路徑；其他算法請設0')
            if action=='start':
                doc=self.store.get('visual-pid')
                if not doc['rev'] or doc['rev']!=p.get('parameter_revision'):raise ValueError('Reload saved PID version')
                geometry_snapshot=Workspace(self).geometry()
                parameters=doc['data']['parameters']
                remote_status=self.integration.request('/api/pid/status')
                if parameters.get('visual_center_offset_px_640',0) and remote_status.get('visual_center_version',0)<1:raise ValueError('Orin尚未部署視覺中心偏移；不可把未套用修正當作新版本試跑')
                if parameters.get('curve_outward_px_640',0) and remote_status.get('curve_outward_version',0)<1:raise ValueError('彎道外移尚未部署Orin')
                if remote_status.get('curve_outward_version',0)<1:parameters={k:v for k,v in parameters.items() if k!='curve_outward_px_640'}
                if remote_status.get('preview_parameter_version',0)<1:
                    if any(k in parameters and parameters[k]!=v for k,v in PREVIEW_DEFAULTS.items()):raise ValueError('Orin尚未部署遠點比重／遠點列參數；改回0.35／0.45或先部署')
                    parameters={k:v for k,v in parameters.items() if k not in PREVIEW_DEFAULTS}
                mode=p.get('execution_mode','browser_lease')
                if mode not in ('browser_lease','orin_timed'):raise ValueError('Unknown PID execution mode')
                if mode=='orin_timed' and remote_status.get('autonomous_pid_version',0)<1:raise ValueError('Orin 尚未更新限時自主PID，請先完成車控更新；不會退回網頁心跳模式')
                if parameters.get('direction_enabled',0) and remote_status.get('direction_version',0)<1:raise ValueError('Orin 尚未支援近遠預判')
                capability=remote_status.get('backlash_version',0)
                if parameters.get('backlash_enabled',0) and capability<1:raise ValueError('Orin 尚未載入回差補償版本；不可啟動，需先完成車控更新')
                if parameters.get('backlash_initial_enabled',0) and capability<2:raise ValueError('Orin 尚未載入首次空行程補償；需完成車控更新後才能啟用')
                if capability<2:parameters={k:v for k,v in parameters.items() if not k.startswith('backlash_initial_')}
                if capability<1:parameters={k:v for k,v in parameters.items() if k not in visual_pid.BACKLASH_DEFAULTS}
                p=dict(p,parameters=parameters)
            result=self.integration.request('/api/pid/'+action,p)
            if action=='start' and result.get('run_id'):
                try:
                    self.store.save('pid-session-'+result['run_id'],0,dict(created=time.time(),execution_mode=mode,parameter_revision=doc['rev'],parameters=doc['data']['parameters'],geometry=geometry_snapshot,source='本輪開始時凍結；幾何未用於控制'))
                except Exception:
                    self.integration.request('/api/pid/stop',{})
                    raise
            return result
        if path == '/api/pid/replay':
            document=self.store.get('visual-pid')
            if not document['rev'] or document['rev']!=p.get('revision'):
                raise ValueError('請先保存PID參數，並使用目前版本')
            result=visual_pid.replay(self.analysis.result(p['job']),p['model'],p['layer'],document['data']['parameters'])
            result.update(parameter_revision=document['rev'],created=time.time(),id=uuid.uuid4().hex)
            self.store.save('pid-run-'+result['id'],0,result)
            return result
        if path == '/api/field-trial/start':return self.trials.start(p)
        if path == '/api/field-trial/stop':return self.trials.stop(p)
        if path == '/api/field-trial/review':return self.trials.review(p)
        if path == "/api/integration/analyze":return self.integration.request("/shadow/api/analyze",p)
        if path == "/api/integration/review":return self.integration.request("/shadow/api/review",p)
        if path == "/api/integration/reviews/sync":return self.integration.sync_reviews()
        if path == "/api/integration/stop":return self.integration.stop_all()
        if path == "/api/analysis/start":return self.analysis.start(p)
        if path == "/api/analysis/cancel":return self.analysis.cancel(p)
        if path == "/api/analysis/review":return self.analysis.review(p)
        if path == '/api/orin/discover':
            return self.orin.records(True)
        if path == '/api/straight/preset':return self.straight.apply_preset(p)
        if path == '/api/orin/sync':
            return self.orin.start_sync(p)
        if path == '/api/orin/record/start':
            return self.orin.call('start',{'job':p.get('job'),'duration_s':p.get('duration_s')})
        if path == '/api/orin/record/stop':
            return self.orin.call('stop',{'job':p.get('job')})
        if path == '/api/document':
            self.validate_document(p)
            result = self.store.save(p['key'], p['rev'], p['data'])
            if p['key'] == 'config':
                change = p['data'].get('changed', 'none')
                affected = {'steering': 'DGH', 'drive': 'CEFGHI', 'camera': 'J', 'none': ''}.get(change, '')
                for code in affected:
                    doc = self.store.get('test-'+code)
                    if doc['rev']:
                        doc['data']['status'] = '改裝後需重測'
                        doc['data']['invalidation'] = f"配置 v{result['rev']}：{change} 改裝"
                        self.store.save(doc['key'], doc['rev'], doc['data'])
            return result
        if path == '/api/start':
            return self.sim.start(p)
        if path == '/api/configure':
            if p.get('action') == 'apply':
                saved = self.store.get('config')
                if saved['rev'] != p.get('config_rev') or saved['data'].get('pwm_limit') != p.get('pwm_limit'):
                    raise Conflict('配置版本／上限不一致，請先保存再套用')
                with self.sim.lock:
                    self.sim.configure(p)
                    self.sim.applied['configuration'] = saved['data']
                    return self.sim.status()
            return self.sim.configure(p)
        if path == '/api/action':
            return self.sim.action(p)
        if path == '/api/attachment':
            content = base64.b64decode(p['content'], validate=True)
            if len(content) > 20*1024*1024:
                raise ValueError('附件上限20MB')
            ext = Path(p['name']).suffix.lower()
            if ext not in ('.jpg','.jpeg','.png','.mp4','.webm','.csv','.json','.txt'):
                raise ValueError('支援照片、MP4/WebM、CSV/JSON/TXT')
            uid = uuid.uuid4().hex + ext
            folder = self.store.root/'attachments'
            folder.mkdir(exist_ok=True)
            (folder/uid).write_bytes(content)
            item = dict(id=uid, name=p['name'], record=p.get('record'), offset_s=number(p.get('offset_s',0),-86400,86400),
                        sha256=hashlib.sha256(content).hexdigest(), bytes=len(content), created=time.time())
            with self.store.lock:
                self.store.db.execute('INSERT INTO attachments VALUES(?,?)', (uid, encode(item)))
                self.store.db.commit()
            return item
        raise ValueError('未知API')

    def index_synced(self):
        # Listing only reads metadata; controls/pid logs and vision evidence (1+ GB in total) load per record in ensure_replay.
        with self.index_lock:
            for item in self.orin.local_records():
                folder=self.orin.local_folder(item['kind'],item['id'])
                shadow=item['kind']=='shadow'
                frames=folder/'frames.json' if shadow else folder/'preview/frames.json'
                if not frames.exists() and not (folder/'controls.jsonl').exists():continue
                uid='orin-'+item['kind']+'-'+item['id']
                if uid in self.replays and self.replays[uid].get('version')==item['version']:continue
                has_images=frames.exists()
                self.replays[uid]=dict(id=uid,title='Orin 已同步 · '+item['kind']+' · '+item['id'][:8]+(' · 僅控制紀錄' if not has_images else ''),kind='shadow' if shadow else 'field',folder=folder,has_images=has_images,version=item['version'],source='orin',verified=True,lazy=item,frames_hint=len(json.loads(frames.read_text(encoding='utf-8'))) if has_images else 0)
                vision=sum(1 for f in folder.glob('vision-*.json') if re.fullmatch(r'vision-\d{6}\.json',f.name))
                if vision:
                    key=uid.replace('orin-manual-','orin-pidvision-')
                    self.replays[key]=dict(id=key,title='PID當時辨識原圖／遮罩 · '+uid[-32:][:8],kind='field',has_images=True,version=item['version'],source='orin',folder=folder,lazy_parent=uid,frames_hint=vision)

    def ensure_replay(self, uid):
        r=self.replays.get(uid)
        if r and r.get('lazy_parent'):r=self.replays.get(r['lazy_parent']);uid=r and r['id']
        if not r or not r.get('lazy'):return
        with self.index_lock:
            r=self.replays.get(uid)
            if r and r.get('lazy'):self._load_synced(r['lazy'])

    def _load_synced(self, item):
            folder=self.orin.local_folder(item['kind'],item['id'])
            shadow=item['kind']=='shadow'
            frames=folder/'frames.json' if shadow else folder/'preview/frames.json'
            controls=folder/'controls.jsonl'
            uid='orin-'+item['kind']+'-'+item['id']
            rows=json.loads(frames.read_text(encoding='utf-8')) if frames.exists() else []
            events=[]
            warnings=[]
            if controls.exists():
                for line_number,line in enumerate(controls.read_text(encoding='utf-8').splitlines(),1):
                    if not line:continue
                    try:events.append(json.loads(line))
                    except ValueError:warnings.append(f'原始控制紀錄第 {line_number} 行不完整或無法解析；原檔保留，回放略過此行。')
                origin=datetime.fromisoformat(rows[0]['time'] if rows else events[0]['time']).timestamp() if rows or events else 0
                for event in events:
                    if event.get('time'):event['t']=datetime.fromisoformat(event['time']).timestamp()-origin
                if not rows:rows=[dict(frame=i,capture_s=e.get('t',0),time=e.get('time')) for i,e in enumerate(events)]
            has_images=frames.exists()
            pid_events=[]
            if (folder/'pid.jsonl').exists():
                for line_number,line in enumerate((folder/'pid.jsonl').read_text(encoding='utf-8').splitlines(),1):
                    if not line:continue
                    try:pid_events.append(json.loads(line))
                    except ValueError:warnings.append(f'PID紀錄第 {line_number} 行無法解析；原檔保留。')
            if not rows:warnings.append('原始紀錄沒有影格或控制事件，僅保留摘要。')
            self.replays[uid]=dict(id=uid,title='Orin 已同步 · '+item['kind']+' · '+item['id'][:8]+(' · 僅控制紀錄' if not has_images else '')+(' · 有資料限制' if warnings else ''),kind='shadow' if shadow else 'field',rows=rows,events=events,folder=folder,has_images=has_images,version=item['version'],source='orin',verified=True,data_warnings=warnings)
            if (folder/'summary.json').exists():self.replays[uid]['record_summary']=json.loads((folder/'summary.json').read_text(encoding='utf-8'))
            self.replays[uid]['pid_events']=pid_events
            from vision_evidence import index as index_evidence
            evidence_id=index_evidence(self,folder,uid,item['version'])
            if evidence_id:self.replays[uid]['evidence_replay_id']=evidence_id
            elif self.replays.get(uid.replace('orin-manual-','orin-pidvision-'),{}).get('lazy_parent')==uid:del self.replays[uid.replace('orin-manual-','orin-pidvision-')]
            images=folder if shadow else folder/'preview'
            for f in images.iterdir() if images.exists() else []:
                if re.fullmatch(r'\d{6}\.(jpg|png)',f.name):self.assets[uid+'/'+f.name]=f

    def validate_document(self, p):
        key, data = p['key'], p['data']
        if key.startswith(('pid-field-review-', 'pid-session-')):raise ValueError('Immutable trial record')
        if key == 'visual-pid':
            visual_pid.validate(data.get('parameters'))
            if data.get('hardware_output') is not False or data.get('imu_enabled') is not False:
                raise ValueError('目前PID僅離線回算；硬體與IMU輸出未開放')
        if not isinstance(data, dict) or type(p['rev']) is not int or p['rev'] < 0:
            raise ValueError('文件內容或版本無效')
        kind = key.removeprefix('default-')
        if kind == 'config':
            if 'pwm_limit' in data:
                number(data['pwm_limit'],0,100)
            for k in ('width','wheelbase','track_width'):
                if data.get(k) is not None:
                    number(data[k],.001,10000)
            if data.get('fps',2) not in (1,2,5,10):
                raise ValueError('預覽FPS無效')
        if kind.startswith('test-'):
            values = data.get('values',{})
            for k,v in values.items():
                if v is not None:
                    number(v,-1e12,1e12)
            if data.get('status') == '已採用':
                if not data.get('reference') or not any(v is not None for v in values.values()):
                    raise ValueError('採用需要量測及參考點')
                if any(data.get('sources',{}).get(k,'未知')=='未知' for k,v in values.items() if v is not None):
                    raise ValueError('採用需要已填數值的來源')
        if kind.startswith('review-'):
            r = self.replay(kind[7:])
            start,end = data.get('start'),data.get('end')
            if type(start) is not int or type(end) is not int or not 0<=start<=end<len(r['rows']):
                raise ValueError('區間超出紀錄範圍')


def handler(app):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def send(self, body, status=200, mime='application/json; charset=utf-8'):
            if not isinstance(body, bytes):
                body = encode(body).encode('utf8')
            try:
                self.send_response(status)
                self.send_header('Content-Type', mime)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.send_header('X-Content-Type-Options', 'nosniff')
                self.end_headers()
                self.wfile.write(body)
            except (ConnectionAbortedError, ConnectionResetError, BrokenPipeError):
                self.close_connection=True

        def do_GET(self):
            try:
                path = unquote(urlparse(self.path).path)
                if path == '/api/ops07/frame':
                    payload,code=app.ops07.frame()
                    return self.send(payload,code,mime='image/jpeg')
                if path == '/ops07-track.webp':return self.send((PROJECT.parent/'跑道地圖_統一版_01-12.webp').read_bytes(),mime='image/webp')
                if path in ('/vehicle/', '/vehicle/manual'):
                    name='manual.html' if path.endswith('/manual') else 'index.html'
                    html=(PROJECT/'web'/name).read_text(encoding='utf-8')
                    for quote in ["'",'"','`']:
                        html=html.replace(quote+'/',quote+'/vehicle/')
                    html=html.replace('const labURL=`http://${location.hostname}:8888/lab`',"const labURL='/research/lab'")
                    html=html.replace('<title>','<title>新工作台 · ')
                    html=html.replace('</style>','iframe{max-height:850px}#motorHistory{font-size:14px}#motorHistory td{padding:6px}#downloads{max-height:320px;overflow:auto}</style>')
                    html=html.replace('</body>','')+"<script src='/last-inputs.js'></script><script src='/vehicle-pagination.js'></script>"
                    return self.send(html.encode(),mime='text/html; charset=utf-8')
                if path == '/vehicle-pagination.js':return self.send((ROOT/'vehicle-pagination.js').read_bytes(),mime='text/javascript')
                if path.startswith('/vehicle/'):
                    if self.headers.get('Upgrade','').lower()=='websocket' and self.headers.get('Origin')!='http://'+self.headers.get('Host',''):return self.send({'error':'Origin rejected'},403)
                    return app.integration.relay(self,self.path[len('/vehicle'):],b'')
                if path.startswith('/research/'):
                    if self.headers.get('Upgrade','').lower()=='websocket' and self.headers.get('Origin')!='http://'+self.headers.get('Host',''):return self.send({'error':'Origin rejected'},403)
                    return app.integration.relay(self,self.path,b'')
                if path == '/orin-camera.jpg':
                    # Same Orin /camera.jpg; prefer the persistent RPC channel over a new SSH per frame.
                    try:
                        d=app.integration.raw('/camera.jpg')
                        if d.get('code')==200 and str(d.get('mime','')).startswith('image/'):return self.send(base64.b64decode(d['body']),mime='image/jpeg')
                    except Exception:pass
                    return self.send(app.orin.call('camera'),mime='image/jpeg')
                if path.startswith('/orin-file/'):
                    parts=path.strip('/').split('/')
                    if len(parts)!=4:raise ValueError('Invalid file path')
                    _,kind,uid,name=parts
                    folder=app.orin.local_folder(kind,uid)
                    allowed={x['name'] for x in json.loads((folder/'sync_manifest.json').read_text())}
                    if name not in allowed:raise ValueError('File not in verified manifest')
                    file=folder/name
                    return self.send(file.read_bytes(),mime=mimetypes.guess_type(file)[0] or 'application/octet-stream')
                if path.startswith('/api/'):
                    if path == '/api/camera-measure/status':return self.send(app.camera_measure.status())
                    return self.send(app.get(path))
                file = None
                if path in ('/', '/app.js', '/workbench-tidy.js', '/style.css', '/guides.json', '/experiments.html', '/analysis.js', '/shadow-offline.js', '/junction-cases.js', '/integration.js', '/field-trials.js', '/pid.js', '/pid-console.js', '/pid-live.js', '/vision-layers.js', '/last-inputs.js', '/straight-test.js', '/pid-start-check.js', '/ops07.js', '/ops07-lab.js', '/ops07.css'):
                    file = ROOT/('index.html' if path=='/' else path[1:])
                elif re.fullmatch(r'/field-image/[0-9a-f]{32}/[0-9]{6}\.jpg',path):
                    parts=path.split('/');file=app.trials.folder(parts[2])/parts[3]
                elif path.startswith('/analysis-file/'):
                    base=(app.store.root/'analysis').resolve();candidate=(base/path[len('/analysis-file/'):]).resolve()
                    if candidate.is_relative_to(base) and candidate.suffix in ('.png','.jpg'):file=candidate
                elif path.startswith('/experiment/'):
                    base = (app.store.root/'experiments').resolve()
                    candidate = (base/path[len('/experiment/'):]).resolve()
                    if candidate.is_relative_to(base) and candidate.suffix in ('.json','.jpg','.png'):
                        file = candidate
                elif path.startswith('/asset/'):
                    file = app.assets.get(path[7:])
                    if not file:
                        app.ensure_replay(path[7:].split('/')[0]);file = app.assets.get(path[7:])
                elif path.startswith('/attachment/') and re.fullmatch(r'[0-9a-f]{32}\.[a-z0-9]+', path[12:]):
                    file = app.store.root/'attachments'/path[12:]
                if file and file.is_file():
                    return self.send(file.read_bytes(), mime=mimetypes.guess_type(file)[0] or 'application/octet-stream')
                self.send({'error':'找不到檔案'},404)
            except (ValueError, KeyError, OSError) as e:
                self.send({'error':str(e)},400)

        def do_PUT(self):self.do_POST()
        def do_PATCH(self):self.do_POST()
        def do_DELETE(self):self.do_POST()

        def do_POST(self):
            try:
                if self.path.startswith('/research/'):
                    if self.headers.get('Origin')!='http://'+self.headers.get('Host',''):return self.send({'error':'Origin rejected'},403)
                    n=int(self.headers.get('Content-Length',0))
                    if not 0<=n<=29*1024*1024:raise ValueError('Request too large')
                    return app.integration.relay(self,self.path,self.rfile.read(n))
                if self.path.startswith('/vehicle/'):
                    if self.headers.get('Origin')!='http://'+self.headers.get('Host','') or self.headers.get('X-Orin-Control')!='1':return self.send({'error':'Origin rejected'},403)
                    n=int(self.headers.get('Content-Length',0))
                    if not 0<n<=2048:raise ValueError('Invalid request size')
                    return app.integration.relay(self,self.path[len('/vehicle'):],self.rfile.read(n))
                # A custom header + same-origin check prevents cross-site form submissions.
                if self.headers.get('X-Workbench') != '1':
                    return self.send({'error':'需要工作台請求標頭'},403)
                origin = self.headers.get('Origin')
                if origin and origin != 'http://'+self.headers.get('Host',''):
                    return self.send({'error':'拒絕跨站寫入'},403)
                size = int(self.headers.get('Content-Length',0))
                if not 0 < size < 29*1024*1024:
                    raise ValueError('請求大小不合法')
                p = json.loads(self.rfile.read(size))
                self.send(app.post(urlparse(self.path).path, p))
            except Conflict as e:
                self.send({'error':str(e)},409)
            except (ValueError, KeyError, TypeError, OSError) as e:
                self.send({'error':str(e)},400)
    return Handler


class WorkbenchHTTPServer(ThreadingHTTPServer):
    # Windows SO_REUSEADDR permits multiple workbenches to share 8093. The
    # login supervisor can then leave clients talking to an old Python process.
    allow_reuse_address = False

    def server_bind(self):
        if hasattr(socket, 'SO_EXCLUSIVEADDRUSE'):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port',type=int,default=8093)
    parser.add_argument('--data',type=Path,default=ROOT/'data')
    args = parser.parse_args()
    app = App(args.data)
    def worker():
        while True:
            app.sim.tick()
            try:app.ops07.lab.tick()
            except Exception:pass  # Candidate stops its fake output on persistence failure.
            time.sleep(.02)
    threading.Thread(target=worker,daemon=True).start()
    server = WorkbenchHTTPServer(('127.0.0.1',args.port),handler(app))
    print(f'Laptop simulator: http://127.0.0.1:{args.port}',flush=True)
    try:
        server.serve_forever()
    finally:
        app.sim.stop('SERVER_SHUTDOWN')
        server.server_close()


if __name__ == '__main__':
    main()
