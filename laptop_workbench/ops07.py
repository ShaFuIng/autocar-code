"""07 laptop adapter. Selection is local; observation never owns vehicle control.

No prepare/start/heartbeat/restart operation is exposed until the Orin session,
package activation, continuous execution and resume protocol is implemented.
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
from ops07_candidate import Candidate
from ops07_client_candidate import Client
from ops07_runtime import REVISION

ROOT = Path(__file__).resolve().parent
PACKAGE_IDS = ('slow-steer-7fbf', 'best-today-46db', 'latest-2lap-97ed')
LOCATIONS = (
    ('flag', '旗子起點'), ('12_to_01', '12 → 01 平地直線'),
    ('02_to_03_before', '02 → 03 路口前'), ('02_to_03_after', '02 → 03 路口後'),
    ('04_to_05_before', '04 → 05 路口前'), ('04_to_05_after', '04 → 05 路口後'),
    ('05_to_06', '05 → 06 藍色路段入口'), ('ramp', '09 後上坡起點'), ('crest', '坡頂、10 前'),
)
WEIGHTS = '8b802fa03d34fa73049b92ac5c3a6df121f94d3e9f7875103ceaf50de3424e54'
KEY = 'ops07-selection'


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def inspect_package(base, package_id):
    if package_id not in PACKAGE_IDS:
        raise ValueError('找不到這個行駛方案，請重新選擇。')
    folder = base / package_id
    manifest = json.loads((folder / 'package.json').read_text(encoding='utf-8'))
    pid_bytes = (folder / 'pid-parameters.json').read_bytes()
    pid = json.loads(pid_bytes)
    if manifest['package'] != package_id or manifest['package_format'] != 1:
        raise ValueError('方案格式不符。請重新取得 Claude 的方案包。')
    recognition = manifest['recognition']
    if recognition != dict(model_id='v4-default', weights_sha256=WEIGHTS, imgsz=640):
        raise ValueError('這個方案的跑道辨識模型不符。')
    if pid['model_id'] != recognition['model_id'] or pid['weights_sha256'] != WEIGHTS or pid['path_algorithm'] != manifest['path_algorithm']:
        raise ValueError('方案與固定參數不一致。')
    files = manifest['files']
    if not files:
        raise ValueError('方案沒有程式清單。')
    for name, digest in files.items():
        target = folder / 'web' / name
        if Path(name).name != name or not target.resolve().is_relative_to((folder / 'web').resolve()):
            raise ValueError('方案檔案路徑不合法。')
        if hashlib.sha256(target.read_bytes()).hexdigest() != digest:
            raise ValueError('方案內容已改變，請重新核對交接檔。')
    if {f.name for f in (folder / 'web').iterdir() if f.is_file()} != set(files):
        raise ValueError('方案有未列入清單的檔案。')
    parameters = pid['parameters']
    if not isinstance(parameters, dict) or any(not finite(v) for v in parameters.values()):
        raise ValueError('固定參數格式不符。')
    if package_id == 'slow-steer-7fbf' and (parameters['min_us'] != 780 or parameters['steer_rate'] != 1000):
        raise ValueError('慢速轉向版缺少現役硬體的相容調整。')
    identity = hashlib.sha256(json.dumps(dict(manifest=manifest, pid=pid), sort_keys=True, ensure_ascii=False).encode()).hexdigest()
    return dict(id=package_id, name=manifest['display_name'], summary=manifest['summary'],
                reference=manifest['run'], compatibility=manifest['compatibility'],
                verified=True, file_count=len(files), identity=identity, parameters=parameters,
                parameter_revision=pid['parameter_revision'], recognition=recognition,
                path_algorithm=manifest['path_algorithm'], installed=False)


def problem(code, title, remedy, level='warning'):
    return dict(code=code, title=title, remedy=remedy, level=level)


def observation(pid, manual):
    """Only known telemetry fields. No cross-machine clock arithmetic or lap inference."""
    run = pid.get('run') or {}
    latest = pid.get('latest') or {}
    envelope = manual.get('vehicle') or {}
    vehicle = envelope.get('vehicle') or {}
    age = envelope.get('uart_age_s')
    hardware_fresh = finite(age) and 0 <= age <= 1 and not envelope.get('uart_error')
    active = run.get('state') == 'running'
    reason = str(run.get('reason') or '')
    current_reason = str(vehicle.get('reason') or '') if hardware_fresh else ''
    issues = []
    # Cumulative I2C counts alone are not an active fault.
    if hardware_fresh and (vehicle.get('pca_ok') is False or ('I2C' in current_reason and vehicle.get('state') == 'FAULT')):
        issues.append(problem('I2C', '轉向通訊異常', '先確認車子停止，再檢查控制板與轉向接線；需要重啟時由你操作。恢復後確認車況與接續位置，不會自動解除故障。', 'danger'))
    if hardware_fresh and (vehicle.get('pins') or {}).get('D11') == 0:
        issues.append(problem('ESTOP', '實體停止按鈕尚未解除', '確認現場可繼續後，解除實體停止按鈕，再重新確認車況。', 'danger'))
    elif hardware_fresh and vehicle.get('state') == 'FAULT' and not issues:
        issues.append(problem('FAULT', '控制板回報故障', '先確認車子停止；展開診斷查看原因，排除後再檢查車況。', 'danger'))
    if not hardware_fresh:
        issues.append(problem('UART_UNKNOWN', '尚未取得新鮮的控制板狀態', '檢查車端電源與控制板連線。畫面連上不代表馬達已停止。'))
    if pid.get('error') or pid.get('state')=='no_reliable_path' or any(x in reason for x in ('VISION', 'CAMERA', 'PATH', 'NO_LINE')) or (not active and pid.get('state') == 'ready' and latest.get('valid') is False):
        issues.append(problem('VISION', '跑道影像或路徑需要檢查', '確認鏡頭能看到白線與行駛路徑；若已搬車，重新選擇接續位置。舊畫面不代表目前辨識結果。'))
    if hardware_fresh and vehicle.get('state') == 'LOCKED':
        issues.append(problem('LOCKED', '車子目前鎖定', '鎖定是停止狀態，不會由網頁自動解除。起跑前需由車端完成放行檢查。', 'info'))
    # Current track-pos-r1 only writes proposal.track_pos to pid.jsonl. It is
    # deliberately NOT invented from latest/path, nor from the browser selection.
    return dict(active=active, state='running' if active else ('stopped' if run else 'idle'),
                run_id=run.get('id'), execution_mode=run.get('execution_mode'),
                reason=reason, model_state=pid.get('state'), model_id=pid.get('model_id'),
                preview_seq=pid.get('preview_seq'), observation_age_s=latest.get('age_s'),
                valid=latest.get('valid'), position=None, timing=None,
                hardware_fresh=hardware_fresh, hardware_state=vehicle.get('state'),
                generation=vehicle.get('generation'), issues=issues,
                diagnostics=dict(vehicle_reason=current_reason, model_error=pid.get('error'),
                                 stop_reason=reason, i2c_total=vehicle.get('i2c_errors'),
                                 uart_age_s=age, control_age_s=latest.get('control_age_s')))


class Operations:
    def __init__(self, app, packages=None):
        self.app = app
        self.base = packages or ROOT / 'data' / 'ops07-packages'
        self.lock = threading.RLock()
        self.polling = False
        self.last_attempt = 0
        self.received = 0
        self.error = None
        self.live = None
        self.frame_lock = threading.Lock()
        self.frame_at = 0
        self.frame_bytes = None
        self.catalog_cache = None
        self.checkpoint_at = 0
        self.lab = Candidate(app.store)
        self.session = None
        self.command_lock = threading.Lock()

    def vehicle_client(self, active=False):
        data=self.app.store.get(KEY)['data']
        if active:
            package=next((row for row in self.catalog()['packages'] if row.get('identity')==(self.session or {}).get('package_identity')),None)
            if not package:raise ValueError('尚未確認車端方案。')
        else:package=inspect_package(self.base,data.get('package_id'))
        client=Client(self.app.integration.request,package)
        client.refresh()
        return client

    def catalog(self):
        with self.lock:
            if self.catalog_cache is None:
                rows = []
                for package_id in PACKAGE_IDS:
                    try:
                        rows.append(inspect_package(self.base, package_id))
                    except (ValueError, OSError, KeyError) as exc:
                        rows.append(dict(id=package_id, name={'slow-steer-7fbf':'慢速轉向版', 'best-today-46db':'今日最佳版', 'latest-2lap-97ed':'最新兩圈版'}[package_id], verified=False, error=str(exc)))
                self.catalog_cache = rows
            active=bool(self.session and self.session.get('revision')==REVISION)
            return dict(packages=self.catalog_cache, locations=[dict(id=k, name=v) for k, v in LOCATIONS],
                        selection=self.app.store.get(KEY), capabilities=dict(start=active, resume=active,
                        traffic_light=active, front_car=False, flag_timing=active, active_package=active),
                        offline_candidate=dict(revision=REVISION,deployed=active,
                            report='/experiment/ops07-adapter-20261109/acceptance.json'),
                        limitation='依車端回報確認啟用方案。切換其他方案需停止、備份並明確授權重啟車控；前車距離來源尚未提供。')

    def _poll(self):
        try:
            pid = self.app.integration.request('/api/pid/status')
            manual = self.app.integration.request('/api/manual/status?compact=1')
            live = observation(pid, manual)
            session=None
            try:
                candidate=self.app.integration.request('/api/ops07/status')
                if candidate.get('revision')==REVISION:
                    session=candidate
                    live['timing']=candidate.get('clock')
                    if candidate.get('position'):
                        live['position']=dict(candidate['position'],sample_age_s=0,
                            current=candidate.get('state')=='running',source='ops07_vehicle')
            except Exception:
                pass
            run_id = live.get('run_id')
            if session is None and isinstance(run_id, str) and re.fullmatch('[0-9a-f]{32}', run_id):
                try:
                    extra = self.app.integration.request('/ops07/observation/' + run_id)
                    position = extra.get('position') or {}
                    age = extra.get('sample_age_s')
                    if extra.get('run_id') == run_id and finite(position.get('s_m')) and finite(age):
                        live['position'] = dict(position, sample_age_s=age, source='vehicle_log',
                                                current=live['active'] and 0 <= age < 3)
                    if extra.get('stop_reason') and not live['active']:
                        live['reason'] = extra['stop_reason']
                except Exception:
                    pass  # Observation enrichment cannot change control or hide core status.
            if live.get('position') and time.monotonic() - self.checkpoint_at >= 10:
                with self.app.store.lock:
                    old = self.app.store.get('ops07-last-observation')
                    self.app.store.save('ops07-last-observation', old['rev'], dict(run_id=run_id,
                                        position=live['position'], observed_at=time.time()))
                self.checkpoint_at = time.monotonic()
            with self.lock:
                self.live, self.received, self.error = live, time.monotonic(), None
                self.session=session
        except Exception as exc:
            with self.lock:
                self.error = type(exc).__name__  # no remote internals on the operator surface
        finally:
            with self.lock:
                self.polling = False

    def status(self):
        with self.lock:
            now = time.monotonic()
            if not self.polling and now - self.last_attempt >= (10 if self.error else 2):
                self.polling, self.last_attempt = True, now
                threading.Thread(target=self._poll, daemon=True).start()
            age = now - self.received if self.received else None
            connected = age is not None and age < 8 and self.error is None
            return dict(connected=connected, checking=self.polling, received_age_s=age,
                        live=self.live, selection=self.app.store.get(KEY),
                        last_observation=self.app.store.get('ops07-last-observation')['data'],
                        last_command=self.app.store.get('ops07-last-command')['data'],
                        transport_error=self.error, session=self.session,
                        control_supported=bool(connected and self.session and self.session.get('revision')==REVISION))

    def frame(self):
        # Cache across viewers; one bounded GET at a time, no stream queue.
        with self.frame_lock:
            with self.lock:
                if not self.received or time.monotonic() - self.received > 8 or self.error:
                    return b'', 503
                live = self.live or {}
            if live.get('model_state') not in ('ready', 'running', 'no_reliable_path'):
                return b'', 503
            if time.monotonic() - self.frame_at < 1 and self.frame_bytes:
                return self.frame_bytes, 200
            response = self.app.integration.raw('/api/pid/frame')
            payload = base64.b64decode(response.get('body', ''))
            if response['code'] != 200 or not payload.startswith(b'\xff\xd8'):
                return b'', 503
            self.frame_bytes, self.frame_at = payload, time.monotonic()
            return payload, 200

    def save_selection(self, p):
        if set(p) - {'rev', 'package_id', 'traffic_light', 'front_car', 'location', 'moved', 'phase', 'x_m'}:
            raise ValueError('07 只接受方案與功能選擇，不接受調參。')
        if type(p.get('traffic_light')) is not bool or p.get('front_car') is not False or type(p.get('moved')) is not bool:
            raise ValueError('選項格式不符；前車停等尚未接入。')
        if p.get('location') not in dict(LOCATIONS):
            raise ValueError('請選擇地圖上的接續位置。')
        if p.get('phase','first') not in ('first','after_down'):
            raise ValueError('請確認起跑所在圈段。')
        if p.get('x_m') is not None and (not finite(p['x_m']) or p['x_m']<0):
            raise ValueError('路口前距離格式不符。')
        package = inspect_package(self.base, p.get('package_id'))
        data = dict(package_id=package['id'], package_name=package['name'], identity=package['identity'],
                    parameters=package['parameters'], parameter_revision=package['parameter_revision'],
                    recognition=package['recognition'], path_algorithm=package['path_algorithm'],
                    traffic_light=p['traffic_light'], front_car=False, location=p['location'], moved=p['moved'],
                    phase=p.get('phase','first'), x_m=p.get('x_m'),
                    saved_at=time.time(), scope='laptop_selection_only', applied_to_vehicle=False)
        return self.app.store.save(KEY, p.get('rev'), data)

    def post(self, action, p):
        if action.startswith('lab-'):
            name=action[4:]
            package=None
            if name in ('prepare','resume'):
                package=inspect_package(self.base,p.get('package_id'))
            return self.lab.command(name,p,package)
        if action == 'load':
            if self.session and inspect_package(self.base,p.get('package_id'))['identity']!=self.session['package_identity']:
                raise ValueError('此方案尚未啟用；切換需先停止、備份並重啟車控。這次已部署今日最佳版，其餘方案已備妥。')
            saved=self.save_selection(p)
            if self.session:
                client=self.vehicle_client()
                client.command('load')
                saved=self.app.store.save(KEY,saved['rev'],dict(saved['data'],scope='vehicle_preview',applied_to_vehicle=True))
            return saved
        if action == 'save-selection':
            return self.save_selection(p)
        if action == 'reset':
            if set(p) != {'rev'}:
                raise ValueError('重置請求格式不符。')
            if p['rev']!=self.app.store.get(KEY)['rev']:
                raise ValueError('設定已變更，請重新讀取後再重置本場。')
            if self.session:
                self.vehicle_client(active=True).command('reset')
            return self.app.store.save(KEY, p['rev'], {})
        if action in ('start', 'resume'):
            if set(p)!={'intent','area_clear','rev'} or p['intent']!=action or p['area_clear'] is not True:
                raise ValueError('請確認現場可以起跑，且可立即使用實體停止按鈕。')
            if not self.command_lock.acquire(blocking=False):raise ValueError('上一個操作尚未回覆，請先確認車況。')
            try:
                saved=self.app.store.get(KEY)
                if p['rev']!=saved['rev']:raise ValueError('設定已由其他頁面變更，請重新讀取。')
                client=self.vehicle_client();data=saved['data']
                payload={k:data.get(k) for k in ('traffic_light','front_car','location','moved','phase','x_m')}
                payload.update(owner=uuid.uuid4().hex,area_clear=True,location_confirmed=True,start_when_ready=True)
                result=client.command('arm-hold' if action=='start' else 'resume',**payload)
                with self.lock:self.session=result;self.last_attempt=0
                return dict(ok=True,message='車端已接受，正在確認回正與路徑；放行及計時由 Orin 自主執行。',session=result)
            finally:self.command_lock.release()
        if action in ('pause', 'stop'):
            if p != {'intent': action}:
                raise ValueError('需要明確的停止操作。')
            if action=='pause' and self.session and self.session.get('id'):
                # Pause retains the route. Failure must not become a fresh start.
                client=self.vehicle_client(active=True)
                result=client.command('pause')
                with self.lock:self.session=result;self.last_attempt=0
                return dict(ok=True,message='已要求暫停並回正；路段與本場計時保留。請確認車子停妥。')
            def end_pid():
                try:
                    return self.app.integration.request('/api/pid/stop', {})
                except Exception:
                    return dict(ok=False)
            # One call per endpoint, no automatic retry after an unknown ACK.
            # Emergency output stop must not queue behind the PID stop response.
            if action == 'stop':
                from concurrent.futures import ThreadPoolExecutor
                with ThreadPoolExecutor(max_workers=2) as pool:
                    pid_stop = pool.submit(end_pid)
                    try:
                        outputs = self.app.integration.stop_all()
                    except Exception:
                        outputs = dict(confirmed=False)
                    results = [pid_stop.result(), outputs]
            else:
                results = [end_pid()]
            acknowledged = all(r.get('ok', r.get('confirmed', False)) is True for r in results)
            event = dict(action=action, at=time.time(), acknowledged=acknowledged, stopped_confirmed=False)
            with self.app.store.lock:
                old = self.app.store.get('ops07-last-command')
                self.app.store.save('ops07-last-command', old['rev'], event)
            with self.lock:
                self.last_attempt = 0
                self.received = 0  # old telemetry cannot confirm a just-issued command
            return dict(**event, message='停止請求已回覆，仍須確認控制板雙輪輸出與現場車況。' if acknowledged else '停止是否送達尚未確認。請直接使用車上實體停止按鈕；網頁不會自動重送。')
        raise ValueError('未知的 07 操作。')
