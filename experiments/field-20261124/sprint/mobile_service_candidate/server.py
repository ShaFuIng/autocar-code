"""mobile-service-r1 (2026-11-21): phone page -> the real car. Runs ON the Orin as its own small HTTP service.

Serves the phone page (web/) and translates its 7 calls to the dashboard on port 8000 of the same machine:
  GET  /api/pid/status   -> dashboard /api/pid/status + /api/status, reshaped for the page (see view())
  GET  /api/pid/mask     -> dashboard /api/pid/mask (unchanged)
  POST /api/pid/prepare  -> fixed model request from start_params.json (already loaded = ok)
  POST /api/pid/start    -> fixed start request from start_params.json, new owner, steering centre / limits of the car's saved profile
  POST /api/pid/stop | pause | resume -> passed through (pause / resume carry the page's run_id)
No other dashboard endpoint is reachable (no servo, motor test, manual drive, settings).
r3 (2026-11-21): POST /api/race/arm | disarm = lap-race start mode; this service reads the organiser's /go flag and sends the same start on 0 -> 1.
r6 (2026-11-23): GET /api/start, POST /api/start/set {back_m} | /api/start/rebuild-ref = start position behind the flag spot (start_cfg.py);
status carries 'start'. Writes only the rule keys of web/lap_pos.json, brake_zones.json, lap_ff.json, start_bias.json (read at every run start).
r7 (field-20261124 sprint, offline candidate): GET /api/sprint, POST /api/sprint/set {on, finish_m} = straight-line sprint mode (sprint_cfg.py;
writes only web/speed_adapt.json hard_cap / max_pwm / up_rate + key sprint_mode and the "sprint" zones of brake_zones.json, both read at every
run start). status carries 'sprint'. While sprint is on: start position changes and the race start mode are refused; a start is refused while the
sprint files are half-written or do not match. With sprint off every other route behaves as r6.

Login: the same HTTP Basic login as the dashboard (user jetson + the token in ~/.config/orin_lab/token). The token stays on the Orin:
it is read here, compared with what the browser sends, and used towards port 8000. It is never sent to the page or printed.
POST needs X-Orin-Control: 1, a JSON body <= 2048 bytes and, when the browser sends Origin, the same host as Host.
Listens only on the machine's own IPv4 addresses (hostname -I) + 127.0.0.1; exits when the address set changes (systemd restarts it).
Python 3 standard library. No control module is imported, no device is opened."""
import base64, hmac, json, os, subprocess, sys, threading, time, urllib.request, urllib.error, uuid
import start_cfg
import sprint_cfg
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

REVISION = 'mobile-service-r7'
ROOT = Path(__file__).resolve().parent
WEB = ROOT / 'web'
PARAMS = ROOT / 'start_params.json'
TOKEN = Path.home() / '.config/orin_lab/token'
PORT = int(os.environ.get('MOBILE_PORT', '8092'))
UPSTREAM_PORT = int(os.environ.get('MOBILE_UPSTREAM_PORT', '8000'))
STATIC = {'/': ('index.html', 'text/html; charset=utf-8'), '/index.html': ('index.html', 'text/html; charset=utf-8'), '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
          '/model.mjs': ('model.mjs', 'text/javascript; charset=utf-8'), '/style.css': ('style.css', 'text/css; charset=utf-8'), '/repair.js': ('repair.js', 'text/javascript; charset=utf-8'),
          '/race.js': ('race.js', 'text/javascript; charset=utf-8'), '/start.js': ('start.js', 'text/javascript; charset=utf-8'),
          '/sprint.js': ('sprint.js', 'text/javascript; charset=utf-8')}
# dashboard refusal text -> code the page knows (anything else: CAR_REFUSED + the original text)
REFUSALS = (('Need 3 consecutive fresh valid model results', 'NO_FRESH_PATH'), ('Another controller is active', 'BUSY'), ('Stop PID first', 'BUSY'),
            ('PID center must equal saved shared steering center', 'CENTER_MISMATCH'), ('Stop vehicle control before loading model', 'BUSY'),
            ('Preview already running', 'ALREADY_LOADED'))
_base = [None]
ACCESS = ROOT / 'access.log'
SERVICE = ROOT / 'service.json'
LOG_LOCK = threading.Lock()


def addresses():
    out = ['127.0.0.1']
    try: out += [a for a in subprocess.check_output(['hostname', '-I'], text=True).split() if ':' not in a]
    except Exception: pass
    return out


def login_required():
    """service.json next to this file: {"login": false} = no login on the phone page. Absent / broken / anything else = login as the dashboard."""
    try: return json.loads(SERVICE.read_text(encoding='utf8')).get('login') is not False
    except (OSError, ValueError, AttributeError): return True


def credential():
    return 'Basic ' + base64.b64encode(('jetson:' + TOKEN.read_text().strip()).encode()).decode()


def upstream(path, body=None, timeout=4.):
    """(http code, parsed JSON or {'raw': ...}) from the dashboard. Tries the last working address first."""
    cred = credential(); last = None
    hosts = ([_base[0]] if _base[0] else []) + [h for h in addresses() if h != _base[0]]
    for host in hosts:
        base = 'http://%s:%d' % (host, UPSTREAM_PORT); h = {'Authorization': cred}; data = None
        if body is not None:
            data = json.dumps(body).encode(); h.update({'Origin': base, 'X-Orin-Control': '1', 'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(urllib.request.Request(base + path, data=data, headers=h), timeout=timeout) as r:
                _base[0] = host; return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            _base[0] = host; raw = e.read()
            try: return e.code, json.loads(raw)
            except Exception: return e.code, {'raw': raw[:200].decode('utf8', 'replace')}
        except (OSError, ValueError) as e: last = e
    raise RuntimeError('dashboard unreachable: ' + type(last).__name__)


def refusal(reply, default='CAR_REFUSED'):
    """dashboard 409 body -> (code, text) for the page."""
    text = str(reply.get('reason') or reply.get('error') or reply.get('raw') or '')
    if reply.get('code'): return str(reply['code']), text
    for needle, code in REFUSALS:
        if needle in text: return code, text
    return default, text


def view(pid, veh, imu=None):
    """reshape the dashboard status for the page (pure function; tested). imu = ImuWatch.snapshot() or None."""
    run = pid.get('run'); r = None
    if run:
        r = dict(id=run.get('id'), state='running' if run.get('state') == 'running' else 'ended', seconds=run.get('seconds'), reason=run.get('reason'))
    state = pid.get('state'); v = (veh or {}).get('vehicle') or {}
    fault = None
    if (v.get('pins') or {}).get('D11') == 0: fault = 'ESTOP'
    elif (veh or {}).get('uart_error') or v.get('pca_ok') is False: fault = 'FAULT'
    elif state in ('failed', 'camera_unavailable'): fault = 'MODEL_FAILED'
    elif imu and imu.get('state') in ('frozen', 'stale', 'missing') and (imu.get('bad_s') or 0) >= 1.: fault = 'IMU_FAULT'   # r2: no start with a dead IMU
    t = pid.get('timing'); timing = {}
    if isinstance(t, dict):
        laps = [dict(active_s=l.get('active_s'), total_s=l.get('total_s')) for l in (t.get('laps') or [])]
        cur = t.get('current_lap_active_s')
        if not laps and (cur is None or t.get('active_s') is None or abs(cur - t['active_s']) < .05): cur = None   # before the first timing point
        timing = dict(active_s=t.get('active_s'), total_s=t.get('total_s'), paused_s=t.get('paused_s'), pauses=t.get('pauses'), current_lap_s=cur, laps=laps)
    return dict(ok=True, service=REVISION, version='%s / pause-resume v%s' % (v.get('fw') or '?', pid.get('pause_resume_version', '?')),
                model_state=state, model_ready=state in ('ready', 'no_reliable_path'), path_ready=state == 'ready', ready_frames=pid.get('ready_frames'),
                run=r, pause=pid.get('pause'), timing=timing, remaining_s=pid.get('remaining_s'), fault=fault, imu=imu,
                vehicle=dict(state=v.get('state'), reason=v.get('reason')), centre_us=((veh or {}).get('servo_settings') or {}).get('center_us'))


def start_body(params, veh):
    s = json.loads(json.dumps(params['start'])); s['owner'] = uuid.uuid4().hex
    prof = (veh or {}).get('servo_settings') or {}
    for k in ('center_us', 'min_us', 'max_us'):   # one shared centre: the saved servo profile of the car wins over the frozen file
        if isinstance(prof.get(k), int): s['parameters'][k] = prof[k]
    return s


# ---------------------------------------------------------------------------------------------- IMU health + repairs (mobile-service-r2, 2026-11-21)
IMU_LATEST = Path.home() / 'orin_interface_v1/logs/imu/latest.json'
IMU_UNIT = 'orin-imu-logger.service'
FROZEN_S, STALE_S, AUTO_AFTER_S, AUTO_GAP_S = 2.0, 1.5, 3.0, 30.0


class ImuWatch:
    """judges the IMU logger's latest.json (read twice a second by imu_loop).
    frozen  = the time stamp advances but gyro / pitch / roll did not change for FROZEN_S. A live MPU6050 changes in the last digit on every
              sample; on 2026-11-21 18:10 the chip returned zeros for 9 minutes while the service stayed 'active' (run 25ca351f).
    stale   = the newest sample is older than STALE_S (logger stopped).      missing = no readable file."""
    def __init__(self):
        self.lock = threading.Lock(); self.key = None; self.changed = None; self.last = None; self.state = 'unknown'; self.bad_since = None
        self.auto = None; self.auto_at = None

    def judge(self, d, now):
        with self.lock:
            if not isinstance(d, dict) or not isinstance(d.get('t'), (int, float)): st = 'missing'
            else:
                key = (tuple(d.get('gyro_dps') or ()), d.get('pitch_deg'), d.get('roll_deg'))
                if key != self.key: self.key = key; self.changed = now
                self.last = dict(pitch_deg=d.get('pitch_deg'), roll_deg=d.get('roll_deg'), t=d['t'])
                if now - d['t'] > STALE_S: st = 'stale'
                elif self.changed is not None and now - self.changed >= FROZEN_S: st = 'frozen'
                else: st = 'ok'
            if st == 'ok': self.bad_since = None
            elif self.bad_since is None: self.bad_since = now
            self.state = st; return st

    def forget(self):
        with self.lock: self.key = None; self.changed = None; self.bad_since = None; self.state = 'unknown'

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            l = self.last or {}
            return dict(state=self.state, pitch_deg=l.get('pitch_deg'), roll_deg=l.get('roll_deg'), age_s=None if l.get('t') is None else round(now - l['t'], 2),
                        bad_s=None if self.bad_since is None else round(now - self.bad_since, 1), auto=self.auto)


IMU = ImuWatch()


def setting(key, default):
    try: v = json.loads(SERVICE.read_text(encoding='utf8')).get(key, default)
    except (OSError, ValueError, AttributeError): v = default
    return v


def restart_unit(unit):
    subprocess.run(['systemctl', '--user', 'restart', unit], check=True, timeout=20, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def run_active():
    _, pid = upstream('/api/pid/status')
    return (pid.get('run') or {}).get('state') == 'running'


def imu_restart(source):
    """restart the IMU logger (it wakes the chip and takes a new gyro zero from one still second). Never while a run is active:
    the yaw of the logger restarts from 0 and the car must stand still. Returns (ok, refusal code)."""
    if run_active(): return False, 'RUN_ACTIVE'
    restart_unit(IMU_UNIT); IMU.forget(); note_line('%s repair imu restart (%s)' % (time.strftime('%H:%M:%S'), source))
    return True, None


BOOST = Path.home() / 'orin_interface_v1/web/imu_boost.json'   # flat_pitch_deg lives here; the car control reads it at every run start
LEVEL_BACKUPS = Path.home() / 'orin_interface_v1/backups'
LEVEL_S, LEVEL_MIN_N, LEVEL_SD_DEG, LEVEL_GYRO_DPS, LEVEL_MAX_PITCH, LEVEL_MAX_STEP = 2.0, 12, 0.15, 1.5, 6.0, 3.0


def level_samples(seconds=LEVEL_S, read=None, sleep=time.sleep, clock=time.monotonic):
    """distinct IMU samples (by time stamp) seen in latest.json during `seconds`."""
    read = read or (lambda: json.loads(IMU_LATEST.read_text(encoding='utf8')))
    out = []; seen = None; end = clock() + seconds
    while clock() < end:
        try: d = read()
        except (OSError, ValueError): d = None
        if isinstance(d, dict) and isinstance(d.get('t'), (int, float)) and d['t'] != seen and isinstance(d.get('pitch_deg'), (int, float)):
            seen = d['t']; out.append(d)
        sleep(.04)
    return out


def level_value(samples, old, offset):
    """(new flat_pitch_deg, static pitch, None) or (None, static pitch or None, refusal code). The car must stand still on the flat track."""
    if len(samples) < LEVEL_MIN_N: return None, None, 'IMU_FAULT'
    p = [float(d['pitch_deg']) for d in samples]; mean = sum(p) / len(p); sd = (sum((x - mean) ** 2 for x in p) / len(p)) ** .5
    gz = max((abs(float(g)) for d in samples for g in (d.get('gyro_dps') or ()) if isinstance(g, (int, float))), default=0.)
    if sd > LEVEL_SD_DEG or gz > LEVEL_GYRO_DPS: return None, round(mean, 2), 'IMU_NOT_STILL'
    if abs(mean) > LEVEL_MAX_PITCH: return None, round(mean, 2), 'IMU_NOT_FLAT'
    new = round(mean + float(offset), 2)
    if not isinstance(old, (int, float)) or isinstance(old, bool) or abs(new - float(old)) > LEVEL_MAX_STEP: return None, round(mean, 2), 'IMU_LEVEL_RANGE'
    return new, round(mean, 2), None


def level_info(pitch):
    """r4: what the page shows next to the level button. calibrated_static_deg = the static pitch the present flat_pitch_deg stands for
    (flat_pitch_deg - offset); diff_deg = the pitch now minus that (meaningful only with the car standing still on the flat track)."""
    try: flat = json.loads(BOOST.read_text(encoding='utf8')).get('flat_pitch_deg')
    except (OSError, ValueError, AttributeError): flat = None
    if not isinstance(flat, (int, float)) or isinstance(flat, bool): return None
    cal = round(float(flat) - float(setting('imu_level_offset_deg', 0.8)), 2)
    return dict(flat_pitch_deg=flat, calibrated_static_deg=cal, diff_deg=None if not isinstance(pitch, (int, float)) else round(float(pitch) - cal, 2))


def imu_now():
    s = IMU.snapshot(); s['level'] = level_info(s.get('pitch_deg')); return s


def imu_level(source, samples=None):
    """mobile-service-r4 (2026-11-22, user: the IMU mount of this car shifts by about a degree and cannot be fixed mechanically): with the
    car standing still on the flat track, set web/imu_boost.json flat_pitch_deg = static pitch + imu_level_offset_deg (service.json,
    default 0.8: on 2026-11-22 static 0.28 with flat 1.1 gave a driving pitch of -0.7 and 0.1 % of flat frames beyond 3 deg). Only that
    one number changes; the old file is copied to backups/imulevel_<time>/ first. The car control reads the file at the next run start.
    Returns (ok, refusal code, detail)."""
    _, pid = upstream('/api/pid/status')
    if (pid.get('run') or {}).get('state') in ('running', 'paused'): return False, 'RUN_ACTIVE', None
    if IMU.snapshot().get('state') != 'ok': return False, 'IMU_FAULT', None
    text = BOOST.read_text(encoding='utf8'); cfg = json.loads(text); old = cfg.get('flat_pitch_deg')
    new, static, code = level_value(level_samples() if samples is None else samples, old, setting('imu_level_offset_deg', 0.8))
    detail = dict(static_pitch_deg=static, old_flat_pitch_deg=old, flat_pitch_deg=new)
    if code: return False, code, detail
    b = LEVEL_BACKUPS / ('imulevel_' + time.strftime('%Y%m%d_%H%M%S')); b.mkdir(parents=True, exist_ok=True); (b / BOOST.name).write_text(text, encoding='utf8')
    cfg['flat_pitch_deg'] = new; tmp = BOOST.with_name(BOOST.name + '.tmp')
    tmp.write_text(json.dumps(cfg, ensure_ascii=False, indent=1) + chr(10), encoding='utf8'); os.replace(tmp, BOOST)
    note_line('%s repair imu level (%s): static %.2f, flat_pitch_deg %s -> %s, backup %s' % (time.strftime('%H:%M:%S'), source, static, old, new, b.name))
    return True, None, detail


def imu_level_default(source):
    """mobile-service-r5 (2026-11-22, user): put flat_pitch_deg back to the tuned value (service.json imu_level_default_deg, default 1.0)
    without measuring anything -- the way back after a level calibration at a spot that reads high. Same backup and write as imu_level.
    Returns (ok, refusal code, detail)."""
    _, pid = upstream('/api/pid/status')
    if (pid.get('run') or {}).get('state') in ('running', 'paused'): return False, 'RUN_ACTIVE', None
    text = BOOST.read_text(encoding='utf8'); cfg = json.loads(text); old = cfg.get('flat_pitch_deg'); new = round(float(setting('imu_level_default_deg', 1.0)), 2)
    detail = dict(old_flat_pitch_deg=old, flat_pitch_deg=new)
    if old == new: return True, None, detail
    b = LEVEL_BACKUPS / ('imulevel_' + time.strftime('%Y%m%d_%H%M%S')); b.mkdir(parents=True, exist_ok=True); (b / BOOST.name).write_text(text, encoding='utf8')
    cfg['flat_pitch_deg'] = new; tmp = BOOST.with_name(BOOST.name + '.tmp')
    tmp.write_text(json.dumps(cfg, ensure_ascii=False, indent=1) + chr(10), encoding='utf8'); os.replace(tmp, BOOST)
    note_line('%s repair imu level default (%s): flat_pitch_deg %s -> %s, backup %s' % (time.strftime('%H:%M:%S'), source, old, new, b.name))
    return True, None, detail


def model_reload():
    if run_active(): return False, 'RUN_ACTIVE', None
    upstream('/api/pid/preview-stop', {}, timeout=20.); time.sleep(1.)
    code, r = upstream('/api/pid/prepare', json.loads(PARAMS.read_text(encoding='utf8'))['prepare'], timeout=8.)
    note_line('%s repair model reload -> %s' % (time.strftime('%H:%M:%S'), code))
    if code == 200 and r.get('ok') is not False: return True, None, None
    c, text = refusal(r); return False, c, text


def note_line(line):
    try:
        with LOG_LOCK:
            with ACCESS.open('a', encoding='utf8') as f: f.write(line + chr(10))
    except Exception: pass


def imu_loop():
    """background: judge the IMU twice a second; with imu_auto_restart (default on) restart the logger when it has been bad for
    AUTO_AFTER_S, no run is active and the last automatic restart is at least AUTO_GAP_S ago."""
    misses = 0; d = None
    while True:
        time.sleep(.5); now = time.monotonic()
        try: d = json.loads(IMU_LATEST.read_text(encoding='utf8')); misses = 0
        except (OSError, ValueError):
            misses += 1
            if misses >= 3: d = None   # one unreadable read (the logger is rewriting the file) is not 'missing'
        st = IMU.judge(d, now)
        try:
            if (st in ('frozen', 'stale', 'missing') and setting('imu_auto_restart', True) is not False and IMU.bad_since is not None
                    and now - IMU.bad_since >= AUTO_AFTER_S and (IMU.auto_at is None or now - IMU.auto_at >= AUTO_GAP_S)):
                IMU.auto_at = now; ok, code = imu_restart('auto: ' + st)
                IMU.auto = dict(at=time.strftime('%H:%M:%S'), was=st, ok=ok, code=code)
        except Exception as e:
            IMU.auto = dict(at=time.strftime('%H:%M:%S'), was=st, ok=False, code=type(e).__name__)


# ---------------------------------------------------------------------------------------------- start position (mobile-service-r6, 2026-11-23)
START_CFG_DIR = Path.home() / 'orin_interface_v1/web'   # the car control reads the four files from here at every run start
START_REF = ROOT / 'start_ref.json'                     # flag-spot values of the shifted keys (phone service only)
_start_cache = [None, None]


def start_now():
    """start_cfg.state() of the car files, re-read only when one of the five files changed (status is polled several times a second)."""
    key = []
    for p in [START_CFG_DIR / n for n in start_cfg.FILES] + [START_REF]:
        try: st = p.stat(); key.append((str(p), st.st_mtime_ns, st.st_size))
        except OSError: key.append((str(p), None, None))
    if _start_cache[0] != key:
        s = start_cfg.state(START_CFG_DIR, START_REF); s.pop('styles', None); _start_cache[:] = [key, s]
    return dict(_start_cache[1])


def start_set(back_m, source, force=False):
    """(ok, refusal code, detail). Never while a run is active or paused (the files are read at the run start; a paused run keeps its old ones,
    but a changed start between pause and the next start is easy to miss) and never when the car control cannot be asked."""
    _, pid = upstream('/api/pid/status')
    if (pid.get('run') or {}).get('state') in ('running', 'paused'): return False, 'RUN_ACTIVE', None
    if sprint_now().get('on') is True: return False, 'SPRINT_ON', None   # r7: the sprint zones were placed for the present start position
    try: r = start_cfg.apply(START_CFG_DIR, START_REF, LEVEL_BACKUPS, back_m, log=lambda t: note_line('%s %s (%s)' % (time.strftime('%H:%M:%S'), t, source)), force=force is True)
    except start_cfg.Refused as e: return False, e.code, dict(problems=e.problems)
    _start_cache[0] = None; r['start'] = start_now(); return True, None, r


def start_rebuild(source):
    try: r = start_cfg.rebuild_ref(START_CFG_DIR, START_REF, LEVEL_BACKUPS, log=lambda t: note_line('%s %s (%s)' % (time.strftime('%H:%M:%S'), t, source)))
    except start_cfg.Refused as e: return False, e.code, dict(problems=e.problems)
    _start_cache[0] = None; r['start'] = start_now(); return True, None, r


_sprint_cache = [None, None]


def sprint_params():
    return sprint_cfg.params(setting('sprint', None))


def sprint_now():
    """r7: sprint_cfg.state() of the car files, re-read only when one of the six files (or service.json) changed."""
    key = []
    for p in [START_CFG_DIR / n for n in sprint_cfg.ALL] + [START_REF, SERVICE]:
        try: st = p.stat(); key.append((str(p), st.st_mtime_ns, st.st_size))
        except OSError: key.append((str(p), None, None))
    if _sprint_cache[0] != key:
        _sprint_cache[:] = [key, sprint_cfg.state(START_CFG_DIR, START_REF, sprint_params())]
    return dict(_sprint_cache[1])


def sprint_set(on, finish_m, source):
    """(ok, refusal code, detail). Same refusals as start_set (run active / paused, car control unreachable); race mode is refused in do_POST."""
    _, pid = upstream('/api/pid/status')
    if (pid.get('run') or {}).get('state') in ('running', 'paused'): return False, 'RUN_ACTIVE', None
    if not isinstance(on, bool): return False, 'SPRINT_BAD_INPUT', None
    try: r = sprint_cfg.apply(START_CFG_DIR, START_REF, LEVEL_BACKUPS, on, finish_m, sprint_params(), log=lambda t: note_line('%s %s (%s)' % (time.strftime('%H:%M:%S'), t, source)))
    except start_cfg.Refused as e: return False, e.code, dict(problems=e.problems)
    _sprint_cache[0] = None; _start_cache[0] = None; r['sprint'] = sprint_now(); return True, None, r


def sprint_block_start():
    """r7: refusal code when a start must not be sent because the sprint files are half-written / do not match, else None.
    Concrete failure: speed_adapt.json hard_cap 100 without the after-cap zone lets speed_adapt's HIGH level command PWM 100 in every
    steady stretch of the lap, curves included (sprint_cfg docstring). A consistent sprint (or sprint off) starts as before."""
    s = sprint_now()
    if s.get('on') and not s.get('consistent'): return s.get('reason') or 'SPRINT_INCONSISTENT'
    return None


def start_run():
    """the one start path (page button and race green light): fixed start_params.json + the car's saved servo profile, new owner."""
    sb = sprint_block_start()   # r7
    if sb: return 409, dict(ok=False, code=sb, reason=sb)
    _, veh = upstream('/api/status')
    return upstream('/api/pid/start', start_body(json.loads(PARAMS.read_text(encoding='utf8')), veh))


# ---------------------------------------------------------------------------------------------- race start flag (mobile-service-r3, 2026-11-21)
# CAR-INTEGRATION.md (race organiser): GET http://192.168.0.149:8080/go -> "1" from the green light of a lap race, otherwise "0".
# Armed from the page; while armed (and only then) the flag is read every RACE_INTERVAL_S with timeout RACE_TIMEOUT_S. A start happens only
# on 0 -> 1 (a flag already at 1 when arming must go back to 0 first); arming is one-shot (cleared before the start is sent). After the
# start the flag is not read any more: no automatic stop on 0 (user decision 2026-11-21: pause / resume / stop by hand instead).
# service.json: race_go_url (default below), race_practice (true = append ?practice=1, light practice also counts; never in a real race).
RACE_URL_DEFAULT = 'http://192.168.0.149:8080/go'
RACE_INTERVAL_S, RACE_TIMEOUT_S, RACE_RETRY_S = .05, .2, 1.0


def race_url():
    url = setting('race_go_url', RACE_URL_DEFAULT)
    if not isinstance(url, str) or not url.startswith('http://'): url = RACE_URL_DEFAULT
    if setting('race_practice', False) is True: url += ('&' if '?' in url else '?') + 'practice=1'
    return url


def read_go(url, timeout=RACE_TIMEOUT_S):
    """1 / 0, or None when the flag cannot be read (offline, timeout, HTTP error)."""
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return 1 if r.read(8).strip() == b'1' else 0
    except (OSError, ValueError):
        return None


class RaceGo:
    """armed / saw_zero / last flag; step() is the pure edge rule (tested)."""
    def __init__(self):
        self.lock = threading.Lock(); self.armed = False; self.saw_zero = False; self.flag = None; self.read_ms = None; self.misses = 0
        self.armed_at = None; self.firing = False; self.result = None; self.reads = 0

    def arm(self):
        with self.lock:
            if self.armed or self.firing: return False
            self.armed = True; self.saw_zero = False; self.flag = None; self.read_ms = None; self.misses = 0; self.reads = 0
            self.armed_at = time.strftime('%H:%M:%S'); self.result = None; return True

    def disarm(self):
        with self.lock: was = self.armed; self.armed = False; return was

    def step(self, value, read_ms=None):
        """one flag read. True = start now (and the race mode is already off, so a second start cannot follow)."""
        with self.lock:
            if not self.armed: return False
            self.flag = value; self.read_ms = read_ms; self.reads += 1
            self.misses = self.misses + 1 if value is None else 0
            if value == 0: self.saw_zero = True
            elif value == 1 and self.saw_zero:
                self.armed = False; self.firing = True; return True
            return False

    def finish(self, result):
        with self.lock: self.firing = False; self.result = result

    def snapshot(self):
        with self.lock:
            return dict(armed=self.armed, firing=self.firing, saw_zero=self.saw_zero, flag=self.flag, read_ms=self.read_ms, misses=self.misses,
                        reads=self.reads, armed_at=self.armed_at, result=self.result, practice=setting('race_practice', False) is True)


RACE = RaceGo()


def race_fire(edge):
    """green light seen at monotonic time edge: check the car once more, send the start. NO_FRESH_PATH is retried for RACE_RETRY_S
    (a frame or two short of the 3 fresh results); a start whose reply is lost is NOT resent (acceptance unknown)."""
    res = dict(at=time.strftime('%H:%M:%S'), ok=False, code=None, text=None, run_id=None, tries=0)
    try:
        while True:
            _, pid = upstream('/api/pid/status'); _, veh = upstream('/api/status'); v = view(pid, veh, imu_now())
            if (v['run'] or {}).get('state') == 'running': res['code'] = 'BUSY'; break
            if v['fault']: res['code'] = v['fault']; break
            if sprint_now().get('on') is True: res['code'] = 'SPRINT_ON'; break   # r7: never a race start in sprint mode (arming is refused too)
            res['tries'] += 1
            try: code, r = upstream('/api/pid/start', start_body(json.loads(PARAMS.read_text(encoding='utf8')), veh))
            except Exception: res['code'] = 'START_UNKNOWN'; break
            if code == 200 and r.get('ok') is not False: res.update(ok=True, run_id=r.get('run_id')); break
            res['code'], res['text'] = refusal(r)
            if res['code'] == 'NO_FRESH_PATH' and time.monotonic() - edge < RACE_RETRY_S: time.sleep(.1); continue
            break
    except Exception as e:
        res.update(code='CAR_UNREACHABLE', text=type(e).__name__)
    res['latency_ms'] = round((time.monotonic() - edge) * 1000)
    RACE.finish(res)
    note_line('%s race green -> %s %s %s ms tries=%d' % (res['at'], 'start ok' if res['ok'] else 'no start', res['code'] or res['run_id'], res['latency_ms'], res['tries']))


def race_loop():
    """background: idle (no network) unless armed; armed: read the flag every RACE_INTERVAL_S."""
    nxt = time.monotonic()
    while True:
        if not RACE.armed:
            time.sleep(.05); nxt = time.monotonic(); continue
        t0 = time.monotonic(); value = read_go(race_url()); t1 = time.monotonic()
        try:
            if RACE.step(value, round((t1 - t0) * 1000)): race_fire(t1)
        except Exception as e:
            RACE.finish(dict(at=time.strftime('%H:%M:%S'), ok=False, code='CAR_UNREACHABLE', text=type(e).__name__))
        nxt = max(nxt + RACE_INTERVAL_S, time.monotonic()); time.sleep(max(0., nxt - time.monotonic()))


def race_arm():
    """(ok, refusal code). The car must be ready to go now: model loaded and seeing the path, no fault, no run."""
    _, pid = upstream('/api/pid/status'); _, veh = upstream('/api/status'); v = view(pid, veh, imu_now())
    if (v['run'] or {}).get('state') == 'running': return False, 'BUSY'
    if v['fault']: return False, v['fault']
    if not v['model_ready']: return False, 'MODEL_NOT_READY'
    if not v['path_ready']: return False, 'NO_FRESH_PATH'
    if sprint_now().get('on') is True: return False, 'SPRINT_ON'   # r7: a lap race must not start with the sprint files
    if not RACE.arm(): return False, 'RACE_ARMED'
    note_line('%s race armed (%s)' % (time.strftime('%H:%M:%S'), race_url()))
    return True, None


class Handler(BaseHTTPRequestHandler):
    server_version = 'OrinMobile/1'; protocol_version = 'HTTP/1.1'; timeout = 8

    def log_message(self, fmt, *a): pass

    def send(self, code, obj=None, raw=None, ctype='application/json; charset=utf-8', extra=None):
        data = raw if raw is not None else json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code); self.send_header('Content-Type', ctype); self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store'); self.send_header('Connection', 'close')
        for k, v in (extra or {}).items(): self.send_header(k, v)
        self.end_headers(); self.wfile.write(data); self.close_connection = True
        self.note(code)

    def note(self, code):
        """access log without credentials: time, client, method, path, status, login (none / ok / wrong), browser."""
        try:
            a = self.headers.get('Authorization'); login = 'none' if not a else ('ok' if hmac.compare_digest(a, credential()) else 'wrong')
            if self.path.startswith('/api/pid/mask') or (self.path.startswith('/api/pid/status') and code == 200): return   # polled several times a second
            line = '%s %s %s %s %s login=%s ua=%s' % (time.strftime('%H:%M:%S'), self.client_address[0], self.command, self.path[:60], code, login, (self.headers.get('User-Agent') or '')[:70]) + chr(10)
            with LOG_LOCK:
                if ACCESS.exists() and ACCESS.stat().st_size > 2_000_000: ACCESS.write_text('')
                with ACCESS.open('a', encoding='utf8') as f: f.write(line)
        except Exception: pass

    def authed(self):
        if not login_required(): return True   # service.json {"login": false}: the page is open to everyone on the network (user decision 2026-11-21)
        try: ok = hmac.compare_digest(self.headers.get('Authorization', ''), credential())
        except Exception: ok = False
        if not ok: self.send(401, raw=b'', extra={'WWW-Authenticate': 'Basic realm="Orin"'})
        return ok

    def do_GET(self):
        if not self.authed(): return
        path = self.path.split('?', 1)[0]
        try:
            if path in STATIC:
                name, ctype = STATIC[path]; return self.send(200, raw=(WEB / name).read_bytes(), ctype=ctype)
            if path == '/api/pid/status':
                _, pid = upstream('/api/pid/status'); _, veh = upstream('/api/status')
                out = view(pid, veh, imu_now()); out['race'] = RACE.snapshot(); out['start'] = start_now(); out['sprint'] = sprint_now(); return self.send(200, out)
            if path == '/api/start': return self.send(200, dict(ok=True, start=start_now()))   # r6: local files only, no car call
            if path == '/api/sprint': return self.send(200, dict(ok=True, sprint=sprint_now()))   # r7: local files only, no car call
            if path == '/api/pid/mask':
                code, m = upstream('/api/pid/mask')
                return self.send(200, m) if code == 200 else self.send(503, dict(ok=False, reason='NO_MASK', reason_text=str(m.get('reason_text') or '尚無辨識畫面')))
            self.send(404, dict(ok=False, reason='NOT_FOUND'))
        except Exception as e:
            self.send(502, dict(ok=False, reason='CAR_UNREACHABLE', reason_text=type(e).__name__))

    def do_POST(self):
        if not self.authed(): return
        path = self.path.split('?', 1)[0]
        if self.headers.get('X-Orin-Control') != '1': return self.send(403, dict(ok=False, reason='FORBIDDEN'))
        origin = self.headers.get('Origin')
        if origin and origin.split('://', 1)[-1] != self.headers.get('Host', ''): return self.send(403, dict(ok=False, reason='FORBIDDEN'))
        try: n = int(self.headers.get('Content-Length', '0'))
        except ValueError: n = -1
        if not 0 < n <= 2048: return self.send(409, dict(ok=False, reason='BAD_REQUEST'))
        try: p = json.loads(self.rfile.read(n))
        except ValueError: return self.send(409, dict(ok=False, reason='BAD_REQUEST'))
        if not isinstance(p, dict): return self.send(409, dict(ok=False, reason='BAD_REQUEST'))
        try:
            if path == '/api/pid/stop':
                if RACE.disarm(): note_line('%s race disarmed (stop)' % time.strftime('%H:%M:%S'))
                code, r = upstream('/api/pid/stop', {})
            elif path in ('/api/pid/pause', '/api/pid/resume'): code, r = upstream(path, {'run_id': p.get('run_id')})
            elif path == '/api/race/arm':   # r3
                ok, c = race_arm()
                return self.send(200, dict(ok=True, race=RACE.snapshot())) if ok else self.send(409, dict(ok=False, reason=c))
            elif path == '/api/race/disarm':
                if RACE.disarm(): note_line('%s race disarmed (page)' % time.strftime('%H:%M:%S'))
                return self.send(200, dict(ok=True, race=RACE.snapshot()))
            elif (path.startswith('/api/repair/') or path.startswith('/api/start/') or path.startswith('/api/sprint/')) and (RACE.armed or RACE.firing):   # r3: keep the car as it was armed (r6: start position too; r7: sprint too)
                return self.send(409, dict(ok=False, reason='RACE_ARMED'))
            elif path == '/api/pid/prepare':
                _, pid = upstream('/api/pid/status')
                if pid.get('state') in ('ready', 'no_reliable_path', 'loading'): return self.send(200, dict(ok=True, already=True))
                code, r = upstream('/api/pid/prepare', json.loads(PARAMS.read_text(encoding='utf8'))['prepare'], timeout=8.)
                if code != 200 and refusal(r)[0] == 'ALREADY_LOADED': return self.send(200, dict(ok=True, already=True))
            elif path == '/api/pid/start':
                if RACE.firing: return self.send(409, dict(ok=False, reason='BUSY'))
                if RACE.disarm(): note_line('%s race disarmed (manual start)' % time.strftime('%H:%M:%S'))
                code, r = start_run()
            elif path == '/api/repair/imu':   # r2: restart the IMU logger (not the car control)
                ok, c = imu_restart('manual')
                return self.send(200, dict(ok=True)) if ok else self.send(409, dict(ok=False, reason=c))
            elif path == '/api/repair/imu-level':   # r4: flat-ground pitch baseline from the car standing still
                ok, c, detail = imu_level('manual')
                return self.send(200, dict(ok=True, **detail)) if ok else self.send(409, dict(ok=False, reason=c, **(detail or {})))
            elif path == '/api/repair/imu-level-default':   # r5: flat-ground baseline back to the tuned value, nothing measured
                ok, c, detail = imu_level_default('manual')
                return self.send(200, dict(ok=True, **detail)) if ok else self.send(409, dict(ok=False, reason=c))
            elif path == '/api/start/set':   # r6: start position behind the flag spot
                ok, c, detail = start_set(p.get('back_m'), 'page', force=p.get('force') is True)   # r6a: force = rewrite from the reference
                return self.send(200, dict(ok=True, **detail)) if ok else self.send(409, dict(ok=False, reason=c, **(detail or {})))
            elif path == '/api/sprint/set':   # r7: straight-line sprint mode on / off
                ok, c, detail = sprint_set(p.get('on'), p.get('finish_m'), 'page')
                return self.send(200, dict(ok=True, **detail)) if ok else self.send(409, dict(ok=False, reason=c, **(detail or {})))
            elif path == '/api/start/rebuild-ref':   # r6: declare the car's present files correct (explicit, from the page)
                if sprint_now().get('on') is True: return self.send(409, dict(ok=False, reason='SPRINT_ON'))   # r7
                ok, c, detail = start_rebuild('page')
                return self.send(200, dict(ok=True, **detail)) if ok else self.send(409, dict(ok=False, reason=c, **(detail or {})))
            elif path == '/api/repair/model':   # r2: unload + load the model again
                ok, c, text = model_reload()
                return self.send(200, dict(ok=True)) if ok else self.send(409, dict(ok=False, reason=c, reason_text=text))
            else: return self.send(404, dict(ok=False, reason='NOT_FOUND'))
            if code == 200 and r.get('ok') is not False: return self.send(200, r)
            c, text = refusal(r); out = dict(ok=False, reason=c, reason_text=text)
            if isinstance(r.get('pause'), dict): out['pause'] = r['pause']
            self.send(409, out)
        except Exception as e:
            self.send(502, dict(ok=False, reason='CAR_UNREACHABLE', reason_text=type(e).__name__))


def main():
    credential()   # fail early when the token file is missing
    addrs = addresses(); servers = []
    for a in addrs:
        try:
            s = ThreadingHTTPServer((a, PORT), Handler); s.daemon_threads = True; servers.append(s)
            threading.Thread(target=s.serve_forever, daemon=True).start()
        except OSError as e: print('cannot listen on %s:%d (%s)' % (a, PORT, e), flush=True)
    if not servers: sys.exit(1)
    threading.Thread(target=imu_loop, daemon=True).start()
    threading.Thread(target=race_loop, daemon=True).start()   # r3: idle until armed from the page
    print('%s listening on %s port %d' % (REVISION, ', '.join(addrs), PORT), flush=True)
    while True:
        time.sleep(5)
        if sorted(addresses()) != sorted(addrs): print('addresses changed; exiting for restart', flush=True); os._exit(0)


if __name__ == '__main__':
    main()
