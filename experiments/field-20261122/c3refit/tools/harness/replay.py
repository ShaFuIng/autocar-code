"""[upper-edges-20261115 copy: env UE_INJECT=<path to upper_edges.py> (+ UE_CAL calibration json) adds the worker's path_diagnostic.upper_edges on frames whose request carries 'ue']
B0 replay harness: runs the real LivePID._loop of a given live_pid.py on a recorded (or fuzzed) input stream, without the car.

  python -B replay.py --live-pid <file> --run <id> --out <jsonl> [--teacher imu] [--imu log|synth] [--upper-single on|off]
  python -B replay.py --live-pid <file> --stream <dir> --out <jsonl>        (fuzz stream written by fuzz.py)

The unmodified file is imported from a private work dir (harness/_work/<tag>/web, snapshot modules + today's config) and driven
from outside: fake worker process (answers each frame with the logged vision result), fake camera, fake clock, IMU latest.json
served from the IMU log, manual-driving stub fed from controls.jsonl, model-worker / threading / subprocess replaced.
Every stub is listed in README.md. One replay per process (the stubs patch module globals)."""
import argparse, copy, json, os, queue, subprocess, sys, threading, time, types, pathlib, uuid   # queue/threading imported before the clock patch
sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import common as C   # noqa: E402

# ------------------------------------------------------------------------------------------------ fake clock
class Clock:
    now = 0.; wall_off = 0.
CLK = Clock()
CLOCK_SHIFT = float(os.environ.get('REPLAY_CLOCK_SHIFT_S', '0')); CLOCK_JITTER = float(os.environ.get('REPLAY_CLOCK_JITTER_S', '0'))
import random as _random; _JR = _random.Random(1)   # sensitivity test only: shifts the teacher-forced in-frame clock


# b23: external-read trace (main thread only): 'm' clock read, 'i' IMU latest.json read, 's' manual.state() read; per dispatched
# command the run-length encoded sequence since the previous dispatch is stored, so old vs new can be compared for read order.
EV = []; MAIN_ID = threading.get_ident()


def ev(c):
    if threading.get_ident() == MAIN_ID: EV.append(c)


def ev_take():
    out = []; prev = None; n = 0
    for c in EV:
        if c == prev: n += 1
        else:
            if prev is not None: out.append('%s%d' % (prev, n))
            prev = c; n = 1
    if prev is not None: out.append('%s%d' % (prev, n))
    del EV[:]
    return ''.join(out)


def _mono():
    ev('m'); return CLK.now


_REAL_PC = time.perf_counter   # b5: the real clock (saved before install_clock) for the per-frame cost measurement only
TIMING = dict(cam=None, frames=[])


def install_clock():
    time.monotonic = _mono
    time.time = lambda: CLK.now + CLK.wall_off
    time.perf_counter = lambda: CLK.now
    time.sleep = lambda s: None


# ------------------------------------------------------------------------------------------------ IMU latest.json
class ImuServer:
    """forced = one sample for every read of the frame (teacher 'imu'); plan = [(loop_line, sample)] per frame (teacher 'imu2'):
    a read whose _loop call site is at line L gets the sample of the last plan entry with line <= L."""
    def __init__(self, src): self.src = src; self.forced = None; self.plan = None; self.reads = 0; self.misses = 0

    def _line(self):
        f = sys._getframe(2)
        while f is not None:
            if f.f_code.co_name == '_loop' and f.f_code.co_filename.endswith('live_pid.py'): return f.f_lineno
            f = f.f_back
        return None

    def read(self):
        self.reads += 1; d = None; ev('i')
        if self.plan:
            ln = self._line(); d = self.plan[0][1]
            if ln is not None:
                for l_, s_ in self.plan:
                    if l_ <= ln: d = s_
        if d is None and self.forced is not None: d = self.src.exact(self.forced)
        if d is None: d = self.src.at(CLK.now)
        if d is None: self.misses += 1; raise FileNotFoundError('replay: no IMU sample')
        return json.dumps(d)


IMU = None
_orig_read_text = pathlib.Path.read_text


def _read_text(self, *a, **k):
    if IMU is not None and self.as_posix().endswith('/logs/imu/latest.json'): return IMU.read()
    return _orig_read_text(self, *a, **k)


# ------------------------------------------------------------------------------------------------ manual-driving session stub
class FakeManual:
    def __init__(self, root, vehicle, profile, run_id):
        self.root = pathlib.Path(root); self.vehicle = vehicle; self._profile = profile; self.run_id = run_id
        self.lock = threading.RLock(); self.current = None; self.started = False; self.sent = []; self.stops = []

    def busy(self): return self.started
    def profile(self): return dict(self._profile)
    def state(self):
        ev('s'); fv = getattr(self, 'forced', None)
        return {'vehicle': fv if fv is not None else self.vehicle.at(CLK.now)}

    def start(self, request, _pid=False):
        self.started = True; self.current = dict(id=self.run_id, status='running'); self.request = request
        (self.root / self.run_id).mkdir(parents=True, exist_ok=True)
        return dict(run_id=self.run_id)

    def pid_input(self, p):
        if TIMING['cam'] is not None: TIMING['frames'].append(_REAL_PC() - TIMING['cam']); TIMING['cam'] = None
        self.sent.append(dict(seq=p.get('seq'), target_us=p.get('target_us'), pwm=p.get('pwm'), pwm_right=p.get('pwm_right'), pwm_left=p.get('pwm_left'), ev=ev_take()))

    def request_stop(self, reason): self.stops.append(reason); self.started = False
    def save_center(self, c): self.stops.append(('save_center', c))


# ------------------------------------------------------------------------------------------------ fake path-stage objects
class FakeDash:
    """DashRoute replaced by the logged dash_route dict (route window + try_path result); mode / followed / lap emulated."""
    def __init__(self, feeder):
        self.f = feeder; self.mode = None; self.followed = False; self.turned = False; self.lap = 0; self.expect_rel = None; self.align_ok = True

    def update_route(self, now):
        dr = self.f.cur_dash
        if 'lap' in dr and dr['lap'] != self.lap: self.lap = dr['lap']; self.mode = None; self.followed = False; self.turned = False
        return dr.get('route') == 'OPEN', dict(dr)

    def try_path(self, now, jpeg, row_ratio, normal_err=None):
        dr = self.f.cur_dash
        if 'mode' in dr:
            self.mode = dr['mode']
            if dr['mode'] == 'FOLLOW': self.followed = True
        elif dr.get('reason') == 'NO_DASH_LINE': self.mode = None
        return (copy.deepcopy(self.f.cur_path) if self.f.cur_selected == 'dash_route' else None), {}


class FakeCrest:
    # crest-latch-20261116: mode 'latch' (only reached with crest_latch.json enabled) = a frame the log took from the normal path;
    # the real crest_fallback.try_path computes the midpoint on the logged mask (IMU from the replay IMU server). Other modes: the
    # logged result already carries the crest path / info (unchanged).
    def __init__(self, lp=None): self.lp = lp; self.real = None
    def normal_valid(self): pass
    def try_path(self, now, result, row_ratio, mode):
        if mode == 'latch':
            if self.real is None:
                from crest_fallback import CrestFallback
                self.real = CrestFallback(self.lp.boost)
            return self.real.try_path(now, result, row_ratio, mode)
        info = result.get('crest_fallback') or dict(revision='replay', used=False, reason='REPLAY_NO_INFO')
        return (copy.deepcopy(result.get('path') or []) if info.get('used') else None), info   # crest-latch-20261116: logged take -> its logged path


# ------------------------------------------------------------------------------------------------ fake worker process
class FakeStdin:
    def __init__(self, proc): self.p = proc
    def write(self, data):
        req = json.loads(data.decode())
        if os.environ.get('CS_REQLOG'):   # crossing-source-20261114: record request fields (no jpeg) to compare old / new live_pid
            with open(os.path.join(os.environ['CS_REQLOG'], '%d.jsonl' % os.getpid()), 'a') as f:
                f.write(json.dumps(dict(argv=' '.join(sys.argv[1:])[-160:], **{k: v for k, v in req.items() if k != 'jpeg'})) + chr(10))
        self.p.on_request(req)
    def flush(self): pass
    def close(self): self.p.q.put(None)


class FakeProc:
    def __init__(self, feeder):
        self.feeder = feeder; self.q = queue.Queue(); self.stdin = FakeStdin(self); self.stdout = self._lines()
        self.q.put((json.dumps(dict(ready=True, protocol=1, model_id=feeder.model_id, weights_sha256=feeder.weights)) + '\n').encode())

    def _lines(self):
        while True:
            x = self.q.get()
            if x is None: return
            yield x

    def on_request(self, req): self.q.put((json.dumps(self.feeder.reply(req)) + '\n').encode())
    def poll(self): return None
    def terminate(self): pass


def fake_subprocess(feeder):
    ns = types.SimpleNamespace(PIPE=-1, DEVNULL=-3, TimeoutExpired=subprocess.TimeoutExpired)
    ns.Popen = lambda *a, **k: FakeProc(feeder)
    ns.run = lambda *a, **k: None
    return ns


class FakeThread:
    """_watch (50 ms safety thread on the wall clock) is not started; _loop is run by the harness in the main thread; others real."""
    REG = []
    def __init__(self, target=None, daemon=None, **k): self.target = target; self.daemon = daemon; self.k = k; self.real = None

    def start(self):
        n = getattr(self.target, '__name__', '')
        if n == '_watch': return
        if n == '_loop': FakeThread.REG.append(self); return
        self.real = threading.Thread(target=self.target, daemon=True); self.real.start()

    def is_alive(self): return False


# ------------------------------------------------------------------------------------------------ feeder
class Feeder:
    """Serves the frames: items = [(vision, frame index or None for a synthetic preview frame)]. Three preview frames (copies of the
    first valid logged vision, 0.3 / 0.2 / 0.1 s before the first frame) make the model 'ready' before start(). restart=True (fuzz):
    when the run has ended (lost path, stale frame) and frames remain, three preview frames are inserted and the run is started again."""
    def __init__(self, start, frames, teacher=(), restart=False):
        self.start = start; self.frames = frames; self.teacher = set(teacher); self.restart = restart; self.restarts = 0
        self.model_id = start['model_id']; self.weights = start['weights_sha256']
        self.tpl = next((s for s in frames if s['vision'].get('valid')), frames[0])['vision']
        c0 = frames[0]['vision']['capture_at']; f0 = frames[0]['vision']['frame_seq']
        self.items = [(dict(self.tpl, capture_at=c0 - d, consumed_at=c0 - d + .05, frame_seq=f0 - k), None) for k, d in ((3, .3), (2, .2), (1, .1))]
        self.items += [(s['vision'], k) for k, s in enumerate(frames)]
        self.i = -1; self.cur = None; self.cur_dash = {}; self.cur_path = []; self.cur_selected = None; self.done = False; self.lp = None
        self.start_t = start['deadline'] - start['seconds'] if start.get('deadline') is not None else c0
        if 'start' in self.teacher:   # teacher-forced run start: start_t from the logged start_centre.elapsed_s (2 decimals) of the first frames
            b = [(s['vision']['consumed_at'] - s['proposal']['start_centre']['elapsed_s']) for s in frames[:40] if isinstance(s['proposal'].get('start_centre'), dict) and 'elapsed_s' in s['proposal']['start_centre']]
            if b:
                lo, hi = max(x - .005 for x in b), min(x + .005 for x in b); self.start_t = (lo + hi) / 2. if lo <= hi else lo
        self.pending_start = False

    def _item(self, i):
        v, k = self.items[i]
        return v, k is None

    def fidx(self):
        return self.items[self.i][1]

    def camera(self):
        self.i += 1
        if self.i >= len(self.items):
            self.done = True; self.lp.worker_stop.set(); return b'x', CLK.now, 10 ** 12
        v, pre = self._item(self.i)
        if not pre and self.lp.run is None:
            CLK.now = max(CLK.now, self.start_t); self.lp.start(self.start_payload()); self.after_start()
        elif not pre and self.lp.run['state'] != 'running':
            if self.pending_start:
                self.pending_start = False; self.restarts += 1
                CLK.now = max(CLK.now, v['capture_at'] - .002); self.lp.start(self.start_payload()); self.after_start()
            elif self.restart:   # insert three preview frames between the previous frame and this one, then start again
                pv = self.items[self.i - 1][0]; a0 = max(pv['capture_at'], pv['consumed_at'] - .5); b0 = v['capture_at']; fa = self.items[self.i - 1][0]['frame_seq']
                if b0 - a0 > .02 and v['frame_seq'] - fa > 3:
                    ins = [(dict(self.tpl, capture_at=a0 + (b0 - a0) * q, consumed_at=max(CLK.now, a0 + (b0 - a0) * q) + .001, frame_seq=fa + j + 1), None) for j, q in enumerate((.25, .5, .75))]
                    self.items[self.i:self.i] = ins; self.pending_start = True; v, pre = self._item(self.i)
        CLK.now = max(CLK.now, v['capture_at'])
        TIMING['cam'] = _REAL_PC()   # b5: frame cost = real time from here (camera returned) to the pid_input of this frame
        return b'replay-jpeg', v['capture_at'], v['frame_seq']

    def start_payload(self):
        s = self.start
        if os.environ.get('REPLAY_MAX_GAP'):   # c2-crossing-20261117: same override on both sides of a comparison (the free-running replay clock can exceed 0.4 s on the upper deck)
            s = dict(s, parameters=dict(s['parameters'], max_gap=float(os.environ['REPLAY_MAX_GAP'])))
        return dict(parameters=copy.deepcopy(s['parameters']), execution_mode=s.get('execution_mode', 'orin_timed'), owner=uuid.uuid4().hex, seconds=s['seconds'],
                    area_clear=True, parameter_revision=s['parameter_revision'], path_algorithm=s['path_algorithm'])

    def after_start(self):
        lp = self.lp; lp.dash = FakeDash(self); lp.crest = FakeCrest(lp)
        if 'clock' in self.teacher:   # teacher-forced clock: from imu_boost.step on, the frame clock = logged imu_t + imu_age_s (the car's clock at that call, 1 ms rounding)
            orig = lp.boost.step; fd = self
            def step(base, ceiling):
                p = fd.frames[fd.fidx()]['proposal'] if fd.fidx() is not None else {}; ib = p.get('imu_boost') or {}
                if ib.get('imu_t') is not None and ib.get('imu_age_s') is not None: CLK.now = max(CLK.now, float(ib['imu_t']) + float(ib['imu_age_s']) + CLOCK_SHIFT + (_JR.uniform(0., CLOCK_JITTER) if CLOCK_JITTER else 0.))
                return orig(base, ceiling)
            lp.boost.step = step

    vplan = None; lines = {}

    def imu_plan(self, p):
        """teacher 'imu2': which IMU sample each logged read site saw (boost imu_t exact; heading_hold yaw_deg, yaw_inner gz, gyro_damp gz
        matched by value among the next 0-2 samples), as [(loop line, sample)]."""
        src = IMU.src; it = (p.get('imu_boost') or {}).get('imu_t')
        if it is None: return None
        import bisect
        k = bisect.bisect_left(src.t, float(it) - 1e-6)
        if k >= len(src.t) or abs(src.t[k] - float(it)) > 1e-4: return None
        plan = [(self.lines.get('boost', 0), src.d[k])]
        hh = p.get('heading_hold') or {}; yi = p.get('yaw_inner') or {}; gd = p.get('gyro_damp') or {}
        anchors = [('heading', hh.get('yaw_deg') if hh.get('reason') not in ('NO_REFERENCE',) else None, lambda d: round(float(d['yaw_deg']), 2)),
                   ('yaw_inner', yi.get('gz'), lambda d: round(float(d['gyro_dps'][2]), 2)),
                   ('gyro_damp', gd.get('gz'), lambda d: round(float(d['gyro_dps'][2]), 2))]
        for name, val, f in anchors:
            if val is None or name not in self.lines: continue
            for kk in (k, k + 1, k + 2):
                if kk < len(src.d) and f(src.d[kk]) == float(val):
                    if kk != k: plan.append((self.lines[name], src.d[kk]))
                    k = kk; break
        return plan

    def reply(self, req):
        v, pre = self._item(self.i); self.cur = v
        CLK.now = max(CLK.now, v['consumed_at'])
        r = copy.deepcopy({k: x for k, x in v.items() if k not in C.STRIP})
        r['valid'] = bool(v.get('valid')) or ('sample_row_ratio' in v)
        r.update(seq=req['seq'], protocol=1, model_id=self.model_id, path_algorithm=req['path_algorithm'], weights_sha256=self.weights)
        self.cur_dash = copy.deepcopy(v.get('dash_route') or {}); self.cur_path = v.get('path') or []; self.cur_selected = v.get('selected_source')
        cf_ = v.get('crest_fallback') or {}
        if v.get('selected_source') == 'crest_midpoint' and cf_.get('used') and cf_.get('mode') == 'invalid':   # crest-latch-20261116: the worker result
            r['valid'] = False; r.pop('selected_source', None)   # before the crest take was invalid; live_pid calls crest (mode 'invalid'), FakeCrest returns the logged path
        nc = v.get('downhill_nocorridor')
        if isinstance(nc, dict) and 'reason' not in nc:   # the inline downhill-nocorridor re-fires on the logged result: make it reproduce the final path
            j = (r.get('path_diagnostic') or {}).get('junction')
            if isinstance(j, dict): j['baseline_path'] = copy.deepcopy(v.get('path') or [])
        if os.environ.get('UE_INJECT') and req.get('ue') and r.get('mask_png'):   # upper-edges-20261115: emulate the worker hunk (detect on the logged mask) when live_pid asks
            _ue_inject(r)
        if os.environ.get('AL_INJECT'):   # arc-lat-20261119: worker arc_fit (pts) output recomputed offline from the logged mask, keyed by frame_seq
            a_ = _al_inj().get(str(v.get('frame_seq')))
            pd_ = r.get('path_diagnostic')
            if a_ is not None and isinstance(pd_, dict): pd_['arc_fit'] = copy.deepcopy(a_)
        IMU.forced = None; IMU.plan = None
        if self.lp is not None: self.lp.manual.forced = None
        if not pre and self.vplan is not None: self.lp.manual.forced = self.vplan.get(self.fidx())
        if 'imu2' in self.teacher and not pre: IMU.plan = self.imu_plan(self.frames[self.fidx()]['proposal'])
        if 'imu' in self.teacher and not pre:
            p = self.frames[self.fidx()]['proposal']; it = (p.get('imu_boost') or {}).get('imu_t')
            if it is not None: IMU.forced = float(it)
        return r


_ALI = None
def _al_inj():
    global _ALI
    if _ALI is None:
        p_ = os.path.join(os.environ['AL_INJECT'], 'inj_%s.json' % os.environ.get('AL_RUN', ''))
        _ALI = json.load(open(p_, encoding='utf8')) if os.path.exists(p_) else {}
    return _ALI


_UEM = None
def _ue_inject(r):
    """worker side of upper-edges-r1 on the logged mask (same call and default config as the live_model_worker hunk)."""
    global _UEM
    import base64, numpy as np, cv2, importlib.util
    if _UEM is None:
        spec = importlib.util.spec_from_file_location('ue_worker_copy', os.environ['UE_INJECT']); _UEM = importlib.util.module_from_spec(spec); spec.loader.exec_module(_UEM)
        _UEM._H = _UEM.load_h(os.environ.get('UE_CAL'))[0]
    m = cv2.imdecode(np.frombuffer(base64.b64decode(r['mask_png']), np.uint8), cv2.IMREAD_GRAYSCALE)
    pd = r.get('path_diagnostic')
    if isinstance(pd, dict): pd['upper_edges'] = _UEM.detect(m, _UEM._H, {'lines': False})


# ------------------------------------------------------------------------------------------------ main
def main(argv=None):
    global IMU
    ap = argparse.ArgumentParser()
    ap.add_argument('--live-pid', required=True); ap.add_argument('--run'); ap.add_argument('--stream'); ap.add_argument('--out', required=True)
    ap.add_argument('--teacher', default=''); ap.add_argument('--imu', default='auto'); ap.add_argument('--upper-single', default='auto')
    ap.add_argument('--tag'); ap.add_argument('--vehicle-time', default='received'); ap.add_argument('--trace', default=''); ap.add_argument('--restart', action='store_true')
    ap.add_argument('--inject', type=int, default=None)   # b23: fault / force injection seed (differential only)
    a = ap.parse_args(argv)
    teacher = [x for x in a.teacher.split(',') if x]
    if a.stream:
        sd = pathlib.Path(a.stream); start, frames, _ = C.load_pid(sd / 'pid.jsonl')
        rows = [json.loads(l) for l in open(sd / 'controls.jsonl', encoding='utf8')]; rid = 'fuzz'
        imu_src = C.ImuLog(-1e9, 1e12, path=sd / 'imu.jsonl')
    else:
        rid = a.run; start, frames, _ = C.load_pid(C.run_dir(rid) / 'pid.jsonl'); rows = C.load_controls(rid) if (C.run_dir(rid) / 'controls.jsonl').exists() and not os.environ.get('L4_SYNTH_VEH') else None; os.environ['AL_RUN'] = rid   # landing4: no controls.jsonl -> SynthVehicle
        im = a.imu if a.imu != 'auto' else ('log' if rid in C.IMU_BOOT_RUNS else 'synth')
        imu_src = C.ImuLog(frames[0]['vision']['capture_at'] - 5., frames[-1]['vision']['consumed_at'] + 5., path=C.IMU_FOR.get(rid, C.IMU_LOG)) if im == 'log' else C.SynthImu(frames)
        if os.environ.get('L4_IMU') == 'rec': imu_src = C.RecImu(frames, float((start.get('imu_boost') or {}).get('flat_pitch_deg', 2.6)))   # landing4
    us = None if a.upper_single == 'auto' else (a.upper_single == 'on')
    tag = a.tag or ('%s_%s_%d' % (rid, pathlib.Path(a.live_pid).parent.name, os.getpid()))
    root = C.build_workdir(tag, a.live_pid, rid=rid if rid in C.RUNS else None, upper_single=us if us is not None else (None if rid in C.RUNS else True),
                           extra_cfg=INJECT_CFG if a.inject is not None else None)
    xc = os.environ.get('REPLAY_EXTRA_CFG')   # b5 (from b4): config overrides merged into web/<file> (one level deep) AFTER the work dir (incl. injection config) is built
    if xc:
        for n_, v_ in json.loads(xc).items():
            p_ = root / 'web' / n_; cur_ = json.loads(p_.read_text(encoding='utf8')) if p_.exists() else {}
            for k_, x_ in v_.items(): cur_[k_] = dict(cur_[k_], **x_) if isinstance(x_, dict) and isinstance(cur_.get(k_), dict) else x_
            p_.write_text(json.dumps(cur_), encoding='utf8')
    cd_ = os.environ.get('L4_CFG_DIR')   # landing4: whole-file config overrides (every *.json of the folder replaces web/<name>)
    if cd_:
        import shutil as _sh
        for q_ in sorted(pathlib.Path(cd_).glob('*.json')): _sh.copyfile(q_, root / 'web' / q_.name)
    os.environ['USERPROFILE'] = str(root / 'home'); os.environ['HOME'] = str(root / 'home')
    CLK.now = frames[0]['vision']['capture_at'] - 1.
    CLK.wall_off = sorted(s['wall'] - s['vision']['consumed_at'] for s in frames)[len(frames) // 2]
    install_clock()
    IMU = ImuServer(imu_src); pathlib.Path.read_text = _read_text
    if not hasattr(os, 'getuid'): os.getuid = lambda: 1000; os.getgid = lambda: 1000
    sys.path.insert(0, str(root / 'web'))
    tracer = None
    if a.trace:   # line coverage of the file under test (lines listed in --trace file, or all)
        target = str((root / 'web' / 'live_pid.py').resolve()); hits = set()
        others = {str((root / 'web' / q.name).resolve()): q.name for q in C.sibling_modules(a.live_pid) if q.suffix == '.py'}; ohits = {}   # b23: stage modules
        def tracer(frame, event, arg):
            fn = frame.f_code.co_filename
            if fn == target:
                if event == 'line': hits.add(frame.f_lineno)
                return tracer
            if fn in others:
                if event == 'line': ohits.setdefault(others[fn], set()).add(frame.f_lineno)
                return tracer
            return None
    import live_pid as L
    if os.environ.get('LD_LAYER_TRACE'):   # landing-20261119 diagnostic only: per-layer steer before / after into proposal['_lt'] (never used for proofs)
        SC = L.segment_ctx; _orig_changed = SC.changed
        def _run(table, obj, F):
            lt = F.p.setdefault('_lt', [])
            for it in table:
                if it.elig is not None and not it.elig(obj, F): continue
                if isinstance(it, SC.Group):
                    if it.on_error is None: _run(it.layers, obj, F)
                    else:
                        try: _run(it.layers, obj, F)
                        except Exception as e: it.on_error(obj, F, e)
                    continue
                b = SC._steer(F.p)
                try: it.fn(obj, F)
                finally:
                    a_ = SC._steer(F.p)
                    if _orig_changed(b, a_): F.owner = it.name; lt.append([it.name, None if b is None else round(float(b), 1), None if a_ is None else round(float(a_), 1)])
        SC.run = _run
    import downhill_entry
    inj = install_inject(a.inject) if a.inject is not None else None
    feeder = Feeder(start, frames, teacher, restart=bool(a.stream) or a.restart)
    src_lines = (root / 'web' / 'live_pid.py').read_text(encoding='utf8').split('\n')
    for name, needle in (('boost', "proposal['pwm'],proposal['imu_boost']=self.boost.step"), ('heading', "proposal['steer_us'],proposal['heading_hold']=self.heading.step"),
                         ('yaw_inner', 'try:gzy=self.heading._gz'), ('gyro_damp', 'try:gzd=self.heading._gz')):
        hits_ = [i + 1 for i, l in enumerate(src_lines) if needle in l]
        if len(hits_) == 1: feeder.lines[name] = hits_[0]
    vehicle = C.Vehicle(rows, mode=a.vehicle_time) if rows is not None else C.SynthVehicle(frames)   # landing4
    if 'vehicle' in teacher: feeder.vplan = vehicle_plan(frames, vehicle)
    L.subprocess = fake_subprocess(feeder)
    L.threading = types.SimpleNamespace(RLock=threading.RLock, Lock=threading.Lock, Event=threading.Event, Thread=FakeThread)
    # path stage P2-P8: the logged vision is already after these stages -> neutralised (their logged info keys are kept)
    L.corner_widen = lambda result: None
    L.downhill_narrow = lambda result: None
    L.downhill_pair = types.SimpleNamespace(apply=lambda result, *x, **k: (None, result.get('downhill_pair')), REVISION=L.downhill_pair.REVISION)
    bl = L.bottom_left
    L.bottom_left = types.SimpleNamespace(in_window=bl.in_window, floor=bl.floor, REVISION=bl.REVISION, gate=lambda result, f: (None, result.get('bottom_left')))
    downhill_entry.DownhillEntry.mirror = lambda self, *x, **k: None
    sp = start['parameters']
    manual = FakeManual(root / 'runs', vehicle, dict(center_us=sp['center_us'], min_us=780, max_us=2050), start['id'])
    lp = L.LivePID(manual, feeder.camera, threading.Lock(), lambda: False)
    feeder.lp = lp
    lp.prepare(dict(model_id=start['model_id'], row_ratio=sp['row_ratio'], path_algorithm=start['path_algorithm'],
                    visual_center_offset_px_640=sp.get('visual_center_offset_px_640', 0.), curve_outward_px_640=sp.get('curve_outward_px_640', 0.)))
    if tracer: sys.settrace(tracer)
    t0 = _wall_now()
    FakeThread.REG[-1].target()   # LivePID._loop, synchronously
    if tracer: sys.settrace(None)
    err = lp.error; state = lp.state
    lp.end('REPLAY_END')
    # collect
    rp = root / 'runs' / start['id'] / 'pid.jsonl'
    rstart, rsamples, rother = C.load_pid(rp) if rp.exists() else (None, [], [])
    sent = {m['seq']: m for m in manual.sent}
    with open(a.out, 'w', encoding='utf8') as f:
        f.write(json.dumps(dict(kind='meta', run=rid, live_pid=os.path.abspath(a.live_pid), teacher=teacher, imu=type(imu_src).__name__, frames_in=len(frames),
                                samples_out=len(rsamples), pid_inputs=len(manual.sent), restarts=feeder.restarts, loop_error=err, loop_state=state, imu_reads=IMU.reads, imu_misses=IMU.misses,
                                start=rstart, other=rother, stops=manual.stops, secs=round(_wall_now() - t0, 1),
                                coverage=sorted(hits) if tracer else None, coverage_modules={k: sorted(v) for k, v in ohits.items()} if tracer else None, state_reads=getattr(manual, 'state_reads', None), inject=inj,
                                frame_cost_s=[round(x, 7) for x in TIMING['frames']])) + '\n')
        for k, s in enumerate(rsamples):
            m = sent.get(s['pid_input_seq']) or {}
            v = {x: y for x, y in (s['vision'] or {}).items() if x not in ('mask_png', 'path_diagnostic')}
            flags = sorted(x for x in s if x not in ('kind', 'wall', 'vision', 'proposal', 'pid_input_seq'))
            f.write(json.dumps(dict(i=k, frame_seq=(s['vision'] or {}).get('frame_seq'), consumed_at=(s['vision'] or {}).get('consumed_at'), reason=s['proposal'].get('reason'),
                                    pre_steer=s['proposal'].get('steer_us'), sent_steer=m.get('target_us'), ev=m.get('ev'), pwm=m.get('pwm'), pwm_right=m.get('pwm_right'), pwm_left=m.get('pwm_left'),
                                    flags=flags, proposal=s['proposal'], vision=v)) + '\n')
    shutil_rm(root)
    return 0


# b23: fault / force injection (differential only). Class methods of the modules live_pid calls are wrapped: each call draws from
# one seeded RNG in call order, raises RuntimeError('replay-inject') with probability RATE, and SolidTakeover.step returns a forced
# takeover / release with probability FORCE. Old and new files see the same injections as long as their call sequence is the same
# (which is part of what is compared). Reaches the ERROR handlers and branches the recorded / fuzz streams never reach.
INJECT_TARGETS = (('heading_hold', 'HeadingHold', ('_gz', '_yaw', 'step', 'blind', 'lost')), ('curve_map', 'CurveMap', ('step', 'lock_dps')),
                  ('yaw_inner', 'YawInner', ('step',)), ('cmd_comp', 'CmdComp', ('step', 'sent')), ('curve_ff', 'CurveFF', ('step',)),
                  ('speed_adapt', 'SpeedAdapt', ('step',)), ('upper_path', 'UpperPath', ('step',)), ('brake_zones', 'BrakeZones', ('update',)),
                  ('imu_boost', 'ImuBoost', ('step',)), ('downhill_speed', 'SpeedMeter', ('update',)), ('downhill_bottom', 'BottomJump', ('step',)),
                  ('lane_segment', 'LaneSegment', ('step',)), ('solid_takeover', 'SolidTakeover', ('step',)), ('flat_dip_guard', 'FlatDipGuard', ('step', 'clamp')),
                  ('downhill_entry', 'DownhillEntry', ('blind_hold',)), ('play_comp', 'PlayComp', ('apply',)), ('crest_marks', 'CrestMarks', ('arm', 'offset')),
                  ('upper_lateral', 'UpperLateral', ('offset',)), ('centre_estimator', 'CentreEstimator', ('step',)))


def install_inject(seed, rate=.03, force=.25):
    import importlib, random
    R = random.Random(seed); stats = {}
    def wrap(mod, cls, name, fn):
        key = '%s.%s.%s' % (mod, cls, name)
        def w(*a, **k):
            if threading.get_ident() != MAIN_ID: return fn(*a, **k)
            x = R.random()
            if x < rate: stats[key] = stats.get(key, 0) + 1; raise RuntimeError('replay-inject')
            if key == 'solid_takeover.SolidTakeover.step' and x < rate + force:
                stats[key + '#force'] = stats.get(key + '#force', 0) + 1
                y = R.random()
                return (1000. + 900. * R.random() if y < .6 else None), dict(revision='solid-takeover-inject', active=y < .6, released=y >= .4)
            return fn(*a, **k)
        return w
    for mod, cls, names in INJECT_TARGETS:
        try: C_ = getattr(importlib.import_module(mod), cls)
        except Exception: continue
        for n in names:
            if hasattr(C_, n): setattr(C_, n, wrap(mod, cls, n, getattr(C_, n)))
    return stats


INJECT_CFG = {   # config variants for injection runs: branches today's car config switches off
    'bottom_map.json': {"enabled": True, "turn_deg": 130, "target_dps": 30, "ff_us": 280, "k_us_per_dps": 6, "cap_us": 420, "pwm": 15, "left_floor_us": 100,
                        "max_s": 2.5, "min_turned_deg": 15, "pre_right_us": 150, "pre_right_after_s": 2.5},
}


def vehicle_plan(frames, veh):
    """teacher 'vehicle': the MCU status each frame saw, recovered from the logged brake_zones odometer d_m (2 decimals,
    = (enc_b counts - counts at the first brake-zone call) x 0.0222 cm). The zero status is chosen among the statuses near the first
    d_m frame so that the most frames match within 0.006 m; frames without d_m (holds, WARMUP) keep the time rule."""
    import bisect
    cnt = []
    for v in veh.v:
        b = (v or {}).get('enc_b') or {}
        try: cnt.append(int(b['forward']) - int(b['reverse']))
        except (KeyError, TypeError, ValueError): cnt.append(None)
    fd = [(k, float((s['proposal'].get('brake_zones') or {})['d_m']), s['vision']['consumed_at']) for k, s in enumerate(frames)
          if isinstance((s['proposal'].get('brake_zones') or {}).get('d_m'), (int, float))]
    if not fd: return {}
    j0 = bisect.bisect_right(veh.t, fd[0][2]) - 1; best = None
    for jz in range(max(0, j0 - 3), min(len(cnt), j0 + 4)):
        if cnt[jz] is None: continue
        plan = {}; ok = 0
        for k, dm, tc in fd:
            jc = bisect.bisect_right(veh.t, tc) - 1; cand = [j for j in range(max(0, jc - 3), min(len(cnt), jc + 4)) if cnt[j] is not None]
            j = min(cand, key=lambda j: abs((cnt[j] - cnt[jz]) * .000222 - dm)) if cand else None
            if j is not None and abs((cnt[j] - cnt[jz]) * .000222 - dm) <= .006: plan[k] = veh.v[j]; ok += 1
        if best is None or ok > best[0]: best = (ok, plan)
    return best[1]


def _wall_now():
    return time.process_time()


def shutil_rm(root):
    import shutil
    try: shutil.rmtree(root)
    except OSError: pass


if __name__ == '__main__':
    sys.exit(main())
