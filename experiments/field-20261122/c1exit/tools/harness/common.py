"""[arc2-20261116 copy of curve4-ff-20261116/harness: + e904 702a d122 853c f70e] [curve4-ff-20261116 copy of merge-20261115e/harness: + 11-16 runs and their 50 Hz IMU] [upper-edges-20261115 copy of lap-ff-20261115/harness: 11-15 runs + 50 Hz IMU for e8e1/3474/d043/2485; replay.py UE_INJECT]
Shared helpers for the B0 replay harness (laptop only, read-only on everything outside integration-20261112/b01).

Data sources (read-only):
  EXP/run-<id>-<day>/pid.jsonl      start record + per-frame samples (vision = self.latest, proposal = per-layer sub-dicts)
  EXP/run-<id>-<day>/controls.jsonl  manual_drive control ticks with the cached MCU status (encoders, steer command)
  EXP/imu-20261112/imu_*.jsonl       50 Hz IMU logger record (t = Orin monotonic clock, same clock as proposal.t)
Code under test is copied into a private work dir (harness/_work/...) together with the snapshot modules and the car's config.
"""
import bisect, json, math, os, shutil, statistics
from datetime import datetime
from pathlib import Path

B01 = Path(__file__).resolve().parents[1]   # arc-lat-20261119: the arc-lat folder
EXP = Path(os.environ.get('L4_EXP', r'D:\OneDrive\Desktop\自走車\Orin_Interface_v1\laptop_workbench\data\experiments'))   # landing4: explicit (the harness lives outside experiments/)
HERE_B01 = Path(__file__).resolve().parent   # stuck-stop-20261114 copy: experiments/
SNAP = EXP / 'orin-web-snapshot-20261114d'   # next-20261115: car 'shut-down' snapshot (9972) # downhill-20261114: the exact car snapshot (live_pid bfd1efeb) # map-20261114 copy: today's car snapshot (live_pid e6cad710 and every module / config on the car)
FIX33 = SNAP   # map-20261114: upper_single.py / mask_dash.py / upper_single.json taken from the same snapshot (no fix33 override)
IMU_LOG = EXP / 'imu-20261112' / 'imu_20261111_205556.jsonl'
WORK = Path(os.environ['DH_WORK']) if os.environ.get('DH_WORK') else B01 / 'harness' / '_work'   # downhill-20261114: work dir outside OneDrive (file locks)
RUNS = {'68f9': '20261112', 'ffc2': '20261112', 'a674': '20261112', '676e': '20261112', '1b6c': '20261112', 'cba7': '20261111'}
# stuck-stop-r1 copy: extra runs (proof C only) from REPLAY_EXTRA_RUNS='4879:20261113,da93:20261110' (synthetic IMU, upper_single as in the brief default)
for _x in [x for x in os.environ.get('REPLAY_EXTRA_RUNS', '').split(',') if x]: RUNS.setdefault(_x.split(':')[0], _x.split(':')[1])
CURRENT = ('68f9', 'ffc2', 'a674')          # produced by fix33 itself
IMU_BOOT_RUNS = ('e8e1', '3474', 'd043', '2485', 'fab5', '796c', '8bb3', 'd077', '4148', 'b3c4', '68f9', 'ffc2', 'a674', '676e', '1b6c', 'a747', '6d63', '1db9', '78d6', 'ac4f', '51a0', '40e9', '3cf4')
IMU_FOR = {r: EXP / 'imu-20261114' / 'imu_20261113_204500.jsonl' for r in ('a747', '6d63', '1db9', '78d6', 'ac4f', '51a0', '40e9')}
IMU_FOR['fab5'] = IMU_FOR['796c'] = IMU_FOR['8bb3'] = IMU_FOR['d077'] = IMU_FOR['4148'] = IMU_FOR['3cf4'] = IMU_FOR['b3c4'] = EXP / 'imu-20261114' / 'imu_20261114_163942.jsonl'   # verify-20261114: after the 16:39 Orin reboot   # map-20261114: today's runs use today's 50 Hz IMU log   # inside the IMU log's boot (cba7 is an earlier boot -> synthetic IMU)
IMU_FOR.update({r: EXP / 'imu-20261115' / 'imu_20261114_194318.jsonl' for r in ('e8e1', '3474', 'd043', '2485')})
IMU_FOR.update({r: EXP / 'imu-20261116' / 'imu_20261115_105817.jsonl' for r in ('f341', 'bfdb', 'a163', '8aba', '0631')})   # curve4-ff-20261116
IMU_BOOT_RUNS = IMU_BOOT_RUNS + ('f341', 'bfdb', 'a163', '8aba', '0631') + ('e904', '702a', 'd122', '853c', 'f70e')   # arc2-20261116: later 11-16 runs
IMU_FOR.update({r: EXP / 'imu-20261116' / 'imu_20261115_105817.jsonl' for r in ('e904', '702a', 'd122', '853c', 'f70e')})   # upper-edges-20261115: 11-15 runs (3c6d: other boot -> synthetic)
UPPER_SINGLE_ON_15 = ('3c6d', 'e8e1', '3474', 'd043', '2485')
# Car config today that differs from the snapshot (brief, 2026-11-12)
PLAY_COMP = json.loads((SNAP / 'play_comp.json').read_text(encoding='utf8'))   # upper-20261115: the car files of snapshot-d (no 11-12 override)
CROSSING_PASS = json.loads((SNAP / 'crossing_pass.json').read_text(encoding='utf8'))   # upper-20261115: snapshot-d
UPPER_SINGLE_ON = {'68f9': True, 'ffc2': True, 'a674': False, '676e': True, '1b6c': False, 'cba7': False}
UPPER_SINGLE_ON.update({'fab5': True, '796c': True, '8bb3': True, 'd077': True, '4148': True, '9972': True, 'b3c4': True, '3cf4': True, '78d6': True, 'ac4f': True, '51a0': True, '40e9': True, 'a747': True, '6d63': True, '1db9': True, '4879': True, '4e63': True})
UPPER_SINGLE_ON.update({r: True for r in ('3c6d', 'e8e1', '3474', 'd043', '2485')})   # upper-edges-20261115   # map-20261114: upper_single enabled on the car for every 11-13 / 11-14 run
UPPER_SINGLE_ON.update({r: True for r in ('f341', 'bfdb', 'a163', '8aba', '0631') + ('e904', '702a', 'd122', '853c', 'f70e')})
# keys live_pid adds to the worker result / self.latest after the worker (stripped before feeding the logged vision back);
# path-stage info keys (crest_fallback, downhill_nocorridor, bottom_left, downhill_mirror_identity, downhill_entry, corner_width,
# downhill_width, downhill_pair) are KEPT because the replay neutralises those stages and they are not recomputed.
STRIP = ('dash_route', 'turn_preview', 'sample_row_ratio', 'requested_row_ratio', 'sample_reason', 'sampling_version', 'derivative_rebased',
         'sample_clamped', 'sample_bounds', 'visual_center_offset_px_640', 'visual_center_version', 'capture_at', 'frame_seq', 'error',
         'consumed_at', 'control_age_s', 'flat_dip_guard', 'downhill_release', 'bottom_commit_state', 'upper_single',
         'curve_outward_px_640', 'curve_outward_version', 'road_center_path', 'outward')


def run_dir(rid):
    return EXP / ('run-%s-%s' % (rid, RUNS[rid]))


def load_pid(path):
    start = None; samples = []; other = []
    with open(path, encoding='utf8') as f:
        for line in f:
            d = json.loads(line)
            if d.get('kind') == 'start' and start is None: start = d
            elif d.get('kind') == 'sample': samples.append(d)
            else: other.append(d)
    return start, samples, other


def iso_wall(s):
    return datetime.fromisoformat(s).timestamp()


class Vehicle:
    """manual.state()['vehicle'] at a monotonic time: the MCU status with the latest receive estimate <= t.
    receive estimate = controls timing.cache_received_estimate_monotonic_s (Orin monotonic); rows without timing use
    wall - (median wall-monotonic offset of the timed rows) - uart age unknown (0)."""
    def __init__(self, rows, mode='received'):
        pts = []; offs = []
        for r in rows:
            tm = (r.get('feedback_observation') or {}).get('timing') or {}
            if tm.get('cache_read_monotonic_s') is not None: offs.append(iso_wall(r['time']) - tm['cache_read_monotonic_s'])
        off = statistics.median(offs) if offs else None
        for r in rows:
            v = r.get('vehicle')
            if not isinstance(v, dict): continue
            tm = (r.get('feedback_observation') or {}).get('timing') or {}
            if mode == 'received' and tm.get('cache_received_estimate_monotonic_s') is not None: t = tm['cache_received_estimate_monotonic_s']
            elif tm.get('cache_read_monotonic_s') is not None: t = tm['cache_read_monotonic_s']
            elif off is not None: t = iso_wall(r['time']) - off
            else: continue
            pts.append((t, v.get('ms'), v))
        pts.sort(key=lambda x: x[0])
        # one entry per distinct MCU status (ms), earliest time seen
        seen = {}; out = []
        for t, ms, v in pts:
            if ms in seen: continue
            seen[ms] = 1; out.append((t, v))
        out.sort(key=lambda x: x[0])
        self.t = [x[0] for x in out]; self.v = [x[1] for x in out]

    def at(self, t):
        if not self.t: return None
        i = bisect.bisect_right(self.t, t) - 1
        return self.v[max(0, i)]


def load_controls(rid):
    p = run_dir(rid) / 'controls.jsonl'
    rows = []
    with open(p, encoding='utf8') as f:
        for line in f:
            try: rows.append(json.loads(line))
            except ValueError: pass
    return rows


class ImuLog:
    """IMU samples from the 50 Hz logger file within [t0, t1]."""
    def __init__(self, t0, t1, path=IMU_LOG):
        self.t = []; self.d = []
        with open(path, encoding='utf8') as f:
            for line in f:
                if not line.startswith('{"kind": "imu"'): continue
                i = line.find('"t": '); j = line.find(',', i)
                t = float(line[i + 5:j])
                if t < t0: continue
                if t > t1: break
                d = json.loads(line); self.t.append(t); self.d.append(d)

    def at(self, t):
        i = bisect.bisect_right(self.t, t + 1e-9) - 1
        return self.d[i] if i >= 0 else None

    def exact(self, t):
        i = bisect.bisect_left(self.t, t - 1e-6)
        if i < len(self.t) and abs(self.t[i] - t) < 1e-4: return self.d[i]
        return None


class SynthImu:
    """IMU stream synthesised from the logged per-frame values (for runs outside the IMU log's boot and for fuzz):
    pitch = dash_route.pitch_rel_deg + flat, gz = yaw_inner.gz / gyro_damp.gz (else 0), yaw integrated from gz;
    50 Hz samples on a grid, values held between frames. Only valid as a common input for old-vs-new differential runs."""
    def __init__(self, frames, flat=2.6):
        self.t = []; self.d = []
        if not frames: return
        fr = []
        for s in frames:
            v = s['vision']; p = s['proposal']; dr = v.get('dash_route') or {}
            pr = dr.get('pitch_rel_deg')
            gz = (p.get('yaw_inner') or {}).get('gz')
            if gz is None: gz = (p.get('gyro_damp') or {}).get('gz')
            fr.append((v['consumed_at'], 0. if pr is None else float(pr) + flat, 0. if gz is None else float(gz)))
        t = fr[0][0] - 3.; yaw = 0.; k = 0; end = fr[-1][0] + 1.
        while t <= end:
            while k + 1 < len(fr) and fr[k + 1][0] <= t: k += 1
            _, pitch, gz = fr[k]
            yaw += gz * .02
            self.t.append(round(t, 4)); self.d.append(dict(kind='imu', t=round(t, 4), pitch_deg=round(pitch, 2), roll_deg=0., yaw_deg=round(yaw, 3),
                                                            pitch_acc_deg=round(pitch, 2), acc_g=[0., 0., 1.], gyro_dps=[0., 0., round(gz, 3)], temp_c=25.))
            t += .02

    at = ImuLog.at
    exact = ImuLog.exact


def build_workdir(tag, live_pid_src, rid=None, upper_single=None, extra_cfg=None):
    """web/ = snapshot modules + fix33 upper_single / mask_dash + today's config + the live_pid under test.
    Nothing outside harness/_work is written. (b23 copy: WORK is b23/harness/_work; stage modules are copied too.)"""
    root = WORK / tag
    if root.exists(): shutil.rmtree(root)
    web = root / 'web'; web.mkdir(parents=True)
    for p in SNAP.iterdir():
        if p.suffix in ('.py', '.json') and p.name not in ('live_pid.py', 'MANIFEST.sha256'): shutil.copyfile(p, web / p.name)
    for n in ('upper_single.py', 'mask_dash.py', 'upper_single.json'): shutil.copyfile(FIX33 / n, web / n)
    (web / 'play_comp.json').write_text(json.dumps(PLAY_COMP), encoding='utf8')
    (web / 'crossing_pass.json').write_text(json.dumps(CROSSING_PASS), encoding='utf8')
    us = json.loads((FIX33 / 'upper_single.json').read_text(encoding='utf8'))
    on = UPPER_SINGLE_ON.get(rid, True) if upper_single is None else upper_single
    us['enabled'] = bool(on); (web / 'upper_single.json').write_text(json.dumps(us, indent=1), encoding='utf8')
    for n, v in (extra_cfg or {}).items(): (web / n).write_text(json.dumps(v), encoding='utf8')
    shutil.copyfile(live_pid_src, web / 'live_pid.py')
    for p in sibling_modules(live_pid_src): shutil.copyfile(p, web / p.name)   # b23: new modules / configs of the stage under test
    (root / 'home').mkdir(); (root / 'runs').mkdir()
    return root


def sibling_modules(live_pid_src):
    """b23: the .py / .json files next to the live_pid.py under test that belong to the deployable set (stage modules and their
    configs); generator / patch / test scripts and the manifest are not copied. b01/b1 has none (only live_pid.py)."""
    d = Path(live_pid_src).resolve().parent; out = []
    for p in sorted(d.iterdir()):
        if p.is_file() and p.suffix in ('.py', '.json') and p.name != 'live_pid.py' and not p.name.startswith(('patch_', 'gen_', 'test_', 'MANIFEST', 'passes_')) \
                and p.name not in HARNESS_OWNED: out.append(p)
    if d == B01:   # v2/upper: this folder carries only live_pid.py + upper_single.py; the other b5 stage modules come from b5/ (unchanged)
        have = {p.name for p in out}
        out = [p for p in sibling_modules(EXP / 'integration-20261112' / 'b5' / 'live_pid.py') if p.name not in have and p.name != 'upper_single.json'] + out
        out = [p for p in out if p.name != 'upper_single.on.json']
    return out


# b5: files the harness itself provides (fix33 module copies, today's car config, per-run upper_single enable): a deploy folder that also
# carries them (b5/) must not overwrite them, so both sides of a comparison see the same config. b5's own crossing_pass.json /
# bottom_map.json (new keys present, OFF) are checked against these separately and replayed with --cfg-new (explicit off values).
HARNESS_OWNED = ('crossing_pass.json', 'bottom_map.json', 'upper_single.json', 'mask_dash.py', 'play_comp.json')   # v2/upper: upper_single.py is taken from next to each side's live_pid.py (b5: 42e943b0 = fix33; v2/upper: r3)


def num(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(float(x))


def sent_logged(p):
    """sent (post play-comp) steering in a fix33 log: play_comp.steer_us (rounded 0.1) when present, else proposal.steer_us."""
    pc = p.get('play_comp')
    if isinstance(pc, dict) and num(pc.get('steer_us')): return float(pc['steer_us'])
    return float(p['steer_us'])
# crest-latch-20261116: all 11-16 runs; IMU files by boot (the 15:26 and 18:03 reboots; earlier copies mapped 702a/d122/853c/f70e to the
# 10:58 boot file, whose clock range overlaps but is another boot)
for _r in ('27a8', '54a1', '0e6a', '5bcf', '3e0c', 'b6a4', '7af0', '1e69', '85dc', 'f341', 'bfdb', 'a163', '8aba', '0631', 'e904', '702a', 'd122', '853c', 'f70e'):
    RUNS.setdefault(_r, '20261116')
IMU_FOR.update({r: EXP / 'imu-20261116' / 'imu_20261116_152615.jsonl' for r in ('702a', 'd122', '853c', 'f70e', '27a8', '54a1', '0e6a', '5bcf')})
IMU_FOR.update({r: EXP / 'imu-20261116' / 'imu_20261116_180354.jsonl' for r in ('3e0c', 'b6a4', '7af0', '1e69', '85dc')})
IMU_BOOT_RUNS = IMU_BOOT_RUNS + ('27a8', '54a1', '0e6a', '5bcf', '3e0c', 'b6a4', '7af0', '1e69', '85dc')
UPPER_SINGLE_ON.update({r: True for r in ('27a8', '54a1', '0e6a', '5bcf', '3e0c', 'b6a4', '7af0', '1e69', '85dc')})
# ---- c2-crossing-20261117: base = the car snapshot of 2026-11-17 (live_pid 166960bf, rate_indep on, pipeline2 switches, every car json);
# today's runs and their 50 Hz IMU file (07:21 boot). The 11-16 runs keep the per-boot IMU mapping above.
SNAP = EXP / 'orin-web-snapshot-20261117' / 'web'
FIX33 = SNAP
PLAY_COMP = json.loads((SNAP / 'play_comp.json').read_text(encoding='utf8'))
CROSSING_PASS = json.loads((SNAP / 'crossing_pass.json').read_text(encoding='utf8'))
for _r in ('7f36', 'f764', '8d97', '47f2', '2161', 'bc6a'):
    RUNS[_r] = '20261117'; IMU_FOR[_r] = EXP / 'imu-20261117' / 'imu_20261117_072144.jsonl'; UPPER_SINGLE_ON[_r] = True
IMU_BOOT_RUNS = IMU_BOOT_RUNS + ('7f36', 'f764', '8d97', '47f2', '2161', 'bc6a', 'd4e1')
RUNS.setdefault('d4e1', '20261116'); IMU_FOR['d4e1'] = EXP / 'imu-20261116' / 'imu_20261116_180354.jsonl'; UPPER_SINGLE_ON['d4e1'] = True
for _r in ('702a', 'd122', '853c', 'f70e', '27a8', '54a1', '0e6a', '5bcf'): IMU_FOR[_r] = EXP / 'imu-20261116' / 'imu_20261116_152615.jsonl'
WORK = Path(os.environ['DH_WORK']) if os.environ.get('DH_WORK') else WORK
# ---- c4-band-20261117: base = the reconstructed car state of 2026-11-17 noon (car-state-20261117b/web: live_pid 9650a81c, lap_ff 1d447e92,
# arc2 34a7ac93, c2 band / blind json on, rate_indep w_quantum 0, worker_overlap.json / model_stage.json (worker side; the replay's fake worker
# never reports overlap -> main loop as without it)). + d0f7 (IMU 09:43 boot), 586b / 18f1 (IMU 10:09 boot) of 11-17.
SNAP = EXP / 'car-state-20261117b' / 'web'
FIX33 = SNAP
PLAY_COMP = json.loads((SNAP / 'play_comp.json').read_text(encoding='utf8'))
CROSSING_PASS = json.loads((SNAP / 'crossing_pass.json').read_text(encoding='utf8'))
RUNS['d0f7'] = '20261117'; IMU_FOR['d0f7'] = EXP / 'imu-20261117' / 'imu_20261117_094313.jsonl'; UPPER_SINGLE_ON['d0f7'] = True
for _r in ('586b', '18f1'):
    RUNS[_r] = '20261117'; IMU_FOR[_r] = EXP / 'imu-20261117' / 'imu_20261117_100932.jsonl'; UPPER_SINGLE_ON[_r] = True
IMU_BOOT_RUNS = IMU_BOOT_RUNS + ('d0f7', '586b', '18f1')
# ---- blind2-20261117: + ef9a (11-17 14:30, IMU 12:59 boot file)
RUNS['ef9a'] = '20261117'; IMU_FOR['ef9a'] = EXP / 'imu-20261117' / 'imu_20261117_125930.jsonl'; UPPER_SINGLE_ON['ef9a'] = True
IMU_BOOT_RUNS = IMU_BOOT_RUNS + ('ef9a',)
# ---- inventory-20261117: base = the car snapshot after run-7afb (orin-web-snapshot-20261117c, flat folder: live_pid c9687c0c, arc2 1ed38926,
# play_comp 55/15, c2 band e_c 0.68, blind r1, ...). Runs of 11-17 afternoon in the 14:18-boot IMU file (named imu_20261117_125930).
SNAP = EXP / 'orin-web-snapshot-20261117c'
FIX33 = SNAP
PLAY_COMP = json.loads((SNAP / 'play_comp.json').read_text(encoding='utf8'))
CROSSING_PASS = json.loads((SNAP / 'crossing_pass.json').read_text(encoding='utf8'))
for _r in ('ef9a', '527b', '58df', '2626', '7afb'):
    RUNS[_r] = '20261117'; IMU_FOR[_r] = EXP / 'imu-20261117' / 'imu_20261117_125930.jsonl'; UPPER_SINGLE_ON[_r] = True
IMU_BOOT_RUNS = IMU_BOOT_RUNS + ('527b', '58df', '2626', '7afb')
# ---- throttle-free-20261118: base = the car snapshot after the evening of 11-17 (orin-web-snapshot-20261117d: live_pid 6e460ceb = merge-20261118c,
# upper_single one, blind r1, speed_adapt 91f4b01c, brake_zones ac3f12cf). Evening runs in the 21:31-boot IMU file (named imu_20261117_161255).
SNAP = EXP / 'orin-web-snapshot-20261117d'
FIX33 = SNAP
PLAY_COMP = json.loads((SNAP / 'play_comp.json').read_text(encoding='utf8'))
CROSSING_PASS = json.loads((SNAP / 'crossing_pass.json').read_text(encoding='utf8'))
for _r in ('8a43', '4a76', 'ab01', 'c4fe'):
    RUNS[_r] = '20261117'; IMU_FOR[_r] = EXP / 'imu-20261117' / 'imu_20261117_161255.jsonl'; UPPER_SINGLE_ON[_r] = True
IMU_BOOT_RUNS = IMU_BOOT_RUNS + ('8a43', '4a76', 'ab01', 'c4fe')
WORK = Path(os.environ['DH_WORK']) if os.environ.get('DH_WORK') else WORK

# ---- crest-blend-20261118: base = the car now (trees/old built by tools/make_trees.py = snapshot d + today's on-car changes).
# Today's runs: 6c1e / 7392 / 0f91 in the 17:29-boot IMU file (named imu_20261118_084154); d2af / d9a4 / 5fe4 / 973e / d0a4 in the 18:26-boot file.
SNAP = EXP / 'orin-web-snapshot-20261118'   # landing-20261119: was crest-blend trees/old
FIX33 = SNAP
PLAY_COMP = json.loads((SNAP / 'play_comp.json').read_text(encoding='utf8'))
CROSSING_PASS = json.loads((SNAP / 'crossing_pass.json').read_text(encoding='utf8'))
for _r in ('6c1e', '7392', '0f91'):
    RUNS[_r] = '20261118'; IMU_FOR[_r] = EXP / 'imu-20261118' / 'imu_20261118_084154.jsonl'; UPPER_SINGLE_ON[_r] = True
for _r in ('d2af', 'd9a4', '5fe4', '973e', 'd0a4'):
    RUNS[_r] = '20261118'; IMU_FOR[_r] = EXP / 'imu-20261118' / 'imu_20261118_182505.jsonl'; UPPER_SINGLE_ON[_r] = True
IMU_BOOT_RUNS = IMU_BOOT_RUNS + ('6c1e', '7392', '0f91', 'd2af', 'd9a4', '5fe4', '973e', 'd0a4')
WORK = Path(os.environ['DH_WORK']) if os.environ.get('DH_WORK') else WORK

# ---- landing-20261119: base = the car snapshot after run-c212 (orin-web-snapshot-20261118, flat folder; live_pid 6e460ceb, arc2.py f722ad5c,
# lap_ff.py 937c5674, centre 1510 + single_centre on). + 7a70 (07:xx boot file imu_20261117_223410) and c212 (18:26 boot file).
SNAP = EXP / 'orin-web-snapshot-20261118'
FIX33 = SNAP
PLAY_COMP = json.loads((SNAP / 'play_comp.json').read_text(encoding='utf8'))
CROSSING_PASS = json.loads((SNAP / 'crossing_pass.json').read_text(encoding='utf8'))
RUNS['7a70'] = '20261118'; IMU_FOR['7a70'] = EXP / 'imu-20261118' / 'imu_20261117_223410.jsonl'; UPPER_SINGLE_ON['7a70'] = True
RUNS['c212'] = '20261118'; IMU_FOR['c212'] = EXP / 'imu-20261118' / 'imu_20261118_182505.jsonl'; UPPER_SINGLE_ON['c212'] = True
IMU_BOOT_RUNS = IMU_BOOT_RUNS + ('7a70', 'c212')
WORK = Path(os.environ['DH_WORK']) if os.environ.get('DH_WORK') else WORK

# ---- arc-lat-20261119: same base snapshot (orin-web-snapshot-20261118); replay sides carry the car-now modules / json (replay_sides/old, new).
# Proof runs (with controls.jsonl + 50 Hz IMU): c212, d0a4, 973e, 5fe4 (11-18). The 11-19 runs have no controls.jsonl (open-loop evaluation offline).
WORK = Path(os.environ['DH_WORK']) if os.environ.get('DH_WORK') else WORK

# ---- arc-lat2-20261119: base = the car snapshot of 11-19 12:55 (orin-web-snapshot-20261119, flat; live_pid 6e460ceb, merge-19b, dash_route f3fb9a08,
# every car json); replay sides overlay the car's current arc-lat control files (old: live_pid 8cd474cb + arc_lat d9361d4c; new: + candidate arc_lat).
SNAP = EXP / 'orin-web-snapshot-20261119'
FIX33 = SNAP
PLAY_COMP = json.loads((SNAP / 'play_comp.json').read_text(encoding='utf8'))
CROSSING_PASS = json.loads((SNAP / 'crossing_pass.json').read_text(encoding='utf8'))

# ---- c3-tail-20261119: base = the car snapshot after run-1f77 (orin-web-snapshot-20261119b, flat, 183 files: live_pid 8cd474cb, arc2.py 0bc66fcf,
# arc_lat.py c53ff31e, every car json). 11-19 runs with controls.jsonl + 50 Hz IMU of their boot.
SNAP = EXP / 'orin-web-snapshot-20261119b'
FIX33 = SNAP
PLAY_COMP = json.loads((SNAP / 'play_comp.json').read_text(encoding='utf8'))
CROSSING_PASS = json.loads((SNAP / 'crossing_pass.json').read_text(encoding='utf8'))
for _r, _f in (('8545', 'imu_20261118_204459.jsonl'), ('9b2e', 'imu_20261118_204459.jsonl'),
               ('cdb0', 'imu_20261119_084249.jsonl'), ('baa7', 'imu_20261119_084249.jsonl'), ('ef8c', 'imu_20261119_084249.jsonl'), ('a674', 'imu_20261119_084249.jsonl'),
               ('e18d', 'imu_20261119_125906.jsonl'), ('66c0', 'imu_20261119_125906.jsonl'), ('1f77', 'imu_20261119_125906.jsonl')):
    RUNS[_r] = '20261119'; IMU_FOR[_r] = EXP / 'imu-20261119' / _f; UPPER_SINGLE_ON[_r] = True
IMU_BOOT_RUNS = IMU_BOOT_RUNS + ('8545', '9b2e', 'cdb0', 'baa7', 'ef8c', 'a674', 'e18d', '66c0', '1f77')
B01 = Path(__file__).resolve().parents[1]
WORK = Path(os.environ['DH_WORK']) if os.environ.get('DH_WORK') else WORK

# ---- landing4-20261122: base = per-run copy of orin-web-snapshot-20261121c + the two config files that differed on that run
# (imu_boost.json flat_pitch_deg, lap_speed.json), given by env L4_BASE. Runs without controls.jsonl: MCU status synthesised from the
# logged proposal.inputs (SynthVehicle). IMU: RecImu = the logged absolute IMU heading where it was logged (heading_hold.yaw_deg on the upper
# deck, lap_ff4.band ref - turned after the landing), gz-integrated in between; pitch from dash_route.pitch_rel_deg + the run's flat pitch.
SNAP = Path(os.environ['L4_BASE']) if os.environ.get('L4_BASE') else SNAP
FIX33 = SNAP
PLAY_COMP = json.loads((SNAP / 'play_comp.json').read_text(encoding='utf8'))
CROSSING_PASS = json.loads((SNAP / 'crossing_pass.json').read_text(encoding='utf8'))
for _r, _d in (('e1b4', '20261121'), ('65ea', '20261122'), ('3197', '20261122'), ('8350', '20261122'), ('46c9', '20261122'), ('d60e', '20261122'), ('84f2', '20261122'), ('18fc', '20261122'), ('ff94', '20261122'), ('ccf1', '20261122'), ('4bbe', '20261122'), ('1eab', '20261122'), ('e0cd', '20261122'), ('6f61', '20261122'), ('c6f3', '20261122'), ('de46', '20261122'), ('5aa9', '20261122'), ('8019', '20261122'), ('676f', '20261122'), ('e757', '20261122'), ('370e', '20261122'), ('8ae8', '20261122'), ('b6e9', '20261122'), ('bcb9', '20261122'), ('6ba6', '20261122'), ('f126', '20261122')):   # c1exit: + its runs   # c3c4-20261122: + the five runs of this task
    RUNS[_r] = _d; UPPER_SINGLE_ON[_r] = True
WORK = Path(os.environ['DH_WORK']) if os.environ.get('DH_WORK') else WORK


class SynthVehicle:
    """MCU status per frame from proposal.inputs (veh_ms, enc_b, steer_cmd); a status becomes visible at the consumed_at of the first
    frame that logged it. enc_a (right wheel, not logged) mirrors enc_b. Only for old-vs-new differential runs and open-loop evaluation."""
    def __init__(self, frames):
        self.t = []; self.v = []; last = None
        for s in frames:
            i = s['proposal'].get('inputs') or {}
            ms = i.get('veh_ms'); e = i.get('enc_b')
            if ms is None or e is None or ms == last: continue
            last = ms
            self.t.append(float(s['vision']['consumed_at']) - 1e-6)
            self.v.append(dict(type='status', ms=ms, enc_sample_ms=ms - 185, mode='ORIN', state='MANUAL', generation=1, enc_enabled=True,
                               enc_b=dict(forward=int(e), reverse=0, seen=True), enc_a=dict(forward=0, reverse=int(e), seen=True),
                               steer_us_command=i.get('steer_cmd'), pca_ok=True, i2c_errors=0, steer_output_enabled=True))

    def at(self, t):
        if not self.t: return None
        i = bisect.bisect_right(self.t, t) - 1
        return self.v[max(0, i)]


class RecImu:
    def __init__(self, frames, flat):
        self.t = []; self.d = []
        if not frames: return
        fr = []; last = None; gzp = 0.; run_ = []
        for s in frames:
            v = s['vision']; p = s['proposal']; tc = float(v['consumed_at'])
            pr = (v.get('dash_route') or {}).get('pitch_rel_deg')
            gz = None
            for a, b in (('inputs', 'gz'), ('yaw_inner', 'gz'), ('downhill_gz', 'gz'), ('down_straight', 'gz'), ('bottom_owner', 'gz'), ('bottom_guide', 'gz')):
                x = (p.get(a) or {}).get(b) if isinstance(p.get(a), dict) else None
                if x is not None: gz = float(x); break
            if gz is None: gz = gzp
            gzp = gz
            y = None
            hh = p.get('heading_hold') if isinstance(p.get('heading_hold'), dict) else {}
            bd = (p.get('lap_ff4') or {}).get('band') if isinstance(p.get('lap_ff4'), dict) else None
            if hh.get('yaw_deg') is not None: y = float(hh['yaw_deg'])
            elif isinstance(bd, dict) and bd.get('ref') is not None and bd.get('turned_deg') is not None: y = float(bd['ref']) - float(bd['turned_deg'])
            anch = y is not None
            if y is None: y = 0. if last is None else last[1] + (gz if abs(gz) < 200. else 0.) * (tc - last[0])
            if anch and run_ and len(run_) >= 2:   # an integrated stretch ends at a logged heading: spread the closing error linearly over the stretch (continuous heading)
                pred = last[1] + (gz if abs(gz) < 200. else 0.) * (tc - last[0]); er = y - pred; t0_ = fr[run_[0]][0]; span = max(1e-6, tc - t0_)
                for k_ in run_: fr[k_] = fr[k_][:3] + (fr[k_][3] + er * (fr[k_][0] - t0_) / span,)
            if anch: run_ = []
            else: run_.append(len(fr))
            last = (tc, y)
            fr.append((tc, 0. if pr is None else float(pr) + flat, gz, y))
        t = fr[0][0] - 3.; k = 0; end = fr[-1][0] + 1.
        while t <= end:
            while k + 1 < len(fr) and fr[k + 1][0] <= t: k += 1
            _, pitch, gz, y = fr[k]
            if k + 1 < len(fr) and t >= fr[k][0]:   # heading linear between the frames
                t0, t1 = fr[k][0], fr[k + 1][0]; y = y + (fr[k + 1][3] - y) * (t - t0) / max(1e-6, t1 - t0)
            self.t.append(round(t, 4)); self.d.append(dict(kind='imu', t=round(t, 4), pitch_deg=round(pitch, 2), roll_deg=0., yaw_deg=round(y, 3),
                                                            pitch_acc_deg=round(pitch, 2), acc_g=[0., 0., 1.], gyro_dps=[0., 0., round(gz, 3)], temp_c=25.))
            t += .02

    at = ImuLog.at
    exact = ImuLog.exact
