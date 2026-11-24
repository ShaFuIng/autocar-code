"""chatter-20261122 tests for candidate/downhill_speed.py (speed-meter-mcu-r1). Offline; standard library only.
python test_speed_meter.py [PROOF_OUT]
1. key absent (no web/speed_meter.json), unreadable json, mode != 'mcu': SpeedMeter.update / rate / k identical (exact floats, None
   included) to the snapshot-20261122c module on (a) the status sequences of 9 recorded runs (inputs.t_send, veh_ms, enc_b) and
   (b) random sequences with missing fields, repeated / backwards / bool ms, None vehicle.
2. mode 'mcu': no one-frame spikes on the recorded runs; equals an independent re-implementation (tools/estimate.v_mcu) after
   warm-up; stale MCU status -> old rate (decays); MCU clock backwards -> entries cleared; missing ms -> old rate; stopped car -> 0."""
import sys, os, json, math, random, shutil, tempfile, importlib.util
sys.dont_write_bytecode = True   # never write __pycache__ into the snapshot folder
from pathlib import Path

HERE = Path(__file__).resolve().parent
CH = HERE.parent
EXP = CH.parents[1]
SNAP = EXP / 'orin-web-snapshot-20261122c'
SCR = Path(r'C:/Users/User/AppData/Local/Temp/claude/1eeee7db7bd1a8b1/02d05eda-955f-4b7f-90df-1bfa92030e70/scratchpad/cx')
RUNS = ['d60e', 'ccf1', '6f61', 'c6f3', '7658', '2728', '9af6', 'e69d', '54cb']
sys.path.insert(0, str(SNAP))          # rate_indep for both modules
sys.path.insert(0, str(CH / 'tools'))
N = dict(ok=0, fail=0); LOG = []


def check(cond, msg):
    if cond: N['ok'] += 1
    else: N['fail'] += 1; LOG.append('FAIL ' + msg)


def load_mod(src, name, cfg=None):
    d = Path(tempfile.mkdtemp(prefix='sm_')); shutil.copy(src, d / 'downhill_speed.py')
    if cfg is not None: (d / 'speed_meter.json').write_text(cfg, encoding='utf8')
    spec = importlib.util.spec_from_file_location(name, d / 'downhill_speed.py'); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    return m, d


def recorded(run):
    seq = []
    for l in open(SCR / f'{run}.jsonl', encoding='utf8'):
        p = json.loads(l)['p']; inp = p.get('inputs') or {}
        now = inp.get('t_send') if isinstance(inp.get('t_send'), (int, float)) else p.get('t')
        if not isinstance(now, (int, float)): continue
        if inp.get('enc_b') is None: seq.append((now, {})); continue
        seq.append((now, {'ms': inp.get('veh_ms'), 'enc_b': {'forward': inp['enc_b'], 'reverse': 0}}))
    return seq


def fuzz(seed, n=3000):
    r = random.Random(seed); t = 100.; ms = 5000; c = 0; out = []
    for i in range(n):
        t += r.choice([0.033, 0.034, 0.035, 0.07, 0.0])
        if r.random() < 0.18: ms += r.choice([200, 200, 200, 190, 210, 400, -200, 3000])
        c += r.randint(0, 60)
        k = r.random()
        if k < 0.03: out.append((t, None))
        elif k < 0.06: out.append((t, {}))
        elif k < 0.08: out.append((t, {'enc_b': {'forward': c, 'reverse': 3}}))
        elif k < 0.09: out.append((t, {'ms': True, 'enc_b': {'forward': c, 'reverse': 0}}))
        elif k < 0.10: out.append((t, {'ms': 'x', 'enc_b': {'forward': c}}))
        else: out.append((t, {'ms': ms, 'enc_b': {'forward': c, 'reverse': r.randint(0, 2)}}))
    return out


def same(a, b):
    if a is None or b is None: return a is b
    return a == b or (math.isnan(a) and math.isnan(b))


def compare(old_m, new_m, seq):
    o = old_m.SpeedMeter(); n = new_m.SpeedMeter(); diff = 0
    for now, veh in seq:
        a = o.update(now, veh); b = n.update(now, veh)
        ka = o.k(a, right=bool(int(now * 1000) % 2)); kb = n.k(b, right=bool(int(now * 1000) % 2))
        if not (same(a, b) and same(ka, kb) and same(o.rate(), n.rate())): diff += 1
    return len(seq), diff


def main():
    proof = []
    old, _ = load_mod(SNAP / 'downhill_speed.py', 'ds_old')
    new0, d0 = load_mod(CH / 'candidate' / 'downhill_speed.py', 'ds_new0')                 # no json
    newbad, _ = load_mod(CH / 'candidate' / 'downhill_speed.py', 'ds_newbad', '{bad json')
    newfr, _ = load_mod(CH / 'candidate' / 'downhill_speed.py', 'ds_newfr', '{"mode": "frame"}')
    newmcu, _ = load_mod(CH / 'candidate' / 'downhill_speed.py', 'ds_newmcu', '{"mode": "mcu"}')
    check(new0.SpeedMeter().mcu is None and newbad.SpeedMeter().mcu is None and newfr.SpeedMeter().mcu is None, 'config off -> mcu None')
    check(newmcu.SpeedMeter().mcu == dict(win_s=0.6, stale_s=0.45), 'mcu config defaults')
    check(new0.REVISION == old.REVISION, 'REVISION string unchanged (logged as DSPEED_REVISION)')
    for k in ('REF_SPEED', 'K_MIN', 'WIN_S', 'ASSIST_US', 'HOLD_RIGHT_US', 'K_RISE', 'GZ_MAX', 'K_GZ', 'GZ_CAP_US', 'HOLD_US', 'ENTRY_DELAY_S', 'REF_SPEED_RIGHT', 'K_MIN_RIGHT'):
        check(getattr(new0, k) == getattr(old, k), f'constant {k} unchanged')
    for g in (-80., -25., 0., 30., 60.): check(new0.gz_term(g) == old.gz_term(g), 'gz_term unchanged')
    TOT = DIF = 0
    for name, mod in (('absent', new0), ('bad_json', newbad), ('mode_frame', newfr)):
        tt = dd = 0
        for run in RUNS:
            n, d = compare(old, mod, recorded(run)); tt += n; dd += d
            if name == 'absent': proof.append(f'  {run}: frames {n} different {d}')
        for s in range(12):
            n, d = compare(old, mod, fuzz(s)); tt += n; dd += d
        check(dd == 0, f'{name}: identical to old module ({dd} different of {tt})')
        proof.append(f'{name}: frames {tt} identical {tt - dd} different {dd}')
        TOT += tt; DIF += dd
    proof.insert(0, f'TOTAL frames {TOT} identical {TOT - DIF} different {DIF} all_identical {DIF == 0}  (9 recorded runs + 12 fuzz sequences, x3 configs without mode mcu)')
    # 2. mcu mode
    from estimate import v_mcu
    from common import load, num
    M = 2.22e-4
    for run in ('e69d', '9af6', 'd60e'):
        seq = recorded(run); m = newmcu.SpeedMeter(); o = old.SpeedMeter(); V = []; VO = []
        for now, veh in seq: V.append(m.update(now, veh)); VO.append(o.update(now, veh))
        def spikes(X):
            return sum(1 for i in range(1, len(X) - 1) if all(x is not None for x in X[i - 1:i + 2]) and (X[i] - max(X[i - 1], X[i + 1])) * M > 0.10)
        so, sn = spikes(VO), spikes(V)
        check(sn == 0, f'{run}: mcu mode spikes {sn}'); check(so > 100, f'{run}: old module reproduces the spikes ({so})')
        proof.append(f'mcu mode {run}: one-frame spikes > 0.10 m/s old {so} -> mcu {sn}')
        # independent re-implementation on the same sequence (same rule as tools/estimate.v_mcu): one entry per new ms, WIN 0.6 s
        h = []; last = None; ref = []
        for now, veh in seq:
            ms = (veh or {}).get('ms'); eb = (veh or {}).get('enc_b') or {}
            if isinstance(ms, (int, float)) and not isinstance(ms, bool) and 'forward' in eb and ms != last:
                last = ms; tt = ms / 1000.; h.append((tt, float(eb['forward']))); h = [x for x in h if tt - x[0] <= 1.]
            if len(h) < 2: ref.append(None); continue
            t1, c1 = h[-1]; od = [x for x in h if t1 - x[0] >= 0.6]; t0, c0 = od[-1] if od else h[0]
            ref.append(None if t1 - t0 < .05 else abs(c1 - c0) / (t1 - t0))
        d = [abs(V[i] - ref[i]) for i in range(len(V)) if V[i] is not None and ref[i] is not None]
        nd = sum(1 for x in d if x > 1e-6)
        check(nd <= len(d) * 0.002, f'{run}: mcu mode equals the re-implementation ({nd} of {len(d)} differ; allowed: stale fallback frames)')
        proof.append(f'mcu mode {run}: equals re-implementation on {len(d) - nd} of {len(d)} frames (rest = stale-status fallback to the old rate)')
    # stale -> old rate (decays)
    m = newmcu.SpeedMeter(); o = old.SpeedMeter(); t = 0.; c = 0
    for i in range(60):
        t += 0.033; ms = 1000 + 200 * int(t / 0.2)
        if i % 6 == 0: c += 1000
        m.update(t, {'ms': ms, 'enc_b': {'forward': c, 'reverse': 0}}); o.update(t, {'ms': ms, 'enc_b': {'forward': c, 'reverse': 0}})
    r_live = m.rate(); check(r_live is not None and r_live > 0, 'mcu live rate > 0')
    for i in range(20):   # status frozen (same ms, same count) for 0.66 s
        t += 0.033; a = m.update(t, {'ms': ms, 'enc_b': {'forward': c, 'reverse': 0}}); b = o.update(t, {'ms': ms, 'enc_b': {'forward': c, 'reverse': 0}})
    check(a == b, 'stale status -> old (decaying) rate')
    # MCU clock backwards -> cleared
    m.update(t + 0.03, {'ms': 10, 'enc_b': {'forward': c + 5, 'reverse': 0}})
    check(len(m.mh) == 1, 'MCU clock backwards clears the entries')
    # missing ms -> old
    m2 = newmcu.SpeedMeter(); o2 = old.SpeedMeter(); eq = True
    for i in range(50):
        veh = {'enc_b': {'forward': i * 37, 'reverse': 0}}; eq &= same(m2.update(i * 0.033, veh), o2.update(i * 0.033, veh))
    check(eq, 'missing ms -> old rate')
    # stopped car -> 0
    m3 = newmcu.SpeedMeter()
    for i in range(40): r = m3.update(i * 0.033, {'ms': 200 * (i // 6), 'enc_b': {'forward': 500, 'reverse': 0}})
    check(r == 0, 'stopped car -> 0')
    # constant speed with frame jitter -> constant output (no sawtooth)
    m4 = newmcu.SpeedMeter(); outs = []
    rr = random.Random(3); t = 0.
    for i in range(200):
        t += rr.choice([0.03, 0.035, 0.04]); ms = int(t / 0.2) * 200
        outs.append(m4.update(t, {'ms': ms, 'enc_b': {'forward': ms * 5, 'reverse': 0}}))
    tail = [x for x in outs[40:] if x is not None]
    check(max(tail) - min(tail) < 1e-6, f'constant speed -> constant mcu rate (spread {max(tail) - min(tail)})')
    out = '\n'.join(proof) + '\n'
    print(out); print(f'tests: {N["ok"]} passed, {N["fail"]} failed'); print('\n'.join(LOG))
    if len(sys.argv) > 1: Path(sys.argv[1]).write_text(out + f'\ntests: {N["ok"]} passed, {N["fail"]} failed\n' + '\n'.join(LOG) + '\n', encoding='utf8')
    return N['fail'] == 0


if __name__ == '__main__':
    sys.exit(0 if main() else 1)
