"""proof C1: old live_pid vs new live_pid (+ land4.py, no land4.json) on the same recorded input stream.
  python cmp_off.py <out dir with <run>.old.jsonl / <run>.new.jsonl> [fidelity]
Identical = same number of frames, and per frame the same dispatched command (target_us, pwm, pwm_right, pwm_left), the same external read
order (clock / IMU / MCU status), the same reason and the same full proposal dict."""
import json, sys, os, statistics as S
from pathlib import Path

D = Path(sys.argv[1]); EXP = Path(os.environ.get('L4_EXP', r'D:\OneDrive\Desktop\自走車\Orin_Interface_v1\laptop_workbench\data\experiments'))
RUNS = (('d60e', '20261122'), ('ccf1', '20261122'), ('4bbe', '20261122'))


def load(p):
    with open(p, encoding='utf8') as f:
        meta = json.loads(f.readline()); rows = [json.loads(l) for l in f]
    return meta, rows


tot = same = 0; allok = True
for run, day in RUNS:
    po, pn = D / ('%s.old.jsonl' % run), D / ('%s.new.jsonl' % run)
    if not (po.exists() and pn.exists()): continue
    mo, ro = load(po); mn, rn = load(pn)
    n = min(len(ro), len(rn)); cmd = rd = prop = 0
    for a, b in zip(ro, rn):
        c = all(a.get(k) == b.get(k) for k in ('frame_seq', 'sent_steer', 'pwm', 'pwm_right', 'pwm_left', 'reason', 'flags'))
        cmd += c; rd += a.get('ev') == b.get('ev'); prop += a.get('proposal') == b.get('proposal')
    ok = len(ro) == len(rn) and cmd == n and rd == n and prop == n and mo.get('stops') == mn.get('stops') and mo.get('loop_error') == mn.get('loop_error')
    allok = allok and ok; tot += len(ro); same += cmd if ok else 0
    print('%s  frames in %5d | replayed old %5d new %5d | commands identical %5d | read order identical %5d | proposals identical %5d | stops %s / %s | loop_err %s / %s | %s' % (
        run, mo['frames_in'], len(ro), len(rn), cmd, rd, prop, mo.get('stops'), mn.get('stops'), mo.get('loop_error'), mn.get('loop_error'), 'IDENTICAL' if ok else 'DIFFERENT'))
    if len(sys.argv) > 2:   # fidelity of the replay (old side) against the car log, frames matched by frame_seq
        log = {}
        with open(EXP / ('run-%s-%s' % (run, day)) / 'pid.jsonl', encoding='utf8') as f:
            for l in f:
                if '"kind": "sample"' not in l[:40]: continue
                i = l.find('"proposal"'); d = json.loads(l)
                log[(d['vision'] or {}).get('frame_seq')] = d['proposal']
        ds = []; dp = 0; m = 0; rs = 0
        for a in ro:
            p = log.get(a['frame_seq'])
            if p is None or p.get('steer_us') is None or a['proposal'].get('steer_us') is None: continue
            m += 1; ds.append(abs(float(p['steer_us']) - float(a['proposal']['steer_us']))); dp += abs(float(p.get('pwm') or 0) - float(a['proposal'].get('pwm') or 0)) < 0.01; rs += p.get('reason') == a['proposal'].get('reason')
        ds.sort()
        print('      fidelity vs car log: matched %d of %d | |steer diff| median %.1f P90 %.1f P99 %.1f max %.0f us, <=1 us on %.1f%% | pwm identical %.1f%% | reason identical %.1f%%' % (
            m, len(ro), ds[len(ds) // 2], ds[int(.9 * len(ds))], ds[int(.99 * len(ds))], ds[-1], 100. * sum(1 for x in ds if x <= 1.) / len(ds), 100. * dp / m, 100. * rs / m))
print('TOTAL frames %d | identical %d | all runs identical: %s' % (tot, same, allok))
