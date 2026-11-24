"""c3front B: the arc2 curve-3 floor (lap_ff.LapFF4 'c3', arc2.json c3) recomputed per recorded frame with another kappa_table.

Open loop, formula level (not a live_pid replay): the recorded proposal.arc2_c3 info of every frame gives the layer's own inputs and
gates of that frame - x_m (metres after C3s), a (= ramp * end/cancel fade * heading guard * camera yield, 3 decimals), g (gain, 4
decimals), pid_steer_us (the command entering the layer), eligible. With them LapFF4._step0 is
    target = clip(c_eff + frac * kappa(x) / g, centre, min(cap_us, max_us));  floor = pid + a * max(0, target - pid)
    out = floor (eligible) ; out >= last - max_drop_us * w (rate limit of the release, w = frame dt / 0.105 s, rate_indep.json) ; last = out if out > pid
c_eff = centre (single_centre.json). The 'top' of the tail (x >= 4.95 m) is not recomputed: frames with a top record keep the recorded
value. First the car's own table is run through the same code and compared with the recorded steer_us (fidelity), then the candidate.
usage: python floor.py <car arc2.json> <candidate arc2.json> [...]"""
import collections, json, sys
from pathlib import Path
import numpy as np
EXP = Path(__file__).resolve().parents[3]
RUNS = ('run-e1b4-20261121', 'run-b93e-20261121', 'run-9934-20261121')
GAIN = -0.163   # deg/m per us of proposal steer in the front part (tools/align.py, r -0.89): only for the 'estimated curvature' column


def interp(table, x):
    t = sorted((float(a), float(b)) for a, b in table)
    if x <= t[0][0]: return t[0][1]
    if x >= t[-1][0]: return t[-1][1]
    for (x0, y0), (x1, y1) in zip(t, t[1:]):
        if x0 <= x <= x1: return y0 + (y1 - y0) * (x - x0) / max(1e-9, x1 - x0)
    return t[-1][1]


def frames(run):
    out = []
    for l in open(str(EXP / run / 'pid.jsonl'), encoding='utf8'):
        if '"kind": "sample"' not in l[:40]: continue
        p = json.loads(l).get('proposal') or {}
        if p.get('t') is None: continue
        out.append(p)
    return out


def emulate(F, c3, centre_default=1475., hi=2050.):
    """per frame: (new layer output or None when the frame has no usable c3 record, recorded layer output, info)"""
    res = []; last = None; tprev = None
    for p in F:
        inf = p.get('arc2_c3'); t = p['t']; dt = (t - tprev) if tprev is not None else 0.033; tprev = t
        cen = float((p.get('single_centre') or {}).get('center_us') or centre_default)
        if not isinstance(inf, dict) or inf.get('x_m') is None or inf.get('pid_steer_us') is None or inf.get('a') is None:
            # TAIL / END / NO_YAW / no record: the release rate limit may still run (state TAIL); not modelled -> keep the recorded frame
            last = None; res.append((None, None, inf)); continue
        st = float(inf['pid_steer_us']); a = float(inf['a']); g = float(inf['g']); x = float(inf['x_m']); el = bool(inf.get('eligible'))
        kap = interp(c3['kappa_table'], max(0., x))
        tgt = max(cen, min(float(c3['cap_us']), hi, cen + float(c3['frac']) * kap / g))
        out = st + a * max(0., tgt - st) if el else st
        md = c3.get('max_drop_us')
        if md is not None and el and last is not None:
            w = min(2., max(0., dt) / 0.105); out = max(out, last - float(md) * w)
        out = max(780., min(hi, out)); last = out if (el and out > st) else None
        rec = float(inf['steer_us']) if 'top' not in inf else None   # with a top record steer_us is the capped value
        res.append((out, rec, dict(x=x, a=a, st=st, tgt=tgt, kap=kap, el=el, top='top' in inf, lap=(p.get('lap_pos') or {}).get('lap'))))
    return res


def main():
    car = json.loads(Path(sys.argv[1]).read_text(encoding='utf8'))['c3']
    data = {r: frames(r) for r in RUNS}
    base = {r: emulate(data[r], car) for r in RUNS}
    d = np.array([o - rec for r in RUNS for (o, rec, i) in base[r] if o is not None and rec is not None])
    print('FIDELITY (car table through this code vs the recorded arc2_c3.steer_us, frames without a top record): n %d | identical within 0.15 us: %.1f %% | within 2 us: %.1f %% | |diff| P99 %.1f us, max %.1f us' % (
        len(d), 100 * np.mean(np.abs(d) <= 0.15), 100 * np.mean(np.abs(d) <= 2.), np.percentile(np.abs(d), 99), np.abs(d).max()))
    for cp in sys.argv[2:]:
        cand = json.loads(Path(cp).read_text(encoding='utf8'))['c3']
        print('=' * 110); print('candidate', Path(cp).name)
        print('  kappa_table old:', json.dumps([k for x, k in car['kappa_table'] if x < 4.2])); print('  kappa_table new:', json.dumps([k for x, k in cand['kappa_table'] if x < 4.2]))
        other = {k: v for k, v in cand.items() if k not in ('kappa_table', 'note_c3front_20261121') and car.get(k) != v}; print('  other c3 keys changed:', other or 'none')
        B = collections.defaultdict(list); tot = collections.Counter(); per = collections.defaultdict(lambda: [0, 0, 0., 0.])
        for r in RUNS:
            new = emulate(data[r], cand); lastlap = max(((p.get('lap_pos') or {}).get('lap') or 0) for p in data[r])
            for (oa, rec, ia), (ob, _, ib) in zip(base[r], new):
                tot['frames'] += 1
                if oa is None: continue
                tot['c3'] += 1; x = ia['x']; df = ob - oa
                if abs(df) > 0.05:
                    tot['changed'] += 1
                    if not 0.17 <= x <= 3.93: tot['outside'] += 1   # the changed table entries are x 0.425..3.675; linear interpolation reaches their neighbours 0.175 and 3.925
                    if ia['top']: tot['top_frames_changed'] += 1
                if ia['lap'] is not None and 1 <= ia['lap'] < lastlap and -0.1 <= x <= 5.0:
                    B[int(np.floor(x / 0.25))].append((ia['st'], ia['tgt'], ib['tgt'], oa, ob, ia['a']))
                    k = (r[4:8], ia['lap']); per[k][0] += 1; per[k][1] += abs(df) > 0.05; per[k][2] = max(per[k][2], df); per[k][3] = min(per[k][3], df)
        print('  TOTAL frames %d | frames with a c3 record %d | command changed on %d | changed outside x 0.17..3.93 m: %d | changed frames that also have a tail top record: %d' % (
            tot['frames'], tot['c3'], tot['changed'], tot['outside'], tot['top_frames_changed']))
        print('  per 0.25 m of x (complete laps): PID command entering the layer | floor target old -> new | layer output old -> new (us above the centre 1475), mean change, floor binds old / new (%), estimated curvature change deg/m')
        for b in sorted(B):
            X = np.array(B[b]); c = 1475.
            bo = np.mean(X[:, 3] > X[:, 0] + 0.05) * 100; bn = np.mean(X[:, 4] > X[:, 0] + 0.05) * 100; dm = np.mean(X[:, 4] - X[:, 3])
            print('    x %+.2f  pid %+4.0f | target %+4.0f -> %+4.0f | out %+4.0f -> %+4.0f  change mean %+5.1f (min %+4.0f max %+4.0f) | binds %3.0f%% -> %3.0f%% | curvature %+.1f' % (
                b * 0.25, np.median(X[:, 0]) - c, np.median(X[:, 1]) - c, np.median(X[:, 2]) - c, np.median(X[:, 3]) - c, np.median(X[:, 4]) - c, dm, (X[:, 4] - X[:, 3]).min(), (X[:, 4] - X[:, 3]).max(), bo, bn, GAIN * dm))
        print('  per lap: frames in x -0.1..5.0, changed, largest raise / largest reduction (us):', ' '.join('%s-L%d %d/%d %+.0f/%+.0f' % (k[0], k[1], v[0], v[1], v[2], v[3]) for k, v in sorted(per.items())))
        # swing of the layer output over the front part, old vs new (how even the command becomes), per lap
        sw = collections.defaultdict(lambda: [[], []])
        for r in RUNS:
            new = emulate(data[r], cand)
            for (oa, rec, ia), (ob, _, ib) in zip(base[r], new):
                if oa is not None and ia['lap'] is not None and 0.6 <= ia['x'] <= 3.6: sw[(r[4:8], ia['lap'])][0].append(oa); sw[(r[4:8], ia['lap'])][1].append(ob)
        s = np.array([(np.std(v[0]), np.std(v[1]), np.max(v[0]) - np.min(v[0]), np.max(v[1]) - np.min(v[1])) for k, v in sw.items() if len(v[0]) > 40])
        print('  command over x 0.6..3.6 m, per pass (n=%d): standard deviation median %.0f -> %.0f us ; max - min median %.0f -> %.0f us (open loop: the PID input is the recorded one)' % (len(s), np.median(s[:, 0]), np.median(s[:, 1]), np.median(s[:, 2]), np.median(s[:, 3])))


if __name__ == '__main__':
    main()
