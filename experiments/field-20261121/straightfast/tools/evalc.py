"""straightfast-20261121 step 3: open-loop evaluation of the candidates (the car's lap_speed layer fed with the recorded frames, tools/emul.py).
usage: python evalc.py <name>=<lap_speed.json>[:cand] ...      ':cand' = use candidate/lap_speed.py (lap-speed-r3) instead of the car module
Old side = the car config 77ffbe32 on the car module, both through the same emulator."""
import sys, json, collections, importlib.util
from pathlib import Path
import numpy as np
from common import *
import emul

HERE = Path(__file__).resolve().parent
CAR = json.loads((SNAP / 'lap_speed.json').read_text(encoding='utf8'))
# evaluation windows per straight: anchor, zone start, 100-zone end, end of the window (the next curve entry), checkpoints (name, metres)
WIN = dict(main=('C4e', 0.8, 7.3, 10.4, (('zone end C4e+7.3', 7.3), ('C4e+9.0', 9.0), ('curve 1 entry C4e+10.2', 10.2))),
           diag=('C1e', 0.8, 8.0, 12.0, (('zone end C1e+8.0', 8.0), ('entry zone C1e+9.8', 9.8), ('curve 2 entry C1e+11.8', 11.8))),
           circle=('C2e', 0.5, 5.0, 6.9, (('zone end C2e+5.0', 5.0), ('left bend C2e+6.8', 6.8))))
GS = json.loads((SNAP / 'gain_schedule.json').read_text(encoding='utf8'))


def cand_module():
    sys.path.insert(0, str(SNAP))
    sp = importlib.util.spec_from_file_location('lap_speed_cand', str(HERE.parent / 'candidate' / 'lap_speed.py')); m = importlib.util.module_from_spec(sp); sp.loader.exec_module(m)
    return m


def lp_abs_gz(t, gz, tau=0.25):
    out = []; y = None; tp = None
    for a, g in zip(t, gz):
        if g is None: out.append(np.nan); continue
        if y is None or a - tp > 0.5: y = g
        else: y = y + (g - y) * (1. - np.exp(-max(0., a - tp) / tau))
        tp = a; out.append(abs(y))
    return np.array(out)


def in_win(r, pad=0.3):
    for z, (anc, a, b, c, _) in WIN.items():
        if r['anchor'] == anc and r['since'] is not None and a - pad <= r['since'] <= c + pad: return z
    return None


def zone100(r):
    for z, (anc, a, b, c, _) in WIN.items():
        if r['anchor'] == anc and r['since'] is not None and a <= r['since'] <= b: return z
    return None


def stats(R, pw):
    """per 100 zone: frames, mean pwm, >=95 share, dip events (>=90 then <70 in the same pass)"""
    out = collections.OrderedDict((z, dict(n=0, s=0., hi=0, dips=0, passes=0)) for z in WIN)
    prev = None; was_hi = False; in_dip = False
    for r, p in zip(R, pw):
        z = zone100(r)
        if z is None or r['lap'] is None or r['lap'] < 1 or not np.isfinite(p): prev = None; continue
        key = (z, r['lap'])
        if key != prev: was_hi = False; in_dip = False; out[z]['passes'] += 1
        prev = key; o = out[z]; o['n'] += 1; o['s'] += p; o['hi'] += p >= 95
        if p >= 90: was_hi = True; in_dip = False
        elif p < 70 and was_hi and not in_dip: o['dips'] += 1; in_dip = True
    return out


def main():
    cm = None; rows = []
    for arg in sys.argv[1:]:
        name, path = arg.split('=', 1); use_c = path.endswith(':cand'); path = path[:-5] if use_c else path
        if use_c and cm is None: cm = cand_module()
        cfg = json.loads(Path(path).read_text(encoding='utf8'))
        print('=' * 118); print('candidate %s  (%s, %s)' % (name, Path(path).name, 'candidate lap_speed.py lap-speed-r3' if use_c else 'car lap_speed.py 3ad2272b'))
        emp = collections.defaultdict(list)
        tot = collections.Counter(); S_old = None; S_new = None; gains = collections.defaultdict(list); cps = collections.defaultdict(list); trouble = collections.Counter(); kch = collections.Counter(); blockers = collections.Counter()
        for run in RUNS:
            A = emul.run(CAR, str(EXP / run)); B = emul.run(cfg, str(EXP / run), module=cm if use_c else None); assert len(A) == len(B)
            last = max(r['lap'] for r in A if r['lap'] is not None)
            t = np.array([r['t'] for r in A]); enc = [r['enc'] for r in A]
            pa = np.array([r['new_pwm'] if isinstance(r['new_pwm'], (int, float)) else np.nan for r in A]); pb = np.array([r['new_pwm'] if isinstance(r['new_pwm'], (int, float)) else np.nan for r in B])
            rec = np.array([r['rec_pwm'] if isinstance(r['rec_pwm'], (int, float)) else np.nan for r in A])
            glp = lp_abs_gz(t, [r['gz'] for r in A]); ea = np.array([abs(r['e']) if r['e'] is not None else np.nan for r in A])
            ch = np.flatnonzero(np.abs(np.nan_to_num(pb - pa)) > 1e-6)
            for i in ch:
                tot['changed'] += 1; z = in_win(A[i])
                if z is None:
                    zl = (B[i]['new_ls'] or A[i]['new_ls'] or {}).get('zone') or 'no zone'
                    tot['outside'] += 1; tot['outside: ' + str(zl)[:28]] += 1
                if pb[i] < pa[i] - 1e-6: tot['lowered'] += 1
                # gain_schedule k = max(k_min, ref_pwm / pwm): changes only when one side is below ref/k_min
                lim = GS['ref_pwm'] / GS['k_min']
                if min(pa[i], pb[i]) < lim: kch['k differs (old PWM < %.1f)' % lim] += 1
            tot['frames'] += len(A)
            for side, pw in (('old', pa), ('new', pb)):
                st = stats(A, pw)
                tgt = S_old if side == 'old' else S_new
                if tgt is None:
                    tgt = st
                    if side == 'old': S_old = tgt
                    else: S_new = tgt
                else:
                    for z in st:
                        for k in st[z]: tgt[z][k] += st[z][k]
            # frames with real trouble inside the 100 zones: does the candidate still reduce?
            for i, r in enumerate(A):
                z = zone100(r)
                if z is None or r['lap'] is None or r['lap'] < 1: continue
                bl = (B[i]['new_ls'] or {})
                if bl.get('blocked'): blockers[bl['blocked']] += 1
                tw = (bl.get('two_level') or {})
                if tw.get('state') == 'TROUBLE': blockers['two_level TROUBLE:' + str(tw.get('why'))] += 1
                if (np.isfinite(ea[i]) and ea[i] > 0.40) or (np.isfinite(glp[i]) and glp[i] > 25.) or r['e'] is None:
                    trouble['frames'] += 1
                    j = min(len(A) - 1, int(np.searchsorted(t, t[i] + 0.25)))
                    trouble['new PWM <= 75 within 0.25 s'] += np.nanmin(pb[i:j + 1]) <= 75.
                    trouble['old PWM <= 75 within 0.25 s'] += np.nanmin(pa[i:j + 1]) <= 75.
            # speed model per pass
            for z, (anc, a, b, c, cks) in WIN.items():
                passes = collections.OrderedDict()
                for i, r in enumerate(A):
                    if r['anchor'] == anc and r['since'] is not None and a - 0.2 <= r['since'] <= c and r['lap'] is not None and 1 <= r['lap'] and (r['lap'] < last or z != 'main'):
                        passes.setdefault((r['lap'], int(i > 0 and A[i - 1]['anchor'] != anc)), None); passes.setdefault(r['lap'], []).append(i)
                for lap, idx in passes.items():
                    if not isinstance(idx, list) or len(idx) < 30 or A[idx[-1]]['since'] < c - 0.6: continue
                    tt = t[idx]; vm = np.array([meas_v(t, enc, i) for i in idx]); ok = np.isfinite(vm)
                    if ok.sum() < 5: continue
                    v0 = vm[ok][0]; pr = np.nan_to_num(rec[idx], nan=50.)
                    k = min(np.arange(0.85, 1.151, 0.01), key=lambda q: np.nanmean((sim(tt, pr, v0, q) - vm) ** 2))
                    va = sim(tt, np.nan_to_num(pa[idx], nan=50.), v0, k); vb = sim(tt, np.nan_to_num(pb[idx], nan=50.), v0, k)
                    da = np.concatenate([[0.], np.cumsum(0.5 * (va[1:] + va[:-1]) * np.diff(tt))]); db = np.concatenate([[0.], np.cumsum(0.5 * (vb[1:] + vb[:-1]) * np.diff(tt))])
                    Tn = float(np.interp(da[-1], db, tt - tt[0])) if db[-1] >= da[-1] else tt[-1] - tt[0]
                    gains[z].append(((tt[-1] - tt[0]) - Tn, float(np.sqrt(np.nanmean((sim(tt, pr, v0, k) - vm) ** 2)))))
                    inz = np.array([a <= A[i]['since'] <= b for i in idx]); dz = np.diff(tt, append=tt[-1]); dz[dz > 0.2] = 0.
                    dist = abs(enc[idx[-1]] - enc[idx[0]]) * M if enc[idx[-1]] is not None and enc[idx[0]] is not None else np.nan
                    emp[z].append((tt[-1] - tt[0], float(np.nansum((100. - rec[idx][inz]) * dz[inz])), float(np.nansum((100. - np.nan_to_num(pa[idx], nan=50.)[inz]) * dz[inz])), float(np.nansum((100. - np.nan_to_num(pb[idx], nan=50.)[inz]) * dz[inz])), dist))
                    since = np.array([A[i]['since'] for i in idx])
                    for nm, m in cks:
                        n = int(np.argmin(np.abs(since - m)))
                        if abs(since[n] - m) > 0.3: continue
                        # the new side reaches the same place earlier: compare the speeds at the same DISTANCE
                        nb = min(len(vb) - 1, int(np.searchsorted(db, da[n])))
                        cps[(z, nm)].append((va[n], vb[nb]))
        print('frames %d | PWM changed %d | changed outside the three straights (zone start-0.3 m .. next curve entry+0.3 m) %d %s | lowered %d' % (
            tot['frames'], tot['changed'], tot['outside'], {k[9:]: v for k, v in tot.items() if k.startswith('outside: ')} or '', tot['lowered']))
        print('steering: gain_schedule k = max(%.2f, %.1f / PWM); frames where the changed PWM crosses PWM %.1f (k would differ): %d of %d changed frames' % (GS['k_min'], GS['ref_pwm'], GS['ref_pwm'] / GS['k_min'], sum(kch.values()), tot['changed']))
        tsum = 0.
        for z in WIN:
            o, n = S_old[z], S_new[z]; g = np.array(gains[z]) if gains[z] else np.zeros((0, 2))
            med = float(np.median(g[:, 0])) if len(g) else float('nan'); tsum += 0. if np.isnan(med) else med
            print('  %-6s passes %2d frames %4d | PWM >= 95: %4.1f%% -> %4.1f%% | mean PWM %5.1f -> %5.1f | dips (>=90 then <70): %2d -> %2d | model time saved per pass: median %.2f s (min %.2f max %.2f, n %d, rms %.3f m/s)'
                  % (z, o['passes'], o['n'], 100. * o['hi'] / max(1, o['n']), 100. * n['hi'] / max(1, n['n']), o['s'] / max(1, o['n']), n['s'] / max(1, n['n']), o['dips'], n['dips'], med, g[:, 0].min() if len(g) else 0, g[:, 0].max() if len(g) else 0, len(g), np.median(g[:, 1]) if len(g) else 0))
            for nm, m in WIN[z][4]:
                c = np.array(cps[(z, nm)]) if cps[(z, nm)] else None
                if c is not None: print('           speed at %-26s old %.2f -> new %.2f m/s (median; max increase %+.2f)' % (nm, np.median(c[:, 0]), np.median(c[:, 1]), (c[:, 1] - c[:, 0]).max()))
        print('  model sum of the three medians: %.2f s per lap' % tsum)
        esum = 0.
        for z in WIN:   # model-free cross-check: recorded window time against the recorded in-zone PWM deficit, pass by pass
            e_ = np.array(emp[z]) if emp[z] else None
            if e_ is None or len(e_) < 5: continue
            sl, ic = np.polyfit(e_[:, 1], e_[:, 0], 1); r_ = np.corrcoef(e_[:, 1], e_[:, 0])[0, 1]; dd = e_[:, 2] - e_[:, 3]
            esum += max(0., sl) * float(np.median(dd))
            print('  empirical %-6s: window time = %.2f s + %.4f s per PWM*s of in-zone deficit (n %d, r %.2f); the candidate removes median %.0f PWM*s -> %.2f s per pass (model above: %.2f)'
                  % (z, ic, sl, len(e_), r_, np.median(dd), max(0., sl) * float(np.median(dd)), float(np.median(np.array(gains[z])[:, 0])) if gains[z] else float('nan')))
        print('  empirical sum: %.2f s per lap (regression over 10-13 recorded passes per straight; a rough cross-check, not a prediction)' % esum)
        print('  in-zone frames with real trouble (|e| > 0.40 or |gz_lp| > 25 dps or no error): %d ; PWM <= 75 within 0.25 s: old %d, new %d' % (trouble['frames'], trouble['old PWM <= 75 within 0.25 s'], trouble['new PWM <= 75 within 0.25 s']))
        print('  what still holds the PWM down inside the 100 zones (new side, frames): %s' % dict(blockers.most_common(8)))
        rows.append((name, tot['changed'], tot['outside'], tot['lowered'], tsum, esum))
    print('=' * 118)
    for r in rows: print('SUMMARY %-22s changed %5d | outside %4d | lowered %3d | model %.2f s/lap | empirical %.2f s/lap' % r)


if __name__ == '__main__':
    main()
