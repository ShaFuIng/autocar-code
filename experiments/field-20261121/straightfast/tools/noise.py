"""straightfast-20261121 step 1b: how noisy is derr at 30 frames/s, how does it relate to the IMU yaw rate, and was the slow-down 'needed'?
usage: python noise.py > ../proof/S2_derr_noise_and_need.txt"""
import sys, collections, json
import numpy as np
from common import *
from breakdown import classify


def lp_abs_gz(t, gz, tau=0.25):
    """causal low-pass of the yaw rate (deg/s), then abs"""
    out = []; y = None; tp = None
    for a, g in zip(t, gz):
        if g is None: out.append(np.nan); continue
        if y is None: y = g
        else: k = 1. - np.exp(-max(0., min(0.2, a - tp)) / tau); y = y + (g - y) * k
        tp = a; out.append(abs(y))
    return np.array(out)


def main():
    ri = json.loads((SNAP / 'rate_indep.json').read_text(encoding='utf8')) if (SNAP / 'rate_indep.json').exists() else None
    print('S2. derr (speed_adapt: |e - e_earlier| / dt) inside the three PWM-100 zones; runs %s. rate_indep.json: %s' % (', '.join(r[4:8] for r in RUNS), json.dumps(ri)[:200] if ri else 'absent'))
    D = collections.defaultdict(list); ev = []; ctl = []
    for run in RUNS:
        F = classify(load(run)); t = np.array([f['t'] for f in F]); gl = lp_abs_gz(t, [f['gz'] for f in F])
        e = np.array([abs(f['e']) if f['e'] is not None else np.nan for f in F]); es = np.array([f['e'] if f['e'] is not None else np.nan for f in F])
        dts = np.diff(t, prepend=t[0])
        for i, f in enumerate(F):
            if not f['zone'] or f['derr'] is None: continue
            D[f['zone']].append((f['derr'], e[i], abs(f['gz']) if f['gz'] is not None else np.nan, gl[i], dts[i], abs(es[i] - es[i - 1]) if i else np.nan))
        # block events: first blocked frame after >= 5 unblocked in-zone frames
        run_ok = 0
        for i, f in enumerate(F):
            if not f['zone']: run_ok = 0; continue
            blocked = f['cls'].startswith('BLOCK')
            if blocked and run_ok >= 5:
                a = (t >= t[i] - 0.3) & (t < t[i]); b = (t > t[i]) & (t <= t[i] + 1.0); b3 = (t > t[i]) & (t <= t[i] + 0.3)
                gz = np.array([F[k]['gz'] if F[k]['gz'] is not None else np.nan for k in range(len(F))])
                ev.append(dict(run=run[4:8], zone=f['zone'], cls=f['cls'], e_now=e[i], derr=f['derr'], gl=gl[i], e_before=np.nanmean(e[a]), e_max_after=np.nanmax(e[b]), e_mean_after=np.nanmean(e[b]),
                               gz_sd_before=np.nanstd(gz[(t >= t[i] - 1.0) & (t < t[i])]), gz_sd_after=np.nanstd(gz[b]), pwm_min=min(F[k]['pwm'] or 0 for k in np.flatnonzero(b)), since=f['since']))
            if not blocked and f['cls'] == 'FULL' and run_ok >= 30 and run_ok % 30 == 0:   # control: full-throttle frames in the same zones
                b = (t > t[i]) & (t <= t[i] + 1.0); a = (t >= t[i] - 0.3) & (t < t[i])
                gz = np.array([F[k]['gz'] if F[k]['gz'] is not None else np.nan for k in range(len(F))])
                ctl.append(dict(zone=f['zone'], e_before=np.nanmean(e[a]), e_max_after=np.nanmax(e[b]), e_mean_after=np.nanmean(e[b]), gz_sd_before=np.nanstd(gz[(t >= t[i] - 1.0) & (t < t[i])]), gz_sd_after=np.nanstd(gz[b])))
            run_ok = 0 if blocked else run_ok + 1
    print('\n(a) derr distribution per zone (all in-zone frames) and on frames with |e| < 0.10')
    for z, _, _ in ZONES:
        a = np.array(D[z]); small = a[a[:, 1] < 0.10]
        print('  %-6s n %4d | frame dt median %.3f s | |delta e| per frame median %.3f p90 %.3f p99 %.3f | derr median %.2f p90 %.2f p99 %.2f | derr > 0.8: %.1f%%  > 1.2: %.1f%%'
              % (z, len(a), np.median(a[:, 4]), np.nanmedian(a[:, 5]), np.nanpercentile(a[:, 5], 90), np.nanpercentile(a[:, 5], 99), np.median(a[:, 0]), np.percentile(a[:, 0], 90), np.percentile(a[:, 0], 99), 100 * np.mean(a[:, 0] > 0.8), 100 * np.mean(a[:, 0] > 1.2)))
        print('         |e| < 0.10: n %4d | derr median %.2f p90 %.2f p99 %.2f | > 0.8: %.1f%%  > 1.2: %.1f%%  (rate_indep on: derr is taken against the newest earlier sample >= 0.0525 s old, about 2 frames / 0.07 s back; derr 1.2 = the error moved ~0.09 in 0.07 s)'
              % (len(small), np.median(small[:, 0]), np.percentile(small[:, 0], 90), np.percentile(small[:, 0], 99), 100 * np.mean(small[:, 0] > 0.8), 100 * np.mean(small[:, 0] > 1.2)))
    allf = np.array([x for z in D for x in D[z]]); ok = np.isfinite(allf[:, 3])
    hi = allf[ok & (allf[:, 0] > 1.2)]; lo = allf[ok & (allf[:, 0] <= 0.8)]
    print('\n(b) derr vs the IMU yaw rate (|gz| low-passed 0.25 s), all three zones')
    print('  correlation derr ~ |gz_lp|: %.2f ; derr ~ |gz| raw: %.2f ; derr ~ |e|: %.2f' % (np.corrcoef(allf[ok, 0], allf[ok, 3])[0, 1], np.corrcoef(allf[ok, 0], np.nan_to_num(allf[ok, 2]))[0, 1], np.corrcoef(allf[ok, 0], allf[ok, 1])[0, 1]))
    print('  frames with derr > 1.2 (n %d): |gz_lp| median %.1f p90 %.1f dps, |e| median %.2f p90 %.2f' % (len(hi), np.median(hi[:, 3]), np.percentile(hi[:, 3], 90), np.median(hi[:, 1]), np.percentile(hi[:, 1], 90)))
    print('  frames with derr <= 0.8 (n %d): |gz_lp| median %.1f p90 %.1f dps, |e| median %.2f p90 %.2f' % (len(lo), np.median(lo[:, 3]), np.percentile(lo[:, 3], 90), np.median(lo[:, 1]), np.percentile(lo[:, 1], 90)))
    print('  |gz_lp| over all in-zone frames: median %.1f p90 %.1f p99 %.1f max %.1f dps ; |e|: median %.2f p90 %.2f p99 %.2f max %.2f' % (np.median(allf[ok, 3]), np.percentile(allf[ok, 3], 90), np.percentile(allf[ok, 3], 99), allf[ok, 3].max(), np.median(allf[:, 1]), np.percentile(allf[:, 1], 90), np.percentile(allf[:, 1], 99), allf[:, 1].max()))
    print('\n(c) block events (first blocked frame after >= 5 unblocked in-zone frames): %d ; control = %d full-throttle frames in the same zones (every 30th frame of a full run)' % (len(ev), len(ctl)))
    by = collections.defaultdict(list)
    for x in ev: by[x['cls']].append(x)
    def line(name, X):
        g = lambda k: np.array([x[k] for x in X], float)
        print('  %-36s n %3d | |e| 0.3 s before %.2f, at the event %s, mean / max in the next 1.0 s %.2f / %.2f | next-1-s max |e| > 0.30: %d, > 0.45: %d | gz sd before %.1f -> after %.1f dps'
              % (name, len(X), np.nanmean(g('e_before')), '%.2f' % np.nanmean(g('e_now')) if 'e_now' in X[0] else '  - ', np.nanmean(g('e_mean_after')), np.nanmean(g('e_max_after')), int(np.sum(g('e_max_after') > 0.30)), int(np.sum(g('e_max_after') > 0.45)), np.nanmean(g('gz_sd_before')), np.nanmean(g('gz_sd_after'))))
    for cls, X in sorted(by.items(), key=lambda kv: -len(kv[1])): line(cls, X)
    line('ALL block events', ev); line('control (full throttle, no block)', ctl)
    print('\n  per event (zone, position, class, |e| at the event, derr, |gz_lp|, PWM minimum in the next second, max |e| in the next second):')
    for x in ev: print('   %s %-6s since %5.2f m  %-34s |e| %.2f derr %.2f gz_lp %4.1f  pwm min %3.0f  max|e| after %.2f' % (x['run'], x['zone'], x['since'] or 0, x['cls'], x['e_now'], x['derr'], x['gl'], x['pwm_min'], x['e_max_after']))


if __name__ == '__main__':
    main()
