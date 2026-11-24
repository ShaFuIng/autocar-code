"""curve-1 exit: grouping and the four hypotheses of the brief, from the logged passes.   python hyp.py [runs]
swing = (max - min) of the pre-play command in the 2.5 s after the first C1e frame.  big: >= 200 us."""
import sys, numpy as np, lib, passes
runs = sys.argv[1:] or lib.MAIN + lib.MORE
R = []
for run in runs:
    Q, A = passes.run_arrays(run)
    for i in passes.exits(A, 'C1e'):
        t = A['t']; t0 = t[i]; ref = passes.straight_ref(A, i); tb = ref - A['yaw']
        a = (t >= t0 - 1.2) & (t < t0); b = (t >= t0) & (t <= t0 + 2.5); cur = (t >= t0 - 3.2) & (t < t0)
        fin = np.nanmean(tb[(t > t0 + 1.2) & (t < t0 + 2.2)])
        pre = A['pre']; sw = np.nanmax(pre[b]) - np.nanmin(pre[b])
        # (1) heading
        over = np.nanmax(tb[(t >= t0 - 0.3) & (t <= t0 + 0.6)]) - fin     # how far the heading went past the final diagonal heading
        # mid-curve sag: lowest 0.3 s mean of the driven curvature between tb 50 and 100
        mid = cur & (tb > 50) & (tb < 100) & (A['v'] > 0.5); kc = (-A['gz'] / A['v'])[mid]
        sag = np.nanpercentile(kc, 15) if mid.sum() > 5 else np.nan
        slam_tb = tb[a][np.nanargmax(A['sent'][a])]
        # (2) play: first left command after the late peak -> latency to a left yaw rate
        j_pk = np.where(a)[0][np.nanargmax(A['sent'][a])]; lat = np.nan; ineff = np.nan
        js = [j for j in range(j_pk, min(len(t), j_pk + 90)) if pre[j] < -20]
        if js:
            j0 = js[0]; jr = [j for j in range(j0, min(len(t), j0 + 40)) if A['gz'][j] > 5]
            if jr: lat = t[jr[0]] - t[j0]
        # frames after the exit with a left command inside the play band (-65..0 pre-play) while the car still turns right faster than 5 dps
        m2 = b & (pre < 0) & (pre > -65); ineff = float(np.sum(m2 & (A['gz'] < -5)) / max(1, m2.sum())) if m2.sum() > 5 else np.nan
        # (3) steps: curve_map FLOOR -> RELEASED frame, integral release
        jr_ = [j for j in np.where(cur | b)[0] if A['cmr'][j] == 'RELEASED' and A['cmr'][j - 1] == 'FLOOR']
        rel_step = pre[jr_[0]] - pre[jr_[0] - 1] if jr_ else np.nan; rel_tb = tb[jr_[0]] if jr_ else np.nan; rel_dt = t[jr_[0]] - t0 if jr_ else np.nan
        I0 = A['I'][i]; I05 = np.nanmean(A['I'][(t >= t0 + 0.45) & (t <= t0 + 0.55)])
        maxstep = np.nanmax(np.abs(np.diff(pre[b])))
        # (4) throttle: when the PWM passes 60 vs when the command extremes happen
        jp = [j for j in np.where(b)[0] if A['pwm'][j] >= 60]; t_pwm = t[jp[0]] - t0 if jp else np.nan
        t_min = t[b][np.nanargmin(pre[b])] - t0; t_max = t[b][np.nanargmax(pre[b])] - t0
        gz_b = A['gz'][(t >= t0 + (t_pwm if np.isfinite(t_pwm) else 9)) & (t <= t0 + (t_pwm if np.isfinite(t_pwm) else 9) + 0.8)]
        R.append(dict(run=run, lap=int(A['lap'][i]), sw=sw, pk=np.nanmax(A['sent'][a]), pk_e=np.nanmax(A['e'][a]), gzmin=np.nanmin(A['gz'][a]), over=over, fin=fin, tb0=tb[i], e0=A['e'][i], sag=sag, slam_tb=slam_tb,
                      lat=lat, ineff=ineff, rel_step=rel_step, rel_tb=rel_tb, rel_dt=rel_dt, I0=I0, I05=I05, maxstep=maxstep, t_pwm=t_pwm, t_min=t_min, t_max=t_max,
                      emin=np.nanmin(A['e'][b]), emax=np.nanmax(A['e'][b]), y0=np.nanmean(A['y'][(t >= t0 - 0.1) & (t <= t0 + 0.1)]), v0=np.nanmean(A['v'][(t >= t0 - 0.3) & (t <= t0 + 0.3)]),
                      gz_after_pwm=np.nanstd(gz_b) if len(gz_b) > 5 else np.nan, cin=np.nanmin(A['cin'][cur | (b & (t < t0 + 1))]) if np.isfinite(A['cin'][cur | (b & (t < t0 + 1))]).any() else np.nan))
g = lambda k, S=R: np.array([r[k] for r in S], float)
big = [r for r in R if r['sw'] >= 200]; mid_ = [r for r in R if 130 <= r['sw'] < 200]; small = [r for r in R if r['sw'] < 130]
print('curve-1 exits: %d passes (%s).  swing >= 200 us: %d | 130..200: %d | < 130: %d' % (len(R), ' '.join(runs), len(big), len(mid_), len(small)))
print('\nper pass (sorted by swing):')
print(' run lap | swing | late pk_sent  pk_e  gz_min  slam@tb | mid-curve low curvature | over-rot  e0    y0   | e min / max  | release step @tb  dt | I0 -> I(0.5 s) | max frame step | t(pwm>=60) t(cmd min) t(cmd max)')
for r in sorted(R, key=lambda r: -r['sw']):
    print(' %s %d  | %4.0f  | %5.0f      %.2f  %5.0f  %5.0f   | %5.1f                   | %+5.1f   %+.2f %+5.1f | %+.2f / %+.2f | %+5.0f  %5.0f %+.2f | %3.0f -> %3.0f     | %4.0f           | %.2f  %.2f  %.2f' % (
        r['run'], r['lap'], r['sw'], r['pk'], r['pk_e'], r['gzmin'], r['slam_tb'], r['sag'], r['over'], r['e0'], r['y0'], r['emin'], r['emax'], r['rel_step'], r['rel_tb'], r['rel_dt'], r['I0'], r['I05'], r['maxstep'], r['t_pwm'], r['t_min'], r['t_max']))
print('\ngroup medians                         big (>=200)   middle    small (<130)')
for k, name in (('pk', 'late peak sent command [us]'), ('pk_e', 'late peak error'), ('gzmin', 'late min yaw rate [dps]'), ('sag', 'mid-curve low curvature [deg/m] (lane 44.9)'), ('over', 'heading past the diagonal [deg]'),
                ('e0', 'error at the first C1e frame'), ('y0', 'lateral position at C1e [cm]'), ('v0', 'speed at C1e [m/s]'), ('I0', 'integral at C1e [us]'), ('rel_step', 'command step at curve_map release [us]'),
                ('rel_dt', 'release time rel. C1e [s]'), ('maxstep', 'largest frame-to-frame step after C1e [us]'), ('lat', 'left command -> left yaw latency [s]'), ('ineff', 'share of small-left frames still turning right'),
                ('t_pwm', 't(PWM >= 60) after C1e [s]'), ('t_min', 't(command minimum) [s]'), ('t_max', 't(command maximum) [s]'), ('cin', 'inner clearance min [cm]')):
    print('  %-46s %8.2f  %8.2f  %8.2f' % (name, np.nanmedian(g(k, big)) if big else np.nan, np.nanmedian(g(k, mid_)) if mid_ else np.nan, np.nanmedian(g(k, small))))
print('\ncorrelation with the swing (n = %d)' % len(R))
for k in ('pk', 'pk_e', 'gzmin', 'sag', 'over', 'e0', 'y0', 'v0', 'I0', 'rel_step', 'rel_dt', 't_pwm'):
    x = g(k); m = np.isfinite(x); print('  %-10s r %+.2f' % (k, np.corrcoef(x[m], g('sw')[m])[0, 1]))
x = g('pk'); y_ = g('over'); m = np.isfinite(y_); print('  late peak command vs heading past the diagonal: r %+.2f' % np.corrcoef(x[m], y_[m])[0, 1])
x = g('sag'); m = np.isfinite(x); print('  mid-curve low curvature vs late peak command:   r %+.2f' % np.corrcoef(x[m], g('pk')[m])[0, 1])
print('\n(2) play: left command -> left yaw latency: median %.2f s (p10 %.2f p90 %.2f, n %d) = the identified actuator delay (ident.py best lag 0.20 s); no extra dead time' % (
    np.nanmedian(g('lat')), np.nanpercentile(g('lat'), 10), np.nanpercentile(g('lat'), 90), int(np.isfinite(g('lat')).sum())))
print('(4) throttle: PWM passes 60 at median %.2f s after C1e; the command minimum is at median %.2f s, the maximum at %.2f s; in the big group: min %.2f s, max %.2f s' % (
    np.nanmedian(g('t_pwm')), np.nanmedian(g('t_min')), np.nanmedian(g('t_max')), np.nanmedian(g('t_min', big)) if big else np.nan, np.nanmedian(g('t_max', big)) if big else np.nan))
