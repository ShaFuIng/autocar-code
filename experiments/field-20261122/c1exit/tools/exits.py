"""per-exit table.  python exits.py [C1e|C2e] [runs...]   (default: C1e, MAIN+MORE)"""
import sys, numpy as np, lib
from lib import g
an = sys.argv[1] if len(sys.argv) > 1 else 'C1e'
runs = sys.argv[2:] or lib.MAIN + lib.MORE
C = lib.CENTRE
rows = []
def collect(run):
    Q = lib.load(run)
    t = np.array([p['t'] for p in Q]); enc = np.array([g(p, 'inputs', 'enc_b') for p in Q], float); v = lib.speed(t, enc)
    gz = np.array([g(p, 'inputs', 'gz') for p in Q], float)
    e = np.array([g(p, 'image_error') for p in Q], float); pre = np.array([g(p, 'steer_us') for p in Q], float) - C
    sent = np.array([g(p, 'play_comp', 'steer_us') for p in Q], float) - C
    I = np.array([g(p, 'i') for p in Q], float); D = np.array([g(p, 'd') for p in Q], float)
    y = np.array([g(p, 'arc_lat', 'y_cm') for p in Q], float); psi = np.array([g(p, 'arc_lat', 'psi_deg') for p in Q], float)
    cin = np.array([g(p, 'arc_lat', 'clr_in_cm') for p in Q], float); cout = np.array([g(p, 'arc_lat', 'clr_out_cm') for p in Q], float)
    pwm = np.array([g(p, 'pwm') for p in Q], float)
    anc = [g(p, 'lap_pos', 'anchor', default=None) for p in Q]; valid = np.array([bool(p.get('valid')) for p in Q])
    yaw = np.concatenate([[0], np.cumsum(np.nan_to_num(gz[1:]) * np.diff(t))])
    out = []
    for i in range(1, len(Q)):
        if anc[i] == an and anc[i - 1] != an:
            t0 = t[i]
            if t[-1] - t0 < 3.0: continue
            A = (t >= t0 - 1.2) & (t < t0); B = (t >= t0) & (t <= t0 + 2.5); B1 = (t >= t0) & (t <= t0 + 1.0); Z = (t >= t0 - 0.1) & (t <= t0 + 0.1)
            Bl = (t >= t0 + 1.5) & (t <= t0 + 3.0)
            r = dict(run=run, lap=Q[i]['_lap'], i=i, t0=t0,
                     pk_sent=np.nanmax(sent[A]), pk_e=np.nanmax(e[A]), gz_min=np.nanmin(gz[A]), t_pk=t[A][np.nanargmax(sent[A])] - t0,
                     e0=np.nanmean(e[Z]), gz0=np.nanmean(gz[Z]), d0=np.nanmean(D[Z]), i0=np.nanmean(I[Z]), v0=np.nanmean(v[Z]),
                     y0=np.nanmean(y[Z]) if np.isfinite(y[Z]).any() else np.nan, psi0=np.nanmean(psi[Z]) if np.isfinite(psi[Z]).any() else np.nan,
                     ymin=np.nanmin(y[A]) if np.isfinite(y[A]).any() else np.nan, ymax=np.nanmax(y[A]) if np.isfinite(y[A]).any() else np.nan,
                     cin=np.nanmin(cin[A | B1]) if np.isfinite(cin[A | B1]).any() else np.nan,
                     pre_min=np.nanmin(pre[B]), pre_max=np.nanmax(pre[B]), sent_min=np.nanmin(sent[B]), sent_max=np.nanmax(sent[B]),
                     e_min=np.nanmin(e[B]), e_max=np.nanmax(e[B]), gzB_min=np.nanmin(gz[B]), gzB_max=np.nanmax(gz[B]),
                     yaw_sw=np.max(yaw[B]) - np.min(yaw[B]), yaw_net=yaw[B][-1] - yaw[B][0], sd=np.nanstd(pre[B]),
                     e_late=np.nanmean(e[Bl]), lost=int((~valid[B]).sum()))
            out.append(r)
    return out
if __name__ == '__main__':
    for run in runs: rows += collect(run)
    print('%s exits: before = t0-1.2..t0 ; after = t0..t0+2.5 s ; t0 = first frame with lap_pos anchor %s (5 Hz update)' % (an, an))
    print('run  lap | pk_sent t_pk  pk_e gz_min | e0    gz0   d0   i0   v0   y0   psi0 | pre_min pre_max sent_min sent_max e_min e_max gzmin gzmax yaw_sw yaw_net sd  clr_in lost')
    for r in rows:
        print('%s %2d  | %5.0f %5.2f %5.2f %5.0f  | %5.2f %5.0f %4.0f %4.0f %4.2f %5.1f %5.1f | %5.0f %5.0f   %5.0f %5.0f   %5.2f %5.2f %5.0f %5.0f %5.1f %5.1f %4.0f %5.1f %d' % (
            r['run'], r['lap'], r['pk_sent'], r['t_pk'], r['pk_e'], r['gz_min'], r['e0'], r['gz0'], r['d0'], r['i0'], r['v0'], r['y0'], r['psi0'],
            r['pre_min'], r['pre_max'], r['sent_min'], r['sent_max'], r['e_min'], r['e_max'], r['gzB_min'], r['gzB_max'], r['yaw_sw'], r['yaw_net'], r['sd'], r['cin'], r['lost']))
    # correlations
    keys = ['pk_sent', 'pk_e', 'gz_min', 't_pk', 'e0', 'gz0', 'd0', 'i0', 'v0', 'y0', 'psi0', 'ymax']
    amp = np.array([r['pre_max'] - r['pre_min'] for r in rows]); em = np.array([r['e_max'] - r['e_min'] for r in rows])
    print('\ncorrelation with the after-exit command swing (pre_max - pre_min) / error swing (e_max - e_min), n=%d' % len(rows))
    for k in keys:
        x = np.array([r[k] for r in rows], float); m = np.isfinite(x)
        print('  %-8s r_cmd %+.2f  r_err %+.2f' % (k, np.corrcoef(x[m], amp[m])[0, 1], np.corrcoef(x[m], em[m])[0, 1]))
