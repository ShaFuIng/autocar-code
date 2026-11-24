"""c1lap1b: lap-1 settings-only option = later arming of the lap-1 window (curves.c1.start.win_m[0]); replays the controller's own rule on
the recorded frames: armed when the (stepping) odometer reading - start.gap_m >= win_m[0]; FAR trigger = far >= 0.4 and near >= -0.05 on
an armed frame; d0 = reading at the trigger; band onset (a >= 0.5) = first frame with reading - d0 - delay_m >= 0.5 * ramp_m (0.30 / 0.20).
Open loop: valid only while the new onset comes no later than ~0.25 m after the recorded onset... frames after the recorded onset carry
the recorded band steering (camera far / near change little within that distance on the straight-to-arc transition; flagged).
Outputs per lap-1 pass: new trigger / onset relative to the camera curvature marker (kmark), shift of the onset vs recorded, estimate of
clr_in_min = recorded + SLOPE * onset shift (SLOPE from B1: y_min vs onset_minus_kmark, all 21 band passes)."""
import statistics as st
from pathlib import Path
from lib import load, num
from passes import curve_passes
from trig import cont
from trigsim import kmark
OUT = Path(__file__).resolve().parents[1] / 'proof'
RUNS = ['3f22', 'fd22', 'b23a', '2b72', 'b619', '1377', 'e69d', '54cb', '1cd7']
GAP = 3.2; DELAY = 0.30; RAMP = 0.20; SLOPE = 25.17


def sim(F, so, p, win0, delay=DELAY, near_k=None, n0=0.03):
    q_rec = p['band'][0]
    on_rec = next(q for q in p['band'] if num(F[q]['lf_a']) and F[q]['lf_a'] >= 0.5)
    qt = None
    for q in range(max(0, q_rec - 40), min(len(F), q_rec + 40)):
        f = F[q]; od = F[q - 1]['odo']   # lap_ff sees the lap_pos state of the previous frame (live_pid _L_lap_ff comment; verified: first logged lap_ff x_m = logged odo - d0)
        if not num(od) or od - GAP < win0: continue
        if not f['valid'] or not num(f['far']) or not num(f['near']) or f['near'] < -0.05: continue
        fe = f['far'] - (near_k * max(0., f['near'] - n0) if near_k else 0.)
        if fe >= 0.4: qt = q; break
    if qt is None: return None
    d0 = F[qt - 1]['odo']
    qo = next((q for q in range(qt, min(len(F), qt + 60)) if num(F[q - 1]['odo']) and F[q - 1]['odo'] - d0 - delay >= 0.5 * RAMP), None)
    if qo is None: return None
    return dict(qt=qt, qo=qo, s_trig=so[qt], s_on=so[qo], shift=so[qo] - so[on_rec], rec_on=so[on_rec], trig_reading=d0)


if __name__ == '__main__':
    P = []
    for run in RUNS:
        F = load(run); so = cont(F, 'odo'); p = curve_passes(F)[0]
        q0 = p['band'][0]; k = kmark(F, so, q0)
        ci = min(F[q]['clr_in'] for q in p['band'] if num(F[q]['clr_in']))
        P.append((run, F, so, p, k, ci))
    L = [__doc__, '', 'recorded: kmark s (m from the start) per lap-1 pass: ' + ' '.join(f'{r}:{k:.2f}' for r, F, so, p, k, ci in P) +
         f'  (sd {st.pstdev([k for *_, k, ci in P]):.3f}); later-lap median onset - kmark = 0.962 m']
    for win0, delay, nk in ((-1.2, 0.30, None), (-0.90, 0.30, None), (-0.85, 0.30, None), (-0.80, 0.30, None), (-0.75, 0.30, None), (-0.70, 0.30, None), (-0.65, 0.30, None), (-0.75, 0.35, None), (-1.2, 0.30, 0.5), (-1.2, 0.35, 0.5)):
        rows = []; est = []; onk = []
        for run, F, so, p, k, ci in P:
            r = sim(F, so, p, win0, delay, nk)
            if r is None: rows.append(f'{run}:none'); continue
            e = ci + SLOPE * r['shift']; est.append(e); onk.append(r['s_on'] - k)
            rows.append(f"{run}:trig@{r['trig_reading']:.2f} on-k {r['s_on'] - k:.2f} shift {r['shift']:+.2f} ci {ci:.1f}->{e:.1f}")
        L.append(f'win_m[0] {win0} (armed at odo reading >= {GAP + win0:.2f}) delay {delay} near_k {nk}: est clr_in median {st.median(est):.1f} min {min(est):.1f}; onset-kmark median {st.median(onk):.2f} range {min(onk):.2f}..{max(onk):.2f}')
        L.append('    ' + ' | '.join(rows))
    (OUT / 'C3_lap1_arming_sim.txt').write_text('\n'.join(L), encoding='utf8')
    print('\n'.join(L))
