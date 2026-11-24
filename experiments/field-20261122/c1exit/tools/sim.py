"""curve 1 + exit: SIMPLE closed-loop model (estimate only; limits below).  Laptop only.

  python sim.py base [N]                 today's chain (PID + gain schedule / right-entry + curve_smooth + yaw_inner + curve_ff by_curve + curve_trim + i_ramp exit + play comp)
  python sim.py <lap_ff.json> [N]        the same chain with that lap_ff.json (curve-1 band) on top
  options: --seed K  --trace (one pass, frame table)  --no-trim  --cff-off

car   : curvature[deg/m] = G * (w + U0), w = wheel position [us] = play(sent, PLAY us) delayed TD and lagged TAU (ident.py: G 0.19, zero -6 us, total 0.20 s)
lane  : straight, then one arc KL deg/m for ARC deg (tape R 1.275 m = 44.9 deg/m, 150 deg), then straight; s = car arc length from the arc start
state : psi = lane heading - car heading (deg, + = car points out / left), y cm (+ = outside): dy = 100 sin(psi) ds
camera: near = f(lane angle at the car) + B*y + C*psi (+ noise), seen CAM s late; f, B, C = ident.py least squares on the 34 logged passes (c1err.json);
        control error = near + 0.2 * (far - near), far - near = 0.75 inside the arc (logged), fading with the arc ahead
control: the real modules curve_ff.py / yaw_inner.py / curve_trim.py / play_comp.py / right_entry.py / lap_ff.py of the car, a re-written camera PID (kp ki kd, D on the near
        error, tau 0.15, right gain 1.45 / left 1.5), curve_map reduced to FLOOR from gz <= -12 until 115 deg, RELEASED after, landmarks C1s / C1e at 10 / 140 deg (+0.3 m hold),
        i_ramp exit C1e (start 0.3 m, tau 0.2).
Limits: one lane shape, linear camera model (resid sd 0.09), no slip / roll, no cmd_comp, no speed layers (speed profile given), lap_pos reduced to the two landmarks.
The model is accepted for comparison only if `base` reproduces the logged pattern (two command pulses per curve, late slam on a part of the passes, exit swing tied to it)."""
import sys, os, json, math, importlib.util
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
EXP = Path(os.environ.get('C1X_EXP', r'D:\OneDrive\Desktop\自走車\Orin_Interface_v1\laptop_workbench\data\experiments'))
SNAP = EXP / 'orin-web-snapshot-20261122'
LAPFF_PY = Path(os.environ.get('C1X_LAPFF', HERE.parent / 'base' / 'lap_ff.car_adcde919.py'))
sys.path.insert(0, str(SNAP))
import curve_ff, yaw_inner, curve_trim, play_comp, right_entry   # noqa: E402  (the car's modules, snapshot 20261122)


def _load(path, name):
    sp = importlib.util.spec_from_file_location(name, str(path)); m = importlib.util.module_from_spec(sp); sp.loader.exec_module(m); return m


lap_ff = _load(LAPFF_PY, 'lap_ff_c1x')
ERR = json.load(open(HERE / 'c1err.json')); ERR['f'] = ERR['free']['f']
DT = 1 / 30.; CENTRE = 1475.; LO = 780.; HI = 2050.
G0 = 0.19; U0 = 6.; TD = 0.12; TAU = 0.08; CAM = 0.066; PLAY = 130.
KL = 44.9; ARC = 150.; S_ARC = ARC / KL
KP, KI, KD, DTAU, RG, LG = 220., 40., 30., 0.15, 1.45, 1.5
B_ST = float(os.environ.get('C1X_BST', 0.034)); C_ST = float(os.environ.get('C1X_CST', 0.023))      # straight: ident_straight.py (gyro based, diagonal + main straight)
B_CV = float(os.environ.get('C1X_BCV', ERR['free']['b'])); C_CV = float(os.environ.get('C1X_CCV', ERR['free']['c']))   # inside the arc: ident.py model A (arc_lat based)


def f_lane(th):
    """centred / tangent near error at lane angle th (deg since the arc start); straight value outside."""
    ed = ERR['edges']; fc = ERR['f']; xs = [(a + b) / 2 for a, b in zip(ed, ed[1:])]
    if th <= -8: return 0.05
    if th >= 158: return 0.06
    return float(np.interp(th, [-8] + xs + [158], [0.05] + fc + [0.06]))


def lane_angle(s):
    return 0. if s <= 0 else min(ARC, s * KL)


class Cfg:
    def __init__(self, lapff_json=None, trim=True, cff=True, iramp=(0.3, 0.2), curve_trim_json=None, curve_ff_json=None):
        self.lapff_json = lapff_json; self.trim = trim; self.cff = cff; self.iramp = iramp
        self.ctj = curve_trim_json or SNAP / 'curve_trim.json'; self.cfj = curve_ff_json or SNAP / 'curve_ff.json'


def run(cfg, seed=0, y0=None, psi0=None, v_curve=None, gscale=None, foff=None, noise=0.02, lap=2, trace=False):
    rng = np.random.default_rng(seed)
    y = rng.normal(0., 3.) if y0 is None else y0; psi = rng.normal(0.5, 1.5) if psi0 is None else psi0
    vc = rng.uniform(1.05, 1.25) if v_curve is None else v_curve; G = G0 * (rng.normal(1., 0.07) if gscale is None else gscale); fo = rng.normal(0., 0.04) if foff is None else foff
    s = -2.6; tb = -psi; th_car0 = 0.   # tb = car heading turned right since the straight (deg)
    # controller objects (the car's modules)
    cff = curve_ff.CurveFF(json.loads(Path(cfg.cfj).read_text(encoding='utf8'))) if cfg.cff else None
    yin = yaw_inner.YawInner(); pc = play_comp.PlayComp()
    ct = curve_trim.build(cfg.ctj) if cfg.trim else None
    lff = None
    if cfg.lapff_json:
        c = lap_ff.load_config(cfg.lapff_json); lff = lap_ff.LapFF(c) if c and c.get('enabled') else None
    # PID state
    I = 0.5 * KI / KI * 0.5; I = 0.5   # integral state (unit*s), +20 us at the entry (logged +16..+26 on later laps)
    dlp = 0.; e_prev = None; near_prev = None
    cs = None; cm = 'IDLE'; cm_turn = 0.; anchor = ('C4e', -10.74 + 0.3 - 0.22, 2) ; D = 100.
    d_c4e = D + s - (10.74 - 0.22) ; c1s_d = None; c1e_d = None; anchor_name = 'C4e'; anchor_d = d_c4e
    w = 0.; w_lag = 0.; buf = [0.] * max(1, int(round(TD / DT))); ebuf = []
    gz_prev = 0.; t = 0.; out = []; i_t = None
    while s < S_ARC + 5.0 and t < 12.:
        v = vc if s < S_ARC - 0.2 + 0.8 else min(1.75, vc + (s - (S_ARC + 0.6)) * 0.33)
        th = lane_angle(s); psi = th - tb
        ahead = max(0., min(1., (S_ARC + 0.1 - s) / 0.5)) * max(0., min(1., (s + 0.6) / 0.5))   # 1 inside the arc (single-line frames), 0 on the straights
        bb = B_ST + (B_CV - B_ST) * ahead; cc_ = C_ST + (C_CV - C_ST) * ahead
        fl_ = f_lane(th) if 0. < s < S_ARC else (0.05 + (f_lane(0.) - 0.05) * max(0., min(1., (s + 0.6) / 0.6)) if s <= 0. else 0.06 + (f_lane(ARC) - 0.06) * max(0., 1. - (s - S_ARC) / 0.3))
        near_true = fl_ + fo * (1 if 0 < th < ARC else 0.3) + bb * y + cc_ * psi
        ebuf.append((near_true + rng.normal(0., noise), ahead)); k = max(0, len(ebuf) - 1 - int(round(CAM / DT))); near, ah = ebuf[k]
        near = max(-1., min(1., near)); far = near + 0.75 * ah; ectl = max(-1., min(1., near + 0.2 * (far - near)))
        gz = -G * (w_lag + U0) * v + rng.normal(0., 3.)
        # landmarks (lap_pos reduced): C1s at 10 deg, C1e at 140 deg, visible 0.3 m later
        D = 100. + s + 2.6
        if c1s_d is None and tb >= 10.: c1s_d = D
        if c1e_d is None and tb >= 140.: c1e_d = D
        if c1e_d is not None and D - c1e_d >= 0.3: anchor_name, anchor_d = 'C1e', c1e_d
        elif c1s_d is not None and D - c1s_d >= 0.3: anchor_name, anchor_d = 'C1s', c1s_d
        seg = 'curve1' if anchor_name == 'C1s' else ('main' if anchor_name == 'C4e' else 'diagonal_v')
        lp = dict(lap=lap, anchor=anchor_name, anchor_d=anchor_d, since_anchor_m=D - anchor_d, lost=False, odo_m=D, yaw_imu=-tb, gz_imu=gz)
        # curve_map reduced
        if cm == 'IDLE' and gz <= -12. and s > -0.5: cm = 'FLOOR'; cm_turn = 0.
        if cm != 'IDLE': cm_turn += -gz * DT
        if cm == 'FLOOR' and cm_turn >= 115.: cm = 'RELEASED'
        # i_ramp exit
        if cfg.iramp and anchor_name == 'C1e' and cfg.iramp[0] <= D - anchor_d <= 3.0 and I > 0: I *= math.exp(-DT / cfg.iramp[1])
        # PID (D on the near error)
        if near_prev is not None: dlp += DT / (DTAU + DT) * ((near - near_prev) / DT - dlp)
        near_prev = near
        I = max(-1., min(1., I + ectl * DT))
        u = KP * ectl + KI * I + KD * dlp; des = u * (RG if u >= 0 else LG)
        st = CENTRE + max(LO - CENTRE, min(HI - CENTRE, des))
        # gain schedule + right entry
        kk, _ = right_entry.k_for(0.57 if s > 0.2 else 0.66, st, CENTRE); st = CENTRE + (st - CENTRE) * kk
        # curve smooth
        if cm == 'FLOOR' and cs is not None: st = cs + (1. - math.exp(-DT / 0.15)) * (st - cs)
        cs = st
        # yaw inner (real class)
        in_curve = cm in ('FLOOR', 'RELEASED') or abs(gz) >= 15.
        st, yii = yin.step(t, gz, st, CENTRE, LO, HI, True, speed=v, bend=0., pitch=0., far=far, near=near, floor=-20. if cm == 'FLOOR' else None, in_curve=in_curve, map_curve=cm == 'FLOOR',
                           no_hold=bool(cm == 'RELEASED' and cm_turn >= 140.))
        chain = st
        if cff is not None: st, cfi = cff.step(st, near, CENTRE, LO, HI, True, now=t, gz=gz, path=None, curve=seg if seg == 'curve1' else None)
        binfo = None
        if lff is not None:
            st2, binfo = lff.step(lp, far, near, True, v, 0., -tb, st, CENTRE, LO, HI, True)
            if binfo is not None: st = st2
        if ct is not None: st, cti = ct.step(lp, near, st, CENTRE, LO, HI, True)
        pre = st - CENTRE
        sent, pci = pc.apply(st, CENTRE, LO, HI); sent -= CENTRE
        # car
        w = min(max(w, sent - PLAY / 2), sent + PLAY / 2)
        buf.append(w); wd = buf.pop(0); w_lag += (wd - w_lag) * (1. - math.exp(-DT / TAU))
        kc = G * (w_lag + U0); ds = v * DT
        out.append((t, s, tb, th, psi, y, near, ectl, pre, sent, gz, kc, KI * I, des, chain - CENTRE, 0. if binfo is None else (binfo.get('band') or {}).get('prof_us', CENTRE) - CENTRE,
                    1. if anchor_name == 'C1e' else 0., D - anchor_d, 0. if binfo is None else 1., v))
        tb += kc * ds; y += 100. * math.sin(math.radians(psi)) * ds; s += ds; t += DT
    return np.array(out)


COLS = ['t', 's', 'tb', 'th', 'psi', 'y', 'near', 'ectl', 'pre', 'sent', 'gz', 'kc', 'I', 'des', 'chain', 'prof', 'c1e', 'x', 'band', 'v']


def metrics(R):
    c = {k: R[:, i] for i, k in enumerate(COLS)}
    i0 = int(np.argmax(c['c1e'] > 0)) if (c['c1e'] > 0).any() else len(R) - 1; t0 = c['t'][i0]
    A = (c['t'] >= t0 - 1.2) & (c['t'] < t0); B = (c['t'] >= t0) & (c['t'] <= t0 + 2.5); Cv = (c['s'] > 0.1) & (c['t'] < t0)
    dpre = np.abs(np.diff(c['pre']))
    return dict(pk_sent=c['sent'][A].max(), pk_e=c['near'][A].max(), gz_min=c['gz'][A].min(), e0=c['near'][i0], psi0=c['psi'][i0], y0=c['y'][i0],
                pre_min=c['pre'][B].min(), pre_max=c['pre'][B].max(), e_min=c['near'][B].min(), e_max=c['near'][B].max(),
                y_in=c['y'][Cv].min(), y_out=c['y'][Cv].max(), yB_min=c['y'][B].min(), yB_max=c['y'][B].max(), sat=int((c['near'][Cv] >= 0.98).sum()),
                jump=dpre[(c['s'][1:] > 0.1)].max(), pre_sd=np.std(c['pre'][Cv & (c['s'] > 0.8)]), tcurve=t0 - c['t'][int(np.argmax(c['s'] > 0))])


def table(name, cfg, n, seed0=0):
    M = [metrics(run(cfg, seed=seed0 + k)) for k in range(n)]
    g = lambda k: np.array([m[k] for m in M])
    sw = g('pre_max') - g('pre_min')
    print('%-34s n %d | in curve: late pk_sent med %4.0f p90 %4.0f >=430: %2d%% | near>=0.98 passes %2d%% | y in %+5.1f (p10 %+5.1f) out %+5.1f (p90 %+5.1f) | cmd sd %3.0f max jump %3.0f'
          % (name, n, np.median(g('pk_sent')), np.percentile(g('pk_sent'), 90), 100 * np.mean(g('pk_sent') >= 430), 100 * np.mean(g('sat') > 0), np.median(g('y_in')), np.percentile(g('y_in'), 10),
             np.median(g('y_out')), np.percentile(g('y_out'), 90), np.median(g('pre_sd')), np.median(g('jump'))))
    print('%-34s      | exit 2.5 s: cmd swing med %4.0f p90 %4.0f >=200: %2d%% | e min med %+.2f p10 %+.2f | e max med %+.2f p90 %+.2f | y after min %+5.1f max %+5.1f | r(pk_sent, swing) %+.2f | psi0 med %+.1f sd %.1f'
          % ('', np.median(sw), np.percentile(sw, 90), 100 * np.mean(sw >= 200), np.median(g('e_min')), np.percentile(g('e_min'), 10), np.median(g('e_max')), np.percentile(g('e_max'), 90),
             np.percentile(g('yB_min'), 10), np.percentile(g('yB_max'), 90), np.corrcoef(g('pk_sent'), sw)[0, 1], np.median(g('psi0')), np.std(g('psi0'))))
    return M


if __name__ == '__main__':
    a = [x for x in sys.argv[1:] if not x.startswith('--')]; o = [x for x in sys.argv[1:] if x.startswith('--')]
    what = a[0] if a else 'base'; n = int(a[1]) if len(a) > 1 else 200
    cfg = Cfg(None if what == 'base' else what, trim='--no-trim' not in o, cff='--cff-off' not in o)
    seed = int(sys.argv[sys.argv.index('--seed') + 1]) if '--seed' in sys.argv else 0
    if '--trace' in o:
        R = run(cfg, seed=seed)
        print('   t     s     tb    psi     y   near  ectl   pre  sent    gz   I  chain  prof  x')
        for r in R[::2]:
            d = dict(zip(COLS, r))
            if d['s'] < -0.6 or d['s'] > S_ARC + 3.0: continue
            print(' %5.2f %5.2f %6.1f %+5.1f %+5.1f %+.2f %+.2f %+5.0f %+5.0f %+6.1f %+3.0f %+5.0f %+5.0f %4.2f %s' % (d['t'], d['s'], d['tb'], d['psi'], d['y'], d['near'], d['ectl'], d['pre'], d['sent'], d['gz'], d['I'], d['chain'], d['prof'], d['x'], 'C1e' if d['c1e'] else ''))
        print(metrics(R))
    else:
        table(what if what == 'base' else Path(what).name, cfg, n, seed)
