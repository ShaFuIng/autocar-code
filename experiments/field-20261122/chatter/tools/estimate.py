"""chatter-20261122 open-loop estimate (recorded images / IMU / speed; the car's response is the recorded one).
python estimate.py OUT.txt RUN [RUN ...]
Scenarios (only the full-authority band frames are recomputed; every other frame keeps its logged pre-compensation command):
  ou       output_unify method smooth 0.05 s + threshold 10 (web/output_unify.json; code on the car since 11-17), mcu_ou = both
  base     logged commands (detector replica on the logged pre = check of the method)
  mcu      wheel speed from the MCU status clock (speed_meter.json mode 'mcu'): v_new rebuilt from inputs.veh_ms / enc_b
  vref108  curve-2 band at a fixed speed v_ref 1.08 (lap_ff.json curves.c2.band.v_ref, config only)
Band terms that follow the speed: profile c_eff + kappa/g(v) (c1/c2 lap_ff band, c4 band; c1 already v_ref 1.10 -> unchanged),
gyro term clip(k_gz*(gz + v*kappa), +-lim) (c1/c2/c4: k 1.5 lim 60; c3 xband: k 1.0 lim 40). c3 xband: profile u_x does not use v;
its x_dr distance extrapolation and the 700 us/s rate limit are NOT recomputed (gyro term only). land4 is not recomputed.
c_eff is taken per frame as prof - kappa/g (logged). gz_imu: from the unclipped logged gyro term, else inputs.gz.
Outputs per region: flips/s, sent |delta| p95, pre |delta| p95, mean command shift (new - logged) on recomputed frames."""
import sys, json, collections, statistics
from pathlib import Path
from common import load, num, pct

EXP = Path(__file__).resolve().parents[3]
LF = json.loads((EXP / 'orin-web-snapshot-20261122c' / 'lap_ff.json').read_text(encoding='utf8'))
GT = sorted((float(a), float(b)) for a, b in LF['gain_table'])
M_PER = json.loads((EXP / 'orin-web-snapshot-20261122c' / 'vehicle_model.json').read_text(encoding='utf8')).get('m_per_count', 2.22e-4)
WIN = 0.6; TH = 15.; OFF = 55.


def gain(v):
    if v <= GT[0][0]: return GT[0][1]
    if v >= GT[-1][0]: return GT[-1][1]
    for (v0, g0), (v1, g1) in zip(GT, GT[1:]):
        if v0 <= v <= v1: return g0 + (g1 - g0) * (v - v0) / (v1 - v0)
    return GT[-1][1]


def v_mcu(F):
    """speed from (MCU ms, count) pairs, one entry per new status, rate over WIN s of MCU time (speed_meter mode 'mcu')."""
    h = []; out = []; last = None
    for f in F:
        inp = f['raw'].get('inputs') or {}
        ms = inp.get('veh_ms'); c = inp.get('enc_b')
        if num(ms) and num(c) and ms != last:
            last = ms; t = ms / 1000.; h.append((t, float(c))); h = [x for x in h if t - x[0] <= 1.]
        if len(h) < 2: out.append(None); continue
        t1, c1 = h[-1]; old = [x for x in h if t1 - x[0] >= WIN]; t0, c0 = old[-1] if old else h[0]
        out.append(None if t1 - t0 < .05 else abs(c1 - c0) / (t1 - t0) * M_PER)
    return out


def recompute(f, vnew_fn):
    """returns new pre (or None = unchanged) for a full-authority band frame."""
    p = f['raw']; pre = f['pre']
    a = p.get('lap_ff4')
    if isinstance(a, dict) and isinstance(a.get('band'), dict) and a['band'].get('state') == 'ON' and (a['band'].get('own') or 0) >= 0.99 \
            and num(a['band'].get('band_us')) and num(pre) and abs(a['band']['band_us'] - pre) < 0.6:
        b = a['band']; vo = a.get('v'); vn = vnew_fn('c4', f)
        if not (num(vo) and num(vn) and num(a.get('g')) and num(b.get('kappa')) and num(b.get('prof_us')) and num(b.get('gz_us'))): return None
        kap = b['kappa']; ce = b['prof_us'] - kap / a['g']; pn = ce + kap / gain(vn)
        gzi = b['gz_us'] / 1.5 - vo * kap if abs(b['gz_us']) < 59.95 else f['gz']
        gn = max(-60., min(60., 1.5 * (gzi + vn * kap))) if num(gzi) else b['gz_us']
        return pre + (pn - b['prof_us']) + (gn - b['gz_us'])
    a = p.get('arc2_c3')
    if isinstance(a, dict) and isinstance(a.get('xband'), dict):
        b = a['xband']
        if (b.get('own') or 0) >= 0.99 and num(b.get('target_us')) and num(pre) and abs(b['target_us'] - pre) < 0.6:
            vo = a.get('v'); vn = vnew_fn('c3', f)
            if not (num(vo) and num(vn) and num(b.get('kappa')) and num(b.get('gz_us'))): return None
            kap = b['kappa']; gzi = b['gz_us'] / 1.0 - vo * kap if abs(b['gz_us']) < 39.95 else f['gz']
            gn = max(-40., min(40., 1.0 * (gzi + vn * kap))) if num(gzi) else b['gz_us']
            return pre + (gn - b['gz_us'])
        return None
    a = p.get('lap_ff')
    if isinstance(a, dict) and isinstance(a.get('band'), dict) and a['band'].get('own') and num(a.get('steer_us')) and num(pre) and abs(a['steer_us'] - pre) < 0.6:
        cv = a.get('curve'); b = a['band']
        if cv == 'c1': return None   # v_ref 1.10 already
        vo = a.get('v'); vn = vnew_fn(cv, f)
        if not (num(vo) and num(vn) and num(a.get('g')) and num(b.get('kappa')) and num(b.get('prof_us')) and num(b.get('gz_us'))): return None
        kap = b['kappa']; ce = b['prof_us'] - kap / a['g']; pn = ce + kap / gain(vn)
        gzi = b['gz_us'] / 1.5 - vo * kap if abs(b['gz_us']) < 59.95 else f['gz']
        gn = max(-60., min(60., 1.5 * (gzi + vn * kap))) if num(gzi) else b['gz_us']
        return pre + (pn - b['prof_us']) + (gn - b['gz_us'])
    return None


HOLD_ROWS = ('UPPER_SINGLE_HOLD', 'LANE_BLIND_HOLD', 'HEADING_BLIND', 'PID_HOLD', 'DOWNHILL_HOLD', 'LOST_HEADING', 'LOST_TURN')


def ou_smooth(F, pres, tau=0.05, dt_max=0.3):
    """output_unify method 'smooth' stage 1 (web/output_unify.json; code already on the car): first-order low pass with time constant
    tau on the capture clock; bypass rows (hold rows, WARMUP, dash TURN, heading-hold TURN, start centring) pass and reset the filter."""
    import math
    out = []; s = None; t0 = None
    for i, f in enumerate(F):
        x = pres[i]; p = f['raw']
        if f['sent'] is None or not num(x): out.append(x); continue
        t = f['cap'] if num(f['cap']) else f['t']
        byp = p.get('valid') is False or p.get('reason') in HOLD_ROWS or p.get('reason') == 'WARMUP' or p.get('warmup_held')             or isinstance(p.get('dash_turn'), dict) or (isinstance(p.get('heading_hold'), dict) and p['heading_hold'].get('reason') == 'TURN')
        dt = None if (t0 is None or not num(t)) else t - t0
        if num(t): t0 = t
        if byp or dt is None or dt <= 0 or dt > dt_max: s = x; out.append(x); continue
        s = s + (1. - math.exp(-dt / tau)) * (x - s); out.append(s)
    return out


def detect(F, pres, TH=TH):
    d = 0; ext = 1475.; revs = []; sent = []
    for i, f in enumerate(F):
        raw = pres[i]
        if f['sent'] is None or not num(raw): revs.append(False); sent.append(None); continue
        sw = False
        if d == 0:
            if raw - ext >= TH: d = 1; ext = raw; sw = True
            elif ext - raw >= TH: d = -1; ext = raw; sw = True
        elif d > 0:
            if raw >= ext: ext = raw
            elif ext - raw >= TH: d = -1; ext = raw; sw = True
        else:
            if raw <= ext: ext = raw
            elif raw - ext >= TH: d = 1; ext = raw; sw = True
        revs.append(sw); sent.append(raw + OFF * d)
    return revs, sent


def stats(F, pres, revs, sent, shifts):
    R = collections.defaultdict(lambda: dict(n=0, rev=0, t=0., ds=[], dp=[], sh=[]))
    for i, f in enumerate(F):
        r = f['reg']
        if r is None: continue
        S = R[r]; S['n'] += 1; S['rev'] += revs[i]
        if i and F[i - 1]['reg'] == r and num(f['t']) and num(F[i - 1]['t']) and 0 < f['t'] - F[i - 1]['t'] <= 0.1:
            S['t'] += f['t'] - F[i - 1]['t']
            if num(sent[i]) and num(sent[i - 1]): S['ds'].append(abs(sent[i] - sent[i - 1]))
            if num(pres[i]) and num(pres[i - 1]): S['dp'].append(abs(pres[i] - pres[i - 1]))
        if i in shifts: S['sh'].append(shifts[i])
    return R


def main():
    out = open(sys.argv[1], 'w', encoding='utf8')
    agg = collections.defaultdict(lambda: collections.defaultdict(lambda: [0, 0., [], [], []]))
    for run in sys.argv[2:]:
        F = load(run); vm = v_mcu(F)
        vo = [None] * len(F)
        for i, f in enumerate(F):
            p = f['raw']
            for k in ('lap_ff4', 'lap_ff', 'arc2_c3'):
                a = p.get(k)
                if isinstance(a, dict) and num(a.get('v')): vo[i] = a['v']; break
        dv = [vm[i] - vo[i] for i in range(len(F)) if num(vm[i]) and num(vo[i])]
        out.write(f'{run}: v(mcu) - v(logged): median {statistics.median(dv):+.3f} mean {statistics.mean(dv):+.3f} p10 {pct(dv, .1):+.3f} p90 {pct(dv, .9):+.3f} m/s (n {len(dv)})\n')
        scen = dict(base=lambda c, f: None, mcu=lambda c, f: vm[f['_i']], vref108=lambda c, f: 1.08 if c == 'c2' else None,
                    ou=lambda c, f: None, mcu_ou=lambda c, f: vm[f['_i']])
        for i, f in enumerate(F): f['_i'] = i
        for name, fn in scen.items():
            pres = [f['pre'] for f in F]; shifts = {}
            if name not in ('base', 'ou'):
                for i, f in enumerate(F):
                    if fn('c2', f) is None and fn('c4', f) is None: continue
                    n = recompute(f, fn)
                    if n is not None: shifts[i] = n - f['pre']; pres[i] = n
            if name in ('ou', 'mcu_ou'):
                pres = ou_smooth(F, pres); revs, sent = detect(F, pres, 10.)
            else: revs, sent = detect(F, pres)
            if name == 'base':
                mm = sum(1 for i, f in enumerate(F) if f['sent'] is not None and revs[i] != f['pc_rev'])
                out.write(f'   base detector vs logged reversal flags: {mm} frames differ\n')
            R = stats(F, pres, revs, sent, shifts)
            for r, S in R.items():
                A = agg[name][r]; A[0] += S['rev']; A[1] += S['t']; A[2] += S['ds']; A[3] += S['dp']; A[4] += S['sh']
    out.write('\nall runs given (open loop):\n')
    for r in sorted(agg['base']):
        for name in ('base', 'mcu', 'vref108', 'ou', 'mcu_ou'):
            A = agg[name][r]
            if not A[1]: continue
            sh = A[4]
            out.write(f'  {r:7s} {name:6s} flips/s {A[0] / A[1]:5.2f}  sent|d| p50 {pct(A[2], .5):5.1f} p95 {pct(A[2], .95):6.1f}  pre|d| p95 {pct(A[3], .95):6.1f}'
                      + (f'  recomputed {len(sh):5d}  shift mean {statistics.mean(sh):+6.1f} p05 {pct(sh, .05):+6.1f} p95 {pct(sh, .95):+6.1f}' if sh else '') + '\n')
    out.close()


if __name__ == '__main__':
    main()
