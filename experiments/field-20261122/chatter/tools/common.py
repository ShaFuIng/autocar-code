"""chatter-20261122 common loader: compact frames (extract.py output) -> list of flat dicts with region labels.
Region rules (one label per frame, first match wins):
  c4own   lap_ff4.band.state ON and band.x? -> lap_ff4.x_m >= 2.1 (falls in the full-authority window of the new runs; old runs: same
          distance window, band not owning)            -> label 'c4win'
  land    land4.state LAND and land4.applied
  c3x     arc2_c3 present, distance (xband.x_m if present else arc2_c3.x_m) in [0.9, 4.05]
  c3tail  lap_pos.seg curve3 after c3x in the same lap (corner + tail to the ramp)
  c1      lap_pos.seg curve1 ;  c2  lap_pos.seg curve2
  main / diag / circle / upper / ramp : lap_pos.seg main_straight / diagonal_v / circle_straight / upper_straight / ramp,
          only frames not taken above and with no band owning the output.
  exit windows (separate, overlapping labels): C1e / C2e / C4e anchor, since_anchor_m 0..3.
"""
import json, math
from pathlib import Path

SCR = Path(r'C:/Users/User/AppData/Local/Temp/claude/1eeee7db7bd1a8b1/02d05eda-955f-4b7f-90df-1bfa92030e70/scratchpad/cx')
SEGMAP = dict(main_straight='main', diagonal_v='diag', circle_straight='circle', upper_straight='upper', ramp='ramp')


def num(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def load(run, src=SCR):
    F = []
    for l in open(Path(src) / f'{run}.jsonl', encoding='utf8'):
        d = json.loads(l); p = d['p']
        pc = p.get('play_comp') or {}
        lp = p.get('lap_pos') or {}
        inp = p.get('inputs') or {}
        yi = p.get('yaw_inner') or {}
        al = p.get('arc_lat') or {}
        gs = p.get('gain_schedule') or {}
        f = dict(t=p.get('t') if num(p.get('t')) else d.get('cap'), cap=d.get('cap'), fs=d.get('fs'), valid=p.get('valid'), reason=p.get('reason'), owner=p.get('owner'),
                 cseg=p.get('seg'), pre=p.get('steer_us'), sent=pc.get('steer_us'), pc_off=pc.get('offset_us'), pc_rev=bool(pc.get('reversal')),
                 pc_dir=pc.get('direction'), pc_raw=pc.get('raw_us'), err=p.get('image_error'), perr=p.get('error'),
                 P=p.get('p'), I=p.get('i'), D=p.get('d'), k=gs.get('k'), pwm=p.get('pwm'),
                 gz=inp.get('gz'), fb=inp.get('steer_cmd'), lap=lp.get('lap'), seg=lp.get('seg'), anchor=lp.get('anchor'),
                 sa=lp.get('since_anchor_m'), odo=lp.get('odo_m'), v=yi.get('speed_mps'), yterm=yi.get('term_us'), yu=yi.get('u_us'),
                 y=al.get('y_cm') if al.get('ok') else None, clr_in=al.get('clr_in_cm'), clr_out=al.get('clr_out_cm'),
                 cs=(p.get('curve_smooth') or {}).get('steer_us'), sa_reason=(p.get('speed_adapt') or {}).get('reason'),
                 sa_level=(p.get('speed_adapt') or {}).get('level'), sa_derr=(p.get('speed_adapt') or {}).get('derr'),
                 ls=p.get('lap_speed'), pitch=(p.get('imu_boost') or {}).get('pitch_rel_deg'), raw=p)
        F.append(f)
    label(F)
    return F


def band_of(p):
    """(name, own_flag, comps dict, target) for the band owning / shaping this frame, or None."""
    a = p.get('lap_ff4')
    if isinstance(a, dict) and isinstance(a.get('band'), dict) and a['band'].get('state') == 'ON':
        b = a['band']
        return ('c4', (b.get('own') or 0) >= 0.99, dict(prof=b.get('prof_us'), e=b.get('e_us'), gz=b.get('gz_us')), b.get('target_us'), a.get('x_m'))
    a = p.get('arc2_c3')
    if isinstance(a, dict) and isinstance(a.get('xband'), dict):
        b = a['xband']
        return ('c3', (b.get('own') or 0) >= 0.99, dict(prof=b.get('prof_us'), e=b.get('e_us'), gz=b.get('gz_us'), h=b.get('h_us'), y=b.get('y_us')),
                b.get('target_us'), b.get('x_m'))
    a = p.get('lap_ff')
    if isinstance(a, dict) and isinstance(a.get('band'), dict):
        b = a['band']
        return (a.get('curve'), bool(b.get('own')), dict(prof=b.get('prof_us'), e=b.get('e_us'), gz=b.get('gz_us')), None, a.get('x_m'))
    return None


def label(F):
    seen_c3x = {}
    for f in F:
        p = f['raw']; f['band'] = band_of(p); reg = None
        a4 = p.get('lap_ff4'); l4 = p.get('land4'); c3 = p.get('arc2_c3')
        if isinstance(a4, dict) and isinstance(a4.get('band'), dict) and a4['band'].get('state') == 'ON' and num(a4.get('x_m')) and a4['x_m'] >= 2.1:
            reg = 'c4win'
        elif isinstance(l4, dict) and l4.get('state') == 'LAND' and l4.get('applied'):
            reg = 'land'
        elif isinstance(c3, dict):
            xb = c3.get('xband'); x = xb.get('x_m') if isinstance(xb, dict) else c3.get('x_m')
            if num(x) and 0.9 <= x <= 4.05: reg = 'c3x'; seen_c3x[f['lap']] = True
        if reg is None and f['seg'] == 'curve3' and seen_c3x.get(f['lap']): reg = 'c3tail'
        if reg is None and f['seg'] == 'curve1': reg = 'c1'
        if reg is None and f['seg'] == 'curve2': reg = 'c2'
        if reg is None and f['seg'] in SEGMAP:
            b = f['band']
            if not (b and b[1]) and f['owner'] not in ('lap_ff4', 'lap_ff', 'arc2_flat', 'land4'): reg = SEGMAP[f['seg']]
        f['reg'] = reg
        ex = None
        if f['anchor'] in ('C1e', 'C2e', 'C4e') and num(f['sa']) and 0 <= f['sa'] <= 3.0: ex = f['anchor']
        f['exit'] = ex


def instances(F, key='reg'):
    """contiguous (lap, label) runs -> list of (label, lap, [frame indices]); gaps of > 3 frames split."""
    out = []; cur = None
    for i, f in enumerate(F):
        r = f[key]
        if r is None: continue
        if cur and cur[0] == r and cur[1] == f['lap'] and i - cur[2][-1] <= 4: cur[2].append(i)
        else:
            cur = [r, f['lap'], [i]]; out.append(cur)
    return [tuple(c) for c in out]


def pct(a, q):
    a = sorted(a)
    if not a: return None
    k = (len(a) - 1) * q; lo = int(math.floor(k)); hi = min(lo + 1, len(a) - 1)
    return a[lo] + (a[hi] - a[lo]) * (k - lo)


def hp(t, x, win=0.5):
    """x minus centred moving average over win seconds (only finite pairs)."""
    out = []
    for i in range(len(x)):
        s = n = 0
        for j in range(len(x)):
            if abs(t[j] - t[i]) <= win / 2: s += x[j]; n += 1
        out.append(x[i] - s / n)
    return out


def std(a):
    if len(a) < 2: return None
    m = sum(a) / len(a); return math.sqrt(sum((v - m) ** 2 for v in a) / (len(a) - 1))


def period(t, x):
    """main period (s) from zero crossings of the high-passed signal; None if < 2 crossings."""
    zc = [t[i] for i in range(1, len(x)) if (x[i - 1] < 0) != (x[i] < 0)]
    if len(zc) < 3: return None
    return 2 * (zc[-1] - zc[0]) / (len(zc) - 1)
