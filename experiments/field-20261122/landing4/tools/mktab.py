"""target tables from the good landings: theta*(u) (deg, relative to the upper-straight heading, left +), camera reference e*(u), speed v(u).
u = metres since the landing detection (negative on the slope). Writes tables.json and prints them."""
from desc import *
import statistics as S, json, pickle
from pathlib import Path
R = all_desc()
GOOD_OUT, GOOD_IN = -4.0, -9.0
US = [round(-2.2 + 0.1 * k, 2) for k in range(0, 73)]   # -2.2 .. 5.0


def ref_of(d):
    F = d['F']; im = d['i_mouth'] - d['a']; tm = d['t'][im]
    up = [float(g(p, 'heading_hold', 'yaw_deg')) for n, p in enumerate(F[:im]) if g(p, 'heading_hold', 'yaw_deg') is not None and tm - d['t'][n] <= 1.2]
    return sum(up) / len(up)


def series(d, fn):
    out = []
    for n, p in enumerate(d['F']):
        x = d['x'][n] if d.get('x') else None
        v = fn(n, p)
        if x is not None and v is not None: out.append((x, v))
    return out


def at(sr, u, win=0.06):
    v = [y for x, y in sr if abs(x - u) <= win]
    if v: return sum(v) / len(v)
    lo = [(x, y) for x, y in sr if x < u]; hi = [(x, y) for x, y in sr if x > u]
    if lo and hi and hi[0][0] - lo[-1][0] < 0.5:
        (x0, y0), (x1, y1) = lo[-1], hi[0]; return y0 + (y1 - y0) * (u - x0) / (x1 - x0)
    return None


L = []
for d in R:
    if d['i_land'] is None: continue
    d['ref'] = ref_of(d); il = d['i_land'] - d['a']
    co = [g(p, 'arc_lat', 'clr_out_cm') for n, p in enumerate(d['F']) if n >= il and d['x'][n] <= 3.0 and g(p, 'arc_lat', 'ok') and g(p, 'arc_lat', 'clr_out_cm') is not None]
    ci = [g(p, 'arc_lat', 'clr_in_cm') for n, p in enumerate(d['F']) if n >= il and d['x'][n] <= 3.5 and g(p, 'arc_lat', 'ok') and g(p, 'arc_lat', 'clr_in_cm') is not None]
    d['co'] = min(co) if co else None; d['ci'] = min(ci) if ci else None
    d['good'] = d['co'] is not None and d['ci'] is not None and d['co'] >= GOOD_OUT and d['ci'] >= GOOD_IN and d['run'] != '46c9'
    d['s_yaw'] = series(d, lambda n, p: None if d['yaw'][n] is None else d['yaw'][n] - d['ref'])
    d['s_e'] = series(d, lambda n, p: p.get('image_error') if p.get('reason') == 'PROPOSAL_ONLY' else None)
    d['s_v'] = series(d, lambda n, p: g(p, 'yaw_inner', 'speed_mps') or g(p, 'lap_ff4', 'v'))
    d['s_cmd'] = series(d, lambda n, p: None if p.get('steer_us') is None else p['steer_us'] - C0)
    L.append(d)
good = [d for d in L if d['good']]
print('good landings (outer clearance >= %.0f cm over x<=3.0 and inner >= %.0f cm over x<=3.5; 46c9 excluded): %d of %d' % (GOOD_OUT, GOOD_IN, len(good), len(L)))
print(' ', [(d['run'], d['lap'], d['co'], d['ci']) for d in good])
print('others:', [(d['run'], d['lap'], d['co'], d['ci']) for d in L if not d['good']])


def med(ds, key, u):
    v = [at(d[key], u) for d in ds]; v = [x for x in v if x is not None]
    return (S.median(v), S.pstdev(v), len(v)) if len(v) >= 3 else (None, None, len(v))


def smooth(tab, w):
    out = []
    for u, y in tab:
        v = [yy for uu, yy in tab if abs(uu - u) <= w + 1e-9]
        out.append((u, sum(v) / len(v)))
    return out


raw = [(u, med(good, 's_yaw', u)) for u in US]
th = [(u, m[0]) for u, m in raw if m[0] is not None]
# slope part: hold 0 until -1.6 m then a straight ramp to the landing value (the good median is within ~1 deg of this)
y0 = dict(th)[0.0]
th2 = []
for u, y in th:
    if u <= -1.6: y = 0.0
    elif u < 0.0: y = y0 * (u + 1.6) / 1.6
    th2.append((u, y))
ths = smooth(th2, 0.1)
ths = [(u, (0.0 if u <= -1.7 else y)) for u, y in ths]
kap = []
for k, (u, y) in enumerate(ths):
    a = ths[max(0, k - 1)]; b = ths[min(len(ths) - 1, k + 1)]
    kap.append((u, -(b[1] - a[1]) / (b[0] - a[0])))
kap = smooth(kap, 0.1)
er = [(u, med(good, 's_e', u)[0]) for u in US]; er = smooth([(u, e) for u, e in er if e is not None], 0.1)
vv = [(u, med(L, 's_v', u)[0]) for u in US]; vv = [(u, v) for u, v in vv if v is not None]
cm = [(u, med(good, 's_cmd', u)[0]) for u in US]
print('\n   u     theta_good(med sd n)   theta*   kappa*   e_ref    v_all   cmd_good(median pre-comp, us from centre)   theta_all(med)')
ea = dict(er); va = dict(vv); ca = dict(cm); ka = dict(kap); ta = dict(ths)
for u, m in raw:
    al = med(L, 's_yaw', u)
    print('%5.1f   %7s %5s %3d   %7.1f  %6.1f  %6s  %6s   %6s   %7s' % (u, 'None' if m[0] is None else '%.1f' % m[0], 'None' if m[1] is None else '%.1f' % m[1], m[2], ta.get(u, float('nan')), ka.get(u, float('nan')),
                                                                 'None' if ea.get(u) is None else '%.2f' % ea[u], 'None' if va.get(u) is None else '%.2f' % va[u], 'None' if ca.get(u) is None else '%.0f' % ca[u], 'None' if al[0] is None else '%.1f' % al[0]))
T = dict(theta=[[u, round(y, 1)] for u, y in ths if -1.8 <= u <= 3.0], kappa=[[u, round(max(0., k), 1)] for u, k in kap if -1.8 <= u <= 3.0],
         e_ref=[[u, round(e, 2)] for u, e in er if -1.6 <= u <= 3.0], v=[[u, round(v, 2)] for u, v in vv if -2.0 <= u <= 3.0],
         good=[[d['run'], d['lap']] for d in good], n_all=len(L))
json.dump(T, open(Path(__file__).with_name('tables.json'), 'w'), indent=0)
