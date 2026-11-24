"""c3front: write the candidate config files from the car's current files (snapshot 20261121c). Only adds / changes what is listed.
usage: python mkcfg.py probes   -> proof/probes/probe_<pwm>_to<end>.json (lap_speed zone variants)
       python mkcfg.py on       -> on/lap_speed.c3front_s1_40.json (40, C3s 0.3..2.0) / c3front_s2_48.json (48, 0.3..1.6), on/arc2.c3flat50.json / c3flat100.json"""
import copy, hashlib, json, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent; ROOT = HERE.parent; EXP = HERE.parents[2]; SNAP = EXP / 'orin-web-snapshot-20261121c'
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()[:8]


def zone(pwm, to_m, eg=0.6, sg=300.0):
    return dict(name='c3 front %d (C3s+0.3..%.1f m; c3front-20261121: camera line R ~2.45 m, driven R ~2.2 m, PWM 32 today; ends %.1f m before the corner peak C3s+4.84 so speed_adapt is back at 32 by ~C3s+%.1f)' % (pwm, to_m, 4.84 - to_m, to_m + (min(pwm, 50) - 32) / 30. * 1.05),
                anchor='C3s', from_m=0.3, to_m=to_m, pwm=pwm, allow_low=True, err_guard=eg, steer_guard_us=sg)


def lap_speed(pwm, to_m, eg=0.6, note=True):
    c = json.loads((SNAP / 'lap_speed.json').read_text(encoding='utf8')); c['zones'].append(zone(pwm, to_m, eg))   # last in the list: no other zone covers these positions (the C2e zones end at C2e+6.3 m, C3s is at C2e+10.2..10.9)
    if note: c['note_c3front_20261121'] = ('c3front-20261121 offline candidate (NOT deployed when written): + one zone at the end of the list "c3 front %d" (anchor C3s, 0.3..%.1f m, allow_low, err_guard %.1f, steer_guard 300). '
                                           'Everything else = car lap_speed.json %s. Back out: delete that zone.' % (pwm, to_m, eg, sha(SNAP / 'lap_speed.json')))
    return c


def arc2(w):
    """curve-3 floor kappa_table: the entries from x 0.425 to 3.675 m moved a fraction w towards their own mean (same integral = same turn)"""
    c = json.loads((SNAP / 'arc2.json').read_text(encoding='utf8')); t = c['c3']['kappa_table']
    idx = [i for i, (x, k) in enumerate(t) if 0.4 <= x <= 3.7]; m = sum(t[i][1] for i in idx) / len(idx)
    old = copy.deepcopy(t)
    for i in idx: t[i][1] = round(t[i][1] + w * (m - t[i][1]), 1)
    c['c3']['note_c3front_20261121'] = ('c3front-20261121 offline candidate (NOT deployed when written): kappa_table x 0.425..3.675 m moved %d %% towards its mean %.1f deg/m (old: %s). '
                                        'The table was the median DRIVEN turn of 11-14..11-16 = the two-step turning itself (12 deg/m at 0.4-0.9 and 2.7-3.4, 40-50 at 1.4-1.9); the camera line there is within +-4 cm of one arc. '
                                        'x <= 0.175 and x >= 3.925 (corner, tail, top) unchanged. Back out = car arc2.json %s.' % (round(w * 100), m, json.dumps([old[i][1] for i in idx]), sha(SNAP / 'arc2.json')))
    return c, m


def dump(obj, path):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(obj, ensure_ascii=False, indent=1) + '\n', encoding='utf8'); print('wrote', Path(path).relative_to(ROOT), sha(path))


if __name__ == '__main__':
    what = sys.argv[1] if len(sys.argv) > 1 else 'probes'
    if what == 'probes':
        for pwm in (40, 48):
            for to in (1.6, 2.0, 2.4, 2.8, 3.2, 3.6): dump(lap_speed(pwm, to, note=False), ROOT / 'proof' / 'probes' / ('probe_%d_to%.1f.json' % (pwm, to)))
        for eg in (0.5, 0.7): dump(lap_speed(48, 3.2, eg, note=False), ROOT / 'proof' / 'probes' / ('probe_48_to3.2_eg%.1f.json' % eg))
    else:
        dump(lap_speed(40, 2.0), ROOT / 'on' / 'lap_speed.c3front_s1_40.json'); dump(lap_speed(48, 1.6), ROOT / 'on' / 'lap_speed.c3front_s2_48.json')
        for w, nm in ((0.5, 'c3flat50'), (1.0, 'c3flat100')):
            c, m = arc2(w); dump(c, ROOT / 'on' / ('arc2.%s.json' % nm))
