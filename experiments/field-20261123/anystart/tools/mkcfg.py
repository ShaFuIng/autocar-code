"""anystart mkcfg: rebuild the anystart config files from the car's current files and verify key by key.
Usage: python mkcfg.py --base <folder with the car's current lap_pos.json brake_zones.json lap_ff.json> --out <folder> [--noband]
Only the keys listed below change; everything else is copied byte-for-byte in value (json re-dumped with indent 1, ensure_ascii False).
"""
import json, argparse, hashlib, copy, sys
from pathlib import Path

FC_STD = 2.6   # first_curve odometer at today's fixed start (C1s crossing ~3.05 m - camera curve-ahead 0.44 m median, 12 starts 11-22)
SKIP_C1S = 9.0
BAND_WIN = [-2.2, 1.4]   # lap_ff c1.start: gap 3.2 -> trigger allowed at odometer 1.0 .. 4.6 m from the start

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def dump(obj, p):
    Path(p).write_text(json.dumps(obj, indent=1, ensure_ascii=False) + '\n', encoding='utf8')

def mk_lap_pos(c):
    c = copy.deepcopy(c)
    c['skip_after_by'] = dict(c.get('skip_after_by') or {}, C1s=SKIP_C1S)
    c['note_anystart'] = ('anystart-20261123: skip_after_by.C1s %.1f (was absent = skip_after_m 1.5). Lap 0: the car may start anywhere on the main '
                          'straight (C1s 2..~12 m ahead); with 1.5 a start > ~4.5 m before C1s gave SKIPPED C1s -> lap 1 placed at the nominal '
                          'position -> C1e missed -> LOST. Later laps: a C1s miss is skipped 9 m late instead of 1.5 m (no C1s* in 59 multi-lap '
                          'runs 11-20..11-22 except the IMU-frozen 25ca). Back = delete skip_after_by.C1s.') % SKIP_C1S
    return c

def mk_brake(c):
    c = copy.deepcopy(c); zs = []; ch = []
    for z in c['zones']:
        if z.get('anchor') == 'start' and not z.get('lap_anchor') and float(z.get('from_m', 0)) < 900:
            if float(z['from_m']) == 1.8 and float(z['to_m']) == 3.5:   # lap-1 curve-1 zone (start+1.8..3.5 cap 36) -> camera-anchored
                zs.append(dict(name='anystart: curve 1 on lap 1, from the first camera curve-ahead/error-large frame (first_curve): straight to the LOW '
                                    'target 24 for 0.5 m instead of the 30/s ramp', anchor='first_curve', from_m=0.0, to_m=0.5, cap_pwm=24))
                zs.append(dict(name='anystart: curve 1 on lap 1, then 36 (= the old start+1.8..3.5 cap) to ~C1s+0.5', anchor='first_curve', from_m=0.5, to_m=1.0, cap_pwm=36))
                ch.append('removed start 1.8..3.5 cap 36 -> first_curve 0..0.5 cap 24 + 0.5..1.0 cap 36'); continue
            z = dict(z); old = (z['from_m'], z['to_m'])
            z['anchor'] = 'first_curve'; z['from_m'] = round(float(z['from_m']) - FC_STD, 2); z['to_m'] = round(float(z['to_m']) - FC_STD, 2)
            z['name'] = z['name'] + ' [anystart: start+%.1f..%.1f -> first_curve+%.2f..%.2f]' % (old[0], old[1], z['from_m'], z['to_m'])
            ch.append('start %.1f..%.1f cap %s -> first_curve %.2f..%.2f' % (old[0], old[1], z['cap_pwm'], z['from_m'], z['to_m']))
        zs.append(z)
    c['zones'] = zs
    c['first_curve_min_m'] = 1.0
    c['note_anystart'] = ('anystart-20261123: the car may start anywhere on the main straight. All start-anchored zones re-anchored to first_curve '
                          '(first frame >= first_curve_min_m 1.0 m with speed reason ERROR_LARGE/CURVE_AHEAD and a positive error = curve 1, '
                          'median C1s-0.44 m, never on the straight in 13 starts + 18 later-lap straights 11-22); offsets = old - %.1f. '
                          'Back = brake_zones.json of snapshot 20261122c. Changes: %s') % (FC_STD, '; '.join(ch))
    return c

def mk_lap_ff(c, noband):
    c = copy.deepcopy(c); c1 = c['curves']['c1']
    if noband:
        c1.pop('start', None)
        c1['note_anystart'] = 'anystart-20261123 noband: curves.c1.start deleted = lap 1 curve 1 without the band (= the 11-22 morning chain; laps 2+ unchanged). Back = restore curves.c1.start {gap_m 3.2, win_m [-1.2, 1.0], pos_m null, ref_win [0.4, 1.8]}.'
    else:
        c1['start'] = dict(c1['start'], win_m=list(BAND_WIN))
        c1['note_anystart'] = ('anystart-20261123: curves.c1.start.win_m [-1.2, 1.0] -> [-2.2, 1.4] = the lap-1 band may trigger at odometer 1.0..4.6 m from the start '
                               '(start up to ~5 m before C1s keeps the band; farther starts pass the window -> no band on lap 1, the old chain). '
                               'Not the whole straight: a misfire = full-authority right turn on the straight. Back = [-1.2, 1.0].')
    return c

def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--base', required=True); ap.add_argument('--out', required=True); ap.add_argument('--noband', action='store_true')
    a = ap.parse_args(); B = Path(a.base); O = Path(a.out); O.mkdir(parents=True, exist_ok=True)
    rd = lambda n: json.loads((B / n).read_text(encoding='utf8'))
    out = {'lap_pos.json': mk_lap_pos(rd('lap_pos.json')), 'brake_zones.json': mk_brake(rd('brake_zones.json')), 'lap_ff.json': mk_lap_ff(rd('lap_ff.json'), a.noband)}
    for n, obj in out.items():
        dump(obj, O / n)
        # key-by-key verification
        old = rd(n); new = json.loads((O / n).read_text(encoding='utf8'))
        diff = []
        def walk(x, y, p):
            if isinstance(x, dict) and isinstance(y, dict):
                for k in sorted(set(x) | set(y)):
                    if k not in x: diff.append(('+', p + '.' + k))
                    elif k not in y: diff.append(('-', p + '.' + k))
                    else: walk(x[k], y[k], p + '.' + k)
            elif x != y: diff.append(('~', p))
        walk(old, new, n)
        print('%s  base %s  new %s' % (n, sha(B / n)[:8], sha(O / n)[:8]))
        for d in diff: print('   ', *d)
    return 0

if __name__ == '__main__': sys.exit(main())
