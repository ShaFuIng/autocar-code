"""anystart mkstart: shift every start-odometer-bound setting by the measured start distance D (m, from the car's front tyre edge... no:
D = distance from the start position to the curve-1 start used by today's fixed start, i.e. the same reference as lap_pos start_s -3.44).
Usage: python mkstart.py --dist D --base <folder with the car's current lap_pos.json brake_zones.json lap_ff.json start_bias.json> --out <folder>
       [--c1s-skip 9.0]   add lap_pos skip_after_by.C1s (insurance: a wrong D does not shift the whole lap; recommended)
D = 3.44 and no --c1s-skip -> every output file is a byte copy of the input.
Changed keys (delta = D - 3.44):
  lap_pos.json      start_s = -D                         (+ skip_after_by.C1s with --c1s-skip)
  brake_zones.json  every zone with anchor 'start' and from_m < 900: from_m/to_m + delta (also when it is the fallback part of a lap zone)
  lap_ff.json       curves.c1.start.gap_m + delta; ref_win[1] = min(1.8, D - 0.9) (only changes for D < 2.7: keep the heading plateau off the curve)
                    win_m unchanged (relative to gap_m)
  start_bias.json   only for D < 3.44: hold_s shortened by the time the car needs for the missing metres (start-ramp table of 12 starts
                    11-22), so the bias still ends ~0.47 s before the C1s crossing; fade_s unchanged; hold_s >= 0.6
  brake_zones first_curve_min_m: unchanged (no zone uses first_curve in the car's current file; it is log only)
"""
import json, argparse, hashlib, copy, shutil, sys
from pathlib import Path
REF = 3.44
# median time (s) from the end of the start hold (PWM > 0) to odometer x (m), 12 starts 11-22 (min/max within +-0.15 s)
T_ODO = [(0.0, 0.0), (1.0, 1.89), (1.5, 2.33), (2.0, 2.79), (2.6, 3.27), (3.0, 3.67)]
C1S_CROSS_BEFORE = 0.44   # the C1s crossing (heading 10 deg) is ~0.44 m before the nominal curve-1 position used by start_s (s_err -0.35..-0.46)

def t_at(x):
    for (x0, t0), (x1, t1) in zip(T_ODO, T_ODO[1:]):
        if x <= x1: return t0 + (t1 - t0) * (x - x0) / (x1 - x0)
    x0, t0 = T_ODO[-1]; return t0 + (x - x0) / 1.15

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def dump(o, p): Path(p).write_text(json.dumps(o, indent=1, ensure_ascii=False) + '\n', encoding='utf8')

def build(B, D, skip):
    d = round(D - REF, 3); rd = lambda n: json.loads((B / n).read_text(encoding='utf8')); out = {}
    lp = rd('lap_pos.json'); lp2 = copy.deepcopy(lp)
    if d != 0: lp2['start_s'] = -round(D, 3)
    if skip is not None: lp2['skip_after_by'] = dict(lp2.get('skip_after_by') or {}, C1s=float(skip))
    out['lap_pos.json'] = (lp, lp2)
    bz = rd('brake_zones.json'); bz2 = copy.deepcopy(bz)
    for z in bz2['zones']:
        if z.get('anchor') == 'start' and float(z.get('from_m', 999)) < 900:
            z['from_m'] = round(float(z['from_m']) + d, 3); z['to_m'] = round(float(z['to_m']) + d, 3)
    out['brake_zones.json'] = (bz, bz2)
    lf = rd('lap_ff.json'); lf2 = copy.deepcopy(lf); st = lf2['curves']['c1'].get('start')
    if isinstance(st, dict):
        st['gap_m'] = round(float(st['gap_m']) + d, 3)
        if st.get('ref_win') and float(st['ref_win'][1]) > D - 0.9: st['ref_win'] = [st['ref_win'][0], round(max(float(st['ref_win'][0]) + 0.5, D - 0.9), 2)]
    out['lap_ff.json'] = (lf, lf2)
    if (B / 'start_bias.json').exists():
        sb = rd('start_bias.json'); sb2 = copy.deepcopy(sb)
        if d < 0:
            dt = t_at(REF - C1S_CROSS_BEFORE) - t_at(max(0., D - C1S_CROSS_BEFORE))
            sb2['hold_s'] = round(max(0.6, float(sb['hold_s']) - dt), 2)
        out['start_bias.json'] = (sb, sb2)
    return out

def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--dist', type=float, required=True); ap.add_argument('--base', required=True); ap.add_argument('--out', required=True)
    ap.add_argument('--c1s-skip', type=float, default=None)
    a = ap.parse_args(); B = Path(a.base); O = Path(a.out); O.mkdir(parents=True, exist_ok=True)
    if not 1.9 <= a.dist <= 12.5: print('D outside 2..12 m: not supported'); return 2
    for n, (old, new) in build(B, a.dist, a.c1s_skip).items():
        if new == old: shutil.copyfile(B / n, O / n)
        else: dump(new, O / n)
        diff = []
        def walk(x, y, p):
            if isinstance(x, dict) and isinstance(y, dict):
                for k in sorted(set(x) | set(y)):
                    if k not in x: diff.append('+ %s = %s' % (p + '.' + k, json.dumps(y[k])))
                    elif k not in y: diff.append('- ' + p + '.' + k)
                    else: walk(x[k], y[k], p + '.' + k)
            elif isinstance(x, list) and isinstance(y, list) and len(x) == len(y) and any(isinstance(v, dict) for v in x):
                for i, (u, v) in enumerate(zip(x, y)): walk(u, v, '%s[%d]' % (p, i))
            elif x != y: diff.append('~ %s: %s -> %s' % (p, json.dumps(x), json.dumps(y)))
        walk(old, json.loads((O / n).read_text(encoding='utf8')), n)
        print('%s  base %s  out %s  %s' % (n, sha(B / n)[:8], sha(O / n)[:8], 'BYTE-IDENTICAL' if sha(B / n) == sha(O / n) else ''))
        for x in diff: print('   ', x)
    return 0

if __name__ == '__main__': sys.exit(main())
