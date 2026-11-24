"""chatter-20261122: does a play-comp flip (sent +-110 us) move the car more than the command change itself?
python fliptest.py OUT.txt RUN [RUN ...]
Events (frames inside a labelled region, valid, both frames with play_comp):
  FLIP   pre moved 15..40 us against the previous direction and the compensation changed side on this frame (sent moved by
         110 us + the pre move).
  SAME   pre moved 15..40 us over the last 1..3 frames in the direction the compensation already holds (no flip in that window;
         sent moved by the pre move only).
Response = gz(t + tau) - gz(t), sign-aligned so that + = yawed the way the command moved (right = gz down), divided by the pre move
(deg/s per us). If the flip moved the wheel by its 110 us, FLIP would be ~(110 + d)/d = 4..8 x SAME; if the compensation only takes
up the linkage play, FLIP ~ SAME. Random-time baseline shown. Caution: the pre move itself partly follows gz (gyro terms), the
same for both groups."""
import sys, statistics, random, collections
from common import load, num, pct


def gz_at(F, ts, t0):
    lo, hi = 0, len(F) - 1
    while lo < hi:
        m = (lo + hi) // 2
        if ts[m] < t0: lo = m + 1
        else: hi = m
    return F[lo]['gz'] if num(F[lo]['gz']) and abs(ts[lo] - t0) < 0.06 else None


def main():
    out = open(sys.argv[1], 'w', encoding='utf8'); res = collections.defaultdict(list); rnd = random.Random(7)
    for run in sys.argv[2:]:
        F = load(run); ts = [f['t'] if num(f['t']) else -1e9 for f in F]
        for i in range(3, len(F)):
            f = F[i]
            if f['reg'] is None or not f['valid'] or f['sent'] is None or not num(f['gz']) or not num(f['pre']): continue
            grp = 'band' if f['reg'] in ('c2', 'c4win', 'c3x', 'land') else 'other'
            ev = None
            d1 = f['pre'] - F[i - 1]['pre'] if num(F[i - 1]['pre']) else None
            if f['pc_rev'] and d1 is not None and 15 <= abs(d1) <= 40 and (1 if d1 > 0 else -1) == f['pc_dir']:
                ev = ('FLIP', d1)
            elif not f['pc_rev']:
                for k in (1, 2, 3):
                    g = F[i - k]
                    if not num(g['pre']) or any(F[j]['pc_rev'] for j in range(i - k + 1, i + 1)): break
                    d = f['pre'] - g['pre']
                    if 15 <= abs(d) <= 40 and (1 if d > 0 else -1) == f['pc_dir'] and g['pc_dir'] == f['pc_dir']:
                        ev = ('SAME', d); break
            for tau in (0.25, 0.35, 0.5):
                g1 = gz_at(F, ts, f['t'] + tau)
                if g1 is None: continue
                if ev: res[(grp, ev[0], tau)].append(-(g1 - f['gz']) / ev[1])
                if rnd.random() < 0.05:
                    res[(grp, 'RANDOM', tau)].append(-(g1 - f['gz']) / (20. if rnd.random() < .5 else -20.))
    out.write(__doc__ + '\n')
    for k in sorted(res):
        v = res[k]
        out.write(f'{k[0]:5s} {k[1]:6s} tau {k[2]:.2f}  n {len(v):5d}  mean {statistics.mean(v):+.3f}  median {statistics.median(v):+.3f}  '
                  f'(deg/s per us; x20 us = {statistics.mean(v) * 20:+.1f} deg/s)\n')
    out.close()


if __name__ == '__main__':
    main()
