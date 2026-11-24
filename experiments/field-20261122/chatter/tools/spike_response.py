"""chatter-20261122: natural experiment. A wheel-speed spike (one frame, +0.34 m/s, a timing artefact unrelated to the car's state)
makes the full-authority band send a one-frame command pulse (curve 2 median +55..66 us, curve 4 +28 us) and usually two play-comp
flips (sent pulse up to ~165-220 us for one frame). Does the car yaw after it?
python spike_response.py OUT.txt RUN [RUN ...]
Event-averaged gz(t0 + tau) - gz(t0 - 0.033) for spike frames inside c2 / c4win / c3x (band owning), against all other frames of the
same regions (baseline). Sign: + = yaw to the right (the way the pulse pushes; gz right = negative, so the sign is flipped).
Expected if the wheel followed a 50 us one-frame servo step (1500 us/s slew): a bump of order 1-3 deg/s around tau 0.25-0.4 s."""
import sys, statistics, collections
from common import load, num
from spikes import vv


def main():
    out = open(sys.argv[1], 'w', encoding='utf8'); out.write(__doc__ + '\n')
    taus = [-0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.7]
    acc = collections.defaultdict(lambda: collections.defaultdict(list)); npulse = collections.defaultdict(list)
    for run in sys.argv[2:]:
        F = load(run); V = [vv(f) for f in F]; ts = [f['t'] if num(f['t']) else -1e9 for f in F]
        def gz_at(t0):
            lo, hi = 0, len(F) - 1
            while lo < hi:
                m = (lo + hi) // 2
                if ts[m] < t0: lo = m + 1
                else: hi = m
            return F[lo]['gz'] if num(F[lo]['gz']) and abs(ts[lo] - t0) < 0.05 else None
        for i in range(2, len(F) - 2):
            f = F[i]; r = f['reg']
            if r not in ('c2', 'c4win', 'c3x') or not num(f['t']): continue
            b = f['band']
            if not (b and b[1]): continue
            a, c = V[i - 1], V[i + 1]
            sp = num(a) and num(c) and num(V[i]) and V[i] - max(a, c) > 0.10
            g0 = gz_at(f['t'] - 0.033)
            if g0 is None: continue
            kind = 'SPIKE' if sp else 'other'
            if sp and num(f['pre']) and num(F[i - 1]['pre']): npulse[r].append(f['pre'] - F[i - 1]['pre'])
            for tau in taus:
                g = gz_at(f['t'] + tau)
                if g is not None: acc[(r, kind)][tau].append(-(g - g0))
    for k in sorted(acc):
        out.write(f'{k[0]:6s} {k[1]:6s} ' + '  '.join(f'{tau:+.1f}s {statistics.mean(v):+5.2f}({len(v)})' for tau, v in sorted(acc[k].items())) + '\n')
    out.write('\npulse size (pre step on the spike frame, us): ' + '  '.join(f'{r} median {statistics.median(v):+.1f} n {len(v)}' for r, v in npulse.items()) + '\n')
    # difference spike - other with a rough standard error
    out.write('\nspike minus baseline (deg/s, +- 2 standard errors):\n')
    for r in ('c2', 'c4win', 'c3x'):
        S = acc.get((r, 'SPIKE')); O = acc.get((r, 'other'))
        if not S or not O: continue
        row = []
        for tau in taus:
            s, o = S[tau], O[tau]
            if len(s) < 5: continue
            d = statistics.mean(s) - statistics.mean(o); se = (statistics.pstdev(s) ** 2 / len(s) + statistics.pstdev(o) ** 2 / len(o)) ** .5
            row.append(f'{tau:+.1f}s {d:+5.2f}+-{2 * se:4.2f}')
        out.write(f'  {r:6s} ' + '  '.join(row) + '\n')
    out.close()


if __name__ == '__main__':
    main()
