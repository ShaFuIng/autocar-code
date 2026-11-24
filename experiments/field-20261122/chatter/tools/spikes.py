"""chatter-20261122: one-frame spikes of the wheel-speed reading (downhill_speed.SpeedMeter via _wheel_cps) and the play-comp flips
they cause. python spikes.py OUT.txt RUN [RUN ...]
spike at frame i: v[i] - max(v[i-1], v[i+1]) > 0.10 m/s (up-spike) or min(v[i-1], v[i+1]) - v[i] > 0.10 (down-spike); v = the speed the
band / lap_ff layer logged on that frame (lap_ff4.v, lap_ff.v, arc2_c3.v; else yaw_inner.speed_mps).
spike-related flip: a play_comp reversal on the spike frame or the frame after it.
Also: the MCU status period (inputs.veh_ms steps) and how often a new status coincides with a spike (mechanism check)."""
import sys, collections, statistics
from common import load, num, pct


def vv(f):
    p = f['raw']
    for k in ('lap_ff4', 'lap_ff', 'arc2_c3'):
        a = p.get(k)
        if isinstance(a, dict) and num(a.get('v')): return a['v']
    return f['v']


def main():
    out = open(sys.argv[1], 'w', encoding='utf8')
    tot = collections.defaultdict(lambda: [0, 0, 0, 0.])
    for run in sys.argv[2:]:
        F = load(run); V = [vv(f) for f in F]
        ms = [(f['raw'].get('inputs') or {}).get('veh_ms') for f in F]
        sp = set(); up = 0; mag = []; newstat = 0
        for i in range(1, len(F) - 1):
            a, b, c = V[i - 1], V[i], V[i + 1]
            if not (num(a) and num(b) and num(c)): continue
            if b - max(a, c) > 0.10: sp.add(i); up += 1; mag.append(b - max(a, c))
            elif min(a, c) - b > 0.10: sp.add(i); mag.append(min(a, c) - b)
            if i in sp and num(ms[i]) and num(ms[i - 1]) and ms[i] != ms[i - 1]: newstat += 1
        steps = [ms[i] - ms[i - 1] for i in range(1, len(ms)) if num(ms[i]) and num(ms[i - 1]) and ms[i] != ms[i - 1]]
        frames_new = sum(1 for i in range(1, len(ms)) if num(ms[i]) and num(ms[i - 1]) and ms[i] != ms[i - 1])
        out.write(f'{run}: frames {len(F)}  speed spikes {len(sp)} (up {up})  = {len(sp) / max(1, len(F)) * 100:.1f}% of frames, '
                  f'{len(sp) / max(1e-9, (F[-1]["t"] - F[0]["t"])):.2f}/s; size median {statistics.median(mag) if mag else 0:.3f} m/s p90 {pct(mag, .9) or 0:.3f}; '
                  f'MCU status step median {statistics.median(steps) if steps else 0} ms; frames with a new status {frames_new}; spikes on a new-status frame {newstat}\n')
        reg = collections.defaultdict(lambda: [0, 0, 0, []])
        for i, f in enumerate(F):
            r = f['reg']
            if r is None: continue
            R = reg[r]; R[0] += 1
            rel = (i in sp) or (i - 1 in sp)
            if f['pc_rev']:
                R[1] += 1; R[2] += rel
            if i in sp and num(f['pre']) and num(F[i - 1]['pre']): R[3].append(f['pre'] - F[i - 1]['pre'])
        for r, R in sorted(reg.items()):
            d = R[3]
            out.write(f'   {r:7s} frames {R[0]:5d} flips {R[1]:4d} spike-related {R[2]:4d} ({R[2] / max(1, R[1]):.0%})  spikes {len(d):3d}  '
                      f'pre step at spike median {statistics.median([abs(x) for x in d]) if d else 0:5.1f} p90 {pct([abs(x) for x in d], .9) or 0:5.1f} us\n')
            T = tot[r]; T[0] += R[1]; T[1] += R[2]; T[2] += len(d)
    out.write('\nall runs given: region flips spike-related share\n')
    for r, T in sorted(tot.items()):
        out.write(f'   {r:7s} flips {T[0]:5d} spike-related {T[1]:5d} ({T[1] / max(1, T[0]):.0%}) spikes {T[2]}\n')
    out.close()


if __name__ == '__main__':
    main()
