"""open-loop replay, one pass frame by frame: what the car sent (reference replay) and what the step would have sent on the same recorded input.
  python show_pass.py <replay out dir> <tag> <run> <lap> [C1e|C2e] [t_from] [t_to]"""
import sys, json, numpy as np
from pathlib import Path
D = Path(sys.argv[1]); tag, run, lap = sys.argv[2], sys.argv[3], int(sys.argv[4]); an = sys.argv[5] if len(sys.argv) > 5 else 'C1e'
a = float(sys.argv[6]) if len(sys.argv) > 6 else -3.0; b = float(sys.argv[7]) if len(sys.argv) > 7 else 1.8
C = 1475.


def load(p):
    with open(p, encoding='utf8') as f:
        f.readline(); return [json.loads(l) for l in f]


O = load(D / 'off' / ('%s.old.jsonl' % run)); N = load(D / tag / ('%s.new.jsonl' % run))
anc = [(r['proposal'].get('lap_pos') or {}).get('anchor') for r in O]; laps = [(r['proposal'].get('lap_pos') or {}).get('lap') for r in O]
k0 = next(k for k in range(1, len(O)) if anc[k] == an and anc[k - 1] != an and laps[k] == lap); t0 = O[k0]['proposal']['t']
print('%s lap %d %s, tag %s (open loop: same recorded camera / gyro / encoder; the car reaction to the new command is not modelled)' % (run, lap, an, tag))
print('    dt   e_near    gz | car: pre  sent | new: pre  sent | band: state   a    tb   prof  e_us  gz_us  e_c | I old -> new')
for o, n in zip(O, N):
    p = o['proposal']; q = n['proposal']
    if 't' not in p: continue
    dt = p['t'] - t0
    if dt < a or dt > b: continue
    lf = q.get('lap_ff') or {}; bd = lf.get('band') or {}
    print(' %6.2f  %+.2f  %+6.1f | %+5.0f %+5.0f | %+5.0f %+5.0f | %-5s %4s %6s %5s %5s %5s %5s | %+3.0f -> %+3.0f' % (
        dt, p.get('image_error') or 0, (p.get('inputs') or {}).get('gz') or 0, (o['pre_steer'] or C) - C, (o['sent_steer'] or C) - C, (n['pre_steer'] or C) - C, (n['sent_steer'] or C) - C,
        lf.get('state', '-') if lf.get('curve') else '-', '%.2f' % lf['a'] if 'a' in lf else '-', '%.1f' % bd['turned_deg'] if bd else '-', '%+.0f' % (bd['prof_us'] - C) if bd else '-',
        '%+.0f' % bd['e_us'] if bd else '-', '%+.0f' % bd['gz_us'] if bd else '-', '%.2f' % bd['e_c'] if bd else '-', p.get('i') or 0, q.get('i') or 0))
