"""c1lap1b: stream run-XXXX-20261122/pid.jsonl -> compact per-frame jsonl (proposal minus bulky keys + vision arc_fit/capture/seq).
Usage: python extract.py OUTDIR RUN [RUN ...]. Offline only. Based on chatter/tools/extract.py."""
import json, sys, os
from pathlib import Path
from multiprocessing import Pool
EXP = Path(__file__).resolve().parents[3]
DROP = ('turn_preview', 'sample_bounds', 'sampling_version', 'visual_center_version', 'requested_row_ratio', 'sample_row_ratio',
        'sample_reason', 'derivative_rebased', 'sample_clamped', 'backlash_version', 'backlash_offset_us', 'backlash_direction',
        'backlash_state', 'backlash_reversals', 'backlash_source', 'backlash_initial_count', 'hardware_output')
dec = json.JSONDecoder()
def one(args):
    out, run = args
    src = EXP / f'run-{run}-20261122' / 'pid.jsonl'; dst = Path(out) / f'{run}.jsonl'; n = 0
    with open(src, encoding='utf8') as f, open(dst, 'w', encoding='utf8') as g:
        for l in f:
            if not l.startswith('{"kind": "sample"'): continue
            j = l.find('"proposal": ')
            if j < 0: continue
            p, _ = dec.raw_decode(l, j + 12)
            tp = p.get('turn_preview') or {}
            for k in DROP: p.pop(k, None)
            p['tp'] = dict(near=tp.get('near_error'), far=tp.get('far_projected_error'), ctl=tp.get('control_error'), why=tp.get('reason'))
            bz = p.get('brake_zones')
            if isinstance(bz, dict) and isinstance(bz.get('zone'), dict):
                z = bz['zone']; bz['zone'] = dict(name=str(z.get('name'))[:40], cap=z.get('cap_pwm', z.get('max_pwm')))
            ex = {}
            for key, off in (('capture_at', 14), ('frame_seq', 13)):
                k = l.find('"%s": ' % key)
                if k > 0: ex[key], _ = dec.raw_decode(l, k + off)
            k = l.find('"arc_fit": {')
            if 0 < k < j:
                try: ex['arc_fit'], _ = dec.raw_decode(l, k + 11)
                except Exception: pass
            k = l.find('"wall": ')
            if 0 < k < 60: ex['wall'], _ = dec.raw_decode(l, k + 8)
            g.write(json.dumps(dict(x=ex, p=p), separators=(',', ':')) + '\n'); n += 1
    return run, n
if __name__ == '__main__':
    out = sys.argv[1]; os.makedirs(out, exist_ok=True)
    with Pool(4) as pool:
        for r in pool.imap_unordered(one, [(out, r) for r in sys.argv[2:]]): print(*r, flush=True)
