"""chatter-20261122: stream run-XXXX-20261122/pid.jsonl and write a compact per-frame jsonl (no vision mask) for the chatter analysis.
Usage: python extract.py OUTDIR RUN [RUN ...]   (RUN = 4-char id, e.g. 9af6). Offline only; reads the experiments folder.
Each output line: {"w": wall, "cap": vision.capture_at, "fs": vision.frame_seq, "p": proposal minus bulky keys}."""
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
    src = EXP / f'run-{run}-20261122' / 'pid.jsonl'
    dst = Path(out) / f'{run}.jsonl'
    n = 0
    with open(src, encoding='utf8') as f, open(dst, 'w', encoding='utf8') as g:
        for l in f:
            if not l.startswith('{"kind": "sample"'): continue
            j = l.find('"proposal": ')
            if j < 0: continue
            p, _ = dec.raw_decode(l, j + 12)
            for k in DROP: p.pop(k, None)
            bz = p.get('brake_zones')
            if isinstance(bz, dict) and isinstance(bz.get('zone'), dict):
                z = bz['zone']; bz['zone'] = dict(name=str(z.get('name'))[:40], cap=z.get('cap_pwm', z.get('max_pwm')))
            cap = fs = None
            k = l.find('"capture_at": ')
            if k > 0: cap, _ = dec.raw_decode(l, k + 14)
            k = l.find('"frame_seq": ')
            if k > 0: fs, _ = dec.raw_decode(l, k + 13)
            w = None
            k = l.find('"wall": ')
            if k > 0 and k < 60: w, _ = dec.raw_decode(l, k + 8)
            g.write(json.dumps(dict(w=w, cap=cap, fs=fs, p=p), separators=(',', ':')) + '\n'); n += 1
    return run, n


if __name__ == '__main__':
    out = sys.argv[1]; os.makedirs(out, exist_ok=True)
    with Pool(4) as pool:
        for r in pool.imap_unordered(one, [(out, r) for r in sys.argv[2:]]): print(*r, flush=True)
