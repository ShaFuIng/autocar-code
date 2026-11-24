"""band-ec-turn-r1 (2026-11-22): the band's error centre as a table over the turned angle.
Static measurement 2026-11-22 (car placed by hand on the lane centre, field-20261122/centred/poses.jsonl): the camera near error of a
CENTRED car changes along curve 2 (0.26 at the entry, 0.56 at 40 deg, 0.77 at 80 deg, 0.49-0.68 after it) and the driving error has the
same shape (run d60e: 0.36 / 0.55 / 0.75 / 0.50 / 0.43 / 0.32 per 30 deg). With one fixed e_c the band reads the 60-90 deg bump as
'outside' while the car is 6 cm inside, and band + PID + curve_ff push it to +370..+460 us every lap.
New optional key in a band block (curves.<cN>.band and c4.band):  "e_c_turn": [[turned_deg, e_c], ...]  -> ec = interp(table, turned)
(no kappa scaling, no out_cm dial). Key absent / not a usable table = the old formula, frame for frame.
usage: python patch_ecturn.py <lap_ff.py 0f739ce5> <out lap_ff.py>"""
import hashlib, sys
from pathlib import Path

s = Path(sys.argv[1]).read_bytes().decode('utf-8')
assert hashlib.sha256(s.encode('utf-8')).hexdigest().startswith('0f739ce5'), 'unexpected base'
NL = chr(13) + chr(10) if chr(13) + chr(10) in s else chr(10)


def rep(old, new):
    global s
    old, new = old.replace(chr(10), NL), new.replace(chr(10), NL)
    assert s.count(old) == 1, (s.count(old), old[:70]); s = s.replace(old, new)


rep("def interp(table, x):\n",
    "def ec_turn(b, tb):\n"
    "    \"\"\"band-ec-turn-r1: error centre from b['e_c_turn'] at the turned angle tb, or None (key absent / unusable -> the old formula).\"\"\"\n"
    "    t = b.get('e_c_turn') if isinstance(b, dict) else None\n"
    "    if not isinstance(t, (list, tuple)) or len(t) < 2 or not _num(tb): return None\n"
    "    try:\n"
    "        if not all(len(q) == 2 and _num(q[0]) and _num(q[1]) and -1. <= float(q[1]) <= 1. for q in t): return None\n"
    "        return float(interp(t, float(tb)))\n"
    "    except (TypeError, ValueError): return None\n"
    "\n\n"
    "def interp(table, x):\n")
rep("        et = max(-float(b['e_lo_us']), min(float(b['e_hi_us']), float(b['k_e_us']) * (float(near) - ec))) if (valid and _num(near)) else 0.\n"
    "        gz = (lp or {}).get('gz_imu'); gt = 0.\n",
    "        ect = ec_turn(b, tb)   # band-ec-turn-r1\n"
    "        if ect is not None: ec = ect\n"
    "        et = max(-float(b['e_lo_us']), min(float(b['e_hi_us']), float(b['k_e_us']) * (float(near) - ec))) if (valid and _num(near)) else 0.\n"
    "        gz = (lp or {}).get('gz_imu'); gt = 0.\n")
rep("        et = max(-float(B['e_lo_us']), min(float(B['e_hi_us']), float(B['k_e_us']) * (float(near) - ec))) if (valid and _num(near)) else 0.\n"
    "        cap = min(float(B['cap_us']), hi)\n",
    "        ect = ec_turn(B, tb)   # band-ec-turn-r1 (c4 band)\n"
    "        if ect is not None: ec = ect\n"
    "        et = max(-float(B['e_lo_us']), min(float(B['e_hi_us']), float(B['k_e_us']) * (float(near) - ec))) if (valid and _num(near)) else 0.\n"
    "        cap = min(float(B['cap_us']), hi)\n")
Path(sys.argv[2]).write_bytes(s.encode('utf-8')); print('written', sys.argv[2], hashlib.sha256(s.encode('utf-8')).hexdigest()[:8])
