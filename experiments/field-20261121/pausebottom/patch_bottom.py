"""pause-resume-r1g (2026-11-21, user: curve 4 about 1 m earlier): pause / resume allowed inside the bottom_owner scope once the car is
bottom_pause_min_m past LAND8 (runs bab2 / 1012 / 9284: the right cap is fully OPEN from LAND8+1.5..2.0 m, pitch calm, lap_ff4 steering).
usage: python patch_bottom.py <pause_resume.py d7f46820> <out>"""
import hashlib, sys
src, dst = sys.argv[1], sys.argv[2]
s = open(src, encoding='utf-8', newline='').read()
assert hashlib.sha256(s.encode('utf-8')).hexdigest().startswith('d7f46820'), 'unexpected base'


def rep(old, new):
    global s
    assert s.count(old) == 1, (s.count(old), old[:70]); s = s.replace(old, new)


rep("                upper_post_turn_resume=False)   # pause-resume-r1b",
    "                bottom_pause_min_m=None,   # pause-resume-r1g: metres after LAND8 from which the bottom_owner scope no longer refuses a pause / blocks a resume\n"
    "                                           # (needs lap_pos anchored on LAND8 and not lost; None = the whole scope blocks, as before)\n"
    "                upper_post_turn_resume=False)   # pause-resume-r1b")
rep("def _tail_blocks(lp, cfg):\n",
    "def _bottom_ok(tp, cfg):\n"
    "    \"\"\"r1g: inside the bottom_owner scope, far enough after the landing for a pause?\"\"\"\n"
    "    mn = cfg.get('bottom_pause_min_m'); a = getattr(tp, 'anchor', None) if tp is not None else None\n"
    "    return bool(isinstance(mn, (int, float)) and not isinstance(mn, bool) and tp is not None and not tp.lost and a and a[0] == 'LAND8' and tp.D - a[1] >= float(mn))\n"
    "\n\n"
    "def _tail_blocks(lp, cfg):\n")
rep("    if getattr(lp, 'bown', None) is not None and getattr(lp.bown, 'active', False): return 'ON_SLOPE'\n",
    "    if getattr(lp, 'bown', None) is not None and getattr(lp.bown, 'active', False) and not _bottom_ok(tp, cfg): return 'ON_SLOPE'\n")
rep(") or c.get('bottom0'):", ") or (c.get('bottom0') and not c.get('bottom_ok0')):")
rep("                 bottom0=bool(getattr(lp, 'bown', None) is not None and getattr(lp.bown, 'active', False)),",
    "                 bottom_ok0=_bottom_ok(tp, cfg),\n"
    "                 bottom0=bool(getattr(lp, 'bown', None) is not None and getattr(lp.bown, 'active', False)),")
open(dst, 'w', encoding='utf-8', newline='').write(s)
print('written', dst, hashlib.sha256(s.encode('utf-8')).hexdigest()[:8])
