"""pause-resume-r1f (2026-11-21, user: pause earlier in curve 4): the 4 s window after the downhill may stop blocking pause / resume.
usage: python patch_tail.py <pause_resume.py ae9fce8e> <out>"""
import hashlib, sys
src, dst = sys.argv[1], sys.argv[2]
s = open(src, encoding='utf-8', newline='').read()
assert hashlib.sha256(s.encode('utf-8')).hexdigest().startswith('ae9fce8e'), 'unexpected base'


def rep(old, new):
    global s
    assert s.count(old) == 1, (s.count(old), old[:70]); s = s.replace(old, new)


rep("                upper_post_turn_resume=False)   # pause-resume-r1b",
    "                down_tail_blocks=True,   # pause-resume-r1f (2026-11-21, run bab2b7f8: bottom_owner ends at LAND8+3.4 m, the 4 s down tail at +4.0..4.6 m): false = the\n"
    "                                         # down tail alone no longer refuses a pause / blocks a resume; the landing stays covered by the bottom_owner scope, the\n"
    "                                         # slope by pitch and lap_pos. Ignored (tail still blocks) when there is no bottom_owner object.\n"
    "                upper_post_turn_resume=False)   # pause-resume-r1b")
rep("def pause_block(lp, cfg=None):\n",
    "def _tail_blocks(lp, cfg):\n"
    "    \"\"\"r1f: does the down-tail window count as a slope? (default yes; no only with down_tail_blocks false AND a bottom_owner object)\"\"\"\n"
    "    return cfg.get('down_tail_blocks') is not False or getattr(lp, 'bown', None) is None\n"
    "\n\n"
    "def pause_block(lp, cfg=None):\n")
rep("    if getattr(lp, 'down_until', None) is not None and _pclk.monotonic() <= lp.down_until: return 'ON_SLOPE'\n",
    "    if getattr(lp, 'down_until', None) is not None and _pclk.monotonic() <= lp.down_until and _tail_blocks(lp, cfg): return 'ON_SLOPE'\n")
rep("or c.get('down_tail0') or c.get('bottom0'):", "or (c.get('down_tail0') and c.get('tail_blocks0', True)) or c.get('bottom0'):")
rep("                 down_tail0=bool(getattr(lp, 'down_until', None) is not None and now_v <= lp.down_until),\n",
    "                 down_tail0=bool(getattr(lp, 'down_until', None) is not None and now_v <= lp.down_until), tail_blocks0=_tail_blocks(lp, cfg),\n")
open(dst, 'w', encoding='utf-8', newline='').write(s)
print('written', dst, hashlib.sha256(s.encode('utf-8')).hexdigest()[:8])
