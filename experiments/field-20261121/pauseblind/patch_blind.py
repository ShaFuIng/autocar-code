"""pause-resume-r1e (2026-11-21): on the upper straight after the dashed turn a resume no longer needs a valid path (fresh image only).
usage: python patch_blind.py <pause_resume.py 8bd1c783> <out>"""
import hashlib, sys
src, dst = sys.argv[1], sys.argv[2]
s = open(src, encoding='utf-8', newline='').read()
assert hashlib.sha256(s.encode('utf-8')).hexdigest().startswith('8bd1c783'), 'unexpected base'


def rep(old, new):
    global s
    assert s.count(old) == 1, (s.count(old), old[:70]); s = s.replace(old, new)


rep("                upper_post_turn_resume=False)   # pause-resume-r1b",
    "                upper_post_blind_resume=False,   # pause-resume-r1e (2026-11-21, run c981aee5: standing at DASHe+1.66 m the worker has no path, 974 white pixels):\n"
    "                                                 # true = a pause taken on the upper straight after the turn may be resumed without a valid path (the image must\n"
    "                                                 # still be fresh); the car then starts on the existing invalid-frame rules (upper-single gyro HOLD, <= hold_max_s)\n"
    "                upper_post_turn_resume=False)   # pause-resume-r1b")
rep("    if not (lt.get('valid') is True and lp.consecutive >= 3 and _pclk.monotonic() - lt.get('capture_at', -1e18) <= lp.run['parameters']['max_gap']):\n"
    "        return 'NO_FRESH_PATH'\n",
    "    fresh = _pclk.monotonic() - lt.get('capture_at', -1e18) <= lp.run['parameters']['max_gap']\n"
    "    if not (lt.get('valid') is True and lp.consecutive >= 3 and fresh):\n"
    "        blind = (cfg.get('upper_post_blind_resume') is True and cfg.get('upper_post_turn_resume') is True and fresh\n"
    "                 and c.get('seg0') == 'upper_straight' and not c.get('lost0') and c.get('turned0') is True)   # r1e\n"
    "        if not blind: return 'NO_FRESH_PATH'\n")
open(dst, 'w', encoding='utf-8', newline='').write(s)
print('written', dst, hashlib.sha256(s.encode('utf-8')).hexdigest()[:8])
