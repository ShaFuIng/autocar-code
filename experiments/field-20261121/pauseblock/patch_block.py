"""pause-resume-r1d (2026-11-21): refuse a pause where a resume is impossible by position + can_pause in the status.
usage: python patch_block.py <pause_resume.py 965307da> <out>"""
import hashlib, sys
src, dst = sys.argv[1], sys.argv[2]
s = open(src, encoding='utf-8', newline='').read()
assert hashlib.sha256(s.encode('utf-8')).hexdigest().startswith('965307da'), 'unexpected base'


def rep(old, new):
    global s
    assert s.count(old) == 1, (s.count(old), old[:70]); s = s.replace(old, new)


# 1. config keys (absent = today's behaviour)
rep("                upper_post_turn_resume=False)   # pause-resume-r1b",
    "                refuse_pause_where_no_resume=False,   # pause-resume-r1d (2026-11-21, user): true = POST /api/pid/pause is REFUSED (409 NO_PAUSE_HERE, the car keeps\n"
    "                                                      # driving) on the ramp / downhill / landing and on the upper deck except the resumable part; status gets can_pause\n"
    "                pause_slope_deg=8.0,                  # r1d: |pitch - flat| at or above this counts as a slope for the refusal (4 deg is hit by a hard stop on the flat)\n"
    "                upper_post_pause_max_m=None,          # r1d: on the upper straight, no pause further than this many metres after DASHe (the coast must not reach the mouth)\n"
    "                upper_post_turn_resume=False)   # pause-resume-r1b")

# 2. text
rep("    UNSUPPORTED_MODE=", "    NO_PAUSE_HERE='這一段不能暫停（坡道、下坡、落地或上層不可接續的路段），車會繼續行駛；要停車請按停止。',\n    UNSUPPORTED_MODE=")

# 3. the rule + status
rep("def _pause_obj(lp):\n",
    "def pause_block(lp, cfg=None):\n"
    "    \"\"\"r1d: None, or where a pause is refused now ('ON_SLOPE' / 'UPPER_DECK' / 'SLOPE_MOUTH'). Off unless refuse_pause_where_no_resume is true.\"\"\"\n"
    "    cfg = cfg or load_config()\n"
    "    if cfg.get('refuse_pause_where_no_resume') is not True: return None\n"
    "    tp = _lpos(lp); lt = lp.latest or {}; pitch = _pitch_rel(lp)\n"
    "    if pitch is not None and abs(pitch) >= float(cfg['pause_slope_deg']): return 'ON_SLOPE'\n"
    "    if getattr(lp, 'down_until', None) is not None and _pclk.monotonic() <= lp.down_until: return 'ON_SLOPE'\n"
    "    if getattr(lp, 'bown', None) is not None and getattr(lp.bown, 'active', False): return 'ON_SLOPE'\n"
    "    seg = tp.segment() if tp else None; lost = bool(tp.lost) if tp else True\n"
    "    if seg in SLOPE_SEGS and not lost: return 'ON_SLOPE'\n"
    "    if (lt.get('dash_route') or {}).get('route') in UPPER_ROUTES or (seg in UPPER_SEGS and not lost):\n"
    "        if not (cfg.get('upper_post_turn_resume') is True and seg == 'upper_straight' and not lost and getattr(getattr(lp, 'dash', None), 'turned', False) is True):\n"
    "            return 'UPPER_DECK'\n"
    "        mx = cfg.get('upper_post_pause_max_m'); a = getattr(tp, 'anchor', None)\n"
    "        if isinstance(mx, (int, float)) and not isinstance(mx, bool) and a and a[0] == 'DASHe' and tp.D - a[1] > float(mx): return 'SLOPE_MOUTH'\n"
    "    return None\n"
    "\n\n"
    "def _pause_obj(lp):\n")
rep("        return dict(state='running', since_s=None, can_resume=False, reason='NOT_PAUSED', reason_text=TEXT['NOT_PAUSED'], coast_m=None,\n"
    "                    pauses=pz['n'] if pz else 0, resume_pending=False)\n",
    "        try: blk = pause_block(lp)\n"
    "        except Exception: blk = None\n"
    "        return dict(state='running', since_s=None, can_resume=False, reason='NOT_PAUSED', reason_text=TEXT['NOT_PAUSED'], coast_m=None,\n"
    "                    pauses=pz['n'] if pz else 0, resume_pending=False, can_pause=blk is None, pause_block=blk)   # r1d\n")

# 4. the refusal (only when a NEW pause episode would start; duplicates / pause-again answer as before)
rep("        cfg = load_config(); now_v = _pclk.monotonic(); lt = lp.latest or {}\n",
    "        cfg = load_config(); now_v = _pclk.monotonic(); lt = lp.latest or {}\n"
    "        blk = pause_block(lp, cfg)   # r1d\n"
    "        if blk is not None:\n"
    "            _log(lp, dict(kind='pause_refused', wall=_pclk._time.time(), run_id=run['id'], where=blk))\n"
    "            raise PauseError('NO_PAUSE_HERE', where=blk)\n")
open(dst, 'w', encoding='utf-8', newline='').write(s)
print('written', dst, hashlib.sha256(s.encode('utf-8')).hexdigest()[:8])
