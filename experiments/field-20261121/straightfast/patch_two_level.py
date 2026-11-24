"""straightfast-20261121 candidate B: lap_speed.py 3ad2272b -> lap-speed-r3 (per-zone two_level policy + per-zone rate_pwm_s).
usage: python patch_two_level.py <lap_speed.py 3ad2272b> <out>     (keys absent = the previous code path, frame for frame)"""
import hashlib, sys
src, dst = sys.argv[1], sys.argv[2]
s = open(src, encoding='utf-8', newline='').read()
assert hashlib.sha256(s.encode('utf-8')).hexdigest().startswith('3ad2272b'), 'unexpected base'


LF, CR = chr(10), chr(13)
CRLF = (CR + LF) in s   # the car file keeps its line endings


def rep(old, new):
    global s
    if CRLF: old = old.replace(LF, CR + LF); new = new.replace(LF, CR + LF)
    assert s.count(old) == 1, (s.count(old), old[:70]); s = s.replace(old, new)


# 1. doc
rep('Config lap_speed.json, read once per run (first call of a run). File absent / enabled false / no zones = no effect, nothing logged.\n',
    'Config lap_speed.json, read once per run (first call of a run). File absent / enabled false / no zones = no effect, nothing logged.\n'
    '\n'
    'lap-speed-r3 (straightfast-20261121, offline candidate, NOT deployed; both keys per zone, absent = r1 / r2 exactly):\n'
    '  rate_pwm_s   per-zone ramp rate (PWM/s) instead of the global rate_pwm_s.\n'
    '  two_level    {"e_trouble": 0.40, "e_ok": 0.30, "steer_trouble_us": 300, "gz_trouble_dps": 25, "gz_tau_s": 0.25,\n'
    '                "mid_pwm": 70, "rate_up_pwm_s": 200, "rate_down_pwm_s": 120}\n'
    '               Why (11-21, runs e1b4 / b93e / 9934, 30 zone passes): 36 of 41 slow-downs inside the PWM-100 zones came from one or\n'
    '               two frames of speed_adapt LOW / ERROR_RATE_LARGE (camera error rate; |e| 0.13 and IMU |gz| ~3 deg/s at that moment),\n'
    '               each one dropped the PWM to speed_adapt\'s <= 50 and restarted the 60 PWM/s ramp (0.8 s back to 100).\n'
    '               In a two_level zone the camera error RATE and speed_adapt\'s LOW / MID levels are not used. Two states only:\n'
    '                 NORMAL  -> the zone PWM;   TROUBLE -> min(zone PWM, mid_pwm)\n'
    '               TROUBLE when |image error| > e_trouble (left again below e_ok), or |steer - centre| > steer_trouble_us, or the\n'
    '               IMU yaw rate low-passed over gz_tau_s is above gz_trouble_dps (null = not used; no IMU value = not used).\n'
    '               The PWM moves at rate_up_pwm_s / rate_down_pwm_s from the last frame of the same zone, without any wait, and is\n'
    '               never below the PWM speed_adapt chose. The hard conditions of r1 still leave the frame untouched: reason not\n'
    '               PROPOSAL_ONLY, speed_adapt SPECIAL_CONTROL / DOWNHILL / AFTER_CREST / NO_ERROR / DISABLED / ZERO_BASE, upper / landing /\n'
    '               crest / downhill-entry frames, special control, route not WAIT_CLIMB, |pitch| > level_deg, no image error.\n'
    '               Log: lap_speed.two_level = {state, why, gz_lp}; revision lap-speed-r3 on these frames.\n')

# 2. revision
rep("REVISION_R2 = 'lap-speed-r2'", "REVISION_R3 = 'lap-speed-r3'   # logged only for frames inside a two_level zone (straightfast-20261121)\nREVISION_R2 = 'lap-speed-r2'")

# 3. blocked(): optional soft flag (two_level skips the LOW test)
rep("    def blocked(self, p, F, center, z=None):\n", "    def blocked(self, p, F, center, z=None, soft=False):\n")
rep("        if sa.get('level') == 'LOW' and not ((z or {}).get('allow_low') and sa.get('reason') in PATH_LOW and not sa.get('weave_slow')): return 'LOW'\n",
    "        if not soft and sa.get('level') == 'LOW' and not ((z or {}).get('allow_low') and sa.get('reason') in PATH_LOW and not sa.get('weave_slow')): return 'LOW'   # r3: soft = two_level zone\n")

# 4. apply(): branch + per-zone rate
rep("        center = float(lp.run['parameters']['center_us'])\n        why = self.blocked(p, F, center, z)\n",
    "        center = float(lp.run['parameters']['center_us'])\n"
    "        tl = z.get('two_level')   # lap-speed-r3: absent / not a dict / enabled false -> the r1 / r2 path below, unchanged\n"
    "        if isinstance(tl, dict) and tl.get('enabled', True) is not False: return self._two_level(lp, F, p, now, z, log, center, tl)\n"
    "        why = self.blocked(p, F, center, z)\n")
rep("        new = min(tgt, start + float(self.c.get('rate_pwm_s', 60.)) * dt) if start < tgt else start\n",
    "        new = min(tgt, start + float(z.get('rate_pwm_s', self.c.get('rate_pwm_s', 60.))) * dt) if start < tgt else start   # r3: per-zone rate (absent = global)\n")

# 5. the two-level method (before get())
rep("def get(lp):\n",
    "def _tl_apply(self, lp, F, p, now, z, log, center, tl):\n"
    "    \"\"\"lap-speed-r3 two_level zone (see the module text). Returns the log dict.\"\"\"\n"
    "    import math\n"
    "    log['revision'] = REVISION_R3\n"
    "    hard = self.blocked(p, F, center, z, soft=True)\n"
    "    e = p.get('image_error'); st = p.get('steer_us'); cur = float(p.get('pwm') or 0.)\n"
    "    if hard is None and (e is None or st is None): hard = 'NO_ERROR'\n"
    "    gz = getattr(F, 'gzy', None); glp = None   # IMU yaw rate (deg/s) of this frame, as yaw_inner / curve_map get it\n"
    "    if isinstance(gz, (int, float)) and not isinstance(gz, bool) and now is not None:\n"
    "        s_ = getattr(self, 'tl_gz', None); tau = max(1e-3, float(tl.get('gz_tau_s', .25)))\n"
    "        y = float(gz) if s_ is None or not (0. <= now - s_[0] <= .5) else s_[1] + (float(gz) - s_[1]) * (1. - math.exp(-(now - s_[0]) / tau))\n"
    "        self.tl_gz = (now, y); glp = abs(y)\n"
    "    else: self.tl_gz = None\n"
    "    if hard is not None:\n"
    "        self.last = None; self.tl_trouble = False; log.update(blocked=hard, pwm=round(cur, 2)); return log\n"
    "    et = float(tl.get('e_trouble', .40)); eo = min(et, float(tl.get('e_ok', et))); gt = tl.get('gz_trouble_dps'); sgt = float(tl.get('steer_trouble_us', 300.))\n"
    "    zi = next((i for i, q in enumerate(self.zones) if q is z), None)\n"
    "    was = bool(getattr(self, 'tl_trouble', False)) and self.last is not None and len(self.last) >= 3 and self.last[2] == zi\n"
    "    ae = abs(float(e)); why = None\n"
    "    if ae > et or (was and ae > eo): why = 'ERROR'\n"
    "    elif abs(float(st) - center) > sgt: why = 'STEER'\n"
    "    elif gt is not None and glp is not None and glp > float(gt): why = 'YAW'\n"
    "    self.tl_trouble = why is not None\n"
    "    tgt = float(z['pwm'])\n"
    "    try: tc = float(throttle_cap.max_pct()) if throttle_cap is not None else 50.\n"
    "    except Exception: tc = 50.\n"
    "    if tgt > tc: tgt = tc; log['cap_pct'] = tc\n"
    "    if why is not None: tgt = min(tgt, float(tl.get('mid_pwm', 70.)))\n"
    "    carry = self.last is not None and now is not None and 0. <= now - self.last[0] <= float(self.c.get('carry_s', .5))\n"
    "    if carry:\n"
    "        dt = now - self.last[0]; prev = max(cur, self.last[1]); same = len(self.last) >= 3 and self.last[2] == zi\n"
    "        if prev < tgt: new = min(tgt, prev + float(tl.get('rate_up_pwm_s', 200.)) * dt)   # also from a lower zone just left (no drop back to speed_adapt's PWM at the hand-over)\n"
    "        elif same: new = max(tgt, prev - float(tl.get('rate_down_pwm_s', 120.)) * dt)\n"
    "        elif self.c.get('down_rate_pwm_s') is not None: new = max(cur, tgt, prev - float(self.c['down_rate_pwm_s']) * dt); log['taper'] = True   # came from a higher zone: the existing taper\n"
    "        else: new = cur\n"
    "    else: new = cur   # first frame after a gap / a hard block: starts from speed_adapt's PWM (as r1)\n"
    "    new = max(cur, new)\n"
    "    if new > cur:\n"
    "        p['pwm'] = new\n"
    "        sp = getattr(lp, 'speed', None); nsa = new\n"
    "        try: nsa = min(new, float(sp.c['hard_cap'])) if sp is not None else new\n"
    "        except Exception: pass\n"
    "        if sp is not None and getattr(sp, 'cur', None) is not None and sp.cur < nsa: sp.cur = nsa\n"
    "        log['pwm_before'] = round(cur, 2)\n"
    "    log['pwm'] = round(new, 2)\n"
    "    log['two_level'] = dict(state='TROUBLE' if why else 'NORMAL', why=why, gz_lp=None if glp is None else round(glp, 1))\n"
    "    self.last = (now, new, zi) if now is not None else None\n"
    "    return log\n"
    "\n\n"
    "LapSpeed._two_level = _tl_apply\n"
    "\n\n"
    "def get(lp):\n")
open(dst, 'w', encoding='utf-8', newline='').write(s)
print('written', dst, hashlib.sha256(s.encode('utf-8')).hexdigest()[:8])
