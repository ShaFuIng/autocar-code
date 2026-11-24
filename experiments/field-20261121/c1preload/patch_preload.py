"""i-preload-r1 (c1preload 2026-11-21): build candidate/i_ramp.py from the car's i_ramp.py (760de37d).
usage: python patch_preload.py <base i_ramp.py> <out i_ramp.py>"""
import hashlib, sys
src, dst = sys.argv[1], sys.argv[2]
s = open(src, encoding='utf-8', newline='').read()
assert hashlib.sha256(s.encode('utf-8')).hexdigest().startswith('760de37d'), 'unexpected base'


def rep(old, new):
    global s
    assert s.count(old) == 1, (s.count(old), old[:70]); s = s.replace(old, new)


# 1. doc
rep('"""\nimport json, math\n',
    "\ni-preload-r1 (c1preload 2026-11-21; offline candidate, NOT deployed): on the lap from the start the camera PID reaches curve 1 with\n"
    "ki*i = -1..-10 us at the curve start (9 lap-1 passes after the 11-21 linkage repair), later laps with +16..+26 us (12 passes). The\n"
    "missing ~28 us is ~0.13 of error: the early error peak is 0.80 instead of 0.72 and curve_ff (floor 0.55..0.85 -> centre+517 us) lifts\n"
    "the command to 1900-1992 us (5/9 vs 2/12), the car over-rotates, the command falls back to centre mid-curve, saturates again and the\n"
    "exit is right of the lane (run-69a8 e -1.00, run-bab2 -0.54). Optional key (absent = no change, log records unchanged):\n"
    "  preload       {\"target_us\": 20, \"e_on\": 0.30, \"n_on\": 2, \"min_s\": 1.5, \"max_s\": 15, \"rate_us_s\": 150}\n"
    "                Waits while lap_pos says lap 0 with no anchor yet (= the lap from the start, before the first landmark). When the PID's\n"
    "                control error (controller.e) is >= e_on on n_on consecutive steps, at least min_s after the first step, the integral\n"
    "                is raised to target_us (ki*i) at rate_us_s (null = in one step) and the preload is finished for the run. It only\n"
    "                raises: an integral already at / above the target is left alone. Never again in the run: a lap_pos lap > 0 or an\n"
    "                anchor before the trigger, lap_pos lost at the trigger, max_s without a trigger, or a replaced PID object (pause /\n"
    "                resume rebuilds the controller) all switch it off. As the releases above it writes controller.i only; this frame's\n"
    "                command is unchanged (the effect starts with the next PID step).\n"
    "                Log: proposal.i_ramp.preload = {state RAMP / DONE / OFF, ...} on the frames where it acts or switches off (revision\n"
    "                i-preload-r1 when no other i_ramp record exists on that frame).\n"
    '"""\nimport json, math\n')

# 2. constants + config
rep("CONFIG_FILE = Path(__file__).with_name('i_ramp.json')\n",
    "PRE_REVISION = 'i-preload-r1'   # c1preload-20261121\n"
    "PRE_DEFAULTS = dict(enabled=True, target_us=20., e_on=0.30, n_on=2, min_s=1.5, max_s=15., rate_us_s=150.)\n"
    "CONFIG_FILE = Path(__file__).with_name('i_ramp.json')\n")
rep("class IRamp(object):\n",
    "def _pre_cfg(raw):\n"
    "    \"\"\"i-preload-r1: the lap-1 curve-1 preload config, or None (key absent / disabled / malformed).\"\"\"\n"
    "    if not isinstance(raw, dict) or raw.get('enabled', True) is False: return None\n"
    "    e = dict(PRE_DEFAULTS); e.update(raw)\n"
    "    if not all(_num(e[k]) for k in ('target_us', 'e_on', 'n_on', 'min_s', 'max_s')) or e['target_us'] <= 0 or e['e_on'] <= 0 or e['n_on'] < 1: return None\n"
    "    if e.get('rate_us_s') is not None and not (_num(e['rate_us_s']) and e['rate_us_s'] > 0): return None\n"
    "    return e\n"
    "\n\n"
    "class IRamp(object):\n")

# 3. state
rep("        self.wants_lap = self.exits is not None or self.land is not None\n",
    "        self.pre = _pre_cfg(c.get('preload'))            # i-preload-r1: absent -> None (step() == the body below, unchanged)\n"
    "        self.wants_lap = self.exits is not None or self.land is not None or self.pre is not None\n"
    "        self.p_state = 'WAIT'; self.p_ctrl = None; self.p_t0 = None; self.p_n = 0; self.p_t = None\n")

# 4. step wrapper (the original body becomes _step_base, text unchanged)
rep("    def step(self, now, pitch, ctrl, lap=None):\n",
    "    def step(self, now, pitch, ctrl, lap=None):\n"
    "        \"\"\"i-preload-r1 wrapper: without the preload key exactly _step_base (the previous step()).\"\"\"\n"
    "        r = self._step_base(now, pitch, ctrl, lap)\n"
    "        if self.pre is None or self.p_state == 'OFF': return r\n"
    "        pl = self._preload(now, ctrl, lap)\n"
    "        if pl is None: return r\n"
    "        return dict(revision=PRE_REVISION, preload=pl) if r is None else dict(r, preload=pl)\n"
    "\n"
    "    def _preload(self, now, ctrl, lap):\n"
    "        \"\"\"i-preload-r1: raise the integral once at the entry of the first curve of the lap from the start.\"\"\"\n"
    "        c = self.pre\n"
    "        if ctrl is not None and not _num(getattr(ctrl, 'i', None)) and getattr(ctrl, 'pid', None) is not None: ctrl = ctrl.pid\n"
    "        if ctrl is None or not _num(getattr(ctrl, 'i', None)): return None\n"
    "        if self.p_ctrl is None: self.p_ctrl = ctrl; self.p_t0 = now\n"
    "        elif ctrl is not self.p_ctrl:   # the PID object was replaced (pause / resume): this is no longer the run's start\n"
    "            self.p_state = 'OFF'; return dict(state='OFF', reason='CONTROLLER_REPLACED')\n"
    "        ki = float((getattr(ctrl, 'p', None) or {}).get('ki') or 0.)\n"
    "        if ki <= 0: self.p_state = 'OFF'; return dict(state='OFF', reason='NO_KI')\n"
    "        e = getattr(ctrl, 'e', None)\n"
    "        if self.p_state == 'WAIT':\n"
    "            if not isinstance(lap, dict): self.p_n = 0; return None   # no lap_pos: cannot tell the first lap\n"
    "            if lap.get('lap') != 0 or lap.get('anchor') is not None:\n"
    "                self.p_state = 'OFF'; return dict(state='OFF', reason='NOT_BEFORE_FIRST_LANDMARK')\n"
    "            if now - self.p_t0 > float(c['max_s']): self.p_state = 'OFF'; return dict(state='OFF', reason='TIMEOUT')\n"
    "            if lap.get('lost') or not _num(e) or e < float(c['e_on']) or now - self.p_t0 < float(c['min_s']): self.p_n = 0; return None\n"
    "            self.p_n += 1\n"
    "            if self.p_n < int(c['n_on']): return None\n"
    "            self.p_state = 'RAMP'; self.p_t = None\n"
    "        tgt = float(c['target_us']) / ki; i0 = float(ctrl.i)\n"
    "        if i0 >= tgt: self.p_state = 'OFF'; return dict(state='DONE', i_us=round(ki * i0, 2))\n"
    "        rate = c.get('rate_us_s'); dt = None if self.p_t is None else now - self.p_t; self.p_t = now\n"
    "        if rate is None: i1 = tgt\n"
    "        else: i1 = min(tgt, i0 + float(rate) / ki * (dt if dt is not None and 0 < dt <= 0.1 else 1. / 30.))\n"
    "        ctrl.i = i1\n"
    "        if i1 >= tgt: self.p_state = 'OFF'\n"
    "        return dict(state='DONE' if i1 >= tgt else 'RAMP', e=None if not _num(e) else round(float(e), 3), i_us_before=round(ki * i0, 2), i_us=round(ki * i1, 2))\n"
    "\n"
    "    def _step_base(self, now, pitch, ctrl, lap=None):\n")
open(dst, 'w', encoding='utf-8', newline='').write(s)
print('written', dst, hashlib.sha256(s.encode('utf-8')).hexdigest()[:8])
