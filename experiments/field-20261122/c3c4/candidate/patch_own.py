"""band-own-r1 (c3c4-20261122): 'the band owns the command' for curve 4 (angle band) and curve 3 (new distance band).
User 2026-11-22: inside a curve the command is  lane-shape feed-forward + K * (camera error - the error a centred car reads there) + yaw-rate term,
and the PID chain / curve_ff must not add on top of it. Curves 1-3 angle bands already have this (band.own_after_deg, c1-band-r1); this patch adds:

(1) lap_ff.json c4.band.own (dict, enabled true):  from x0_m (ramp ramp_m, metres after the predicted LAND8 point) the band output is the band target itself
    (floor = top = target), instead of min(max(PID chain, target), top). Escape: the share falls 1 -> 0 as the near error goes e_max - e_band -> e_max
    (car far outside: the old band rule returns, so the chain and the top margin can add; lesson of 11-17 run ab01, curve-1 own mode: profile too low,
    error term limited to +150, nothing else allowed to add -> outer edge). After the band's end angle nothing changes (old exit rule).
(2) arc2.json c3.xband (dict, enabled true): a band indexed by the lap_pos distance since C3s (curve 3 is a polyline, not an arc):
    target = c_eff + kappa_x(x)/g(v) + clip(k_e*(near - e_c_x(x)), -e_lo, +e_hi) + clip(k_gz*(gz + v*kappa_x(x)), +-gz_lim), limited to
    [centre - floor_below_us, centre + cap_rel_us];  out = out + w_x(x) * escape * (target - out)   (w_x = distance table 0..1 = where the band owns:
    ramp in after the layer's trigger, ramp out before the corner; outside it the car's present control is untouched). Same escape as (1).
    xband.shadow true = computed and logged (proposal.arc2_c3.xband), command unchanged.
    Runs inside the arc2 c3 layer (after curve_ff / lap_ff, before curve_trim), only on eligible frames while the c3 block is active and not cancelled.
    live_pid adds one IMU yaw-rate read to the layer's lap_pos dict only while an xband is configured (patch_live_xb.py).
Keys absent / enabled not true = lap_ff.py 1a1ea129 value for value (no extra read, no extra log key).
usage: python patch_own.py <lap_ff.py 1a1ea129> <out lap_ff.py>      python patch_own.py --check <lap_ff.py 1a1ea129> <patched lap_ff.py>"""
import hashlib, sys
from pathlib import Path

args = [a for a in sys.argv[1:] if a != '--check']; CHECK = '--check' in sys.argv
s = Path(args[0]).read_bytes().decode('utf-8')
assert hashlib.sha256(s.encode('utf-8')).hexdigest().startswith('1a1ea129'), 'unexpected base'
NL = chr(13) + chr(10) if chr(13) + chr(10) in s else chr(10)


def rep(old, new):
    global s
    old, new = old.replace(chr(10), NL), new.replace(chr(10), NL)
    assert s.count(old) == 1, (s.count(old), old[:80]); s = s.replace(old, new)


# ---- (1) curve-4 band: own
rep("        band = min(st, top) if ended else min(max(st, tgt), top)   # after the end angle only the top stays (no floor at the centre while fading out)\n"
    "        return dict(w=w, out=st + a_out * (band - st), kap=kap, prof=prof, et=et, gt=gt, ec=ec, tgt=tgt, top=top, ended=ended)\n",
    "        band = min(st, top) if ended else min(max(st, tgt), top)   # after the end angle only the top stays (no floor at the centre while fading out)\n"
    "        ow = None if ended else own_share(B.get('own'), x, near, valid)   # band-own-r1: None = key absent / off (old rule)\n"
    "        if ow is not None: band = band + ow * (tgt - band)\n"
    "        r = dict(w=w, out=st + a_out * (band - st), kap=kap, prof=prof, et=et, gt=gt, ec=ec, tgt=tgt, top=top, ended=ended)\n"
    "        if ow is not None: r['own'] = ow\n"
    "        return r\n")
rep("                       gz_us=round(bi['gt'], 1), target_us=round(bi['tgt'], 1), top_us=round(bi['top'], 1), band_us=round(bi['out'], 1))\n"
    "        return d\n",
    "                       gz_us=round(bi['gt'], 1), target_us=round(bi['tgt'], 1), top_us=round(bi['top'], 1), band_us=round(bi['out'], 1))\n"
    "        if bi is not None and bi.get('own') is not None: d['own'] = round(bi['own'], 3)   # band-own-r1\n"
    "        return d\n")
# ---- shared helper
rep("def ec_turn(b, tb):\n",
    "def own_share(O, x, near, valid):\n"
    "    \"\"\"band-own-r1: share 0..1 of 'output = band target' at distance x, or None (O absent / not a dict / enabled not true).\n"
    "    ramp over ramp_m from x0_m; falls to 0 as the near error goes e_max - e_band -> e_max (car far outside); 0 on a frame without a usable error.\"\"\"\n"
    "    if not isinstance(O, dict) or O.get('enabled') is not True: return None\n"
    "    try:\n"
    "        wx = max(0., min(1., (float(x) - float(O.get('x0_m', 0.))) / max(1e-3, float(O.get('ramp_m', 0.3)))))\n"
    "        if not (valid and _num(near)): return 0.\n"
    "        return wx * max(0., min(1., (float(O.get('e_max', 0.9)) - float(near)) / max(1e-3, float(O.get('e_band', 0.1)))))\n"
    "    except (TypeError, ValueError): return None\n"
    "\n\n"
    "def ec_turn(b, tb):\n")
# ---- (2) distance band of a LapFF4 block (arc2 c3)
rep("        self._b4_init()   # c4-band-r1: band_on False when the block has no enabled 'band' dict (r1 code path)\n",
    "        self._b4_init()   # c4-band-r1: band_on False when the block has no enabled 'band' dict (r1 code path)\n"
    "        self._xb_init()   # x-band-r1 (band-own-r1): xb None when the block has no enabled 'xband' dict\n")
rep("        if bi is not None and eligible: out = out + bi['w'] * (bi['out'] - out)\n"
    "        md = c.get('max_drop_us')\n",
    "        if bi is not None and eligible: out = out + bi['w'] * (bi['out'] - out)\n"
    "        xb = self._xb(A, lp, x, near, valid, vv, g, out, float(centre), float(hi)) if (eligible and getattr(self, 'xb', None) is not None) else None   # x-band-r1\n"
    "        if xb is not None: out = xb['out']\n"
    "        md = c.get('max_drop_us')\n")
rep("        if b4: info['band'] = self._b4_info(A, tb, bi)   # c4-band-r1\n",
    "        if b4: info['band'] = self._b4_info(A, tb, bi)   # c4-band-r1\n"
    "        if xb is not None: info['xband'] = xb['info']   # x-band-r1\n")
rep("    def _b4_init(self):\n",
    "    # ---- x-band-r1 (band-own-r1, c3c4-20261122). Key 'xband' of a LapFF4 block (arc2.json c3.xband; dict, enabled true; absent = nothing runs).\n"
    "    XB_DEFAULTS = dict(enabled=False, kappa_x=None, e_c_x=None, w_x=None, k_e_us=200., e_lo_us=250., e_hi_us=200., k_gz_us=1.5, gz_lim_us=60., cap_rel_us=330.,\n"
    "                       floor_below_us=0., e_max=0.9, e_band=0.1)\n"
    "\n"
    "    def _xb_init(self):\n"
    "        self.xb = None; self.xb_on = False\n"
    "        X = self.c.get('xband') if isinstance(self.c, dict) else None\n"
    "        if not isinstance(X, dict) or X.get('enabled') is not True: return\n"
    "        m = dict(self.XB_DEFAULTS); m.update(X)\n"
    "        try:\n"
    "            for k in ('kappa_x', 'e_c_x', 'w_x'):\n"
    "                t = m[k]\n"
    "                if not isinstance(t, (list, tuple)) or len(t) < 2 or not all(len(q) == 2 and _num(q[0]) and _num(q[1]) for q in t): return\n"
    "            if not all(0. <= float(q[1]) <= 1. for q in m['w_x']) or not all(-1. <= float(q[1]) <= 1. for q in m['e_c_x']): return\n"
    "            for k in ('k_e_us', 'e_lo_us', 'e_hi_us', 'k_gz_us', 'gz_lim_us', 'cap_rel_us', 'floor_below_us', 'e_max', 'e_band'):\n"
    "                if not _num(m[k]): return\n"
    "        except (TypeError, ValueError): return\n"
    "        self.xb = m; self.xb_on = True\n"
    "\n"
    "    def _xb(self, A, lp, x, near, valid, v, g, out, centre, hi):\n"
    "        X = self.xb\n"
    "        if A.get('cancel') is not None or A.get('end_d') is not None: return None\n"
    "        w = interp(X['w_x'], x)\n"
    "        if w <= 0. or not (valid and _num(near)): return None\n"
    "        kap = interp(X['kappa_x'], x); prof = float(self.c['_c_eff_us']) + kap / g; ec = interp(X['e_c_x'], x)\n"
    "        et = max(-float(X['e_lo_us']), min(float(X['e_hi_us']), float(X['k_e_us']) * (float(near) - ec)))\n"
    "        gz = (lp or {}).get('gz_imu'); gt = 0.\n"
    "        if _num(gz) and X['k_gz_us']: gt = max(-float(X['gz_lim_us']), min(float(X['gz_lim_us']), float(X['k_gz_us']) * (float(gz) + float(v) * kap)))\n"
    "        tgt = max(centre - float(X['floor_below_us']), min(centre + float(X['cap_rel_us']), hi, prof + et + gt))\n"
    "        we = max(0., min(1., (float(X['e_max']) - float(near)) / max(1e-3, float(X['e_band']))))\n"
    "        o = out if X.get('shadow') else out + w * we * (tgt - out)   # shadow true = computed and logged only\n"
    "        return dict(out=o, info=dict(revision='x-band-r1', x_m=round(x, 3), w=round(w, 3), own=round(w * we, 3), kappa=round(kap, 1), prof_us=round(prof, 1), e_c=round(ec, 3),\n"
    "                                     e_us=round(et, 1), gz_us=round(gt, 1), target_us=round(tgt, 1), in_us=round(out, 1), out_us=round(o, 1)))\n"
    "\n"
    "    def _b4_init(self):\n")
h = hashlib.sha256(s.encode('utf-8')).hexdigest()[:8]
if CHECK:
    t = Path(args[1]).read_bytes().decode('utf-8'); print('check', 'OK' if t == s else 'DIFFERENT', h); sys.exit(0 if t == s else 1)
Path(args[1]).write_bytes(s.encode('utf-8')); print('written', args[1], h)
