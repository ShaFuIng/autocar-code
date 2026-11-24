"""x-band-r2 (c3refit-20261122): the distance band of curve 3 made smooth and anchored on the gyro. On top of lap_ff.py adcde919 (x-band-r1, on the car).
Why (run c6f3, x-band r1 owning; proof/ of c3refit): (1) the band's distance is the 5 Hz encoder staircase (steps of 0.2-0.25 m): the feed-forward and the error
centre jump at every step; (2) the feed-forward followed each kink of the lane; (3) the camera term (K 300) is the only feedback and the camera reading leaves
the table by +-0.3 as soon as the car is off the old path -> two hard turns with a straight between them, deeper inside, +330 us spike at 3.0-3.6 m.
New OPTIONAL keys of arc2.json c3.xband (any of them present = r2 path; none = x-band-r1 code, value for value):
  x_dr true (+ dr_max_s 0.3)   distance used by the tables = odometer + wheel speed * time since the odometer last stepped (at most dr_max_s)
  u_x [[x, us] ..]             feed-forward directly in us (replaces kappa_x / g(v); kappa_x then only feeds the yaw-rate term)
  h_x [[x, deg] ..], k_h_us, h_lim_us (80)     heading term  clip(k_h * (h_x(x) - h), +-h_lim);  h = degrees turned right since the encoder step that carries
                               the C3s crossing (the layer's turned_deg moved to the START of that step: lap_pos history, no extra read);
                               h_dev_max (25): |h_x - h| beyond it = heading not trusted on that frame, the term is 0 (h_deg / h_ref still logged)
  y_x [[x, cm] ..], k_y_us, y_lim_us (80), y_hold_s (0.6), y_age_s (0.2)   position term  clip(k_y * (y - y_x(x)), +-y_lim);  y = arc_lat's lateral position of the
                               previous frame (live_pid hands it over, patch_live_r2.py); no fresh value: the last one is held y_hold_s, then the term is 0
  rate_us_s                    the band target moves at most this fast (us per second)
Unchanged: w_x (where the band owns), the escape (share -> 0 between e_max - e_band and e_max), shadow, cap / floor, the layer's own eligibility, max_drop tail.
Log: proposal.arc2_c3.xband gets revision x-band-r2 and xs_m (staircase), h_deg, h_ref, h_us, y_cm, y_us, raw_us (before the rate limit).
usage: python patch_r2.py <lap_ff.py adcde919> <out lap_ff.py>      python patch_r2.py --check <lap_ff.py adcde919> <patched lap_ff.py>"""
import hashlib, sys
from pathlib import Path

args = [a for a in sys.argv[1:] if a != '--check']; CHECK = '--check' in sys.argv
s = Path(args[0]).read_bytes().decode('utf-8')
assert hashlib.sha256(s.encode('utf-8')).hexdigest().startswith('adcde919'), 'unexpected base'
NL = chr(13) + chr(10) if chr(13) + chr(10) in s else chr(10)


def rep(old, new):
    global s
    old, new = old.replace(chr(10), NL), new.replace(chr(10), NL)
    assert s.count(old) == 1, (s.count(old), old[:80]); s = s.replace(old, new)


# ---- the odometer-step tracker runs on every frame the layer is stepped (only when x_dr is configured)
rep("        c = self.c; st = float(steer)\n"
    "        b4 = getattr(self, 'band_on', False)   # c4-band-r1\n",
    "        c = self.c; st = float(steer)\n"
    "        if getattr(self, 'xb_dr', False): self._xb_tick(lp, now)   # x-band-r2: when did the odometer last step\n"
    "        b4 = getattr(self, 'band_on', False)   # c4-band-r1\n")
# ---- call: hand over the clock and the layer's turned angle
rep("        xb = self._xb(A, lp, x, near, valid, vv, g, out, float(centre), float(hi)) if (eligible and getattr(self, 'xb', None) is not None) else None   # x-band-r1\n",
    "        xb = self._xb(A, lp, x, near, valid, vv, g, out, float(centre), float(hi), now, turned) if (eligible and getattr(self, 'xb', None) is not None) else None   # x-band-r1 / r2\n")
# ---- init: validate the r2 keys
rep("        except (TypeError, ValueError): return\n"
    "        self.xb = m; self.xb_on = True\n",
    "            m['_r2'] = any(k in X for k in ('x_dr', 'u_x', 'h_x', 'y_x', 'rate_us_s'))   # x-band-r2 (c3refit): none of these keys = the r1 code below\n"
    "            for k in ('u_x', 'h_x', 'y_x'):\n"
    "                t = m.get(k)\n"
    "                if t is not None and (not isinstance(t, (list, tuple)) or len(t) < 2 or not all(len(q) == 2 and _num(q[0]) and _num(q[1]) for q in t)): return\n"
    "            for k, dflt in (('k_h_us', 0.), ('h_lim_us', 80.), ('h_dev_max', 25.), ('k_y_us', 0.), ('y_lim_us', 80.), ('y_hold_s', 0.6), ('y_age_s', 0.2), ('rate_us_s', 0.), ('dr_max_s', 0.3)):\n"
    "                m.setdefault(k, dflt)\n"
    "                if not _num(m[k]) or float(m[k]) < 0.: return\n"
    "        except (TypeError, ValueError): return\n"
    "        self.xb = m; self.xb_on = True; self.xb_dr = bool(m['_r2'] and m.get('x_dr') is True); self._xd = None; self._xy = None; self._xl = None\n")
# ---- the band: r2 branch
rep("    def _xb(self, A, lp, x, near, valid, v, g, out, centre, hi):\n"
    "        X = self.xb\n",
    "    def _xb_tick(self, lp, now):   # x-band-r2: [odometer value, clock when it last stepped (None until a step has been seen)]\n"
    "        D = (lp or {}).get('odo_m')\n"
    "        if not _num(D): return\n"
    "        D = float(D); S = self._xd\n"
    "        if S is None or D < S[0] - 1e-9 or D - S[0] > 2.0: self._xd = [D, None]\n"
    "        elif D > S[0] + 1e-9: S[0] = D; S[1] = float(now) if _num(now) else None\n"
    "\n"
    "    def _xb2(self, A, lp, xs, near, valid, v, g, out, centre, hi, now, turned):\n"
    "        X = self.xb; lp = lp or {}; x = xs; tn = float(now) if _num(now) else None\n"
    "        S = self._xd\n"
    "        if self.xb_dr and S is not None and S[1] is not None and tn is not None and _num(v):\n"
    "            x = xs + max(0., min(float(X['dr_max_s']), tn - S[1])) * float(v)\n"
    "        w = interp(X['w_x'], x)\n"
    "        if w <= 0. or not (valid and _num(near)):\n"
    "            self._xl = None; return None\n"
    "        kap = interp(X['kappa_x'], x); ec = interp(X['e_c_x'], x)\n"
    "        prof = float(self.c['_c_eff_us']) + (interp(X['u_x'], x) if X.get('u_x') is not None else kap / g)\n"
    "        et = max(-float(X['e_lo_us']), min(float(X['e_hi_us']), float(X['k_e_us']) * (float(near) - ec)))\n"
    "        gz = lp.get('gz_imu'); gt = 0.\n"
    "        if _num(gz) and X['k_gz_us']: gt = max(-float(X['gz_lim_us']), min(float(X['gz_lim_us']), float(X['k_gz_us']) * (float(gz) + float(v) * kap)))\n"
    "        ht = 0.; hd = None; href = None\n"
    "        if X.get('h_x') is not None and float(X['k_h_us']) > 0. and _num(turned):\n"
    "            if 'h_off' not in A:   # heading at the START of the encoder step that carries the anchor crossing (turned_deg refers to its end)\n"
    "                f = lp.get('yaw_at'); A['h_off'] = None\n"
    "                try:\n"
    "                    if callable(f): A['h_off'] = float(f(float(A['d_land']) - 1e-6)) - float(f(float(A['d_land'])))\n"
    "                except Exception: A['h_off'] = None\n"
    "            if A['h_off'] is not None and abs(A['h_off']) <= 25.:\n"
    "                hd = float(turned) + A['h_off']; href = interp(X['h_x'], x)\n"
    "                if abs(href - hd) <= float(X['h_dev_max']): ht = max(-float(X['h_lim_us']), min(float(X['h_lim_us']), float(X['k_h_us']) * (href - hd)))   # further off the plan than h_dev_max: heading not trusted, no term\n"
    "        yt = 0.; ym = None\n"
    "        if X.get('y_x') is not None and float(X['k_y_us']) > 0. and tn is not None:\n"
    "            al = lp.get('al')\n"
    "            if isinstance(al, (list, tuple)) and len(al) >= 2 and _num(al[0]) and _num(al[1]) and 0. <= tn - float(al[0]) <= float(X['y_age_s']): self._xy = (float(al[1]), tn)\n"
    "            if self._xy is not None and 0. <= tn - self._xy[1] <= float(X['y_hold_s']):\n"
    "                ym = self._xy[0]; yt = max(-float(X['y_lim_us']), min(float(X['y_lim_us']), float(X['k_y_us']) * (ym - interp(X['y_x'], x))))\n"
    "        raw = max(centre - float(X['floor_below_us']), min(centre + float(X['cap_rel_us']), hi, prof + et + gt + ht + yt)); tgt = raw\n"
    "        L = self._xl\n"
    "        if float(X['rate_us_s']) > 0. and L is not None and tn is not None and 0. < tn - L[1] <= 0.25:\n"
    "            r = float(X['rate_us_s']) * (tn - L[1]); tgt = max(L[0] - r, min(L[0] + r, raw))\n"
    "        self._xl = (tgt, tn) if tn is not None else None\n"
    "        we = max(0., min(1., (float(X['e_max']) - float(near)) / max(1e-3, float(X['e_band']))))\n"
    "        o = out if X.get('shadow') else out + w * we * (tgt - out)\n"
    "        r3 = lambda q: None if q is None else round(q, 1)\n"
    "        return dict(out=o, info=dict(revision='x-band-r2', x_m=round(x, 3), xs_m=round(xs, 3), w=round(w, 3), own=round(w * we, 3), kappa=round(kap, 1), prof_us=round(prof, 1), e_c=round(ec, 3),\n"
    "                                     e_us=round(et, 1), gz_us=round(gt, 1), h_deg=r3(hd), h_ref=r3(href), h_us=round(ht, 1), y_cm=r3(ym), y_us=round(yt, 1), raw_us=round(raw, 1),\n"
    "                                     target_us=round(tgt, 1), in_us=round(out, 1), out_us=round(o, 1)))\n"
    "\n"
    "    def _xb(self, A, lp, x, near, valid, v, g, out, centre, hi, now=None, turned=None):\n"
    "        X = self.xb\n"
    "        if X.get('_r2'):   # x-band-r2 (c3refit)\n"
    "            if A.get('cancel') is not None or A.get('end_d') is not None: return None\n"
    "            return self._xb2(A, lp, x, near, valid, v, g, out, centre, hi, now, turned)\n")
h = hashlib.sha256(s.encode('utf-8')).hexdigest()[:8]
if CHECK:
    t = Path(args[1]).read_bytes().decode('utf-8'); print('check', 'OK' if t == s else 'DIFFERENT', h); sys.exit(0 if t == s else 1)
Path(args[1]).write_bytes(s.encode('utf-8')); print('written', args[1], h)
