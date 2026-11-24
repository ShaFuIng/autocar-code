"""arc-lat-r1 (arc-lat-20261119, offline candidate, NOT deployed): lateral feedback in the curves from the measured distance to the lines.

Why: in the right curves the camera error (image row ~60 cm ahead, clipped at +-1.0) mostly reads the car heading, not where the car is
in the lane; a centred car already reads 0.6-0.8 (arc-fit-20261119), so the band / PID layers hold the car on the inside line (curve 2
mid -6.5 cm, curve 1 -4.6 cm; curve-3 ramp foot: inside wheel over the mat gap). This layer measures the car's lateral position directly
and steers on it.

Measurement (no single-arc assumption): the worker (arc_fit.py with crossings_worker.json arc_fit.pts) writes, per frame, the centre
points of the chosen left / right line on the ground (path_diagnostic.arc_fit.L/R.pts, camera ground frame: x cm right, z cm forward of the
front-tyre foremost edge). This module keeps the points of the last keep_s seconds in a local odometric map (dead reckoning: rear-axle
pose from the wheel speed _cm_speed x dt and the 50 Hz IMU heading of each frame; each frame's points are placed with the pose
interpolated at that frame's capture time). The line beside the car (rear axle, 24 cm behind the front-tyre edge; front axle -4 cm)
was seen 0.3-0.9 m earlier by the camera; a local straight / parabola fit of the map points around the reference point gives the
lateral offset of each line and its direction relative to the car. Entry / exit / the curve-3 polygon need no shape model.
Car lateral position y (cm, + = left of the lane centre) = lane/2 - d_L or d_R - lane/2, d = rear-axle centre to the line INNER edge;
lane = 46 cm (inner edge to inner edge, measured); inner clearance clr_in = lane/2 + y - 16 (right wheel outer face to the right line inner edge); both lines -> weighted mean when they agree within pair_tol_cm, else the outer (left) line.

Control (per zone = lap_pos nominal metre window of one curve; each zone has its own enabled / control switch):
   u = u_ff + clip(ky*(y - y_t) + kpsi*psi, +-fb_lim)   (psi = car heading relative to the lane at the reference point, + = car points left)
   u_ff = c_eff + (-kappa_centre)/g(v)  (kappa from the same map, lane centre, ff_ahead_cm ahead of the front axle; ff 'band' = lap_ff band
          profile of this frame when present; 'none' = c_eff) ; y_t = y_target_cm + out_cm (out = toward the outside, + = left for the right curves)
   output = PID chain + w*(u - chain), w ramps 0->w_max over ramp_s while the measurement is valid, holds the last u for hold_s on invalid
   frames, then fades to 0 over fade_s (hand back to the existing layers). u is limited to [centre - lo_us, cap_us] and to rate_us_s.
r2 law (arc-lat2-20261119; only where the zone (or the file) sets "law": "r2"; absent = the r1 law above, frame-for-frame identical):
   u = u_ff2 + clip(kpsi2*(psi_p - psi_ref) + w_i*u_i2 [guards], +-fb2_lim)   (defaults kpsi2 10 us/deg, k_yp 0.8 deg/cm, pred2 0.2 s: linear
       margins gain 2.1 / phase 46 deg at g 0.205 + 0.30 s delay, 1.4 / 24 deg at g 0.24 + 0.40 s; r1: 3.2 / 55 and 2.0 / 40)
   u_ff2  = c_eff - b*kap2, b = ff_us_per_dpm (per zone; None = 1/(gain*gain_scale) as r1), kap2 = lane-centre curvature of the map at
            ff_ref_cm ahead of the REAR axle (where the car is + about the steering delay), low-pass ff2_tau_s, clipped to kap_clip_dpm
   psi_ref = clip(-k_yp*(ey + v*pred2*sin psi), +-psi_ref_max)  (lateral error -> wanted heading; cascade, ky_eff = kpsi2*k_yp)
   psi_p  = psi + (gz - v*kap2)*pred2 ; u_i2 += ki2*ey*dt only while |ey| <= i_band_cm and the output was not limited; w_i = min(1, |kap2|/kap_full)
   exit (peak |kap2| >= kap_full and now |kap2| < kap_exit): psi_ref limit psi_ref_max_exit, u_i2 decays with i_exit_tau_s (no left turn on the straight)
   guards: clr_out_p = clr_out - v*guard_s*sin psi < out_min_cm -> no outward heading wanted (psi_ref <= 0) and fb >= k_guard*(out_min - clr_out_p)
           (k_guard 0 = fb >= 0: never pushes further out); clr_in_p = clr_in + v*guard_s*sin psi < in_min_cm -> psi_ref >= 0, fb <= -k_guard*(in_min - clr_in_p)
   control false + law r2 = log only, the r2 command logged as proposal.arc_lat.u_shadow (shadow, not applied).
Not active: outside the zones, on lost / invalid frames (layer does not run), crossing windows (eligibility = lap_ff's), downhill and its
4 s tail, upper deck (no zone), |pitch| > pitch_max. Inputs: lap_pos state, the frame's arc_fit pts / capture time, one IMU latest-file read
per frame (t, yaw, gz), _cm_speed, chain steer. Standard library only. arc_lat.json absent / enabled false -> the layer is not built.
"""
import json, math
from pathlib import Path

REVISION = 'arc-lat-r1'
CONFIG_FILE = Path(__file__).with_name('arc_lat.json')
DEG = 180. / math.pi
DEFAULTS = dict(enabled=False, lane_cm=46.0, lat_scale=1.0, car_half_cm=16.0, pitch_ins_max=4.0, cap_shift_s=-0.04, max_extrap_cm=25., rear_z_cm=-24.0, ref_z_cm=-24.0, front_z_cm=-4.0, keep_s=1.2, max_fit_pts=60, ins_zmax_cm=85., ins_zmin_cm=25.,
                win_cm=[-20., 50.], min_pts=5, min_span_cm=12., max_rms_cm=2.5, rej_cm=2.0, gate_cm=8.0, pair_tol_cm=6., y_max_cm=20., jump_cm=6., jump_n=3,
                max_pose_gap_s=0.5, fresh_s=0.25, hold_s=0.3, fade_s=0.3, ramp_s=0.3, w_max=1.0,
                ky_us_cm=3.0, kpsi_us_deg=8.0, ki_us_cms=2.0, i_lim_us=60., fb_lim_us=160., pred_s=0.15,
                ff='map', ff_ahead_cm=25., ff_win_cm=[-35., 45.], ff_tau_s=0.15, kap_max_dpm=90., c_eff_off_us=0., gain_scale=1.0,
                gain_table=[[0.5, 0.185], [1.5, 0.185]], v_default=0.95,
                lo_us=200., cap_us=1960., rate_us_s=1200., log_margin_m=1.0, lap_m=67.72, zones={},
                # r2 law (arc-lat2-20261119): read only where "law": "r2" (zone or file)
                law='r1', ff_ref_cm=30., ff2_win_cm=[-25., 40.], ff2_tau_s=0.10, ff_us_per_dpm=None, kap_clip_dpm=70.,
                kpsi2_us_deg=10., k_yp_deg_cm=0.8, psi_ref_max_deg=6., psi_ref_max_exit_deg=3., pred2_s=0.20, ki2_us_cms=3., i2_lim_us=80., i_band_cm=10.,
                kap_full_dpm=30., kap_exit_dpm=12., i_exit_tau_s=0.3, ui_carry=0., out_min_cm=None, in_min_cm=None, guard_s=0.3, k_guard_us_cm=0., fb2_lim_us=220.)
# zone windows: lap_pos nominal metres (lap_pos.json nominal_m; negative start = before the lap origin C1s)
ZONE_DEFAULTS = {
    'c1': dict(enabled=False, control=True, s=[-0.8, 3.6], y_target_cm=0.0, out_cm=0.0, pitch_max=3.0, ins_zmax_cm=None, kind='flat'),
    'c2': dict(enabled=False, control=True, s=[14.5, 19.6], y_target_cm=0.0, out_cm=0.0, pitch_max=3.0, ins_zmax_cm=None, kind='flat', gain_scale=1.15),   # map curvature / IMU gz/v = 1.17 in curve 2 (cause unknown) -> feed-forward scaled down
    'c3': dict(enabled=False, control=True, s=[29.0, 36.75], y_target_cm=0.0, out_cm=0.0, pitch_max=2.5, ins_zmax_cm=75., kind='flat'),
    'c4': dict(enabled=False, control=True, s=[50.2, 56.8], y_target_cm=0.0, out_cm=0.0, pitch_max=3.0, ins_zmax_cm=None, kind='land'),
}
ZONE_KEYS = ('ky_us_cm', 'kpsi_us_deg', 'ki_us_cms', 'i_lim_us', 'fb_lim_us', 'pred_s', 'ff', 'cap_us', 'w_max', 'c_eff_off_us', 'gain_scale')


def _num(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(float(x))


def load_config(path=None):
    p = Path(path) if path else CONFIG_FILE
    try: raw = json.loads(p.read_text(encoding='utf8'))
    except (OSError, ValueError): return None
    if not isinstance(raw, dict) or not raw.get('enabled'): return None
    c = dict(DEFAULTS); c.update({k: v for k, v in raw.items() if k != 'zones'})
    z = {}
    for k, d in ZONE_DEFAULTS.items():
        x = dict(d); x.update((raw.get('zones') or {}).get(k) or {}); z[k] = x
    c['zones'] = z
    return c


def gain(table, v):
    t = sorted((float(a), float(b)) for a, b in table)
    if v <= t[0][0]: return t[0][1]
    if v >= t[-1][0]: return t[-1][1]
    for (v0, g0), (v1, g1) in zip(t, t[1:]):
        if v0 <= v <= v1: return g0 + (g1 - g0) * (v - v0) / (v1 - v0)
    return t[-1][1]


def _solve3(A, b):
    """3x3 (or 2x2 when A is 2x2) linear solve, Gauss elimination; None when singular."""
    n = len(b); M = [list(A[i]) + [b[i]] for i in range(n)]
    for i in range(n):
        p = max(range(i, n), key=lambda r: abs(M[r][i]))
        if abs(M[p][i]) < 1e-12: return None
        M[i], M[p] = M[p], M[i]
        for r in range(n):
            if r != i:
                f = M[r][i] / M[i][i]
                for cc in range(i, n + 1): M[r][cc] -= f * M[i][cc]
    return [M[i][n] / M[i][i] for i in range(n)]


def polyfit(us, ws, deg):
    """least squares w = a + b u (+ c u^2); returns (coef list, rms) or None."""
    n = len(us)
    if n < deg + 2: return None
    m = deg + 1; S = [0.] * (2 * m - 1); T = [0.] * m
    for u, w in zip(us, ws):
        pu = 1.
        for k in range(2 * m - 1):
            if k < m: T[k] += w * pu
            S[k] += pu; pu *= u
    A = [[S[i + j] for j in range(m)] for i in range(m)]
    co = _solve3(A, T)
    if co is None: return None
    r = 0.
    for u, w in zip(us, ws):
        e = w - sum(co[k] * u ** k for k in range(m)); r += e * e
    return co, math.sqrt(r / n)


class LineMap:
    """rear-axle pose by dead reckoning + line centre points of the last keep_s seconds in that frame."""

    def __init__(self, c):
        self.c = c; self.poses = []; self.pts = []; self.X = 0.; self.Y = 0.; self.tp = None

    def pose_update(self, now, yaw_deg, v):
        if self.tp is not None:
            dt = now - self.tp
            if dt > float(self.c['max_pose_gap_s']) or dt < 0: self.poses = []; self.pts = []   # long gap: the map is dropped (dead reckoning not trusted)
            elif _num(v):
                a = math.radians(yaw_deg); self.X += float(v) * 100. * dt * math.cos(a); self.Y += float(v) * 100. * dt * math.sin(a)
        self.tp = now; self.poses.append((now, self.X, self.Y, float(yaw_deg)))
        ks = float(self.c['keep_s']) + 0.5
        while self.poses and now - self.poses[0][0] > ks: self.poses.pop(0)
        while self.pts and now - self.pts[0][0] > float(self.c['keep_s']): self.pts.pop(0)

    def pose_at(self, t):
        P = self.poses
        if not P or t < P[0][0] - 1e-6 or t > P[-1][0] + 0.05: return None
        if t >= P[-1][0]: return P[-1][1:]
        for i in range(len(P) - 1, 0, -1):
            if P[i - 1][0] <= t:
                t0, x0, y0, a0 = P[i - 1]; t1, x1, y1, a1 = P[i]; f = (t - t0) / max(1e-6, t1 - t0)
                return x0 + f * (x1 - x0), y0 + f * (y1 - y0), a0 + f * (a1 - a0)
        return P[0][1:]

    def gate_ok(self, pose, side, new_w):
        """frame gate: the new points of one side (world) must agree with the map points already there (median |residual| <= gate_cm);
        True when the map has too few points in that range to judge."""
        g = self.c.get('gate_cm')
        if not g or pose is None or not getattr(self, 'gate_on', True): return True
        X, Y, a = pose; ca, sa = math.cos(math.radians(a)), math.sin(math.radians(a))
        nw = [((px - X) * ca + (py - Y) * sa, -(px - X) * sa + (py - Y) * ca) for px, py in new_w]
        lo = min(q[0] for q in nw) - 10.; hi = max(q[0] for q in nw) + 10.
        old = [(fw, lt) for fw, lt, _ in self.car_frame(pose)[side] if lo <= fw <= hi]
        if len(old) < 12: return True
        mf = int(self.c.get('max_fit_pts') or 0)
        if mf and len(old) > mf: old = old[::-(-len(old) // mf)]
        r = polyfit([q[0] for q in old], [q[1] for q in old], 2 if hi - lo >= 40 else 1)
        if r is None: return True
        co = r[0]; res = sorted(abs(lt - sum(co[k] * fw ** k for k in range(len(co)))) for fw, lt in nw)
        return res[len(res) // 2] <= float(g)

    def insert(self, now, t_cap, af, zmax):
        """af = path_diagnostic.arc_fit of this frame (dict) -> number of points inserted."""
        self.rejected = []
        if not isinstance(af, dict) or not af.get('ok') or not _num(t_cap): return 0
        ps = self.pose_at(float(t_cap) + float(self.c.get('cap_shift_s') or 0.))
        if ps is None: return 0
        X, Y, a = ps; ca, sa = math.cos(math.radians(a)), math.sin(math.radians(a)); rz = float(self.c['rear_z_cm']); n = 0; ls = float(self.c.get('lat_scale') or 1.)   # optional lateral camera scale (default 1.0: the calibration is right at the line inner edges)
        for sd in ('L', 'R'):
            f = af.get(sd)
            if not isinstance(f, dict) or not isinstance(f.get('pts'), list): continue
            out = []
            for q in f['pts']:
                if not (isinstance(q, (list, tuple)) and len(q) == 2 and _num(q[0]) and _num(q[1])): continue
                x, z = float(q[0]), float(q[1])
                if z < float(self.c['ins_zmin_cm']) or z > zmax: continue
                fw = z - rz; lf = -x * ls
                out.append((X + fw * ca - lf * sa, Y + fw * sa + lf * ca))
            if out and not self.gate_ok(self.poses[-1][1:] if self.poses else None, sd, out): self.rejected.append(sd); out = []
            if out: self.pts.append((now, sd, out, max(1., min(12., float(f.get('lw_cm') or 2.5))) * ls)); n += len(out)
        return n

    def car_frame(self, pose):
        """all map points in the car frame of `pose` (fwd from the rear axle, left), per side; computed once per pose."""
        if getattr(self, '_cf_pose', None) is pose and getattr(self, '_cf_n', -1) == len(self.pts): return self._cf
        X, Y, a = pose; ca, sa = math.cos(math.radians(a)), math.sin(math.radians(a)); o = {'L': [], 'R': []}
        for _, sd, pts, lw in self.pts:
            L_ = o[sd]
            for px, py in pts:
                dx, dy = px - X, py - Y; L_.append((dx * ca + dy * sa, -dx * sa + dy * ca, lw))
        self._cf_pose = pose; self._cf_n = len(self.pts); self._cf = o
        return o

    def local(self, pose, side, z_ref, win, deg):
        """map points of one side in the car frame of `pose` (fwd from the rear axle, left), fitted around fwd0 = z_ref - rear_z
        over win: returns dict(off=lateral offset of the line centre at fwd0 (+ left), ang=line direction deg (+ = toward the left),
        kap=curvature 1/cm left +, n, span, rms, lw) or None."""
        f0 = float(z_ref) - float(self.c['rear_z_cm'])
        us = []; ws = []; lws = []
        lo, hi = f0 + float(win[0]), f0 + float(win[1])
        for fw, lt, lw in self.car_frame(pose)[side]:
            if lo <= fw <= hi: us.append(fw - f0); ws.append(lt); lws.append(lw)
        n = len(us)
        if n < int(self.c['min_pts']): return None
        mf = int(self.c.get('max_fit_pts') or 0)
        if mf and n > mf:   # cost cap: every k-th point (deterministic)
            k = -(-n // mf); us = us[::k]; ws = ws[::k]; lws = lws[::k]; n = len(us)
        span = max(us) - min(us)
        if span < float(self.c['min_span_cm']): return None
        ex = max(0., min(us), -max(us))   # extrapolation distance from the nearest map point to the reference point
        if ex > float(self.c['max_extrap_cm']): return None
        dg = deg if span >= 30. else 1
        r = polyfit(us, ws, dg)
        if r is None: return None
        co, rms = r
        res = [abs(ws[i] - sum(co[k] * us[i] ** k for k in range(len(co)))) for i in range(n)]
        lim = max(float(self.c['rej_cm']), 3. * 1.4826 * sorted(res)[n // 2])   # one outlier-rejection pass, robust (median absolute residual)
        keep = [i for i in range(n) if res[i] <= lim]
        if len(keep) < n:
            if len(keep) < int(self.c['min_pts']): return None
            us = [us[i] for i in keep]; ws = [ws[i] for i in keep]; lws = [lws[i] for i in keep]; n = len(us)
            r = polyfit(us, ws, dg)
            if r is None: return None
            co, rms = r
        b = co[1]; c2 = co[2] if len(co) > 2 else 0.
        kap = 2. * c2 / (1. + b * b) ** 1.5
        lws.sort()
        return dict(off=co[0], ang=math.degrees(math.atan(b)), kap=kap, n=n, span=span, rms=rms, lw=lws[len(lws) // 2], ex=ex)


class ArcLat:
    def __init__(self, cfg):
        self.c = cfg; self.map = LineMap(cfg); self.w = 0.; self.u = None; self.t_last = None; self.t_ok = None; self.held = None
        self.zone = None; self.ui = 0.; self.kf = None; self.y_prev = None; self.jn = 0; self.n = 0; self.vlast = None
        self.kf2 = None; self.s2 = {}; self.zone2 = None; self.ui2_last = {}; self._u2 = None   # r2 law state (unused by the r1 law)

    @staticmethod
    def _in(x, s, lap):
        a, b = float(s[0]), float(s[1])
        if a < 0: return x >= lap + a or x <= b
        return a <= x <= b

    def zone_at(self, s_m, margin=0.):
        if not _num(s_m): return None
        lap = float(self.c['lap_m']); x = float(s_m) % lap
        for k, z in self.c['zones'].items():
            if not z.get('enabled'): continue
            s = z['s']
            if self._in(x, [float(s[0]) - margin, float(s[1]) + margin], lap): return k
        return None

    def measure(self, pose, z_ref):
        c = self.c; L = self.map.local(pose, 'L', z_ref, c['win_cm'], 2); R = self.map.local(pose, 'R', z_ref, c['win_cm'], 2)
        if L is not None and (L['off'] <= 0 or L['rms'] > float(c['max_rms_cm'])): L = None   # a left line must be left of the reference point
        if R is not None and (R['off'] >= 0 or R['rms'] > float(c['max_rms_cm'])): R = None
        h = float(c['lane_cm']) / 2.   # lane_cm = INNER width (white-line inner edge to inner edge, measured 46 cm)
        # inner edge = the mask boundary on the lane side of the line (in a right curve the left line's edge toward the curve centre):
        # line centre offset -/+ half the line width (measured per frame, perpendicular to the line) projected on the car's lateral axis
        dl = None if L is None else L['off'] - L['lw'] / 2. / max(0.5, math.cos(math.radians(L['ang'])))
        dr = None if R is None else -R['off'] - R['lw'] / 2. / max(0.5, math.cos(math.radians(R['ang'])))
        yl = None if dl is None else h - dl; yr = None if dr is None else dr - h
        if yl is None and yr is None: return None
        if yl is not None and yr is not None:
            if abs(yl - yr) <= float(c['pair_tol_cm']):
                wl, wr = L['span'] / max(.3, L['rms']), R['span'] / max(.3, R['rms']); y = (wl * yl + wr * yr) / (wl + wr)
                ang = (wl * L['ang'] + wr * R['ang']) / (wl + wr); src = 'LR'
            else: y, ang, src = yl, L['ang'], 'L!'      # disagreement: the outer (left) line of the right curves
        elif yl is not None: y, ang, src = yl, L['ang'], 'L'
        else: y, ang, src = yr, R['ang'], 'R'
        return dict(y=y, psi=-ang, src=src, yl=yl, yr=yr, nl=0 if L is None else L['n'], nr=0 if R is None else R['n'],
                    dl=dl, dr=dr, clr_in=h + y - float(c['car_half_cm']), clr_out=h - y - float(c['car_half_cm']))   # wheel outer face to the right / left line inner edge

    def kappa(self, pose, zf=None, win=None):
        c = self.c; zf = float(c['front_z_cm']) + float(c['ff_ahead_cm']) if zf is None else zf; h = float(c['lane_cm']) / 2.; ks = []; ws = []
        for sd, sg in (('L', 1.), ('R', -1.)):   # lane centre = h + half line width to the right of the left line centre (+ = right)
            f = self.map.local(pose, sd, zf, c['ff_win_cm'] if win is None else win, 2)
            if f is None or f['span'] < 30. or f['rms'] > float(c['max_rms_cm']): continue
            off = sg * (h + f['lw'] / 2.)
            k = f['kap']; den = 1. + k * off
            if abs(den) < 0.2: continue
            ks.append(k / den * 100. * DEG); ws.append(f['span'])
        if not ks: return None
        k = sum(a * b for a, b in zip(ks, ws)) / sum(ws)
        return k if abs(k) <= float(self.c['kap_max_dpm']) else None

    def step(self, now, imu, v, t_cap, af, lp, eligible, pitch, steer, centre, lo, hi, band_prof=None):
        """imu = (t, yaw_deg, gz) of the latest IMU sample or None; lp = dict(s_m, lost) of lap_pos; returns (steer, info or None)."""
        c = self.c; st = float(steer); self.n += 1
        v = float(v) if _num(v) and float(v) > 0.05 else (self.vlast if self.vlast is not None else 0.)
        if _num(v) and v > 0.05: self.vlast = v
        if imu is None or not _num(imu[1]):
            return self._fade(now, st, lo, hi, 'NO_IMU')
        self.map.pose_update(now, float(imu[1]), v)
        s_m = None if (lp is None or lp.get('lost')) else lp.get('s_m')
        zk = self.zone_at(s_m)
        zlog = zk or self.zone_at(s_m, float(c['log_margin_m']))
        zc = c['zones'].get(zlog) if zlog else None
        zmax = float(c['ins_zmax_cm']) if (zc is None or zc.get('ins_zmax_cm') is None) else float(zc['ins_zmax_cm'])
        if _num(pitch) and abs(float(pitch)) > float(c['pitch_ins_max']): self.map.pts = []; ni = 0   # slope: the flat-ground projection is wrong -> map dropped
        else: self.map.gate_on = zlog is not None; ni = self.map.insert(now, t_cap, af, zmax)   # frame gate only near the zones (cost)
        if zlog is None and self.w <= 0.:
            self.zone = None; self.zone2 = None; return st, None   # zone2: r2 state restarts at the next activation (unused by the r1 law)
        dtf = 0. if self.t_last is None else max(0., min(0.2, now - self.t_last)); self.t_last = now
        pose = self.map.poses[-1][1:] if self.map.poses else None
        gz = float(imu[2]) if len(imu) > 2 and _num(imu[2]) else 0.
        m = self.measure(pose, c['ref_z_cm']) if pose else None
        mf = self.measure(pose, c['front_z_cm']) if pose else None
        why = None
        if m is not None and abs(m['y']) > float(c['y_max_cm']): why = 'Y_RANGE'
        if m is not None and why is None and self.y_prev is not None and abs(m['y'] - self.y_prev) > float(c['jump_cm']):
            self.jn += 1
            if self.jn < int(c['jump_n']): why = 'JUMP'
        else: self.jn = 0
        if m is not None and why is None: self.y_prev = m['y']
        if ni > 0: self.t_ins = now
        if m is not None and why is None and (getattr(self, 't_ins', None) is None or now - self.t_ins > float(c['fresh_s'])): why = 'STALE'   # camera sees no line: map only, not trusted beyond fresh_s
        ok = m is not None and why is None
        kap = self.kappa(pose) if pose else None
        if kap is not None:
            a = 1. if self.kf is None or dtf <= 0 else 1. - math.exp(-dtf / max(1e-3, float(c['ff_tau_s'])))
            self.kf = kap if self.kf is None else self.kf + a * (kap - self.kf)
        z = c['zones'].get(zk) if zk else None
        P = lambda k: float(z[k]) if (z is not None and z.get(k) is not None) else float(c[k])
        z2 = c['zones'].get(zk or zlog) if (zk or zlog) else None
        law2 = z2 is not None and (z2.get('law') or c.get('law')) == 'r2'   # r2 law (arc-lat2): nothing of it runs for the r1 law
        if not law2: self.zone2 = None   # r2 state restarts at the next activation
        if law2:
            P2 = lambda k: z2[k] if z2.get(k) is not None else c[k]
            P2f = lambda k: (None if P2(k) is None else (P2(k) if isinstance(P2(k), (list, str)) else float(P2(k))))
            kap2 = self.kappa(pose, float(c['rear_z_cm']) + float(P2('ff_ref_cm')), P2('ff2_win_cm')) if pose else None
            if kap2 is not None:
                a2 = 1. if self.kf2 is None or dtf <= 0 else 1. - math.exp(-dtf / max(1e-3, float(P2('ff2_tau_s'))))
                self.kf2 = kap2 if self.kf2 is None else self.kf2 + a2 * (kap2 - self.kf2)
            if self.zone2 != (zk or zlog):   # new activation: r2 state restarts (integral optionally carried from the last pass of this zone)
                self.s2 = dict(ui=float(P2('ui_carry') or 0.) * self.ui2_last.get(zk or zlog, 0.)); self.zone2 = zk or zlog
        info = dict(revision=REVISION, zone=zk or zlog, n_ins=ni, gate_rej=''.join(getattr(self.map, 'rejected', []) or []) or None, ok=bool(ok), why=why if m is not None else 'NO_LINE', v=round(v, 3),
                    y_cm=None if m is None else round(m['y'], 1), y_front_cm=None if mf is None else round(mf['y'], 1), src=None if m is None else m['src'],
                    psi_deg=None if m is None else round(m['psi'], 1), dl_cm=None if m is None or m['dl'] is None else round(m['dl'], 1),
                    dr_cm=None if m is None or m['dr'] is None else round(m['dr'], 1), clr_in_cm=None if m is None else round(m['clr_in'], 1),
                    clr_in_front_cm=None if mf is None else round(mf['clr_in'], 1), clr_out_cm=None if m is None else round(m['clr_out'], 1), yl=None if m is None or m['yl'] is None else round(m['yl'], 1),
                    yr=None if m is None or m['yr'] is None else round(m['yr'], 1), kap_dpm=None if self.kf is None else round(self.kf, 1), map_n=sum(len(q[2]) for q in self.map.pts))
        if law2:
            info.update(law='r2', kap2_dpm=None if self.kf2 is None else round(self.kf2, 1)); self._u2 = None
            if ok:
                vv2 = v if v > 0.2 else float(c['v_default'])
                self._u2 = law_r2(self.s2, m, self.kf2, gz, vv2, dtf, P2f, centre)
                if self.s2.get('ui_peak') is not None: self.ui2_last[zk or zlog] = self.s2['ui_peak']
        elg = eligible.get(z.get('kind') or 'flat') if (isinstance(eligible, dict) and z is not None) else eligible
        ctrl = z is not None and z.get('control', True) and bool(elg) and (not _num(pitch) or abs(float(pitch)) <= float(z.get('pitch_max') or 99.))
        if not ctrl:
            if self.w > 0.:
                o, i2 = self._fade(now, st, lo, hi, 'OUT' if z is None else ('LOG_ONLY' if not z.get('control', True) else 'NOT_ELIGIBLE'), dtf)
                info.update(i2); return o, info
            info.update(state='LOG' if z is None or z.get('control', True) is False else 'NOT_ELIGIBLE', w=0., steer_us=round(st, 1), pid_steer_us=round(st, 1), applied=False)
            if law2 and self._u2 is not None:   # r2 shadow (log only): what the r2 law would command (output limits applied, no rate limit)
                u2, i2_ = self._u2; self.s2['sat'] = False
                info.update(u_shadow=round(max(float(centre) - float(c['lo_us']), min(float(P2('cap_us')), float(hi), u2)), 1), **{'sh_' + k_: v_ for k_, v_ in i2_.items()})
            self.u = None; return st, info
        if ok:
            self.t_ok = now
            vv = v if v > 0.2 else float(c['v_default']); g = gain(c['gain_table'], vv) * P('gain_scale')
            ffm = z.get('ff') or c['ff']
            ceff = float(centre) + P('c_eff_off_us')
            if ffm == 'band' and _num(band_prof): uff = float(band_prof)
            elif ffm == 'none' or self.kf is None: uff = ceff
            else: uff = ceff - self.kf / g
            yt = float(z.get('y_target_cm') or 0.) + float(z.get('out_cm') or 0.)
            ps = P('pred_s'); ey = m['y'] - yt; psi = m['psi']
            if law2 and self._u2 is not None:   # r2 law (arc-lat2-20261119)
                self.zone = zk; u, i2_ = self._u2; uff = i2_['u_ff']; fb = i2_['u_fb']; ey = i2_['ey_cm']; psi = i2_['psi_p']; ui_log = i2_['u_i']
            else:
                if ps > 0:
                    kk = self.kf or 0.
                    ey = ey + vv * 100. * ps * math.sin(math.radians(psi)); psi = psi + (gz - vv * kk) * ps
                if self.w <= 0. or self.zone != zk: self.ui = 0.   # integral restarts with each zone activation
                self.zone = zk
                self.ui = max(-P('i_lim_us'), min(P('i_lim_us'), self.ui + P('ki_us_cms') * (m['y'] - yt) * dtf))
                fb = max(-P('fb_lim_us'), min(P('fb_lim_us'), P('ky_us_cm') * ey + P('kpsi_us_deg') * psi + self.ui))
                u = uff + fb; ui_log = self.ui
            u_raw = u
            u = max(float(centre) - float(c['lo_us']), min(P('cap_us'), float(hi), u))
            if self.u is None: self.u = st
            r = float(c['rate_us_s']) * max(dtf, 1. / 30.)
            u = max(self.u - r, min(self.u + r, u)); self.u = u
            if law2: self.s2['sat'] = abs(u - u_raw) > 0.5   # r2 anti-windup: the integral holds while the output is limited
            self.w = min(P('w_max'), self.w + (dtf / max(1e-3, float(c['ramp_s'])) if self.w > 0 or dtf > 0 else 0.)) if self.w < P('w_max') else P('w_max')
            if self.w <= 0.: self.w = min(P('w_max'), 1. / 30. / max(1e-3, float(c['ramp_s'])))
            out = max(float(lo), min(float(hi), st + self.w * (u - st)))
            info.update(state='ON' if self.w >= P('w_max') else 'RAMP', w=round(self.w, 3), u_ff=round(uff, 1), u_fb=round(fb, 1), u_i=round(ui_log, 1), ey_cm=round(ey, 1), psi_p=round(psi, 1),
                        u_us=round(u, 1), pid_steer_us=round(st, 1), steer_us=round(out, 1), applied=bool(out != st))
            if law2: info.update({k_: v_ for k_, v_ in i2_.items() if k_ not in ('u_ff', 'u_fb', 'u_i', 'ey_cm', 'psi_p')})
            return out, info
        if self.u is not None and self.t_ok is not None and now - self.t_ok <= float(c['hold_s']):
            out = max(float(lo), min(float(hi), st + self.w * (self.u - st)))
            info.update(state='HOLD', w=round(self.w, 3), u_us=round(self.u, 1), pid_steer_us=round(st, 1), steer_us=round(out, 1), applied=bool(out != st))
            return out, info
        o, i2 = self._fade(now, st, lo, hi, 'NO_MEAS', dtf); info.update(i2); return o, info

    def _fade(self, now, st, lo, hi, why, dtf=None):
        if dtf is None:
            dtf = 0. if self.t_last is None else max(0., min(0.2, now - self.t_last)); self.t_last = now
        if self.w <= 0. or self.u is None:
            self.w = 0.; self.u = None
            return st, dict(state='OFF', why_off=why, w=0., steer_us=round(st, 1), applied=False)
        self.w = max(0., self.w - dtf / max(1e-3, float(self.c['fade_s'])))
        out = max(float(lo), min(float(hi), st + self.w * (self.u - st)))
        if self.w <= 0.: self.u = None
        return out, dict(state='FADE', why_off=why, w=round(self.w, 3), pid_steer_us=round(st, 1), steer_us=round(out, 1), applied=bool(out != st))


def law_r2(S, m, kap, gz, v, dt, P, centre):
    """arc-lat r2 control law (pure). S = per-activation state dict (ui, kpk, sat, ui_peak); m = dict(y, psi, clr_out, clr_in) cm / deg;
    kap = lane-centre curvature at the rear axle + ff_ref (deg/m, left +) or None; gz = yaw rate deg/s (left +); v m/s; P(key) -> param.
    Returns (u before the output limits / rate limit, info dict)."""
    b = P('ff_us_per_dpm')
    if b is None: b = 1. / (gain(P('gain_table'), v) * P('gain_scale'))
    ceff = float(centre) + P('c_eff_off_us')
    kc = P('kap_clip_dpm'); k = 0. if kap is None else max(-kc, min(kc, float(kap)))
    uff = ceff - k * b
    yt = float(P('y_target_cm') or 0.) + float(P('out_cm') or 0.)
    ey = m['y'] - yt; psi = m['psi']; ps = P('pred2_s')
    eyp = ey + v * 100. * ps * math.sin(math.radians(psi)); psip = psi + (gz - v * k) * ps
    ak = abs(k); S['kpk'] = max(S.get('kpk', 0.), ak)
    exiting = S['kpk'] >= P('kap_full_dpm') and ak < P('kap_exit_dpm')
    pm = P('psi_ref_max_exit_deg') if exiting else P('psi_ref_max_deg')
    pref = max(-pm, min(pm, -P('k_yp_deg_cm') * eyp))
    ui = S.get('ui', 0.); il = P('i2_lim_us')
    if exiting: ui *= math.exp(-dt / max(1e-3, P('i_exit_tau_s')))
    elif abs(ey) <= P('i_band_cm') and not S.get('sat'):
        ui = max(-il, min(il, ui + P('ki2_us_cms') * ey * dt)); S['ui_peak'] = ui
    S['ui'] = ui
    kf_ = P('kap_full_dpm'); wi = min(1., ak / kf_) if kf_ > 0 else 1.
    guard = None; sp = math.sin(math.radians(psi)) * v * 100. * P('guard_s'); kg = P('k_guard_us_cm')
    om = P('out_min_cm'); im = P('in_min_cm')
    cop = None if m.get('clr_out') is None else m['clr_out'] - sp   # predicted outer clearance guard_s ahead (psi + = heading out)
    cip = None if m.get('clr_in') is None else m['clr_in'] + sp
    go = om is not None and cop is not None and cop < float(om); gi = im is not None and cip is not None and cip < float(im)
    if go and not gi: pref = min(pref, 0.); guard = 'OUT'   # near the outer line: no outward heading wanted
    if gi and not go: pref = max(pref, 0.); guard = 'IN'
    fb = P('kpsi2_us_deg') * (psip - pref) + wi * ui
    if guard == 'OUT' and fb < kg * (float(om) - cop): fb = kg * (float(om) - cop)   # ... and never less right than this
    if guard == 'IN' and fb > -kg * (float(im) - cip): fb = -kg * (float(im) - cip)
    if go and gi: guard = 'BOTH'
    fl = P('fb2_lim_us'); fb = max(-fl, min(fl, fb))
    return uff + fb, dict(u_ff=round(uff, 1), u_fb=round(fb, 1), u_i=round(wi * ui, 1), psi_ref=round(pref, 1), ey_cm=round(ey, 1), psi_p=round(psip, 1),
                          phase='EXIT' if exiting else 'BODY', guard=guard, clr_out_p=None if cop is None else round(cop, 1), clr_in_p=None if cip is None else round(cip, 1))


def build(cfg=None):
    c = load_config() if cfg is None else cfg
    return ArcLat(c) if isinstance(c, dict) and c.get('enabled') else None
