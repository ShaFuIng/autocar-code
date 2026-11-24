"""C3. Closed-loop sketch of the slope -> landing -> 1.9 m with the land4 law (the real land4.Land4 object in the loop).
Plant (AGENTS.md 'vehicle facts'): kinematic bicycle on the rear axle, curvature = gain * (effective steering - straight point) deg/m,
steering = play_comp (+-55 us, threshold 15, emulated) -> pure delay -> first-order lag (total ~0.24-0.30 s) -> backlash (play width ~130 us);
rear encoder status every 0.2 s carrying counts sampled 0.185 s earlier; frames every 0.035 s, camera error 0.07 s old.
Lane model = the good-landing median heading theta*(u) (config table) taken as the lane direction, laid out along the TRUE distance with the
same encoder lag the logs had; camera near error e = e_ref(u) + (y_cm + 64 sin(dpsi)) / 40 (AGENTS: y_cam = 40 e - 64 sin dpsi), clipped to +-1.
LIMITS: no tyre slip, no load transfer / bounce at the landing, the lane geometry is the median trajectory (not a survey), the camera model is a
two-parameter fit from another road section, the baseline is NOT simulated (the car's chain has ~20 layers). Past offline models of this car
under-estimated the real swing; read the numbers as 'does the law converge and how sensitive is it', not as predicted centimetres."""
import json, math, random, sys, statistics as S
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent / 'cand')); sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'candidate'))
import land4

CFG = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[1] / 'on' / 'step2_slope_land' / 'land4.json'
N = int(sys.argv[2]) if len(sys.argv) > 2 else 300
C0 = 1475.; M = land4.M_PER_COUNT


class Comp:   # play_comp emulation (direction / extreme logic of play-comp-r1)
    def __init__(s): s.dir = 0; s.ext = C0
    def step(s, raw, r=55., l=55., th=15.):
        if s.dir == 0:
            if raw - s.ext >= th: s.dir = 1; s.ext = raw
            elif s.ext - raw >= th: s.dir = -1; s.ext = raw
        elif s.dir > 0:
            if raw >= s.ext: s.ext = raw
            elif s.ext - raw >= th: s.dir = -1; s.ext = raw
        else:
            if raw <= s.ext: s.ext = raw
            elif raw - s.ext >= th: s.dir = 1; s.ext = raw
        return raw + (r if s.dir > 0 else (-l if s.dir < 0 else 0.))


def run(cfg, seed=None, k=None, verbose=False, **ov):
    R = random.Random(seed)
    par = dict(y0=0., psi0=0., straight=1467., gain_scale=1., delay=0.19, lag=0.08, play=130., glitch=0., len_err=0., cam_ky=1., v_scale=1., e_bias=0.)
    if seed is not None:
        par.update(y0=R.gauss(0., 5.), psi0=R.gauss(0.6, 1.5), straight=1467. + R.uniform(-15., 15.), gain_scale=R.uniform(.8, 1.2), delay=R.uniform(.15, .22), play=R.uniform(110., 155.),
                   glitch=(R.choice((-5., 5.)) if R.random() < .15 else 0.), len_err=R.gauss(0., .05), cam_ky=R.uniform(.6, 1.4), v_scale=R.uniform(.93, 1.07), e_bias=R.gauss(0., .05))
    par.update(ov)
    c = land4.merge(cfg)
    if k:
        for kk, vv in k.items():
            if kk in c['slope'] or kk in c['land']: c['slope'][kk] = vv; c['land'][kk] = vv
            else: c[kk] = vv
    L = land4.Land4(c)
    th_tab = c['theta']; er_tab = c['e_ref']; vt = VT
    lagd = 0.2                                     # mean encoder lag (s) of the logs: table u = true distance - v * lagd
    dt = 0.005; t = 0.; s = 0.                     # s = true distance since the mouth
    Ls = c['slope_len_m']                          # the car starts at table u = -slope_len_m (mouth)
    yaw = par['psi0']; X = 0.; Y = par['y0'] / 100.   # world: x along the upper straight, y left +
    # reference path (lane centre) in world coordinates, parametrised by table u
    ref = []; rx = 0.; ry = 0.; uu = -Ls - 0.3
    while uu < 2.6:
        h = land4.interp(th_tab, uu); ref.append((uu, rx, ry, h)); rx += 0.01 * math.cos(math.radians(h)); ry += 0.01 * math.sin(math.radians(h)); uu += 0.01
    # car start: at table u = -Ls (mouth), true position offset so that the apparent coordinate matches
    i0 = min(range(len(ref)), key=lambda i: abs(ref[i][0] + Ls)); X = ref[i0][1]; Y = ref[i0][2] + par['y0'] / 100.
    comp = Comp(); q = []; eff_f = C0; bl = C0; cmd = C0; enc_hist = []; stat = None; landed_t = None; frames = []; ecam = []
    for i in range(45):   # upper straight before the mouth: the reference = the car's own (gyro-held) heading there, i.e. reference error = psi0
        L.step(-1.6 + i * 0.035, True, 0., par['psi0'] + R.gauss(0., .3), 0., 100000 - int((45 - i) * 0.035 * 0.93 / M), int((-1.6 + i * 0.035) * 1000), None, True, False, None, C0, C0, 780., 2050., C0)
    next_frame = 0.; next_stat = 0.05; out = []; glitch_done = False; yaw_meas_off = 0.; gz = 0.
    ymax = -9.; ymin = 9.; cmdmax = 0.; dcmd = 0.; prev_cmd = C0; gzmin = 0.; sgn = 0; psis = []
    while True:
        v = land4.interp(vt, min(3., ref[i0][0])) * par['v_scale']
        # nearest reference point -> lateral offset (left +) and lane heading
        j = min(range(max(0, i0 - 5), min(len(ref), i0 + 60)), key=lambda i: (ref[i][1] - X) ** 2 + (ref[i][2] - Y) ** 2); i0 = j
        ru, rxx, ryy, rh = ref[j]
        ylat = (-(X - rxx) * math.sin(math.radians(rh)) + (Y - ryy) * math.cos(math.radians(rh))) * 100.
        u_true = ru
        if u_true > 1.9 or t > 6.: break
        if t >= next_stat:
            enc_hist.append((t, s)); so = next((ss for tt, ss in reversed(enc_hist) if tt <= t - 0.185), enc_hist[0][1])
            stat = (int(so / M) + 100000, int(t * 1000)); next_stat += 0.2
        if landed_t is None and u_true >= par['len_err']:   # the landing detection defines u = 0 of the tables; len_err = the slope is longer / shorter than slope_len_m
            landed_t = t
            if par['glitch'] and not glitch_done: yaw_meas_off += par['glitch']; glitch_done = True
        if t >= next_frame:
            e_now = land4.interp(er_tab, u_true) + par['e_bias'] + par['cam_ky'] * (ylat + 64. * math.sin(math.radians(yaw - rh))) / 40.
            ecam.append((t, max(-1., min(1., e_now)))); e_old = next((ee for tt, ee in reversed(ecam) if tt <= t - 0.07), ecam[0][1])
            chain = C0 + max(-300., min(240., 220. * 1.45 * e_old))   # stand-in for the chain after the hand-over only (camera P, capped)
            st, info = L.step(t, True, e_old, yaw + yaw_meas_off, gz, None if stat is None else stat[0], None if stat is None else stat[1], None, False, True, landed_t, chain, C0, 780., 2050., C0)
            cmd = st; next_frame += 0.035
            if landed_t is not None and info and info.get('state') == 'LAND':
                cmdmax = max(cmdmax, cmd - C0); dcmd = max(dcmd, abs(cmd - prev_cmd)); psis.append((info.get('x_m'), info.get('psi'), info.get('th')))
            prev_cmd = cmd
            sent = comp.step(cmd); q.append((t + par['delay'], sent))
            if verbose and info: print('%.2f u %.2f y %.1f yaw %.1f lane %.1f e %.2f cmd %.0f %s' % (t, u_true, ylat, yaw, rh, e_old, cmd - C0, info.get('state')))
        while q and q[0][0] <= t: target = q.pop(0)[1]; q_t = target; eff_in = target; run.last_in = target
        tgt = getattr(run, 'last_in', C0)
        eff_f += (tgt - eff_f) * dt / par['lag']
        h = par['play'] / 2.
        if eff_f - bl > h: bl = eff_f - h
        elif bl - eff_f > h: bl = eff_f + h
        g = (0.186 if landed_t is not None else 0.14) * par['gain_scale']   # tools/ident.py: landing zone 0.186, lower slope 0.14 deg/m per us
        kap = g * (bl - par['straight'])            # deg/m, right +
        gz = -kap * v; yaw += gz * dt
        X += v * dt * math.cos(math.radians(yaw)); Y += v * dt * math.sin(math.radians(yaw)); s += v * dt; t += dt
        if landed_t is not None:
            ymax = max(ymax, ylat); ymin = min(ymin, ylat); gzmin = min(gzmin, gz)
    run.last_in = C0
    he = [abs(p - th) for x, p, th in psis if x is not None and p is not None and x >= 0.9]
    return dict(y_out=ymax, y_in=ymin, y_end=ylat, cmdmax=cmdmax, dcmd=dcmd, gzmin=gzmin, head_end=yaw - rh, he=max(he) if he else None, par=par)


VT = [[-2.2, 0.94], [-1.0, 1.01], [0.0, 1.17], [0.5, 1.21], [1.0, 1.10], [1.5, 1.05], [2.0, 1.0], [3.0, 0.98]]

if __name__ == '__main__':
    cfg = json.loads(CFG.read_text(encoding='utf8'))
    print('config', CFG)
    print('nominal:', {k: (round(v, 1) if isinstance(v, float) else v) for k, v in run(cfg).items() if k != 'par'})
    for name, ov in (('start 10 cm left (outside)', dict(y0=10.)), ('start 10 cm right (inside)', dict(y0=-10.)), ('heading +4 deg at the mouth', dict(psi0=4.)), ('heading -4 deg', dict(psi0=-4.)),
                     ('yaw glitch +5 deg at the landing (despike on)', dict(glitch=5.)), ('yaw glitch -5 deg (despike on)', dict(glitch=-5.)), ('yaw glitch +5 deg, despike OFF', dict(glitch=5., k=dict(spike_on=False))), ('yaw glitch -5 deg, despike OFF', dict(glitch=-5., k=dict(spike_on=False))), ('table 0.1 m earlier (x_shift_m 0.1)', dict(k=dict(x_shift_m=0.1))), ('steering gain x0.8', dict(gain_scale=.8)), ('steering gain x1.2', dict(gain_scale=1.2)),
                     ('play 155 us', dict(play=155.)), ('delay 0.22+0.08 s', dict(delay=.22)), ('straight point 1452', dict(straight=1452.)), ('straight point 1482', dict(straight=1482.)),
                     ('landing 0.1 m later than the table', dict(len_err=.1)), ('landing 0.1 m earlier', dict(len_err=-.1)), ('speed +7%', dict(v_scale=1.07))):
        r = run(cfg, **ov)
        print('  %-46s lateral after landing: max left(out) %+5.1f cm, max right(in) %+5.1f cm, at 1.9 m %+5.1f cm | heading err at 1.9 m %+5.1f deg | cmd max %+4.0f us, max step %3.0f us, gz min %4.0f dps' % (
            name, r['y_out'], r['y_in'], r['y_end'], r['head_end'], r['cmdmax'], r['dcmd'], r['gzmin']))
    for gains in (None, dict(k_psi=5.), dict(k_psi=15.), dict(k_psi=20.), dict(k_gz=0.), dict(k_gz=3.), dict(k_cam=0.), dict(k_cam=25.), dict(x_shift_m=0.1)):
        rs = [run(cfg, seed=i, k=gains) for i in range(N)]
        yo = sorted(r['y_out'] for r in rs); yi = sorted(r['y_in'] for r in rs); ye = [r['y_end'] for r in rs]; he = [r['head_end'] for r in rs]; dc = sorted(r['dcmd'] for r in rs)
        print('MC %d %-18s | out max: median %+5.1f P90 %+5.1f worst %+5.1f | in max: median %+5.1f P10 %+5.1f worst %+5.1f | y at 1.9 m sd %.1f | heading err at 1.9 m sd %.1f | >+12 cm out: %d, <-12 cm in: %d' % (
            N, 'config gains' if gains is None else json.dumps(gains), yo[len(yo) // 2], yo[int(.9 * len(yo))], yo[-1], yi[len(yi) // 2], yi[int(.1 * len(yi))], yi[0], S.pstdev(ye), S.pstdev(he),
            sum(1 for y in yo if y > 12.), sum(1 for y in yi if y < -12.)))
