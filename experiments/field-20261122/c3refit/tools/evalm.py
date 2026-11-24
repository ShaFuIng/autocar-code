"""closed-loop model runs of the candidate tables against the car's present table (2b81e9b2) and the chain alone. Estimate only (see calib.py for what the model reproduces).
usage: python evalm.py <arc2.json of the car> [name=<arc2.json> ...]   (without extra files: the VARIANTS of plan.py)"""
import json, sys, numpy as np, sim, plan
CB = [[1.95, 0.], [2.25, -0.15], [2.55, -0.15], [2.85, 0.], [3.15, 0.1], [3.45, 0.25], [3.75, 0.], [4.05, -0.3], [4.35, -0.25]]   # the c6f3 camera residual pattern (ident2.py): the camera reading the model does not explain


def met(R):
    m = (R[:, 0] >= 0.9) & (R[:, 0] <= 4.2); x = R[:, 0]; g = lambda x0, c: R[int(np.argmin(np.abs(x - x0))), c]
    u = R[m, 1]; xs = x[m]; n3 = int(round(0.3 / (xs[1] - xs[0]))); du = np.abs(u[n3:] - u[:-n3]).max()
    mc = (x >= 4.2) & (x <= 5.6)
    return dict(ymin=R[m, 2].min(), ymax=R[m, 2].max(), y435=g(4.35, 2), psi435=g(4.35, 3), umax=u.max(), umin=u.min(), du=du, emax=R[m, 4].max(), ecorner=R[mc, 4].max(), h57=g(5.7, 5))


def line(name, R):
    q = met(R); return '%-36s y %+5.1f..%+5.1f | at 4.35 m y %+5.1f psi %+5.1f | cmd %+4.0f..%+4.0f, largest change in 0.3 m %3.0f | err max %.2f | corner err max %.2f | heading at 5.7 m %.0f' % (
        name, q['ymin'], q['ymax'], q['y435'], q['psi435'], q['umin'], q['umax'], q['du'], q['emax'], q['ecorner'], q['h57'])


SC = [('nominal', {}), ('distance reads 0.15 m long', dict(xerr=0.15)), ('distance reads 0.15 m short', dict(xerr=-0.15)), ('steering gain x 0.85', dict(g_scale=0.85)), ('steering gain x 1.15', dict(g_scale=1.15)),
      ('straight point +20 us', dict(u0_shift=20.)), ('straight point -20 us', dict(u0_shift=-20.)), ('start +6 deg more turned, 3 cm in', dict(dh=6., dy=-3.)), ('start 6 deg less turned, 3 cm out', dict(dh=-6., dy=3.)),
      ('heading reference +3 deg', dict(h_bias=3.)), ('heading reference -3 deg', dict(h_bias=-3.)), ('arc_lat y reads 3 cm more outside', dict(y_bias=3.)), ('arc_lat y reads 3 cm more inside', dict(y_bias=-3.)), ('arc_lat y missing', dict(y_off=True)), ('camera off-model (c6f3 pattern)', dict(cam_bias=CB)), ('camera off-model (reversed)', dict(cam_bias=[[a, -b] for a, b in CB]))]


def start(dh=0., dy=0.):
    x0 = 0.45; return dict(x0=x0, h0=float(np.interp(x0, plan.BX, plan.BH)) + dh, y0=float(np.interp(x0, plan.BX, plan.BY)) + dy, v_of_x=lambda x: sim.ip(plan.VMED, x))


def study(name, xb):
    print('==== ' + name)
    for lab, kw in SC:
        kw = dict(kw); st = start(kw.pop('dh', 0.), kw.pop('dy', 0.)); print('  ' + line(lab, sim.run(xb, noise=0.0, **st, **kw)))
    Q = []
    for i in range(80):
        r = np.random.default_rng(1000 + i); st = start(r.normal(0, 4.), r.normal(0, 3.))
        Q.append(met(sim.run(xb, seed=i, noise=0.06, xerr=r.normal(0, 0.08), g_scale=1 + r.normal(0, 0.08), u0_shift=r.normal(0, 12.), h_bias=r.normal(0, 2.), y_bias=r.normal(0, 2.), cam_bias=[[a, b * r.normal(0, 0.7)] for a, b in CB], **st)))
    f = lambda k: np.array([q[k] for q in Q])
    print('  80 scattered runs (start sd 4 deg / 3 cm, distance sd 0.08 m, gain sd 8 %%, straight point sd 12 us, heading reference sd 2 deg, arc_lat y bias sd 2 cm, camera off-model x N(0, 0.7), noise 0.06):')
    print('     y min: median %+.1f worst %+.1f | y max: median %+.1f worst %+.1f | runs with y < -14: %d, y > +3: %d | at 4.35 m: y %+.1f (p5 %+.1f p95 %+.1f) psi %+.1f (p5 %+.1f p95 %+.1f) | cmd max median %+.0f worst %+.0f | change in 0.3 m median %.0f worst %.0f | corner err max median %.2f | heading 5.7 m median %.0f (p5 %.0f p95 %.0f)' % (
        np.median(f('ymin')), f('ymin').min(), np.median(f('ymax')), f('ymax').max(), (f('ymin') < -14).sum(), (f('ymax') > 3).sum(), np.median(f('y435')), np.percentile(f('y435'), 5), np.percentile(f('y435'), 95),
        np.median(f('psi435')), np.percentile(f('psi435'), 5), np.percentile(f('psi435'), 95), np.median(f('umax')), f('umax').max(), np.median(f('du')), f('du').max(), np.median(f('ecorner')), np.median(f('h57')), np.percentile(f('h57'), 5), np.percentile(f('h57'), 95)))


if __name__ == '__main__':
    car = json.load(open(sys.argv[1], encoding='utf8'))['c3']['xband']
    study('chain alone (no x-band) = like runs d60e / 6f61', None)
    study("the car's table 2b81e9b2 (x-band r1) = like run c6f3", car)
    if len(sys.argv) > 2:
        for a in sys.argv[2:]:
            n, p = a.split('=', 1); study(n, json.load(open(p, encoding='utf8'))['c3']['xband'])
    else:
        for n, (pk, bk) in plan.VARIANTS.items(): study('c3refit ' + n, plan.xband(plan.plan(**pk), **bk))
