"""straightfast-20261121 unit tests for candidate/lap_speed.py (lap-speed-r3: two_level zones, per-zone rate_pwm_s).
cd tests; python -m unittest -v test_two_level      (standard library; imports the candidate and the car module next to each other)"""
import importlib.util, sys, types, unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXP = HERE.parents[2]
SNAP = EXP / 'orin-web-snapshot-20261121c'
sys.path.insert(0, str(SNAP))   # throttle_cap (max_pct 100 from the car's throttle_cap.json)


def mod(path, name):
    sp = importlib.util.spec_from_file_location(name, str(path)); m = importlib.util.module_from_spec(sp); sp.loader.exec_module(m); return m


NEW = mod(HERE.parent / 'candidate' / 'lap_speed.py', 'ls_new')
OLD = mod(SNAP / 'lap_speed.py', 'ls_old')
TL = dict(e_trouble=0.40, e_ok=0.30, steer_trouble_us=300., gz_trouble_dps=25., gz_tau_s=0.25, mid_pwm=70., rate_up_pwm_s=200., rate_down_pwm_s=120.)


class Lap(object):
    def __init__(self, D=3.0): self.D = D; self.lap = 1; self.lost = False; self.events = [dict(name='C4e', d_cross=0., lap=1)]


def rig(M, zones, **top):
    cfg = dict(M.DEFAULTS); cfg.update(enabled=True, zones=zones, **top)
    g = M.LapSpeed(cfg, 'r'); lap = Lap()
    lp = types.SimpleNamespace(run=dict(parameters=dict(center_us=1475.)), _lpos=lap, speed=types.SimpleNamespace(c=dict(hard_cap=50.), cur=50., t=0.))
    return g, lp, lap


def frame(pwm=50., e=0.05, steer=1480., level='HIGH', reason='STABLE', weave=False, why='PROPOSAL_ONLY'):
    sa = dict(level=level, reason=reason, pwm=pwm)
    if weave: sa['weave_slow'] = True
    return dict(reason=why, speed_adapt=sa, image_error=e, steer_us=steer, pwm=pwm)


def F(gz=0., route='WAIT_CLIMB', pr=-1.0, special=False):
    return types.SimpleNamespace(dr=dict(route=route), pr=pr, special=special, gzy=gz)


def drive(g, lp, seq, t0=0., dt=1 / 30.):
    """seq = list of (frame kwargs, F kwargs); returns the list of (pwm, log)"""
    out = []; t = t0
    for fk, Fk in seq:
        p = frame(**fk); t += dt; lp.speed.cur = p['pwm']
        log = g.apply(lp, F(**Fk), p, t); out.append((p['pwm'], log))
    return out


ZONE = dict(name='A main straight 100', anchor='C4e', from_m=0.8, to_m=7.3, pwm=100)


class T(unittest.TestCase):
    def test_keys_absent_is_the_car_module(self):
        seq = [(dict(), dict())] * 10 + [(dict(level='LOW', reason='ERROR_RATE_LARGE', weave=True, pwm=44.), dict())] + [(dict(), dict())] * 20 + [(dict(e=0.35), dict())] + [(dict(), dict())] * 5
        a = drive(*rig(OLD, [dict(ZONE)])[:2], seq); b = drive(*rig(NEW, [dict(ZONE)])[:2], seq)
        self.assertEqual(a, b)
        self.assertIn('blocked', a[10][1]); self.assertLess(a[11][0], 60.)   # r1: one LOW frame restarts the ramp from speed_adapt's PWM

    def test_error_rate_low_no_longer_drops(self):
        g, lp, _ = rig(NEW, [dict(ZONE, two_level=dict(TL))])
        seq = [(dict(), dict())] * 20 + [(dict(level='LOW', reason='ERROR_RATE_LARGE', weave=True, pwm=44.), dict(gz=3.))] + [(dict(level='MID', reason='STABLE_PENDING', pwm=45.), dict())] * 5
        out = drive(g, lp, seq)
        self.assertAlmostEqual(out[19][0], 100.); self.assertAlmostEqual(out[20][0], 100.); self.assertAlmostEqual(out[25][0], 100.)
        self.assertEqual(out[20][1]['two_level']['state'], 'NORMAL'); self.assertEqual(out[20][1]['revision'], 'lap-speed-r3'); self.assertNotIn('blocked', out[20][1])

    def test_entry_ramp_rate(self):
        g, lp, _ = rig(NEW, [dict(ZONE, two_level=dict(TL))])
        out = drive(g, lp, [(dict(pwm=40.), dict())] * 12)
        self.assertAlmostEqual(out[0][0], 40.)                              # first frame: speed_adapt's PWM
        self.assertAlmostEqual(out[1][0], 40. + 200. / 30., places=3)       # then 200 PWM/s
        self.assertAlmostEqual(out[9][0], 100.)                             # 0.3 s later at the zone PWM

    def test_trouble_by_error_with_hysteresis(self):
        g, lp, _ = rig(NEW, [dict(ZONE, two_level=dict(TL))])
        seq = [(dict(), dict())] * 15 + [(dict(e=0.45), dict())] * 12 + [(dict(e=0.35), dict())] * 3 + [(dict(e=0.20), dict())] * 6
        out = drive(g, lp, seq)
        self.assertAlmostEqual(out[14][0], 100.)
        self.assertAlmostEqual(out[15][0], 100. - 120. / 30., places=3); self.assertEqual(out[15][1]['two_level'], dict(state='TROUBLE', why='ERROR', gz_lp=0.0))
        self.assertAlmostEqual(out[26][0], 70.)                             # down to mid_pwm at 120 PWM/s, not to speed_adapt's 50
        self.assertEqual(out[28][1]['two_level']['state'], 'TROUBLE')       # 0.35 is between e_ok and e_trouble: still trouble
        self.assertEqual(out[30][1]['two_level']['state'], 'NORMAL'); self.assertAlmostEqual(out[30][0], 70. + 200. / 30., places=3)   # back at once, no wait
        self.assertAlmostEqual(out[35][0], 100.)

    def test_trouble_by_imu_yaw_rate(self):
        g, lp, _ = rig(NEW, [dict(ZONE, two_level=dict(TL))])
        out = drive(g, lp, [(dict(), dict(gz=2.))] * 15 + [(dict(), dict(gz=60.))] * 10 + [(dict(), dict(gz=0.))] * 30)
        st = [o[1]['two_level']['state'] for o in out]
        self.assertEqual(st[15], 'NORMAL')                                  # one frame of 60 dps is still filtered
        self.assertIn('TROUBLE', st[16:25]); self.assertEqual(out[24][1]['two_level']['why'], 'YAW')
        self.assertEqual(st[-1], 'NORMAL'); self.assertAlmostEqual(out[-1][0], 100.)
        g2, lp2, _ = rig(NEW, [dict(ZONE, two_level=dict(TL, gz_trouble_dps=None))])
        self.assertNotIn('TROUBLE', [o[1]['two_level']['state'] for o in drive(g2, lp2, [(dict(), dict(gz=60.))] * 20)])   # null = not used
        g3, lp3, _ = rig(NEW, [dict(ZONE, two_level=dict(TL))])
        self.assertNotIn('TROUBLE', [o[1]['two_level']['state'] for o in drive(g3, lp3, [(dict(), dict(gz=None))] * 20)])  # no IMU value: error only

    def test_steer_trouble(self):
        g, lp, _ = rig(NEW, [dict(ZONE, two_level=dict(TL))])
        out = drive(g, lp, [(dict(), dict())] * 12 + [(dict(steer=1475. + 320.), dict())] * 3)
        self.assertEqual(out[-1][1]['two_level']['why'], 'STEER')

    def test_hard_blocks_unchanged(self):
        for fk, Fk, code in ((dict(why='PID_HOLD'), dict(), 'REASON'), (dict(level='LOW', reason='SPECIAL_CONTROL'), dict(), 'SPEED_ADAPT'), (dict(level='LOW', reason='DOWNHILL'), dict(), 'SPEED_ADAPT'),
                             (dict(), dict(route='OPEN'), 'ROUTE'), (dict(), dict(pr=6.5), 'PITCH'), (dict(), dict(special=True), 'SPECIAL'), (dict(e=None), dict(), 'NO_ERROR')):
            g, lp, _ = rig(NEW, [dict(ZONE, two_level=dict(TL))])
            out = drive(g, lp, [(dict(), dict())] * 12 + [(dict(pwm=30., **fk), Fk)] + [(dict(pwm=30.), dict())] * 2)
            self.assertEqual(out[12][1].get('blocked'), code, (fk, Fk)); self.assertAlmostEqual(out[12][0], 30.)       # frame left as speed_adapt made it
            self.assertAlmostEqual(out[13][0], 30.)                                                                   # and the carry is gone: restart from speed_adapt's PWM
        g, lp, _ = rig(NEW, [dict(ZONE, two_level=dict(TL))])
        p = frame(pwm=30.); p['speed_adapt']['landing'] = dict(pwm=33); drive(g, lp, [(dict(), dict())] * 5); lp.speed.cur = 30.
        self.assertEqual(g.apply(lp, F(), p, 9.)['blocked'], 'LANDING')

    def test_never_lowers_and_caps(self):
        g, lp, _ = rig(NEW, [dict(ZONE, pwm=40, two_level=dict(TL))])
        out = drive(g, lp, [(dict(pwm=50.), dict())] * 6)
        self.assertTrue(all(abs(o[0] - 50.) < 1e-9 for o in out))            # zone PWM below speed_adapt's: nothing lowered
        g, lp, _ = rig(NEW, [dict(ZONE, two_level=dict(TL, mid_pwm=30.))])
        out = drive(g, lp, [(dict(pwm=50.), dict())] * 12 + [(dict(pwm=50., e=0.5), dict())] * 30)
        self.assertGreaterEqual(min(o[0] for o in out), 50.)                  # mid_pwm below speed_adapt's PWM: stops at speed_adapt's value

    def test_hand_over_between_zones(self):
        lower = dict(name='c4 back half 70', anchor='LAND8', from_m=3.4, to_m=9.0, pwm=70, allow_low=True)
        g, lp, lap = rig(NEW, [dict(ZONE, two_level=dict(TL)), lower, dict(name='main 50', anchor='C4e', from_m=0.8, to_m=8.0, pwm=50)], down_rate_pwm_s=80.)
        lap.events = [dict(name='LAND8', d_cross=0., lap=1)]; lap.D = 5.0
        drive(g, lp, [(dict(pwm=48.), dict())] * 20)                          # inside the LAND8 zone: up to 70
        lap.events.append(dict(name='C4e', d_cross=4.0, lap=1)); lap.D = 4.9  # C4e+0.9: the two_level zone is first in the list
        out = drive(g, lp, [(dict(pwm=48.), dict())] * 6, t0=20 / 30.)
        self.assertGreater(out[0][0], 70.); self.assertAlmostEqual(out[5][0], 100.)   # r3: continues upward from 70 (r1 would restart at 48)
        lap.D = 11.5                                                          # C4e+7.5: the PWM-50 zone (plain zone): the existing taper from 100
        out = drive(g, lp, [(dict(pwm=50.), dict())] * 3, t0=26 / 30.)
        self.assertTrue(out[0][1].get('taper')); self.assertAlmostEqual(out[0][0], 100. - 80. / 30., places=3)

    def test_per_zone_rate(self):
        g, lp, _ = rig(NEW, [dict(ZONE, rate_pwm_s=150.)])
        out = drive(g, lp, [(dict(pwm=40.), dict())] * 4)
        self.assertAlmostEqual(out[1][0], 40. + 150. / 30., places=3)
        g, lp, _ = rig(NEW, [dict(ZONE)])
        self.assertAlmostEqual(drive(g, lp, [(dict(pwm=40.), dict())] * 4)[1][0], 40. + 60. / 30., places=3)   # absent = global 60

    def test_disabled_key(self):
        seq = [(dict(), dict())] * 8 + [(dict(level='LOW', reason='ERROR_RATE_LARGE', weave=True, pwm=44.), dict())] + [(dict(), dict())] * 6
        a = drive(*rig(OLD, [dict(ZONE)])[:2], seq)
        for tl in (dict(TL, enabled=False), 'on', None, 5):
            self.assertEqual(drive(*rig(NEW, [dict(ZONE, two_level=tl)])[:2], seq), a, tl)


if __name__ == '__main__':
    unittest.main()
