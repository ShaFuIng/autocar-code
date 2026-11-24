"""pause-resume-r1e (2026-11-21, run c981aee5): blind resume on the upper straight after the dashed turn."""
import types, unittest
import test_pause_unit as U
PR, PC, CLOCK = U.PR, U.PC, U.CLOCK


class TP:
    def __init__(self, seg, lost=False):
        self.seg = seg; self.lost = lost; self.events = []; self.lap = 1; self.D = 40.; self.yaw = 0.; self.anchor = ('DASHe', 39., 0.)

    def segment(self): return self.seg

    def pending(self): return 'DOWN'

    def update(self, now, gz, pitch, enc): self.D += .01


def invalid_hold(lp, n=8, stale=False):
    """held frames whose worker result has no path (as standing at DASHe+1.66 m)"""
    for _ in range(n):
        CLOCK[0] += 1 / 30.
        cap = PC.monotonic() - (.9 if stale else .05)
        PR.hold_frame(lp, dict(valid=False, path=[], mask_png=U.png(), seq=1), cap, 11, 1, None)


def code(seg='upper_straight', turned=True, blind=True, upper=True, route='OPEN', lost=False, stale=False, key=True):
    PC._P = 0.; CLOCK[0] = 1000.
    cfg = dict(PR.DEFAULTS, coast_max_m=2.0, coast_max_yaw_deg=25., upper_post_turn_resume=upper, allow_segment_cross=True)
    if key: cfg['upper_post_blind_resume'] = blind
    PR.load_config = lambda path=None: dict(cfg)
    lp = U.LP(); lp.latest['dash_route'] = dict(route=route); lp.dash = types.SimpleNamespace(turned=turned)
    lp._lpos = TP(seg, lost); lp._lpos_run = lp.run['id']
    PR.pause(lp, {'run_id': 'r1'}); invalid_hold(lp, 8, stale)
    st = lp.status()['pause']
    return None if st['can_resume'] else st['reason']


class T(unittest.TestCase):
    def test_default_off(self):
        self.assertEqual(PR.DEFAULTS['upper_post_blind_resume'], False)
        self.assertEqual(code(blind=False), 'NO_FRESH_PATH')
        self.assertEqual(code(key=False), 'NO_FRESH_PATH')
        self.assertEqual(code(blind=1), 'NO_FRESH_PATH')            # only the JSON true counts

    def test_blind_resume_on_the_post_turn_straight(self):
        self.assertIsNone(code())                                   # the case of run c981aee5

    def test_still_refused(self):
        self.assertEqual(code(stale=True), 'NO_FRESH_PATH')         # the image itself is old
        self.assertEqual(code(seg='main_straight', route='WAIT_CLIMB', turned=False), 'NO_FRESH_PATH')   # the flat still needs a path
        self.assertEqual(code(seg='curve2', route='WAIT_CLIMB', turned=False), 'NO_FRESH_PATH')
        self.assertEqual(code(turned=False), 'UPPER_DECK')          # turn not started
        self.assertEqual(code(seg='top'), 'UPPER_DECK')
        self.assertEqual(code(upper=False), 'UPPER_DECK')           # the post-turn resume itself is off

    def test_resume_is_applied(self):
        PC._P = 0.; CLOCK[0] = 1000.
        cfg = dict(PR.DEFAULTS, coast_max_m=2.0, upper_post_turn_resume=True, upper_post_blind_resume=True); PR.load_config = lambda path=None: dict(cfg)
        lp = U.LP(); lp.latest['dash_route'] = dict(route='OPEN'); lp.dash = types.SimpleNamespace(turned=True)
        lp._lpos = TP('upper_straight'); lp._lpos_run = lp.run['id']
        PR.pause(lp, {'run_id': 'r1'}); invalid_hold(lp, 8)
        r = PR.resume(lp, {'run_id': 'r1'}); self.assertEqual((r['ok'], r['state']), (True, 'running'))
        PR.loop_top(lp, None, None); self.assertEqual(lp.status()['pause']['state'], 'running')
        self.assertLess(PC.monotonic() - lp.latest['capture_at'], lp.run['parameters']['max_gap'])


if __name__ == '__main__':
    unittest.main()
