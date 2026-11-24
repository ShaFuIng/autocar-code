"""pause-resume-r1c (2026-11-21, run f0938b9d): coasting over a lap_pos landmark on the flat may be resumed when allow_segment_cross is on."""
import types, unittest
import test_pause_unit as U
PR, PC, CLOCK = U.PR, U.PC, U.CLOCK


class TP:
    def __init__(self, seg):
        self.seg = seg; self.lost = False; self.events = []; self.lap = 1; self.D = 18.2; self.yaw = 0.; self.anchor = ('C1e', 0.); self.nxt = 'C2s'

    def segment(self): return self.seg

    def pending(self): return self.nxt

    def update(self, now, gz, pitch, enc): self.D += .01


def code(seg0, seg1, on, key=True, lost=False, lap=False, route='WAIT_CLIMB', turned=False, upper=False):
    """pause in seg0; during the hold the car coasts into seg1 (a landmark event); returns the refusal code or None."""
    PC._P = 0.; CLOCK[0] = 1000.
    cfg = dict(PR.DEFAULTS, coast_max_m=2.0, coast_max_yaw_deg=25., upper_post_turn_resume=upper)
    if key: cfg['allow_segment_cross'] = on
    PR.load_config = lambda path=None: dict(cfg)
    lp = U.LP(); lp.latest['dash_route'] = dict(route=route); lp.dash = types.SimpleNamespace(turned=turned)
    tp = TP(seg0); lp._lpos = tp; lp._lpos_run = lp.run['id']
    PR.pause(lp, {'run_id': 'r1'}); U.held(lp, 2)
    if seg1 != seg0 or lap or lost:
        tp.seg = seg1; tp.events.append(('C2s', 18.7)); tp.nxt = 'C2e'; tp.lost = lost
        if lap: tp.lap += 1
    U.held(lp, 8)
    st = lp.status()['pause']
    return None if st['can_resume'] else st['reason']


class T(unittest.TestCase):
    def test_default_and_off(self):
        self.assertEqual(PR.DEFAULTS['allow_segment_cross'], False)
        self.assertEqual(code('diagonal_v', 'curve2', False), 'MOVED_ACROSS_SEGMENT')
        self.assertEqual(code('diagonal_v', 'curve2', None, key=False), 'MOVED_ACROSS_SEGMENT')
        self.assertEqual(code('diagonal_v', 'curve2', 1), 'MOVED_ACROSS_SEGMENT')   # only the JSON true counts

    def test_on_flat(self):
        self.assertIsNone(code('diagonal_v', 'curve2', True))            # the case of run f0938b9d
        self.assertIsNone(code('main_straight', 'curve1', True, lap=True))   # over the lap boundary
        self.assertIsNone(code('diagonal_v', 'diagonal_v', True))        # nothing crossed: as before
        self.assertIsNone(code('diagonal_v', 'diagonal_v', False))

    def test_on_but_into_forbidden(self):
        self.assertEqual(code('curve3', 'ramp', True), 'ON_SLOPE')
        self.assertEqual(code('curve3', 'top', True), 'UPPER_DECK')
        self.assertEqual(code('diagonal_v', 'curve2', True, lost=True), 'MOVED_ACROSS_SEGMENT')   # lap_pos lost during the coast
        self.assertEqual(code('upper_straight', 'downhill', True, route='OPEN', turned=True, upper=True), 'ON_SLOPE')
        self.assertEqual(code('ramp', 'top', True), 'ON_SLOPE')          # paused on the ramp: still by the pause-time rule

    def test_coast_limits_still_apply(self):
        PC._P = 0.; CLOCK[0] = 1000.
        cfg = dict(PR.DEFAULTS, coast_max_m=0.3, allow_segment_cross=True); PR.load_config = lambda path=None: dict(cfg)
        lp = U.LP(); tp = TP('diagonal_v'); lp._lpos = tp; lp._lpos_run = lp.run['id']
        PR.pause(lp, {'run_id': 'r1'}); enc = 1000
        for _ in range(8): enc += 300; U.held(lp, 1, enc=enc)   # 0.53 m of coasting > 0.3 m
        tp.seg = 'curve2'; tp.events.append(('C2s', 18.7)); U.held(lp, 8)
        self.assertEqual(lp.status()['pause']['reason'], 'MOVED_ACROSS_SEGMENT')


if __name__ == '__main__':
    unittest.main()
