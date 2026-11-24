"""pause-resume-r1b (2026-11-21): resume allowed only on the upper straight after the dashed turn, and only with the config key on."""
import types, unittest
import test_pause_unit as U
PR, PC, CLOCK = U.PR, U.PC, U.CLOCK

class TP:
    def __init__(self, seg, lost=False): self.seg = seg; self.lost = lost; self.events = []; self.lap = 1; self.D = 40.; self.yaw = 0.; self.anchor = ('DASHe', 0.)
    def segment(self): return self.seg
    def pending(self): return 'DOWN'
    def update(self, now, gz, pitch, enc): self.D += .01

def code(seg, turned, on, lost=False, route='OPEN', key=True):
    PC._P = 0.; CLOCK[0] = 1000.
    cfg = dict(PR.DEFAULTS, coast_max_m=2.0, coast_max_yaw_deg=25.)
    if key: cfg['upper_post_turn_resume'] = on
    PR.load_config = lambda path=None: dict(cfg)
    lp = U.LP(); lp.latest['dash_route'] = dict(route=route); lp.dash = types.SimpleNamespace(turned=turned)
    lp._lpos = TP(seg, lost); lp._lpos_run = lp.run['id']
    PR.pause(lp, {'run_id': 'r1'}); U.held(lp, 8)
    st = lp.status()['pause']; return st['reason'] if not st['can_resume'] else None

class T(unittest.TestCase):
    def test_default_off(self):
        self.assertEqual(PR.DEFAULTS['upper_post_turn_resume'], False)
        self.assertEqual(code('upper_straight', True, False), 'UPPER_DECK')
    def test_on_post_turn(self):
        self.assertIsNone(code('upper_straight', True, True))
    def test_on_but_elsewhere(self):
        self.assertEqual(code('top', False, True), 'UPPER_DECK')              # crest -> turn straight
        self.assertEqual(code('top', True, True), 'UPPER_DECK')
        self.assertEqual(code('dashed_turn', True, True), 'UPPER_DECK')       # in the turn
        self.assertEqual(code('upper_straight', False, True), 'UPPER_DECK')   # turn not started by DashRoute
        self.assertEqual(code('upper_straight', True, True, lost=True), 'UPPER_DECK')   # lap_pos lost, route still OPEN
        self.assertEqual(code('upper_straight', True, 1), 'UPPER_DECK')       # only the JSON true counts
        self.assertEqual(code('ramp', False, True, route='WAIT_CLIMB'), 'ON_SLOPE')
        self.assertEqual(code('downhill', True, True, route='DONE'), 'ON_SLOPE')
    def test_flat_unchanged(self):
        self.assertIsNone(code('main_straight', False, True, route='WAIT_CLIMB'))
        self.assertIsNone(code('main_straight', False, False, route='WAIT_CLIMB'))
if __name__ == '__main__': unittest.main()
