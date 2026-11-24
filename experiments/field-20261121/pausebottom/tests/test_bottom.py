"""pause-resume-r1g (2026-11-21): pause / resume inside the bottom_owner scope from bottom_pause_min_m after LAND8."""
import types, unittest
import test_pause_unit as U
PR, PC, CLOCK = U.PR, U.PC, U.CLOCK


class TP:
    def __init__(self, since, anchor='LAND8', lost=False):
        self.seg = 'curve4'; self.lost = lost; self.events = []; self.lap = 1; self.D = 60.; self.yaw = 0.; self.anchor = (anchor, 60. - since, 0.)

    def segment(self): return self.seg

    def pending(self): return 'C4e'

    def update(self, now, gz, pitch, enc): self.D += .01


def outcome(since, mn=2.4, bottom=True, anchor='LAND8', lost=False, refuse=True, key=True):
    PC._P = 0.; CLOCK[0] = 1000.
    cfg = dict(PR.DEFAULTS, coast_max_m=2.0, coast_max_yaw_deg=25., allow_segment_cross=True, refuse_pause_where_no_resume=refuse, down_tail_blocks=False)
    if key: cfg['bottom_pause_min_m'] = mn
    PR.load_config = lambda path=None: dict(cfg)
    lp = U.LP(); lp.dash = types.SimpleNamespace(turned=False)
    lp._lpos = TP(since, anchor, lost); lp._lpos_run = lp.run['id']
    lp.down_until = PC.monotonic() + 3.; lp.bown = types.SimpleNamespace(active=bottom)
    try: PR.pause(lp, {'run_id': 'r1'})
    except PR.PauseError as e: return 'refused', (e.extra or {}).get('where')
    U.held(lp, 8); st = lp.status()['pause']
    return 'paused', (None if st['can_resume'] else st['reason'])


class T(unittest.TestCase):
    def test_default_whole_scope_blocks(self):
        self.assertIsNone(PR.DEFAULTS['bottom_pause_min_m'])
        self.assertEqual(outcome(3.0, key=False), ('refused', 'ON_SLOPE'))
        self.assertEqual(outcome(3.0, mn=None), ('refused', 'ON_SLOPE'))
        self.assertEqual(outcome(3.0, mn=True), ('refused', 'ON_SLOPE'))      # a number is needed

    def test_from_min_m(self):
        self.assertEqual(outcome(2.45), ('paused', None))                     # just past the limit (2.4 itself is 2.3999.. in this test's arithmetic)
        self.assertEqual(outcome(3.2), ('paused', None))
        self.assertEqual(outcome(2.3), ('refused', 'ON_SLOPE'))               # landing zone proper
        self.assertEqual(outcome(0.5), ('refused', 'ON_SLOPE'))

    def test_needs_the_landing_anchor(self):
        self.assertEqual(outcome(3.0, anchor='DOWN'), ('refused', 'ON_SLOPE'))    # LAND8 not found: no distance to trust
        self.assertEqual(outcome(3.0, lost=True), ('refused', 'ON_SLOPE'))

    def test_resume_side_without_refusal(self):
        self.assertEqual(outcome(1.0, refuse=False), ('paused', 'ON_SLOPE'))  # pause taken (refusal off) but not resumable
        self.assertEqual(outcome(3.0, refuse=False), ('paused', None))

    def test_outside_the_scope_unchanged(self):
        self.assertEqual(outcome(4.5, bottom=False), ('paused', None))
        self.assertEqual(outcome(4.5, bottom=False, key=False), ('paused', None))


if __name__ == '__main__':
    unittest.main()
