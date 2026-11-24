"""pause-resume-r1f (2026-11-21): the down-tail window can stop blocking pause / resume in curve 4 (the bottom_owner scope still does)."""
import types, unittest
import test_pause_unit as U
PR, PC, CLOCK = U.PR, U.PC, U.CLOCK


class TP:
    def __init__(self, seg):
        self.seg = seg; self.lost = False; self.events = []; self.lap = 1; self.D = 60.; self.yaw = 0.; self.anchor = ('LAND8', 56.5, 0.)

    def segment(self): return self.seg

    def pending(self): return 'C4e'

    def update(self, now, gz, pitch, enc): self.D += .01


def make(tail=True, bottom=False, blocks=None, has_bown=True, seg='curve4', pitch=0.):
    PC._P = 0.; CLOCK[0] = 1000.
    cfg = dict(PR.DEFAULTS, coast_max_m=2.0, coast_max_yaw_deg=25., allow_segment_cross=True, refuse_pause_where_no_resume=True)
    if blocks is not None: cfg['down_tail_blocks'] = blocks
    PR.load_config = lambda path=None: dict(cfg)
    lp = U.LP(); lp.dash = types.SimpleNamespace(turned=False); lp.boost.pitch = lp.boost.c['flat_pitch_deg'] + pitch
    lp._lpos = TP(seg); lp._lpos_run = lp.run['id']
    lp.down_until = PC.monotonic() + (3. if tail else -1.)
    lp.bown = types.SimpleNamespace(active=bottom) if has_bown else None
    return lp


def outcome(lp):
    """('refused', where) or ('paused', resume code after standing still)"""
    try: PR.pause(lp, {'run_id': 'r1'})
    except PR.PauseError as e: return 'refused', (e.extra or {}).get('where')
    U.held(lp, 8); st = lp.status()['pause']
    return 'paused', (None if st['can_resume'] else st['reason'])


class T(unittest.TestCase):
    def test_default_tail_blocks(self):
        self.assertEqual(PR.DEFAULTS['down_tail_blocks'], True)
        self.assertEqual(outcome(make()), ('refused', 'ON_SLOPE'))
        self.assertEqual(outcome(make(blocks=True)), ('refused', 'ON_SLOPE'))
        self.assertEqual(outcome(make(blocks=0)), ('refused', 'ON_SLOPE'))        # only the JSON false counts

    def test_tail_alone_no_longer_blocks(self):
        self.assertEqual(outcome(make(blocks=False)), ('paused', None))           # LAND8+3.4 .. 4.6 m: pause and resume

    def test_landing_and_slope_still_block(self):
        self.assertEqual(outcome(make(blocks=False, bottom=True)), ('refused', 'ON_SLOPE'))            # bottom_owner scope
        self.assertEqual(outcome(make(blocks=False, seg='downhill')), ('refused', 'ON_SLOPE'))         # on the slope by lap_pos
        self.assertEqual(outcome(make(blocks=False, pitch=-16.)), ('refused', 'ON_SLOPE'))             # on the slope by pitch
        self.assertEqual(outcome(make(blocks=False, has_bown=False)), ('refused', 'ON_SLOPE'))         # no bottom_owner: the tail keeps blocking

    def test_resume_side(self):
        cfg = dict(PR.DEFAULTS, coast_max_m=2.0)   # refusal off: the pause is taken, the resume rule decides
        PR.load_config = lambda path=None: dict(cfg)
        lp = make(); PR.load_config = lambda path=None: dict(cfg)
        PR.pause(lp, {'run_id': 'r1'}); U.held(lp, 8); self.assertEqual(lp.status()['pause']['reason'], 'ON_SLOPE')
        cfg2 = dict(cfg, down_tail_blocks=False); lp = make(); PR.load_config = lambda path=None: dict(cfg2)
        PR.pause(lp, {'run_id': 'r1'}); U.held(lp, 8); self.assertTrue(lp.status()['pause']['can_resume'])

    def test_outside_the_tail_unchanged(self):
        self.assertEqual(outcome(make(tail=False)), ('paused', None))
        self.assertEqual(outcome(make(tail=False, blocks=False)), ('paused', None))


if __name__ == '__main__':
    unittest.main()
