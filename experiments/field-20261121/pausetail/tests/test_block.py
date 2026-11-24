"""pause-resume-r1d (2026-11-21): a pause is refused where a resume is impossible by position; status carries can_pause."""
import types, unittest
import test_pause_unit as U
PR, PC, CLOCK = U.PR, U.PC, U.CLOCK


class TP:
    def __init__(self, seg, lost=False, since=1.0, anchor='DASHe'):
        self.seg = seg; self.lost = lost; self.events = []; self.lap = 1; self.D = 40.; self.yaw = 0.; self.anchor = (anchor, 40. - since, 0.)

    def segment(self): return self.seg

    def pending(self): return 'X'

    def update(self, now, gz, pitch, enc): self.D += .01


def make(seg, on=True, route='WAIT_CLIMB', turned=False, lost=False, pitch=0., since=1.0, down=False, bottom=False, **cfgx):
    PC._P = 0.; CLOCK[0] = 1000.
    cfg = dict(PR.DEFAULTS, coast_max_m=2.0, coast_max_yaw_deg=25., upper_post_turn_resume=True, allow_segment_cross=True, **cfgx)
    if on is not None: cfg['refuse_pause_where_no_resume'] = on
    PR.load_config = lambda path=None: dict(cfg)
    lp = U.LP(); lp.latest['dash_route'] = dict(route=route); lp.dash = types.SimpleNamespace(turned=turned)
    lp.boost.pitch = lp.boost.c['flat_pitch_deg'] + pitch
    lp._lpos = TP(seg, lost, since); lp._lpos_run = lp.run['id']
    if down: lp.down_until = PC.monotonic() + 2.
    if bottom: lp.bown = types.SimpleNamespace(active=True)
    return lp


def try_pause(lp):
    """(refusal code or None, where)"""
    try: PR.pause(lp, {'run_id': 'r1'}); return None, None
    except PR.PauseError as e: return e.code, (e.extra or {}).get('where')


class T(unittest.TestCase):
    def test_default_off_everything_pauses(self):
        self.assertEqual(PR.DEFAULTS['refuse_pause_where_no_resume'], False)
        for on in (False, None, 1):
            lp = make('ramp', on=on, pitch=15.); self.assertEqual(try_pause(lp), (None, None))
            self.assertEqual(lp.manual.latch, [True])   # the car was stopped as before

    def test_refused_where_no_resume(self):
        cases = [(dict(seg='ramp'), 'ON_SLOPE'), (dict(seg='downhill', route='DONE'), 'ON_SLOPE'), (dict(seg='curve4', bottom=True), 'ON_SLOPE'),
                 (dict(seg='curve4', down=True), 'ON_SLOPE'), (dict(seg='main_straight', lost=True, pitch=-16.), 'ON_SLOPE'),
                 (dict(seg='top', route='CLIMBED'), 'UPPER_DECK'), (dict(seg='top', route='OPEN'), 'UPPER_DECK'),
                 (dict(seg='dashed_turn', route='OPEN', turned=True), 'UPPER_DECK'), (dict(seg='upper_straight', route='OPEN', turned=False), 'UPPER_DECK'),
                 (dict(seg='curve3', route='OPEN', lost=True), 'UPPER_DECK')]
        for kw, where in cases:
            lp = make(**kw); st = lp.status()['pause']
            self.assertEqual((st['can_pause'], st['pause_block']), (False, where), kw)
            self.assertEqual(try_pause(lp), ('NO_PAUSE_HERE', where), kw)
            self.assertEqual(lp.manual.latch, [], kw)            # nothing was sent to the car: it keeps driving
            self.assertEqual(lp.status()['pause']['state'], 'running', kw)
            self.assertIn('pause_refused', [x['kind'] for x in lp.log.lines], kw)

    def test_allowed_on_the_flat_and_post_turn(self):
        for kw in (dict(seg='main_straight'), dict(seg='curve2'), dict(seg='curve3', pitch=5.), dict(seg='upper_straight', route='OPEN', turned=True),
                   dict(seg='upper_straight', route='OPEN', turned=True, since=1.9, upper_post_pause_max_m=2.0)):
            lp = make(**kw); st = lp.status()['pause']
            self.assertEqual((st['can_pause'], st['pause_block']), (True, None), kw)
            self.assertEqual(try_pause(lp), (None, None), kw); self.assertEqual(lp.manual.latch, [True], kw)
            lp.boost.pitch = lp.boost.c['flat_pitch_deg']   # level again (5 deg is allowed for the pause but a resume still needs < slope_deg)
            U.held(lp, 8)
            if not kw.get('pitch'): self.assertTrue(lp.status()['pause']['can_resume'], kw)

    def test_slope_mouth_margin(self):
        lp = make('upper_straight', route='OPEN', turned=True, since=2.3, upper_post_pause_max_m=2.0)
        self.assertEqual(try_pause(lp), ('NO_PAUSE_HERE', 'SLOPE_MOUTH'))
        lp = make('upper_straight', route='OPEN', turned=True, since=2.3)   # no margin configured
        self.assertEqual(try_pause(lp), (None, None))

    def test_duplicate_pause_still_answers(self):
        lp = make('curve3'); self.assertEqual(try_pause(lp), (None, None))
        lp._lpos.seg = 'ramp'   # coasted onto the ramp while paused
        r = PR.pause(lp, {'run_id': 'r1'}); self.assertTrue(r['duplicate'])
        st = lp.status()['pause']; self.assertEqual(st['state'], 'paused'); self.assertNotIn('can_pause', st)


if __name__ == '__main__':
    unittest.main()
