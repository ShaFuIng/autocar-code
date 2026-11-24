"""i-preload-r1 unit tests (laptop, standard library).  cd tests; python -m unittest -v test_preload
Imports candidate/i_ramp.py and, for the unchanged-behaviour checks, the car's i_ramp.py (orin-web-snapshot-20261121)."""
import importlib.util, json, sys, unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXP = HERE.parents[2]


def mod(path, name):
    sp = importlib.util.spec_from_file_location(name, str(path)); m = importlib.util.module_from_spec(sp); sp.loader.exec_module(m); return m


NEW = mod(HERE.parent / 'candidate' / 'i_ramp.py', 'i_ramp_new')
OLD = mod(EXP / 'orin-web-snapshot-20261121' / 'i_ramp.py', 'i_ramp_old')
CAR = json.loads((EXP / 'orin-web-snapshot-20261121' / 'i_ramp.json').read_text(encoding='utf8'))
START = dict(lap=0, anchor=None, since_anchor_m=None, lost=False)


class Ctrl:
    def __init__(self, i=0., e=0., ki=40.): self.i = i; self.e = e; self.p = dict(ki=ki)


class Wrap:
    def __init__(self, c): self.pid = c


def ramp(cfg=None):
    c = dict(NEW.DEFAULTS); c.update(CAR); c['preload'] = dict(cfg or {})
    return NEW.IRamp(c)


def drive(r, ctrl, seq, t=0., dt=1 / 30.):
    """seq: list of (error, lap dict); returns the preload records (None when absent)"""
    out = []
    for e, lap in seq:
        t += dt; (ctrl.pid if hasattr(ctrl, 'pid') else ctrl).e = e
        x = r.step(t, 0., ctrl, lap=lap); out.append((x or {}).get('preload'))
    return out, t


class T(unittest.TestCase):
    def test_key_absent_is_the_car_module(self):
        a = NEW.IRamp(dict(NEW.DEFAULTS, **CAR)); b = OLD.IRamp(dict(OLD.DEFAULTS, **CAR))
        self.assertIsNone(a.pre); self.assertEqual(a.wants_lap, b.wants_lap)
        ca, cb = Ctrl(i=-.3), Ctrl(i=-.3); t = 0.
        laps = [START] * 150 + [dict(lap=1, anchor='C1s', since_anchor_m=.1 * k, lost=False) for k in range(40)] + [dict(lap=1, anchor='C1e', since_anchor_m=.1 * k, lost=False) for k in range(40)]
        for k, lap in enumerate(laps):
            t += 1 / 30.; e = .6 if k > 100 else -.2; ca.e = cb.e = e; ca.i += e / 30.; cb.i += e / 30.
            self.assertEqual(a.step(t, 0., ca, lap=lap), b.step(t, 0., cb, lap=lap)); self.assertEqual(ca.i, cb.i)
        for bad in (None, False, 5, dict(enabled=False), dict(target_us=0), dict(target_us='x'), dict(rate_us_s=0)):
            self.assertIsNone(NEW._pre_cfg(bad))

    def test_triggers_once_and_ramps(self):
        r = ramp(); c = Ctrl(i=-.25)   # -10 us
        out, t = drive(r, c, [(-.2, START)] * 60 + [(.31, START), (.35, START), (.45, START), (.5, START), (.6, START), (.6, START), (.6, START), (.6, START), (.6, START)])
        self.assertTrue(all(x is None for x in out[:61]))                 # start straight, and the first frame above e_on (n_on 2)
        self.assertEqual(out[61]['state'], 'RAMP'); self.assertAlmostEqual(out[61]['i_us_before'], -10., 3); self.assertAlmostEqual(out[61]['i_us'], -5., 3)   # 150 us/s x 1/30 s
        states = [x['state'] for x in out[61:] if x]; self.assertEqual(states[-1], 'DONE'); self.assertEqual(states.count('DONE'), 1)
        self.assertAlmostEqual(40. * c.i, 20., 6); self.assertEqual(r.p_state, 'OFF')
        c.i = -.5; out2, _ = drive(r, c, [(.9, START)] * 30, t)           # never again in this run
        self.assertTrue(all(x is None for x in out2)); self.assertEqual(c.i, -.5)

    def test_one_step_and_targets(self):
        for tgt in (15., 20., 25.):
            r = ramp(dict(target_us=tgt, rate_us_s=None)); c = Ctrl(i=-.2)
            out, _ = drive(r, c, [(-.1, START)] * 50 + [(.4, START)] * 3)
            self.assertEqual([x['state'] for x in out if x], ['DONE']); self.assertAlmostEqual(40. * c.i, tgt, 6)

    def test_already_above_target_untouched(self):
        r = ramp(); c = Ctrl(i=.7)   # +28 us
        out, _ = drive(r, c, [(-.1, START)] * 50 + [(.5, START)] * 5)
        self.assertEqual([x for x in out if x], [dict(state='DONE', i_us=28.)]); self.assertEqual(c.i, .7)

    def test_not_on_later_laps_or_after_a_landmark(self):
        for lap in (dict(lap=1, anchor='C4e', since_anchor_m=9., lost=False), dict(lap=2, anchor='C4e', since_anchor_m=9., lost=False),
                    dict(lap=0, anchor='C1s*', since_anchor_m=.2, lost=False)):
            r = ramp(); c = Ctrl(i=-.3)
            out, _ = drive(r, c, [(-.1, lap)] * 50 + [(.8, lap)] * 20)
            self.assertEqual([x for x in out if x], [dict(state='OFF', reason='NOT_BEFORE_FIRST_LANDMARK')]); self.assertEqual(c.i, -.3)
        r = ramp(); c = Ctrl(i=-.3)   # the landmark arrives before the error does: off for the run
        out, _ = drive(r, c, [(-.1, START)] * 50 + [(.1, dict(lap=1, anchor='C1s', since_anchor_m=.1, lost=False))] * 3 + [(.8, START)] * 10)
        self.assertEqual(c.i, -.3)

    def test_guards(self):
        r = ramp(); c = Ctrl(i=-.3)   # too early (min_s 1.5 s) and single frames do not trigger
        out, t = drive(r, c, [(.9, START)] * 40); self.assertTrue(all(x is None for x in out)); self.assertEqual(c.i, -.3)
        out, t = drive(r, c, [(.9, START), (.1, START), (.9, START), (.1, START)] * 5, t + 1.); self.assertTrue(all(x is None for x in out))
        r = ramp(); c = Ctrl(i=-.3)   # lap_pos lost: no count
        out, _ = drive(r, c, [(-.1, START)] * 50 + [(.9, dict(START, lost=True))] * 10); self.assertTrue(all(x is None for x in out)); self.assertEqual(c.i, -.3)
        r = ramp(dict(max_s=3.)); c = Ctrl(i=-.3)   # nothing for max_s: off
        out, _ = drive(r, c, [(-.1, START)] * 120 + [(.9, START)] * 10); self.assertEqual([x for x in out if x], [dict(state='OFF', reason='TIMEOUT')]); self.assertEqual(c.i, -.3)
        r = ramp(); c = Ctrl(i=-.3)   # no lap_pos information
        out, _ = drive(r, c, [(.9, None)] * 100); self.assertTrue(all(x is None for x in out)); self.assertEqual(c.i, -.3)

    def test_pause_resume_rebuilds_the_controller(self):
        r = ramp(); c = Ctrl(i=-.3)
        out, t = drive(r, c, [(-.1, START)] * 60)
        c2 = Ctrl(i=0.)   # apply_resets: lp.controller = type(lp.controller)(parameters)
        out, t = drive(r, c2, [(-.1, START)] * 5 + [(.9, START)] * 20, t)
        self.assertEqual([x for x in out if x], [dict(state='OFF', reason='CONTROLLER_REPLACED')]); self.assertEqual(c2.i, 0.)

    def test_wrapped_pid(self):
        r = ramp(); inner = Ctrl(i=-.25); w = Wrap(inner)
        out, _ = drive(r, w, [(-.1, START)] * 50 + [(.6, START)] * 12)
        self.assertAlmostEqual(40. * inner.i, 20., 6)

    def test_other_releases_still_run(self):
        r = ramp(); c = Ctrl(i=1.0)   # after the preload is off, the C1e exit release of the car config still acts
        drive(r, c, [(-.1, dict(lap=1, anchor='C4e', since_anchor_m=9., lost=False))] * 3)
        x = None
        for k in range(10): x = r.step(100. + k / 30., 0., c, lap=dict(lap=1, anchor='C1e', since_anchor_m=1.0, lost=False))
        self.assertEqual(x['exit']['state'], 'ON'); self.assertLess(c.i, 1.0); self.assertNotIn('preload', x)


if __name__ == '__main__':
    unittest.main()
