"""curve-ff-bycurve-r1: unit tests + differential replay against the car's curve_ff.py (a6ed733e).
run from the experiments folder:  python field-20261122/cff_bycurve/test_bycurve.py"""
import importlib.util, json, sys, unittest
from pathlib import Path
HERE = Path(__file__).resolve().parent; EXP = HERE.parents[1]
sys.path.insert(0, str(EXP / 'field-20261118'))
from ld import load


def mod(path, name):
    s = importlib.util.spec_from_file_location(name, path); m = importlib.util.module_from_spec(s); s.loader.exec_module(m); return m


OLD = mod(EXP / 'orin-web-snapshot-20261121c' / 'curve_ff.py', 'cff_old'); NEW = mod(HERE / 'candidate' / 'curve_ff.py', 'cff_new')
CAR = json.loads((EXP / 'orin-web-snapshot-20261121c' / 'curve_ff.json').read_text(encoding='utf8'))
ON = json.loads((HERE / 'on' / 'curve_ff.c1_soft.json').read_text(encoding='utf8'))
RUNS = ['run-3197-20261122', 'run-8350-20261122', 'run-687a-20261122']


def frames(run):
    for p in load(str(EXP / run)):
        cf = p.get('curve_ff') or {}; inp = p.get('inputs') or {}
        yield (cf.get('pid_steer_us', p.get('raw_steer_us') or p.get('steer_us') or 1475.), p.get('image_error'), inp.get('gz'), p['t'], p['_s'])


class T(unittest.TestCase):
    def test_on_file_only_adds_by_curve(self):
        self.assertEqual({k: v for k, v in ON.items() if k not in ('by_curve', 'note_by_curve')}, CAR)
        self.assertEqual(list(ON['by_curve']), ['curve1'])

    def test_key_absent_identical(self):
        n = 0
        for run in RUNS:
            a, b, c = OLD.CurveFF(dict(CAR)), NEW.CurveFF(dict(CAR)), NEW.CurveFF(dict(ON))
            for st, e, gz, t, seg in frames(run):
                ra = a.step(st, e, 1475., 780., 2050., True, now=t, gz=gz, path=None)
                self.assertEqual(ra, b.step(st, e, 1475., 780., 2050., True, now=t, gz=gz, path=None, curve=seg))   # no by_curve key: same with any curve name
                self.assertEqual(ra, c.step(st, e, 1475., 780., 2050., True, now=t, gz=gz, path=None, curve=None))   # by_curve present, lap_pos unknown: same
                n += 1
        print('key-absent / curve-unknown frames identical:', n)

    def test_only_curve1_changes(self):
        tot = ch = 0; out_of = 0; lower = higher = 0
        for run in RUNS:
            a, c = NEW.CurveFF(dict(CAR)), NEW.CurveFF(dict(ON))
            for st, e, gz, t, seg in frames(run):
                ra = a.step(st, e, 1475., 780., 2050., True, now=t, gz=gz, path=None); rc = c.step(st, e, 1475., 780., 2050., True, now=t, gz=gz, path=None, curve=seg)
                tot += 1
                if ra[0] != rc[0]:
                    ch += 1; out_of += seg != 'curve1'; lower += rc[0] < ra[0]; higher += rc[0] > ra[0]
        print('frames %d, command changed %d (lower %d, higher %d), changed outside curve1 %d' % (tot, ch, lower, higher, out_of))
        self.assertGreater(ch, 0); self.assertEqual(higher, 0)
        self.assertLessEqual(out_of, 15)   # only the hysteresis carry-over of the frames right after curve 1

    def test_values(self):
        f = NEW.CurveFF(dict(ON, gate=False)); g = NEW.CurveFF(dict(CAR, gate=False))
        for e, want_new, want_old in ((0.60, None, 1561.2), (0.72, 1578.5, 1768.2), (0.80, 1696.8, 1906.2), (1.00, 1992.5, 1992.5)):
            f.side = g.side = 0
            rn = f.step(1400., e, 1475., 780., 2050., True, curve='curve1'); ro = g.step(1400., e, 1475., 780., 2050., True)
            self.assertAlmostEqual(ro[0], want_old, 0)
            if want_new is None: self.assertEqual(rn, (1400., None))
            else: self.assertAlmostEqual(rn[0], want_new, 0); self.assertEqual(rn[1]['by_curve'], 'curve1')
        f.side = 0; self.assertAlmostEqual(f.step(1400., 0.72, 1475., 780., 2050., True, curve='curve3')[0], 1768.2, 0)   # other curves: global values
        bad = NEW.CurveFF(dict(CAR, gate=False, by_curve=dict(curve1=dict(on_err='x', gate=False, max_frac=True))))
        self.assertAlmostEqual(bad.step(1400., 0.72, 1475., 780., 2050., True, curve='curve1')[0], 1768.2, 0)        # unusable values are ignored


if __name__ == '__main__':
    unittest.main()
