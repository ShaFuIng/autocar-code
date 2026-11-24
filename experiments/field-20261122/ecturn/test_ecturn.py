import importlib.util, json, unittest
from pathlib import Path
H = Path(__file__).resolve().parent
import sys; sys.path.insert(0, str(H.parents[1] / 'orin-web-snapshot-20261121c'))   # lap_ff.py imports its sibling modules


def mod(p, n):
    s = importlib.util.spec_from_file_location(n, p); m = importlib.util.module_from_spec(s); s.loader.exec_module(m); return m


N = mod(H / 'candidate' / 'lap_ff.py', 'lff_new')
ON = json.loads((H / 'on' / 'lap_ff.c2_ecturn.json').read_text(encoding='utf8'))['curves']['c2']['band']


class T(unittest.TestCase):
    def test_table(self):
        self.assertIsNone(N.ec_turn({}, 50.)); self.assertIsNone(N.ec_turn(dict(e_c_turn=None), 50.)); self.assertIsNone(N.ec_turn(dict(e_c_turn=[[0, .3]]), 50.))
        self.assertIsNone(N.ec_turn(dict(e_c_turn=[[0, .3], [10, 'x']]), 5.)); self.assertIsNone(N.ec_turn(dict(e_c_turn=[[0, .3], [10, 3.]]), 5.))
        self.assertIsNone(N.ec_turn(ON, None))
        for tb, want in ((-5, .30), (0, .30), (10, .35), (42, .56), (83, .75), (92.5, .725), (180, .64), (198, .30), (300, .10)):
            self.assertAlmostEqual(N.ec_turn(ON, tb), want, 6, tb)

    def test_on_file(self):
        t = ON['e_c_turn']; self.assertEqual(t, sorted(t)); self.assertTrue(all(0. <= e <= .8 for _, e in t)); self.assertEqual(ON['e_c'], 0.56)


if __name__ == '__main__':
    unittest.main()
